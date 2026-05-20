"""
Phase 1 — Mission precomputation.

Run once when obstacle data and satellite overpass schedule are known
(typically days before fieldwork).  Saves mission_cache.pkl containing:
  - obstacle unions (restricted, ATC, dropsonde)
  - static vis-graph: BASE + obstacle polygon vertices + satellite entry/exit nodes
    (all arc-length variants are included so Phase 2 can extend arcs post-hoc)
  - all-pairs shortest-path distances & predecessors for static nodes
  - sat_candidates : min-arc only (85 km) for fast search
  - sat_arc_variants: full arc menu per entry for post-hoc Method-A extension

Fixed coordinate origin: GOOSE BAY BASE (lon=-60.43°, lat=53.32°).
The cache is valid for any TPV contour; only chord endpoints change per mission.

Usage:
    python precompute_mission.py
Output:
    mission_cache.pkl
"""
import heapq
import os
import pickle
import time
import warnings

import geopandas as gpd
import numpy as np
from math import cos, radians
from shapely.geometry import LineString, Polygon
from shapely.ops import unary_union

warnings.filterwarnings('ignore')

# ── Fixed parameters ─────────────────────────────────────────────────────────
DATA_DIR           = r'C:\nasa-ffp-nurture\test_data'
CACHE_FILE         = 'mission_cache.pkl'

ORIGIN_LON         = -60.43   # GOOSE BAY — fixed coordinate origin
ORIGIN_LAT         =  53.32
BASE               = np.array([0.0, 0.0])

AIRCRAFT_SPEED_KMH = 850.0
TOTAL_BUDGET_KM    = 4250.0
ATC_PENALTY_FACTOR = 1.35
TURN_PENALTY_KM    = 7.5 / 60.0 * AIRCRAFT_SPEED_KMH   # ≈106 km
T_SAT_H            = 4.0

SAT_MIN_LENGTH_KM  = 6.0 / 60.0 * AIRCRAFT_SPEED_KMH   # 85 km
SAT_ARC_STEP_KM    = SAT_MIN_LENGTH_KM                   # extend in 85 km steps
SAT_ARC_MAX_KM     = 30.0 / 60.0 * AIRCRAFT_SPEED_KMH  # 425 km (30-min ideal)
ENTRY_STEP_KM      = 50.0
FILTER_RADIUS_KM   = 2000.0


# ── Coordinate helpers ────────────────────────────────────────────────────────
def wgs84_to_km(lon, lat):
    x = (np.asarray(lon, float) - ORIGIN_LON) * cos(radians(ORIGIN_LAT)) * 111.32
    y = (np.asarray(lat, float) - ORIGIN_LAT) * 110.54
    return x, y


def _load_polys_km(shp_path, filter_km_radius=None):
    from shapely.geometry import Point as _Pt
    gdf = gpd.read_file(shp_path).to_crs('EPSG:4326')
    clip_circle = _Pt(0, 0).buffer(filter_km_radius) if filter_km_radius else None
    km_polys = []
    for geom in gdf.geometry:
        if geom is None or geom.is_empty:
            continue
        parts = list(geom.geoms) if geom.geom_type == 'MultiPolygon' else [geom]
        for poly in parts:
            if poly.geom_type != 'Polygon':
                continue
            lon_arr = np.array([c[0] for c in poly.exterior.coords])
            lat_arr = np.array([c[1] for c in poly.exterior.coords])
            x_arr, y_arr = wgs84_to_km(lon_arr, lat_arr)
            km_poly = Polygon(zip(x_arr, y_arr)).buffer(0)
            if not km_poly.is_valid or km_poly.is_empty or km_poly.area < 0.1:
                continue
            if clip_circle is not None:
                if not km_poly.intersects(clip_circle):
                    continue
                km_poly = km_poly.intersection(clip_circle).buffer(0)
                if km_poly.is_empty:
                    continue
            if km_poly.geom_type in ('MultiPolygon', 'GeometryCollection'):
                for part in km_poly.geoms:
                    if part.geom_type == 'Polygon' and part.area >= 0.1:
                        km_polys.append(part)
            elif km_poly.geom_type == 'Polygon' and km_poly.area >= 0.1:
                km_polys.append(km_poly)
    return km_polys


# ── Geometry helpers ──────────────────────────────────────────────────────────
def seg_crosses_restricted(p1, p2, restricted_geom):
    seg = LineString([p1, p2])
    return restricted_geom.intersects(seg) and not restricted_geom.touches(seg)


def seg_atc_cost(p1, p2, atc_geom, penalty=ATC_PENALTY_FACTOR):
    seg = LineString([p1, p2])
    length = seg.length
    if length < 1e-9:
        return 0.0
    inside = seg.intersection(atc_geom)
    len_inside = inside.length if not inside.is_empty else 0.0
    return (length - len_inside) + len_inside * penalty


def polygon_vertices(polys):
    verts = []
    for poly in polys:
        coords = list(poly.exterior.coords)[:-1]
        verts.extend([np.array(v, float) for v in coords])
    return verts


def build_visibility_graph(nodes, restricted, atc):
    n = len(nodes)
    adj = {i: [] for i in range(n)}
    for i in range(n):
        for j in range(i + 1, n):
            if seg_crosses_restricted(nodes[i], nodes[j], restricted):
                continue
            cost = seg_atc_cost(nodes[i], nodes[j], atc)
            adj[i].append((j, cost))
            adj[j].append((i, cost))
    return adj


def all_pairs_dijkstra(adj, n):
    sp   = [dict() for _ in range(n)]
    prev = [dict() for _ in range(n)]
    for s in range(n):
        sp[s][s]   = 0.0
        prev[s][s] = None
        heap = [(0.0, s)]
        while heap:
            d, u = heapq.heappop(heap)
            if d > sp[s].get(u, 1e18):
                continue
            for v, w in adj[u]:
                nd = d + w
                if nd < sp[s].get(v, 1e18):
                    sp[s][v]   = nd
                    prev[s][v] = u
                    heapq.heappush(heap, (nd, v))
    return sp, prev


# ── Satellite track helpers ───────────────────────────────────────────────────
def arc_interp(pts, cum, t):
    k = int(np.searchsorted(cum, t, side='right')) - 1
    k = max(0, min(len(pts) - 2, k))
    frac = (t - cum[k]) / max(float(np.linalg.norm(pts[k + 1] - pts[k])), 1e-9)
    return pts[k] + frac * (pts[k + 1] - pts[k])


# ── Load obstacles ────────────────────────────────────────────────────────────
print('Loading obstacle data...')
t0 = time.time()

restricted_polys = _load_polys_km(
    os.path.join(DATA_DIR, 'Restricted_Airspace_Footprint.shp'),
    filter_km_radius=FILTER_RADIUS_KM)
restricted_union = unary_union(restricted_polys) if restricted_polys else Polygon()

atc_polys = _load_polys_km(os.path.join(DATA_DIR, 'Ganderairtraff_Polygon.shp'))
atc_union = unary_union(atc_polys) if atc_polys else Polygon()

dropsonde_polys = _load_polys_km(
    os.path.join(DATA_DIR, 'dropsonde_available.shp'),
    filter_km_radius=FILTER_RADIUS_KM)
dropsonde_union = unary_union(dropsonde_polys) if dropsonde_polys else Polygon()

print(f'  Restricted: {len(restricted_polys)}  ATC: {len(atc_polys)}'
      f'  Dropsonde: {len(dropsonde_polys)}  [{time.time()-t0:.1f}s]')


# ── Load satellite track ─────────────────────────────────────────────────────
print('Loading satellite track...')
_sat_gdf    = gpd.read_file(
    os.path.join(DATA_DIR, 'satellite overpass_0516_simplified.shp'))
_sat_wgs    = np.array([(c[0], c[1])
                        for c in _sat_gdf.geometry.iloc[0].coords])
_sat_x, _sat_y = wgs84_to_km(_sat_wgs[:, 0], _sat_wgs[:, 1])
_sat_km_all    = np.column_stack([_sat_x, _sat_y])

_d_all  = np.sqrt(_sat_km_all[:, 0]**2 + _sat_km_all[:, 1]**2)
_in_reg = np.where(_d_all < FILTER_RADIUS_KM)[0]
_i0 = max(0, _in_reg[0] - 1)
_i1 = min(len(_sat_km_all) - 1, _in_reg[-1] + 1)
seg_pts  = _sat_km_all[_i0:_i1 + 1]
seg_lens = np.linalg.norm(np.diff(seg_pts, axis=0), axis=1)
cum_arc  = np.concatenate([[0.0], np.cumsum(seg_lens)])
print(f'  {len(seg_pts)} vertices, arc={cum_arc[-1]:.0f} km')


# ── Generate satellite candidates ────────────────────────────────────────────
# sat_candidates : (pa, pb_min, arc=SAT_MIN_LENGTH_KM) for search
# sat_arc_variants: entry_key → [(pb_ext, arc_km), ...] for Method-A post-hoc extension
# ALL exit points (min and extended) are added to static nodes so they live in
# the precomputed vis-graph and need no on-the-fly graph rebuilding.
print('Generating satellite candidates...')
t_entries        = np.arange(0, cum_arc[-1] - SAT_MIN_LENGTH_KM + 1e-6, ENTRY_STEP_KM)
arc_steps        = np.arange(SAT_MIN_LENGTH_KM, SAT_ARC_MAX_KM + 1e-6, SAT_ARC_STEP_KM)

sat_candidates   = []   # search candidates: min arc only
sat_arc_variants = {}   # entry_key → [(pb, arc_km), ...]

for t_e in t_entries:
    pa      = arc_interp(seg_pts, cum_arc, t_e)
    pa_key  = tuple(np.round(pa, 3))
    variants = []
    for arc_km in arc_steps:
        t_x = t_e + arc_km
        if t_x > cum_arc[-1]:
            break
        pb = arc_interp(seg_pts, cum_arc, t_x)
        if seg_crosses_restricted(pa, pb, restricted_union):
            break   # longer arcs from this entry also blocked
        variants.append((pb.copy(), float(arc_km)))

    if not variants:
        continue
    # Min-arc candidate for search
    pb_min, arc_min = variants[0]
    sat_candidates.append((pa.copy(), pb_min, arc_min))
    # All arc variants for Method-A post-processing
    sat_arc_variants[pa_key] = variants

print(f'  Entries: {len(t_entries)}  '
      f'search candidates (min arc): {len(sat_candidates)}  '
      f'arc variants stored: {sum(len(v) for v in sat_arc_variants.values())}')


# ── Collect static nodes ─────────────────────────────────────────────────────
print('Collecting static nodes...')
static_nodes = [BASE.copy()]  # index 0 = BASE

for v in polygon_vertices(restricted_polys):
    static_nodes.append(v)
for v in polygon_vertices(atc_polys):
    static_nodes.append(v)

# Add ALL unique satellite points (entry + exit for every arc length)
seen = set()
def _add_pt(pt):
    key = tuple(np.round(pt, 3))
    if key not in seen:
        seen.add(key)
        static_nodes.append(pt.copy())

for pa, pb_min, _ in sat_candidates:
    _add_pt(pa)
    _add_pt(pb_min)

for pa_key, variants in sat_arc_variants.items():
    for pb, arc_km in variants:
        _add_pt(pb)

n_static = len(static_nodes)
print(f'  {n_static} static nodes  '
      f'(1 BASE + {len(polygon_vertices(restricted_polys+atc_polys))} obstacle verts'
      f' + {len(seen)} sat pts)')


# ── Build visibility graph ────────────────────────────────────────────────────
print('Building visibility graph...')
t0 = time.time()
adj = build_visibility_graph(static_nodes, restricted_union, atc_union)
n_edges = sum(len(v) for v in adj.values()) // 2
print(f'  {n_static} nodes, {n_edges} edges  [{time.time()-t0:.1f}s]')


# ── All-pairs shortest paths ──────────────────────────────────────────────────
print('Computing all-pairs SP...')
t0 = time.time()
sp, prev = all_pairs_dijkstra(adj, n_static)
print(f'  Done  [{time.time()-t0:.1f}s]')

node_key = {tuple(np.round(n, 3)): i for i, n in enumerate(static_nodes)}


# ── Save cache ────────────────────────────────────────────────────────────────
cache = dict(
    origin           = (ORIGIN_LON, ORIGIN_LAT),
    BASE             = BASE.copy(),
    restricted_union = restricted_union,
    atc_union        = atc_union,
    dropsonde_union  = dropsonde_union,
    static_nodes     = static_nodes,
    n_static         = n_static,
    adj              = adj,
    sp               = sp,
    prev             = prev,
    node_key         = node_key,
    sat_candidates   = sat_candidates,    # (pa, pb_min, arc_min) for search
    sat_arc_variants = sat_arc_variants,  # entry_key → [(pb, arc_km), ...]
    sat_track        = dict(seg_pts=seg_pts, cum_arc=cum_arc),
    sat_label        = 'EarthCARE overpass (2016-05-16)',
    T_sat_h          = T_SAT_H,
    params           = dict(
        AIRCRAFT_SPEED_KMH = AIRCRAFT_SPEED_KMH,
        TOTAL_BUDGET_KM    = TOTAL_BUDGET_KM,
        ATC_PENALTY_FACTOR = ATC_PENALTY_FACTOR,
        TURN_PENALTY_KM    = TURN_PENALTY_KM,
        SAT_MIN_LENGTH_KM  = SAT_MIN_LENGTH_KM,
        SAT_ARC_MAX_KM     = SAT_ARC_MAX_KM,
    ),
)

with open(CACHE_FILE, 'wb') as f:
    pickle.dump(cache, f, protocol=4)

size_mb = os.path.getsize(CACHE_FILE) / 1e6
print(f'\nSaved {CACHE_FILE}  ({size_mb:.1f} MB)')
print(f'Static nodes: {n_static}  |  search candidates: {len(sat_candidates)}'
      f'  |  arc variants: {sum(len(v) for v in sat_arc_variants.values())}')
print('Phase 1 complete.')
