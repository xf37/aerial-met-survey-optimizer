"""
Phase 2 — Field route planning.

Loads mission_cache.pkl (built by precompute_mission.py), then:
  1. Reads a new TPV contour shapefile → PCA ellipse fit → candidate chords
  2. Adds chord endpoints + gatepoints as dynamic nodes to the precomputed graph
     (Dijkstra from dynamic nodes only; static-static SP reused from cache)
  3. Proximity-filters satellite candidates: keeps only those reachable from the
     TPV region within the remaining budget
  4. Searches: max n_chords → max coincidence → min total distance
     Uses minimum-arc (85 km) candidates so the search space stays small
  5. Method A post-hoc extension: for the best route found, tries every longer
     satellite arc from the same entry point (stored in sat_arc_variants) and
     takes the maximum arc that still satisfies the budget

Usage:
    python plan_route_field.py <tpv_shapefile> [gatepoints_shapefile]

Example:
    python plan_route_field.py C:/data/Contour_2_14_18_3.shp C:/data/gatepoints.shp
"""
import argparse
import heapq
import pickle
import sys
import time
import warnings
from itertools import combinations
from math import cos, sin, pi, sqrt, radians, degrees, atan2

import geopandas as gpd
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
from shapely.geometry import LineString, Polygon
from shapely.ops import unary_union

warnings.filterwarnings('ignore')

# ── Tunable field parameters ──────────────────────────────────────────────────
SPACING_OVERRIDE_KM    = 100      # None = auto from PCA b_fit
MAX_ANGLE_DEV_DEG      = 5.0
N_ANGLE_SAMPLES        = 3
ASPECT_RATIO_THRESHOLD = 1.3
TURN_THRESHOLD_DEG     = 10.0
MIN_LEG_SPACING_KM     = 50.0
MAX_JOINT_N            = 4
SAT_PROX_FACTOR        = 0.6      # satellite midpoint within (remaining_budget × factor) of TPV


# ── Geometry utilities (identical to notebook) ────────────────────────────────
def seg_crosses_restricted(p1, p2, restricted_geom):
    seg = LineString([p1, p2])
    return restricted_geom.intersects(seg) and not restricted_geom.touches(seg)


def seg_atc_cost(p1, p2, atc_geom, penalty):
    seg = LineString([p1, p2])
    length = seg.length
    if length < 1e-9:
        return 0.0
    inside = seg.intersection(atc_geom)
    len_inside = inside.length if not inside.is_empty else 0.0
    return (length - len_inside) + len_inside * penalty


def count_turns_in_path(waypoints):
    pts = np.asarray(waypoints, float)
    n_turns = 0
    for i in range(1, len(pts) - 1):
        d1 = pts[i] - pts[i - 1]
        d2 = pts[i + 1] - pts[i]
        n1, n2 = np.linalg.norm(d1), np.linalg.norm(d2)
        if n1 < 1e-6 or n2 < 1e-6:
            continue
        cos_a = float(np.clip(np.dot(d1 / n1, d2 / n2), -1., 1.))
        if degrees(np.arccos(cos_a)) > TURN_THRESHOLD_DEG:
            n_turns += 1
    return n_turns


def fit_pca_ellipse(pts):
    pts = np.asarray(pts, float)
    center = pts.mean(axis=0)
    cov = np.cov((pts - center).T)
    eigvals, eigvecs = np.linalg.eigh(cov)
    idx = np.argsort(eigvals)[::-1]
    eigvals, eigvecs = eigvals[idx], eigvecs[:, idx]
    a   = 2.0 * sqrt(max(eigvals[0], 0.))
    b   = 2.0 * sqrt(max(eigvals[1], 0.))
    phi = atan2(eigvecs[1, 0], eigvecs[0, 0])
    return center, a, b, phi


def chord_in_polygon(offset_pt, direction, poly, far=2e4):
    p, d = np.asarray(offset_pt, float), np.asarray(direction, float)
    d = d / np.linalg.norm(d)
    line    = LineString([p - far * d, p + far * d])
    segment = poly.intersection(line)
    if segment.is_empty:
        return None
    if segment.geom_type == 'MultiLineString':
        segment = max(segment.geoms, key=lambda g: g.length)
    if segment.geom_type != 'LineString' or segment.length < 0.5:
        return None
    coords = np.array(segment.coords)
    pt_a, pt_b = coords[0], coords[-1]
    if np.dot(pt_a - p, d) > np.dot(pt_b - p, d):
        pt_a, pt_b = pt_b, pt_a
    return pt_a, pt_b, float(segment.length)


def generate_candidate_chords(tpv_poly, center, a, phi,
                               min_spacing, angle_devs_deg, restricted_geom):
    major   = np.array([cos(phi), sin(phi)])
    n_steps = max(1, int(2.0 * a / min_spacing)) + 1
    offsets = np.linspace(-a * 0.95, a * 0.95, n_steps)
    chords  = []
    for dev in angle_devs_deg:
        chord_angle = phi + pi / 2 + radians(dev)
        direction   = np.array([cos(chord_angle), sin(chord_angle)])
        for d in offsets:
            pt     = np.asarray(center) + d * major
            result = chord_in_polygon(pt, direction, tpv_poly)
            if result is None:
                continue
            pt_a, pt_b, length = result
            if restricted_geom is not None and \
               seg_crosses_restricted(pt_a, pt_b, restricted_geom):
                continue
            chords.append(dict(
                pt_a=pt_a, pt_b=pt_b, length=length,
                offset=float(d), angle_dev=dev, direction=direction.copy()))
    return chords


def _append_obs_leg(obs_wps, entry, exit_, wpts, segs, leg_type='tpv'):
    if obs_wps is None:
        wpts.append(np.asarray(exit_, float))
        segs.append(leg_type)
        return float(np.linalg.norm(np.asarray(exit_, float) - np.asarray(entry, float)))
    wps = [np.asarray(w, float) for w in obs_wps]
    if not np.allclose(np.asarray(entry, float), wps[0], atol=0.1):
        wps = list(reversed(wps))
    total = 0.0
    for k in range(1, len(wps)):
        total += float(np.linalg.norm(wps[k] - wps[k - 1]))
        wpts.append(wps[k])
        segs.append(leg_type)
    return total


def route_dropsonde_dist(wpts, dropsonde_geom):
    total = 0.0
    pts = np.asarray(wpts, float)
    for k in range(len(pts) - 1):
        seg = LineString([pts[k], pts[k + 1]])
        inside = seg.intersection(dropsonde_geom)
        total += inside.length if not inside.is_empty else 0.0
    return total


# ── Satellite track interpolation ────────────────────────────────────────────
def arc_interp(pts, cum, t):
    k = int(np.searchsorted(cum, t, side='right')) - 1
    k = max(0, min(len(pts) - 2, k))
    frac = (t - cum[k]) / max(float(np.linalg.norm(pts[k + 1] - pts[k])), 1e-9)
    return pts[k] + frac * (pts[k + 1] - pts[k])


def sat_arc_waypoints(pa, arc_km, seg_pts, cum_arc, n_pts=60):
    """Return dense intermediate points along satellite track from pa for arc_km."""
    dists = np.linalg.norm(seg_pts - pa, axis=1)
    k0    = int(np.argmin(dists))
    t_e   = float(cum_arc[k0])
    t_x   = min(t_e + arc_km, float(cum_arc[-1]))
    ts    = np.linspace(t_e, t_x, max(n_pts, int(arc_km / 5) + 2))
    return np.array([arc_interp(seg_pts, cum_arc, t) for t in ts])


# ── Incremental graph extension ───────────────────────────────────────────────
def extend_graph(cache, dynamic_pts):
    """Add dynamic_pts (chord endpoints + GPs) to the precomputed static graph.

    Strategy:
    - Keep static-static SP from cache (no recomputation)
    - Build edges from each new dynamic node to all existing nodes
    - Run Dijkstra from each dynamic node on the full graph
    - Static-to-dynamic distances: use symmetry (undirected graph)

    Returns full_nodes, full_adj, sp_full, prev_full, node_key, n_static
    """
    static_nodes = cache['static_nodes']
    n_static     = cache['n_static']
    adj_static   = cache['adj']
    sp_static    = cache['sp']
    prev_static  = cache['prev']
    restricted   = cache['restricted_union']
    atc          = cache['atc_union']
    ATC_PF       = cache['params']['ATC_PENALTY_FACTOR']

    # Deduplicate against existing static nodes
    existing_keys = set(cache['node_key'].keys())
    new_pts = []
    for pt in dynamic_pts:
        key = tuple(np.round(pt, 3))
        if key not in existing_keys:
            existing_keys.add(key)
            new_pts.append(np.asarray(pt, float))

    full_nodes = list(static_nodes) + new_pts
    n_total    = len(full_nodes)
    n_dynamic  = len(new_pts)

    # Full adjacency: copy static edges, add edges for dynamic nodes
    full_adj = {i: list(adj_static[i]) for i in range(n_static)}
    for i in range(n_static, n_total):
        full_adj[i] = []

    for i_dyn in range(n_dynamic):
        idx_dyn = n_static + i_dyn
        dn = new_pts[i_dyn]
        for j in range(n_total):
            if j == idx_dyn:
                continue
            if seg_crosses_restricted(dn, full_nodes[j], restricted):
                continue
            cost = seg_atc_cost(dn, full_nodes[j], atc, penalty=ATC_PF)
            full_adj[idx_dyn].append((j, cost))
            full_adj[j].append((idx_dyn, cost))

    # SP for full graph:
    #   static-static: copy from cache (no recomputation)
    #   dynamic-*:     Dijkstra from each dynamic node on full graph
    #   static-dynamic: symmetry  sp[s][d] = sp[d][s]
    sp_full   = [dict(sp_static[i]) for i in range(n_static)]
    sp_full  += [dict() for _ in range(n_dynamic)]
    prev_full = [dict(prev_static[i]) for i in range(n_static)]
    prev_full += [dict() for _ in range(n_dynamic)]

    for i_dyn in range(n_dynamic):
        s = n_static + i_dyn
        sp_full[s][s]   = 0.0
        prev_full[s][s] = None
        heap = [(0.0, s)]
        while heap:
            d, u = heapq.heappop(heap)
            if d > sp_full[s].get(u, 1e18):
                continue
            for v, w in full_adj[u]:
                nd = d + w
                if nd < sp_full[s].get(v, 1e18):
                    sp_full[s][v]   = nd
                    prev_full[s][v] = u
                    heapq.heappush(heap, (nd, v))
        # Extend static rows with this dynamic column (symmetry)
        for t in range(n_static):
            d_t_s = sp_full[s].get(t, 1e18)
            if d_t_s < sp_full[t].get(s, 1e18):
                sp_full[t][s] = d_t_s
                # prev_full[t][s] not stored; obs_transit uses reverse path from s

    node_key = {tuple(np.round(n, 3)): i for i, n in enumerate(full_nodes)}
    return full_nodes, full_adj, sp_full, prev_full, node_key, n_static


# ── Obstacle-aware transit ─────────────────────────────────────────────────────
def obs_transit(p, q, full_nodes, sp_full, prev_full, node_key, n_static):
    """Shortest-path transit between two points using precomputed SP.

    Path reconstruction handles three cases:
      both static        → use prev_full[i] (from cache)
      dynamic source i   → use prev_full[i] (from Dijkstra from i)
      static i → dynamic j → reverse path: follow prev_full[j] from i to j
    """
    i = node_key.get(tuple(np.round(p, 3)))
    j = node_key.get(tuple(np.round(q, 3)))
    if i is None or j is None:
        return float('inf'), np.array([p, q])
    cost = sp_full[i].get(j, 1e18)
    if cost >= 1e18:
        return float('inf'), np.array([p, q])

    if i >= n_static or j < n_static:
        # i has full prev_full data (dynamic source, or both static)
        path, cur = [], j
        visited = set()
        while cur is not None and cur not in visited:
            visited.add(cur)
            path.append(cur)
            cur = prev_full[i].get(cur)
        if path and path[-1] == i:
            return cost, np.array([full_nodes[k] for k in reversed(path)])

    # static i → dynamic j: follow prev_full[j] starting from i
    # In undirected graph, path(i→j) can be reconstructed from prev_full[j]:
    # prev_full[j][v] = node before v in path j→v
    # Starting at i and following backwards reaches j, giving path i→j
    path, cur = [], i
    visited = set()
    while cur is not None and cur not in visited:
        visited.add(cur)
        path.append(cur)
        cur = prev_full[j].get(cur)
    return cost, np.array([full_nodes[k] for k in path])


# ── Route builder ─────────────────────────────────────────────────────────────
def build_route_field(base, stops, T_sat_h,
                      full_nodes, sp_full, prev_full, node_key, n_static,
                      TOTAL_BUDGET_KM, AIRCRAFT_SPEED_KMH, TURN_PENALTY_KM):
    pos = np.asarray(base, float)
    wpts = [pos.copy()]; segs = []
    eff_dist = geo_dist = tpv_dist = coinc_dist = 0.0
    geo_to_sat_mid = None

    def transit(src, dst):
        return obs_transit(src, dst, full_nodes, sp_full, prev_full, node_key, n_static)

    for stop in stops:
        stype = stop['type']
        if stype == 'gp':
            gp = np.asarray(stop['pt'], float)
            cost, t_wpts = transit(pos, gp)
            if cost == float('inf'): return None
            geo_t = sum(float(np.linalg.norm(t_wpts[k+1]-t_wpts[k]))
                        for k in range(len(t_wpts)-1))
            eff_dist += cost; geo_dist += geo_t
            for wp in t_wpts[1:]: wpts.append(wp); segs.append('transit')
            pos = gp

        elif stype == 'chord':
            c = stop['chord']
            pt_a, pt_b = np.asarray(c['pt_a'], float), np.asarray(c['pt_b'], float)
            cost_a, wpts_a = transit(pos, pt_a)
            cost_b, wpts_b = transit(pos, pt_b)
            if cost_a <= cost_b:
                entry, exit_, eff_t, t_wpts = pt_a, pt_b, cost_a, wpts_a
            else:
                entry, exit_, eff_t, t_wpts = pt_b, pt_a, cost_b, wpts_b
            if eff_t == float('inf'): return None
            geo_t = sum(float(np.linalg.norm(t_wpts[k+1]-t_wpts[k]))
                        for k in range(len(t_wpts)-1))
            eff_dist += eff_t; geo_dist += geo_t
            for wp in t_wpts[1:]: wpts.append(wp); segs.append('transit')
            obs_len = _append_obs_leg(c.get('waypoints'), entry, exit_, wpts, segs, 'tpv')
            tpv_dist += obs_len; eff_dist += obs_len; geo_dist += obs_len
            pos = exit_

        elif stype == 'sat':
            sa = np.asarray(stop['pt_a'], float)
            sb = np.asarray(stop['pt_b'], float)
            sl = stop['length']
            cost_a, wpts_a = transit(pos, sa)
            cost_b, wpts_b = transit(pos, sb)
            if cost_a <= cost_b:
                entry, exit_, eff_t, t_wpts = sa, sb, cost_a, wpts_a
            else:
                entry, exit_, eff_t, t_wpts = sb, sa, cost_b, wpts_b
            if eff_t == float('inf'): return None
            geo_t = sum(float(np.linalg.norm(t_wpts[k+1]-t_wpts[k]))
                        for k in range(len(t_wpts)-1))
            eff_dist += eff_t; geo_dist += geo_t
            for wp in t_wpts[1:]: wpts.append(wp); segs.append('transit')
            geo_to_sat_mid = geo_dist + sl / 2.0
            coinc_dist = sl; eff_dist += sl; geo_dist += sl
            wpts.append(exit_); segs.append('sat')
            pos = exit_

    ret_cost, ret_wpts = transit(pos, base)
    if ret_cost == float('inf'): return None
    ret_geo = sum(float(np.linalg.norm(ret_wpts[k+1]-ret_wpts[k]))
                  for k in range(len(ret_wpts)-1))
    eff_dist += ret_cost; geo_dist += ret_geo
    for wp in ret_wpts[1:]: wpts.append(wp); segs.append('transit')

    n_turns   = count_turns_in_path(wpts)
    eff_dist += n_turns * TURN_PENALTY_KM
    T_dep_h   = (T_sat_h - geo_to_sat_mid / AIRCRAFT_SPEED_KMH
                 if geo_to_sat_mid is not None else None)
    feasible  = (eff_dist <= TOTAL_BUDGET_KM and
                 (T_dep_h is None or T_dep_h >= 0.0))

    return dict(
        waypoints=np.array(wpts), seg_types=segs,
        total_dist=eff_dist, geo_dist=geo_dist,
        tpv_dist=tpv_dist, coinc_dist=coinc_dist,
        n_turns=n_turns, feasible=feasible,
        budget_remaining=TOTAL_BUDGET_KM - eff_dist, T_dep_h=T_dep_h,
    )


# ── Joint optimiser ───────────────────────────────────────────────────────────
def find_best_route_field(base, chords, gatepoints_km, sat_cands,
                          T_sat_h, full_nodes, sp_full, prev_full, node_key, n_static,
                          TOTAL_BUDGET_KM, AIRCRAFT_SPEED_KMH, TURN_PENALTY_KM,
                          min_spacing=MIN_LEG_SPACING_KM, max_n=MAX_JOINT_N):
    """Max n_chords → max coincidence → min total dist.
    sat_cands: min-arc candidates after proximity filtering.
    Returns (best_route, best_n, best_sat_entry_key).
    """
    from itertools import permutations as _iperms

    base_idx = node_key.get(tuple(np.round(base, 3)))
    gp_stops = [{'type': 'gp', 'pt': np.asarray(gp),
                 'idx': node_key.get(tuple(np.round(gp, 3)))}
                for gp in gatepoints_km]

    best = None; best_n = 0; best_coinc = -1.0; best_total = 1e18
    best_sat_entry_key = None

    for n in range(min(len(chords), max_n), 0, -1):
        found = False
        for combo in combinations(chords, n):
            offsets = sorted(c['offset'] for c in combo)
            if n > 1 and any(offsets[i+1]-offsets[i] < min_spacing
                             for i in range(n-1)):
                continue
            chord_stops = [{'type': 'chord', 'chord': c,
                            'idx_a': node_key.get(tuple(np.round(c['pt_a'], 3))),
                            'idx_b': node_key.get(tuple(np.round(c['pt_b'], 3))),
                            'length': c['length']}
                           for c in combo]

            for pa, pb, arc_km in sat_cands:
                sat_stop = {'type': 'sat', 'pt_a': pa, 'pt_b': pb, 'length': arc_km,
                            'idx_a': node_key.get(tuple(np.round(pa, 3))),
                            'idx_b': node_key.get(tuple(np.round(pb, 3)))}
                all_stops = gp_stops + chord_stops + [sat_stop]

                for perm in _iperms(range(len(all_stops))):
                    pos_i = base_idx; approx = 0.0; ok = True
                    for s in (all_stops[k] for k in perm):
                        ia = s.get('idx') if s['type'] == 'gp' else s.get('idx_a')
                        ib = s.get('idx_b')
                        if s['type'] == 'gp':
                            c_t = sp_full[pos_i].get(ia, 1e18) if ia is not None else 1e18
                            if c_t >= 1e17: ok = False; break
                            approx += c_t; pos_i = ia
                        else:
                            ca = sp_full[pos_i].get(ia, 1e18) if ia is not None else 1e18
                            cb = sp_full[pos_i].get(ib, 1e18) if ib is not None else 1e18
                            best_c = min(ca, cb)
                            if best_c >= 1e17: ok = False; break
                            approx += best_c + s['length']
                            pos_i = ib if ca <= cb else ia
                    if not ok: continue
                    rc = sp_full[pos_i].get(base_idx, 1e18)
                    if rc >= 1e17: continue
                    approx += rc
                    if approx > TOTAL_BUDGET_KM: continue

                    r = build_route_field(
                        base, [all_stops[k] for k in perm], T_sat_h,
                        full_nodes, sp_full, prev_full, node_key, n_static,
                        TOTAL_BUDGET_KM, AIRCRAFT_SPEED_KMH, TURN_PENALTY_KM)
                    if r and r['feasible']:
                        coinc_b  = r['coinc_dist'] > best_coinc + 1e-6
                        coinc_eq = abs(r['coinc_dist'] - best_coinc) <= 1e-6
                        dist_b   = r['total_dist'] < best_total - 1e-6
                        if coinc_b or (coinc_eq and dist_b):
                            best_coinc = r['coinc_dist']
                            best_total = r['total_dist']
                            best = r; best_n = n; found = True
                            best_sat_entry_key = tuple(np.round(pa, 3))
        if found:
            return best, best_n, best_sat_entry_key
    return None, 0, None


# ── Method A: post-hoc satellite arc extension ────────────────────────────────
def extend_satellite_arc(best_route, best_stop_list, sat_arc_variants,
                         sat_entry_key, T_sat_h,
                         full_nodes, sp_full, prev_full, node_key, n_static,
                         TOTAL_BUDGET_KM, AIRCRAFT_SPEED_KMH, TURN_PENALTY_KM):
    """Replace the satellite stop in best_stop_list with longer arcs until budget is exceeded.
    Returns the best (route, arc_km) — longest feasible coincidence.
    """
    variants = sat_arc_variants.get(sat_entry_key, [])
    if len(variants) <= 1:
        return best_route, best_route['coinc_dist']

    best_r   = best_route
    best_arc = best_route['coinc_dist']

    # Find which stop is the satellite stop
    sat_idx = next((i for i, s in enumerate(best_stop_list) if s['type'] == 'sat'), None)
    if sat_idx is None:
        return best_route, best_arc

    for pb_ext, arc_km in variants:
        if arc_km <= best_arc + 1e-6:
            continue  # already covered by current best
        pb_key = tuple(np.round(pb_ext, 3))
        if pb_key not in node_key:
            continue  # exit point not in graph (shouldn't happen)

        new_stops = list(best_stop_list)
        new_stops[sat_idx] = {
            'type': 'sat',
            'pt_a': best_stop_list[sat_idx]['pt_a'],
            'pt_b': pb_ext,
            'length': arc_km,
            'idx_a': best_stop_list[sat_idx]['idx_a'],
            'idx_b': node_key[pb_key],
        }
        r = build_route_field(
            np.zeros(2), new_stops, T_sat_h,
            full_nodes, sp_full, prev_full, node_key, n_static,
            TOTAL_BUDGET_KM, AIRCRAFT_SPEED_KMH, TURN_PENALTY_KM)
        if r and r['feasible']:
            best_r   = r
            best_arc = arc_km
        else:
            break  # longer arcs will also exceed budget

    return best_r, best_arc


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description='Field TPV route planner (Phase 2)')
    parser.add_argument('tpv_shp', help='TPV contour shapefile')
    parser.add_argument('gp_shp',  nargs='?', default=None,
                        help='Gatepoints shapefile (optional)')
    parser.add_argument('--cache', default='mission_cache.pkl')
    parser.add_argument('--out', default='route_result',
                        help='Output file prefix (default: route_result)')
    args = parser.parse_args()

    # ── Load cache ────────────────────────────────────────────────────────────
    t_start = time.time()
    print(f'Loading cache: {args.cache}')
    with open(args.cache, 'rb') as f:
        cache = pickle.load(f)
    P         = cache['params']
    TOTAL_BUD = P['TOTAL_BUDGET_KM']
    AC_SPEED  = P['AIRCRAFT_SPEED_KMH']
    TURN_PEN  = P['TURN_PENALTY_KM']
    BASE      = np.asarray(cache['BASE'], float)
    T_SAT_H   = cache['T_sat_h']
    ORIGIN_LON, ORIGIN_LAT = cache['origin']
    print(f'  {cache["n_static"]} static nodes | '
          f'{len(cache["sat_candidates"])} search candidates | '
          f'{sum(len(v) for v in cache["sat_arc_variants"].values())} arc variants  '
          f'[{time.time()-t_start:.1f}s]')

    def wgs84_to_km(lon, lat):
        x = (np.asarray(lon, float) - ORIGIN_LON) * cos(radians(ORIGIN_LAT)) * 111.32
        y = (np.asarray(lat, float) - ORIGIN_LAT) * 110.54
        return x, y

    # ── Load TPV contour ──────────────────────────────────────────────────────
    t0 = time.time()
    print(f'Loading TPV: {args.tpv_shp}')
    _gdf     = gpd.read_file(args.tpv_shp)
    _tpv_wgs = np.array(_gdf.geometry.iloc[0].coords)
    _x, _y   = wgs84_to_km(_tpv_wgs[:, 0], _tpv_wgs[:, 1])
    TPV_PTS  = np.column_stack([_x, _y])
    tpv_poly = Polygon(TPV_PTS).buffer(0)
    tpv_centroid = TPV_PTS.mean(axis=0)
    print(f'  {len(TPV_PTS)} vertices  centroid=({tpv_centroid[0]:.0f}, {tpv_centroid[1]:.0f}) km  '
          f'dist_from_BASE={np.linalg.norm(tpv_centroid-BASE):.0f} km')

    # ── Load gatepoints ───────────────────────────────────────────────────────
    gatepoints_km = []
    if args.gp_shp:
        _gp_gdf = gpd.read_file(args.gp_shp).to_crs('EPSG:4326')
        for _geom in _gp_gdf.geometry:
            gx, gy = wgs84_to_km(np.array([_geom.x]), np.array([_geom.y]))
            gatepoints_km.append(np.array([gx[0], gy[0]]))
        print(f'  {len(gatepoints_km)} gatepoints')

    # ── PCA + chord generation ────────────────────────────────────────────────
    center, a_fit, b_fit, phi_fit = fit_pca_ellipse(TPV_PTS)
    min_spacing = SPACING_OVERRIDE_KM if SPACING_OVERRIDE_KM else b_fit
    angle_devs  = np.linspace(-MAX_ANGLE_DEV_DEG, MAX_ANGLE_DEV_DEG,
                               N_ANGLE_SAMPLES).tolist()
    chords = generate_candidate_chords(
        tpv_poly, center, a_fit, phi_fit,
        min_spacing=min_spacing,
        angle_devs_deg=angle_devs,
        restricted_geom=cache['restricted_union'])
    print(f'Chords: {len(chords)} (a={a_fit:.0f} b={b_fit:.0f} km, '
          f'spacing={min_spacing:.0f} km)  [{time.time()-t0:.1f}s]')

    # ── Extend graph with dynamic nodes ───────────────────────────────────────
    t0 = time.time()
    dynamic_pts  = [c['pt_a'] for c in chords] + [c['pt_b'] for c in chords]
    dynamic_pts += list(gatepoints_km)
    full_nodes, full_adj, sp_full, prev_full, node_key, n_static = \
        extend_graph(cache, dynamic_pts)
    n_new = len(full_nodes) - cache['n_static']
    print(f'Graph extended: +{n_new} dynamic nodes -> {len(full_nodes)} total  '
          f'[{time.time()-t0:.1f}s]')

    # ── Proximity-filter satellite candidates ─────────────────────────────────
    d_base_tpv      = float(np.linalg.norm(tpv_centroid - BASE))
    rough_remaining = max(TOTAL_BUD - 2.0 * d_base_tpv, 200.0)
    sat_radius      = rough_remaining * SAT_PROX_FACTOR

    sat_all      = cache['sat_candidates']
    sat_filtered = [(pa, pb, arc) for pa, pb, arc in sat_all
                    if float(np.linalg.norm((pa + pb) / 2 - tpv_centroid)) < sat_radius]
    if not sat_filtered:
        sat_filtered = sat_all
        print(f'WARNING: proximity filter removed all candidates; using all {len(sat_all)}')
    else:
        print(f'Sat candidates: {len(sat_all)} -> {len(sat_filtered)} '
              f'(proximity radius={sat_radius:.0f} km from TPV)')

    # ── Search ────────────────────────────────────────────────────────────────
    print(f'\nSearching (max_n={MAX_JOINT_N})...')
    t0 = time.time()
    best_r, best_n, sat_entry_key = find_best_route_field(
        BASE, chords, gatepoints_km, sat_filtered, T_SAT_H,
        full_nodes, sp_full, prev_full, node_key, n_static,
        TOTAL_BUD, AC_SPEED, TURN_PEN,
        min_spacing=min_spacing, max_n=MAX_JOINT_N)
    t_search = time.time() - t0
    print(f'Search done [{t_search:.1f}s]')

    if best_r is None:
        print('No feasible route found.')
        return

    # ── Method A: extend satellite arc post-hoc ───────────────────────────────
    # Reconstruct the stop list for the best route to pass to extend_satellite_arc.
    # Rather than storing the full stop list during search (complex), we do a single
    # re-build of the best route and try longer arcs directly.
    arc_km_min = best_r['coinc_dist']
    arc_km_ext = arc_km_min

    if sat_entry_key and sat_entry_key in cache['sat_arc_variants']:
        variants = cache['sat_arc_variants'][sat_entry_key]
        # Find entry point and current exit
        pa_best = np.array(sat_entry_key)
        # Try each longer arc from this entry
        for pb_ext, arc_km in variants:
            if arc_km <= arc_km_ext + 1e-6:
                continue
            pb_key = tuple(np.round(pb_ext, 3))
            if pb_key not in node_key:
                continue
            # Quick budget check: can this arc even fit?
            budget_spare = best_r['budget_remaining'] - (arc_km - arc_km_min)
            if budget_spare < -TURN_PEN:  # rough check; turn count might change
                break
            # For simplicity: swap the sat stop in the best route's waypoint budget
            # A full re-search with this arc would be slow; instead use budget_remaining
            # to confirm feasibility (conservative: doesn't account for transit change).
            if best_r['budget_remaining'] >= arc_km - arc_km_min:
                arc_km_ext = arc_km
            else:
                break

    # ── Results ───────────────────────────────────────────────────────────────
    t_total = time.time() - t_start
    coinc_min = arc_km_ext / AC_SPEED * 60.0
    coinc_orig_min = arc_km_min / AC_SPEED * 60.0
    offset = best_r['T_dep_h'] - T_SAT_H
    sign   = '+' if offset >= 0 else ''
    drop_dist = route_dropsonde_dist(best_r['waypoints'], cache['dropsonde_union'])
    drop_min  = drop_dist / AC_SPEED * 60.0
    flight_h  = best_r['geo_dist'] / AC_SPEED
    flight_min = flight_h * 60.0

    print(f'\n{"="*60}')
    print(f'Total time: {t_total:.1f}s  (search {t_search:.1f}s)')
    print(f'n_chords        : {best_n}')
    print(f'TPV distance    : {best_r["tpv_dist"]:.0f} km')
    print(f'Coincidence     : {arc_km_ext:.0f} km  ({coinc_min:.0f} min)'
          + (f'  [extended from {arc_km_min:.0f} km ({coinc_orig_min:.0f} min)]'
             if arc_km_ext > arc_km_min + 1e-3 else ''))
    print(f'Dropsonde       : {drop_dist:.0f} km  ({drop_min:.0f} min)')
    print(f'Total geo. dist : {best_r["geo_dist"]:.0f} km  '
          f'| flight time: {flight_h:.2f} h  ({flight_min:.0f} min)')
    print(f'Total eff. dist : {best_r["total_dist"]:.0f} km  '
          f'(budget {TOTAL_BUD:.0f} km, remaining {best_r["budget_remaining"]:.0f} km)')
    print(f'Turns           : {best_r["n_turns"]}')
    print(f'T_dep           : T_sat {sign}{offset:.2f} h')
    print(f'{"="*60}')

    # ── Save text summary ─────────────────────────────────────────────────────
    txt_path = args.out + '.txt'
    with open(txt_path, 'w') as f:
        f.write(f'TPV shapefile   : {args.tpv_shp}\n')
        f.write(f'Gatepoints      : {args.gp_shp or "none"}\n')
        f.write(f'Cache           : {args.cache}\n')
        f.write(f'Total time      : {t_total:.1f}s  (search {t_search:.1f}s)\n')
        f.write(f'n_chords        : {best_n}\n')
        f.write(f'TPV distance    : {best_r["tpv_dist"]:.0f} km\n')
        ext_note = (f'  [extended from {arc_km_min:.0f} km ({coinc_orig_min:.0f} min)]'
                    if arc_km_ext > arc_km_min + 1e-3 else '')
        f.write(f'Coincidence     : {arc_km_ext:.0f} km  ({coinc_min:.0f} min){ext_note}\n')
        f.write(f'Dropsonde       : {drop_dist:.0f} km  ({drop_min:.0f} min)\n')
        f.write(f'Total geo. dist : {best_r["geo_dist"]:.0f} km\n')
        f.write(f'Flight time     : {flight_h:.2f} h  ({flight_min:.0f} min)\n')
        f.write(f'Total eff. dist : {best_r["total_dist"]:.0f} km  '
                f'(budget {TOTAL_BUD:.0f} km, remaining {best_r["budget_remaining"]:.0f} km)\n')
        f.write(f'Turns           : {best_r["n_turns"]}\n')
        f.write(f'T_dep           : T_sat {sign}{offset:.2f} h\n')
        f.write('\nWaypoints (km from GOOSE BAY):\n')
        for i, wp in enumerate(best_r['waypoints']):
            seg = best_r['seg_types'][i - 1] if i > 0 else 'start'
            f.write(f'  {i:3d}  ({wp[0]:8.2f}, {wp[1]:8.2f})  [{seg}]\n')
    print(f'Saved {txt_path}')

    # ── Save route map ────────────────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(12, 10))

    def _draw_shapely(geom, ax, **kw):
        if geom is None or geom.is_empty:
            return
        parts = list(geom.geoms) if geom.geom_type in ('MultiPolygon', 'GeometryCollection') else [geom]
        for part in parts:
            if part.geom_type != 'Polygon' or part.is_empty:
                continue
            x, y = part.exterior.xy
            ax.fill(x, y, **kw)

    # Obstacles
    _draw_shapely(cache['restricted_union'], ax,
                  color='#ff4444', alpha=0.25, label='Restricted airspace')
    _draw_shapely(cache['atc_union'], ax,
                  color='#ff9900', alpha=0.15, label='ATC zone')
    _draw_shapely(cache['dropsonde_union'], ax,
                  color='#88cc88', alpha=0.15, label='Dropsonde area')

    # TPV contour
    tpv_closed = np.vstack([TPV_PTS, TPV_PTS[0]])
    ax.fill(tpv_closed[:, 0], tpv_closed[:, 1],
            color='#aaddff', alpha=0.35, zorder=2)
    ax.plot(tpv_closed[:, 0], tpv_closed[:, 1],
            color='#2266cc', lw=1.5, zorder=3, label='TPV contour')

    # Satellite track
    st = cache['sat_track']['seg_pts']
    ax.plot(st[:, 0], st[:, 1], color='gray', lw=1, ls='--',
            zorder=2, label='Satellite track')

    # Route coloured by segment type (skip 'sat' — drawn separately along track)
    seg_colors = {'transit': '#666666', 'tpv': '#0055cc', 'sat': '#cc0000'}
    wpts = best_r['waypoints']
    segs = best_r['seg_types']
    for k in range(len(wpts) - 1):
        seg = segs[k] if k < len(segs) else 'transit'
        if seg == 'sat':
            continue  # drawn below using actual track geometry
        col = seg_colors.get(seg, '#333333')
        ax.plot([wpts[k, 0], wpts[k + 1, 0]], [wpts[k, 1], wpts[k + 1, 1]],
                color=col, lw=2.2, zorder=5)

    # Satellite arc: draw along actual track geometry, not chord
    if sat_entry_key is not None:
        _seg_pts = cache['sat_track']['seg_pts']
        _cum_arc = cache['sat_track']['cum_arc']
        pa_best  = np.array(sat_entry_key)
        arc_wpts = sat_arc_waypoints(pa_best, arc_km_ext, _seg_pts, _cum_arc)
        ax.plot(arc_wpts[:, 0], arc_wpts[:, 1],
                color='#cc0000', lw=2.5, zorder=6)

    # Waypoint markers
    ax.scatter(wpts[1:-1, 0], wpts[1:-1, 1], s=18, color='#333333', zorder=6)

    # Gatepoints
    if gatepoints_km:
        gp_arr = np.array(gatepoints_km)
        ax.scatter(gp_arr[:, 0], gp_arr[:, 1], s=80, marker='D',
                   color='#ff6600', zorder=7, label='Gatepoints')

    # BASE
    ax.scatter(*BASE, s=180, marker='*', color='black', zorder=8, label='BASE (Goose Bay)')

    # Legend entries for route segments
    for lab, col in [('Transit', '#666666'), ('TPV chord', '#0055cc'), ('Satellite', '#cc0000')]:
        ax.plot([], [], color=col, lw=2.2, label=lab)

    ax.set_aspect('equal')
    ax.set_xlabel('East (km)')
    ax.set_ylabel('North (km)')
    ax.set_title(
        f'Route: {best_n} TPV chord(s) | coinc={arc_km_ext:.0f} km ({coinc_min:.0f} min) '
        f'| geo={best_r["geo_dist"]:.0f} km ({flight_h:.1f} h) '
        f'| eff={best_r["total_dist"]:.0f}/{TOTAL_BUD:.0f} km',
        fontsize=10)
    ax.legend(loc='upper left', fontsize=8)
    ax.grid(True, lw=0.4, alpha=0.5)

    png_path = args.out + '.png'
    fig.savefig(png_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved {png_path}')


if __name__ == '__main__':
    main()
