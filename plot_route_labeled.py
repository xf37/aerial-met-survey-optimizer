"""
Regenerate route map with labeled waypoints from saved route_result.txt.
No optimization needed — reads cache + text output + shapefiles only.
"""
import pickle, re
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import geopandas as gpd
from math import cos, radians
from shapely.geometry import Polygon

# ── Parse route_result.txt ────────────────────────────────────────────────────
meta = {}
wpts_list = []
segs_at = []   # segs_at[i] = arriving segment label for wpts[i]

with open('route_result.txt') as f:
    lines = f.readlines()

in_wps = False
for line in lines:
    s = line.strip()
    if not s:
        continue
    if s.startswith('TPV shapefile'):
        meta['tpv_shp'] = s.split(':', 1)[1].strip()
    elif s.startswith('Gatepoints'):
        meta['gp_shp'] = s.split(':', 1)[1].strip()
    elif s.startswith('Coincidence'):
        m = re.search(r':\s*([\d.]+)\s*km', s)
        if m: meta['arc_km'] = float(m.group(1))
    elif 'Waypoints' in s:
        in_wps = True
    elif in_wps:
        m = re.match(r'\s*(\d+)\s+\(\s*([-\d.]+),\s+([-\d.]+)\)\s+\[(\w+)\]', s)
        if m:
            wpts_list.append(np.array([float(m.group(2)), float(m.group(3))]))
            segs_at.append(m.group(4))

wpts = np.array(wpts_list)
arc_km_ext = meta.get('arc_km', 85.0)

# segs_at[i] = label written for wpts[i] in txt = seg_types[i-1] (departing from wpts[i-1])
# Departing segment from wpts[i] to wpts[i+1] = segs_at[i+1]
def depart_seg(i):
    return segs_at[i + 1] if i + 1 < len(segs_at) else 'transit'

# Satellite arc: segment from wpts[sat_k] to wpts[sat_k+1] is 'sat'
sat_k = next((i for i in range(len(wpts) - 1) if depart_seg(i) == 'sat'), None)

# ── Load cache ────────────────────────────────────────────────────────────────
with open('mission_cache.pkl', 'rb') as f:
    cache = pickle.load(f)

ORIGIN_LON, ORIGIN_LAT = cache['origin']
BASE = np.asarray(cache['BASE'], float)

def wgs84_to_km(lon, lat):
    x = (np.asarray(lon, float) - ORIGIN_LON) * cos(radians(ORIGIN_LAT)) * 111.32
    y = (np.asarray(lat, float) - ORIGIN_LAT) * 110.54
    return x, y

# ── Load TPV contour ──────────────────────────────────────────────────────────
_gdf    = gpd.read_file(meta['tpv_shp'])
_tpv_wg = np.array(_gdf.geometry.iloc[0].coords)
_x, _y  = wgs84_to_km(_tpv_wg[:, 0], _tpv_wg[:, 1])
TPV_PTS = np.column_stack([_x, _y])

# ── Load gatepoints ───────────────────────────────────────────────────────────
gatepoints_km = []
if meta.get('gp_shp', 'none') != 'none':
    _gp_gdf = gpd.read_file(meta['gp_shp']).to_crs('EPSG:4326')
    for _geom in _gp_gdf.geometry:
        gx, gy = wgs84_to_km(np.array([_geom.x]), np.array([_geom.y]))
        gatepoints_km.append(np.array([gx[0], gy[0]]))

# ── Satellite arc geometry ────────────────────────────────────────────────────
def arc_interp(pts, cum, t):
    k = int(np.searchsorted(cum, t, side='right')) - 1
    k = max(0, min(len(pts) - 2, k))
    frac = (t - cum[k]) / max(float(np.linalg.norm(pts[k+1] - pts[k])), 1e-9)
    return pts[k] + frac * (pts[k+1] - pts[k])

seg_pts = cache['sat_track']['seg_pts']
cum_arc = cache['sat_track']['cum_arc']

arc_wpts = None
if sat_k is not None:
    sat_entry_pt = wpts[sat_k].copy()
    sat_exit_pt  = wpts[sat_k + 1].copy()
    t_entry = float(cum_arc[int(np.argmin(np.linalg.norm(seg_pts - sat_entry_pt, axis=1)))])
    t_exit  = float(cum_arc[int(np.argmin(np.linalg.norm(seg_pts - sat_exit_pt,  axis=1)))])
    n_pts   = max(80, int(arc_km_ext / 5) + 2)
    ts      = np.linspace(t_entry, t_exit, n_pts)
    arc_wpts = np.array([arc_interp(seg_pts, cum_arc, t) for t in ts])
    arc_wpts[0]  = sat_entry_pt
    arc_wpts[-1] = sat_exit_pt

# ── Draw ──────────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(13, 11))

def _draw_shapely(geom, ax, **kw):
    if geom is None or geom.is_empty:
        return
    parts = list(geom.geoms) if geom.geom_type in ('MultiPolygon', 'GeometryCollection') else [geom]
    for part in parts:
        if part.geom_type != 'Polygon' or part.is_empty:
            continue
        x, y = part.exterior.xy
        ax.fill(x, y, **kw)

_draw_shapely(cache['restricted_union'], ax, color='#ff4444', alpha=0.25, label='Restricted airspace')
_draw_shapely(cache['atc_union'],        ax, color='#ff9900', alpha=0.15, label='ATC zone')
_draw_shapely(cache['dropsonde_union'],  ax, color='#88cc88', alpha=0.15, label='Dropsonde area')

tpv_closed = np.vstack([TPV_PTS, TPV_PTS[0]])
ax.fill(tpv_closed[:, 0], tpv_closed[:, 1], color='#aaddff', alpha=0.35, zorder=2)
ax.plot(tpv_closed[:, 0], tpv_closed[:, 1], color='#2266cc', lw=1.5, zorder=3, label='TPV contour')

ax.plot(seg_pts[:, 0], seg_pts[:, 1], color='gray', lw=1, ls='--', zorder=2, label='Satellite track')

# Route segments (skip sat — drawn separately)
seg_colors = {'transit': '#666666', 'tpv': '#0055cc', 'sat': '#cc0000'}
for i in range(len(wpts) - 1):
    seg = depart_seg(i)
    if seg == 'sat':
        continue
    col = seg_colors.get(seg, '#333333')
    ax.plot([wpts[i, 0], wpts[i+1, 0]], [wpts[i, 1], wpts[i+1, 1]],
            color=col, lw=2.2, zorder=5)

# Satellite arc along track geometry
if arc_wpts is not None:
    ax.plot(arc_wpts[:, 0], arc_wpts[:, 1], color='#666666', lw=2.2, zorder=5)
    ax.plot(arc_wpts[:, 0], arc_wpts[:, 1], color='#cc0000', lw=2.5, zorder=6)

# Gatepoints
if gatepoints_km:
    gp_arr = np.array(gatepoints_km)
    ax.scatter(gp_arr[:, 0], gp_arr[:, 1], s=80, marker='D',
               color='#ff6600', zorder=7, label='Gatepoints')

# BASE
ax.scatter(*BASE, s=200, marker='*', color='black', zorder=9, label='BASE (Goose Bay)')

# Waypoint markers + labels
# Label all intermediate waypoints (skip 0=BASE start, skip last=BASE end)
for i in range(1, len(wpts) - 1):
    ax.scatter(wpts[i, 0], wpts[i, 1], s=30, color='#333333', zorder=8)
    # Offset label slightly to avoid overlap
    dx, dy = 18, 18
    label_txt = str(i)
    # Mark satellite entry and exit distinctly
    if sat_k is not None and i == sat_k:
        label_txt = f'{i}\n(SAT in)'
        ax.scatter(wpts[i, 0], wpts[i, 1], s=90, color='#cc0000', zorder=9, marker='o', facecolors='none', linewidths=2)
    elif sat_k is not None and i == sat_k + 1:
        label_txt = f'{i}\n(SAT out)'
        ax.scatter(wpts[i, 0], wpts[i, 1], s=90, color='#cc0000', zorder=9, marker='s', facecolors='none', linewidths=2)
    ax.annotate(label_txt, wpts[i], xytext=(dx, dy), textcoords='offset points',
                fontsize=8, fontweight='bold', color='#111111', zorder=10,
                bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.7, edgecolor='none'))

# Legend entries
for lab, col in [('Transit', '#666666'), ('TPV chord', '#0055cc'), ('Satellite coincidence', '#cc0000')]:
    ax.plot([], [], color=col, lw=2.2, label=lab)

ax.set_aspect('equal')
ax.set_xlabel('East (km)')
ax.set_ylabel('North (km)')
coinc_min = arc_km_ext / cache['params']['AIRCRAFT_SPEED_KMH'] * 60.0
ax.set_title(f'Route (labeled waypoints) | coincidence={arc_km_ext:.0f} km ({coinc_min:.0f} min)\n'
             f'Red circle = SAT entry (wp{sat_k})  |  Red square = SAT exit (wp{sat_k+1})',
             fontsize=10)
ax.legend(loc='upper left', fontsize=8)
ax.grid(True, lw=0.4, alpha=0.5)

out = 'route_result_labeled.png'
fig.savefig(out, dpi=150, bbox_inches='tight')
plt.close(fig)
print(f'Saved {out}')
print(f'\nWaypoint summary:')
for i, (wp, seg) in enumerate(zip(wpts, segs_at)):
    note = ''
    if sat_k is not None and i == sat_k:     note = '  <- SAT entry (circle)'
    if sat_k is not None and i == sat_k + 1: note = '  <- SAT exit  (square)'
    print(f'  wp{i:2d}  ({wp[0]:8.2f}, {wp[1]:8.2f})  [{seg}]{note}')
