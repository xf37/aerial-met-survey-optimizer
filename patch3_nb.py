"""
Patch 3 for 07_tpv_route_planning.ipynb
  1. Cell 5:  load gatepoints.shp → GATEPOINTS_KM
  2. Cell 13: add GP nodes to all_nodes (Module 2 vis-graph)
  3. Cell 14: append GP-aware route builder + Module 2 solver
  4. Cell 15: update Figure 4 to single panel with GP markers
  5. Cell 18: replace build_route_coinc / find_best_multi_coinc with
              build_route_ordered_b / find_best_multi_coinc_gp
  6. Cell 19: add GP markers to Figure 6
  7. Cell 22: replace find_best_m4 with find_best_m4_gp (fixed priority:
              n_chords -> sat -> dropsonde -> min_dist) + GP support
  8. Cell 23: add GP markers to Figure 7
"""
import json

NB = '07_tpv_route_planning.ipynb'

with open(NB, 'r', encoding='utf-8') as f:
    nb = json.load(f)
cells = nb['cells']


def src(i):
    s = cells[i]['source']
    return ''.join(s) if isinstance(s, list) else s


def set_src(i, code):
    cells[i]['source'] = code
    cells[i]['outputs'] = []
    cells[i]['execution_count'] = None


# ── 1. Cell 5: append gatepoint loading ───────────────────────────────────────
s5 = src(5)
gp_load = """

# ── Mandatory gatepoints ─────────────────────────────────────────────────────
_gp_gdf = gpd.read_file(os.path.join(DATA_DIR, 'gatepoints.shp')).to_crs('EPSG:4326')
GATEPOINTS_KM = []
for _gp_geom in _gp_gdf.geometry:
    _gx, _gy = wgs84_to_km(np.array([_gp_geom.x]), np.array([_gp_geom.y]))
    GATEPOINTS_KM.append(np.array([_gx[0], _gy[0]]))
print(f'Gatepoints : {len(GATEPOINTS_KM)} mandatory waypoints')
for _i, _gp in enumerate(GATEPOINTS_KM):
    print(f'  GP{_i}: ({_gp[0]:.1f}, {_gp[1]:.1f}) km  '
          f'dist_from_TPV={np.linalg.norm(_gp):.0f} km  '
          f'dist_from_BASE={np.linalg.norm(_gp - BASE):.0f} km')
"""
set_src(5, s5 + gp_load)
print('Cell 5: gatepoint loading appended')


# ── 2. Cell 13: add GP nodes to all_nodes before vis-graph build ──────────────
s13 = src(13)
old_nodes = """all_nodes = [np.asarray(n, float) for n in all_nodes]

BASE_IDX = 0"""
new_nodes = """all_nodes = [np.asarray(n, float) for n in all_nodes]

# Append mandatory gatepoints as vis-graph nodes (after chord/polygon nodes)
GP_IDX_M2 = []
for _gp in GATEPOINTS_KM:
    GP_IDX_M2.append(len(all_nodes))
    all_nodes.append(np.asarray(_gp, float))
print(f'Gatepoints added to Module 2 vis-graph at node indices {GP_IDX_M2}')

BASE_IDX = 0"""
assert old_nodes in s13, 'Cell 13 node block not found'
set_src(13, s13.replace(old_nodes, new_nodes, 1))
print('Cell 13: GP nodes added to all_nodes')


# ── 3. Cell 14: append GP-aware Module 2 functions ────────────────────────────
gp_m2_code = """

# ── GP-aware Module 2 route builder ──────────────────────────────────────────
def build_route_obs_ordered(base, stops):
    \"\"\"Obstacle-aware route builder with arbitrary stop order.
    Stops: list of dicts, type in ('gp', 'chord').
    \"\"\"
    pos  = np.asarray(base, float)
    wpts = [pos.copy()]; segs = []
    total_dist = tpv_dist = 0.0

    for stop in stops:
        if stop['type'] == 'gp':
            gp = np.asarray(stop['pt'], float)
            cost, t_wpts = obs_transit(pos, gp)
            if cost == float('inf'): return None
            total_dist += cost
            for wp in t_wpts[1:]: wpts.append(wp); segs.append('transit')
            pos = gp
        else:  # chord
            c = stop['chord']
            pt_a, pt_b = np.asarray(c['pt_a'], float), np.asarray(c['pt_b'], float)
            cost_a, wpts_a = obs_transit(pos, pt_a)
            cost_b, wpts_b = obs_transit(pos, pt_b)
            if cost_a <= cost_b:
                entry, exit_, eff_t, t_wpts = pt_a, pt_b, cost_a, wpts_a
            else:
                entry, exit_, eff_t, t_wpts = pt_b, pt_a, cost_b, wpts_b
            if eff_t == float('inf'): return None
            total_dist += eff_t
            for wp in t_wpts[1:]: wpts.append(wp); segs.append('transit')
            obs_len = _append_obs_leg(c.get('waypoints'), entry, exit_, wpts, segs, 'tpv')
            tpv_dist += obs_len; total_dist += obs_len
            pos = exit_

    ret_cost, ret_wpts = obs_transit(pos, base)
    if ret_cost == float('inf'): return None
    total_dist += ret_cost
    for wp in ret_wpts[1:]: wpts.append(wp); segs.append('transit')
    n_turns    = count_turns_in_path(wpts)
    total_dist += n_turns * TURN_PENALTY_KM

    return dict(waypoints=np.array(wpts), seg_types=segs,
                total_dist=total_dist, tpv_dist=tpv_dist, n_turns=n_turns,
                feasible=total_dist <= TOTAL_BUDGET_KM,
                budget_remaining=TOTAL_BUDGET_KM - total_dist)


_MAX_M2_CHORDS = 4

def find_best_route_m2(base, chords, gatepoints_km,
                        min_spacing=MIN_LEG_SPACING_KM):
    \"\"\"Best route visiting all gatepoints + max n_chords.
    Priority: max n_chords -> max TPV dist -> min total dist.
    Full permutation search over all stop orderings.
    \"\"\"
    from itertools import permutations as _iperms
    gp_stops = [{'type': 'gp', 'pt': np.asarray(gp)} for gp in gatepoints_km]
    best = None; best_n = 0; best_tpv = -1.0; best_total = 1e18

    for n in range(min(len(chords), _MAX_M2_CHORDS), 0, -1):
        found = False
        for combo in combinations(chords, n):
            offsets = sorted(c['offset'] for c in combo)
            if n > 1 and any(offsets[i+1] - offsets[i] < min_spacing
                             for i in range(n-1)):
                continue
            chord_stops = [{'type': 'chord', 'chord': c} for c in combo]
            all_stops = gp_stops + chord_stops
            for perm in _iperms(range(len(all_stops))):
                r = build_route_obs_ordered(base, [all_stops[i] for i in perm])
                if r and r['feasible']:
                    tpv_b  = r['tpv_dist'] > best_tpv  + 1e-6
                    tpv_eq = abs(r['tpv_dist'] - best_tpv) <= 1e-6
                    dist_b = r['total_dist'] < best_total - 1e-6
                    if tpv_b or (tpv_eq and dist_b):
                        best_tpv = r['tpv_dist']; best_total = r['total_dist']
                        best = r; best_n = n; found = True
        if found:
            return best, best_n
    return None, 0


import time as _time_m2
print('Module 2 (with gatepoints): searching...')
_t0_m2 = _time_m2.time()
best_r_m2, best_n_m2 = find_best_route_m2(BASE, chords_obs, GATEPOINTS_KM)
print(f'Done [{_time_m2.time()-_t0_m2:.1f}s]')
if best_r_m2:
    print(f'  n={best_n_m2}  TPV={best_r_m2["tpv_dist"]:.0f} km  '
          f'total={best_r_m2["total_dist"]:.0f} km  '
          f'turns={best_r_m2["n_turns"]}  '
          f'remaining={best_r_m2["budget_remaining"]:.0f} km')
else:
    print('  No feasible route with gatepoints found')
"""
set_src(14, src(14) + gp_m2_code)
print('Cell 14: GP-aware Module 2 functions appended')


# ── 4. Cell 15: update Figure 4 to single panel with GP markers ───────────────
fig4_code = """# ── Figure 4: Obstacle-aware route with mandatory gatepoints ─────────────────
def plot_route_obs(ax, route, tpv_pts, ell_xy, base,
                   restricted_polys, atc_polys, title):
    closed = np.vstack([tpv_pts, tpv_pts[0]])
    ax.fill(tpv_pts[:,0], tpv_pts[:,1], color='#cfe0f0', alpha=0.45, zorder=1)
    ax.plot(closed[:,0],  closed[:,1],  color='steelblue', lw=1.2, zorder=2)
    ax.plot(ell_xy[:,0],  ell_xy[:,1],  '--', color='darkorange', lw=1.2, alpha=0.5, zorder=3)

    for poly in atc_polys:
        xs, ys = poly.exterior.xy
        ax.fill(xs, ys, color='#ffe066', alpha=0.35, zorder=4)
        ax.plot(xs, ys, color='#b8860b', lw=1.2, zorder=5)
    for poly in restricted_polys:
        xs, ys = poly.exterior.xy
        ax.fill(xs, ys, color='#ff6666', alpha=0.40, zorder=6)
        ax.plot(xs, ys, color='#cc0000', lw=1.5, zorder=7)

    if route is None:
        ax.set_title(title + '\\n(no feasible route)', fontsize=10)
        ax.set_aspect('equal'); ax.grid(True, alpha=0.2)
        return

    wpts = route['waypoints']
    segs = route['seg_types']
    for i, seg_type in enumerate(segs):
        p1, p2 = wpts[i], wpts[i+1]
        if seg_type == 'tpv':
            ax.plot([p1[0],p2[0]], [p1[1],p2[1]], '-', color='#d62728',
                    lw=3, zorder=9, solid_capstyle='round')
        else:
            ax.plot([p1[0],p2[0]], [p1[1],p2[1]], '--', color='#333333',
                    lw=1.5, zorder=8, alpha=0.85)
    for i, s in enumerate(segs):
        if s == 'tpv':
            ax.plot(*wpts[i],   'o', color='green', ms=8, zorder=10)
            ax.plot(*wpts[i+1], 's', color='red',   ms=8, zorder=10)

    ax.plot(*base, '*', color='black', ms=14, zorder=11)
    ax.annotate('BASE', base, textcoords='offset points',
                xytext=(6, 4), fontsize=8, fontweight='bold')
    ax.set_aspect('equal'); ax.grid(True, alpha=0.2)
    ax.set_title(f'{title}\\n'
                 f'TPV={route["tpv_dist"]:.0f} km  total={route["total_dist"]:.0f} km  '
                 f'turns={route["n_turns"]}  remaining={route["budget_remaining"]:.0f} km',
                 fontsize=10)


fig, ax = plt.subplots(figsize=(13, 13))
plot_route_obs(ax, best_r_m2, TPV_PTS, ell_xy, BASE,
               restricted_polys, atc_polys,
               f'Module 2 — {best_n_m2}-chord route  [mandatory gatepoints]')

for _i, _gp in enumerate(GATEPOINTS_KM):
    ax.plot(*_gp, 'D', color='#ff7f0e', ms=12, zorder=12)
    ax.annotate(f'GP{_i}', _gp, textcoords='offset points',
                xytext=(8, 4), fontsize=10, fontweight='bold', color='#ff7f0e')

ax.set_xlabel('Easting (km)', fontsize=11)
ax.set_ylabel('Northing (km)', fontsize=11)

from matplotlib.lines import Line2D
from matplotlib.patches import Patch
_leg = [
    Line2D([0],[0], color='#d62728', lw=3,            label='TPV observation leg'),
    Line2D([0],[0], color='#333333', lw=1.5, ls='--', label='Transit (obstacle-avoiding)'),
    Line2D([0],[0], marker='o', color='green', ms=8, ls='none', label='Chord entry'),
    Line2D([0],[0], marker='s', color='red',   ms=8, ls='none', label='Chord exit'),
    Line2D([0],[0], marker='*', color='black', ms=12, ls='none', label='BASE'),
    Line2D([0],[0], marker='D', color='#ff7f0e', ms=10, ls='none', label='Mandatory gatepoint'),
    Patch(facecolor='#ff6666', alpha=0.5, label='Restricted Airspace'),
    Patch(facecolor='#ffe066', alpha=0.5, label='ATC Zone (penalty)'),
]
ax.legend(handles=_leg, fontsize=9, loc='lower right')
fig.suptitle(
    f'Figure 4 — Module 2: obstacle-aware route with mandatory gatepoints\\n'
    f'Budget = {TOTAL_BUDGET_KM:.0f} km  |  ATC penalty ×{ATC_PENALTY_FACTOR}  |  '
    f'Turn penalty {TURN_PENALTY_KM:.0f} km  |  Gatepoints (orange ◆) are mandatory stops',
    fontsize=11)
plt.tight_layout()
plt.show()
"""
set_src(15, fig4_code)
print('Cell 15: Figure 4 replaced with single GP panel')


# ── 5. Cell 18: replace build_route_coinc + find_best_multi_coinc ─────────────
s18 = src(18)

# Find the split point: everything before 'def build_route_coinc(' is kept
split_marker = '\ndef build_route_coinc('
split_pos = s18.find(split_marker)
assert split_pos != -1, 'build_route_coinc not found in cell 18'

keep_part = s18[:split_pos]

new_solver_code = """

# ── Generalized route builder for Module 3b / 4 ──────────────────────────────
def build_route_ordered_b(base, stops, T_sat_h):
    \"\"\"Route builder using Module 3b vis-graph.
    stops: list of dicts with type in ('gp', 'chord', 'sat').
    All orderings supported; satellite timing constraint enforced.
    \"\"\"
    pos = np.asarray(base, float)
    wpts = [pos.copy()]; segs = []
    eff_dist = geo_dist = tpv_dist = coinc_dist = 0.0
    geo_to_sat_mid = None

    for stop in stops:
        if stop['type'] == 'gp':
            gp = np.asarray(stop['pt'], float)
            cost, t_wpts = obs_transit_b(pos, gp)
            if cost == float('inf'): return None
            geo_t = sum(float(np.linalg.norm(t_wpts[k+1] - t_wpts[k]))
                        for k in range(len(t_wpts) - 1))
            eff_dist += cost; geo_dist += geo_t
            for wp in t_wpts[1:]: wpts.append(wp); segs.append('transit')
            pos = gp

        elif stop['type'] == 'chord':
            c = stop['chord']
            pt_a, pt_b = np.asarray(c['pt_a'], float), np.asarray(c['pt_b'], float)
            cost_a, wpts_a = obs_transit_b(pos, pt_a)
            cost_b, wpts_b = obs_transit_b(pos, pt_b)
            if cost_a <= cost_b:
                entry, exit_, eff_t, t_wpts = pt_a, pt_b, cost_a, wpts_a
            else:
                entry, exit_, eff_t, t_wpts = pt_b, pt_a, cost_b, wpts_b
            if eff_t == float('inf'): return None
            geo_t = sum(float(np.linalg.norm(t_wpts[k+1] - t_wpts[k]))
                        for k in range(len(t_wpts) - 1))
            eff_dist += eff_t; geo_dist += geo_t
            for wp in t_wpts[1:]: wpts.append(wp); segs.append('transit')
            obs_len = _append_obs_leg(c.get('waypoints'), entry, exit_, wpts, segs, 'tpv')
            tpv_dist += obs_len; eff_dist += obs_len; geo_dist += obs_len
            pos = exit_

        elif stop['type'] == 'sat':
            sa = np.asarray(stop['pt_a'], float)
            sb = np.asarray(stop['pt_b'], float)
            sl = stop['length']
            cost_a, wpts_a = obs_transit_b(pos, sa)
            cost_b, wpts_b = obs_transit_b(pos, sb)
            if cost_a <= cost_b:
                entry, exit_, eff_t, t_wpts = sa, sb, cost_a, wpts_a
            else:
                entry, exit_, eff_t, t_wpts = sb, sa, cost_b, wpts_b
            if eff_t == float('inf'): return None
            geo_t = sum(float(np.linalg.norm(t_wpts[k+1] - t_wpts[k]))
                        for k in range(len(t_wpts) - 1))
            eff_dist += eff_t; geo_dist += geo_t
            for wp in t_wpts[1:]: wpts.append(wp); segs.append('transit')
            geo_to_sat_mid = geo_dist + sl / 2.0
            coinc_dist = sl; eff_dist += sl; geo_dist += sl
            wpts.append(exit_); segs.append('sat')
            pos = exit_

    ret_cost, ret_wpts = obs_transit_b(pos, base)
    if ret_cost == float('inf'): return None
    ret_geo = sum(float(np.linalg.norm(ret_wpts[k+1] - ret_wpts[k]))
                  for k in range(len(ret_wpts) - 1))
    eff_dist += ret_cost; geo_dist += ret_geo
    for wp in ret_wpts[1:]: wpts.append(wp); segs.append('transit')

    n_turns  = count_turns_in_path(wpts)
    eff_dist += n_turns * TURN_PENALTY_KM
    T_dep_h  = (T_sat_h - geo_to_sat_mid / AIRCRAFT_SPEED_KMH
                if geo_to_sat_mid is not None else None)
    feasible = (eff_dist <= TOTAL_BUDGET_KM and
                (T_dep_h is None or T_dep_h >= 0.0))

    return dict(
        waypoints=np.array(wpts), seg_types=segs,
        total_dist=eff_dist, tpv_dist=tpv_dist, coinc_dist=coinc_dist,
        n_turns=n_turns, feasible=feasible,
        budget_remaining=TOTAL_BUDGET_KM - eff_dist, T_dep_h=T_dep_h,
    )


def find_best_multi_coinc_gp(base, chords, gatepoints_km, combined_candidates, T_sat_h,
                              min_spacing=MIN_LEG_SPACING_KM, max_n=MAX_JOINT_N_B):
    \"\"\"Joint optimizer with mandatory gatepoints.
    Priority: max n_chords -> max coincidence dist -> min total dist.
    Uses precomputed _eff_b for fast pruning before full route builds.
    \"\"\"
    from itertools import permutations as _iperms

    base_idx = _node_key_b.get(tuple(np.round(base, 3)))
    gp_stops = [{'type': 'gp', 'pt': np.asarray(gp),
                 'idx': _node_key_b.get(tuple(np.round(gp, 3)))}
                for gp in gatepoints_km]

    best = None; best_n = 0; best_coinc = -1.0; best_total = 1e18

    for n in range(min(len(chords), max_n), 0, -1):
        found = False
        for combo in combinations(chords, n):
            offsets = sorted(c['offset'] for c in combo)
            if n > 1 and any(offsets[i+1] - offsets[i] < min_spacing
                             for i in range(n - 1)):
                continue
            chord_stops = [{'type': 'chord', 'chord': c,
                            'idx_a': _node_key_b.get(tuple(np.round(c['pt_a'], 3))),
                            'idx_b': _node_key_b.get(tuple(np.round(c['pt_b'], 3))),
                            'length': c['length']}
                           for c in combo]

            for sa, sb, sl, tr_idx in combined_candidates:
                sat_stop = {'type': 'sat',
                            'pt_a': np.asarray(sa), 'pt_b': np.asarray(sb), 'length': sl,
                            'idx_a': _node_key_b.get(tuple(np.round(sa, 3))),
                            'idx_b': _node_key_b.get(tuple(np.round(sb, 3)))}
                all_stops = gp_stops + chord_stops + [sat_stop]
                n_stops   = len(all_stops)

                for perm in _iperms(range(n_stops)):
                    # Fast cost estimate from precomputed _eff_b (no path reconstruction)
                    pos_i = base_idx; approx = 0.0; ok = True
                    for s in (all_stops[i] for i in perm):
                        if s['type'] == 'gp':
                            c_t = _eff_b[pos_i].get(s['idx'], 1e18)
                            if c_t >= 1e17: ok = False; break
                            approx += c_t; pos_i = s['idx']
                        else:
                            ia, ib = s['idx_a'], s['idx_b']
                            ca = _eff_b[pos_i].get(ia, 1e18)
                            cb = _eff_b[pos_i].get(ib, 1e18)
                            best_c = min(ca, cb)
                            if best_c >= 1e17: ok = False; break
                            approx += best_c + s['length']
                            pos_i = ib if ca <= cb else ia
                    if not ok: continue
                    rc = _eff_b[pos_i].get(base_idx, 1e18)
                    if rc >= 1e17: continue
                    approx += rc
                    if approx > TOTAL_BUDGET_KM: continue

                    r = build_route_ordered_b(base, [all_stops[i] for i in perm], T_sat_h)
                    if r and r['feasible']:
                        coinc_b  = r['coinc_dist'] > best_coinc + 1e-6
                        coinc_eq = abs(r['coinc_dist'] - best_coinc) <= 1e-6
                        dist_b   = r['total_dist'] < best_total - 1e-6
                        if coinc_b or (coinc_eq and dist_b):
                            best_coinc = r['coinc_dist']; best_total = r['total_dist']
                            best = r; best_n = n; found = True
        if found:
            return best, best_n
    return None, 0


# ── Solve Module 3b ───────────────────────────────────────────────────────────
print()
t0 = time.time()
best_r_b, best_n_b = find_best_multi_coinc_gp(
    BASE, chords_obs, GATEPOINTS_KM, combined_cands_b, T_SAT_H)
chosen_tr = 0   # single EarthCARE track
elapsed = time.time() - t0

if best_r_b:
    offset     = best_r_b['T_dep_h'] - T_SAT_H
    sign       = '+' if offset >= 0 else ''
    coinc_min  = best_r_b['coinc_dist'] / AIRCRAFT_SPEED_KMH * 60.0
    print(f'Best solution found in {elapsed:.1f}s:')
    print(f'  TPV chords (n)    : {best_n_b}')
    print(f'  TPV distance      : {best_r_b["tpv_dist"]:.0f} km')
    print(f'  Coincidence seg   : {best_r_b["coinc_dist"]:.0f} km ({coinc_min:.0f} min)')
    print(f'  Total eff. dist   : {best_r_b["total_dist"]:.0f} km'
          f'  (budget {TOTAL_BUDGET_KM:.0f} km,'
          f' remaining {best_r_b["budget_remaining"]:.0f} km)')
    print(f'  Turns             : {best_r_b["n_turns"]}')
    print(f'  T_dep             : T_coinc {sign}{offset:.2f} h')
else:
    print(f'No feasible route found  [{elapsed:.1f}s]')

sat_track_defs = coinc_track_defs   # alias used by Module 4
"""

set_src(18, keep_part + new_solver_code)
print('Cell 18: build_route_ordered_b + find_best_multi_coinc_gp installed')


# ── 6. Cell 19: add GP markers to Figure 6 ────────────────────────────────────
s19 = src(19)
old_base_annot = """ax.plot(*BASE, '*', color='black', ms=18, zorder=13)
ax.annotate('BASE', BASE, textcoords='offset points',
            xytext=(8, 4), fontsize=10, fontweight='bold')"""
new_base_annot = """ax.plot(*BASE, '*', color='black', ms=18, zorder=13)
ax.annotate('BASE', BASE, textcoords='offset points',
            xytext=(8, 4), fontsize=10, fontweight='bold')

for _i, _gp in enumerate(GATEPOINTS_KM):
    ax.plot(*_gp, 'D', color='#ff7f0e', ms=12, zorder=14)
    ax.annotate(f'GP{_i}', _gp, textcoords='offset points',
                xytext=(8, 4), fontsize=9, fontweight='bold', color='#ff7f0e')"""
assert old_base_annot in s19, 'BASE annotation block not found in cell 19'
s19 = s19.replace(old_base_annot, new_base_annot, 1)

# Add GP to legend
old_leg_end = """    Line2D([0],[0], marker='s', color='red',   ms=7, ls='none', label='Leg exit'),
    Patch(facecolor='#ff6666', alpha=0.5, label='Restricted Airspace'),"""
new_leg_end = """    Line2D([0],[0], marker='s', color='red',   ms=7, ls='none', label='Leg exit'),
    Line2D([0],[0], marker='D', color='#ff7f0e', ms=9, ls='none', label='Mandatory gatepoint'),
    Patch(facecolor='#ff6666', alpha=0.5, label='Restricted Airspace'),"""
assert old_leg_end in s19, 'Legend block not found in cell 19'
s19 = s19.replace(old_leg_end, new_leg_end, 1)

set_src(19, s19)
print('Cell 19: GP markers added to Figure 6')


# ── 7. Cell 22: replace find_best_m4 with GP-aware version + fixed priority ───
new_cell22 = """# ── Dropsonde scoring helper ─────────────────────────────────────────────────
def route_dropsonde_dist(wpts, dropsonde_geom):
    \"\"\"Total geometric distance inside dropsonde zones across all route segments.\"\"\"
    total = 0.0
    pts = np.asarray(wpts, float)
    for k in range(len(pts) - 1):
        seg = LineString([pts[k], pts[k+1]])
        inside = seg.intersection(dropsonde_geom)
        total += inside.length if not inside.is_empty else 0.0
    return total


def build_route_m4_gp(base, stops, T_sat_h):
    \"\"\"Module 4 route builder: generalized stops + dropsonde coverage scoring.\"\"\"
    r = build_route_ordered_b(base, stops, T_sat_h)
    if r is None:
        return None
    r['sat_dist']       = r['coinc_dist']
    r['dropsonde_dist'] = route_dropsonde_dist(r['waypoints'], dropsonde_union)
    return r


def find_best_m4_gp(base, chords, gatepoints_km, sat_candidates, T_sat_h,
                    min_spacing=MIN_LEG_SPACING_KM, max_n=MAX_JOINT_N):
    \"\"\"Module 4 joint optimizer with mandatory gatepoints.
    Priority: max n_chords -> max sat time (cap ideal) -> max dropsonde -> min total dist.
    \"\"\"
    from itertools import permutations as _iperms

    base_idx = _node_key_b.get(tuple(np.round(base, 3)))
    gp_stops = [{'type': 'gp', 'pt': np.asarray(gp),
                 'idx': _node_key_b.get(tuple(np.round(gp, 3)))}
                for gp in gatepoints_km]

    best = None; best_n = 0
    best_sat = -1.0; best_drop = -1.0; best_total = 1e18

    for n in range(min(len(chords), max_n), 0, -1):
        found = False
        for combo in combinations(chords, n):
            offsets = sorted(c['offset'] for c in combo)
            if n > 1 and any(offsets[i+1] - offsets[i] < min_spacing
                             for i in range(n - 1)):
                continue
            chord_stops = [{'type': 'chord', 'chord': c,
                            'idx_a': _node_key_b.get(tuple(np.round(c['pt_a'], 3))),
                            'idx_b': _node_key_b.get(tuple(np.round(c['pt_b'], 3))),
                            'length': c['length']}
                           for c in combo]

            for sa, sb, sl in sat_candidates:
                sat_stop = {'type': 'sat',
                            'pt_a': np.asarray(sa), 'pt_b': np.asarray(sb), 'length': sl,
                            'idx_a': _node_key_b.get(tuple(np.round(sa, 3))),
                            'idx_b': _node_key_b.get(tuple(np.round(sb, 3)))}
                all_stops = gp_stops + chord_stops + [sat_stop]
                n_stops   = len(all_stops)

                for perm in _iperms(range(n_stops)):
                    pos_i = base_idx; approx = 0.0; ok = True
                    for s in (all_stops[i] for i in perm):
                        if s['type'] == 'gp':
                            c_t = _eff_b[pos_i].get(s['idx'], 1e18)
                            if c_t >= 1e17: ok = False; break
                            approx += c_t; pos_i = s['idx']
                        else:
                            ia, ib = s['idx_a'], s['idx_b']
                            ca = _eff_b[pos_i].get(ia, 1e18)
                            cb = _eff_b[pos_i].get(ib, 1e18)
                            best_c = min(ca, cb)
                            if best_c >= 1e17: ok = False; break
                            approx += best_c + s['length']
                            pos_i = ib if ca <= cb else ia
                    if not ok: continue
                    rc = _eff_b[pos_i].get(base_idx, 1e18)
                    if rc >= 1e17: continue
                    approx += rc
                    if approx > TOTAL_BUDGET_KM: continue

                    r = build_route_m4_gp(base, [all_stops[i] for i in perm], T_sat_h)
                    if r and r['feasible']:
                        sat_sc  = min(r['sat_dist'], SAT_IDEAL_LENGTH_KM)
                        drop_sc = r['dropsonde_dist']
                        sat_b   = sat_sc  > best_sat  + 1e-6
                        sat_eq  = abs(sat_sc  - best_sat)  <= 1e-6
                        drop_b  = drop_sc > best_drop + 1e-6
                        drop_eq = abs(drop_sc - best_drop) <= 1e-6
                        dist_b  = r['total_dist'] < best_total - 1e-6
                        if (sat_b or
                            (sat_eq and drop_b) or
                            (sat_eq and drop_eq and dist_b)):
                            best_sat   = sat_sc;  best_drop  = drop_sc
                            best_total = r['total_dist']
                            best = r; best_n = n; found = True
        if found:
            return best, best_n
    return None, 0


print()
m4_results = {}
for tr in sat_track_defs:
    t0 = time.time()
    t_sat = tr.get('T_sat_h', T_SAT_H)
    r, n = find_best_m4_gp(BASE, chords_obs, GATEPOINTS_KM, tr['candidates'], t_sat)
    elapsed = time.time() - t0
    m4_results[tr['label']] = {'route': r, 'n_chords': n, 'track': tr, 'T_sat_h': t_sat}
    if r:
        r['T_sat_h'] = t_sat
        drop_min = r['dropsonde_dist'] / AIRCRAFT_SPEED_KMH * 60.0
        sat_min  = r['sat_dist']       / AIRCRAFT_SPEED_KMH * 60.0
        offset   = r['T_dep_h'] - t_sat
        sign     = '+' if offset >= 0 else ''
        print(f"{tr['label']}: n={n}  TPV={r['tpv_dist']:.0f} km  "
              f"sat={r['sat_dist']:.0f} km ({sat_min:.0f} min)  "
              f"drop={r['dropsonde_dist']:.0f} km ({drop_min:.0f} min)  "
              f"total={r['total_dist']:.0f} km  "
              f"turns={r['n_turns']}  T_dep=T_sat{sign}{offset:.2f}h  [{elapsed:.1f}s]")
    else:
        print(f"{tr['label']}: no feasible route  [{elapsed:.1f}s]")
"""
set_src(22, new_cell22)
print('Cell 22: find_best_m4_gp installed with n_chords -> sat -> drop -> dist priority')


# ── 8. Cell 23: add GP markers to Figure 7 ────────────────────────────────────
s23 = src(23)
old_base23 = """    ax.plot(*base, '*', color='black', ms=14, zorder=11)
    ax.annotate('BASE', base, textcoords='offset points',
                xytext=(6, 4), fontsize=8, fontweight='bold')
    ax.set_aspect('equal'); ax.grid(True, alpha=0.2)"""
new_base23 = """    ax.plot(*base, '*', color='black', ms=14, zorder=11)
    ax.annotate('BASE', base, textcoords='offset points',
                xytext=(6, 4), fontsize=8, fontweight='bold')
    for _i_gp, _gp_pt in enumerate(GATEPOINTS_KM):
        ax.plot(*_gp_pt, 'D', color='#ff7f0e', ms=11, zorder=12)
        ax.annotate(f'GP{_i_gp}', _gp_pt, textcoords='offset points',
                    xytext=(6, 3), fontsize=8, fontweight='bold', color='#ff7f0e')
    ax.set_aspect('equal'); ax.grid(True, alpha=0.2)"""
assert old_base23 in s23, 'BASE annotation block not found in cell 23'
s23 = s23.replace(old_base23, new_base23, 1)

# Add GP to legend
old_leg23 = """    Line2D([0],[0], marker='s', color='red',   ms=7, ls='none', label='Leg exit'),
    Line2D([0],[0], marker='*', color='black', ms=12, ls='none', label='BASE'),"""
new_leg23 = """    Line2D([0],[0], marker='s', color='red',   ms=7, ls='none', label='Leg exit'),
    Line2D([0],[0], marker='*', color='black', ms=12, ls='none', label='BASE'),
    Line2D([0],[0], marker='D', color='#ff7f0e', ms=9, ls='none', label='Mandatory gatepoint'),"""
assert old_leg23 in s23, 'Legend block not found in cell 23'
s23 = s23.replace(old_leg23, new_leg23, 1)

# Update suptitle
old_title23 = "f'Figure 7 — Module 4: full multi-objective route (EarthCARE overpass 2016-05-16)\\n'\n    f'Priority: max chords → max dropsonde coverage → max sat (ideal={T_IDEAL_SAT_MIN:.0f} min) → min dist\\n'"
new_title23 = "f'Figure 7 — Module 4: full multi-objective route (EarthCARE overpass 2016-05-16)\\n'\n    f'Priority: max chords → max sat (ideal={T_IDEAL_SAT_MIN:.0f} min) → max dropsonde → min dist\\n'"
if old_title23 in s23:
    s23 = s23.replace(old_title23, new_title23, 1)
else:
    print('  WARNING: Figure 7 suptitle not matched — skipping title update')

set_src(23, s23)
print('Cell 23: GP markers added to Figure 7')

# ── Also clear outputs of cells that feed results ─────────────────────────────
for idx in [5, 13, 14, 15, 18, 19, 22, 23]:
    cells[idx]['outputs'] = []
    cells[idx]['execution_count'] = None

with open(NB, 'w', encoding='utf-8') as f:
    json.dump(nb, f, ensure_ascii=False, indent=1)
print(f'\nSaved. Total cells: {len(nb["cells"])}')
