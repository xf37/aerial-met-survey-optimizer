"""
Patch 5: Simplify Module 4 — reuse Module 3b result, compute dropsonde post-hoc.
Dropsonde is a passive measurement (no route detours), so search is identical to
Module 3b. This eliminates the per-route dropsonde shapely cost and the timeout.
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

# ── Replace cell 22 ───────────────────────────────────────────────────────────
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


# ── Module 4: reuse Module 3b search, add post-hoc dropsonde measurement ─────
# Dropsonde coverage is a passive metric: it does not drive routing decisions.
# We use the same find_best_multi_coinc_gp search as Module 3b, then measure
# how much of that optimal route overlaps with dropsonde deployment zones.
print()
m4_results = {}
for tr in sat_track_defs:
    t0 = time.time()
    t_sat = tr.get('T_sat_h', T_SAT_H)
    r, n = find_best_multi_coinc_gp(
        BASE, chords_obs, GATEPOINTS_KM, combined_cands_b, t_sat,
        max_n=MAX_JOINT_N)
    elapsed = time.time() - t0

    if r is not None:
        r['sat_dist']       = r['coinc_dist']
        r['T_sat_h']        = t_sat
        r['dropsonde_dist'] = route_dropsonde_dist(r['waypoints'], dropsonde_union)

    m4_results[tr['label']] = {'route': r, 'n_chords': n, 'track': tr, 'T_sat_h': t_sat}

    if r:
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
print('Cell 22: Module 4 simplified to post-hoc dropsonde measurement')

with open(NB, 'w', encoding='utf-8') as f:
    json.dump(nb, f, ensure_ascii=False, indent=1)
print('Saved.')
