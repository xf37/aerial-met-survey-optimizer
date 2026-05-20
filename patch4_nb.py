"""
Patch 4:
  1. Fix markdown cell 4 (remove invalid execution_count/outputs fields)
  2. Add all-pairs Dijkstra precomputation to cell 13 (for Module 2 quick-eval)
  3. Add quick_eval pruning to find_best_route_m2 in cell 14
"""
import json

NB = '07_tpv_route_planning.ipynb'
with open(NB, 'r', encoding='utf-8') as f:
    nb = json.load(f)
cells = nb['cells']

# ── 1. Fix markdown cells that incorrectly have execution_count/outputs ────────
fixed = 0
for c in cells:
    if c.get('cell_type') == 'markdown':
        if 'execution_count' in c or 'outputs' in c:
            c.pop('execution_count', None)
            c.pop('outputs', None)
            fixed += 1
print(f'Fixed {fixed} markdown cells')

# ── 2. Add all-pairs Dijkstra to cell 13 (Module 2 precomputed distances) ──────
def src(i):
    s = cells[i]['source']
    return ''.join(s) if isinstance(s, list) else s

def set_src(i, code):
    cells[i]['source'] = code
    cells[i]['outputs'] = []
    cells[i]['execution_count'] = None

s13 = src(13)
old_print = "print(f'Done: {n_edges} valid edges')"
new_print = """print(f'Done: {n_edges} valid edges')

# Precompute all-pairs shortest paths for Module 2 (enables fast pruning in find_best_route_m2)
_N_M2   = len(all_nodes)
_eff_m2 = [dict() for _ in range(_N_M2)]
for _src in range(_N_M2):
    _eff_m2[_src][_src] = 0.0
    _heap = [(0.0, _src)]
    while _heap:
        _d, _u = heapq.heappop(_heap)
        if _d > _eff_m2[_src].get(_u, 1e18):
            continue
        for _v, _w in adj[_u]:
            _nd = _d + _w
            if _nd < _eff_m2[_src].get(_v, 1e18):
                _eff_m2[_src][_v] = _nd
                heapq.heappush(_heap, (_nd, _v))
_node_key_m2 = {tuple(np.round(_n, 3)): _i for _i, _n in enumerate(all_nodes)}
print(f'Module 2 all-pairs SP precomputed ({_N_M2} nodes)')"""

assert old_print in s13, 'Done: edges line not found in cell 13'
set_src(13, s13.replace(old_print, new_print, 1))
print('Cell 13: all-pairs Dijkstra precomputation added')

# ── 3. Replace find_best_route_m2 in cell 14 with quick-eval version ───────────
s14 = src(14)

old_fn = '''def find_best_route_m2(base, chords, gatepoints_km,
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
    return None, 0'''

new_fn = '''def find_best_route_m2(base, chords, gatepoints_km,
                        min_spacing=MIN_LEG_SPACING_KM):
    \"\"\"Best route visiting all gatepoints + max n_chords.
    Priority: max n_chords -> max TPV dist -> min total dist.
    Uses precomputed _eff_m2 for fast budget pruning before full route builds.
    \"\"\"
    from itertools import permutations as _iperms
    base_idx = _node_key_m2.get(tuple(np.round(base, 3)))
    gp_stops = [{'type': 'gp', 'pt': np.asarray(gp),
                 'idx': _node_key_m2.get(tuple(np.round(gp, 3)))}
                for gp in gatepoints_km]
    best = None; best_n = 0; best_tpv = -1.0; best_total = 1e18

    for n in range(min(len(chords), _MAX_M2_CHORDS), 0, -1):
        found = False
        for combo in combinations(chords, n):
            offsets = sorted(c['offset'] for c in combo)
            if n > 1 and any(offsets[i+1] - offsets[i] < min_spacing
                             for i in range(n-1)):
                continue
            chord_stops = [{'type': 'chord', 'chord': c,
                            'idx_a': _node_key_m2.get(tuple(np.round(c['pt_a'], 3))),
                            'idx_b': _node_key_m2.get(tuple(np.round(c['pt_b'], 3))),
                            'length': c['length']}
                           for c in combo]
            all_stops = gp_stops + chord_stops
            for perm in _iperms(range(len(all_stops))):
                # Fast cost estimate using precomputed _eff_m2
                pos_i = base_idx; approx = 0.0; ok = True
                for s in (all_stops[i] for i in perm):
                    if s['type'] == 'gp':
                        c_t = _eff_m2[pos_i].get(s['idx'], 1e18)
                        if c_t >= 1e17: ok = False; break
                        approx += c_t; pos_i = s['idx']
                    else:
                        ia, ib = s['idx_a'], s['idx_b']
                        ca = _eff_m2[pos_i].get(ia, 1e18)
                        cb = _eff_m2[pos_i].get(ib, 1e18)
                        best_c = min(ca, cb)
                        if best_c >= 1e17: ok = False; break
                        approx += best_c + s['length']
                        pos_i = ib if ca <= cb else ia
                if not ok: continue
                rc = _eff_m2[pos_i].get(base_idx, 1e18)
                if rc >= 1e17: continue
                approx += rc
                if approx > TOTAL_BUDGET_KM: continue

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
    return None, 0'''

assert old_fn in s14, 'find_best_route_m2 not found in cell 14'
set_src(14, s14.replace(old_fn, new_fn, 1))
print('Cell 14: find_best_route_m2 updated with quick-eval pruning')

with open(NB, 'w', encoding='utf-8') as f:
    json.dump(nb, f, ensure_ascii=False, indent=1)
print(f'Saved. Total cells: {len(nb["cells"])}')
