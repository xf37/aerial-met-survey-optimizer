# TPV Route Planning — Project Skill

This file gives Claude full context to continue work on this repo without re-explanation.
Invoke with `/tpv-route-planning` at the start of a session.

---

## Project in One Sentence

Route-planning framework for a G3 research aircraft (5-hour / 4250 km budget) to maximise
scientific observation of a Tropopause Polar Vortex (TPV) while respecting hard airspace
constraints and secondary objectives (satellite coincidence, dropsonde zones).

---

## Notebook Map

### Synthetic Scenario (notebooks 00–03 + unit tests)

| Notebook | Role |
|----------|------|
| `00_simulation_data.ipynb` | Generate 6-cell scenario; write `data/scenario_N6_seed42.pkl` |
| `01_survey_patterns.ipynb` | Visualise cross/boustrophedon patterns; score functions |
| `02_tsp_all_visit.ipynb` | TSP baseline: NN, 2-opt, Held-Karp DP, MILP |
| `03_orienteering.ipynb` | Full OP: exhaustive → MILP → ALNS |
| `unit_test_circular.ipynb` | 3 single-cell circular scenarios |
| `unit_test_elliptical.ipynb` | 2 single-cell elliptical scenarios |
| `unit_test_combined.ipynb` | 1 circular + 1 elliptical; MILP global optimality proof |

### Real-data notebooks (NASA shapefiles in `NASA/`, git-ignored)

| Notebook | Role |
|----------|------|
| `04_nasa_data_exploration.ipynb` | Load shapefiles; reproject; PCA ellipse fit; shape analysis |
| `05_nasa_route_optimization.ipynb` | Contour Coverage Problem (CCP): exact solver + budget curves |
| `06_nasa_ellipse_route.ipynb` | Extended ellipse-based route analysis |
| `07_tpv_route_planning.ipynb` | **Main system** — general-purpose TPV route planner (Modules 1–3 done, Module 4 pending) |

Run order: `00` first (writes pickle), all others independent. `04–07` load directly from `NASA/`.

---

## Problem Formulations

### Synthetic OP (notebooks 00–03)

Two supercell types:

| Type | Pattern | Score |
|------|---------|-------|
| Circular (TPV) | Two perpendicular crossing legs | Fixed `w_k` |
| Elliptical (jet deformation) | `n` parallel legs, boustrophedon | `w_k × [CovArea + 0.5×OvlpArea] / (π a b)` |

**Key decoupling:** every survey starts and ends at cell centre → inter-cell routing sees
only a precomputed transit matrix `T[i,j]` + per-cell `(score, survey_dist)` pairs.

**Benchmark N=6:** C2→E2→E1→C3, score=12.1182, dist=2629.7 km — all four algorithms agree.

### Contour Coverage Problem / CCP (notebook 05)

Single closed PV contour. Aircraft picks entry vertex `i`, exit vertex `j`, direction `d`:

```
max   arc(i → j, d)
s.t.  T[BASE, i] + arc(i→j, d) + T[j, BASE] ≤ B
```

Solved by exact enumeration of N×N×2 triples using a precomputed cumulative arc-length
array. Runtime < 1 ms for 110 vertices. Full 1363 km contour covered at B ≈ 2800 km.

### General TPV Route Planner (notebook 07)

Aircraft: G3, `AIRCRAFT_SPEED_KMH = 850`, `FLIGHT_TIME_H = 5.0`, `TOTAL_BUDGET_KM = 4250`.

**Objectives (priority order):**
1. **Primary** — Maximise TPV observation distance via parallel chords ⊥ major axis
2. **Secondary** — Fly ≥ T_MIN_SAT_MIN minutes coincident with EarthCARE ground track
3. **Tertiary** — Maximise flight distance inside dropsonde-deployable zones

**Hard constraint:** No part of any route leg (including chord legs) may enter restricted airspace.

**Cost model:**
```
effective_dist = geometric_dist
              + (ATC_PENALTY_FACTOR - 1) × dist_inside_ATC   # fuel penalty
              + n_turns × TURN_PENALTY_KM                     # time penalty
```
where `TURN_PENALTY_KM = TURN_PENALTY_MIN / 60 × AIRCRAFT_SPEED_KMH = 106.25 km`.
Budget check uses `effective_dist`; departure time calculation uses `geometric_dist` only.

---

## Key Parameters (notebook 07)

```python
AIRCRAFT_SPEED_KMH  = 850.0
FLIGHT_TIME_H       = 5.0
TOTAL_BUDGET_KM     = 4250.0      # = FLIGHT_TIME_H × AIRCRAFT_SPEED_KMH
ATC_PENALTY_FACTOR  = 1.35
TURN_PENALTY_MIN    = 7.5
TURN_PENALTY_KM     = 106.25      # = 7.5 / 60 × 850
TURN_THRESHOLD_DEG  = 10.0        # direction change that counts as a turn
MIN_LEG_SPACING_KM  = 50.0        # spacing between parallel chords
MAX_ANGLE_DEV_DEG   = 5.0         # max deviation from chord's ideal direction
N_ANGLE_SAMPLES     = 3
T_MIN_SAT_MIN       = 10.0        # minimum satellite coincidence time (minutes)
SAT_MIN_LENGTH_KM   = T_MIN_SAT_MIN / 60 × 850  # ≈ 141.7 km
```

---

## Key Functions (notebook 07)

### Geometry

```python
seg_crosses_restricted(p1, p2, restricted_geom)
    # → True if segment intersects restricted zone (not just touches)

seg_atc_cost(p1, p2, atc_geom, penalty=ATC_PENALTY_FACTOR)
    # → effective cost: non-ATC length + ATC length × penalty

count_turns_in_path(waypoints, threshold_deg=TURN_THRESHOLD_DEG)
    # → int: number of direction changes > threshold across full waypoint sequence
    # IMPORTANT: call this on the COMPLETE waypoint list after route is built,
    # not per-segment — visibility-graph detour bends count as turns too
```

### Chord generation

```python
generate_candidate_chords(tpv_poly, center, a, phi,
                          min_spacing=MIN_LEG_SPACING_KM,
                          angle_devs_deg=None,
                          restricted_geom=None)
    # Returns list of chord dicts: {pt_a, pt_b, length, offset, angle_deg}
    # Filters out chords whose legs cross restricted_geom
    # IMPORTANT: call AFTER restricted zones are defined (not in the chord-only cell)
```

### Routing

```python
build_visibility_graph(nodes, restricted_union, atc_union)
    # → adjacency dict {i: [(j, cost), ...]}
    # edge cost = seg_atc_cost; edges crossing restricted are excluded

dijkstra(adj, src, dst)
    # → (dist, path_indices)

build_route_obs(base, ordered_chords)
    # builds full waypoint list via Dijkstra transits between chords
    # computes effective_dist including ATC penalty + turn penalty
    # uses count_turns_in_path on the FULL waypoint sequence at the end

best_multi_chord_route_obs(base, chords, n)
    # exhaustive search over all ordered subsets of n chords from filtered list
```

### Module 3 — Satellite coincidence

```python
build_route_m3(base, tpv_chords, T_sat_h, sat_first=False)
    # builds route: BASE → [chords] → [sat segment] → BASE (or sat first)
    # tracks geo_dist separately from eff_dist for timing
    # T_dep_h = T_sat_h - geo_to_sat_midpoint / AIRCRAFT_SPEED_KMH
```

**Satellite segment design:**
- Track spans 1000 km, sampled every N km to give candidate (entry, exit) pairs
- All pairs with arc ≥ SAT_MIN_LENGTH_KM are enumerated; optimizer picks best
- Segment must not cross restricted airspace
- Track defined by `SAT_REF_PT` (offset from TPV center) + `sat_phi` (angle relative to TPV major axis)

---

## Architecture Decisions

1. **All logic in notebooks, no standalone `.py` modules.** Functions duplicated across
   notebooks by design — each notebook is self-contained. Do not refactor into a shared module.

2. **Restricted zones must NOT overlap or be adjacent to the TPV polygon.**
   Zones belong on transit corridors (between BASE and TPV, or far from TPV). Placing
   restricted airspace inside TPV breaks chord generation and makes routes infeasible.

3. **Chord filtering must happen AFTER zone definitions.**
   The chord generation cell (early in notebook) runs without zone info.
   Re-run `generate_candidate_chords(..., restricted_geom=restricted_union)` in the
   vis-graph cell (after zones are defined) to get `chords_obs` for Modules 2+.

4. **Turn counting on the full path, not per-segment.**
   Inline turn estimates (e.g., "2 turns per chord") miss bends introduced by the
   visibility graph when routing around obstacles. Always call `count_turns_in_path`
   on the complete `wpts` list at route completion.

5. **Budget uses effective distance; timing uses geometric distance.**
   ATC penalty and turn penalty inflate fuel/time cost (effective_dist ≤ budget check).
   But departure time back-calculation uses only geometric_dist because the aircraft
   physically flies at 850 km/h regardless of penalty accounting.

6. **Maximum chord count via automatic search.**
   Do not fix n=1,2,3 a priori. Instead, try n = max_possible down to 1 and report
   the largest n whose effective_dist ≤ TOTAL_BUDGET_KM.

7. **Satellite entry/exit as decision variables.**
   Sample track every N km, enumerate all (entry, exit) pairs with arc ≥ SAT_MIN_LENGTH_KM,
   let the optimizer choose. Do not pre-fix the segment position.

---

## Known Bugs Fixed

| Bug | Root Cause | Fix |
|-----|-----------|-----|
| Chord legs crossing restricted airspace in Module 2 | `chords` generated before zones defined | Regenerate as `chords_obs` in vis-graph cell with `restricted_geom=restricted_union` |
| Turn count underestimated | Counted only chord entry/exit turns, missed vis-graph detour bends | Call `count_turns_in_path(wpts)` on full waypoint sequence |
| Module 3 always infeasible | Satellite segment too long (475 km clipped track vs. budget) | Use fixed `SAT_MIN_LENGTH_KM` segment with auto-shift loop to clear restricted zones |
| Zone shapes rectangular | `make_rect` used instead of irregular polygon | Use `make_irregular_poly` (convex hull of random points) |

---

## Constraints & Preferences

- **Open-source only:** `numpy`, `matplotlib`, `scipy`, `geopandas` — no ArcPy or ArcGIS tools ever
- **Workflow:** create branch + open PR at session start; all session changes on that branch
- **Language:** user communicates in Chinese; code/comments in English
- **Jupyter execution:** `C:\Program Files\ArcGIS\Pro\bin\Python\envs\arcgispro-py3\python.exe -m jupyter nbconvert --execute --to notebook --inplace <notebook>`
  (not `jupyter` directly — not in PATH on this machine)

---

## Module Status in `07_tpv_route_planning.ipynb`

| Module | Status | Description |
|--------|--------|-------------|
| Module 1 | Done | PCA ellipse fit; chord generation (no obstacles) |
| Module 2 | Done | Visibility graph + Dijkstra; obstacle-avoiding routes; turn/ATC penalties |
| Module 3 | In progress | Satellite coincidence — needs rewrite: flexible entry/exit, max chord count, multiple track examples |
| Module 4 | Pending | Full multi-objective: dropsonde zone scoring; combined objective |

### Module 3 rewrite requirements (confirmed by user)
1. Move restricted zones off TPV onto transit corridors
2. Flexible satellite (entry, exit) as decision variables — enumerate all feasible pairs
3. Find maximum feasible chord count automatically (not fixed 1/2/3)
4. Show 4 EarthCARE track examples at different positions/angles as 4-panel Figure 5
5. Longer tracks (1000 km) so optimizer must choose which sub-segment to fly

---

## Data Sources

- `NASA/test_basepoint.shp` — GOOSE BAY airbase, WGS84, lon=-60.43°, lat=53.32°
- `NASA/test_polygon.shp` — closed PV=296 contour, ESRI:102010, 110 vertices, perimeter≈1363 km
- `data/scenario_N6_seed42.pkl` — serialized 6-cell synthetic scenario (written by `00`)

---

## What to Do at Session Start

1. Read this skill file (`/tpv-route-planning`)
2. Check memory: `~/.claude/projects/C--Users-apple-aerial-met-survey-optimizer/memory/MEMORY.md`
3. Check git status and current branch; create new branch + PR if starting new work
4. Read the relevant notebook cell(s) before editing
5. Run notebook with full path to `python.exe` above
# avatar test
