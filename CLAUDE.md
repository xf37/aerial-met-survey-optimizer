# Aerial Met Survey Optimizer

Route-planning framework for airborne meteorological observation missions.
Core problem: find the optimal flight path maximising scientific score within a
flight distance budget — a variant of the **Orienteering Problem (OP)**.

---

## Problem Structure

### Two supercell types

| Type | Pattern | Score |
|------|---------|-------|
| **Circular** (TPV) | Two perpendicular crossing legs through cell centre | `w_k` (fixed) |
| **Elliptical** (jet deformation zone) | `n` parallel legs, boustrophedon (lawnmower) | `w_k × coverage_fraction` |

**Circular score:** constant `w_k` regardless of scan direction θ or leg length L.
Arms are reversible; task ordering (arms + sensitivity points) is found by exhaustive
permutation inside `route_circ_from()`.

**Elliptical score:**
```
f_k(n, s) = w_k × [CovArea(n, s) + 0.5 × OvlpArea(n, s)] / (π · a · b)
```
Coverage is computed on a 2-km grid inside `compute_coverage()`. More legs → higher
score but longer survey distance — the solver must balance the trade-off.

**Key decoupling:** each cell's survey starts and ends at its own centre, so internal
survey geometry is independent of inter-cell routing. The OP solver only sees a
precomputed transit matrix `T[i,j]` plus per-cell `(score, survey_dist)` pairs.

### Orienteering Problem formulation

```
max  Σ_k  f_k · y_k
s.t. Σ T[i,j]·x[ij] + Σ d_k·y_k ≤ B    (budget)
     flow conservation + subtour elimination (MTZ)
     x[ij], y_k ∈ {0,1}
```

Budget B = midpoint between minimum 4-cell and minimum 5-cell tour costs, forcing
non-trivial subset selection among all 6 cells.

### Obstacle avoidance

`obs_free(start, end, polygon_verts)` returns the shortest obstacle-avoiding path
(direct if clear, otherwise via polygon vertices). `multi_obs()` chains this across
multiple obstacles. The obstacle polygons must be **CCW** (enforced by `ensure_ccw()`).

---

## Code Layout

All logic lives in Jupyter notebooks — there are no standalone Python modules.
Utility functions are **duplicated** across notebooks (by design for self-containment).

### Simulation notebooks (synthetic scenario)

| Notebook | Role |
|----------|------|
| `00_simulation_data.ipynb` | Define scenario; write `data/scenario_N6_seed42.pkl` |
| `01_survey_patterns.ipynb` | Visualise cross/boustrophedon patterns; score analysis |
| `02_tsp_all_visit.ipynb` | TSP baseline: NN, 2-opt, Held-Karp DP, MILP |
| `03_orienteering.ipynb` | Full OP: precompute → exhaustive → MILP → ALNS |
| `unit_test_circular.ipynb` | 3 single-cell circular scenarios (with/without obstacle) |
| `unit_test_elliptical.ipynb` | 2 single-cell elliptical scenarios |
| `unit_test_combined.ipynb` | 1 circular + 1 elliptical; MILP global optimality proof |

Run order: `00` first (writes the pickle), then any other notebook independently.
Unit test notebooks are self-contained (no pickle dependency).

### Real-data notebooks (NASA shapefiles)

Raw data lives in `NASA/` (git-ignored). Two shapefiles:
- `NASA/test_basepoint.shp` — GOOSE BAY airbase, WGS84, lon=-60.43°, lat=53.32°
- `NASA/test_polygon.shp` — closed PV=296 contour (TPV boundary), ESRI:102010, 110 vertices, perimeter≈1363 km

| Notebook | Role |
|----------|------|
| `04_nasa_data_exploration.ipynb` | Load shapefiles; reproject to WGS84 and local km frame; PCA ellipse fit; shape analysis |
| `05_nasa_route_optimization.ipynb` | Contour Coverage Problem: exact solver + coverage vs. budget analysis + route visualisation |

`04` and `05` are self-contained (load directly from `NASA/`; no pickle dependency).

---

## Key Functions

### Geometry
- `obs_free(start, end, verts)` — shortest path avoiding a single convex polygon
- `multi_obs(start, end, obstacles)` — chains `obs_free` across multiple obstacles
- `transit_dist(p, q, obstacles)` — scalar obstacle-avoiding distance

### Survey routing
- `route_circ_from(start, cell, theta, L, all_obs)` — best ordering of arms +
  sensitivity points; returns `(dist, exit_pt)`
- `route_ellip_from(start, cell, n, s, dt, rev, etop, all_obs)` — boustrophedon scan;
  `rev` reverses leg order, `etop` flips entry side; returns `(dist, exit_pt)`
- `precompute_cell(cell, ...)` — enumerates all `(theta, L)` or `(n, s, rev, etop)`
  configs; returns list of `{score, survey_dist, exit_pt, params}` dicts

### Solvers
- `tour_dist_score(tour_list)` — fast distance+score from precomputed `T` and `sdist_vec`
- `solve_op_milp(N, scores, sdist, T_mat, budget)` — MILP via scipy/HiGHS with MTZ
  subtour elimination; returns `{tour, score, dist}` or `None`
- `alns_op(n_iter, seed, random_init)` — ALNS with 4 destroy/repair operators;
  adaptive weights `×1.2` on improvement, `×0.98` decay per iteration, clipped `[0.1, 10]`

---

## Benchmark (N=6 scenario)

**Cells:** C1(w=1.8), C2(w=3.0), C3(w=2.0), E1(w=3.5), E2(w=2.5), E3(w=2.0)
**Obstacles:** OBS1 (near C1), OBS2 (near C3/E1)
**Budget:** 2680 km (midpoint: min-4-cell=2400 km, min-5-cell=2960 km)

**Global optimum:** `C2 → E2 → E1 → C3`, score=12.1182, dist=2629.7 km

All four algorithms (exhaustive, MILP, ALNS greedy ×5, ALNS random ×10) recover
the same global optimum.

---

## Real-data Problem: Contour Coverage Problem (CCP)

Defined in `05_nasa_route_optimization.ipynb`. Different from the simulation OP:
instead of selecting among discrete supercells, the target is a **single continuous
closed PV contour**. The aircraft must decide where to enter/exit the contour and
which direction to fly, maximising the arc length surveyed.

```
max   arc(i → j, d)
s.t.  T[BASE, i] + arc(i → j, d) + T[j, BASE] ≤ B
      i, j ∈ {0..N-1},  d ∈ {CCW, CW}
```

**Why contour-following (not cross/boustrophedon)?** TPV scientific value is at the
boundary (tropopause fold, PV gradient) — flying the contour maximises boundary
coverage per km. A cross-leg only probes one diameter.

**Solver:** exact enumeration of N×N×2 = 24,200 triples, O(1) per triple using
precomputed cumulative arc-length array. Runtime < 1 ms.

**Key result (GOOSE BAY / PV=296 scenario):**
- Full coverage (1363 km contour) achieved at B ≈ 2800 km
- Optimal entry/exit cluster near the contour vertex closest to BASE (~arc 15–20%)

---

## Dependencies

```
numpy  matplotlib  scipy  geopandas
```

Python 3.10+. Uses `scipy.optimize.milp` (HiGHS backend; no external solver needed).
`geopandas` required only for the NASA real-data notebooks (04, 05).

---

## Potential Extensions

- Larger N: ALNS scales; exhaustive ground truth unavailable for N > 8
- Continuous parameter optimisation (replace discrete grids with Bayesian/gradient)
- Multiple TPV contours: extend CCP to multi-target orienteering over real NWP data
- Multi-aircraft cooperative routing
- Time-varying targets incorporating storm motion forecasts
