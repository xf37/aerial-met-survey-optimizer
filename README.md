# Aerial Met Survey Optimizer

A route-planning framework for airborne meteorological observation missions. Given a
set of supercell targets scattered across a flight region, the system finds the
optimal flight path that maximises scientific observation score within a flight
distance budget.

---

## Overview

Research aircraft must fly specific sampling patterns through each supercell.
The route planning problem is a variant of the **Orienteering Problem (OP)**:
select a subset of supercells and visit them in the best order, subject to a
total distance budget, to maximise scientific value.

This project models two types of supercell with physically-motivated observation
patterns, solves the routing problem exactly (exhaustive enumeration, MILP) and
heuristically (ALNS), and provides verified ground-truth comparisons.

---

## Supercell Models

### Circular Supercell (Tropopause Polar Vortex)

- Observed by flying **two perpendicular crossing legs** through the cell centre
- Score is **fixed** = `w_k` (weight only; independent of scan direction θ or leg length L)
- Tunable: leg half-length L, scan direction θ
- May include **sensitivity points** visited after the cross survey
- Survey waypoints: `entry_tip → centre → far_tip → arm2_tip → centre → arm2_far_tip`

### Elliptical Supercell (Jet Deformation Zone)

- Observed by flying **n parallel legs** in a boustrophedon (lawnmower) pattern
- Score depends on spatial coverage:

```
f_k(n, s) = w_k × [CovArea(n, s) + 0.5 × OvlpArea(n, s)] / (π · a · b)
```

where `CovArea` is the ellipse area covered by at least one leg (strip width = SW = 12 km),
`OvlpArea` is the area covered by two or more legs, and `π a b` is the full ellipse area.

- Tunable: number of legs n, leg spacing s, scan direction θ

### Key Geometric Insight

Once all observation parameters (θ, L, n, s) are fixed, all waypoints and entry/exit
positions are fully determined and precomputable. Survey starts **and ends at the cell
centre**, decoupling internal flight geometry from inter-cell routing.

---

## Problem Formulation

### Orienteering Problem

$$\max_{S,\,\sigma} \sum_{k \in S} f_k \qquad
  \text{s.t.}\quad
  \sum_{k \in S} d_k^{\text{survey}} + \sum_{\text{transit}} T_{ij} \leq B$$

| Symbol | Meaning |
|--------|---------|
| S | selected subset of supercells to visit |
| σ | visit order |
| f_k | score of supercell k (fixed for circular; coverage-dependent for elliptical) |
| d_k^survey | internal survey distance of cell k (start/end at cell centre) |
| T_{ij} | obstacle-avoiding transit distance between cell centres i and j |
| B | total flight distance budget |

The budget B is calibrated as the midpoint between the minimum 4-cell and 5-cell tour
costs, forcing a non-trivial selection trade-off among all 6 cells.

### MILP Formulation (MTZ subtour elimination)

Variables: binary arc indicators x_{ij} ∈ {0,1}, binary visit indicators y_k ∈ {0,1},
continuous MTZ position variables u_k ∈ [1, N].

```
max  Σ_k  f_k · y_k

s.t.  Σ_j x_{0j} = 1                        (depart BASE once)
      Σ_i x_{i0} = 1                        (return BASE once)
      Σ_j x_{kj} = y_k  ∀k                  (flow out = visit)
      Σ_i x_{ik} = y_k  ∀k                  (flow in  = visit)
      Σ_{ij} T_{ij}·x_{ij} + Σ_k d_k·y_k ≤ B   (budget)
      u_i − u_j + N·x_{ij} ≤ N−1  ∀i≠j≥1   (MTZ subtour elimination)

      x_{ij}, y_k ∈ {0,1},   u_k ∈ [1,N] continuous
```

Solved to **certified global optimality** via scipy / HiGHS backend.

---

## Algorithms

### TSP Baseline (notebook 02)

| Algorithm | Optimality | Complexity |
|-----------|-----------|-----------|
| Nearest Neighbor | Heuristic | O(N²) |
| 2-opt local search | **2-optimal** local | O(N²) per pass |
| Held-Karp DP | **Global optimum** | O(2^N · N²) |
| MILP (MTZ + HiGHS) | **Global optimum** | Branch-and-bound |

### Orienteering Problem (notebook 03)

| Algorithm | Optimality | Notes |
|-----------|-----------|-------|
| Exhaustive enumeration | **Global optimum** ✓ | Feasible only for N ≤ ~8 |
| MILP (MTZ + HiGHS) | **Global optimum** ✓ | 42 binary + 6 continuous vars |
| ALNS greedy init | Local optimum | Greedy construction + ALNS |
| ALNS random init | Local optimum | Random start + ALNS |

### ALNS (Adaptive Large Neighbourhood Search)

State: ordered list of visited cells. Iteratively destroys part of the current solution
and repairs it; operator selection probabilities adapt to historical performance.

**Destroy/repair operators:**

| ID | Operator | Description |
|----|----------|-------------|
| D0 | rm1 + reinsert | Remove 1 random cell; greedily reinsert all unvisited by score/cost ratio |
| D1 | swap_unvisited | Swap one visited cell with one random unvisited cell |
| D2 | 2-opt | Reverse a random subsequence of the visit order |
| D3 | rm2 + reinsert | Remove 2 random cells; greedily reinsert |

**Weight update:** improvement → weight × 1.2; every iteration → all weights × 0.98
(exponential decay toward uniform); clipped to [0.1, 10].

---

## Benchmark Results

**Scenario:** 6 supercells (3 circular + 3 elliptical), 2 obstacles, strict no-overlap.
Cell weights: C1=1.8, C2=3.0, C3=2.0, E1=3.5, E2=2.5, E3=2.0.

### TSP (visit all 6 cells)

All exact algorithms agree on the same optimal transit cost. Results are regenerated
on each run of `02_tsp_all_visit.ipynb`.

### Orienteering (budget-constrained subset selection)

Budget forces selection of exactly 4 out of 6 cells (midpoint between min 4-cell
and min 5-cell tour costs = 2779 km).

| Algorithm | Tour | Score | Dist (km) | Time | Optimal |
|-----------|------|-------|-----------|------|---------|
| Exhaustive | C2 → E2 → E1 → C3 | 12.1182 | 2682.4 | < 1 ms | ✓ |
| MILP (HiGHS) | C2 → E2 → E1 → C3 | 12.1182 | 2682.4 | ~16 ms | ✓ |
| ALNS greedy (5/5 seeds) | C2 → E2 → E1 → C3 | 12.1182 | 2682.4 | ~60 ms | ✓ |
| ALNS random (10/10 seeds) | C2 → E2 → E1 → C3 | 12.1182 | 2682.4 | ~65 ms | ✓ |

All algorithms recover the global optimum. The exhaustive enumeration proves global
optimality; MILP independently confirms it with a certified zero optimality gap.

---

## Project Structure

```
aerial-met-survey-optimizer/
├── 00_simulation_data.ipynb        # Define scenario → save data/scenario_N6_seed42.pkl
├── 01_survey_patterns.ipynb        # Survey pattern concepts + score function analysis
├── 02_tsp_all_visit.ipynb          # TSP baseline: NN / 2-opt / Held-Karp DP / MILP
├── 03_orienteering.ipynb           # OP: precompute → exhaustive → MILP → ALNS
├── unit_test_circular.ipynb        # Unit tests: circular supercell (3 scenarios)
├── unit_test_elliptical.ipynb      # Unit tests: elliptical supercell (2 scenarios)
├── unit_test_combined.ipynb        # Unit test: 1 circular + 1 elliptical, global optimality proof
├── data/
│   └── scenario_N6_seed42.pkl      # Serialized 6-cell scenario
└── figures/
    ├── scenario_overview.png        # Cell layout with obstacles
    ├── circular_pattern_demo.png    # Cross survey at 3 scan directions
    ├── elliptical_pattern_demo.png  # Boustrophedon scan at 3 leg counts
    ├── score_analysis.png           # Score vs n-legs and score vs survey-dist
    ├── tsp_comparison.png           # TSP optimal route with survey patterns
    ├── orienteering_route.png       # Full 6-cell route visualisation (all cells)
    ├── orienteering_comparison.png  # ALNS convergence + operator stats + score distribution
    ├── unit_test_s1.png             # Circular unit test: no obstacle
    ├── unit_test_s2.png             # Circular unit test: obstacle between BASE and cell
    ├── unit_test_s3.png             # Circular unit test: obstacle + sensitivity points
    ├── unit_test_elliptical_s1.png  # Elliptical unit test: no obstacle
    ├── unit_test_elliptical_s2.png  # Elliptical unit test: obstacle between BASE and cell
    └── unit_test_combined.png       # Combined unit test: circular + elliptical
```

### Main notebooks

| Notebook | Purpose | Run time |
|----------|---------|----------|
| `00` | Generate & save benchmark scenario (hardcoded, reproducible) | < 5 s |
| `01` | Visualise survey patterns; explain score functions | < 30 s |
| `02` | TSP on all 6 cells; compare 4 algorithms | < 10 s |
| `03` | Full orienteering problem; exhaustive ground truth + MILP + ALNS | ~5 min |

### Unit test notebooks

Each unit test uses a minimal 1- or 2-cell scenario where the global optimum can be
verified by exhaustive enumeration, confirming correctness of the route geometry and
score computation before scaling up to the full 6-cell benchmark.

| Notebook | Scenarios | What is tested |
|----------|-----------|----------------|
| `unit_test_circular` | S1: no obstacle<br>S2: obstacle between BASE and cell<br>S3: obstacle + sensitivity points | Cross survey waypoints, obstacle avoidance, sensitivity point routing |
| `unit_test_elliptical` | S1: no obstacle<br>S2: large obstacle between BASE and ellipse | Boustrophedon scan, coverage score, leg ordering |
| `unit_test_combined` | 1 circular + 1 elliptical cell | Joint routing over mixed cell types; MILP global optimality proof |

---

## Installation

```bash
pip install numpy matplotlib scipy
```

Python 3.10+ recommended. Uses `scipy.optimize.milp` (HiGHS backend; no external
solver installation required).

---

## Usage

Run main notebooks in order:

```bash
jupyter notebook 00_simulation_data.ipynb       # generates scenario pickle
jupyter notebook 01_survey_patterns.ipynb        # pattern visualisations
jupyter notebook 02_tsp_all_visit.ipynb          # TSP baseline
jupyter notebook 03_orienteering.ipynb           # full OP solution
```

Unit test notebooks are self-contained (no pickle dependency) and can be run independently:

```bash
jupyter notebook unit_test_circular.ipynb        # 3 single-cell circular scenarios
jupyter notebook unit_test_elliptical.ipynb      # 2 single-cell elliptical scenarios
jupyter notebook unit_test_combined.ipynb        # 1 circular + 1 elliptical, optimality proof
```

All main notebooks load the scenario from `data/scenario_N6_seed42.pkl` (written by `00`).
The scenario is hardcoded (not randomly generated) for full reproducibility.

---

## Potential Extensions

- **Larger N:** ALNS scales well; exhaustive ground truth becomes unavailable for N > 8
- **Continuous parameter optimisation:** replace discrete (θ, L, n, s) grids with
  gradient-based or Bayesian optimisation
- **Real NWP data:** replace synthetic supercells with actual forecast fields
- **Multi-aircraft:** cooperative routing with multiple research aircraft
- **Time-varying targets:** incorporate storm motion forecasts into the routing model
