# Aerial Met Survey Optimizer

A progressive optimization framework for planning airborne meteorological observation routes. Given a set of meteorological targets (e.g., supercells, jet deformation zones, tropopause polar vortices) scattered across a flight region, the system finds the optimal flight path that maximizes scientific observation score within a flight distance budget.

---

## Overview

Research aircraft must fly specific sampling patterns through each supercell to collect meaningful data. The route planning problem is a variant of the **Orienteering Problem** (OP): select a subset of supercells to visit and determine the flight order, subject to a total distance budget, to maximize scientific value.

This project models two types of supercells with physically-motivated observation patterns, then solves the routing problem across three levels of increasing complexity (V1 → V2 → V3), comparing exact algorithms (MILP, dynamic programming) against the heuristic **Adaptive Large Neighborhood Search (ALNS)**.

---

## Supercell Models

Two types of supercell are modeled, each with a distinct observation pattern:

### Circular Supercell (Tropopause Polar Vortex)
- Exhibits radial symmetry
- Observed by flying **two perpendicular crossing legs** through the center
- Score is **invariant to scan direction** θ due to symmetry
- Tunable parameter: leg half-length L
- Score: `score_i = w_i` (weight only, independent of θ and L)

### Elliptical Supercell (Jet Deformation Zone)
- Elongated along a principal axis at orientation φ
- Observed by flying **m parallel legs** in a boustrophedon (lawnmower) pattern
- Score depends strongly on scan direction θ: peaks when θ = φ (legs run along the major axis)
- Tunable parameters: scan direction θ, number of legs m, leg spacing s

Score function for elliptical supercells:

```
score_i(θ, m, s) = w_i · [β · WidthCoverage(θ, m, s)  +  (1−β) · DirectionAlignment(θ)]

  WidthCoverage(θ, m, s)  = min((m−1)·s,  W_⊥(θ)) / W_⊥(θ)
                            (fraction of cross-axis width covered by the legs)

  DirectionAlignment(θ)   = length of center leg / (2a)
                            (ratio of center-leg length to major-axis diameter)

  W_⊥(θ)  = full ellipse width measured perpendicular to scan direction θ
```

**Key geometric insight:** once all observation parameters (θ, L, m, s) are fixed, all waypoints and entry/exit positions are fully determined — the distance matrix is precomputable.

---

## Problem Formulation

### Notation

| Symbol | Meaning |
|---|---|
| N | number of supercells |
| 0 | base depot (airport) |
| w_i | scientific weight of supercell i |
| B | total flight distance budget |
| x_{ij} ∈ {0,1} | arc variable: 1 if flight goes directly from i to j |
| y_i ∈ {0,1} | visit indicator: 1 if supercell i is visited |
| u_i ∈ [1,N] | MTZ position variable for subtour elimination |
| c_{ij} | transit distance from exit point(s) of i to entry point(s) of j |
| d^int_i | internal flight distance of the observation pattern at supercell i |
| c'_{ij} | effective arc cost = c_{ij} + d^int_j (folds internal flight of destination into arc) |

### V1 — Standard Orienteering Problem (fixed θ, L, m, s)

All observation parameters are fixed. Decision variables are visit selection and sequencing only.

**MILP (MTZ formulation):**

```
Maximize    Σ_{i=1..N}  w_i · y_i

Subject to:
  Σ_j x_{0j} = 1                              (depart base exactly once)
  Σ_i x_{i0} = 1                              (return to base exactly once)
  Σ_{j≠i} x_{ij} = y_i   ∀i = 1..N          (flow out = visit indicator)
  Σ_{j≠i} x_{ji} = y_i   ∀i = 1..N          (flow in  = visit indicator)
  Σ_{i,j} c'_{ij} · x_{ij} ≤ B              (budget constraint)
  u_i − u_j + N·x_{ij} ≤ N−1   ∀i≠j≥1      (MTZ subtour elimination)
  u_i ≥ 1                        ∀i≥1
  x_{ij} ∈ {0,1},  y_i ∈ {0,1}
```

This is a variant of the **Selective Travelling Salesman Problem** (STSP / Orienteering Problem). It is **NP-hard** in general, but solvable to optimality by MILP for small N (≤ ~20 with CBC solver).

### V2 — Orienteering with Free Scan Direction

Extends V1 by making **θ_k ∈ Θ = {0°, 15°, 30°, …, 165°}** (12 discrete values) a per-supercell decision variable alongside visit selection and order.

**Key consequence:** the transit cost between two supercells now depends on both their scan directions, so the 2D cost matrix becomes a 4D tensor:

```
C^(4)[i, θ_i, j, θ_j]  =  min_{e ∈ exits(i,θ_i),  n ∈ entries(j,θ_j)}  ‖e − n‖
```

**V2 trade-off for elliptical cells:**
- θ = φ maximizes score, but exit/entry geometry may be poor → high transit cost
- θ ≠ φ slightly reduces score, but better geometry → lower transit → budget saved → possibly visit one more cell

**V2 MILP (extended graph):** create a virtual node (i, k) for every (supercell i, theta index k) pair. Add a "one-config-per-cell" constraint: each physical supercell may be visited in at most one theta configuration. The MILP is exact over the discretized θ grid.

### V3 — Full Observation Parameter Optimization

Extends V2 by also making **pattern geometry parameters** decision variables:

| Cell type | Additional V3 variables | Discrete levels |
|---|---|---|
| Circular | Leg half-length L | L = r + {15, 35, 55, 75} km (4 values) |
| Elliptical | Leg count m, spacing s | m ∈ {2,3,4,5}, s ∈ {10,15,20} km (12 combinations) |

V3 enumerates all feasible **(θ, L)** or **(θ, m, s)** configurations per supercell. No MILP formulation is used at this scale — only ALNS with configuration-level local search.

**V3 trade-off:** sparser patterns (fewer legs, smaller L) sacrifice a small amount of score per cell but free up flight budget to visit an additional high-value cell, yielding a net positive gain.

---

## Algorithms

### Summary Table

| Algorithm | Problem | Optimality | Complexity | Notes |
|---|---|---|---|---|
| Nearest Neighbor | TSP | Heuristic | O(N²) | ~71% above optimal |
| 2-opt | TSP | **Local optimum** | O(N²/iter) | Practical for N ≤ 30 |
| Held-Karp DP | TSP | **Global optimum** | O(2^N · N²) | Exact, feasible N ≤ 20 |
| MILP MTZ (CBC) | TSP, V1-OP, V2-OP | **Global optimum** | Exponential worst-case | Practical N ≤ ~20 |
| Greedy ratio | V1/V2/V3-OP | Heuristic | O(N²) | No optimality guarantee |
| **ALNS** | V1/V2/V3-OP | **Local optimum** (heuristic) | O(iter × N) | Near-optimal in practice |

### Local vs Global Optimality — Analysis by Problem Version

#### TSP (notebook 02)
- **Held-Karp DP** and **MILP**: provably find the **global optimum** for the given instance. Held-Karp is feasible for N ≤ 20; MILP (branch-and-bound) practical for N ≤ ~15.
- **2-opt**: converges to a **local optimum** with respect to 2-edge swaps. In practice finds the optimal tour for the N=20 benchmark, but this is not guaranteed.
- **Nearest Neighbor**: pure greedy, no optimality property.

#### V1-OP (notebook 03)
- **MILP (MTZ + CBC)**: finds the **global optimum** of the V1 integer program. Verified on benchmark: score 20.150, 81.8 s.
- **ALNS**: a metaheuristic with **no optimality guarantee**. Destroy operators allow escape from local optima, but convergence to global optimum is not guaranteed. On the benchmark, ALNS matched the MILP optimum in 229 ms (360× faster).
- **Greedy**: inserts by score/cost ratio, no backtracking. Pure heuristic, can miss globally better solutions.

#### V2-OP (notebook 04)
- **V2 MILP**: globally optimal **over the coarser discrete θ grid** (T = 6 values). Not directly comparable to V2 ALNS which uses a finer grid (T = 12). On the benchmark, ALNS outperforms MILP precisely because it searches over twice as many θ values.
- **V2 ALNS**: heuristic over the T = 12 grid. No global optimality guarantee, but outperforms MILP due to finer θ discretization and richer local search (theta sweep after every repair).

#### V3-OP (notebook 05)
- **No exact algorithm is applied.** The joint V3 problem (select supercells + sequence + choose θ, L, m, s) is NP-hard and the search space per cell is ~48× larger than V2.
- **V3 ALNS** is the sole method: local optimum / heuristic only. The configuration-level local search (`param_ls`) improves solution quality but gives no global guarantee.
- Even the MILP results for V1/V2 are exact only within the **discretized** parameter space; the underlying continuous problem (θ ∈ [0°, 180°)) has no known exact method at scale.

### ALNS Design

The same ALNS framework is reused across V1, V2, V3 with progressive extensions:

**Destroy operators** (select which nodes to remove):
- `rand_1` — remove 1 random node from the current tour
- `rand_2` — remove 2 random nodes
- `worst` — remove the node with the lowest marginal value (score per unit of saved distance)

**Repair operators** (re-insert removed nodes):
- `greedy_ratio` — at each step, insert the node with the highest score / insertion-cost ratio
- `score_first` — insert the highest-score feasible node first

**V2 extension:**
- `best_theta_for_insert` — when inserting a node, scan all T θ values and pick the one with best score/cost ratio
- `theta_local_search` — after each repair, sweep all θ values for every visited node and accept any improving move

**V3 extension:**
- `best_cfg_for_insert` — when inserting a node, enumerate all (θ, L) or (θ, m, s) configurations
- `param_ls` — after each repair, sweep all configurations for every visited node and accept improving moves

**Operator weight update:** multiplicative (+20% on improvement, ×0.98 decay per iteration). Designed for future replacement with a contextual bandit / RL policy (Learning-Augmented ALNS).

**Acceptance criterion:** always keep the global best; the working solution follows a random walk (always accept the result of destroy+repair regardless of quality), which helps diversity.

---

## Results

All results use a scenario of **N = 20 supercells** (8 circular + 12 elliptical), budget B = 55% of the nearest-neighbor full-tour cost.

### TSP baseline (notebook 02, visit all N supercells)

| Algorithm | Optimality | Transit Distance | Time |
|---|---|---|---|
| Nearest Neighbor | Heuristic | 2,704 km | < 1 ms |
| 2-opt | Local optimum | 1,582 km | 0.27 ms |
| Held-Karp DP | **Global optimum** | **1,582 km** | 117 ms |
| MILP (MTZ + CBC) | **Global optimum** | **1,582 km** | 0.64 s |

2-opt and both exact methods converge to the same tour (1,582 km) on this instance.

### V1-OP algorithm comparison (notebook 03, fixed parameters)

| Algorithm | Optimality | Supercells visited | Score | Distance | Time |
|---|---|---|---|---|---|
| Greedy | Heuristic | 11 | 19.980 | 7,972 km | 1.3 ms |
| ALNS (800 iter) | Local optimum | 11 | **20.150** | 8,124 km | 229 ms |
| MILP (CBC) | **Global optimum** | 11 | **20.150** | 8,124 km | 81.8 s |

ALNS matches the MILP global optimum at **360× lower runtime**. ALNS optimality gap = 0.00%.

### V1 → V2 → V3 progression (notebook 05, same scenario)

| Version | Decision variables | Optimality | Supercells visited | Score | % of max | Distance | Time |
|---|---|---|---|---|---|---|---|
| V1 ALNS | visit + order | Local opt. (heuristic) | 13 | 10.650 | 55.0% | 1,640 km | 0.31 s |
| V2 ALNS | + scan direction θ | Local opt. (heuristic) | 14 | 11.256 | 58.1% | 1,518 km | 0.35 s |
| V3 ALNS | + leg params L/m/s | Local opt. (heuristic) | 14 | **11.473** | **59.2%** | 1,520 km | 2.35 s |

Key observations:
- **V2 over V1**: +0.606 score (+5.7%), visits 1 extra supercell. Gained by adjusting θ to improve routing geometry.
- **V3 over V2**: +0.217 score (+1.9%), same cell count. Gained by freeing L/m/s to reduce internal flight distances.
- **V3 over V1**: +0.823 score total (+7.7% improvement).
- V2 and V3 are **heuristics only** — they may not be globally optimal, but demonstrate clear measurable gains over the more constrained formulations.

### Budget sensitivity

Across all tested budget levels (25%–100% of full-tour cost):
- V2 consistently outperforms V1 (largest gap at tight budgets where routing geometry matters most)
- V3 adds further improvement on top of V2 at all budget levels
- At very high budgets (≥ 85%), all versions converge as nearly all supercells can be visited

---

## Project Structure

```
aerial-met-survey-optimizer/
├── 00_simulation_data.ipynb              # Scenario generation
├── 01_supercell_concepts_and_visualization.ipynb  # Concepts, geometry, score functions
├── 02_tsp_all_visit.ipynb                # TSP baseline: visit all supercells
├── 03_orienteering_op.ipynb              # V1-OP: fixed parameters, select subset
├── 04_v2_orienteering.ipynb              # V2-OP: optimize scan direction θ
├── 05_v3_orienteering.ipynb              # V3-OP: optimize all pattern parameters
├── data/
│   └── scenario_N12_seed42.pkl           # Serialized scenario
└── figures/                              # Output visualizations (~18 PNGs)
```

| Notebook | Purpose |
|---|---|
| `00` | Generates a reproducible synthetic scenario: N supercells in an 800×600 km region |
| `01` | Visualizes supercell patterns, derives score functions, introduces the three problem versions |
| `02` | Solves the TSP (visit-all) subproblem: Nearest Neighbor, 2-opt, Held-Karp DP, MILP |
| `03` | Solves **V1-OP** (fixed parameters): Greedy, ALNS, exact MILP + budget sensitivity |
| `04` | Solves **V2-OP** (free θ): 4D transit tensor, V2 ALNS with θ local search, V2 MILP |
| `05` | Solves **V3-OP** (free θ + L/m/s): ALNS with full config enumeration, V1/V2/V3 comparison |

---

## Installation

```bash
pip install numpy matplotlib scipy pulp
```

Python 3.8+ recommended. For faster MILP solving, a commercial solver (Gurobi, CPLEX) can replace the default CBC backend in PuLP.

---

## Usage

Run notebooks in order:

```bash
jupyter notebook 00_simulation_data.ipynb
jupyter notebook 01_supercell_concepts_and_visualization.ipynb
jupyter notebook 02_tsp_all_visit.ipynb
jupyter notebook 03_orienteering_op.ipynb
jupyter notebook 04_v2_orienteering.ipynb
jupyter notebook 05_v3_orienteering.ipynb
```

To generate a different scenario, modify parameters in `00_simulation_data.ipynb`:

```python
N = 20              # number of supercells
frac_circular = 0.4 # fraction that are circular
seed = 42           # random seed for reproducibility
```

---

## Potential Extensions

- **Learning-Augmented ALNS (LA-ALNS):** replace the multiplicative weight update with a contextual bandit or RL policy trained on historical scenarios
- **Real data:** replace synthetic supercells with actual NWP (numerical weather prediction) forecast fields
- **Multi-aircraft:** extend to cooperative routing with multiple research aircraft sharing a budget
- **Continuous θ optimization:** replace discrete grid with gradient-based or Bayesian optimization for scan direction
- **Time-varying targets:** incorporate target motion forecasts (storm drift) into the routing model
