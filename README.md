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
| Nearest Neighbor | TSP | Heuristic | O(N²) | ~71% above optimal on benchmark |
| 2-opt | TSP | **Local optimum** | O(N²/iter) | Practical for N ≤ 30 |
| Held-Karp DP | TSP | **Global optimum** | O(2^N · N²) | Exact, feasible N ≤ 20 |
| MILP MTZ (CBC) | TSP, V1-OP, V2-OP | **Global optimum** | Exponential worst-case | Practical N ≤ ~20 |
| Greedy ratio | V1/V2/V3-OP | Heuristic | O(N²) | No optimality guarantee |
| **ALNS** | V1/V2/V3-OP | **Local optimum** (heuristic) | O(iter × N) | Near-optimal in practice |

### Algorithm Descriptions

#### Nearest Neighbor (NN)
Greedy tour construction for TSP. Starting from the base, always fly to the nearest unvisited supercell; return to base at the end. Each step is O(N), total O(N²). Fast and simple, but locally greedy decisions lead to globally poor routes — typically 20–70% above optimal. Used here as a baseline and to calibrate the budget B (B = 55% of NN full-tour cost).

#### 2-opt Local Search
Iterative improvement heuristic for TSP. At each pass, try all O(N²) pairs of edges; if reversing the segment between them shortens the total tour, accept the swap and restart. Repeat until no improving 2-swap exists. Converges to a **2-optimal** local solution: no single pair of edges can be swapped to improve the tour. In practice very effective — finds the optimal tour on this benchmark — but no global guarantee.

#### Held-Karp Dynamic Programming
Exact TSP algorithm based on bitmask DP over subsets of nodes.

- **State:** `dp[S][v]` = minimum cost to start at base, visit exactly the nodes in set S, and end at node v
- **Recurrence:** `dp[S][v] = min over u in S\{v} of  dp[S\{v}][u] + c[u][v]`
- **Base case:** `dp[{v}][v] = c[0][v]`
- **Answer:** `min over v of  dp[all nodes][v] + c[v][0]`
- **Time:** O(2^N · N²), **Space:** O(2^N · N)

Provably finds the **global optimum**. Practical for N ≤ ~20 (at N=20: ~117 ms; at N=25: ~30 s; at N=30: ~hours).

#### MILP with MTZ Subtour Elimination
Formulates the routing problem as a **Mixed-Integer Linear Program** solved by branch-and-bound (PuLP + CBC open-source solver).

The key challenge in routing MILPs is preventing **subtours** — disconnected cycles that don't include the base. The **Miller-Tucker-Zemlin (MTZ)** formulation eliminates subtours compactly using a position variable u_i ∈ [1,N] per node with the single-constraint family:
```
u_i − u_j + N · x_{ij} ≤ N − 1    ∀ i ≠ j ≥ 1
```
This forces a consistent ordering of visited nodes, making disconnected subtours infeasible. The MILP is solved to **certified global optimality** (zero optimality gap) via branch-and-bound. Worst-case exponential, but practical for N ≤ ~20 with a good solver.

**Why can V2 use MILP if θ is geometrically a continuous variable?**
In this formulation, θ is **explicitly discretized** to a finite grid Θ = {0°, 15°, …, 165°} (T values) — this is a design choice, not a physical constraint. Each (supercell i, θ_k) pair is treated as a distinct virtual node in an extended graph. The V2 MILP then routes over these virtual nodes with standard binary arc variables and linear constraints — it is still a pure integer linear program.

If θ were kept continuous, the transit cost c(exit(i, θ_i), entry(j, θ_j)) would be a **nonlinear** (trigonometric) function of θ_i and θ_j, turning the problem into a Mixed-Integer Nonlinear Program (MINLP) — far harder to solve globally. Discretization trades a coarser θ resolution for the ability to use efficient MILP solvers. ALNS uses T=12 (15° steps); MILP uses T=6 (30° steps) to keep the extended graph smaller.

#### Greedy Ratio (for Orienteering Problem)
Constructive heuristic for the OP. Maintains a current partial tour. At each step:
1. For every unvisited supercell k, find the cheapest insertion position in the current tour (O(|tour|) per node)
2. Compute the **ratio** score_k / Δ_k, where Δ_k is the additional distance from inserting k at its best position
3. Insert the k with the highest ratio, if doing so keeps total distance ≤ B
4. Repeat until no insertion is budget-feasible

O(N²) total. Fast, no backtracking. Can be suboptimal because a greedy insertion at step t may block a higher-value combination at step t+1.

#### ALNS (Adaptive Large Neighborhood Search)
Metaheuristic for the OP, adapted for V1/V2/V3. Iteratively destroys part of the current solution and repairs it, using a **portfolio of operators** whose selection probabilities adapt based on historical performance.

**Core loop (800 iterations for V1, 600 for V2/V3):**
```
1. Destroy:   select operator D_i by weighted random draw; remove nodes from tour
2. Repair:    select operator R_j by weighted random draw; reinsert removed nodes
3. Local search (V2/V3): sweep all θ / (θ,L,m,s) configs for every visited node
4. Update:    if new solution improves global best → weight[D_i] *= 1.20,  weight[R_j] *= 1.20
              every iteration → all weights *= 0.98  (decay toward uniform)
5. Accept:    always update working solution (random walk); keep global best separately
```

**Destroy operators:**
- `rand_1` — remove 1 random node; explores broadly
- `rand_2` — remove 2 random nodes; larger perturbation
- `worst` — remove the node with the lowest score / marginal-savings ratio; targeted improvement

**Repair operators:**
- `greedy_ratio` — reinsert by score / insertion-cost ratio (same logic as greedy above)
- `score_first` — greedily insert the highest-score node that fits within budget

**V2 additions:**
- `best_theta_for_insert` — at insertion time, scan all T θ values; pick (position, θ) jointly maximising score/cost
- `theta_local_search` — post-repair: for each node in tour, try all T θ values; accept any score-improving θ swap that keeps distance ≤ B

**V3 additions:**
- `best_cfg_for_insert` — enumerate all (θ, L) or (θ, m, s) configs at insertion time
- `param_ls` — post-repair: for each node, try all configs; accept improving ones

ALNS can escape local optima via the destroy step (unlike pure local search), but **convergence to the global optimum is not guaranteed**. The random-walk acceptance criterion maintains solution diversity; the adaptive weights steer the search toward operators that have been productive.

### Local vs Global Optimality — Summary by Problem Version

| Problem | Algorithm | Optimality guarantee |
|---|---|---|
| TSP | Nearest Neighbor | None (heuristic) |
| TSP | 2-opt | **2-optimal local optimum** |
| TSP | Held-Karp DP | **Global optimum** ✓ |
| TSP / V1-OP | MILP (MTZ + CBC) | **Global optimum** ✓ |
| V1-OP | Greedy ratio | None (heuristic) |
| V1-OP | ALNS | Local optimum (no global guarantee) |
| V2-OP | MILP (extended graph) | **Global optimum over discretized θ grid** ✓ |
| V2-OP | ALNS | Local optimum (no global guarantee) |
| V3-OP | ALNS only | Local optimum (no global guarantee) |

**Key caveats:**
- All MILP results are optimal only within the **discretized** parameter space (finite θ grid, fixed L/m/s). The underlying continuous problem has no known polynomial exact method.
- V2 MILP uses T=6 while V2 ALNS uses T=12 — they solve slightly different discretized problems, which is why ALNS can outperform MILP here.
- V3 has no exact method: the joint problem (selection + sequence + all parameters) is NP-hard with ~144 configs/cell.

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

### V1 → V2 → V3 progression (notebook 05, N = 20, budget = 8,262 km)

Max possible score (all 20 cells): **32.260**

| Version | Decision variables | Optimality | Supercells visited | Score | % of max | Distance | Time |
|---|---|---|---|---|---|---|---|
| V1 ALNS | visit + order | Local opt. (heuristic) | 11 | 18.579 | 57.6% | 7,709 km | 28 s |
| V2 ALNS | + scan direction θ | Local opt. (heuristic) | 13 | 20.007 | 62.0% | 8,227 km | 500 s |
| V3 ALNS | + leg params L/m/s | Local opt. (heuristic) | **17** | **26.882** | **83.3%** | 8,257 km | 2,033 s |

Key observations:
- **V2 over V1**: +1.428 score (+7.7%), 2 extra supercells — θ freedom improves routing geometry
- **V3 over V2**: +6.875 score (+34.4%), 4 extra supercells — freeing L/m/s dramatically reduces internal flight distances, unlocking visits to previously unreachable cells
- **V3 over V1**: +8.303 score total (+44.7%), 6 extra supercells — V3 visits 17/20 cells vs V1's 11/20 within the same budget
- All three versions are **heuristics** — results represent high-quality local optima, not provably global optima

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
