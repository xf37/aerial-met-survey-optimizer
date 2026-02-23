# Aerial Met Survey Optimizer

A progressive optimization framework for planning airborne meteorological observation routes. Given a set of meteorological targets (e.g., supercells, jet deformation zones, tropopause polar vortices) scattered across a flight region, the system finds the optimal flight path that maximizes scientific observation score within a flight distance budget.

---

## Overview

Research aircraft must fly specific sampling patterns through each supercell to collect meaningful data. The route planning problem is a variant of the **Orienteering Problem** (OP): select a subset of supercells to visit and determine the flight order, subject to a total distance budget, to maximize scientific value.

This project models two types of supercells with physically-motivated observation patterns, then solves the routing problem across three levels of increasing complexity (V1 → V2 → V3), comparing exact algorithms (MILP, dynamic programming) against the heuristic **Adaptive Large Neighborhood Search (ALNS)**.

---

## Problem Statement

**Given:**
- A set of N supercells, each with a location, type (circular/elliptical), and scientific weight
- A home base (airport)
- A total flight distance budget B

**Decide:**
- Which subset of supercells to visit
- In what order to visit them
- For each visited supercell: the scan direction θ, leg length L, number of legs m, and leg spacing s

**Objective:** Maximize the total scientific observation score collected within budget B.

---

## Supercell Models

Two types of supercell are modeled, each with a distinct observation pattern:

### Circular Supercell (Tropopause Polar Vortex)
- Exhibits radial symmetry
- Observed by flying **two perpendicular crossing legs** through the center
- Score is **invariant to scan direction** (θ) due to symmetry
- Tunable parameter: leg half-length L

### Elliptical Supercell (Jet Deformation Zone)
- Elongated along a principal axis at orientation φ
- Observed by flying **m parallel legs** in a boustrophedon (lawnmower) pattern
- Score depends strongly on scan direction θ: peaks when θ = φ (legs run along the major axis)
- Tunable parameters: scan direction θ, number of legs m, leg spacing s

**Key geometric insight:** once all observation parameters (θ, L, m, s) are fixed, all waypoints and entry/exit positions are fully determined — the distance matrix is precomputable.

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
│   └── scenario_N12_seed42.pkl           # Serialized scenario (N=12, seed=42)
└── figures/                              # Output visualizations
```

### Notebook Descriptions

| Notebook | Purpose |
|---|---|
| `00` | Generates a reproducible synthetic scenario: N=12 supercells (4 circular, 8 elliptical) in an 800×600 km region |
| `01` | Visualizes supercell patterns, derives score functions, introduces the three problem versions |
| `02` | Solves the TSP (visit-all) subproblem using Nearest Neighbor, 2-opt, Held-Karp DP, and MILP |
| `03` | Solves V1-OP (select subset, fixed patterns) using greedy, ALNS, and exact MILP |
| `04` | Solves V2-OP (also optimizes θ per supercell) with 4D transit cost tensor and extended ALNS |
| `05` | Solves V3-OP (also optimizes L/m/s per supercell) using ALNS with full configuration enumeration |

---

## Optimization Formulation

### V1 — Fixed Parameters (Standard Orienteering Problem)

```
Maximize   Σ_i  score_i · y_i

Subject to:
  Σ_j x_{0j} = 1                     (depart base)
  Σ_i x_{i0} = 1                     (return base)
  Σ_{j≠i} x_{ij} = y_i  ∀i          (flow conservation)
  Σ_{i,j} c'_{ij} · x_{ij} ≤ B      (budget constraint)
  u_i − u_j + N·x_{ij} ≤ N−1        (MTZ subtour elimination)
  x_{ij} ∈ {0,1},  y_i ∈ {0,1}
```

where `c'[i,j] = transit[i,j] + internal_distance[j]` folds the supercell's internal flight cost into the arc cost.

### V2 — Free Scan Direction

Extends V1 by introducing a discrete scan direction θ_i ∈ {0°, 15°, …, 165°} per supercell. The cost matrix becomes a 4D tensor C[i, θ_i, j, θ_j]. Both the score and transit cost depend on θ.

### V3 — Full Parameter Optimization

Further extends V2 by making leg length L (circular) and leg count m, spacing s (elliptical) optimization variables. All feasible configurations (θ, L, m, s) are enumerated per supercell and explored by ALNS with configuration-level local search.

---

## Algorithms

| Algorithm | Applies to | Optimality | Notes |
|---|---|---|---|
| Nearest Neighbor | TSP | Heuristic (~71% gap) | O(N²), fast baseline |
| 2-opt Local Search | TSP | Local optimum | O(N²/iter), practical for N≤30 |
| Held-Karp DP | TSP | **Exact** | O(2^N · N²), feasible N≤20 |
| MILP (MTZ) | TSP, V1-OP, V2-OP | **Exact** (over discrete grid) | Solved with PuLP + CBC |
| Greedy Constructive | V1-OP | Heuristic | Insert by score/cost ratio |
| **ALNS** | V1/V2/V3-OP | Near-optimal heuristic | Adaptive destroy/repair operators |

### ALNS Design

**Destroy operators:**
- `rand_1` — remove 1 random node
- `rand_2` — remove 2 random nodes
- `worst` — remove node with lowest marginal value

**Repair operators:**
- `greedy_ratio` — reinsert by score/insertion-cost ratio
- `score_first` — reinsert highest-score feasible nodes first

**Local search (V2/V3):** after each repair, sweep all θ (and L/m/s in V3) for every visited node and accept improving moves.

**Weight update:** multiplicative (+20% on improvement, −2% decay), designed for extension to contextual bandit / learning-augmented ALNS.

---

## Results

Benchmark scenario: N=12 supercells, budget = 8,144 km (55% of visit-all cost).

### TSP (visit-all, N=12)

| Algorithm | Transit Distance | Time |
|---|---|---|
| Nearest Neighbor | 2,704 km | < 1 ms |
| 2-opt | 1,582 km | 0.27 ms |
| Held-Karp DP | 1,582 km (optimal) | 117 ms |
| MILP (MTZ) | 1,582 km (optimal) | 0.64 s |

### Orienteering Problem (select subset within budget)

| Version | Supercells visited | Score | ALNS time |
|---|---|---|---|
| V1 (fixed parameters) | 11 | 18.58 | ~200 ms |
| V2 (free θ) | **13** | **20.33** | 2.2 s |
| V3 (free θ, L, m, s) | **13** | **20.34** | 2.0 s |

- V2 visits 2 extra supercells (+9.4% score) over V1 by adjusting scan directions to improve routing geometry.
- V3 over V2 yields marginal additional gain; its main benefit is enabling feasible solutions at tight budgets.
- ALNS matches exact MILP solutions in V1 while running **360× faster** (229 ms vs 81.8 s).

---

## On Optimality

The full V3 problem (select supercells + sequence + choose all parameters) is **NP-hard** — it generalizes both the Travelling Salesman Problem and the Orienteering Problem. No polynomial-time exact algorithm is known.

- **TSP (notebook 02):** Held-Karp and MILP give **provably optimal** solutions for N ≤ ~20.
- **V1-OP (notebook 03):** MILP gives the **exact optimum** over fixed parameters.
- **V2-OP (notebook 04):** MILP gives the **exact optimum** over the discretized θ grid.
- **V3-OP (notebook 05):** Only ALNS is used — **no global optimality guarantee**. Solutions are high-quality heuristics.

Even the MILP results are exact only within the discretized parameter space; the underlying continuous problem (e.g., θ ∈ [0°, 180°)) has no exact method at scale.

---

## Installation

```bash
pip install numpy matplotlib scipy pulp
```

Python 3.8+ recommended. For faster MILP solving, a commercial solver (Gurobi, CPLEX) can be used as a PuLP backend in place of the default CBC.

---

## Usage

Run notebooks in order:

```bash
jupyter notebook 00_simulation_data.ipynb          # generates data/scenario_N12_seed42.pkl
jupyter notebook 01_supercell_concepts_and_visualization.ipynb
jupyter notebook 02_tsp_all_visit.ipynb
jupyter notebook 03_orienteering_op.ipynb
jupyter notebook 04_v2_orienteering.ipynb
jupyter notebook 05_v3_orienteering.ipynb
```

To generate a different scenario, modify the parameters in `00_simulation_data.ipynb`:

```python
N = 12          # number of supercells
frac_circular = 0.33  # fraction that are circular
seed = 42       # random seed for reproducibility
```

---

## Potential Extensions

- **Learning-Augmented ALNS (LA-ALNS):** replace the multiplicative weight update with a contextual bandit or RL policy trained on historical scenarios
- **Real storm data:** replace synthetic supercells with actual NWP (numerical weather prediction) storm forecasts
- **Multi-aircraft:** extend to cooperative routing with multiple research aircraft
- **Continuous θ optimization:** replace discrete grid with gradient-based or Bayesian optimization for scan direction
- **Time-varying storms:** incorporate storm motion forecasts into the routing model
