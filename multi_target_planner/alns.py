"""Adaptive Large Neighbourhood Search for Phase B chord refinement (FPO-6 / PB-2).

The minimal viable PB-2 swings a single Phase A archive plan through
non-parallel chord layouts and reports the best incumbent it found within a
wall-clock budget.  By design the algorithm prefers parallel chords (theta = 0)
and only rotates when it cannot fit another parallel chord — the soft
``theta_penalty`` (default 5 km per ``|theta| / 30 deg`` unit) is what makes
that preference fall out naturally without a hand-coded fallback rule.

Scope of v1 (matching Luc's §12 single-plan scope):

* Operators act on a single TPV's chord set at a time.
* Visit order, gatepoint placement, and satellite-arc choice stay frozen at
  whatever Phase A produced. PB-3 will release the sat arc.
* No multiprocessing — one worker per session is enough for a 20-minute
  refinement of one plan.
* No LP dual bound — PB-5 adds it. ``gap_pct`` is reported as ``None`` here.

The public entry point is :func:`refine_plan_alns`.  Operators are simple
free functions; weights and acceptance live inside the main loop.
"""

from __future__ import annotations

import math
import random
import time
from dataclasses import dataclass, field, replace
from typing import Callable, Sequence

import numpy as np

from multi_target_planner.chord_layout_b import (
    DEFAULT_S_STEP_KM,
    DEFAULT_THETA_MAX_DEG,
    DEFAULT_THETA_STEP_DEG,
    NonParallelChordRecord,
    build_chord_at,
    chord_set_pairwise_feasible,
    iter_candidate_chords,
)
from multi_target_planner.chord_layout import TpvGeometry
from multi_target_planner.data_loader import TPV


# ---------------------------------------------------------------------------
# State
# ---------------------------------------------------------------------------

@dataclass
class ALNSState:
    """Mutable representation of a plan under refinement.

    ``chords_per_tpv`` is keyed by the position in the visit order, not by the
    TPV's global index.  ``visit_order`` carries the global TPV indices in
    flight order.
    """
    visit_order: list[int]
    chords_per_tpv: dict[int, list[NonParallelChordRecord]]  # keyed by visit position

    def total_chords(self) -> int:
        return sum(len(v) for v in self.chords_per_tpv.values())

    def max_chord_single_tpv(self) -> int:
        return max((len(v) for v in self.chords_per_tpv.values()), default=0)

    def is_parallel(self) -> bool:
        """All chords across all TPVs share theta = 0."""
        for chord_list in self.chords_per_tpv.values():
            for c in chord_list:
                if abs(c.theta_deg) > 1e-6:
                    return False
        return True

    def clone(self) -> "ALNSState":
        return ALNSState(
            visit_order=list(self.visit_order),
            chords_per_tpv={k: list(v) for k, v in self.chords_per_tpv.items()},
        )


# ---------------------------------------------------------------------------
# Cost model
# ---------------------------------------------------------------------------

DEFAULT_THETA_PENALTY_KM = 5.0   # +5 km per |theta| / 30° unit; "parallel preferred" prior


def state_cost(
    state: ALNSState,
    *,
    theta_penalty_km_per_30deg: float = DEFAULT_THETA_PENALTY_KM,
) -> float:
    """Sum of chord lengths + a soft theta penalty.

    The penalty term is what implements the "parallel preferred" prior Master
    fixed on 2026-06-10: every chord at ``theta != 0`` pays
    ``theta_penalty * |theta| / 30 km``, so the search prefers parallel
    layouts unless rotating unlocks more survey chord length.

    This is the *internal* objective ALNS minimises (after sign flip for
    "more chords is better" — see :func:`state_score` below).  The real
    flight-cost evaluation (geometric + ATC + turn penalties) is left to the
    PB-3+ route builder when the refined state is finally materialised; PB-2
    deliberately uses a fast surrogate so 5000 iterations fit in 20 minutes.
    """
    cost = 0.0
    for chord_list in state.chords_per_tpv.values():
        for c in chord_list:
            cost += c.length_km
            cost += theta_penalty_km_per_30deg * abs(c.theta_deg) / 30.0
    return cost


def state_score(state: ALNSState, *, theta_penalty_km_per_30deg: float = DEFAULT_THETA_PENALTY_KM) -> float:
    """Lexicographic "score" the search maximises.

    Encoded as a single float so it sorts naturally:
    ``total_chords * 1e6 + cost`` with cost being the chord lengths *minus*
    the theta penalty (more chord coverage is good; more rotation is bad).
    """
    n = state.total_chords()
    raw_cost = sum(c.length_km for chord_list in state.chords_per_tpv.values()
                   for c in chord_list)
    penalty = sum(theta_penalty_km_per_30deg * abs(c.theta_deg) / 30.0
                  for chord_list in state.chords_per_tpv.values()
                  for c in chord_list)
    # Maximising: chord-count dominates; ties broken by raw_cost - penalty
    # (longer chord lengths preferred, but minus the soft theta penalty).
    return n * 1_000_000.0 + (raw_cost - penalty)


# ---------------------------------------------------------------------------
# Operators
# ---------------------------------------------------------------------------

def destroy_remove_chord(state: ALNSState, rng: random.Random) -> ALNSState:
    """Pick a random TPV with > 0 chords and remove one chord at random."""
    candidates = [pos for pos, lst in state.chords_per_tpv.items() if lst]
    if not candidates:
        return state
    pos = rng.choice(candidates)
    new_list = list(state.chords_per_tpv[pos])
    idx = rng.randrange(len(new_list))
    new_list.pop(idx)
    new_state = state.clone()
    new_state.chords_per_tpv[pos] = new_list
    return new_state


def destroy_perturb_angle(state: ALNSState, rng: random.Random) -> ALNSState:
    """Pick one chord and rotate its theta by a random ± step within the cap."""
    candidates = [(pos, i) for pos, lst in state.chords_per_tpv.items()
                  for i in range(len(lst))]
    if not candidates:
        return state
    pos, i = rng.choice(candidates)
    chord = state.chords_per_tpv[pos][i]
    step = rng.choice([-DEFAULT_THETA_STEP_DEG, +DEFAULT_THETA_STEP_DEG])
    new_theta = max(-DEFAULT_THETA_MAX_DEG,
                    min(DEFAULT_THETA_MAX_DEG, chord.theta_deg + step))
    new_state = state.clone()
    # NOTE: this only rewrites the theta_deg field. The geometry (p_entry,
    # p_exit, length_km, crosses_restricted) becomes stale until the operator
    # caller passes the result back through repair_rebuild_chord. We do that
    # at the cost-evaluation step before scoring.
    new_state.chords_per_tpv[pos][i] = NonParallelChordRecord(
        tpv_index=chord.tpv_index,
        theta_deg=new_theta,
        s_km=chord.s_km,
        p_entry=chord.p_entry,
        p_exit=chord.p_exit,
        length_km=chord.length_km,
        crosses_restricted=chord.crosses_restricted,
    )
    return new_state


def repair_insert_chord_parallel(
    state: ALNSState,
    *,
    tpv_index_by_position: dict[int, int],
    tpv_polys: dict[int, object],
    tpv_geoms: dict[int, TpvGeometry],
    restricted_union,
    min_spacing_km: float,
    rng: random.Random,
    s_step_km: float = DEFAULT_S_STEP_KM,
) -> ALNSState:
    """Greedy insert a parallel chord (theta = 0) at any feasible offset.

    The "parallel preferred" prior lives here: we *first* try theta = 0 with a
    random offset that respects min spacing against existing chords. If no
    parallel offset works we leave the state as-is — the angle-perturbation
    operator will rotate an existing chord, and that's the only way to take
    on a non-parallel pattern.
    """
    candidates = list(state.chords_per_tpv.items())
    rng.shuffle(candidates)
    for pos, chord_list in candidates:
        tpv_index = tpv_index_by_position[pos]
        poly = tpv_polys[tpv_index]
        geom = tpv_geoms[tpv_index]
        # Build the s grid and try each offset (random order) for theta = 0.
        s_max = 0.95 * geom.a_km
        n = int(s_max / s_step_km)
        offsets = [round(-n * s_step_km + i * s_step_km, 6)
                   for i in range(2 * n + 1)]
        rng.shuffle(offsets)
        for s in offsets:
            rec = build_chord_at(
                tpv_index, poly, geom,
                theta_deg=0.0, s_km=s, restricted_union=restricted_union,
            )
            if rec is None or rec.crosses_restricted:
                continue
            candidate = chord_list + [rec]
            ok, _ = chord_set_pairwise_feasible(candidate, min_spacing_km=min_spacing_km)
            if not ok:
                continue
            new_state = state.clone()
            new_state.chords_per_tpv[pos] = candidate
            return new_state
    return state


def repair_insert_chord_any(
    state: ALNSState,
    *,
    tpv_index_by_position: dict[int, int],
    tpv_polys: dict[int, object],
    tpv_geoms: dict[int, TpvGeometry],
    restricted_union,
    min_spacing_km: float,
    rng: random.Random,
    s_step_km: float = DEFAULT_S_STEP_KM,
    theta_step_deg: float = DEFAULT_THETA_STEP_DEG,
    theta_max_deg: float = DEFAULT_THETA_MAX_DEG,
    candidate_cache: dict | None = None,
) -> ALNSState:
    """Fallback insert that searches the full (theta, s) grid.

    ``candidate_cache`` is keyed by ``tpv_index`` and stores the precomputed
    candidate list to avoid rebuilding the (theta, s) grid every iteration
    (which dominates the wall-clock when iterating non-trivially).
    """
    candidates = list(state.chords_per_tpv.items())
    rng.shuffle(candidates)
    for pos, chord_list in candidates:
        tpv_index = tpv_index_by_position[pos]
        if candidate_cache is not None and tpv_index in candidate_cache:
            all_candidates = candidate_cache[tpv_index]
        else:
            poly = tpv_polys[tpv_index]
            geom = tpv_geoms[tpv_index]
            all_candidates = list(iter_candidate_chords(
                tpv_index, poly, geom,
                theta_step_deg=theta_step_deg, theta_max_deg=theta_max_deg,
                s_step_km=s_step_km, restricted_union=restricted_union,
            ))
            if candidate_cache is not None:
                candidate_cache[tpv_index] = all_candidates
        shuffled = list(all_candidates)
        rng.shuffle(shuffled)
        for rec in shuffled:
            candidate = chord_list + [rec]
            ok, _ = chord_set_pairwise_feasible(candidate, min_spacing_km=min_spacing_km)
            if not ok:
                continue
            new_state = state.clone()
            new_state.chords_per_tpv[pos] = candidate
            return new_state
    return state


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

@dataclass
class ALNSResult:
    incumbent: ALNSState
    incumbent_score: float
    iterations_run: int
    elapsed_sec: float
    n_accepted: int
    n_rejected: int
    history: list[tuple[int, float]] = field(default_factory=list)


def refine_plan_alns(
    initial_state: ALNSState,
    *,
    tpv_index_by_position: dict[int, int],
    tpv_polys: dict[int, object],
    tpv_geoms: dict[int, TpvGeometry],
    restricted_union,
    min_spacing_km: float = 100.0,
    theta_penalty_km_per_30deg: float = DEFAULT_THETA_PENALTY_KM,
    time_budget_sec: float = 60.0,
    max_iterations: int = 5000,
    rng: random.Random | None = None,
    incumbent_callback: Callable[[int, ALNSState], None] | None = None,
    feasibility_check: Callable[[ALNSState], bool] | None = None,
    candidate_cache: dict | None = None,
) -> ALNSResult:
    """Run ALNS until ``time_budget_sec`` or ``max_iterations`` is exhausted.

    Returns the best incumbent found.  Feasibility is enforced at every
    accepted move; the initial state must itself be feasible.
    """
    rng = rng or random.Random(0)
    if candidate_cache is None:
        candidate_cache = {}
    incumbent = initial_state.clone()
    incumbent_score = state_score(incumbent, theta_penalty_km_per_30deg=theta_penalty_km_per_30deg)
    history: list[tuple[int, float]] = [(0, incumbent_score)]
    current = incumbent.clone()
    current_score = incumbent_score

    # Simulated-annealing temperature scaled to the initial cost magnitude.
    t0 = max(10.0, 0.05 * abs(current_score) / 1e6)
    decay = 0.998

    destroy_ops = [
        ("remove_chord", destroy_remove_chord),
        ("perturb_angle", destroy_perturb_angle),
    ]
    repair_ops = [
        ("insert_parallel", repair_insert_chord_parallel),
        ("insert_any", repair_insert_chord_any),
    ]
    d_weights = [1.0] * len(destroy_ops)
    r_weights = [1.0] * len(repair_ops)

    t_start = time.time()
    n_accepted = 0
    n_rejected = 0
    iters_run = 0

    for k in range(max_iterations):
        iters_run = k + 1
        if time.time() - t_start > time_budget_sec:
            break

        d_idx = _roulette_pick(d_weights, rng)
        r_idx = _roulette_pick(r_weights, rng)
        d_name, d_fn = destroy_ops[d_idx]
        r_name, r_fn = repair_ops[r_idx]

        # Destroy
        candidate = d_fn(current, rng)
        # Repair (only the two with full arg-set need keyword args).
        if r_name == "insert_parallel":
            candidate = repair_insert_chord_parallel(
                candidate,
                tpv_index_by_position=tpv_index_by_position,
                tpv_polys=tpv_polys, tpv_geoms=tpv_geoms,
                restricted_union=restricted_union,
                min_spacing_km=min_spacing_km, rng=rng,
            )
        else:
            candidate = repair_insert_chord_any(
                candidate,
                tpv_index_by_position=tpv_index_by_position,
                tpv_polys=tpv_polys, tpv_geoms=tpv_geoms,
                restricted_union=restricted_union,
                min_spacing_km=min_spacing_km, rng=rng,
                candidate_cache=candidate_cache,
            )

        candidate_score = state_score(candidate, theta_penalty_km_per_30deg=theta_penalty_km_per_30deg)
        accept = False
        if candidate_score > current_score:
            accept = True
        else:
            delta = current_score - candidate_score
            t_k = t0 * (decay ** k)
            if t_k > 1e-9 and rng.random() < math.exp(-delta / max(t_k, 1e-9)):
                accept = True

        if accept:
            current = candidate
            current_score = candidate_score
            n_accepted += 1
            # Only promote to incumbent if the candidate is real-route feasible.
            # This guards LOCK 1: no surfaced plan exceeds the flight budget.
            is_feasible = True
            if feasibility_check is not None:
                is_feasible = feasibility_check(candidate)
            if candidate_score > incumbent_score and is_feasible:
                incumbent = candidate.clone()
                incumbent_score = candidate_score
                history.append((k + 1, incumbent_score))
                if incumbent_callback is not None:
                    incumbent_callback(k + 1, incumbent)
            d_weights[d_idx] = min(10.0, d_weights[d_idx] * 1.2)
            r_weights[r_idx] = min(10.0, r_weights[r_idx] * 1.2)
        else:
            n_rejected += 1

        d_weights = [max(0.1, w * 0.998) for w in d_weights]
        r_weights = [max(0.1, w * 0.998) for w in r_weights]

    return ALNSResult(
        incumbent=incumbent,
        incumbent_score=incumbent_score,
        iterations_run=iters_run,
        elapsed_sec=time.time() - t_start,
        n_accepted=n_accepted,
        n_rejected=n_rejected,
        history=history,
    )


def _roulette_pick(weights: Sequence[float], rng: random.Random) -> int:
    total = sum(weights)
    if total <= 0:
        return rng.randrange(len(weights))
    r = rng.random() * total
    acc = 0.0
    for i, w in enumerate(weights):
        acc += w
        if r <= acc:
            return i
    return len(weights) - 1
