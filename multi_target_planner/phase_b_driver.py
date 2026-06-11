"""End-to-end driver: refine a Phase A plan with PB-2 ALNS (FPO-6 / PB-4 MVP).

Phase B v1 happens in one shot — no separate stages, no archive.pkl, no
notebook cell-level caching.  The user picks a Phase A plan by index, hands
it to :func:`refine_phase_a_plan`, and gets back the refined ALNS state
together with a materialised route polyline + cost so the comparison PNG
can be rendered immediately.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Sequence

import numpy as np

from multi_target_planner.alns import (
    ALNSResult,
    ALNSState,
    refine_plan_alns,
)
from multi_target_planner.phase_a import _interleave_with_gatepoints
from multi_target_planner.chord_layout import (
    ChordSet,
    TpvGeometry,
    fit_tpv_geometry,
)
from multi_target_planner.chord_layout_b import NonParallelChordRecord
from multi_target_planner.data_loader import TPV
from multi_target_planner.pareto import Plan
from multi_target_planner.route_builder import (
    BuiltRoute,
    Stop,
    build_route,
    make_gatepoint_stop,
    make_sat_stop,
    make_tpv_stop,
    DEFAULT_AIRCRAFT_SPEED_KMH,
)
from multi_target_planner.sat_arc import (
    DEFAULT_T_SAT_H,
    SatArcCandidate,
    _cum_arc_length,
    slice_track_between,
)


# ---------------------------------------------------------------------------
# Conversion between Phase A Plan and ALNS state
# ---------------------------------------------------------------------------

def phase_a_plan_to_alns_state(plan: Plan) -> ALNSState:
    """Convert a Phase A :class:`Plan` into a mutable :class:`ALNSState`.

    Each parallel chord in the Phase A ChordSet maps to a
    :class:`NonParallelChordRecord` with the same ``angle_dev`` carried over
    as ``theta_deg`` and the same ``offset`` carried as ``s_km``.  Geometry
    fields (``p_entry``, ``p_exit``, ``length_km``) are pulled from the
    Phase A chord dictionaries.
    """
    chords_per_tpv: dict[int, list[NonParallelChordRecord]] = {}
    for position, (tpv_idx, chord_set) in enumerate(
        zip(plan.tpv_indices, plan.chord_choices)
    ):
        records: list[NonParallelChordRecord] = []
        for c in chord_set.chords:
            records.append(NonParallelChordRecord(
                tpv_index=tpv_idx,
                theta_deg=float(c["angle_dev"]),
                s_km=float(c["offset"]),
                p_entry=np.asarray(c["pt_a"], dtype=np.float64),
                p_exit=np.asarray(c["pt_b"], dtype=np.float64),
                length_km=float(c["length"]),
                crosses_restricted=False,
            ))
        chords_per_tpv[position] = records
    return ALNSState(
        visit_order=list(plan.tpv_indices),
        chords_per_tpv=chords_per_tpv,
    )


def _records_to_chord_set(
    tpv_index: int,
    records: Sequence[NonParallelChordRecord],
    *,
    min_spacing_km: float,
) -> ChordSet:
    """Wrap a list of :class:`NonParallelChordRecord` in a Phase A
    :class:`ChordSet` so the existing :func:`build_route` accepts it.

    Records are sorted by ``s_km`` so the boustrophedon traversal inside
    :func:`build_route` flies them in major-axis order.
    """
    sorted_records = sorted(records, key=lambda r: r.s_km)
    chord_dicts = tuple(
        {
            "pt_a": rec.p_entry,
            "pt_b": rec.p_exit,
            "length": rec.length_km,
            "offset": rec.s_km,
            "angle_dev": rec.theta_deg,
            "direction": np.array(
                [math.cos(math.radians(rec.theta_deg)),
                 math.sin(math.radians(rec.theta_deg))],
                dtype=np.float64,
            ),
        }
        for rec in sorted_records
    )
    n = len(chord_dicts)
    if n == 0:
        survey = 0.0
    else:
        survey = sum(c["length"] for c in chord_dicts)
        for i in range(n - 1):
            survey += max(min_spacing_km,
                          abs(chord_dicts[i + 1]["offset"]
                              - chord_dicts[i]["offset"]))
    return ChordSet(
        tpv_index=tpv_index,
        n_chords=n,
        angle_dev_deg=sorted_records[0].theta_deg if sorted_records else 0.0,
        chords=chord_dicts,
        survey_dist_km=float(survey),
        n_turns_inside=max(0, n + 1),
    )


def _ordered_stops_from_state(
    state: ALNSState,
    *,
    min_spacing_km: float,
    sat_stop: Stop | None = None,
    sat_position: int | None = None,
) -> list[Stop]:
    """Build the *ordered* (TPV + optional sat) stop list from an ALNSState.

    Gatepoints are NOT included here — the caller is responsible for
    interleaving them via :func:`_interleave_with_gatepoints` so the
    optimal placement is searched, not forced to the head.
    """
    tpv_stops: list[Stop] = []
    for position, tpv_idx in enumerate(state.visit_order):
        records = state.chords_per_tpv.get(position, [])
        if not records:
            continue
        chord_set = _records_to_chord_set(
            tpv_idx, records, min_spacing_km=min_spacing_km,
        )
        tpv_stops.append(make_tpv_stop(tpv_idx, chord_set))
    if sat_stop is not None and sat_position is not None:
        sp = max(0, min(len(tpv_stops), sat_position))
        tpv_stops.insert(sp, sat_stop)
    return tpv_stops


def materialise_route(
    state: ALNSState,
    *,
    base_km: np.ndarray,
    gatepoints_km: np.ndarray,
    restricted_union=None,
    atc_union=None,
    turn_penalty_km: float,
    turn_threshold_deg: float = 30.0,
    turn_factor: float = 1.2,
    atc_factor: float = 1.35,
    min_spacing_km: float = 100.0,
    sat_stop: Stop | None = None,
    sat_position: int | None = None,
    t_sat_h: float | None = None,
    aircraft_speed_kmh: float = DEFAULT_AIRCRAFT_SPEED_KMH,
    dropsonde_union=None,
) -> BuiltRoute | None:
    """Build the actual flight polyline for an ALNS state via Phase A's
    :func:`build_route`.

    Gatepoint placement is found via Phase A's
    :func:`_interleave_with_gatepoints` enumeration (m! * C(k+m, m) layouts
    for k TPVs and m gatepoints — 20 layouts at k=3, m=2).  The cheapest
    feasible layout wins.  Master 2026-06-10: putting gatepoints at the
    head was a bug — the optimal route for case 15 puts them between TPV-2
    and TPV-3 so transit segments don't waste-cross TPV-1.

    If ``sat_stop`` + ``sat_position`` are provided, the sat arc is
    inserted into the TPV-ordered stop list at ``sat_position`` BEFORE
    the gatepoints are interleaved (so sat keeps its TPV-relative slot).
    ``t_sat_h`` is forwarded to :func:`build_route` so the returned
    BuiltRoute carries the computed ``t_dep_h`` field.
    """
    tpv_and_sat_stops = _ordered_stops_from_state(
        state,
        min_spacing_km=min_spacing_km,
        sat_stop=sat_stop,
        sat_position=sat_position,
    )
    gp_stops = [
        make_gatepoint_stop(g)
        for g in np.asarray(gatepoints_km).reshape(-1, 2)
    ]

    best: BuiltRoute | None = None
    for ordered_stops in _interleave_with_gatepoints(tpv_and_sat_stops, gp_stops):
        route = build_route(
            base=base_km,
            stops=ordered_stops,
            restricted_union=restricted_union,
            atc_union=atc_union,
            turn_penalty_km=turn_penalty_km,
            turn_threshold_deg=turn_threshold_deg,
            turn_factor=turn_factor,
            atc_factor=atc_factor,
            t_sat_h=t_sat_h,
            aircraft_speed_kmh=aircraft_speed_kmh,
            dropsonde_union=dropsonde_union,
        )
        if route is None:
            continue
        if best is None or route.cost.total_km < best.cost.total_km - 1e-6:
            best = route
    return best


# ---------------------------------------------------------------------------
# Top-level refine
# ---------------------------------------------------------------------------

@dataclass
class PhaseBRefinement:
    refined_state: ALNSState
    refined_route: BuiltRoute | None
    refined_cost_km: float
    baseline_route: BuiltRoute | None
    baseline_cost_km: float
    chord_delta: int
    cost_delta_km: float
    alns_result: ALNSResult
    refined_sat_candidate: SatArcCandidate | None = None
    refined_sat_position: int | None = None
    baseline_sat_candidate: SatArcCandidate | None = None
    baseline_sat_position: int | None = None
    infeasible_real_sat_insert: bool = False
    infeasibility_reason: str = ""


# Master 2026-06-11 / Luc analysis: satellite has a *physical* direction
# (EarthCARE 2016-05-16 overpass is north→south along the cropped track).
# The aircraft must enter the sat segment with its heading aligned with
# the sat direction — opposite-heading entry is unphysical (the relative
# velocity is 2× orbital, not a coincidence).  Use cos(angle) >= 0.5
# (within 60°) as the alignment threshold.
DEFAULT_SAT_DIR_COS_THRESHOLD = 0.5


def _check_sat_direction_alignment(
    route: BuiltRoute,
    cos_threshold: float = DEFAULT_SAT_DIR_COS_THRESHOLD,
) -> bool:
    """True iff the aircraft's heading approaching the satellite entry is
    aligned with the satellite ground-track direction at that point.

    The polyline carries the materialised stop-by-stop waypoints; we look
    for the FIRST 'sat'-kind segment, take the heading at the waypoint
    before it (aircraft approach) and the heading of the sat segment
    itself, and check the cosine of the angle between them.  Angles > 60°
    (cos < 0.5) are rejected.
    """
    poly = route.polyline
    kinds = route.seg_kinds
    sat_seg_idx: int | None = None
    for k, kind in enumerate(kinds):
        if kind == "sat":
            sat_seg_idx = k
            break
    if sat_seg_idx is None or sat_seg_idx == 0:
        return False
    # Approach: waypoint[sat_seg_idx] is sat entry pt_a; the prior segment
    # ends at sat_seg_idx and started at sat_seg_idx-1.
    approach = poly[sat_seg_idx] - poly[sat_seg_idx - 1]
    sat_dir = poly[sat_seg_idx + 1] - poly[sat_seg_idx]
    a_norm = float(np.linalg.norm(approach))
    s_norm = float(np.linalg.norm(sat_dir))
    if a_norm < 1e-9 or s_norm < 1e-9:
        return False
    cos_angle = float(np.dot(approach, sat_dir) / (a_norm * s_norm))
    return cos_angle >= cos_threshold


def embed_best_sat_arc(
    state: ALNSState,
    *,
    sat_candidates: Sequence[SatArcCandidate],
    sat_track_xy: np.ndarray,
    sat_cum_arc: np.ndarray,
    base_km: np.ndarray,
    gatepoints_km: np.ndarray,
    restricted_union=None,
    atc_union=None,
    turn_penalty_km: float,
    budget_km: float = 10860.0,
    aircraft_speed_kmh: float = DEFAULT_AIRCRAFT_SPEED_KMH,
    t_sat_h: float = DEFAULT_T_SAT_H,
    min_spacing_km: float = 100.0,
    max_candidates_to_try: int = 200,
) -> tuple[BuiltRoute | None, SatArcCandidate | None, int | None]:
    """Try inserting each sat candidate at each viable position; return the
    best feasible route along with the chosen candidate and insertion index.

    Picks max ``coinc_dist_km`` first, then min ``total_km`` as tiebreak.
    Skips candidates with ``T_dep < 0`` or total cost > budget.
    """
    n_tpv_stops = len([1 for pos in range(len(state.visit_order))
                       if state.chords_per_tpv.get(pos)])
    if n_tpv_stops == 0:
        return None, None, None

    # Order: longest arc first; within the same arc-bucket, midpoints closest
    # to BASE first (best T_dep margin).  This guarantees we test the most
    # promising long-arc candidates before falling back to shorter ones.
    base_arr = np.asarray(base_km).reshape(2)
    ordered = sorted(
        sat_candidates,
        key=lambda c: (-c.arc_km, float(np.linalg.norm(c.mid_xy - base_arr))),
    )[:max_candidates_to_try]

    best_route: BuiltRoute | None = None
    best_cand: SatArcCandidate | None = None
    best_pos: int | None = None

    for cand in ordered:
        # Pre-screen with the conservative side-trip cost from BASE.  Even
        # the perfect insertion can't beat side_trip_cost, so this gives a
        # cheap lower bound before invoking the full route builder.
        geo_to_mid = float(np.linalg.norm(cand.mid_xy - base_km))
        if t_sat_h - geo_to_mid / aircraft_speed_kmh < 0.0:
            continue

        track_slice = slice_track_between(
            sat_track_xy, sat_cum_arc, cand.entry_s_km, cand.exit_s_km,
        )
        sat_stop = make_sat_stop(
            pt_a=cand.entry_xy, pt_b=cand.exit_xy,
            arc_km=cand.arc_km, track_waypoints=track_slice,
        )

        for pos in range(n_tpv_stops + 1):
            route = materialise_route(
                state,
                base_km=base_km, gatepoints_km=gatepoints_km,
                restricted_union=restricted_union, atc_union=atc_union,
                turn_penalty_km=turn_penalty_km,
                min_spacing_km=min_spacing_km,
                sat_stop=sat_stop, sat_position=pos,
                t_sat_h=t_sat_h, aircraft_speed_kmh=aircraft_speed_kmh,
            )
            if route is None:
                continue
            if route.cost.total_km > budget_km + 1e-6:
                continue
            if route.cost.t_dep_h is None or route.cost.t_dep_h < 0.0:
                continue
            # Sat direction check: aircraft approach heading must align with
            # satellite ground-track direction (cos(angle) >= 0.5, within 60°)
            # — opposite-heading entry is unphysical.
            if not _check_sat_direction_alignment(route):
                continue
            better = (
                best_route is None
                or route.cost.coinc_dist_km > best_route.cost.coinc_dist_km + 1e-6
                or (abs(route.cost.coinc_dist_km - best_route.cost.coinc_dist_km) <= 1e-6
                    and route.cost.total_km < best_route.cost.total_km - 1e-6)
            )
            if better:
                best_route = route
                best_cand = cand
                best_pos = pos

        # Early exit: if we already have a route at the longest-arc candidate
        # tried so far, no later (shorter-arc) candidate can beat it on coinc.
        if best_route is not None and best_route.cost.coinc_dist_km >= cand.arc_km - 1e-6:
            break

    return best_route, best_cand, best_pos


def refine_phase_a_plan(
    plan: Plan,
    tpvs: Sequence[TPV],
    *,
    base_km: np.ndarray,
    gatepoints_km: np.ndarray,
    restricted_union=None,
    atc_union=None,
    turn_penalty_km: float,
    budget_km: float = 10860.0,
    min_spacing_km: float = 100.0,
    time_budget_sec: float = 60.0,
    max_iterations: int = 5000,
    rng_seed: int = 0,
    incumbent_callback=None,
    sat_candidates: Sequence[SatArcCandidate] | None = None,
    sat_track_xy: np.ndarray | None = None,
    t_sat_h: float = DEFAULT_T_SAT_H,
    aircraft_speed_kmh: float = DEFAULT_AIRCRAFT_SPEED_KMH,
    parallel_only: bool = False,
    freeze_chord_count: bool = False,
    dropsonde_union=None,
    w_chord_len: float = 1.0,
    w_coinc: float = 2.0,
    w_dropsonde: float = 0.5,
    runtime_warn_sec: float = 30 * 60,
) -> PhaseBRefinement:
    """Refine one Phase A plan with ALNS and materialise the result.

    Returns a :class:`PhaseBRefinement` carrying both the baseline (the
    Phase A plan rebuilt with the same machinery so the comparison is
    apples-to-apples) and the refined route.

    A runtime warning is emitted to stderr if wall-clock time exceeds
    ``runtime_warn_sec`` (default 30 min) — Master 2026-06-11: "如果
    Phase B 运行超过 30 分钟，就要报告" — this is defense in depth alongside
    whatever the calling script does.
    """
    import sys as _sys
    import time as _time
    _t_start = _time.time()
    state = phase_a_plan_to_alns_state(plan)
    tpv_index_by_position = {
        position: tpv_idx for position, tpv_idx in enumerate(state.visit_order)
    }
    tpv_polys = {i: t.polygon_km for i, t in enumerate(tpvs)}
    tpv_geoms = {
        i: fit_tpv_geometry(t.label, t.vertices_km) for i, t in enumerate(tpvs)
    }

    # Baseline = Phase A's *actual* optimal route.  Using plan.route
    # directly preserves Phase A's optimal gatepoint placement (vs my
    # earlier shortcut that always put gatepoints at the head and inflated
    # cost by ~1000 km on case 15).
    baseline_route = plan.route
    baseline_cost = baseline_route.cost.total_km

    # Feasibility check: real-route cost must stay under the budget so LOCK 1
    # holds at incumbent promotion. The materialise call is expensive (~50-
    # 200 ms), so we only invoke it when an accepted move actually beats the
    # current incumbent score — which is rare relative to iteration count.
    def _is_budget_feasible(candidate_state):
        rt = materialise_route(
            candidate_state,
            base_km=base_km, gatepoints_km=gatepoints_km,
            restricted_union=restricted_union, atc_union=atc_union,
            turn_penalty_km=turn_penalty_km,
            min_spacing_km=min_spacing_km,
            dropsonde_union=dropsonde_union,
        )
        if rt is None:
            return False
        return rt.cost.total_km <= budget_km + 1e-6

    # Score = -real total cost (lower cost ⇒ higher score, ALNS maximizes).
    # Master 2026-06-11: "想的把模型改正确" — surrogate must align with the
    # real objective (min total flight km), not a proxy like chord length sum.
    # Returns -infinity for infeasible (unroutable) states so ALNS rejects.
    score_cache: dict = {}
    sat_cum_arc_local = (
        _cum_arc_length(sat_track_xy) if sat_track_xy is not None else None
    )

    def _spec_objective_score(candidate_state):
        sig = (
            tuple(candidate_state.visit_order),
            tuple(
                tuple((round(c.s_km, 3), round(c.theta_deg, 3))
                      for c in sorted(candidate_state.chords_per_tpv.get(p, []),
                                      key=lambda c: c.s_km))
                for p in range(len(candidate_state.visit_order))
            ),
        )
        if sig in score_cache:
            return score_cache[sig]
        # Try to embed the best sat arc into THIS candidate state, so the
        # score reflects what Phase B actually delivers (coinc_dist is the
        # main objective ≥ chord length — without embedding, ALNS pushes
        # chords until sat headroom is gone, regressing case 15).
        embedded_route = None
        if sat_candidates and sat_track_xy is not None and sat_cum_arc_local is not None:
            embedded_route, _, _ = embed_best_sat_arc(
                candidate_state,
                sat_candidates=sat_candidates,
                sat_track_xy=sat_track_xy, sat_cum_arc=sat_cum_arc_local,
                base_km=base_km, gatepoints_km=gatepoints_km,
                restricted_union=restricted_union, atc_union=atc_union,
                turn_penalty_km=turn_penalty_km,
                budget_km=budget_km,
                aircraft_speed_kmh=aircraft_speed_kmh,
                t_sat_h=t_sat_h,
                min_spacing_km=min_spacing_km,
                max_candidates_to_try=20,  # subset for speed during scoring
            )
        rt = embedded_route
        # HARD REJECT: if sat can't be embedded in this state, the state
        # fails objective 2 (the most heavily weighted) AND fails the spec's
        # sat 6-min hard constraint.  ALNS must NOT pick such a state.
        if rt is None or rt.cost.total_km > budget_km + 1e-6:
            score = float("-inf")
        else:
            # Spec objectives (formulated objectives&constraints-multi):
            # 1) max chord_length_sum (n_chords locked by case identity)
            # 2) max coinc_dist (sat overpass)
            # 3) max dropsonde flight km (passive metric)
            # Budget = hard constraint, NOT objective.
            score = (
                w_chord_len * rt.cost.chord_length_sum_km
                + w_coinc * rt.cost.coinc_dist_km
                + w_dropsonde * rt.cost.dropsonde_dist_km
            )
        score_cache[sig] = score
        return score

    alns_result = refine_plan_alns(
        state,
        tpv_index_by_position=tpv_index_by_position,
        tpv_polys=tpv_polys, tpv_geoms=tpv_geoms,
        restricted_union=restricted_union,
        min_spacing_km=min_spacing_km,
        time_budget_sec=time_budget_sec,
        max_iterations=max_iterations,
        rng=random.Random(rng_seed),
        incumbent_callback=incumbent_callback,
        feasibility_check=_is_budget_feasible,
        parallel_only=parallel_only,
        freeze_chord_count=freeze_chord_count,
        score_fn=_spec_objective_score if freeze_chord_count else None,
    )

    refined_route = materialise_route(
        alns_result.incumbent,
        base_km=base_km, gatepoints_km=gatepoints_km,
        restricted_union=restricted_union, atc_union=atc_union,
        turn_penalty_km=turn_penalty_km,
        min_spacing_km=min_spacing_km,
    )
    refined_cost = (
        refined_route.cost.total_km if refined_route is not None else float("inf")
    )

    # Embed the best satellite arc into baseline AND refined routes so the
    # comparison includes the same hard sat constraint Phase A only checked
    # existentially.  This is what Master flagged on 2026-06-10: Phase B
    # without real sat insertion is just chord shuffling.
    baseline_sat_cand: SatArcCandidate | None = None
    baseline_sat_pos: int | None = None
    refined_sat_cand: SatArcCandidate | None = None
    refined_sat_pos: int | None = None

    if sat_candidates and sat_track_xy is not None and sat_track_xy.shape[0] >= 2:
        sat_cum_arc = _cum_arc_length(sat_track_xy)
        baseline_with_sat, baseline_sat_cand, baseline_sat_pos = embed_best_sat_arc(
            state,
            sat_candidates=sat_candidates,
            sat_track_xy=sat_track_xy, sat_cum_arc=sat_cum_arc,
            base_km=base_km, gatepoints_km=gatepoints_km,
            restricted_union=restricted_union, atc_union=atc_union,
            turn_penalty_km=turn_penalty_km,
            budget_km=budget_km,
            aircraft_speed_kmh=aircraft_speed_kmh,
            t_sat_h=t_sat_h,
            min_spacing_km=min_spacing_km,
        )
        if baseline_with_sat is not None:
            baseline_route = baseline_with_sat
            baseline_cost = baseline_with_sat.cost.total_km
        refined_with_sat, refined_sat_cand, refined_sat_pos = embed_best_sat_arc(
            alns_result.incumbent,
            sat_candidates=sat_candidates,
            sat_track_xy=sat_track_xy, sat_cum_arc=sat_cum_arc,
            base_km=base_km, gatepoints_km=gatepoints_km,
            restricted_union=restricted_union, atc_union=atc_union,
            turn_penalty_km=turn_penalty_km,
            budget_km=budget_km,
            aircraft_speed_kmh=aircraft_speed_kmh,
            t_sat_h=t_sat_h,
            min_spacing_km=min_spacing_km,
        )
        if refined_with_sat is not None:
            refined_route = refined_with_sat
            refined_cost = refined_with_sat.cost.total_km

    # Hard reject: if sat candidates were supplied but no candidate ≥ 6 min
    # could be embedded in the refined route under the precise insertion
    # cost model, the plan is infeasible (Master + Luc 2026-06-10: 6 min is a
    # HARD lower bound, not "skip sat").  Phase A's conservative side-trip
    # estimate over-stated the headroom; precise insertion catches it.
    infeasible_real_sat = False
    reason = ""
    if sat_candidates and sat_track_xy is not None and refined_sat_cand is None:
        infeasible_real_sat = True
        reason = (
            "no sat arc >= 6 min fits the refined route under precise "
            "insertion cost; Phase A's conservative side-trip estimate "
            "over-stated headroom for this case"
        )

    _elapsed = _time.time() - _t_start
    if _elapsed > runtime_warn_sec:
        print(
            f"*** Phase B WARNING: refine_phase_a_plan took {_elapsed:.0f} s "
            f"(> {runtime_warn_sec:.0f} s threshold). Plan signature: "
            f"n_tpvs={len(plan.tpv_indices)}, total_chords={plan.total_chords}. "
            f"Likely cause: large chord-count makes per-iter materialise + sat "
            f"embed quadratic. Cap max_iterations or shorten time_budget_sec.",
            file=_sys.stderr,
        )

    return PhaseBRefinement(
        refined_state=alns_result.incumbent,
        refined_route=refined_route,
        refined_cost_km=float(refined_cost),
        baseline_route=baseline_route,
        baseline_cost_km=float(baseline_cost),
        chord_delta=int(
            alns_result.incumbent.total_chords() - state.total_chords()
        ),
        cost_delta_km=float(refined_cost - baseline_cost),
        alns_result=alns_result,
        refined_sat_candidate=refined_sat_cand,
        refined_sat_position=refined_sat_pos,
        baseline_sat_candidate=baseline_sat_cand,
        baseline_sat_position=baseline_sat_pos,
        infeasible_real_sat_insert=infeasible_real_sat,
        infeasibility_reason=reason,
    )
