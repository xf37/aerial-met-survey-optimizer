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
    build_route,
    make_gatepoint_stop,
    make_tpv_stop,
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
) -> BuiltRoute | None:
    """Build the actual flight polyline for an ALNS state via Phase A's
    :func:`build_route`.

    Gatepoints are placed at the head of the stop sequence (same order
    they came in).  PB-2 v1 freezes Phase A's gatepoint placement; PB-5
    will release it.
    """
    stops = []
    for gp in np.asarray(gatepoints_km).reshape(-1, 2):
        stops.append(make_gatepoint_stop(gp))
    for position, tpv_idx in enumerate(state.visit_order):
        records = state.chords_per_tpv.get(position, [])
        if not records:
            continue
        chord_set = _records_to_chord_set(
            tpv_idx, records, min_spacing_km=min_spacing_km,
        )
        stops.append(make_tpv_stop(tpv_idx, chord_set))
    return build_route(
        base=base_km,
        stops=stops,
        restricted_union=restricted_union,
        atc_union=atc_union,
        turn_penalty_km=turn_penalty_km,
        turn_threshold_deg=turn_threshold_deg,
        turn_factor=turn_factor,
        atc_factor=atc_factor,
    )


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


def refine_phase_a_plan(
    plan: Plan,
    tpvs: Sequence[TPV],
    *,
    base_km: np.ndarray,
    gatepoints_km: np.ndarray,
    restricted_union=None,
    atc_union=None,
    turn_penalty_km: float,
    min_spacing_km: float = 100.0,
    time_budget_sec: float = 60.0,
    max_iterations: int = 5000,
    rng_seed: int = 0,
    incumbent_callback=None,
) -> PhaseBRefinement:
    """Refine one Phase A plan with ALNS and materialise the result.

    Returns a :class:`PhaseBRefinement` carrying both the baseline (the
    Phase A plan rebuilt with the same machinery so the comparison is
    apples-to-apples) and the refined route.
    """
    state = phase_a_plan_to_alns_state(plan)
    tpv_index_by_position = {
        position: tpv_idx for position, tpv_idx in enumerate(state.visit_order)
    }
    tpv_polys = {i: t.polygon_km for i, t in enumerate(tpvs)}
    tpv_geoms = {
        i: fit_tpv_geometry(t.label, t.vertices_km) for i, t in enumerate(tpvs)
    }

    baseline_route = materialise_route(
        state,
        base_km=base_km, gatepoints_km=gatepoints_km,
        restricted_union=restricted_union, atc_union=atc_union,
        turn_penalty_km=turn_penalty_km,
        min_spacing_km=min_spacing_km,
    )
    baseline_cost = (
        baseline_route.cost.total_km if baseline_route is not None else float("inf")
    )

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
    )
