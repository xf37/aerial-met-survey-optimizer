"""Unit + smoke tests for multi_target_planner.verifier.

The verifier is D5 — independent of the solver.  These tests pin its
negative-path behaviour: budget overrun, restricted-zone crossing,
gatepoint omission, undersized chord spacing, and the two satellite
failure modes (no budget headroom for the side trip, and T_dep < 0
because the sat midpoint is unreachable in the satellite's coincidence
window).
"""

from __future__ import annotations

import os
from dataclasses import replace

import numpy as np
import pytest
from shapely.geometry import Polygon
from shapely.ops import unary_union

from multi_target_planner.chord_layout import (
    ChordSet,
    enumerate_chord_sets,
    fit_tpv_geometry,
)
from multi_target_planner.data_loader import (
    DEFAULT_DATA_DIR,
    TPV,
    TPV_FILENAME_PHASE0A,
    LocalFrame,
)
from multi_target_planner.pareto import Plan
from multi_target_planner.route_builder import (
    BuiltRoute,
    CostBreakdown,
    build_route,
    make_tpv_stop,
)
from multi_target_planner.sat_arc import (
    DEFAULT_SAT_MIN_LENGTH_KM,
    DEFAULT_T_SAT_H,
    SatArcCandidate,
    enumerate_sat_arc_candidates,
)
from multi_target_planner.verifier import (
    IndependentMissionState,
    VerifierReport,
    verify_plan,
)


pytestmark = pytest.mark.skipif(
    not os.path.isdir(DEFAULT_DATA_DIR),
    reason=f"DATA_DIR not present: {DEFAULT_DATA_DIR}",
)


def _square_tpv(label: str, x0: float, y0: float, side: float = 500.0) -> TPV:
    verts = np.array(
        [
            [x0 - side / 2, y0 - side / 2],
            [x0 + side / 2, y0 - side / 2],
            [x0 + side / 2, y0 + side / 2],
            [x0 - side / 2, y0 + side / 2],
        ],
        dtype=np.float64,
    )
    return TPV(label=label, polygon_km=Polygon(verts).buffer(0), vertices_km=verts)


def _chord_dict(pt_a, pt_b, offset, angle_dev=0.0):
    direction = np.asarray(pt_b, dtype=np.float64) - np.asarray(pt_a, dtype=np.float64)
    length = float(np.linalg.norm(direction))
    if length > 1e-9:
        direction = direction / length
    return {
        "pt_a": np.asarray(pt_a, dtype=np.float64),
        "pt_b": np.asarray(pt_b, dtype=np.float64),
        "length": length,
        "offset": offset,
        "angle_dev": angle_dev,
        "direction": direction,
    }


def _synthetic_plan(
    polyline: np.ndarray,
    *,
    cost_km: float,
    chord_choices: tuple = (),
    tpv_indices: tuple = (),
) -> Plan:
    """Build a Plan from a polyline with an externally specified total cost.

    Mostly bypasses the solver to give the verifier something to chew on.
    """
    cb = CostBreakdown(
        geometric_km=cost_km,
        atc_extra_km=0.0,
        turn_penalty_km=0.0,
        total_km=cost_km,
        n_sharp_turns=0,
        n_total_turns=0,
    )
    seg_kinds = tuple("transit" for _ in range(polyline.shape[0] - 1))
    route = BuiltRoute(polyline=polyline, seg_kinds=seg_kinds, cost=cb)
    return Plan(
        tpv_indices=tpv_indices or tuple(range(len(chord_choices))),
        chord_choices=tuple(chord_choices),
        route=route,
    )


# ---------------------------------------------------------------------------
# Positive smokes
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def mission_state() -> IndependentMissionState:
    return IndependentMissionState.from_disk()


def test_state_reloads_seven_layers_from_disk(mission_state):
    assert mission_state.frame is not None
    assert mission_state.base.shape == (2,)
    assert mission_state.gatepoints_km.shape[1] == 2
    assert mission_state.restricted_union is not None
    assert mission_state.atc_union is not None
    assert mission_state.sat_track_xy.shape[1] == 2
    assert len(mission_state.sat_candidates) > 0


# ---------------------------------------------------------------------------
# N1 — budget violation
# ---------------------------------------------------------------------------

def test_verify_plan_flags_budget_overrun(mission_state):
    tpv = _square_tpv("sq", mission_state.base[0] + 5000.0, 0.0)
    geom = fit_tpv_geometry(tpv.label, tpv.vertices_km)
    sets = enumerate_chord_sets(
        tpv_index=0, tpv_polygon_km=tpv.polygon_km, geom=geom,
        n_chord_options=(1,), angle_devs_deg=(0.0,), min_spacing_km=100.0,
    )
    if not sets:
        pytest.skip("chord enumeration empty for synthetic TPV")
    route = build_route(base=mission_state.base, stops=[make_tpv_stop(0, sets[0])])
    plan = Plan(tpv_indices=(0,), chord_choices=(sets[0],), route=route)
    report = verify_plan(plan, state=mission_state, budget_km=100.0)
    assert isinstance(report, VerifierReport)
    assert not report.ok
    assert any(v.startswith("budget") for v in report.violations)
    assert report.independent_cost_km > 100.0


# ---------------------------------------------------------------------------
# N2 — restricted airspace crossing
# ---------------------------------------------------------------------------

def test_verify_plan_flags_restricted_crossing(mission_state):
    # Inject a fat restricted polygon centered at the frame origin into the
    # state copy; build a synthetic plan whose route punches straight through.
    restricted_block = Polygon([[-200, -200], [200, -200], [200, 200], [-200, 200]])
    spoofed = replace(mission_state, restricted_union=unary_union([restricted_block]))
    polyline = np.array([[-500, 0], [500, 0], [-500, 0]], dtype=np.float64)
    plan = _synthetic_plan(polyline, cost_km=2000.0)
    report = verify_plan(plan, state=spoofed, budget_km=10_000.0)
    assert not report.ok
    assert any(v.startswith("restricted") for v in report.violations)


# ---------------------------------------------------------------------------
# N3 — gatepoint not visited
# ---------------------------------------------------------------------------

def test_verify_plan_flags_unvisited_gatepoint(mission_state):
    if mission_state.gatepoints_km.size == 0:
        pytest.skip("Real data has no gatepoints — N3 not applicable")
    # Polyline that goes nowhere near the gatepoints.
    polyline = np.array(
        [[mission_state.base[0], mission_state.base[1]],
         [mission_state.base[0] + 5.0, mission_state.base[1]],
         [mission_state.base[0], mission_state.base[1]]],
        dtype=np.float64,
    )
    plan = _synthetic_plan(polyline, cost_km=10.0)
    # Drop restricted check by giving a wide-open state; keep gatepoints in.
    spoofed = replace(mission_state, restricted_union=None)
    report = verify_plan(plan, state=spoofed, budget_km=10_000.0)
    assert not report.ok
    assert any("gatepoint" in v.lower() for v in report.violations)


# ---------------------------------------------------------------------------
# N4 — undersized chord spacing
# ---------------------------------------------------------------------------

def test_verify_plan_flags_chord_spacing_under_floor(mission_state):
    # Two chords at offsets 0 and 50 — well below the 100 km spec floor.
    bad_chords = (
        _chord_dict(pt_a=(-100, 0),   pt_b=(100, 0),   offset=0.0),
        _chord_dict(pt_a=(-100, 50),  pt_b=(100, 50),  offset=50.0),
    )
    bad_set = ChordSet(
        tpv_index=0, n_chords=2, angle_dev_deg=0.0,
        chords=bad_chords, survey_dist_km=200.0 + 100.0, n_turns_inside=3,
    )
    polyline = np.array(
        [mission_state.base, [-100, 0], [100, 0], [-100, 50], [100, 50],
         mission_state.base],
        dtype=np.float64,
    )
    plan = _synthetic_plan(
        polyline, cost_km=100.0,
        chord_choices=(bad_set,), tpv_indices=(0,),
    )
    # Replace gatepoints with empty and restricted with None to isolate N4.
    spoofed = replace(
        mission_state,
        restricted_union=None,
        gatepoints_km=np.empty((0, 2), dtype=np.float64),
    )
    report = verify_plan(
        plan, state=spoofed, budget_km=10_000.0,
        min_chord_spacing_km=100.0,
    )
    assert not report.ok
    assert any("chord spacing" in v.lower() for v in report.violations)


# ---------------------------------------------------------------------------
# N5 — sat: no budget headroom for any side trip
# ---------------------------------------------------------------------------

def test_verify_plan_flags_sat_when_no_headroom():
    track = np.array([[0, 0], [1000, 0]], dtype=np.float64)
    candidates = tuple(enumerate_sat_arc_candidates(track))
    frame = LocalFrame(lon_c=0.0, lat_c=0.0)
    spoofed = IndependentMissionState(
        frame=frame,
        base=np.array([0.0, 0.0]),
        gatepoints_km=np.empty((0, 2), dtype=np.float64),
        restricted_union=None,
        atc_union=None,
        sat_track_xy=track,
        sat_candidates=candidates,
    )
    # Plan cost == budget -> zero headroom -> sat infeasible.
    polyline = np.array([[0, 0], [10, 0], [0, 0]], dtype=np.float64)
    plan = _synthetic_plan(polyline, cost_km=10_000.0)
    report = verify_plan(plan, state=spoofed, budget_km=10_000.0)
    assert not report.ok
    assert any(v.startswith("sat") for v in report.violations)


# ---------------------------------------------------------------------------
# N6 — sat: T_dep < 0 because midpoint is beyond satellite's reach window
# ---------------------------------------------------------------------------

def test_verify_plan_flags_sat_when_t_dep_negative():
    # Track sits far enough away that all midpoints exceed speed * T_sat.
    far_track = np.array([[5000, 0], [10_000, 0]], dtype=np.float64)
    candidates = tuple(enumerate_sat_arc_candidates(far_track))
    frame = LocalFrame(lon_c=0.0, lat_c=0.0)
    spoofed = IndependentMissionState(
        frame=frame,
        base=np.array([0.0, 0.0]),
        gatepoints_km=np.empty((0, 2), dtype=np.float64),
        restricted_union=None,
        atc_union=None,
        sat_track_xy=far_track,
        sat_candidates=candidates,
    )
    polyline = np.array([[0, 0], [10, 0], [0, 0]], dtype=np.float64)
    plan = _synthetic_plan(polyline, cost_km=10.0)
    report = verify_plan(
        plan,
        state=spoofed,
        budget_km=100_000.0,             # headroom is not the issue
        t_sat_h=DEFAULT_T_SAT_H,          # speed * T_sat = 3400 km < 5000 km
    )
    assert not report.ok
    assert any(v.startswith("sat") for v in report.violations)


# ---------------------------------------------------------------------------
# Independence assertion — the verifier must not just echo the solver
# ---------------------------------------------------------------------------

def test_verify_plan_independent_cost_disagrees_with_zeroed_breakdown(mission_state):
    """If a plan's BuiltRoute claims total_km=0 but the polyline is long, the
    verifier's independently-recomputed cost must NOT be 0.  This pins the
    R3 property: D5 doesn't trust solver-internal numbers, it recomputes.
    """
    polyline = np.array([[0, 0], [1000, 0]], dtype=np.float64)
    plan = _synthetic_plan(polyline, cost_km=0.0)
    spoofed = replace(mission_state, restricted_union=None,
                      gatepoints_km=np.empty((0, 2), dtype=np.float64))
    report = verify_plan(plan, state=spoofed, budget_km=10_000.0)
    # solver said 0, verifier independently sees ~1000 km of geometry.
    assert report.independent_cost_km > 900.0
