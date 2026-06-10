"""Unit tests for multi_target_planner.sat_arc."""

from __future__ import annotations

import math

import numpy as np
import pytest

from multi_target_planner.sat_arc import (
    DEFAULT_SAT_ARC_MAX_KM,
    DEFAULT_SAT_ARC_STEP_KM,
    DEFAULT_SAT_MIN_LENGTH_KM,
    DEFAULT_T_SAT_H,
    SatArcCandidate,
    SatFeasibility,
    check_sat_feasibility,
    enumerate_sat_arc_candidates,
)
from multi_target_planner.route_builder import BuiltRoute, CostBreakdown


# ---------------------------------------------------------------------------
# enumerate_sat_arc_candidates
# ---------------------------------------------------------------------------

def test_enumerate_sat_arc_candidates_empty_for_too_short_track():
    track = np.array([[0.0, 0.0], [40.0, 0.0]])    # 40 km < 85 km min
    out = enumerate_sat_arc_candidates(track)
    assert out == []


def test_enumerate_sat_arc_candidates_generates_grid():
    track = np.array([[0.0, 0.0], [1000.0, 0.0]], dtype=np.float64)
    out = enumerate_sat_arc_candidates(
        track,
        entry_step_km=100.0,
        arc_step_km=85.0,
        sat_min_length_km=85.0,
        sat_arc_max_km=425.0,
    )
    assert out, "Expected several candidates"
    for c in out:
        assert isinstance(c, SatArcCandidate)
        assert c.arc_km >= 85.0
        assert c.arc_km <= 425.0 + 1e-6
        # entry, exit, mid lie on the x-axis (the track).
        assert math.isclose(c.entry_xy[1], 0.0, abs_tol=1e-6)
        assert math.isclose(c.exit_xy[1], 0.0, abs_tol=1e-6)
        assert math.isclose(c.mid_xy[1], 0.0, abs_tol=1e-6)


def test_enumerate_sat_arc_candidates_arc_steps_match_default():
    track = np.array([[0.0, 0.0], [2000.0, 0.0]], dtype=np.float64)
    out = enumerate_sat_arc_candidates(track)
    # Should have several arc lengths >= 85 km.
    arcs = sorted({round(c.arc_km, 2) for c in out})
    assert 85.0 in arcs
    # Final arc_km should not exceed SAT_ARC_MAX.
    assert max(arcs) <= DEFAULT_SAT_ARC_MAX_KM + 1e-6


# ---------------------------------------------------------------------------
# check_sat_feasibility
# ---------------------------------------------------------------------------

def _fake_route(total_km: float) -> BuiltRoute:
    """Return a BuiltRoute with a placeholder polyline + given total cost."""
    poly = np.array([[0.0, 0.0], [10.0, 0.0]], dtype=np.float64)
    cb = CostBreakdown(
        geometric_km=total_km, atc_extra_km=0.0, turn_penalty_km=0.0,
        total_km=total_km, n_sharp_turns=0, n_total_turns=0,
    )
    return BuiltRoute(polyline=poly, seg_kinds=("transit",), cost=cb)


def test_check_sat_feasibility_no_headroom_rejects():
    route = _fake_route(total_km=4250.0)   # at budget
    sat = check_sat_feasibility(
        plan_route=route,
        base=np.zeros(2),
        candidates=[
            SatArcCandidate(
                entry_idx=0, entry_xy=np.array([100, 0]), exit_xy=np.array([200, 0]),
                arc_km=100.0, mid_xy=np.array([150, 0]),
            )
        ],
        budget_km=4250.0,
    )
    assert sat.feasible is False
    assert "headroom" in sat.reason


def test_check_sat_feasibility_t_dep_negative_when_arc_too_far():
    # speed = 850, T_sat = 4h -> max geo_to_mid = 3400 km. 4000 km is unreachable.
    route = _fake_route(total_km=100.0)
    sat = check_sat_feasibility(
        plan_route=route,
        base=np.zeros(2),
        candidates=[
            SatArcCandidate(
                entry_idx=0, entry_xy=np.array([3950, 0]),
                exit_xy=np.array([4050, 0]), arc_km=100.0,
                mid_xy=np.array([4000, 0]),
            )
        ],
        budget_km=4250.0,
    )
    assert sat.feasible is False


def test_check_sat_feasibility_passes_when_arc_within_budget_and_t_dep_ok():
    route = _fake_route(total_km=100.0)   # tons of headroom
    cand = SatArcCandidate(
        entry_idx=0, entry_xy=np.array([200, 0]), exit_xy=np.array([300, 0]),
        arc_km=100.0, mid_xy=np.array([250, 0]),
    )
    sat = check_sat_feasibility(
        plan_route=route,
        base=np.zeros(2),
        candidates=[cand],
        budget_km=4250.0,
    )
    assert sat.feasible is True
    assert sat.chosen_arc_km == 100.0
    # T_dep = 4 - 250/850 ~ 3.706 h
    assert sat.t_dep_h > 0.0


def test_check_sat_feasibility_picks_largest_arc():
    route = _fake_route(total_km=100.0)
    short = SatArcCandidate(
        entry_idx=0, entry_xy=np.array([200, 0]), exit_xy=np.array([285, 0]),
        arc_km=85.0, mid_xy=np.array([242.5, 0]),
    )
    long = SatArcCandidate(
        entry_idx=1, entry_xy=np.array([200, 0]), exit_xy=np.array([625, 0]),
        arc_km=425.0, mid_xy=np.array([412.5, 0]),
    )
    sat = check_sat_feasibility(
        plan_route=route,
        base=np.zeros(2),
        candidates=[short, long],
        budget_km=4250.0,
    )
    assert sat.feasible is True
    assert sat.chosen_arc_km == 425.0
