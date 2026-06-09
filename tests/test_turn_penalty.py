"""Tests for multi_target_planner.turn_penalty (FPO-5 / Phase 0a / S1)."""

from __future__ import annotations

import math

import numpy as np
import pytest

from multi_target_planner.turn_penalty import (
    compute_turn_angles_deg,
    turn_penalty_cost,
)


# Default parameter triple per Master's directive 2026-06-09 + precompute_mission.py L46.
DEFAULT_PENALTY_KM = 106.0
DEFAULT_THRESHOLD_DEG = 30.0
DEFAULT_FACTOR = 1.2


# ---------------------------------------------------------------------------
# compute_turn_angles_deg — geometry edges
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("n", [0, 1, 2])
def test_empty_or_short_polyline_returns_no_turns(n):
    pts = np.zeros((n, 2))
    out = compute_turn_angles_deg(pts)
    assert out.shape == (0,)


def test_wrong_shape_raises():
    with pytest.raises(ValueError):
        compute_turn_angles_deg(np.zeros((5,)))
    with pytest.raises(ValueError):
        compute_turn_angles_deg(np.zeros((5, 3)))


def test_collinear_polyline_yields_zero_angles():
    # 5 evenly spaced points along x-axis: every interior vertex is a 0° vertex.
    pts = np.column_stack([np.arange(5, dtype=np.float64), np.zeros(5)])
    angles = compute_turn_angles_deg(pts)
    assert angles.shape == (3,)
    np.testing.assert_allclose(angles, 0.0, atol=1e-9)


def test_single_90_degree_turn():
    # right-angle: east then north.
    pts = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]])
    angles = compute_turn_angles_deg(pts)
    assert angles.shape == (1,)
    assert math.isclose(angles[0], 90.0, abs_tol=1e-9)


def test_single_180_degree_uturn():
    # backtrack along the same line.
    pts = np.array([[0.0, 0.0], [1.0, 0.0], [0.0, 0.0]])
    angles = compute_turn_angles_deg(pts)
    assert angles.shape == (1,)
    assert math.isclose(angles[0], 180.0, abs_tol=1e-9)


def test_zero_length_segment_returns_zero_not_nan():
    # repeated middle point -> first segment degenerate at vertex 1.
    pts = np.array([[0.0, 0.0], [0.0, 0.0], [1.0, 1.0]])
    angles = compute_turn_angles_deg(pts)
    assert angles.shape == (1,)
    assert math.isfinite(angles[0])
    assert math.isclose(angles[0], 0.0, abs_tol=1e-9)


def test_acute_turn_30_degrees_exact():
    # vectors (1,0) and (cos30°, sin30°) → exactly 30° deviation.
    a = math.radians(30.0)
    pts = np.array([[0.0, 0.0], [1.0, 0.0], [1.0 + math.cos(a), math.sin(a)]])
    angles = compute_turn_angles_deg(pts)
    assert angles.shape == (1,)
    assert math.isclose(angles[0], 30.0, abs_tol=1e-9)


def test_mixed_polyline_returns_correct_angles_in_order():
    # 0° (collinear), 90° (right turn), 45° (slight diagonal)
    pts = np.array(
        [
            [0.0, 0.0],     # p0
            [1.0, 0.0],     # p1 — interior, but it's endpoint of seg 0 so not a vertex angle yet
            [2.0, 0.0],     # p2 — first interior vertex angle: 0° (straight east)
            [2.0, 1.0],     # p3 — second interior vertex angle: 90° (north after east)
            [2.0 + math.cos(math.radians(45)), 1.0 + math.sin(math.radians(45))],
                            # p4 — third interior vertex angle: 45° from "north"
        ]
    )
    angles = compute_turn_angles_deg(pts)
    assert angles.shape == (3,)
    np.testing.assert_allclose(angles, [0.0, 90.0, 45.0], atol=1e-9)


# ---------------------------------------------------------------------------
# turn_penalty_cost — gating + accumulation
# ---------------------------------------------------------------------------

def test_empty_angles_yields_zero_cost():
    assert turn_penalty_cost(
        np.empty((0,)), DEFAULT_PENALTY_KM, DEFAULT_THRESHOLD_DEG, DEFAULT_FACTOR
    ) == 0.0


def test_threshold_strict_below_is_free():
    # Reading B: 29° ≤ 30 → SMALL → FREE (0 km)
    cost = turn_penalty_cost(
        np.array([29.0]), DEFAULT_PENALTY_KM, DEFAULT_THRESHOLD_DEG, DEFAULT_FACTOR
    )
    assert cost == 0.0


def test_threshold_strict_exact_is_free():
    # Reading B: 30.0 == threshold → STRICT > → SMALL → FREE (0 km)
    cost = turn_penalty_cost(
        np.array([30.0]), DEFAULT_PENALTY_KM, DEFAULT_THRESHOLD_DEG, DEFAULT_FACTOR
    )
    assert cost == 0.0


def test_threshold_strict_above_applies_factor():
    # 30.0001° → sharp → 1.2× penalty (127.2 km).
    cost = turn_penalty_cost(
        np.array([30.0001]), DEFAULT_PENALTY_KM, DEFAULT_THRESHOLD_DEG, DEFAULT_FACTOR
    )
    assert math.isclose(cost, DEFAULT_PENALTY_KM * DEFAULT_FACTOR, rel_tol=1e-9)


def test_90_degree_turn_uses_factor():
    cost = turn_penalty_cost(
        np.array([90.0]), DEFAULT_PENALTY_KM, DEFAULT_THRESHOLD_DEG, DEFAULT_FACTOR
    )
    assert math.isclose(cost, DEFAULT_PENALTY_KM * DEFAULT_FACTOR, rel_tol=1e-12)


def test_180_degree_uturn_uses_factor():
    cost = turn_penalty_cost(
        np.array([180.0]), DEFAULT_PENALTY_KM, DEFAULT_THRESHOLD_DEG, DEFAULT_FACTOR
    )
    assert math.isclose(cost, DEFAULT_PENALTY_KM * DEFAULT_FACTOR, rel_tol=1e-12)


def test_mixed_angles_sum_correctly():
    # 0°, 30° (exact, FREE), 31° (sharp), 90° (sharp), 180° (sharp) — three sharp, two small (free).
    angles = np.array([0.0, 30.0, 31.0, 90.0, 180.0])
    cost = turn_penalty_cost(
        angles, DEFAULT_PENALTY_KM, DEFAULT_THRESHOLD_DEG, DEFAULT_FACTOR
    )
    expected = 3 * DEFAULT_PENALTY_KM * DEFAULT_FACTOR  # only 3 sharp turns contribute
    assert math.isclose(cost, expected, rel_tol=1e-12)


def test_all_small_turns_yield_zero_cost():
    # Reading B: a plan with ONLY sub-threshold turns costs nothing.
    # This was the 07-idiom behaviour; we preserve it.
    small_angles = np.full(7, 20.0)
    cost = turn_penalty_cost(
        small_angles, DEFAULT_PENALTY_KM, DEFAULT_THRESHOLD_DEG, DEFAULT_FACTOR
    )
    assert cost == 0.0


def test_only_sharp_turns_count():
    # 10 turns total, 4 sharp → cost = 4 × 106 × 1.2 = 508.8 km
    angles = np.array([5, 10, 31, 45, 25, 90, 20, 180, 28, 31], dtype=float)
    cost = turn_penalty_cost(
        angles, DEFAULT_PENALTY_KM, DEFAULT_THRESHOLD_DEG, DEFAULT_FACTOR
    )
    expected = 5 * DEFAULT_PENALTY_KM * DEFAULT_FACTOR  # 31, 45, 90, 180, 31 are sharp
    assert math.isclose(cost, expected, rel_tol=1e-12)


# ---------------------------------------------------------------------------
# Integration smoke: polyline → angles → cost
# ---------------------------------------------------------------------------

def test_end_to_end_polyline_to_cost():
    # Square route with 3 explicit 90° turns + final return-to-base (not an interior vertex).
    pts = np.array(
        [
            [0.0, 0.0],
            [10.0, 0.0],
            [10.0, 10.0],
            [0.0, 10.0],
            [0.0, 0.0],
        ]
    )
    angles = compute_turn_angles_deg(pts)
    assert angles.shape == (3,)
    np.testing.assert_allclose(angles, [90.0, 90.0, 90.0], atol=1e-9)

    cost = turn_penalty_cost(
        angles, DEFAULT_PENALTY_KM, DEFAULT_THRESHOLD_DEG, DEFAULT_FACTOR
    )
    assert math.isclose(cost, 3 * DEFAULT_PENALTY_KM * DEFAULT_FACTOR, rel_tol=1e-12)


def test_end_to_end_zigzag_only_small_turns_is_free():
    # A gentle zigzag where every turn is < 30° — under Reading B this is free.
    a = math.radians(15.0)
    pts = np.array(
        [
            [0.0, 0.0],
            [1.0, 0.0],
            [1.0 + math.cos(a),  math.sin(a)],
            [2.0 + math.cos(a),  math.sin(a) - math.sin(a)],  # back toward x-axis
            [3.0 + math.cos(a),  math.sin(a) - math.sin(a)],  # straight on
        ]
    )
    angles = compute_turn_angles_deg(pts)
    # All angles ≤ 30° → all small → free.
    assert all(angle <= 30.0 + 1e-9 for angle in angles)
    cost = turn_penalty_cost(
        angles, DEFAULT_PENALTY_KM, DEFAULT_THRESHOLD_DEG, DEFAULT_FACTOR
    )
    assert cost == 0.0
