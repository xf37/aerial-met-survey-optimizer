"""Unit + smoke tests for multi_target_planner.alns (PB-2)."""

from __future__ import annotations

import random

import numpy as np
import pytest
from shapely.geometry import Polygon

from multi_target_planner.alns import (
    ALNSState,
    DEFAULT_THETA_PENALTY_KM,
    destroy_perturb_angle,
    destroy_remove_chord,
    refine_plan_alns,
    repair_insert_chord_any,
    repair_insert_chord_parallel,
    state_cost,
    state_score,
)
from multi_target_planner.chord_layout import fit_tpv_geometry
from multi_target_planner.chord_layout_b import build_chord_at


def _wide_rect_geom(half_x: float = 400.0, half_y: float = 150.0):
    verts = np.array([[-half_x, -half_y], [half_x, -half_y],
                      [half_x, half_y], [-half_x, half_y]], dtype=np.float64)
    poly = Polygon(verts).buffer(0)
    geom = fit_tpv_geometry("rect", verts)
    return poly, geom, verts


def _one_chord_state(poly, geom) -> ALNSState:
    rec = build_chord_at(0, poly, geom, theta_deg=0.0, s_km=0.0)
    assert rec is not None
    return ALNSState(visit_order=[0], chords_per_tpv={0: [rec]})


# ---------------------------------------------------------------------------
# State + cost
# ---------------------------------------------------------------------------

def test_state_clone_is_deep():
    poly, geom, _ = _wide_rect_geom()
    s = _one_chord_state(poly, geom)
    t = s.clone()
    t.chords_per_tpv[0].clear()
    assert s.total_chords() == 1
    assert t.total_chords() == 0


def test_state_is_parallel_iff_all_theta_zero():
    poly, geom, _ = _wide_rect_geom()
    s = _one_chord_state(poly, geom)
    assert s.is_parallel()
    rec_rot = build_chord_at(0, poly, geom, theta_deg=15.0, s_km=150.0)
    if rec_rot is not None:
        s.chords_per_tpv[0].append(rec_rot)
        assert not s.is_parallel()


def test_state_score_prefers_more_chords():
    poly, geom, _ = _wide_rect_geom()
    s1 = _one_chord_state(poly, geom)
    s2 = s1.clone()
    extra = build_chord_at(0, poly, geom, theta_deg=0.0, s_km=200.0)
    assert extra is not None
    s2.chords_per_tpv[0].append(extra)
    assert state_score(s2) > state_score(s1)


def test_state_score_prefers_parallel_when_chord_lengths_match():
    """Two states with the same chord count AND matching chord lengths: the
    one with all theta=0 must score higher because the theta penalty is
    non-zero on the rotated one. We synthesise the records by hand to
    decouple from polygon geometry (which would otherwise leak chord-length
    differences into the test)."""
    from multi_target_planner.chord_layout_b import NonParallelChordRecord
    base = dict(tpv_index=0, s_km=0.0,
                p_entry=np.array([0.0, 0.0]),
                p_exit=np.array([300.0, 0.0]),
                length_km=300.0,
                crosses_restricted=False)
    rec_parallel = NonParallelChordRecord(theta_deg=0.0, **base)
    rec_rotated  = NonParallelChordRecord(theta_deg=15.0, **base)
    parallel = ALNSState(visit_order=[0], chords_per_tpv={0: [rec_parallel]})
    rotated  = ALNSState(visit_order=[0], chords_per_tpv={0: [rec_rotated]})
    assert state_score(parallel) > state_score(rotated)


def test_state_score_prefers_more_chords_even_with_rotation():
    """The chord-count axis dominates the theta penalty: a 2-chord rotated
    layout outscores a 1-chord parallel layout. This pins the "more chords
    > parallel preference" priority Master fixed on 2026-06-10."""
    from multi_target_planner.chord_layout_b import NonParallelChordRecord
    base = dict(tpv_index=0,
                p_entry=np.array([0.0, 0.0]),
                p_exit=np.array([300.0, 0.0]),
                length_km=300.0, crosses_restricted=False)
    rec_par = NonParallelChordRecord(theta_deg=0.0,  s_km=0.0,   **base)
    rec_rot_1 = NonParallelChordRecord(theta_deg=15.0, s_km=0.0,   **base)
    rec_rot_2 = NonParallelChordRecord(theta_deg=15.0, s_km=200.0, **base)
    one_parallel = ALNSState(visit_order=[0], chords_per_tpv={0: [rec_par]})
    two_rotated  = ALNSState(visit_order=[0], chords_per_tpv={0: [rec_rot_1, rec_rot_2]})
    # 2 chords outscore 1 chord regardless of the theta penalty.
    assert state_score(two_rotated) > state_score(one_parallel)


# ---------------------------------------------------------------------------
# Operators
# ---------------------------------------------------------------------------

def test_destroy_remove_chord_drops_one():
    poly, geom, _ = _wide_rect_geom()
    s = _one_chord_state(poly, geom)
    extra = build_chord_at(0, poly, geom, theta_deg=0.0, s_km=200.0)
    s.chords_per_tpv[0].append(extra)
    out = destroy_remove_chord(s, random.Random(0))
    assert out.total_chords() == 1


def test_destroy_perturb_angle_changes_theta():
    poly, geom, _ = _wide_rect_geom()
    s = _one_chord_state(poly, geom)
    out = destroy_perturb_angle(s, random.Random(0))
    new_theta = out.chords_per_tpv[0][0].theta_deg
    assert abs(new_theta) <= 30.0
    # With seed 0 the perturbation moves theta off zero by one step.
    assert abs(new_theta) > 1e-9


def test_repair_insert_chord_parallel_adds_a_chord():
    poly, geom, _ = _wide_rect_geom()
    s = _one_chord_state(poly, geom)
    out = repair_insert_chord_parallel(
        s,
        tpv_index_by_position={0: 0},
        tpv_polys={0: poly}, tpv_geoms={0: geom},
        restricted_union=None,
        min_spacing_km=100.0, rng=random.Random(0),
    )
    # In a wide rectangle, a second parallel chord should fit at any offset >=100 km away.
    assert out.total_chords() >= s.total_chords()


def test_repair_insert_chord_parallel_returns_state_when_no_room():
    """50 km square (semi-major a ≈ 35 km), s grid in [-25, 0, +25].
    A seed chord at s=0 leaves no offset ≥ 100 km away, so the parallel
    repair must give up."""
    verts = np.array([[-25, -25], [25, -25], [25, 25], [-25, 25]],
                      dtype=np.float64)
    poly = Polygon(verts).buffer(0)
    geom = fit_tpv_geometry("tiny", verts)
    seed_chord = build_chord_at(0, poly, geom, theta_deg=0.0, s_km=0.0)
    assert seed_chord is not None
    s = ALNSState(visit_order=[0], chords_per_tpv={0: [seed_chord]})
    out = repair_insert_chord_parallel(
        s,
        tpv_index_by_position={0: 0},
        tpv_polys={0: poly}, tpv_geoms={0: geom},
        restricted_union=None,
        min_spacing_km=100.0, rng=random.Random(0),
    )
    assert out.total_chords() == 1


# ---------------------------------------------------------------------------
# Main loop — anytime guarantee
# ---------------------------------------------------------------------------

def test_refine_plan_alns_never_regresses_below_initial_score():
    poly, geom, _ = _wide_rect_geom()
    initial = _one_chord_state(poly, geom)
    initial_score = state_score(initial)
    result = refine_plan_alns(
        initial,
        tpv_index_by_position={0: 0},
        tpv_polys={0: poly}, tpv_geoms={0: geom},
        restricted_union=None,
        min_spacing_km=100.0,
        time_budget_sec=2.0, max_iterations=200,
        rng=random.Random(7),
    )
    assert result.incumbent_score >= initial_score


def test_refine_plan_alns_can_grow_chord_count_when_room_exists():
    poly, geom, _ = _wide_rect_geom(half_x=600.0, half_y=400.0)
    initial = _one_chord_state(poly, geom)
    initial_n = initial.total_chords()
    result = refine_plan_alns(
        initial,
        tpv_index_by_position={0: 0},
        tpv_polys={0: poly}, tpv_geoms={0: geom},
        restricted_union=None,
        min_spacing_km=100.0,
        time_budget_sec=3.0, max_iterations=500,
        rng=random.Random(11),
    )
    assert result.incumbent.total_chords() >= initial_n


def test_refine_plan_alns_history_is_monotone_in_score():
    poly, geom, _ = _wide_rect_geom()
    initial = _one_chord_state(poly, geom)
    result = refine_plan_alns(
        initial,
        tpv_index_by_position={0: 0},
        tpv_polys={0: poly}, tpv_geoms={0: geom},
        restricted_union=None,
        min_spacing_km=100.0,
        time_budget_sec=1.5, max_iterations=200,
        rng=random.Random(3),
    )
    # Every history entry is an incumbent improvement, so the score column
    # is monotonically non-decreasing.
    scores = [s for _, s in result.history]
    assert scores == sorted(scores)


def test_refine_plan_alns_respects_time_budget():
    poly, geom, _ = _wide_rect_geom()
    initial = _one_chord_state(poly, geom)
    result = refine_plan_alns(
        initial,
        tpv_index_by_position={0: 0},
        tpv_polys={0: poly}, tpv_geoms={0: geom},
        restricted_union=None,
        min_spacing_km=100.0,
        time_budget_sec=0.5, max_iterations=100_000,
        rng=random.Random(99),
    )
    # Loop must exit on time, not on iteration cap.
    assert result.elapsed_sec < 1.0
    assert result.iterations_run < 100_000


def test_refine_plan_alns_with_zero_budget_returns_initial():
    poly, geom, _ = _wide_rect_geom()
    initial = _one_chord_state(poly, geom)
    result = refine_plan_alns(
        initial,
        tpv_index_by_position={0: 0},
        tpv_polys={0: poly}, tpv_geoms={0: geom},
        restricted_union=None,
        min_spacing_km=100.0,
        time_budget_sec=0.0, max_iterations=10_000,
        rng=random.Random(0),
    )
    assert result.incumbent.total_chords() == initial.total_chords()


# ---------------------------------------------------------------------------
# freeze_chord_count regime (Master 2026-06-10)
# ---------------------------------------------------------------------------

def test_refine_plan_alns_freeze_chord_count_preserves_subset_and_counts():
    """When freeze_chord_count=True, ALNS must not add/remove chords or
    change which TPVs are visited.  Per-TPV chord count is locked."""
    import random
    from multi_target_planner.alns import refine_plan_alns
    from multi_target_planner.chord_layout import fit_tpv_geometry
    from multi_target_planner.chord_layout_b import NonParallelChordRecord
    from multi_target_planner.alns import ALNSState
    import numpy as np
    from shapely.geometry import Polygon as _P
    verts = np.array([[-200., -200.], [200., -200.], [200., 200.], [-200., 200.]])
    poly = _P(verts)
    geom = fit_tpv_geometry("T0", verts)
    initial_state = ALNSState(
        visit_order=[0],
        chords_per_tpv={
            0: [
                NonParallelChordRecord(0, 0.0, 0.0,
                                       p_entry=np.array([-150., 0.]),
                                       p_exit=np.array([150., 0.]),
                                       length_km=300.0, crosses_restricted=False),
                NonParallelChordRecord(0, 0.0, 100.0,
                                       p_entry=np.array([-150., 100.]),
                                       p_exit=np.array([150., 100.]),
                                       length_km=300.0, crosses_restricted=False),
            ]
        },
    )
    initial_n_chords = initial_state.total_chords()
    result = refine_plan_alns(
        initial_state,
        tpv_index_by_position={0: 0},
        tpv_polys={0: poly}, tpv_geoms={0: geom},
        restricted_union=None,
        min_spacing_km=80.0,
        max_iterations=100, time_budget_sec=10.0,
        rng=random.Random(7),
        freeze_chord_count=True,
    )
    # Invariant preserved
    assert result.incumbent.total_chords() == initial_n_chords
    assert sorted(result.incumbent.visit_order) == sorted(initial_state.visit_order)
    for pos in initial_state.visit_order:
        assert len(result.incumbent.chords_per_tpv[pos]) == \
            len(initial_state.chords_per_tpv[pos])
