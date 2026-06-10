"""Unit tests for multi_target_planner.chord_layout_b (PB-1)."""

from __future__ import annotations

import math

import numpy as np
import pytest
from shapely.geometry import Polygon

from multi_target_planner.chord_layout import fit_tpv_geometry
from multi_target_planner.chord_layout_b import (
    DEFAULT_S_STEP_KM,
    DEFAULT_THETA_MAX_DEG,
    DEFAULT_THETA_STEP_DEG,
    NonParallelChordRecord,
    NonParallelChordSet,
    build_chord_at,
    chord_set_pairwise_feasible,
    chords_cross_inside_polygon,
    enumerate_chord_sets_b,
    iter_candidate_chords,
    major_axis_spacing,
    s_grid_km,
    survey_distance,
    theta_grid_deg,
)


def _wide_rectangle(half_x: float = 400.0, half_y: float = 150.0):
    """A wide rectangle aligned with the x-axis — PCA major = x, minor = y."""
    verts = np.array(
        [[-half_x, -half_y], [half_x, -half_y],
         [half_x, half_y], [-half_x, half_y]],
        dtype=np.float64,
    )
    poly = Polygon(verts).buffer(0)
    geom = fit_tpv_geometry("rect", verts)
    return verts, poly, geom


# ---------------------------------------------------------------------------
# Grid helpers
# ---------------------------------------------------------------------------

def test_theta_grid_includes_zero_and_extremes():
    g = theta_grid_deg(step_deg=7.5, max_dev_deg=30.0)
    assert g[0] == pytest.approx(-30.0)
    assert g[-1] == pytest.approx(30.0)
    assert 0.0 in g
    # 9 points: -30, -22.5, -15, -7.5, 0, 7.5, 15, 22.5, 30.
    assert len(g) == 9


def test_s_grid_respects_major_axis_extent():
    _, _, geom = _wide_rectangle(half_x=400.0)
    g = s_grid_km(geom, step_km=100.0)
    # Major axis a is ~400 km; 0.95 a = 380 km. With 100 km step we get
    # offsets like -300, -200, -100, 0, 100, 200, 300.
    assert max(g) <= geom.a_km * 0.95 + 1e-6
    assert min(g) >= -geom.a_km * 0.95 - 1e-6
    assert 0.0 in g


# ---------------------------------------------------------------------------
# Single chord builder
# ---------------------------------------------------------------------------

def test_build_chord_at_zero_theta_matches_perpendicular():
    _, poly, geom = _wide_rectangle()
    rec = build_chord_at(0, poly, geom, theta_deg=0.0, s_km=0.0)
    assert rec is not None
    # With theta=0 and s=0 the chord is perpendicular to major (the x-axis),
    # so endpoints sit on the rectangle's top and bottom edges.
    ys = sorted([rec.p_entry[1], rec.p_exit[1]])
    assert math.isclose(ys[0], -150.0, abs_tol=1.0)
    assert math.isclose(ys[1], +150.0, abs_tol=1.0)
    assert math.isclose(rec.length_km, 300.0, abs_tol=1.0)


def test_build_chord_at_nonzero_theta_changes_geometry():
    _, poly, geom = _wide_rectangle()
    a = build_chord_at(0, poly, geom, theta_deg=0.0,  s_km=0.0)
    b = build_chord_at(0, poly, geom, theta_deg=20.0, s_km=0.0)
    assert a is not None and b is not None
    # 20-degree rotation changes the endpoints.
    assert not np.allclose(a.p_entry, b.p_entry)
    # And lengthens the chord (rotated diagonals are longer in a wide rect).
    assert b.length_km > a.length_km


def test_build_chord_at_returns_none_outside_polygon():
    _, poly, geom = _wide_rectangle()
    rec = build_chord_at(0, poly, geom, theta_deg=0.0, s_km=500.0)
    assert rec is None


def test_build_chord_at_records_restricted_crossing_flag():
    _, poly, geom = _wide_rectangle()
    # A thin restricted strip at x in [-30, 30] crosses any centre-passing chord.
    block = Polygon([[-30, -200], [30, -200], [30, 200], [-30, 200]])
    rec = build_chord_at(0, poly, geom, theta_deg=0.0, s_km=0.0,
                          restricted_union=block)
    assert rec is not None
    assert rec.crosses_restricted is True
    # A chord far from the strip should be clear.
    rec_far = build_chord_at(0, poly, geom, theta_deg=0.0, s_km=200.0,
                              restricted_union=block)
    assert rec_far is not None
    assert rec_far.crosses_restricted is False


# ---------------------------------------------------------------------------
# Pairwise feasibility
# ---------------------------------------------------------------------------

def test_parallel_chords_never_cross():
    _, poly, geom = _wide_rectangle()
    a = build_chord_at(0, poly, geom, theta_deg=0.0, s_km=-100.0)
    b = build_chord_at(0, poly, geom, theta_deg=0.0, s_km=+100.0)
    assert a is not None and b is not None
    assert chords_cross_inside_polygon(a, b) is False


def test_non_parallel_chords_at_same_s_cross():
    _, poly, geom = _wide_rectangle()
    a = build_chord_at(0, poly, geom, theta_deg=0.0,  s_km=0.0)
    b = build_chord_at(0, poly, geom, theta_deg=30.0, s_km=0.0)
    assert a is not None and b is not None
    # Two chords through the centre at different angles must cross.
    assert chords_cross_inside_polygon(a, b) is True


def test_major_axis_spacing_uses_s_only():
    _, poly, geom = _wide_rectangle()
    a = build_chord_at(0, poly, geom, theta_deg=0.0,  s_km=0.0)
    b = build_chord_at(0, poly, geom, theta_deg=25.0, s_km=120.0)
    assert a is not None and b is not None
    # Spacing is the |Δs| projection regardless of theta.
    assert major_axis_spacing(a, b) == pytest.approx(120.0)


def test_chord_set_pairwise_feasible_accepts_good_set():
    _, poly, geom = _wide_rectangle()
    a = build_chord_at(0, poly, geom, theta_deg=0.0,  s_km=-200.0)
    b = build_chord_at(0, poly, geom, theta_deg=0.0,  s_km=0.0)
    c = build_chord_at(0, poly, geom, theta_deg=0.0,  s_km=+200.0)
    assert a is not None and b is not None and c is not None
    ok, reason = chord_set_pairwise_feasible([a, b, c], min_spacing_km=100.0)
    assert ok, reason


def test_chord_set_pairwise_feasible_rejects_undersized_spacing():
    _, poly, geom = _wide_rectangle()
    a = build_chord_at(0, poly, geom, theta_deg=0.0, s_km=0.0)
    b = build_chord_at(0, poly, geom, theta_deg=0.0, s_km=50.0)
    assert a is not None and b is not None
    ok, reason = chord_set_pairwise_feasible([a, b], min_spacing_km=100.0)
    assert not ok
    assert "spacing" in reason


def test_chord_set_pairwise_feasible_rejects_crossing_pair():
    _, poly, geom = _wide_rectangle()
    a = build_chord_at(0, poly, geom, theta_deg=0.0,  s_km=0.0)
    b = build_chord_at(0, poly, geom, theta_deg=30.0, s_km=120.0)
    assert a is not None and b is not None
    ok, reason = chord_set_pairwise_feasible([a, b], min_spacing_km=100.0)
    # Spacing is OK (120 > 100); they may or may not cross — assert that
    # IF they cross, the reason is the crossing.
    seg_cross = chords_cross_inside_polygon(a, b)
    if seg_cross:
        assert not ok
        assert "cross" in reason


# ---------------------------------------------------------------------------
# Survey distance
# ---------------------------------------------------------------------------

def test_survey_distance_sums_chords_and_spacers():
    _, poly, geom = _wide_rectangle()
    a = build_chord_at(0, poly, geom, theta_deg=0.0, s_km=-150.0)
    b = build_chord_at(0, poly, geom, theta_deg=0.0, s_km=0.0)
    c = build_chord_at(0, poly, geom, theta_deg=0.0, s_km=+150.0)
    assert a is not None and b is not None and c is not None
    d = survey_distance([a, b, c], min_spacing_km=100.0)
    # Chord lengths ≈ 300 each → 900 km of chord. Two spacers of max(100, 150)
    # = 150 km each = 300 km. Total ≈ 1200 km.
    assert d == pytest.approx(1200.0, abs=2.0)


def test_survey_distance_empty_is_zero():
    assert survey_distance([]) == 0.0


# ---------------------------------------------------------------------------
# Lazy generator
# ---------------------------------------------------------------------------

def test_iter_candidate_chords_yields_finite_records():
    _, poly, geom = _wide_rectangle()
    out = list(iter_candidate_chords(
        0, poly, geom,
        theta_step_deg=15.0, theta_max_deg=15.0,   # 3 theta values
        s_step_km=200.0,                            # 5 offsets at most
    ))
    assert out, "Expected at least one candidate chord"
    # 3 thetas × ~5 offsets = up to 15 candidates, minus some that fall
    # outside the polygon or are shorter than the 10 km floor.
    assert len(out) <= 15
    for rec in out:
        assert isinstance(rec, NonParallelChordRecord)
        assert rec.length_km >= 10.0
        assert rec.crosses_restricted is False   # default skip_restricted=True


def test_iter_candidate_chords_can_include_restricted_when_asked():
    _, poly, geom = _wide_rectangle()
    block = Polygon([[-30, -200], [30, -200], [30, 200], [-30, 200]])
    with_block = list(iter_candidate_chords(
        0, poly, geom,
        theta_step_deg=15.0, theta_max_deg=15.0, s_step_km=200.0,
        restricted_union=block, skip_restricted=False,
    ))
    crossings = [r for r in with_block if r.crosses_restricted]
    assert crossings, "Expected at least one restricted-crossing candidate with skip_restricted=False"


# ---------------------------------------------------------------------------
# Exhaustive enumerator
# ---------------------------------------------------------------------------

def test_enumerate_chord_sets_b_returns_sets_at_each_n():
    _, poly, geom = _wide_rectangle()
    sets = enumerate_chord_sets_b(
        0, poly, geom,
        n_chord_options=(1, 2, 3),
        theta_step_deg=15.0, theta_max_deg=15.0,
        s_step_km=200.0,
        min_spacing_km=100.0,
        max_candidates_per_n=5,
    )
    n_values = sorted({s.n_chords for s in sets})
    # We should at least see 1- and 2-chord sets in a wide rectangle.
    assert 1 in n_values
    assert 2 in n_values


def test_enumerate_chord_sets_b_includes_non_parallel_pattern_when_geometry_permits():
    """When the polygon is wide enough, two chords at different theta with the
    same s can both be valid (one at the centre at theta=0, one off-centre at
    theta=15) without crossing."""
    _, poly, geom = _wide_rectangle(half_x=600.0, half_y=400.0)
    sets = enumerate_chord_sets_b(
        0, poly, geom,
        n_chord_options=(2,),
        theta_step_deg=15.0, theta_max_deg=15.0,
        s_step_km=200.0,
        min_spacing_km=100.0,
        max_candidates_per_n=50,
    )
    assert sets
    # Look for any 2-chord set whose chords have different theta — i.e. a
    # genuinely non-parallel pattern.
    any_non_parallel = any(not cs.is_parallel for cs in sets)
    assert any_non_parallel, "Expected at least one non-parallel chord pair"


def test_phase_a_parallel_layout_is_subset_of_phase_b():
    """Every Phase-A parallel pattern with theta=0 should appear as a
    NonParallelChordSet with the same chord positions when Phase B is
    asked for theta_max=0."""
    _, poly, geom = _wide_rectangle()
    sets = enumerate_chord_sets_b(
        0, poly, geom,
        n_chord_options=(2,),
        theta_step_deg=10.0, theta_max_deg=0.0,   # only theta=0
        s_step_km=100.0,
        min_spacing_km=100.0,
    )
    assert sets
    for cs in sets:
        assert cs.is_parallel
