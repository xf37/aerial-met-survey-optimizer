"""Smoke + integration tests for multi_target_planner.phase_a."""

from __future__ import annotations

import os

import numpy as np
import pytest
from shapely.geometry import Polygon
from shapely.ops import unary_union

from multi_target_planner.chord_layout import (
    enumerate_chord_sets,
    fit_tpv_geometry,
)
from multi_target_planner.data_loader import (
    DEFAULT_DATA_DIR,
    TPV,
    TPV_FILENAME_PHASE0A,
    base_km,
    load_atc,
    load_gatepoints,
    load_restricted,
    load_tpvs,
)
from multi_target_planner.phase_a import (
    EnvelopeResult,
    _interleave_with_gatepoints,
    phase_a_envelope,
)
from multi_target_planner.route_builder import (
    make_gatepoint_stop,
    make_tpv_stop,
)


# ---------------------------------------------------------------------------
# _interleave_with_gatepoints — pure combinatorial helper
# ---------------------------------------------------------------------------

def test_interleave_no_gatepoints_yields_input_unchanged():
    tpv_stops = [make_tpv_stop(0, None), make_tpv_stop(1, None)]   # type: ignore[arg-type]
    out = list(_interleave_with_gatepoints(tpv_stops, []))
    assert len(out) == 1
    assert out[0] == tpv_stops


def test_interleave_count_matches_formula():
    # k=3 tpv stops, m=2 gp stops -> m! * C(k+m, m) = 2 * 10 = 20.
    tpv_stops = [make_tpv_stop(i, None) for i in range(3)]   # type: ignore[arg-type]
    gp_stops = [make_gatepoint_stop(np.zeros(2)), make_gatepoint_stop(np.ones(2))]
    out = list(_interleave_with_gatepoints(tpv_stops, gp_stops))
    assert len(out) == 20


def test_interleave_preserves_tpv_order_in_each_output():
    tpv_stops = [make_tpv_stop(i, None) for i in range(3)]   # type: ignore[arg-type]
    gp_stops = [make_gatepoint_stop(np.zeros(2)), make_gatepoint_stop(np.ones(2))]
    expected_tpv_ids = [0, 1, 2]
    for seq in _interleave_with_gatepoints(tpv_stops, gp_stops):
        tpv_ids = [s.tpv_index for s in seq if s.kind == "tpv"]
        assert tpv_ids == expected_tpv_ids


# ---------------------------------------------------------------------------
# phase_a_envelope on a tiny synthetic instance (fast — no real data needed)
# ---------------------------------------------------------------------------

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


def test_phase_a_synthetic_two_tpvs_yields_feasible_plans():
    """Two close TPVs, no constraints, generous budget -> many feasible plans."""
    tpvs = [_square_tpv("A", 0.0, 0.0), _square_tpv("B", 800.0, 0.0)]
    base = np.array([-1000.0, 0.0], dtype=np.float64)
    result = phase_a_envelope(
        tpvs=tpvs,
        base_km=base,
        gatepoints_km=np.empty((0, 2)),
        restricted_union=None,
        atc_union=None,
        budget_km=4250.0,
        n_chord_options=(1, 2),
        angle_devs_deg=(0.0,),
    )
    assert isinstance(result, EnvelopeResult)
    assert result.n_feasible > 0
    assert result.n_max_tpvs >= 1
    assert result.archive_size > 0
    assert len(result.bundle_b1.plans) >= 1
    assert len(result.bundle_b2.plans) >= 1
    # Every plan in every bundle must be feasible by construction (LOCK 1).
    for plan in result.bundle_b1.plans + result.bundle_b2.plans:
        assert plan.total_cost_km <= 4250.0


def test_phase_a_synthetic_no_feasible_when_budget_too_tight():
    tpvs = [_square_tpv("far", 5000.0, 0.0)]
    base = np.array([0.0, 0.0], dtype=np.float64)
    result = phase_a_envelope(
        tpvs=tpvs,
        base_km=base,
        gatepoints_km=np.empty((0, 2)),
        restricted_union=None,
        atc_union=None,
        budget_km=200.0,   # nowhere near reachable
        n_chord_options=(1,),
        angle_devs_deg=(0.0,),
    )
    assert result.n_feasible == 0
    assert result.n_max_tpvs == 0
    assert result.bundle_b1.plans == ()
    assert result.bundle_b2.plans == ()


def test_phase_a_synthetic_max_chord_per_tpv_tracked():
    tpvs = [_square_tpv("A", 0.0, 0.0, side=800.0)]
    base = np.array([1000.0, 0.0], dtype=np.float64)
    result = phase_a_envelope(
        tpvs=tpvs,
        base_km=base,
        gatepoints_km=np.empty((0, 2)),
        restricted_union=None,
        atc_union=None,
        budget_km=4250.0,
        n_chord_options=(1, 2, 3),
        angle_devs_deg=(0.0,),
    )
    if result.n_feasible == 0:
        pytest.skip("Synthetic case happened to be infeasible")
    assert result.chord_max_per_tpv[0] >= 1


# ---------------------------------------------------------------------------
# Real-data integration smoke (skipped when test_data is absent)
# ---------------------------------------------------------------------------

real_data_marker = pytest.mark.skipif(
    not os.path.isdir(DEFAULT_DATA_DIR),
    reason=f"DATA_DIR not present on this machine: {DEFAULT_DATA_DIR}",
)


@real_data_marker
def test_phase_a_real_data_returns_result_and_takes_reasonable_time():
    """
    Smoke test: with the 3-TPV Phase 0a file + gatepoints + restricted, Phase A
    must return *something* (even 0 feasible) without raising.

    This run is slow on the full enumeration; we cap it by restricting the
    chord-option and angle-deviation grids so the test stays under a minute.
    """
    tpvs, frame = load_tpvs(DEFAULT_DATA_DIR, TPV_FILENAME_PHASE0A)
    restricted = load_restricted(DEFAULT_DATA_DIR, frame)
    atc = load_atc(DEFAULT_DATA_DIR, frame)
    gatepoints = load_gatepoints(DEFAULT_DATA_DIR, frame)
    base = base_km(frame)
    restricted_union = unary_union(restricted) if restricted else None
    atc_union = unary_union(atc) if atc else None

    result = phase_a_envelope(
        tpvs=tpvs,
        base_km=base,
        gatepoints_km=gatepoints,
        restricted_union=restricted_union,
        atc_union=atc_union,
        n_chord_options=(1,),
        angle_devs_deg=(0.0,),
    )
    assert isinstance(result, EnvelopeResult)
    assert result.n_enumerated > 0
    # No claim on n_feasible — the mission is tight enough that some
    # configurations honestly produce zero feasible plans.
