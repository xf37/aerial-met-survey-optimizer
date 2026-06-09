"""Unit + smoke tests for multi_target_planner.verifier."""

from __future__ import annotations

import os

import numpy as np
import pytest
from shapely.geometry import Polygon

from multi_target_planner.chord_layout import (
    enumerate_chord_sets,
    fit_tpv_geometry,
)
from multi_target_planner.data_loader import (
    DEFAULT_DATA_DIR,
    TPV,
    TPV_FILENAME_PHASE0A,
)
from multi_target_planner.pareto import Plan
from multi_target_planner.route_builder import (
    build_route,
    make_tpv_stop,
)
from multi_target_planner.sat_arc import (
    DEFAULT_SAT_MIN_LENGTH_KM,
    SatArcCandidate,
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


def test_verify_plan_returns_independent_cost(mission_state):
    """A synthetic plan that obviously violates budget gets independent_cost > budget."""
    tpv = _square_tpv("sq", mission_state.base[0] + 5000.0, 0.0)
    geom = fit_tpv_geometry(tpv.label, tpv.vertices_km)
    sets = enumerate_chord_sets(
        tpv_index=0, tpv_polygon_km=tpv.polygon_km, geom=geom,
        n_chord_options=(1,), angle_devs_deg=(0.0,), min_spacing_km=100.0,
    )
    if not sets:
        pytest.skip("chord enumeration empty for synthetic TPV")
    route = build_route(
        base=mission_state.base,
        stops=[make_tpv_stop(0, sets[0])],
    )
    plan = Plan(tpv_indices=(0,), chord_choices=(sets[0],), route=route)
    report = verify_plan(plan, state=mission_state, budget_km=100.0)
    # Cost was much larger than 100 km -> budget violation.
    assert isinstance(report, VerifierReport)
    assert not report.ok
    assert any(v.startswith("budget") for v in report.violations)
    assert report.independent_cost_km > 100.0
