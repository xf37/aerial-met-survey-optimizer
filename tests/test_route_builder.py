"""Unit tests for multi_target_planner.route_builder + cost evaluator."""

from __future__ import annotations

import math

import numpy as np
import pytest
from shapely.geometry import Polygon

from multi_target_planner.chord_layout import (
    ChordSet,
    enumerate_chord_sets,
    fit_tpv_geometry,
)
from multi_target_planner.data_loader import TPV
from multi_target_planner.route_builder import (
    DEFAULT_ATC_FACTOR,
    DEFAULT_TURN_FACTOR,
    DEFAULT_TURN_PENALTY_KM,
    DEFAULT_TURN_THRESHOLD_DEG,
    build_route,
    compute_route_cost,
    make_gatepoint_stop,
    make_tpv_stop,
)


# ---------------------------------------------------------------------------
# compute_route_cost
# ---------------------------------------------------------------------------

def test_compute_route_cost_geometric_only():
    pts = np.array([[0.0, 0.0], [100.0, 0.0], [100.0, 100.0]], dtype=np.float64)
    cb = compute_route_cost(
        pts,
        atc_union=None,
        turn_penalty_km=DEFAULT_TURN_PENALTY_KM,
        turn_threshold_deg=DEFAULT_TURN_THRESHOLD_DEG,
        turn_factor=DEFAULT_TURN_FACTOR,
        atc_factor=DEFAULT_ATC_FACTOR,
    )
    # Distances 100 + 100 = 200 km; one 90-degree turn -> 1 sharp turn.
    assert math.isclose(cb.geometric_km, 200.0, rel_tol=1e-9)
    assert cb.atc_extra_km == 0.0
    assert cb.n_sharp_turns == 1
    assert math.isclose(
        cb.turn_penalty_km,
        DEFAULT_TURN_PENALTY_KM * DEFAULT_TURN_FACTOR,
        rel_tol=1e-9,
    )
    assert math.isclose(
        cb.total_km, 200.0 + DEFAULT_TURN_PENALTY_KM * DEFAULT_TURN_FACTOR, rel_tol=1e-9
    )


def test_compute_route_cost_atc_penalty():
    # ATC covers x in [40, 60], y arbitrary. Polyline is a single 100 km segment
    # along the x-axis; 20 km lies inside.
    atc = Polygon([[40, -10], [60, -10], [60, 10], [40, 10]])
    pts = np.array([[0.0, 0.0], [100.0, 0.0]], dtype=np.float64)
    cb = compute_route_cost(
        pts,
        atc_union=atc,
        turn_penalty_km=DEFAULT_TURN_PENALTY_KM,
        turn_threshold_deg=DEFAULT_TURN_THRESHOLD_DEG,
        turn_factor=DEFAULT_TURN_FACTOR,
        atc_factor=DEFAULT_ATC_FACTOR,
    )
    assert math.isclose(cb.geometric_km, 100.0, rel_tol=1e-9)
    # 20 km inside ATC * (1.35 - 1.0) = 7.0 km extra.
    assert math.isclose(cb.atc_extra_km, 20.0 * 0.35, rel_tol=1e-9)
    assert math.isclose(cb.total_km, 100.0 + 7.0, rel_tol=1e-9)


def test_compute_route_cost_empty_polyline():
    cb = compute_route_cost(np.empty((0, 2)), atc_union=None)
    assert cb.total_km == 0.0
    assert cb.n_total_turns == 0
    assert cb.n_sharp_turns == 0


# ---------------------------------------------------------------------------
# build_route — legacy gatepoints/tpv_visit path
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


def test_build_route_legacy_returns_polyline_with_base_endpoints():
    tpv = _square_tpv("sq", 0.0, 0.0)
    geom = fit_tpv_geometry(tpv.label, tpv.vertices_km)
    sets = enumerate_chord_sets(
        tpv_index=0,
        tpv_polygon_km=tpv.polygon_km,
        geom=geom,
        n_chord_options=(1,),
        angle_devs_deg=(0.0,),
        min_spacing_km=100.0,
    )
    assert sets, "Setup invariant — need at least one ChordSet"
    base = np.array([1500.0, 0.0], dtype=np.float64)
    route = build_route(
        base=base,
        gatepoints_km=np.empty((0, 2)),
        tpv_visit=[(0, sets[0])],
    )
    assert route is not None
    pts = route.polyline
    assert pts.shape[1] == 2
    # First and last vertices are BASE.
    assert np.allclose(pts[0], base)
    assert np.allclose(pts[-1], base)


def test_build_route_stops_path_with_gatepoint_interleaved():
    tpv = _square_tpv("sq", 0.0, 0.0)
    geom = fit_tpv_geometry(tpv.label, tpv.vertices_km)
    sets = enumerate_chord_sets(
        tpv_index=0,
        tpv_polygon_km=tpv.polygon_km,
        geom=geom,
        n_chord_options=(1,),
        angle_devs_deg=(0.0,),
        min_spacing_km=100.0,
    )
    base = np.array([1500.0, 0.0], dtype=np.float64)
    gp = np.array([200.0, 0.0], dtype=np.float64)
    stops = [make_gatepoint_stop(gp), make_tpv_stop(0, sets[0])]
    route = build_route(base=base, stops=stops)
    assert route is not None
    pts = route.polyline
    # The gatepoint should appear somewhere on the polyline.
    d = np.linalg.norm(pts - gp, axis=1)
    assert d.min() < 0.5


def test_build_route_returns_none_when_route_boxed_by_restricted():
    # A single restricted polygon enclosing BASE completely -> no path out.
    base = np.array([0.0, 0.0], dtype=np.float64)
    enclosed = Polygon(
        [[-1, -1], [1, -1], [1, 1], [-1, 1]]
    ).buffer(0)
    # BASE itself is inside `enclosed`, so any departure crosses it.
    # build_route will get None from _transit_polyline.
    target = np.array([10.0, 0.0], dtype=np.float64)
    stops = [make_gatepoint_stop(target)]
    route = build_route(base=base, stops=stops, restricted_union=enclosed)
    assert route is None


def test_build_route_seg_kinds_match_polyline_length():
    tpv = _square_tpv("sq", 0.0, 0.0)
    geom = fit_tpv_geometry(tpv.label, tpv.vertices_km)
    sets = enumerate_chord_sets(
        tpv_index=0,
        tpv_polygon_km=tpv.polygon_km,
        geom=geom,
        n_chord_options=(2,),
        angle_devs_deg=(0.0,),
        min_spacing_km=100.0,
    )
    if not sets:
        pytest.skip("n=2 not feasible at 100 km spacing inside this square")
    base = np.array([1500.0, 0.0], dtype=np.float64)
    route = build_route(base=base, gatepoints_km=np.empty((0, 2)), tpv_visit=[(0, sets[0])])
    assert route is not None
    # Each segment has a kind.
    assert len(route.seg_kinds) == route.polyline.shape[0] - 1
    # At least one chord and at least two transits (out + return).
    assert "chord" in route.seg_kinds
    assert route.seg_kinds.count("transit") >= 2
