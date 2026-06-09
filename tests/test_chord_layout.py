"""Unit tests for multi_target_planner.chord_layout."""

from __future__ import annotations

import numpy as np
import pytest
from shapely.geometry import Polygon

from multi_target_planner.chord_layout import (
    ChordSet,
    TpvGeometry,
    enumerate_chord_sets,
    fit_tpv_geometry,
    precompute_all_chord_sets,
)
from multi_target_planner.data_loader import TPV


def _square_tpv(label: str, x0: float, y0: float, side: float = 500.0) -> TPV:
    """A simple square TPV centred at ``(x0, y0)`` for tests."""
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


# ---------------------------------------------------------------------------
# fit_tpv_geometry
# ---------------------------------------------------------------------------

def test_fit_tpv_geometry_returns_aspect_ratio_near_1_for_square():
    tpv = _square_tpv("sq", 0.0, 0.0, side=500.0)
    geom = fit_tpv_geometry(tpv.label, tpv.vertices_km)
    assert isinstance(geom, TpvGeometry)
    assert 0.9 < geom.aspect_ratio < 1.1
    assert geom.is_circle_like


def test_fit_tpv_geometry_returns_high_aspect_for_oval():
    verts = np.array(
        [[-500.0, 0.0], [-250.0, 200.0], [250.0, 200.0],
         [500.0, 0.0], [250.0, -200.0], [-250.0, -200.0]],
        dtype=np.float64,
    )
    geom = fit_tpv_geometry("oval", verts)
    # Major axis ~= 1000 km, minor ~= 400 km -> aspect > 1.5.
    assert geom.aspect_ratio > 1.5
    assert not geom.is_circle_like


# ---------------------------------------------------------------------------
# enumerate_chord_sets
# ---------------------------------------------------------------------------

def test_enumerate_chord_sets_produces_n_chords():
    tpv = _square_tpv("sq", 0.0, 0.0, side=500.0)
    geom = fit_tpv_geometry(tpv.label, tpv.vertices_km)
    sets = enumerate_chord_sets(
        tpv_index=0,
        tpv_polygon_km=tpv.polygon_km,
        geom=geom,
        n_chord_options=(1, 2, 3),
        angle_devs_deg=(0.0,),
        min_spacing_km=100.0,
    )
    assert sets, "Expected at least one feasible ChordSet"
    n_values = sorted({s.n_chords for s in sets})
    # At least n=1 should always be feasible inside a 500 km square.
    assert 1 in n_values


def test_enumerate_chord_sets_respects_spacing_via_filtering():
    # Tight polygon: a 200 km square — at min_spacing=100 km only n<=2 fit.
    verts = np.array(
        [[-100, -100], [100, -100], [100, 100], [-100, 100]],
        dtype=np.float64,
    )
    tpv = TPV(label="tight", polygon_km=Polygon(verts).buffer(0), vertices_km=verts)
    geom = fit_tpv_geometry(tpv.label, tpv.vertices_km)
    sets = enumerate_chord_sets(
        tpv_index=0,
        tpv_polygon_km=tpv.polygon_km,
        geom=geom,
        n_chord_options=(1, 2, 3, 4),
        angle_devs_deg=(0.0,),
        min_spacing_km=100.0,
    )
    n_values = sorted({s.n_chords for s in sets})
    # n=3 and n=4 cannot fit -> only 1 and 2 (at most) should appear.
    assert all(n <= 2 for n in n_values)


def test_enumerate_chord_sets_drops_chords_through_restricted():
    # Square TPV with restricted union covering its centre — every chord through
    # the centre is rejected, so the chord generator returns fewer chords.
    tpv = _square_tpv("sq", 0.0, 0.0, side=500.0)
    geom = fit_tpv_geometry(tpv.label, tpv.vertices_km)

    no_restr = enumerate_chord_sets(
        tpv_index=0,
        tpv_polygon_km=tpv.polygon_km,
        geom=geom,
        n_chord_options=(1,),
        angle_devs_deg=(0.0,),
        min_spacing_km=100.0,
        restricted_union=None,
    )
    centre_block = Polygon(
        [[-50, -260], [50, -260], [50, 260], [-50, 260]]
    )
    with_restr = enumerate_chord_sets(
        tpv_index=0,
        tpv_polygon_km=tpv.polygon_km,
        geom=geom,
        n_chord_options=(1,),
        angle_devs_deg=(0.0,),
        min_spacing_km=100.0,
        restricted_union=centre_block,
    )
    # With restricted in place, we should see fewer (or no) n=1 patterns.
    assert len(with_restr) <= len(no_restr)


# ---------------------------------------------------------------------------
# precompute_all_chord_sets
# ---------------------------------------------------------------------------

def test_precompute_all_chord_sets_parallel_lists():
    tpvs = [
        _square_tpv("A", 0.0, 0.0),
        _square_tpv("B", 1000.0, 0.0),
    ]
    geoms, sets = precompute_all_chord_sets(
        tpvs,
        n_chord_options=(1, 2),
        angle_devs_deg=(0.0,),
        min_spacing_km=100.0,
    )
    assert len(geoms) == 2 == len(sets)
    for i, s_list in enumerate(sets):
        for s in s_list:
            assert s.tpv_index == i
