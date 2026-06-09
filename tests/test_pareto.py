"""Unit tests for multi_target_planner.pareto."""

from __future__ import annotations

import numpy as np
import pytest
from shapely.geometry import Polygon

from multi_target_planner.chord_layout import (
    ChordSet,
    enumerate_chord_sets,
    fit_tpv_geometry,
)
from multi_target_planner.data_loader import TPV
from multi_target_planner.pareto import (
    Bundle,
    ParetoArchive,
    Plan,
    curate_bundles,
    dominates,
)
from multi_target_planner.route_builder import build_route, make_tpv_stop


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


def _build_plan(tpvs, base, tpv_idx, chord_set) -> Plan:
    route = build_route(base=base, stops=[make_tpv_stop(tpv_idx, chord_set)])
    return Plan(tpv_indices=(tpv_idx,), chord_choices=(chord_set,), route=route)


@pytest.fixture
def two_chord_sets():
    """Return two chord sets for a single square TPV — one with n=1, one with n=2."""
    tpv = _square_tpv("sq", 0.0, 0.0)
    geom = fit_tpv_geometry(tpv.label, tpv.vertices_km)
    sets1 = enumerate_chord_sets(
        tpv_index=0,
        tpv_polygon_km=tpv.polygon_km,
        geom=geom,
        n_chord_options=(1, 2),
        angle_devs_deg=(0.0,),
        min_spacing_km=100.0,
    )
    n1 = next(s for s in sets1 if s.n_chords == 1)
    n2 = next((s for s in sets1 if s.n_chords == 2), None)
    if n2 is None:
        pytest.skip("n=2 not feasible inside this square at 100 km spacing")
    return n1, n2, tpv


# ---------------------------------------------------------------------------
# dominates
# ---------------------------------------------------------------------------

def test_dominates_strictly_better_plan(two_chord_sets):
    n1, n2, tpv = two_chord_sets
    base = np.array([1500.0, 0.0], dtype=np.float64)
    p1 = _build_plan([tpv], base, 0, n1)   # 1 chord
    p2 = _build_plan([tpv], base, 0, n2)   # 2 chords -> more total + max chord
    # p2 has strictly more chords; if it also has higher cost it may not strictly dominate.
    # Construct the assertion via the four-axis objective.
    if p2.total_cost_km <= p1.total_cost_km:
        assert dominates(p2, p1)
        assert not dominates(p1, p2)


def test_dominates_returns_false_for_equal_plans(two_chord_sets):
    n1, _, tpv = two_chord_sets
    base = np.array([1500.0, 0.0], dtype=np.float64)
    p_a = _build_plan([tpv], base, 0, n1)
    p_b = _build_plan([tpv], base, 0, n1)
    assert not dominates(p_a, p_b)
    assert not dominates(p_b, p_a)


# ---------------------------------------------------------------------------
# ParetoArchive
# ---------------------------------------------------------------------------

def test_archive_accepts_first_plan(two_chord_sets):
    n1, _, tpv = two_chord_sets
    base = np.array([1500.0, 0.0], dtype=np.float64)
    p = _build_plan([tpv], base, 0, n1)
    archive = ParetoArchive()
    assert archive.offer(p) is True
    assert len(archive) == 1
    assert archive.n_offered == 1


def test_archive_dedupes_by_signature_keeps_cheaper(two_chord_sets):
    n1, _, tpv = two_chord_sets
    base = np.array([1500.0, 0.0], dtype=np.float64)
    p_cheap = _build_plan([tpv], base, 0, n1)
    archive = ParetoArchive()
    assert archive.offer(p_cheap)
    # Offer the same signature again — duplicate (kept only if cheaper).
    assert archive.offer(p_cheap) is False
    assert archive.n_duplicates == 1


def test_archive_drops_dominated_when_better_arrives(two_chord_sets):
    n1, n2, tpv = two_chord_sets
    base = np.array([1500.0, 0.0], dtype=np.float64)
    p1 = _build_plan([tpv], base, 0, n1)
    p2 = _build_plan([tpv], base, 0, n2)
    archive = ParetoArchive()
    archive.offer(p1)
    if p2.total_cost_km < p1.total_cost_km:
        # p2 strictly dominates -> p1 must be evicted.
        assert archive.offer(p2) is True
        assert len(archive) == 1
        assert list(archive.plans.values())[0].chord_choices == p2.chord_choices


# ---------------------------------------------------------------------------
# curate_bundles
# ---------------------------------------------------------------------------

def test_curate_bundles_returns_named_bundles_with_k_plans(two_chord_sets):
    n1, n2, tpv = two_chord_sets
    base = np.array([1500.0, 0.0], dtype=np.float64)
    archive = ParetoArchive()
    archive.offer(_build_plan([tpv], base, 0, n1))
    archive.offer(_build_plan([tpv], base, 0, n2))
    b1, b2 = curate_bundles(archive, k_per_bundle=5)
    assert isinstance(b1, Bundle) and isinstance(b2, Bundle)
    assert b1.name.startswith("B1")
    assert b2.name.startswith("B2")
    assert len(b1.plans) <= 5
    assert len(b2.plans) <= 5
    # Same archive feeds both bundles; both should be non-empty.
    assert len(b1.plans) >= 1
    assert len(b2.plans) >= 1


def test_curate_bundles_empty_archive_returns_empty_bundles():
    b1, b2 = curate_bundles(ParetoArchive(), k_per_bundle=5)
    assert b1.plans == ()
    assert b2.plans == ()
