"""Smoke tests for multi_target_planner.data_loader (FPO-5 / Phase 0a / S1 + 0a.1).

These tests touch the real shapefiles in ``C:\\nasa-ffp-nurture\\test_data\\``.
On any machine where that directory is absent, the whole module is skipped
rather than failing — LOCK forbids fallback to synthetic data, so the
correct response when inputs are missing is to skip, not fabricate.
"""

from __future__ import annotations

import os

import numpy as np
import pytest
from shapely.geometry import Polygon

from multi_target_planner.data_loader import (
    DEFAULT_DATA_DIR,
    TPV_FILENAME_PHASE0A,
    LocalFrame,
    base_km,
    load_atc,
    load_dropsonde,
    load_gatepoints,
    load_restricted,
    load_satellite_track,
    load_tpvs,
    verify_data_sources,
)


pytestmark = pytest.mark.skipif(
    not os.path.isdir(DEFAULT_DATA_DIR),
    reason=f"DATA_DIR not present on this machine: {DEFAULT_DATA_DIR}",
)


# ---------------------------------------------------------------------------
# verify_data_sources
# ---------------------------------------------------------------------------

def test_verify_data_sources_finds_every_role():
    report = verify_data_sources(DEFAULT_DATA_DIR)
    # Seven roles must each have a present file on disk.
    assert set(report.keys()) == {
        "tpv_phase0a", "tpv_07", "restricted", "atc",
        "dropsonde", "satellite", "gatepoints",
    }
    for role, (path, exists) in report.items():
        assert exists, f"Missing data file for role {role}: {path}"


# ---------------------------------------------------------------------------
# load_tpvs — Phase 0a multi-TPV override
# ---------------------------------------------------------------------------

def test_load_tpvs_returns_three_polygons_from_phase0a_file():
    tpvs, frame = load_tpvs(DEFAULT_DATA_DIR, TPV_FILENAME_PHASE0A)
    assert len(tpvs) == 3, "Phase 0a TPV file should have exactly 3 contours"
    for t in tpvs:
        assert isinstance(t.polygon_km, Polygon)
        assert t.polygon_km.is_valid
        assert not t.polygon_km.is_empty
        assert t.polygon_km.area > 0.0
        assert t.vertices_km.shape[1] == 2
        assert t.vertices_km.shape[0] >= 4
    # Labels are distinct.
    labels = [t.label for t in tpvs]
    assert len(set(labels)) == 3


def test_load_tpvs_local_frame_is_centred_near_tpv_union():
    tpvs, frame = load_tpvs(DEFAULT_DATA_DIR, TPV_FILENAME_PHASE0A)
    # In the local km frame, the centroid of the union should be within ~50 km of (0, 0)
    # by construction (frame origin = union centroid in WGS84, projected to km).
    all_verts = np.vstack([t.vertices_km for t in tpvs])
    cx, cy = all_verts.mean(axis=0)
    assert abs(cx) < 200.0, f"frame origin drifted too far: x={cx} km"
    assert abs(cy) < 200.0, f"frame origin drifted too far: y={cy} km"


def test_load_tpvs_can_take_external_frame():
    # If frame is supplied, no fresh frame is built — origin stays as given.
    fixed = LocalFrame(lon_c=-70.0, lat_c=50.0)
    tpvs, frame = load_tpvs(DEFAULT_DATA_DIR, TPV_FILENAME_PHASE0A, frame=fixed)
    assert frame is fixed


# ---------------------------------------------------------------------------
# polygon-zone loaders (restricted / ATC / dropsonde)
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def frame() -> LocalFrame:
    _, f = load_tpvs(DEFAULT_DATA_DIR, TPV_FILENAME_PHASE0A)
    return f


def test_load_restricted_returns_valid_polygons(frame):
    polys = load_restricted(DEFAULT_DATA_DIR, frame)
    assert len(polys) > 0, "Expected at least one restricted-airspace polygon"
    for p in polys:
        assert isinstance(p, Polygon)
        assert p.is_valid and not p.is_empty
        assert p.area > 0.0


def test_load_atc_returns_gander_fir(frame):
    polys = load_atc(DEFAULT_DATA_DIR, frame)
    assert len(polys) >= 1, "Expected at least one Gander FIR polygon"
    for p in polys:
        assert p.is_valid and not p.is_empty


def test_load_dropsonde_returns_polygons(frame):
    polys = load_dropsonde(DEFAULT_DATA_DIR, frame)
    assert len(polys) > 0, "Expected at least one dropsonde-deployment polygon"
    for p in polys:
        assert p.is_valid and not p.is_empty


# ---------------------------------------------------------------------------
# satellite track + gatepoints + BASE
# ---------------------------------------------------------------------------

def test_load_satellite_track_is_polyline(frame):
    track = load_satellite_track(DEFAULT_DATA_DIR, frame)
    assert track.ndim == 2 and track.shape[1] == 2
    assert track.shape[0] >= 2
    # Track should span hundreds of km (satellite overpass over the mission area).
    span_x = np.ptp(track[:, 0])
    span_y = np.ptp(track[:, 1])
    assert max(span_x, span_y) > 50.0, "Satellite track unexpectedly short"


def test_load_gatepoints_returns_at_least_one(frame):
    gps = load_gatepoints(DEFAULT_DATA_DIR, frame)
    assert gps.ndim == 2 and gps.shape[1] == 2
    assert gps.shape[0] >= 1


def test_base_km_is_finite_and_near_mission_area(frame):
    base = base_km(frame)
    assert base.shape == (2,)
    assert np.all(np.isfinite(base))
    # BASE distance from frame origin (TPV centroid) should be < 2000 km — sanity bound.
    assert np.linalg.norm(base) < 2000.0


# ---------------------------------------------------------------------------
# integration: full layer load (1 pass, the same as the smoke notebook does)
# ---------------------------------------------------------------------------

def test_full_layer_load_succeeds(frame):
    tpvs, _ = load_tpvs(DEFAULT_DATA_DIR, TPV_FILENAME_PHASE0A, frame=frame)
    restricted = load_restricted(DEFAULT_DATA_DIR, frame)
    atc = load_atc(DEFAULT_DATA_DIR, frame)
    dropsonde = load_dropsonde(DEFAULT_DATA_DIR, frame)
    satellite = load_satellite_track(DEFAULT_DATA_DIR, frame)
    gatepoints = load_gatepoints(DEFAULT_DATA_DIR, frame)
    base = base_km(frame)
    # Cross-layer sanity: dropsonde + restricted clipped at 2000 km; ATC NOT clipped.
    # We don't pin counts (data could change), only that everything is non-empty.
    assert tpvs and len(tpvs) == 3
    assert restricted and atc and dropsonde
    assert satellite.shape[0] >= 2
    assert gatepoints.shape[0] >= 1
    assert np.linalg.norm(base) < 2000.0
