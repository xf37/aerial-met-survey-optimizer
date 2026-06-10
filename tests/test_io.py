"""Unit tests for multi_target_planner.io (PB-0)."""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np
import pytest
from shapely.geometry import Polygon

from multi_target_planner.data_loader import TPV, LocalFrame
from multi_target_planner.io import (
    SCHEMA_VERSION,
    PhaseARun,
    build_run,
    load_phase_a_run,
    save_phase_a_run,
)
from multi_target_planner.pareto import Bundle
from multi_target_planner.phase_a import EnvelopeResult


def _tiny_envelope() -> EnvelopeResult:
    """An EnvelopeResult with empty bundles — enough for round-trip tests."""
    empty_bundle_b1 = Bundle(name="B1 (max TPVs flown)", plans=())
    empty_bundle_b2 = Bundle(name="B2 (max chords on a single TPV)", plans=())
    return EnvelopeResult(
        n_max_tpvs=0,
        chord_max_per_tpv=(),
        bundle_b1=empty_bundle_b1,
        bundle_b2=empty_bundle_b2,
        archive_size=0,
        n_enumerated=0,
        n_feasible=0,
        n_feasible_with_sat=0,
        n_sat_rejected=0,
        elapsed_sec=0.0,
        geoms=(),
        sat_constraint_enforced=False,
    )


def _tiny_tpv() -> TPV:
    verts = np.array(
        [[-100.0, -100.0], [100.0, -100.0], [100.0, 100.0], [-100.0, 100.0]],
        dtype=np.float64,
    )
    return TPV(label="X", polygon_km=Polygon(verts).buffer(0), vertices_km=verts)


def _tiny_run() -> PhaseARun:
    return build_run(
        envelope=_tiny_envelope(),
        tpvs=[_tiny_tpv()],
        frame=LocalFrame(lon_c=-70.0, lat_c=50.0),
        base_km=np.array([1000.0, 0.0]),
        gatepoints_km=np.array([[-400.0, 0.0]]),
        restricted_polygons=[Polygon([[-50, -50], [50, -50], [50, 50], [-50, 50]])],
        atc_polygons=[],
        dropsonde_polygons=[],
        sat_track_xy=np.array([[0.0, 0.0], [1000.0, 0.0]]),
        mission_profile={
            "AIRCRAFT_SPEED_KMH": 905.0,
            "TOTAL_FLIGHT_HOURS": 12.0,
            "BUDGET_KM": 10860.0,
        },
    )


# ---------------------------------------------------------------------------
# build_run
# ---------------------------------------------------------------------------

def test_build_run_stamps_timestamp_and_schema_version():
    run = _tiny_run()
    assert isinstance(run, PhaseARun)
    assert run.schema_version == SCHEMA_VERSION
    # ISO-8601-ish, ending in Z.
    assert run.timestamp_iso.endswith("Z")
    assert run.base_km.shape == (2,)
    assert run.gatepoints_km.shape == (1, 2)
    assert run.sat_track_xy.shape == (2, 2)


def test_build_run_copies_inputs():
    tpvs_in = [_tiny_tpv()]
    run = build_run(
        envelope=_tiny_envelope(),
        tpvs=tpvs_in,
        frame=LocalFrame(lon_c=-70.0, lat_c=50.0),
        base_km=np.array([0.0, 0.0]),
        gatepoints_km=np.empty((0, 2)),
        restricted_polygons=[],
        atc_polygons=[],
        dropsonde_polygons=[],
        sat_track_xy=np.array([[0.0, 0.0], [1.0, 0.0]]),
        mission_profile={"BUDGET_KM": 4250.0},
    )
    tpvs_in.append(_tiny_tpv())
    # Caller mutation must not leak into the snapshot.
    assert len(run.tpvs) == 1


# ---------------------------------------------------------------------------
# round-trip: save -> load
# ---------------------------------------------------------------------------

def test_save_load_round_trip(tmp_path: Path):
    run = _tiny_run()
    path = tmp_path / "snap.pkl"
    digest = save_phase_a_run(run, str(path))
    assert path.exists()
    sha_path = path.with_suffix(".pkl.sha1")
    assert sha_path.exists()
    assert sha_path.read_text(encoding="ascii").strip() == digest

    loaded = load_phase_a_run(str(path))
    assert isinstance(loaded, PhaseARun)
    assert loaded.schema_version == run.schema_version
    assert loaded.mission_profile == run.mission_profile
    assert np.array_equal(loaded.base_km, run.base_km)
    assert np.array_equal(loaded.sat_track_xy, run.sat_track_xy)
    assert len(loaded.tpvs) == len(run.tpvs)


def test_load_detects_sha1_tamper(tmp_path: Path):
    run = _tiny_run()
    path = tmp_path / "snap.pkl"
    save_phase_a_run(run, str(path))
    # Corrupt the pickle by appending a byte.
    with open(path, "ab") as f:
        f.write(b"\x00")
    with pytest.raises(IOError):
        load_phase_a_run(str(path))


def test_load_can_skip_sha1_check(tmp_path: Path):
    """With verify_sha1=False, a tampered sha file is ignored."""
    run = _tiny_run()
    path = tmp_path / "snap.pkl"
    save_phase_a_run(run, str(path))
    # Overwrite the sha digest with garbage; with verify_sha1=True this
    # would raise, with verify_sha1=False the load succeeds because the
    # pickle bytes themselves are intact.
    sha_path = path.with_suffix(".pkl.sha1")
    sha_path.write_text("0" * 40 + "\n", encoding="ascii")
    with pytest.raises(IOError):
        load_phase_a_run(str(path), verify_sha1=True)
    loaded = load_phase_a_run(str(path), verify_sha1=False)
    assert isinstance(loaded, PhaseARun)


def test_load_requires_phase_a_run_type(tmp_path: Path):
    path = tmp_path / "wrong.pkl"
    import pickle
    with open(path, "wb") as f:
        pickle.dump({"not": "a PhaseARun"}, f)
    with pytest.raises(TypeError):
        load_phase_a_run(str(path), verify_sha1=False)


def test_load_can_enforce_schema_version(tmp_path: Path):
    run = _tiny_run()
    path = tmp_path / "snap.pkl"
    save_phase_a_run(run, str(path))
    with pytest.raises(ValueError):
        load_phase_a_run(str(path), require_schema_version=SCHEMA_VERSION + 1)
