"""Phase-A archive (de)serialisation (FPO-6 / PB-0).

The Phase A enumeration takes ~4 minutes on the 777 / 12 h profile. Re-running
the model every time Master tweaks a figure colour is wasteful, so this module
splits the work into two stages:

* **Stage 1** (slow, ~4 min) — run Phase A end to end, snapshot the result.
* **Stage 2** (fast, ~5 s) — load the snapshot, render whichever figure.

The snapshot is a :class:`PhaseARun` dataclass bundling everything the renderer
needs: the :class:`EnvelopeResult` (which carries the Pareto archive and bundles),
the input geometry (TPVs, polygon unions, gatepoints, satellite track, BASE),
and the mission profile (speed/hours/budget/etc).

We serialise with :mod:`pickle` because the contained objects (Shapely polygons,
numpy arrays, frozen dataclasses) are all picklable.  A SHA-1 of the archive
file's contents is stored next to it so a stale snapshot is detected at load
time.
"""

from __future__ import annotations

import hashlib
import os
import pickle
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any

import numpy as np

from multi_target_planner.data_loader import TPV, LocalFrame
from multi_target_planner.phase_a import EnvelopeResult


SCHEMA_VERSION = 1


@dataclass
class PhaseARun:
    """Everything a renderer needs to reproduce a Phase A figure offline."""

    envelope: EnvelopeResult
    tpvs: list[TPV]
    frame: LocalFrame
    base_km: np.ndarray
    gatepoints_km: np.ndarray
    restricted_polygons: list
    atc_polygons: list
    dropsonde_polygons: list
    sat_track_xy: np.ndarray
    mission_profile: dict[str, float]
    timestamp_iso: str
    schema_version: int = SCHEMA_VERSION
    notes: dict[str, Any] = field(default_factory=dict)


def _sha1_file(path: str) -> str:
    h = hashlib.sha1()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def save_phase_a_run(run: PhaseARun, path: str) -> str:
    """Pickle ``run`` to ``path``; write a sibling ``.sha1`` digest file.

    Returns the SHA-1 hex digest of the pickle.  The digest is also written to
    ``<path>.sha1`` so callers can detect tampering or corruption later.
    """
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "wb") as f:
        pickle.dump(run, f, protocol=pickle.HIGHEST_PROTOCOL)
    digest = _sha1_file(path)
    with open(path + ".sha1", "w", encoding="ascii") as f:
        f.write(digest + "\n")
    return digest


def load_phase_a_run(
    path: str,
    *,
    verify_sha1: bool = True,
    require_schema_version: int | None = None,
) -> PhaseARun:
    """Read a snapshot back.

    If ``verify_sha1`` is True (default) and a ``<path>.sha1`` file exists,
    the file's current digest is compared.  ``require_schema_version`` lets
    callers fail loud when the snapshot was produced by an incompatible
    planner build.
    """
    if verify_sha1:
        sha_path = path + ".sha1"
        if os.path.isfile(sha_path):
            with open(sha_path, encoding="ascii") as f:
                expected = f.read().strip()
            actual = _sha1_file(path)
            if expected != actual:
                raise IOError(
                    f"SHA-1 mismatch for {path}: expected {expected[:12]}…, "
                    f"got {actual[:12]}…"
                )
    with open(path, "rb") as f:
        run = pickle.load(f)
    if not isinstance(run, PhaseARun):
        raise TypeError(
            f"{path} did not contain a PhaseARun (got {type(run).__name__})"
        )
    if require_schema_version is not None and run.schema_version != require_schema_version:
        raise ValueError(
            f"{path} schema_version={run.schema_version}, "
            f"required={require_schema_version}"
        )
    return run


def build_run(
    envelope: EnvelopeResult,
    *,
    tpvs: list[TPV],
    frame: LocalFrame,
    base_km: np.ndarray,
    gatepoints_km: np.ndarray,
    restricted_polygons: list,
    atc_polygons: list,
    dropsonde_polygons: list,
    sat_track_xy: np.ndarray,
    mission_profile: dict[str, float],
    notes: dict[str, Any] | None = None,
) -> PhaseARun:
    """Helper: package a Phase A result + its input geometry into a ``PhaseARun``."""
    return PhaseARun(
        envelope=envelope,
        tpvs=list(tpvs),
        frame=frame,
        base_km=np.asarray(base_km, dtype=np.float64).reshape(2),
        gatepoints_km=np.asarray(gatepoints_km, dtype=np.float64).reshape(-1, 2),
        restricted_polygons=list(restricted_polygons),
        atc_polygons=list(atc_polygons),
        dropsonde_polygons=list(dropsonde_polygons),
        sat_track_xy=np.asarray(sat_track_xy, dtype=np.float64).reshape(-1, 2),
        mission_profile=dict(mission_profile),
        timestamp_iso=datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        schema_version=SCHEMA_VERSION,
        notes=dict(notes or {}),
    )
