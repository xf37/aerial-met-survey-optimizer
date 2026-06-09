"""Independent D5 verifier (FPO-5 / Phase 0a / 0a.3).

Per the FPO-3 R3 review (Jing) + Serge's 0a.2 review:

    The verifier MUST reload every shapefile from disk and recompute geometry
    from scratch.  It MUST NOT share any precomputed atomic block, cached SP
    matrix, planner-internal intermediate, or in-memory data structure with
    the solver.  Otherwise it degenerates into "did the cache agree with
    itself?" — which catches nothing.

So this module:

* Re-reads the seven shapefiles from ``DATA_DIR``;
* Rebuilds restricted / ATC / dropsonde polygon unions;
* Reloads the satellite track;
* Recomputes the route's geometric distance, ATC penalty contribution, turn
  count, sat-feasibility decision — all *from the polyline alone*; and
* Checks the six hard constraints listed in the spec.

If any check fails the plan is rejected outright — LOCK 1 means the planner
never surfaces an infeasible plan, so a verifier failure is a *solver bug*
and the caller is expected to escalate, not silently filter.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Sequence

import numpy as np
from shapely.geometry import LineString
from shapely.ops import unary_union

from multi_target_planner.data_loader import (
    DEFAULT_DATA_DIR,
    TPV_FILENAME_PHASE0A,
    LocalFrame,
    base_km,
    load_atc,
    load_gatepoints,
    load_restricted,
    load_satellite_track,
    load_tpvs,
)
from multi_target_planner.sat_arc import (
    DEFAULT_AIRCRAFT_SPEED_KMH,
    DEFAULT_ENTRY_STEP_KM,
    DEFAULT_SAT_ARC_MAX_KM,
    DEFAULT_SAT_ARC_STEP_KM,
    DEFAULT_SAT_MIN_LENGTH_KM,
    DEFAULT_T_SAT_H,
    check_sat_feasibility,
    enumerate_sat_arc_candidates,
)


@dataclass(frozen=True)
class VerifierReport:
    """Per-plan verification outcome."""
    ok: bool
    violations: tuple[str, ...]
    independent_cost_km: float
    chord_endpoints_checked: int


@dataclass
class IndependentMissionState:
    """Re-read-from-disk view of the mission inputs."""
    frame: LocalFrame
    base: np.ndarray
    gatepoints_km: np.ndarray
    restricted_union: object
    atc_union: object
    sat_track_xy: np.ndarray
    sat_candidates: tuple

    @classmethod
    def from_disk(
        cls,
        data_dir: str = DEFAULT_DATA_DIR,
        tpv_filename: str = TPV_FILENAME_PHASE0A,
    ) -> "IndependentMissionState":
        tpvs, frame = load_tpvs(data_dir, tpv_filename)
        gatepoints = load_gatepoints(data_dir, frame)
        restricted = load_restricted(data_dir, frame)
        atc = load_atc(data_dir, frame)
        sat_track = load_satellite_track(data_dir, frame)
        return cls(
            frame=frame,
            base=base_km(frame),
            gatepoints_km=gatepoints,
            restricted_union=unary_union(restricted) if restricted else None,
            atc_union=unary_union(atc) if atc else None,
            sat_track_xy=sat_track,
            sat_candidates=tuple(
                enumerate_sat_arc_candidates(
                    sat_track,
                    entry_step_km=DEFAULT_ENTRY_STEP_KM,
                    arc_step_km=DEFAULT_SAT_ARC_STEP_KM,
                    sat_min_length_km=DEFAULT_SAT_MIN_LENGTH_KM,
                    sat_arc_max_km=DEFAULT_SAT_ARC_MAX_KM,
                )
            ),
        )


def verify_plan(
    plan,
    *,
    state: IndependentMissionState,
    budget_km: float = 4250.0,
    aircraft_speed_kmh: float = DEFAULT_AIRCRAFT_SPEED_KMH,
    turn_penalty_km: float = 7.5 / 60.0 * DEFAULT_AIRCRAFT_SPEED_KMH,
    turn_threshold_deg: float = 30.0,
    turn_factor: float = 1.2,
    atc_factor: float = 1.35,
    sat_min_length_km: float = DEFAULT_SAT_MIN_LENGTH_KM,
    t_sat_h: float = DEFAULT_T_SAT_H,
    min_chord_spacing_km: float = 100.0,
) -> VerifierReport:
    """Run all six hard checks against ``plan`` independently of the solver."""
    violations: list[str] = []

    polyline = plan.route.polyline

    # --- (1) Geometric distance + ATC penalty + turn penalty recomputed
    # --- from the polyline only.  We do NOT call route_builder.compute_route_cost;
    # we re-derive every quantity in-line here so a bug there cannot mask a bug.
    diffs = polyline[1:] - polyline[:-1]
    seg_lens = np.linalg.norm(diffs, axis=1)
    geom_km = float(np.sum(seg_lens))

    atc_extra = 0.0
    if state.atc_union is not None and not state.atc_union.is_empty:
        for i in range(polyline.shape[0] - 1):
            if seg_lens[i] < 1e-9:
                continue
            seg = LineString([polyline[i], polyline[i + 1]])
            inside = seg.intersection(state.atc_union)
            if not inside.is_empty:
                atc_extra += inside.length * (atc_factor - 1.0)

    # Turn penalty: count vertices whose deviation from straight > threshold.
    sharp = 0
    for i in range(1, polyline.shape[0] - 1):
        v1 = polyline[i] - polyline[i - 1]
        v2 = polyline[i + 1] - polyline[i]
        l1 = float(np.linalg.norm(v1))
        l2 = float(np.linalg.norm(v2))
        if l1 < 1e-9 or l2 < 1e-9:
            continue
        cos_a = float(np.clip(np.dot(v1 / l1, v2 / l2), -1.0, 1.0))
        dev_deg = float(np.degrees(np.arccos(cos_a)))
        if dev_deg >= turn_threshold_deg:    # inclusive, matches S4 (turn_penalty.py)
            sharp += 1
    turn_km = sharp * turn_penalty_km * turn_factor

    independent_cost = geom_km + atc_extra + turn_km

    # (1) Budget.
    if independent_cost > budget_km + 1e-6:
        violations.append(
            f"budget: independent_cost {independent_cost:.2f} > {budget_km:.2f}"
        )

    # (2) Restricted airspace — every segment.
    if state.restricted_union is not None and not state.restricted_union.is_empty:
        for i in range(polyline.shape[0] - 1):
            seg = LineString([polyline[i], polyline[i + 1]])
            if (
                state.restricted_union.intersects(seg)
                and not state.restricted_union.touches(seg)
            ):
                violations.append(f"restricted: segment {i} crosses no-fly zone")
                break

    # (3) Gatepoints — every one must appear on the polyline (within 0.5 km).
    if state.gatepoints_km.size > 0:
        for j, gp in enumerate(state.gatepoints_km):
            d = np.linalg.norm(polyline - gp, axis=1)
            if d.min() > 0.5:
                violations.append(f"gatepoint GP{j} not on route polyline")

    # (4) Minimum chord spacing (Reading B is by design, not a hard check; the
    # spec says >= 100 km).  We look at each TPV's chord set in the plan.
    chord_endpoints_checked = 0
    for cs in plan.chord_choices:
        offsets = sorted(c["offset"] for c in cs.chords)
        chord_endpoints_checked += 2 * len(cs.chords)
        for k in range(len(offsets) - 1):
            if abs(offsets[k + 1] - offsets[k]) + 1e-6 < min_chord_spacing_km:
                violations.append(
                    f"chord spacing < {min_chord_spacing_km} km in {cs.tpv_index}"
                )
                break

    # (5) + (6) sat feasibility (SAT_MIN + T_dep) — same arc candidates the
    # solver enumerated, but recomputed against the independently-reloaded
    # sat track and BASE.
    sat = check_sat_feasibility(
        plan.route,
        state.base,
        state.sat_candidates,
        budget_km=budget_km,
        aircraft_speed_kmh=aircraft_speed_kmh,
        t_sat_h=t_sat_h,
        sat_min_length_km=sat_min_length_km,
    )
    if not sat.feasible:
        violations.append(f"sat: {sat.reason}")

    return VerifierReport(
        ok=not violations,
        violations=tuple(violations),
        independent_cost_km=independent_cost,
        chord_endpoints_checked=chord_endpoints_checked,
    )


def verify_bundle(plans: Sequence, *, state: IndependentMissionState, **kwargs) -> list[VerifierReport]:
    """Run :func:`verify_plan` over a list of plans."""
    return [verify_plan(p, state=state, **kwargs) for p in plans]
