"""Satellite-coincidence hard constraint (FPO-5 / Phase 0a / 0a.3 / S2-min).

Phase A.v2 treats sat coincidence as a *plan-level hard constraint*, not an
objective:

    a plan is feasible iff there exists at least one (entry, arc_km) along
    the satellite ground track such that
        arc_km          >= SAT_MIN_LENGTH_KM        (default 85 km)
        AND
        T_dep = T_sat - geo_to_sat_mid / speed >= 0

Maximising ``coinc_dist`` is *not* implemented here — that belongs to Phase B
(per Luc 2026-06-09).  We also do not attempt to embed the sat arc into the
route polyline; we estimate whether a viable arc *exists* given the plan's
budget headroom and the geometry of the satellite track.

The candidate grid matches ``precompute_mission.py``:
* entry points sampled every ``ENTRY_STEP_KM`` (50 km) along the track,
* arc lengths from ``SAT_MIN_LENGTH_KM`` to ``SAT_ARC_MAX_KM`` in
  ``SAT_ARC_STEP_KM`` steps (85 km).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np

from multi_target_planner.route_builder import BuiltRoute


DEFAULT_AIRCRAFT_SPEED_KMH = 850.0
DEFAULT_T_SAT_H = 4.0
DEFAULT_SAT_MIN_LENGTH_KM = 6.0 / 60.0 * DEFAULT_AIRCRAFT_SPEED_KMH         # ≈ 85 km
DEFAULT_SAT_ARC_MAX_KM = 30.0 / 60.0 * DEFAULT_AIRCRAFT_SPEED_KMH           # ≈ 425 km
DEFAULT_SAT_ARC_STEP_KM = DEFAULT_SAT_MIN_LENGTH_KM                          # 85 km steps
DEFAULT_ENTRY_STEP_KM = 50.0


@dataclass(frozen=True)
class SatArcCandidate:
    entry_idx: int          # index of entry point on the sampled grid (debug)
    entry_xy: np.ndarray    # (2,)
    exit_xy: np.ndarray     # (2,)
    arc_km: float
    mid_xy: np.ndarray      # (2,)


def _cum_arc_length(track_xy: np.ndarray) -> np.ndarray:
    """Cumulative arc length along a polyline."""
    diffs = np.diff(track_xy, axis=0)
    seg = np.linalg.norm(diffs, axis=1)
    return np.concatenate([[0.0], np.cumsum(seg)])


def _interp_track(track_xy: np.ndarray, cum: np.ndarray, s: float) -> np.ndarray:
    """Linear interpolation along the track at arc-length ``s``."""
    s = float(np.clip(s, cum[0], cum[-1]))
    i = int(np.searchsorted(cum, s, side="right") - 1)
    i = max(0, min(i, len(cum) - 2))
    span = cum[i + 1] - cum[i]
    t = 0.0 if span < 1e-9 else (s - cum[i]) / span
    return track_xy[i] + t * (track_xy[i + 1] - track_xy[i])


def enumerate_sat_arc_candidates(
    sat_track_xy: np.ndarray,
    *,
    entry_step_km: float = DEFAULT_ENTRY_STEP_KM,
    arc_step_km: float = DEFAULT_SAT_ARC_STEP_KM,
    sat_min_length_km: float = DEFAULT_SAT_MIN_LENGTH_KM,
    sat_arc_max_km: float = DEFAULT_SAT_ARC_MAX_KM,
) -> list[SatArcCandidate]:
    """Build the grid of feasible (entry, arc_km) candidates along the track."""
    if sat_track_xy.shape[0] < 2:
        return []
    cum = _cum_arc_length(sat_track_xy)
    total = float(cum[-1])
    if total < sat_min_length_km:
        return []

    candidates: list[SatArcCandidate] = []
    arcs = np.arange(sat_min_length_km, sat_arc_max_km + 1e-6, arc_step_km)
    for ei, entry_s in enumerate(np.arange(0.0, total - sat_min_length_km + 1e-6, entry_step_km)):
        for arc in arcs:
            exit_s = entry_s + arc
            if exit_s > total:
                break
            entry = _interp_track(sat_track_xy, cum, entry_s)
            exit_pt = _interp_track(sat_track_xy, cum, exit_s)
            mid = _interp_track(sat_track_xy, cum, (entry_s + exit_s) / 2.0)
            candidates.append(
                SatArcCandidate(
                    entry_idx=ei,
                    entry_xy=entry,
                    exit_xy=exit_pt,
                    arc_km=float(arc),
                    mid_xy=mid,
                )
            )
    return candidates


@dataclass(frozen=True)
class SatFeasibility:
    feasible: bool
    chosen_arc_km: float = 0.0
    chosen_entry: np.ndarray | None = None
    chosen_exit: np.ndarray | None = None
    t_dep_h: float = 0.0
    side_trip_cost_km: float = 0.0
    reason: str = ""


def check_sat_feasibility(
    plan_route: BuiltRoute,
    base: np.ndarray,
    candidates: Sequence[SatArcCandidate],
    *,
    budget_km: float = 4250.0,
    aircraft_speed_kmh: float = DEFAULT_AIRCRAFT_SPEED_KMH,
    t_sat_h: float = DEFAULT_T_SAT_H,
    sat_min_length_km: float = DEFAULT_SAT_MIN_LENGTH_KM,
) -> SatFeasibility:
    """Decide whether the plan admits *any* sat arc satisfying both hard
    constraints simultaneously.

    Approach (conservative, fits the "hard constraint not objective" rule):
    for each candidate arc we ask whether the plan has enough budget headroom
    to absorb a round-trip side-trip from BASE to the arc midpoint
    (``2 * geo_to_mid + arc_km``).  This over-estimates the true insertion
    cost (a smart router would tap the arc near an existing route segment),
    so passing it is a *sufficient* condition for hard feasibility; failing
    it is a necessary condition for infeasibility.

    The ``T_dep >= 0`` check is exact:
        T_dep = t_sat_h - (geo_to_mid / aircraft_speed_kmh)
    """
    base = np.asarray(base, dtype=np.float64).reshape(2)
    headroom = budget_km - plan_route.cost.total_km
    if headroom <= 0.0:
        return SatFeasibility(feasible=False, reason="no budget headroom for sat arc")

    best: SatFeasibility | None = None
    for cand in candidates:
        if cand.arc_km < sat_min_length_km:
            continue
        geo_to_mid = float(np.linalg.norm(cand.mid_xy - base))
        t_dep = t_sat_h - geo_to_mid / aircraft_speed_kmh
        if t_dep < 0.0:
            continue
        side_trip = 2.0 * geo_to_mid + cand.arc_km
        if side_trip > headroom:
            continue
        result = SatFeasibility(
            feasible=True,
            chosen_arc_km=cand.arc_km,
            chosen_entry=cand.entry_xy,
            chosen_exit=cand.exit_xy,
            t_dep_h=t_dep,
            side_trip_cost_km=side_trip,
            reason="",
        )
        if best is None or result.chosen_arc_km > best.chosen_arc_km:
            best = result
    if best is None:
        return SatFeasibility(
            feasible=False,
            reason="no sat arc fits both T_dep>=0 and budget headroom",
        )
    return best
