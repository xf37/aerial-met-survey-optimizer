"""Polyline-builder + cost evaluator for the multi-target planner.

A *route* is an ordered sequence of :class:`Stop` items, optionally with
gatepoints interleaved at arbitrary positions:

    BASE -> [stops in caller-specified order] -> BASE

Each TPV stop is a survey of n parallel chords traversed boustrophedon-style:
enter chord 1 at the endpoint closest to current position, exit at the far
endpoint, fly to the closest endpoint of chord 2, etc.  The exit endpoint of
chord n is also the exit of the TPV survey — the next transit begins there.

**Transit routing around restricted airspace.**  Every transit segment
(BASE -> first stop, stop -> stop, last stop -> BASE) is routed via
:func:`multi_target_planner.visibility_graph.route_around_restricted`, which
returns the shortest poly-line that detours around any restricted polygon
that would otherwise be crossed.  Detour vertices appear as additional
points on the route polyline with kind ``"transit"``.  Inside a TPV survey
we trust the chord-generation filter to have already dropped any chord that
crosses restricted airspace, so chord and spacer segments are kept as direct
lines.

Cost model (per the FPO-1 spec, codified by Reading B for turn penalty):

    total_cost = geometric_distance
               + sum(seg.length_inside_ATC * (ATC_FACTOR - 1))
               + n_sharp_turns * TURN_PENALTY_KM * TURN_FACTOR

where a "sharp" turn is one whose deviation from straight is **>=**
``TURN_THRESHOLD_DEG`` (default 30°), matching the inclusive Chinese reading
"30 度以上" Master picked on 2026-06-09.
"""

from __future__ import annotations

import os
import sys
from dataclasses import dataclass
from typing import Sequence

import numpy as np
from shapely.geometry import LineString

# Borrowed 07 ops live at the repo root.
_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

import plan_route_field as prf  # noqa: E402

from multi_target_planner.turn_penalty import (
    compute_turn_angles_deg,
    turn_penalty_cost,
)
from multi_target_planner.chord_layout import ChordSet
from multi_target_planner.visibility_graph import route_around_restricted


# ---------------------------------------------------------------------------
# Defaults — every value pulled from precompute_mission.py / 07 Cell 1
# ---------------------------------------------------------------------------

DEFAULT_BUDGET_KM = 4250.0
DEFAULT_AIRCRAFT_SPEED_KMH = 850.0
DEFAULT_TURN_PENALTY_KM = 7.5 / 60.0 * DEFAULT_AIRCRAFT_SPEED_KMH   # ≈ 106 km
DEFAULT_TURN_THRESHOLD_DEG = 30.0
DEFAULT_TURN_FACTOR = 1.2
DEFAULT_ATC_FACTOR = 1.35


@dataclass(frozen=True)
class CostBreakdown:
    geometric_km: float
    atc_extra_km: float
    turn_penalty_km: float
    total_km: float
    n_sharp_turns: int
    n_total_turns: int
    coinc_dist_km: float = 0.0          # total km flown along a satellite track
    t_dep_h: float | None = None        # departure time = T_sat - d(BASE->sat_mid)/v
    sat_entry_km: tuple[float, float] | None = None
    sat_exit_km: tuple[float, float] | None = None


@dataclass(frozen=True)
class BuiltRoute:
    polyline: np.ndarray         # (N, 2)
    seg_kinds: tuple[str, ...]   # one per segment (N-1 of them): "transit" | "chord" | "spacer" | "sat"
    cost: CostBreakdown


# ---------------------------------------------------------------------------
# Polyline construction
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Stop:
    """One leg of the route — either a TPV survey or a single gatepoint.

    For ``kind == "tpv"``: ``payload`` is the :class:`ChordSet` and
    ``tpv_index`` records which TPV (so the caller can map back to labels).
    For ``kind == "gatepoint"``: ``payload`` is the (2,) coordinate and
    ``tpv_index`` is -1.
    """
    kind: str
    payload: object
    tpv_index: int = -1


def make_tpv_stop(tpv_index: int, chord_set: ChordSet) -> Stop:
    return Stop(kind="tpv", payload=chord_set, tpv_index=tpv_index)


def make_gatepoint_stop(coord_km: np.ndarray) -> Stop:
    return Stop(kind="gatepoint", payload=np.asarray(coord_km, dtype=np.float64).reshape(2), tpv_index=-1)


@dataclass(frozen=True)
class SatStopPayload:
    """Payload for a SatStop: enter at ``pt_a``, fly arc forward to ``pt_b``."""
    pt_a: np.ndarray       # entry point (km)
    pt_b: np.ndarray       # exit point (km)
    arc_km: float          # geometric length of the satellite arc (km)
    track_waypoints: np.ndarray  # (N, 2) intermediate points along the sat track


def make_sat_stop(pt_a, pt_b, arc_km: float, track_waypoints) -> Stop:
    """Build a Stop carrying a SatStopPayload. The aircraft flies pt_a -> pt_b
    along the satellite ground track (track_waypoints are the intermediate
    vertices along the cropped sat track between pt_a and pt_b)."""
    payload = SatStopPayload(
        pt_a=np.asarray(pt_a, dtype=np.float64).reshape(2),
        pt_b=np.asarray(pt_b, dtype=np.float64).reshape(2),
        arc_km=float(arc_km),
        track_waypoints=np.asarray(track_waypoints, dtype=np.float64).reshape(-1, 2),
    )
    return Stop(kind="sat", payload=payload, tpv_index=-1)


def _transit_polyline(
    src: np.ndarray,
    dst: np.ndarray,
    restricted_union,
) -> np.ndarray | None:
    """Compute the actual transit polyline between two waypoints, routing
    around any restricted polygon that would block the direct line.

    Returns ``None`` if no route is reachable (a fully encircled point).
    """
    if restricted_union is None or restricted_union.is_empty:
        return np.vstack([src, dst])
    poly = route_around_restricted(src, dst, restricted_union)
    return poly  # may be None when boxed in


def build_route(
    base: np.ndarray,
    gatepoints_km: np.ndarray | None = None,
    tpv_visit: Sequence[tuple[int, ChordSet]] | None = None,
    *,
    stops: Sequence[Stop] | None = None,
    restricted_union=None,
    atc_union=None,
    turn_penalty_km: float = DEFAULT_TURN_PENALTY_KM,
    turn_threshold_deg: float = DEFAULT_TURN_THRESHOLD_DEG,
    turn_factor: float = DEFAULT_TURN_FACTOR,
    atc_factor: float = DEFAULT_ATC_FACTOR,
    t_sat_h: float | None = None,
    aircraft_speed_kmh: float = DEFAULT_AIRCRAFT_SPEED_KMH,
) -> BuiltRoute | None:
    """Build a route as ``BASE -> stops in order -> BASE``.

    Either pass an explicit ``stops`` list (preferred — allows gatepoints to
    be interleaved with TPVs at arbitrary positions) or use the legacy
    ``(gatepoints_km, tpv_visit)`` pair which forces ``BASE -> all gatepoints
    -> all TPVs -> BASE`` and is kept for backwards compatibility with the
    earlier smoke tests.
    """
    base = np.asarray(base, dtype=np.float64).reshape(2)

    if stops is None:
        # Legacy path: gatepoints first (in order), then TPVs (in tpv_visit order).
        gp = (
            np.asarray(gatepoints_km, dtype=np.float64).reshape(-1, 2)
            if gatepoints_km is not None
            else np.empty((0, 2), dtype=np.float64)
        )
        legacy_stops: list[Stop] = []
        for g in gp:
            legacy_stops.append(make_gatepoint_stop(g))
        for tpv_idx, chord_set in (tpv_visit or ()):
            legacy_stops.append(make_tpv_stop(tpv_idx, chord_set))
        stops = legacy_stops

    waypoints: list[np.ndarray] = [base]
    kinds: list[str] = []
    cur = base

    # Tracking for sat arc midpoint -> T_dep computation.
    geo_dist_so_far = 0.0
    geo_to_sat_mid: float | None = None
    coinc_dist_km = 0.0
    sat_entry_xy: tuple[float, float] | None = None
    sat_exit_xy: tuple[float, float] | None = None

    def _add_transit_to(target: np.ndarray) -> bool:
        nonlocal cur, geo_dist_so_far
        poly = _transit_polyline(cur, target, restricted_union)
        if poly is None:
            return False
        # poly[0] == cur (already in waypoints); append everything from poly[1:].
        for k in range(1, poly.shape[0]):
            waypoints.append(poly[k])
            kinds.append("transit")
            geo_dist_so_far += float(np.linalg.norm(poly[k] - poly[k - 1]))
        cur = waypoints[-1]
        return True

    for stop in stops:
        if stop.kind == "gatepoint":
            gp_xy = np.asarray(stop.payload, dtype=np.float64).reshape(2)
            if not _add_transit_to(gp_xy):
                return None
        elif stop.kind == "tpv":
            chord_set: ChordSet = stop.payload  # type: ignore[assignment]
            # Pick the entry endpoint of the first chord closer to `cur`.
            first = chord_set.chords[0]
            entry_pt = np.asarray(first["pt_a"], dtype=np.float64)
            other_pt = np.asarray(first["pt_b"], dtype=np.float64)
            if np.linalg.norm(other_pt - cur) < np.linalg.norm(entry_pt - cur):
                entry_pt, other_pt = other_pt, entry_pt
            if not _add_transit_to(entry_pt):
                return None
            # Now we're at the first chord's entry; traverse the rest of the
            # chord set boustrophedon-style (no obstacle routing needed inside
            # the TPV — generate_candidate_chords already filtered chords that
            # cross restricted airspace).
            cur = entry_pt
            # Append the exit of chord 1.
            waypoints.append(other_pt)
            kinds.append("chord")
            geo_dist_so_far += float(np.linalg.norm(other_pt - cur))
            cur = other_pt
            for c in chord_set.chords[1:]:
                pt_a = np.asarray(c["pt_a"], dtype=np.float64)
                pt_b = np.asarray(c["pt_b"], dtype=np.float64)
                d_a = float(np.linalg.norm(pt_a - cur))
                d_b = float(np.linalg.norm(pt_b - cur))
                if d_a <= d_b:
                    nxt_entry, nxt_exit = pt_a, pt_b
                else:
                    nxt_entry, nxt_exit = pt_b, pt_a
                # Inter-chord spacer (still inside the TPV; treated as straight).
                waypoints.append(nxt_entry)
                kinds.append("spacer")
                geo_dist_so_far += float(np.linalg.norm(nxt_entry - cur))
                waypoints.append(nxt_exit)
                kinds.append("chord")
                geo_dist_so_far += float(np.linalg.norm(nxt_exit - nxt_entry))
                cur = nxt_exit
        elif stop.kind == "sat":
            sat: SatStopPayload = stop.payload  # type: ignore[assignment]
            # 1. Transit cur -> sat entry (pt_a).
            if not _add_transit_to(sat.pt_a):
                return None
            # 2. Fly the cropped satellite track from pt_a to pt_b. The track
            #    waypoints already include pt_a as the first vertex; skip it.
            track = sat.track_waypoints
            if track.shape[0] < 2 or np.linalg.norm(track[0] - sat.pt_a) > 1e-6:
                # Defensive fallback: straight pt_a -> pt_b.
                track = np.vstack([sat.pt_a, sat.pt_b])
            # Distance up to sat midpoint for T_dep:
            arc_so_far = 0.0
            for k in range(1, track.shape[0]):
                seg_len = float(np.linalg.norm(track[k] - track[k - 1]))
                if (geo_to_sat_mid is None
                        and arc_so_far + seg_len >= sat.arc_km / 2.0):
                    frac = (sat.arc_km / 2.0 - arc_so_far) / max(seg_len, 1e-9)
                    geo_to_sat_mid = geo_dist_so_far + frac * seg_len
                arc_so_far += seg_len
                waypoints.append(track[k])
                kinds.append("sat")
                geo_dist_so_far += seg_len
            if geo_to_sat_mid is None:
                # Whole arc fit in last segment without crossing midpoint trigger
                geo_to_sat_mid = geo_dist_so_far - sat.arc_km / 2.0
            coinc_dist_km += sat.arc_km
            sat_entry_xy = (float(sat.pt_a[0]), float(sat.pt_a[1]))
            sat_exit_xy = (float(sat.pt_b[0]), float(sat.pt_b[1]))
            cur = waypoints[-1]
        else:
            raise ValueError(f"Unknown stop kind: {stop.kind!r}")

    # Final return-to-base (may need to route around restricted on the way back).
    if not _add_transit_to(base):
        return None

    polyline = np.vstack(waypoints)
    cost = compute_route_cost(
        polyline=polyline,
        atc_union=atc_union,
        turn_penalty_km=turn_penalty_km,
        turn_threshold_deg=turn_threshold_deg,
        turn_factor=turn_factor,
        atc_factor=atc_factor,
    )
    # T_dep = T_sat - d(BASE -> sat midpoint) / v.  Negative T_dep means the
    # aircraft would have had to leave before T=0 (infeasible).
    t_dep_h = None
    if t_sat_h is not None and geo_to_sat_mid is not None:
        t_dep_h = float(t_sat_h) - float(geo_to_sat_mid) / float(aircraft_speed_kmh)
    cost = CostBreakdown(
        geometric_km=cost.geometric_km,
        atc_extra_km=cost.atc_extra_km,
        turn_penalty_km=cost.turn_penalty_km,
        total_km=cost.total_km,
        n_sharp_turns=cost.n_sharp_turns,
        n_total_turns=cost.n_total_turns,
        coinc_dist_km=float(coinc_dist_km),
        t_dep_h=t_dep_h,
        sat_entry_km=sat_entry_xy,
        sat_exit_km=sat_exit_xy,
    )
    return BuiltRoute(polyline=polyline, seg_kinds=tuple(kinds), cost=cost)


# ---------------------------------------------------------------------------
# Cost evaluator
# ---------------------------------------------------------------------------

def _segment_lengths_and_atc(polyline: np.ndarray, atc_union) -> tuple[np.ndarray, float]:
    """Return (lengths, atc_extra_km).

    ``lengths`` is the per-segment geometric distance.  ``atc_extra_km`` is the
    total extra km charged by the ATC penalty (i.e. ``inside_len * (factor-1)``
    accumulated across segments).
    """
    n_seg = polyline.shape[0] - 1
    if n_seg <= 0:
        return np.zeros((0,)), 0.0
    diffs = polyline[1:] - polyline[:-1]
    lengths = np.linalg.norm(diffs, axis=1)
    atc_inside_total = 0.0
    if atc_union is not None and not atc_union.is_empty:
        for i in range(n_seg):
            p1, p2 = polyline[i], polyline[i + 1]
            if lengths[i] < 1e-9:
                continue
            seg = LineString([p1, p2])
            inside = seg.intersection(atc_union)
            if not inside.is_empty:
                atc_inside_total += inside.length
    return lengths, atc_inside_total


def compute_route_cost(
    polyline: np.ndarray,
    *,
    atc_union=None,
    turn_penalty_km: float = DEFAULT_TURN_PENALTY_KM,
    turn_threshold_deg: float = DEFAULT_TURN_THRESHOLD_DEG,
    turn_factor: float = DEFAULT_TURN_FACTOR,
    atc_factor: float = DEFAULT_ATC_FACTOR,
) -> CostBreakdown:
    """Compute :class:`CostBreakdown` for a polyline."""
    polyline = np.asarray(polyline, dtype=np.float64)
    lengths, atc_inside_total = _segment_lengths_and_atc(polyline, atc_union)
    geometric_km = float(np.sum(lengths))
    atc_extra_km = float(atc_inside_total) * (atc_factor - 1.0)

    angles = compute_turn_angles_deg(polyline)
    n_sharp = int(np.sum(angles > turn_threshold_deg))
    n_total = int(angles.size)
    turn_km = turn_penalty_cost(
        angles, turn_penalty_km, turn_threshold_deg, turn_factor
    )

    total = geometric_km + atc_extra_km + turn_km
    return CostBreakdown(
        geometric_km=geometric_km,
        atc_extra_km=atc_extra_km,
        turn_penalty_km=turn_km,
        total_km=total,
        n_sharp_turns=n_sharp,
        n_total_turns=n_total,
    )
