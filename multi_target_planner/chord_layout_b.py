"""Non-parallel chord enumeration (FPO-6 / PB-1).

Phase A's ``chord_layout.enumerate_chord_sets`` forces every chord inside a
TPV survey to be perpendicular to the PCA major axis, sampled on a uniform
spacing grid.  When a restricted polygon lies across the TPV's centreline
(as it does for TPV-1 in the 2026 mission data), the inter-chord spacers
between parallel chords have no choice but to cross restricted territory,
and 3- and 4-chord patterns get rejected even though the chord lines
themselves are clear.

This module is Phase B's geometric upgrade: each chord carries its own
``(theta_deg, s_km)`` where

* ``theta_deg`` is the deviation from the PCA-perpendicular ideal (default
  grid ±30° in 7.5° steps; 9 values), and
* ``s_km`` is the signed offset of the chord midpoint along the major axis
  (default 25 km step on ``[-0.95 a, +0.95 a]``).

The chord line itself is the intersection of the polygon with the line
through ``TPV.center + s · major_dir`` in direction
``rotate(major_dir, π/2 + theta_deg)`` — exactly what
``plan_route_field.chord_in_polygon`` already computes.

Phase A's parallel layout is a strict subset: every Phase A chord with
``angle_dev_deg=θ`` becomes a Phase B chord at ``(θ, s)`` with the same θ
on every chord.  Phase B widens the search to allow each chord to choose
its own angle, which is the lever needed to route 3-4 chord layouts past a
TPV-interior restricted polygon.

Two new feasibility checks become relevant once chords are no longer
parallel:

* **Pairwise major-axis spacing** — for every chord pair the projection of
  the midpoints onto the major axis must differ by at least
  ``min_spacing_km`` (the spec's 100 km floor).  Parallel-chord layouts
  get this for free; non-parallel layouts do not.
* **Pairwise non-crossing inside the polygon** — two clipped chord
  segments must not cross each other.  Parallel chords cannot cross by
  construction; non-parallel ones can, and a crossing inside the TPV
  produces a nonsensical survey pattern.

The module exposes both single-chord builders (used by ALNS operators
when sampling on demand) and a small exhaustive enumerator suitable for
unit tests and synthetic instances.
"""

from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np
from shapely.geometry import LineString

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

import plan_route_field as prf  # noqa: E402

from multi_target_planner.chord_layout import TpvGeometry


# ---------------------------------------------------------------------------
# Dataclasses
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class NonParallelChordRecord:
    """A single chord inside a TPV at a chosen ``(theta_deg, s_km)``.

    ``p_entry`` and ``p_exit`` are the endpoints where the chord line meets
    the TPV polygon (i.e. the chord-segment endpoints).  ``length_km`` is
    the geometric length of that segment.

    ``crosses_restricted`` is a precomputed flag — Phase A drops chords that
    cross restricted at generation time, but Phase B's ALNS may want to
    *see* such candidates and reject them at scoring time, so we keep the
    flag rather than the chord.
    """

    tpv_index: int
    theta_deg: float
    s_km: float
    p_entry: np.ndarray
    p_exit: np.ndarray
    length_km: float
    crosses_restricted: bool


@dataclass(frozen=True)
class NonParallelChordSet:
    """A collection of non-parallel chords planned to fly inside one TPV.

    ``survey_dist_km`` is the sum of chord lengths plus the inter-chord
    transition lengths along the major-axis projection.  Spacers will route
    around restricted polygons at build time (Phase B carries the
    visibility-graph routing forward), so this is a *lower bound* on the
    actual flown distance; the route builder may report a higher number
    once detours are accounted for.
    """

    tpv_index: int
    chords: tuple[NonParallelChordRecord, ...]
    survey_dist_km: float

    @property
    def n_chords(self) -> int:
        return len(self.chords)

    @property
    def is_parallel(self) -> bool:
        """True iff every chord shares the same theta — i.e. this set is
        equivalent to a Phase A parallel layout."""
        if not self.chords:
            return True
        first = self.chords[0].theta_deg
        return all(abs(c.theta_deg - first) < 1e-6 for c in self.chords)


# ---------------------------------------------------------------------------
# Geometry primitives
# ---------------------------------------------------------------------------

def _direction_for(theta_deg: float, phi_rad: float) -> np.ndarray:
    """Unit direction vector for a chord whose theta-deviation is
    ``theta_deg`` from the PCA-perpendicular axis."""
    chord_angle = phi_rad + math.pi / 2 + math.radians(theta_deg)
    return np.array([math.cos(chord_angle), math.sin(chord_angle)], dtype=np.float64)


def _major_dir_for(phi_rad: float) -> np.ndarray:
    return np.array([math.cos(phi_rad), math.sin(phi_rad)], dtype=np.float64)


def build_chord_at(
    tpv_index: int,
    tpv_polygon_km,
    geom: TpvGeometry,
    *,
    theta_deg: float,
    s_km: float,
    restricted_union=None,
    far_km: float = 2e4,
) -> NonParallelChordRecord | None:
    """Construct one chord at ``(theta_deg, s_km)``.

    Returns ``None`` when the chord line does not intersect the polygon
    (which happens at ``|s|`` close to the major-axis extent or when the
    polygon is highly non-convex).  Restricted-zone crossing is *recorded*
    but does not by itself reject the chord — callers can choose to keep
    or discard.
    """
    major = _major_dir_for(geom.phi_rad)
    direction = _direction_for(theta_deg, geom.phi_rad)
    midpoint = np.asarray(geom.center_km, dtype=np.float64) + s_km * major
    # Use plan_route_field's chord_in_polygon for parity with Phase A.
    result = prf.chord_in_polygon(midpoint, direction, tpv_polygon_km, far=far_km)
    if result is None:
        return None
    pt_a, pt_b, length = result
    if length < 10.0:
        return None
    crosses = False
    if restricted_union is not None and not restricted_union.is_empty:
        crosses = prf.seg_crosses_restricted(pt_a, pt_b, restricted_union)
    return NonParallelChordRecord(
        tpv_index=tpv_index,
        theta_deg=float(theta_deg),
        s_km=float(s_km),
        p_entry=np.asarray(pt_a, dtype=np.float64),
        p_exit=np.asarray(pt_b, dtype=np.float64),
        length_km=float(length),
        crosses_restricted=bool(crosses),
    )


# ---------------------------------------------------------------------------
# Pairwise feasibility
# ---------------------------------------------------------------------------

def chords_cross_inside_polygon(
    c1: NonParallelChordRecord,
    c2: NonParallelChordRecord,
) -> bool:
    """``True`` iff the two clipped chord SEGMENTS cross.

    Two parallel chords never cross.  Two non-parallel chord *lines* always
    cross somewhere; the question is whether that crossing falls inside
    both clipped segments (= inside the polygon, since each segment is
    already clipped to the polygon).  We delegate to Shapely's robust
    segment-intersection test.
    """
    seg1 = LineString([c1.p_entry, c1.p_exit])
    seg2 = LineString([c2.p_entry, c2.p_exit])
    return bool(seg1.intersects(seg2) and not seg1.touches(seg2))


def major_axis_spacing(
    c1: NonParallelChordRecord,
    c2: NonParallelChordRecord,
) -> float:
    """Distance between the two chord midpoints projected onto the major axis.

    This is the quantity the spec's 100 km floor applies to, regardless of
    each chord's individual theta.
    """
    return abs(c1.s_km - c2.s_km)


def chord_set_pairwise_feasible(
    chords: Sequence[NonParallelChordRecord],
    *,
    min_spacing_km: float = 100.0,
) -> tuple[bool, str]:
    """Check spacing and non-crossing for every pair.

    Returns ``(ok, reason)``.  ``reason`` is empty when ``ok`` is True.
    """
    n = len(chords)
    for i in range(n):
        for j in range(i + 1, n):
            d = major_axis_spacing(chords[i], chords[j])
            if d + 1e-6 < min_spacing_km:
                return False, (
                    f"chord pair ({i}, {j}) spacing {d:.1f} km < "
                    f"{min_spacing_km:.0f} km (major-axis projection)"
                )
            if chords_cross_inside_polygon(chords[i], chords[j]):
                return False, (
                    f"chord pair ({i}, {j}) crosses inside the polygon"
                )
    return True, ""


def survey_distance(
    chords: Sequence[NonParallelChordRecord],
    *,
    min_spacing_km: float = 100.0,
) -> float:
    """Sum of chord lengths plus inter-chord spacer lengths.

    Chords are flown boustrophedon-style; we approximate the spacer as the
    spacing along the major axis between consecutive chords (sorted by s).
    The route builder will adjust for restricted-zone detours at build
    time.
    """
    if not chords:
        return 0.0
    by_s = sorted(chords, key=lambda c: c.s_km)
    total = sum(c.length_km for c in by_s)
    for i in range(len(by_s) - 1):
        total += max(min_spacing_km, abs(by_s[i + 1].s_km - by_s[i].s_km))
    return float(total)


# ---------------------------------------------------------------------------
# Grid generators (lazy)
# ---------------------------------------------------------------------------

DEFAULT_THETA_STEP_DEG = 7.5
DEFAULT_THETA_MAX_DEG = 30.0
DEFAULT_S_STEP_KM = 25.0


def theta_grid_deg(
    *,
    step_deg: float = DEFAULT_THETA_STEP_DEG,
    max_dev_deg: float = DEFAULT_THETA_MAX_DEG,
) -> list[float]:
    """``[-max, …, 0, …, +max]`` in ``step`` increments, inclusive."""
    n = int(round(max_dev_deg / step_deg))
    return [round(-max_dev_deg + i * step_deg, 6) for i in range(2 * n + 1)]


def s_grid_km(geom: TpvGeometry, *, step_km: float = DEFAULT_S_STEP_KM) -> list[float]:
    """Offset grid along the major axis covering ``[-0.95 a, +0.95 a]``."""
    s_max = 0.95 * geom.a_km
    n = max(0, int(s_max / step_km))
    return [round(-n * step_km + i * step_km, 6) for i in range(2 * n + 1)]


def iter_candidate_chords(
    tpv_index: int,
    tpv_polygon_km,
    geom: TpvGeometry,
    *,
    theta_step_deg: float = DEFAULT_THETA_STEP_DEG,
    theta_max_deg: float = DEFAULT_THETA_MAX_DEG,
    s_step_km: float = DEFAULT_S_STEP_KM,
    restricted_union=None,
    skip_restricted: bool = True,
) -> Iterable[NonParallelChordRecord]:
    """Lazy generator over the ``(theta, s)`` grid.

    By default chords flagged as restricted-crossing are filtered out so
    callers can treat the stream as drop-in candidates.  Pass
    ``skip_restricted=False`` if a downstream ALNS operator wants to see
    all candidates (e.g. to score them in a cost-and-penalty model).
    """
    for theta in theta_grid_deg(step_deg=theta_step_deg, max_dev_deg=theta_max_deg):
        for s in s_grid_km(geom, step_km=s_step_km):
            rec = build_chord_at(
                tpv_index, tpv_polygon_km, geom,
                theta_deg=theta, s_km=s,
                restricted_union=restricted_union,
            )
            if rec is None:
                continue
            if skip_restricted and rec.crosses_restricted:
                continue
            yield rec


# ---------------------------------------------------------------------------
# Small exhaustive enumerator (unit-test scale; ALNS won't call this)
# ---------------------------------------------------------------------------

def enumerate_chord_sets_b(
    tpv_index: int,
    tpv_polygon_km,
    geom: TpvGeometry,
    *,
    n_chord_options: Sequence[int] = (1, 2, 3, 4),
    theta_step_deg: float = DEFAULT_THETA_STEP_DEG,
    theta_max_deg: float = DEFAULT_THETA_MAX_DEG,
    s_step_km: float = DEFAULT_S_STEP_KM,
    min_spacing_km: float = 100.0,
    restricted_union=None,
    max_candidates_per_n: int | None = None,
) -> list[NonParallelChordSet]:
    """Brute-force enumerate non-parallel chord sets at small N.

    For each ``n`` in ``n_chord_options``, take every ``n``-subset of the
    candidate-chord stream that passes :func:`chord_set_pairwise_feasible`.
    Returns the best ``max_candidates_per_n`` per ``n`` (sorted by total
    chord length, descending — preferring "more survey" patterns).

    This is exponential in ``n`` and meant for tests, demos, and synthetic
    instances.  Production ALNS samples candidates on demand from
    :func:`iter_candidate_chords` instead.
    """
    candidates = list(iter_candidate_chords(
        tpv_index, tpv_polygon_km, geom,
        theta_step_deg=theta_step_deg,
        theta_max_deg=theta_max_deg,
        s_step_km=s_step_km,
        restricted_union=restricted_union,
        skip_restricted=True,
    ))
    if not candidates:
        return []

    from itertools import combinations

    out: list[NonParallelChordSet] = []
    for n in n_chord_options:
        if n <= 0 or n > len(candidates):
            continue
        feasible_sets: list[NonParallelChordSet] = []
        for combo in combinations(candidates, n):
            ok, _ = chord_set_pairwise_feasible(combo, min_spacing_km=min_spacing_km)
            if not ok:
                continue
            feasible_sets.append(NonParallelChordSet(
                tpv_index=tpv_index,
                chords=tuple(combo),
                survey_dist_km=survey_distance(combo, min_spacing_km=min_spacing_km),
            ))
            if max_candidates_per_n is not None and len(feasible_sets) >= max_candidates_per_n:
                break
        feasible_sets.sort(key=lambda cs: cs.survey_dist_km, reverse=True)
        if max_candidates_per_n is not None:
            feasible_sets = feasible_sets[:max_candidates_per_n]
        out.extend(feasible_sets)
    return out
