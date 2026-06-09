"""TPV chord-layout enumeration (FPO-5 / Phase 0a / 0a.2).

For each TPV polygon we build a set of candidate "chord configurations":

* ``ChordSet(n_chords, angle_dev, chords)`` — a particular survey pattern,
  where ``chords`` are the actual line segments inside the polygon.

The geometry helpers come straight from ``plan_route_field.py``
(``fit_pca_ellipse`` and ``generate_candidate_chords``) — those are 07's
operators-only borrow path Luc approved for 0a.2.  We do not modify the
underlying file (LOCK 2).
"""

from __future__ import annotations

import sys
import os
from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np

# plan_route_field.py lives at the repo root next to this package; ensure the
# parent directory is on sys.path so the module is importable when this
# package is used from a notebook.
_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

import plan_route_field as prf  # noqa: E402  -- intentional; deferred to extend sys.path


# ---------------------------------------------------------------------------
# Public dataclasses
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class TpvGeometry:
    """PCA ellipse fit for one TPV polygon."""
    tpv_label: str
    center_km: np.ndarray         # (2,)
    a_km: float                    # semi-major axis length (km, 2*sqrt(eigval))
    b_km: float                    # semi-minor axis length
    phi_rad: float                 # major-axis orientation (rad)
    aspect_ratio: float            # a / b

    @property
    def is_circle_like(self) -> bool:
        """Spec calls a TPV with aspect ratio <1.25 a circle (Master 2026-06-09)."""
        return self.aspect_ratio < 1.25


@dataclass(frozen=True)
class ChordSet:
    """One survey pattern for a TPV: ``n_chords`` parallel chords at ``angle_dev``."""
    tpv_index: int
    n_chords: int
    angle_dev_deg: float
    chords: tuple                  # list of dicts from generate_candidate_chords
    survey_dist_km: float          # sum of chord lengths + inter-chord spacing
    n_turns_inside: int            # turns at chord endpoints (entry, between, exit)

    @property
    def is_feasible(self) -> bool:
        """A ChordSet is feasible iff it actually contains ``n_chords`` chords."""
        return len(self.chords) == self.n_chords


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def fit_tpv_geometry(tpv_label: str, vertices_km: np.ndarray) -> TpvGeometry:
    """Fit PCA ellipse — wraps ``plan_route_field.fit_pca_ellipse``."""
    center, a, b, phi = prf.fit_pca_ellipse(vertices_km)
    aspect = (a / b) if b > 1e-6 else float("inf")
    return TpvGeometry(
        tpv_label=tpv_label,
        center_km=np.asarray(center, dtype=np.float64),
        a_km=float(a),
        b_km=float(b),
        phi_rad=float(phi),
        aspect_ratio=float(aspect),
    )


def enumerate_chord_sets(
    tpv_index: int,
    tpv_polygon_km,
    geom: TpvGeometry,
    *,
    n_chord_options: Sequence[int] = (1, 2, 3, 4),
    angle_devs_deg: Sequence[float] = (-5.0, 0.0, 5.0),
    min_spacing_km: float = 100.0,
    restricted_union=None,
) -> list[ChordSet]:
    """Return every viable (n_chords, angle_dev) pattern for one TPV.

    ``min_spacing_km`` is the parallel-chord spacing (07 uses
    ``MIN_LEG_SPACING_KM = 100``; the spec calls for ``≥ 100 km``).

    Implementation note: ``plan_route_field.generate_candidate_chords`` returns
    the *full* set of candidate chord lines at each angle deviation.  We then
    select the largest valid subset of ``n_chord`` chords whose offsets are
    at least ``min_spacing_km`` apart (greedy from centre-most outward) — this
    is what 07 does in ``best_multi_chord_route``.
    """
    sets: list[ChordSet] = []

    for dev in angle_devs_deg:
        all_chords = prf.generate_candidate_chords(
            tpv_polygon_km,
            geom.center_km,
            geom.a_km,
            geom.phi_rad,
            min_spacing=min_spacing_km,
            angle_devs_deg=[dev],
            restricted_geom=restricted_union,
        )
        if not all_chords:
            continue

        # Sort by signed offset (centre-most first), pick greedily.
        all_chords.sort(key=lambda c: abs(c["offset"]))

        for n in n_chord_options:
            picked: list[dict] = []
            picked_offsets: list[float] = []
            for c in all_chords:
                off = c["offset"]
                if all(abs(off - po) >= min_spacing_km - 1e-6 for po in picked_offsets):
                    picked.append(c)
                    picked_offsets.append(off)
                    if len(picked) == n:
                        break
            if len(picked) != n:
                continue   # cannot fit n chords with the required spacing

            # Order by offset along the major axis so chords are flown in order.
            picked.sort(key=lambda c: c["offset"])
            survey_dist = sum(c["length"] for c in picked) + (n - 1) * min_spacing_km
            # Turns inside a TPV: enter + (n-1) U-turns between chords + exit = n + 1.
            # Reading B will only penalise the sharp ones — but for the rough
            # envelope scan we treat all inside-TPV turns as sharp (≥ 30°).
            n_turns_inside = n + 1
            sets.append(
                ChordSet(
                    tpv_index=tpv_index,
                    n_chords=n,
                    angle_dev_deg=float(dev),
                    chords=tuple(picked),
                    survey_dist_km=float(survey_dist),
                    n_turns_inside=int(n_turns_inside),
                )
            )

    # De-duplicate by (n_chords, angle_dev_deg) — keep the first occurrence.
    seen = set()
    unique: list[ChordSet] = []
    for s in sets:
        key = (s.n_chords, round(s.angle_dev_deg, 6))
        if key in seen:
            continue
        seen.add(key)
        unique.append(s)
    return unique


def precompute_all_chord_sets(
    tpvs,
    restricted_union=None,
    *,
    n_chord_options=(1, 2, 3, 4),
    angle_devs_deg=(-5.0, 0.0, 5.0),
    min_spacing_km: float = 100.0,
) -> tuple[list[TpvGeometry], list[list[ChordSet]]]:
    """Build ``(geom_for_each_tpv, chord_sets_for_each_tpv)``.

    The returned ``chord_sets`` is parallel to ``tpvs``: ``chord_sets[i]`` is
    the list of viable patterns for ``tpvs[i]``.  An empty list means *no*
    feasible pattern under the given spacing / restricted-zone constraints,
    which the caller must surface as "this TPV cannot be observed".
    """
    geoms: list[TpvGeometry] = []
    all_sets: list[list[ChordSet]] = []
    for i, tpv in enumerate(tpvs):
        geom = fit_tpv_geometry(tpv.label, tpv.vertices_km)
        sets = enumerate_chord_sets(
            tpv_index=i,
            tpv_polygon_km=tpv.polygon_km,
            geom=geom,
            n_chord_options=n_chord_options,
            angle_devs_deg=angle_devs_deg,
            min_spacing_km=min_spacing_km,
            restricted_union=restricted_union,
        )
        geoms.append(geom)
        all_sets.append(sets)
    return geoms, all_sets
