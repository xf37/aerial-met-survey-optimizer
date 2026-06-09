"""Angle-gated turn-penalty model (FPO-5 / Phase 0a / S1).

Replaces the placeholder ``n_turns_est = n + 2`` used in 08_multiple_tpv_route_planning.ipynb.
The new model computes the actual deviation-from-straight at every interior vertex of a
flight polyline and applies the turn-penalty multiplier only when that deviation exceeds
a configurable threshold (default 30°, per Master directive 2026-06-09).
"""

from __future__ import annotations

import numpy as np


_EPS = 1e-9


def compute_turn_angles_deg(polyline_xy: np.ndarray) -> np.ndarray:
    """Return the deviation-from-straight angle (degrees) at each interior vertex.

    Parameters
    ----------
    polyline_xy : np.ndarray
        Shape (N, 2). Sequence of 2-D points forming the flight path.

    Returns
    -------
    np.ndarray
        Shape (max(0, N - 2),). Angles in degrees, all in [0, 180].
        A value of 0 means the path is locally straight; 180 means a full U-turn.
        Degenerate inputs (fewer than 3 points, or a zero-length adjacent segment)
        contribute 0 at the affected vertex rather than NaN.
    """
    pts = np.asarray(polyline_xy, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 2:
        raise ValueError(
            f"polyline_xy must have shape (N, 2); got {pts.shape}"
        )
    n = pts.shape[0]
    if n < 3:
        return np.empty((0,), dtype=np.float64)

    segs = pts[1:] - pts[:-1]                       # (N-1, 2)
    lens = np.linalg.norm(segs, axis=1)             # (N-1,)
    v1 = segs[:-1]                                  # (N-2, 2)
    v2 = segs[1:]                                   # (N-2, 2)
    l1 = lens[:-1]
    l2 = lens[1:]

    denom = l1 * l2
    safe = denom > _EPS                             # both adjacent segments non-degenerate
    dot = np.einsum("ij,ij->i", v1, v2)
    cos_theta = np.where(safe, np.clip(dot / np.where(safe, denom, 1.0), -1.0, 1.0), 1.0)
    angles_rad = np.arccos(cos_theta)
    return np.degrees(angles_rad)


def turn_penalty_cost(
    angles_deg: np.ndarray,
    turn_penalty_km: float,
    threshold_deg: float,
    factor: float,
) -> float:
    """Sum of per-turn penalty costs in km (Reading B per Master 2026-06-09).

    For each turn with deviation ``angle`` (degrees) the contribution is::

        0.0                          if angle <= threshold_deg     (small turn, FREE)
        turn_penalty_km * factor     if angle  > threshold_deg     (sharp turn)

    Small turns are FREE — this matches the existing 07 / plan_route_field.py
    idiom (`count_turns_in_path`: turns at or below the threshold contribute
    nothing). Master's 2026-06-09 directive layers the `factor` multiplier on
    top of the existing free-small-turn semantics so sharp turns now cost
    ``turn_penalty_km * factor`` instead of ``turn_penalty_km`` alone.

    The threshold is a STRICT greater-than: ``angle == threshold_deg`` is
    treated as a small turn. Flip to ``>=`` if the spec is later clarified.

    Parameters
    ----------
    angles_deg : np.ndarray
        Output of :func:`compute_turn_angles_deg`.
    turn_penalty_km : float
        Base penalty added per sharp turn, in km. Defaults to ≈106 km
        (7.5 min × 850 km/h) in upstream callers.
    threshold_deg : float
        Strict greater-than threshold separating small (free) from sharp turns.
    factor : float
        Multiplier applied to ``turn_penalty_km`` for sharp turns.

    Returns
    -------
    float
        Total turn-penalty cost in km. Returns 0.0 for an empty input or for a
        path with only small turns.
    """
    angles = np.asarray(angles_deg, dtype=np.float64)
    if angles.size == 0:
        return 0.0
    sharp = angles > threshold_deg
    n_sharp = int(np.sum(sharp))
    return float(n_sharp) * turn_penalty_km * factor
