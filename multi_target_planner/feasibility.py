"""Feasibility check for built routes.

LOCK 1 (Master 2026-06-09): the planner MUST NOT surface infeasible plans.
Anything failing this check is rejected before it can enter any Pareto
archive or bundle.

Phase A *does* re-route around restricted airspace at build time via
:func:`multi_target_planner.visibility_graph.route_around_restricted`.  The
per-segment crossing check below is therefore a defence-in-depth pass: it
catches the corner cases where the visibility-graph router gave up
(``BuiltRoute`` is ``None`` because an endpoint is boxed in) or where a
returned polyline still clips a small restricted polygon (e.g. when
``shapely.touches`` and ``shapely.intersects`` disagree on a numerical
boundary).  If both routing and this check agree the segment is clear, the
plan is accepted.
"""

from __future__ import annotations

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

from multi_target_planner.route_builder import BuiltRoute, DEFAULT_BUDGET_KM


@dataclass(frozen=True)
class FeasibilityResult:
    ok: bool
    reason: str
    failing_segment_index: int = -1   # -1 if not applicable


def is_route_feasible(
    route: BuiltRoute,
    *,
    restricted_union=None,
    gatepoints_km: np.ndarray | None = None,
    budget_km: float = DEFAULT_BUDGET_KM,
) -> FeasibilityResult:
    """Hard-feasibility check against budget + restricted airspace + gatepoints.

    The check is composed of three layers:

    1. **Budget**: ``cost.total_km`` must be ``<= budget_km``.
    2. **Restricted airspace**: no straight-line segment in the polyline may
       cross the restricted union.
    3. **Gatepoints**: every gatepoint must appear (within 0.5 km) on the
       polyline.  Phase A puts them in by construction, so this is mainly a
       defence against malformed callers.
    """
    if route.cost.total_km > budget_km:
        return FeasibilityResult(
            ok=False,
            reason=f"cost {route.cost.total_km:.1f} km exceeds budget {budget_km:.0f} km",
        )

    if restricted_union is not None and not restricted_union.is_empty:
        pts = route.polyline
        for i in range(pts.shape[0] - 1):
            p1, p2 = pts[i], pts[i + 1]
            if prf.seg_crosses_restricted(p1, p2, restricted_union):
                return FeasibilityResult(
                    ok=False,
                    reason=f"segment {i} crosses restricted airspace",
                    failing_segment_index=i,
                )

    if gatepoints_km is not None and gatepoints_km.size > 0:
        pts = route.polyline
        missing = []
        for j, gp in enumerate(np.asarray(gatepoints_km).reshape(-1, 2)):
            d = np.linalg.norm(pts - gp, axis=1)
            if d.min() > 0.5:
                missing.append(j)
        if missing:
            return FeasibilityResult(
                ok=False,
                reason=f"gatepoint(s) {missing} not visited",
            )

    return FeasibilityResult(ok=True, reason="")
