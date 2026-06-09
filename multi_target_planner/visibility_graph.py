"""Tiny visibility-graph router for restricted-airspace avoidance.

Phase A's direct-line transits trip the LOCK 1 feasibility check whenever the
straight line between two stops would cut a restricted polygon.  For
real-data instances (3-TPV mission, 13 restricted polys around the Gander
FIR) this kills every plan.

This module provides a *per-call* visibility graph: given two endpoints
(``p1, p2``) and the restricted union, it builds a small graph over
``[p1, p2] + restricted_polygon_vertices``, then Dijkstras to find the
shortest path that avoids every restricted polygon.  The returned polyline
is suitable for substitution into the route builder.

The implementation is a verbatim port of 07's ``_route_chord_via_vertices``
(see ``07_tpv_route_planning.ipynb`` cell with the same name) — Luc's 0a.2
ticket lets us borrow operators-only from 07.  We rely on the borrowed
``seg_crosses_restricted`` from ``plan_route_field`` for the visibility
check.
"""

from __future__ import annotations

import heapq
import os
import sys
from typing import Iterable, Sequence

import numpy as np

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

import plan_route_field as prf  # noqa: E402


def _polygon_vertices(restricted_union) -> list[np.ndarray]:
    """Flatten every exterior ring of every polygon in ``restricted_union``."""
    if restricted_union is None or restricted_union.is_empty:
        return []
    geoms = (
        list(restricted_union.geoms)
        if hasattr(restricted_union, "geoms")
        else [restricted_union]
    )
    verts: list[np.ndarray] = []
    for g in geoms:
        if hasattr(g, "exterior"):
            for v in g.exterior.coords[:-1]:
                verts.append(np.asarray(v, dtype=np.float64))
    return verts


_route_cache: dict[tuple, np.ndarray | None] = {}


def clear_route_cache() -> None:
    """Drop the memoization cache (only useful for tests; ordinary callers
    don't need this because the cache key now includes the restricted-union
    geometry hash — different unions can't collide)."""
    _route_cache.clear()


def _restricted_signature(restricted_union) -> str:
    """Stable signature for ``restricted_union`` used in the cache key.

    Uses Shapely's WKT representation, which is deterministic for a given
    geometry and changes the moment any vertex moves.  Empty / None unions
    map to a single fixed signature so they share a cache.
    """
    if restricted_union is None or restricted_union.is_empty:
        return "__empty__"
    return restricted_union.wkt


def route_around_restricted(
    p1: np.ndarray,
    p2: np.ndarray,
    restricted_union,
) -> np.ndarray | None:
    """Return the shortest ``p1 -> p2`` polyline avoiding restricted airspace.

    Returns
    -------
    polyline
        Shape ``(K, 2)`` array, with ``polyline[0] == p1`` and
        ``polyline[-1] == p2``.  ``K == 2`` when the direct line is already
        clear; ``K > 2`` when detour vertices are inserted.  ``None`` when
        no path exists (the route is fully boxed in).
    """
    p1 = np.asarray(p1, dtype=np.float64).reshape(2)
    p2 = np.asarray(p2, dtype=np.float64).reshape(2)

    # Trivial direct case (no obstacle data).
    if restricted_union is None or restricted_union.is_empty:
        return np.vstack([p1, p2])

    # Memoize on endpoint coords (rounded to 0.01 km so floating-point noise
    # doesn't generate spurious cache misses) AND the restricted-union
    # signature so changing the union invalidates entries automatically.
    # The endpoint pair is stored in canonical (smaller-first) order so
    # direction-flipped queries hit too.
    a = (round(float(p1[0]), 2), round(float(p1[1]), 2))
    b = (round(float(p2[0]), 2), round(float(p2[1]), 2))
    flipped = b < a
    sig = _restricted_signature(restricted_union)
    key = (sig, b, a) if flipped else (sig, a, b)
    cached = _route_cache.get(key)
    if cached is not None:
        return cached if not flipped else cached[::-1]
    if key in _route_cache:
        # Cached None (unreachable).
        return None

    if not prf.seg_crosses_restricted(p1, p2, restricted_union):
        result = np.vstack([p1, p2])
        _route_cache[key] = result if not flipped else result[::-1]
        return result

    # Small Dijkstra over [p1, p2] + restricted polygon vertices.
    verts = _polygon_vertices(restricted_union)
    nodes: list[np.ndarray] = [p1, p2] + verts
    n = len(nodes)

    adj: dict[int, list[tuple[int, float]]] = {i: [] for i in range(n)}
    for i in range(n):
        for j in range(i + 1, n):
            if not prf.seg_crosses_restricted(nodes[i], nodes[j], restricted_union):
                d = float(np.linalg.norm(nodes[j] - nodes[i]))
                adj[i].append((j, d))
                adj[j].append((i, d))

    dist: dict[int, float] = {0: 0.0}
    prev: dict[int, int | None] = {0: None}
    heap: list[tuple[float, int]] = [(0.0, 0)]
    while heap:
        dd, u = heapq.heappop(heap)
        if dd > dist.get(u, float("inf")):
            continue
        if u == 1:
            break
        for v, w in adj[u]:
            nd = dd + w
            if nd < dist.get(v, float("inf")):
                dist[v] = nd
                prev[v] = u
                heapq.heappush(heap, (nd, v))

    if 1 not in dist:
        _route_cache[key] = None
        return None

    path_nodes: list[np.ndarray] = []
    cur: int | None = 1
    while cur is not None:
        path_nodes.append(nodes[cur])
        cur = prev.get(cur)
    path_nodes.reverse()
    result = np.vstack(path_nodes)
    _route_cache[key] = result if not flipped else result[::-1]
    return result
