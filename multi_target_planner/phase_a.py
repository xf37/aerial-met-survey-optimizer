"""Phase A — envelope scan (FPO-5 / Phase 0a / 0a.2).

What it does
============
Given a small set of TPVs (Phase 0a targets N=3), Phase A enumerates every
combination of:

* a TPV subset (including the empty one — though we never surface it),
* a visit order over that subset, and
* a ChordSet choice for each TPV in the order,

then for each combination builds the route polyline, evaluates the cost
breakdown, and runs the feasibility check.  Feasible plans are dropped into a
:class:`ParetoArchive`.  Once enumeration is done the archive is curated into
the two Master-priority bundles (B1 = max TPVs, B2 = max single-TPV chord).

Why exhaustive
==============
For N ≤ 5 with up to 4 chord options × 3 angle deviations = 12 patterns per
TPV, the worst-case combination count is

    Σ_{k=1..N}  C(N, k) · k! · 12^k

which for N=5 is roughly 4 × 10^6 — well within a 5-minute Phase A budget on
a single core.  ALNS is overkill for this scale and is deferred to 0a.3 /
larger-N follow-ups.

Output
======
:class:`EnvelopeResult` carries the headline numbers Master asked for —
``n_max_tpvs`` and ``chord_max_per_tpv`` — plus the two curated bundles, plus
counters for the eyeball-test (``n_evaluated``, ``n_feasible``, etc.).
"""

from __future__ import annotations

import itertools
import time
from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np

from multi_target_planner.chord_layout import (
    ChordSet,
    TpvGeometry,
    precompute_all_chord_sets,
)
from multi_target_planner.data_loader import TPV
from multi_target_planner.feasibility import (
    FeasibilityResult,
    is_route_feasible,
)
from multi_target_planner.pareto import (
    Bundle,
    ParetoArchive,
    Plan,
    curate_bundles,
)
from multi_target_planner.sat_arc import (
    DEFAULT_AIRCRAFT_SPEED_KMH as SAT_DEFAULT_AIRCRAFT_SPEED_KMH,
    DEFAULT_ENTRY_STEP_KM,
    DEFAULT_SAT_ARC_MAX_KM,
    DEFAULT_SAT_ARC_STEP_KM,
    DEFAULT_SAT_MIN_LENGTH_KM,
    DEFAULT_T_SAT_H,
    SatArcCandidate,
    check_sat_feasibility,
    enumerate_sat_arc_candidates,
)
from multi_target_planner.route_builder import (
    DEFAULT_AIRCRAFT_SPEED_KMH,
    DEFAULT_ATC_FACTOR,
    DEFAULT_BUDGET_KM,
    DEFAULT_TURN_FACTOR,
    DEFAULT_TURN_PENALTY_KM,
    DEFAULT_TURN_THRESHOLD_DEG,
    Stop,
    build_route,
    make_gatepoint_stop,
    make_tpv_stop,
)


def _interleave_with_gatepoints(
    tpv_stops: Sequence[Stop],
    gp_stops: Sequence[Stop],
) -> Iterable[list[Stop]]:
    """Yield every way to place ``gp_stops`` among ``tpv_stops`` keeping
    ``tpv_stops`` order fixed.

    For ``k`` TPVs and ``m`` gatepoints this produces ``m! * C(k+m, m)``
    distinct stop sequences (``20`` when ``k=3, m=2``).
    """
    k = len(tpv_stops)
    m = len(gp_stops)
    if m == 0:
        yield list(tpv_stops)
        return
    n_total = k + m
    for gp_perm in itertools.permutations(gp_stops):
        for gp_positions in itertools.combinations(range(n_total), m):
            seq: list[Stop | None] = [None] * n_total
            for pos, gp in zip(gp_positions, gp_perm):
                seq[pos] = gp
            tpv_iter = iter(tpv_stops)
            for i in range(n_total):
                if seq[i] is None:
                    seq[i] = next(tpv_iter)
            yield seq   # type: ignore[misc]


@dataclass(frozen=True)
class EnvelopeResult:
    n_max_tpvs: int                              # max TPVs any feasible plan flies
    chord_max_per_tpv: tuple[int, ...]           # max chords feasible for each TPV (length = N_TPV)
    bundle_b1: Bundle                            # max-TPV bundle
    bundle_b2: Bundle                            # max-single-TPV-chord bundle
    archive_size: int
    n_enumerated: int
    n_feasible: int                              # feasibility ignoring sat constraint
    n_feasible_with_sat: int                     # feasibility AFTER sat hard constraint (v2 metric)
    n_sat_rejected: int                          # plans dropped at sat hard constraint
    elapsed_sec: float
    geoms: tuple[TpvGeometry, ...]
    sat_constraint_enforced: bool                # True for v2 runs, False for v1

    @property
    def has_any_feasible(self) -> bool:
        return self.n_feasible_with_sat > 0


def phase_a_envelope(
    tpvs: Sequence[TPV],
    base_km: np.ndarray,
    gatepoints_km: np.ndarray,
    *,
    restricted_union=None,
    atc_union=None,
    sat_track_xy: np.ndarray | None = None,
    enforce_sat_constraint: bool = False,
    t_sat_h: float = DEFAULT_T_SAT_H,
    aircraft_speed_kmh: float = SAT_DEFAULT_AIRCRAFT_SPEED_KMH,
    sat_min_length_km: float = DEFAULT_SAT_MIN_LENGTH_KM,
    sat_arc_max_km: float = DEFAULT_SAT_ARC_MAX_KM,
    sat_arc_step_km: float = DEFAULT_SAT_ARC_STEP_KM,
    sat_entry_step_km: float = DEFAULT_ENTRY_STEP_KM,
    n_chord_options: Sequence[int] = (1, 2, 3, 4),
    angle_devs_deg: Sequence[float] = (-5.0, 0.0, 5.0),
    min_spacing_km: float = 100.0,
    budget_km: float = DEFAULT_BUDGET_KM,
    turn_penalty_km: float = DEFAULT_TURN_PENALTY_KM,
    turn_threshold_deg: float = DEFAULT_TURN_THRESHOLD_DEG,
    turn_factor: float = DEFAULT_TURN_FACTOR,
    atc_factor: float = DEFAULT_ATC_FACTOR,
    k_per_bundle: int = 5,
) -> EnvelopeResult:
    """Run the envelope scan.

    The function is deterministic for a given input.  All cost / penalty
    parameters default to the values baked into ``precompute_mission.py``.
    """
    t0 = time.time()
    n = len(tpvs)

    geoms, all_sets = precompute_all_chord_sets(
        tpvs,
        restricted_union=restricted_union,
        n_chord_options=n_chord_options,
        angle_devs_deg=angle_devs_deg,
        min_spacing_km=min_spacing_km,
    )

    sat_candidates: list[SatArcCandidate] = []
    if enforce_sat_constraint and sat_track_xy is not None and sat_track_xy.shape[0] >= 2:
        sat_candidates = enumerate_sat_arc_candidates(
            sat_track_xy,
            entry_step_km=sat_entry_step_km,
            arc_step_km=sat_arc_step_km,
            sat_min_length_km=sat_min_length_km,
            sat_arc_max_km=sat_arc_max_km,
        )

    archive = ParetoArchive()
    chord_max_per_tpv = [0] * n
    n_enumerated = 0
    n_feasible = 0
    n_feasible_with_sat = 0
    n_sat_rejected = 0

    gp_arr = (
        np.asarray(gatepoints_km, dtype=np.float64).reshape(-1, 2)
        if gatepoints_km is not None
        else np.empty((0, 2), dtype=np.float64)
    )
    n_gp = gp_arr.shape[0]
    gp_stops_all = [make_gatepoint_stop(g) for g in gp_arr]

    # Enumerate every non-empty subset, every TPV permutation, every chord
    # combo, and every interleaving of the gatepoints into the visit sequence.
    for k in range(1, n + 1):
        for subset in itertools.combinations(range(n), k):
            if any(not all_sets[i] for i in subset):
                continue
            for order in itertools.permutations(subset):
                for chord_combo in itertools.product(*(all_sets[i] for i in order)):
                    tpv_stops = [make_tpv_stop(i, cs) for i, cs in zip(order, chord_combo)]
                    for stops in _interleave_with_gatepoints(tpv_stops, gp_stops_all):
                        n_enumerated += 1
                        route = build_route(
                            base=base_km,
                            stops=stops,
                            restricted_union=restricted_union,
                            atc_union=atc_union,
                            turn_penalty_km=turn_penalty_km,
                            turn_threshold_deg=turn_threshold_deg,
                            turn_factor=turn_factor,
                            atc_factor=atc_factor,
                        )
                        if route is None:
                            continue  # boxed-in by restricted airspace
                        feasibility = is_route_feasible(
                            route,
                            restricted_union=restricted_union,
                            gatepoints_km=gp_arr if n_gp > 0 else None,
                            budget_km=budget_km,
                        )
                        if not feasibility.ok:
                            continue
                        n_feasible += 1
                        # S2-min sat hard constraint check (Phase A.v2 only).
                        if enforce_sat_constraint:
                            sat = check_sat_feasibility(
                                route,
                                base_km,
                                sat_candidates,
                                budget_km=budget_km,
                                aircraft_speed_kmh=aircraft_speed_kmh,
                                t_sat_h=t_sat_h,
                                sat_min_length_km=sat_min_length_km,
                            )
                            if not sat.feasible:
                                n_sat_rejected += 1
                                continue
                        plan = Plan(
                            tpv_indices=tuple(order),
                            chord_choices=tuple(chord_combo),
                            route=route,
                        )
                        archive.offer(plan)
                        n_feasible_with_sat += 1
                        for tpv_i, cs in zip(order, chord_combo):
                            if cs.n_chords > chord_max_per_tpv[tpv_i]:
                                chord_max_per_tpv[tpv_i] = cs.n_chords

    plans = archive.all_plans()
    n_max_tpvs = max((p.n_tpvs for p in plans), default=0)
    b1, b2 = curate_bundles(archive, k_per_bundle=k_per_bundle)

    return EnvelopeResult(
        n_max_tpvs=int(n_max_tpvs),
        chord_max_per_tpv=tuple(chord_max_per_tpv),
        bundle_b1=b1,
        bundle_b2=b2,
        archive_size=len(archive),
        n_enumerated=n_enumerated,
        n_feasible=n_feasible,
        n_feasible_with_sat=n_feasible_with_sat,
        n_sat_rejected=n_sat_rejected,
        elapsed_sec=time.time() - t0,
        geoms=tuple(geoms),
        sat_constraint_enforced=enforce_sat_constraint,
    )


# ---------------------------------------------------------------------------
# Pretty-printer (used by the notebook + CLI smoke)
# ---------------------------------------------------------------------------

def format_envelope_summary(result: EnvelopeResult, tpv_labels: Sequence[str]) -> str:
    v_tag = "v2 (sat enforced)" if result.sat_constraint_enforced else "v1 (no sat constraint)"
    lines = [
        "=" * 72,
        f"Phase A envelope scan  [{v_tag}]  ({result.elapsed_sec:.2f} s)",
        "=" * 72,
        f"Enumerated combinations    : {result.n_enumerated:,}",
        f"Feasible (geom + budget)   : {result.n_feasible:,}",
        f"Sat-rejected               : {result.n_sat_rejected:,}",
        f"Feasible AFTER sat (final) : {result.n_feasible_with_sat:,}",
        f"Pareto archive size        : {result.archive_size}",
        "",
        f"n_max_tpvs (most TPVs in one feasible flight) : {result.n_max_tpvs}",
        "chord_max_per_tpv:",
    ]
    for label, m in zip(tpv_labels, result.chord_max_per_tpv):
        lines.append(f"  {label:>10s}: {m} chord(s)")
    lines.append("")

    for bundle in (result.bundle_b1, result.bundle_b2):
        lines.append(f"--- {bundle.name}  (top {len(bundle.plans)}) ---")
        if not bundle.plans:
            lines.append("  (no feasible plan)")
            continue
        for rank, plan in enumerate(bundle.plans, start=1):
            visit = ", ".join(
                f"{tpv_labels[i]}(x{cs.n_chords})"
                for i, cs in zip(plan.tpv_indices, plan.chord_choices)
            )
            lines.append(
                f"  #{rank}: n_tpvs={plan.n_tpvs}  total_chords={plan.total_chords}  "
                f"max_chord={plan.max_chord_single_tpv}  "
                f"cost={plan.total_cost_km:.0f} km  "
                f"({visit})"
            )
        lines.append("")

    lines.append("=" * 72)
    return "\n".join(lines)
