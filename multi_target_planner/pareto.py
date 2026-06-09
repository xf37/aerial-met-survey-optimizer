"""Pareto archive + 2-bundle curation (FPO-5 / Phase 0a / 0a.2).

A Phase A plan is described by :class:`Plan` and ranked on four axes:

1. ``n_tpvs``                  — number of TPVs surveyed (maximise)
2. ``max_chord_single_tpv``    — most chords on any one TPV (maximise)
3. ``total_chords``            — sum of chord counts across all visited TPVs (maximise)
4. ``-total_cost_km``          — negative cost (so all axes are maximised)

The archive stores only non-dominated plans, then exposes two named bundles:

* **B1 — Max-TPV**:   top 5 plans by ``(n_tpvs, total_chords, -total_cost)``
* **B2 — Max-chord**: top 5 plans by ``(max_chord_single_tpv, n_tpvs, -total_cost)``

Master 2026-06-09 set B1 and B2 as the priority bundles; the other three from
the original FPO-3 design (max-coincidence / shortest / most-robust) are
deferred to 0a.3+.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence

import numpy as np

from multi_target_planner.chord_layout import ChordSet
from multi_target_planner.route_builder import BuiltRoute, CostBreakdown


# ---------------------------------------------------------------------------
# Plan record
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Plan:
    """A feasible Phase A plan."""
    tpv_indices: tuple[int, ...]            # which TPVs (in visit order)
    chord_choices: tuple[ChordSet, ...]      # ChordSet per visited TPV (parallel to tpv_indices)
    route: BuiltRoute                        # built polyline + cost

    @property
    def n_tpvs(self) -> int:
        return len(self.tpv_indices)

    @property
    def chord_counts(self) -> tuple[int, ...]:
        return tuple(cs.n_chords for cs in self.chord_choices)

    @property
    def total_chords(self) -> int:
        return sum(self.chord_counts)

    @property
    def max_chord_single_tpv(self) -> int:
        return max(self.chord_counts) if self.chord_counts else 0

    @property
    def total_cost_km(self) -> float:
        return self.route.cost.total_km

    @property
    def diversity_signature(self) -> tuple:
        """Hashable signature for de-duplication / diversity gating.

        Identical (tpv_set, ordered_chord_counts, ordered_angle_devs) → same
        signature.  Order of TPVs matters since a different visit order is a
        genuinely different plan.
        """
        return tuple(
            (idx, cs.n_chords, round(cs.angle_dev_deg, 4))
            for idx, cs in zip(self.tpv_indices, self.chord_choices)
        )

    # Convenience for sorting.
    def _b1_key(self) -> tuple:
        # B1: max TPVs, then total chords, then min cost.
        return (-self.n_tpvs, -self.total_chords, self.total_cost_km)

    def _b2_key(self) -> tuple:
        # B2: max single-TPV chord, then n_tpvs, then min cost.
        return (-self.max_chord_single_tpv, -self.n_tpvs, self.total_cost_km)


# ---------------------------------------------------------------------------
# Pareto dominance
# ---------------------------------------------------------------------------

def _objectives(plan: Plan) -> tuple[float, float, float, float]:
    """The 4-tuple used for dominance checking — all axes are *maximised*."""
    return (
        float(plan.n_tpvs),
        float(plan.max_chord_single_tpv),
        float(plan.total_chords),
        -float(plan.total_cost_km),
    )


def dominates(a: Plan, b: Plan) -> bool:
    """``a`` dominates ``b`` iff a is ≥ b on every axis and > on at least one."""
    oa, ob = _objectives(a), _objectives(b)
    return all(x >= y for x, y in zip(oa, ob)) and any(x > y for x, y in zip(oa, ob))


# ---------------------------------------------------------------------------
# Archive
# ---------------------------------------------------------------------------

@dataclass
class ParetoArchive:
    """Non-dominated set keyed by :class:`Plan.diversity_signature`."""
    plans: dict = field(default_factory=dict)   # signature -> Plan
    n_offered: int = 0
    n_dominated: int = 0
    n_duplicates: int = 0

    def offer(self, plan: Plan) -> bool:
        """Add ``plan`` if non-dominated; evict any plans it now dominates.

        Returns ``True`` when the plan ends up in the archive.
        """
        self.n_offered += 1
        sig = plan.diversity_signature

        # De-duplicate by signature, keeping the cheaper plan (smaller total cost).
        if sig in self.plans:
            self.n_duplicates += 1
            existing = self.plans[sig]
            if plan.total_cost_km < existing.total_cost_km:
                self.plans[sig] = plan
                return True
            return False

        # Dominance pass.
        to_remove: list = []
        for other in self.plans.values():
            if dominates(other, plan):
                self.n_dominated += 1
                return False
            if dominates(plan, other):
                to_remove.append(other.diversity_signature)
        for k in to_remove:
            del self.plans[k]

        self.plans[sig] = plan
        return True

    def __len__(self) -> int:
        return len(self.plans)

    def all_plans(self) -> list[Plan]:
        return list(self.plans.values())


# ---------------------------------------------------------------------------
# Curation — 2 bundles
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Bundle:
    name: str
    plans: tuple[Plan, ...]


def curate_bundles(archive: ParetoArchive, *, k_per_bundle: int = 5) -> tuple[Bundle, Bundle]:
    """Pick top-K for each of the two priority bundles.

    Bundles draw from the same archive; the same plan can appear in both if it
    tops both ranking orders — that's intentional.  Within each bundle plans
    are returned in ranked order (best first).
    """
    plans = archive.all_plans()
    b1 = Bundle(
        name="B1 (max TPVs flown)",
        plans=tuple(sorted(plans, key=Plan._b1_key)[:k_per_bundle]),
    )
    b2 = Bundle(
        name="B2 (max chords on a single TPV)",
        plans=tuple(sorted(plans, key=Plan._b2_key)[:k_per_bundle]),
    )
    return b1, b2
