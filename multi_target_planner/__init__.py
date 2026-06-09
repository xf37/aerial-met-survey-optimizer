"""Multi-TPV flight-route planner for the aerial met survey optimiser.

This package is new — added under FPO-5 (Phase 0a). It does NOT modify any
existing module in the repository; instead it borrows functions from them
and builds a new fast pipeline for multi-target route planning.

Phase 0a sub-task S1 is the turn-penalty rewrite: an angle-gated turn cost
replacing the placeholder `n_turns_est = n + 2` previously used in 08.
"""

from multi_target_planner.turn_penalty import (
    compute_turn_angles_deg,
    turn_penalty_cost,
)

__all__ = [
    "compute_turn_angles_deg",
    "turn_penalty_cost",
]
