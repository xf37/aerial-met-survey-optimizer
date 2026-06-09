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
from multi_target_planner.data_loader import (
    DEFAULT_DATA_DIR,
    BASE_WGS84_LON,
    BASE_WGS84_LAT,
    TPV_FILENAME_PHASE0A,
    LocalFrame,
    TPV,
    verify_data_sources,
    load_tpvs,
    load_polygons_km,
    load_restricted,
    load_atc,
    load_dropsonde,
    load_satellite_track,
    load_gatepoints,
    base_km,
)

__all__ = [
    "compute_turn_angles_deg",
    "turn_penalty_cost",
    "DEFAULT_DATA_DIR",
    "BASE_WGS84_LON",
    "BASE_WGS84_LAT",
    "TPV_FILENAME_PHASE0A",
    "LocalFrame",
    "TPV",
    "verify_data_sources",
    "load_tpvs",
    "load_polygons_km",
    "load_restricted",
    "load_atc",
    "load_dropsonde",
    "load_satellite_track",
    "load_gatepoints",
    "base_km",
]
