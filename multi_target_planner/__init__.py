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
from multi_target_planner.chord_layout import (
    ChordSet,
    TpvGeometry,
    enumerate_chord_sets,
    fit_tpv_geometry,
    precompute_all_chord_sets,
)
from multi_target_planner.route_builder import (
    BuiltRoute,
    CostBreakdown,
    DEFAULT_ATC_FACTOR,
    DEFAULT_BUDGET_KM,
    DEFAULT_TURN_FACTOR,
    DEFAULT_TURN_PENALTY_KM,
    DEFAULT_TURN_THRESHOLD_DEG,
    build_route,
    compute_route_cost,
)
from multi_target_planner.feasibility import (
    FeasibilityResult,
    is_route_feasible,
)
from multi_target_planner.pareto import (
    Bundle,
    ParetoArchive,
    Plan,
    curate_bundles,
    dominates,
)
from multi_target_planner.phase_a import (
    EnvelopeResult,
    format_envelope_summary,
    phase_a_envelope,
)
from multi_target_planner.sat_arc import (
    DEFAULT_ENTRY_STEP_KM,
    DEFAULT_SAT_ARC_MAX_KM,
    DEFAULT_SAT_ARC_STEP_KM,
    DEFAULT_SAT_MIN_LENGTH_KM,
    DEFAULT_T_SAT_H,
    SatArcCandidate,
    SatFeasibility,
    check_sat_feasibility,
    enumerate_sat_arc_candidates,
)
from multi_target_planner.verifier import (
    IndependentMissionState,
    VerifierReport,
    verify_bundle,
    verify_plan,
)
from multi_target_planner.chord_layout_b import (
    DEFAULT_S_STEP_KM,
    DEFAULT_THETA_MAX_DEG,
    DEFAULT_THETA_STEP_DEG,
    NonParallelChordRecord,
    NonParallelChordSet,
    build_chord_at,
    chord_set_pairwise_feasible,
    chords_cross_inside_polygon,
    enumerate_chord_sets_b,
    iter_candidate_chords,
    major_axis_spacing,
    s_grid_km,
    survey_distance,
    theta_grid_deg,
)

__all__ = [
    # turn penalty
    "compute_turn_angles_deg",
    "turn_penalty_cost",
    # data loader
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
    # chord layout
    "ChordSet",
    "TpvGeometry",
    "enumerate_chord_sets",
    "fit_tpv_geometry",
    "precompute_all_chord_sets",
    # route builder
    "BuiltRoute",
    "CostBreakdown",
    "DEFAULT_ATC_FACTOR",
    "DEFAULT_BUDGET_KM",
    "DEFAULT_TURN_FACTOR",
    "DEFAULT_TURN_PENALTY_KM",
    "DEFAULT_TURN_THRESHOLD_DEG",
    "build_route",
    "compute_route_cost",
    # feasibility
    "FeasibilityResult",
    "is_route_feasible",
    # pareto
    "Bundle",
    "ParetoArchive",
    "Plan",
    "curate_bundles",
    "dominates",
    # phase A
    "EnvelopeResult",
    "format_envelope_summary",
    "phase_a_envelope",
    # sat arc
    "DEFAULT_ENTRY_STEP_KM",
    "DEFAULT_SAT_ARC_MAX_KM",
    "DEFAULT_SAT_ARC_STEP_KM",
    "DEFAULT_SAT_MIN_LENGTH_KM",
    "DEFAULT_T_SAT_H",
    "SatArcCandidate",
    "SatFeasibility",
    "check_sat_feasibility",
    "enumerate_sat_arc_candidates",
    # verifier
    "IndependentMissionState",
    "VerifierReport",
    "verify_bundle",
    "verify_plan",
    # PB-1 non-parallel chord layout
    "DEFAULT_S_STEP_KM",
    "DEFAULT_THETA_MAX_DEG",
    "DEFAULT_THETA_STEP_DEG",
    "NonParallelChordRecord",
    "NonParallelChordSet",
    "build_chord_at",
    "chord_set_pairwise_feasible",
    "chords_cross_inside_polygon",
    "enumerate_chord_sets_b",
    "iter_candidate_chords",
    "major_axis_spacing",
    "s_grid_km",
    "survey_distance",
    "theta_grid_deg",
]
