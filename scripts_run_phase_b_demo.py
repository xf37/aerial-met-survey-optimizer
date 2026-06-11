"""Phase B demo: refine Phase A plan #15 (and #18) with PB-2 ALNS.

Master 2026-06-10: "If case 15 produces output, try case 18 too."  This
script runs Phase A inline (~4 min), refines the requested plans, and
renders a side-by-side before-vs-after PNG per plan.
"""

from __future__ import annotations

import os
import pickle
import sys
import time

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
from shapely.ops import unary_union

sys.path.insert(0, os.path.dirname(__file__))

from multi_target_planner import (
    DEFAULT_DATA_DIR, TPV_FILENAME_PHASE0A,
    load_tpvs, load_restricted, load_atc, load_dropsonde,
    load_satellite_track, load_gatepoints, base_km,
    phase_a_envelope,
)
from multi_target_planner.phase_b_driver import refine_phase_a_plan
from multi_target_planner.sat_arc import (
    enumerate_sat_arc_candidates,
)


T_SAT_H = 4.0  # satellite passes reference point at T = 4 h after departure

# Master 2026-06-11 02:40: if Phase B runs longer than 30 min on ANY case,
# emit a loud warning so the user knows something is off.
PHASE_B_RUNTIME_WARN_SEC = 30 * 60


# 777 / 12 h mission profile (matches notebook defaults).
AIRCRAFT_SPEED_KMH = 905.0
TOTAL_FLIGHT_HOURS = 12.0
TURN_TIME_MIN = 7.5
BUDGET_KM = AIRCRAFT_SPEED_KMH * TOTAL_FLIGHT_HOURS
TURN_PENALTY_KM = TURN_TIME_MIN / 60.0 * AIRCRAFT_SPEED_KMH
SAT_MIN_LENGTH_KM = 6.0 / 60.0 * AIRCRAFT_SPEED_KMH
SAT_ARC_MAX_KM = 30.0 / 60.0 * AIRCRAFT_SPEED_KMH

# Master 2026-06-11 02:38: spec-aligned objectives (max chord_len_sum +
# coinc + dropsonde, budget as constraint).  Re-run 15, 16, 17, 18.
PLAN_INDICES = [15, 16, 17, 18]

# Budget per refinement.  Each iter calls embed_best_sat_arc + materialise
# so cost is ~5-15s/iter.  Cap iters hard so total stays under the 30-min
# runtime alarm even on big cases (8 chords).
TIME_BUDGET_SEC = 300.0   # 5 min hard cap per case (4 cases ≈ 20 min)
MAX_ITERATIONS = 30


def _objective_line(route) -> str:
    """3-objective summary: chord len sum (obj 1), coinc (obj 2), dropsonde (obj 3)."""
    if route is None:
        return "obj1 chord_len=n/a   obj2 coinc=n/a   obj3 dropsonde=n/a"
    chord_len = route.cost.chord_length_sum_km
    coinc = route.cost.coinc_dist_km
    drop = route.cost.dropsonde_dist_km
    return (
        f"obj1 chord_len_sum={chord_len:.0f} km   "
        f"obj2 coinc={coinc:.0f} km   "
        f"obj3 dropsonde={drop:.0f} km"
    )


def _sat_title_line(route) -> str:
    """One-line summary of the satellite arc embedded in ``route``."""
    if route is None or route.cost.sat_entry_km is None:
        return "sat: NOT EMBEDDED"
    ex, ey = route.cost.sat_entry_km
    xx, xy = route.cost.sat_exit_km
    coinc_km = route.cost.coinc_dist_km
    coinc_min = coinc_km / AIRCRAFT_SPEED_KMH * 60.0
    t_dep = route.cost.t_dep_h
    if t_dep is None:
        t_dep_str = "n/a"
    else:
        offset = t_dep - T_SAT_H
        sign = "+" if offset >= 0 else ""
        t_dep_str = f"T_sat{sign}{offset:.2f}h"
    return (
        f"sat IN=({ex:.0f},{ey:.0f}) km  OUT=({xx:.0f},{xy:.0f}) km  "
        f"coinc={coinc_km:.0f} km ({coinc_min:.0f} min)  T_dep={t_dep_str}"
    )


def _draw_base_layers(ax, *, tpvs, base, gatepoints, restricted, atc, dropsonde, satellite):
    for poly in dropsonde:
        xs, ys = poly.exterior.xy
        ax.fill(xs, ys, color="#22bb55", alpha=0.10, zorder=2)
    for poly in atc:
        xs, ys = poly.exterior.xy
        ax.fill(xs, ys, color="#ffe066", alpha=0.30, zorder=3)
    for poly in restricted:
        xs, ys = poly.exterior.xy
        ax.fill(xs, ys, color="#ff6666", alpha=0.40, zorder=4)
    ax.plot(satellite[:, 0], satellite[:, 1], "--", color="#8800bb",
            lw=1.0, alpha=0.55, zorder=5)
    colours = ["#1f77b4", "#d62728", "#2ca02c"]
    for tpv, col in zip(tpvs, colours):
        v = tpv.vertices_km
        closed = np.vstack([v, v[0]])
        ax.fill(v[:, 0], v[:, 1], color=col, alpha=0.18, zorder=6)
        ax.plot(closed[:, 0], closed[:, 1], color=col, lw=1.0, zorder=6.5)
        c = tpv.polygon_km.centroid
        ax.annotate(tpv.label, (c.x, c.y), color=col, fontweight="bold",
                    ha="center", va="center", fontsize=9, zorder=10)
    for gx, gy in gatepoints:
        ax.plot(gx, gy, marker="D", color="#ff7f0e", ms=7, zorder=8)
    ax.plot(base[0], base[1], marker="*", color="black", ms=14, zorder=9)


def _plot_route(ax, route, *, colour, lw_chord=2.4, lw_spacer=1.6, lw_transit=1.0,
                ls_transit="--", sat_colour="#8800bb"):
    if route is None:
        ax.text(0.5, 0.5, "no feasible route", transform=ax.transAxes,
                ha="center", va="center", fontsize=12, color="#888888")
        return
    pts = route.polyline
    kinds = route.seg_kinds
    for k, kind in enumerate(kinds):
        p1, p2 = pts[k], pts[k + 1]
        if kind == "chord":
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]], "-",
                    color=colour, lw=lw_chord, zorder=8.5)
        elif kind == "spacer":
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]], "-",
                    color=colour, lw=lw_spacer, alpha=0.85, zorder=8)
        elif kind == "sat":
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]], "-",
                    color=sat_colour, lw=2.8, alpha=0.95, zorder=9)
        else:
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]], ls_transit,
                    color=colour, lw=lw_transit, alpha=0.75, zorder=7.5)
    # Annotate sat entry / exit if present.
    if route.cost.sat_entry_km is not None and route.cost.sat_exit_km is not None:
        ex, ey = route.cost.sat_entry_km
        xx, xy = route.cost.sat_exit_km
        ax.plot(ex, ey, marker="o", color=sat_colour, ms=8,
                markerfacecolor="white", markeredgewidth=2.0, zorder=10)
        ax.plot(xx, xy, marker="s", color=sat_colour, ms=8,
                markerfacecolor="white", markeredgewidth=2.0, zorder=10)
        ax.annotate(
            f"sat IN\n({ex:.0f}, {ey:.0f}) km",
            (ex, ey), textcoords="offset points", xytext=(8, 8),
            color=sat_colour, fontsize=7, fontweight="bold", zorder=11,
        )
        ax.annotate(
            f"sat OUT\n({xx:.0f}, {xy:.0f}) km",
            (xx, xy), textcoords="offset points", xytext=(8, -16),
            color=sat_colour, fontsize=7, fontweight="bold", zorder=11,
        )


def _viewport(tpvs, base, gatepoints, pad=500.0):
    all_x = np.concatenate(
        [t.vertices_km[:, 0] for t in tpvs] + [[base[0]], gatepoints[:, 0]]
    )
    all_y = np.concatenate(
        [t.vertices_km[:, 1] for t in tpvs] + [[base[1]], gatepoints[:, 1]]
    )
    return (all_x.min() - pad, all_x.max() + pad,
            all_y.min() - pad, all_y.max() + pad)


def _render_before_after(out_path, plan, refinement, *, tpvs, base, gatepoints,
                         restricted, atc, dropsonde, satellite, plan_idx):
    fig, axes = plt.subplots(1, 2, figsize=(15, 8))
    xmin, xmax, ymin, ymax = _viewport(tpvs, base, gatepoints)

    labels = [t.label for t in tpvs]
    initial_visit = ", ".join(
        f"{labels[i]}(x{cs.n_chords})"
        for i, cs in zip(plan.tpv_indices, plan.chord_choices)
    )
    refined_visit = ", ".join(
        f"{labels[tpv_idx]}(x{len(refinement.refined_state.chords_per_tpv.get(pos, []))})"
        for pos, tpv_idx in enumerate(refinement.refined_state.visit_order)
        if refinement.refined_state.chords_per_tpv.get(pos)
    )

    # Baseline
    ax = axes[0]
    _draw_base_layers(ax, tpvs=tpvs, base=base, gatepoints=gatepoints,
                      restricted=restricted, atc=atc, dropsonde=dropsonde,
                      satellite=satellite)
    _plot_route(ax, refinement.baseline_route, colour="#444444")
    ax.set_xlim(xmin, xmax); ax.set_ylim(ymin, ymax); ax.set_aspect("equal")
    ax.grid(True, alpha=0.25)
    base_sat = _sat_title_line(refinement.baseline_route)
    base_obj = _objective_line(refinement.baseline_route)
    ax.set_title(
        f"Plan #{plan_idx}  BEFORE (Phase A baseline)\n"
        f"n_tpvs={plan.n_tpvs}  total_chords={plan.total_chords}  "
        f"max_chord={plan.max_chord_single_tpv}\n"
        f"cost={refinement.baseline_cost_km:.0f}/{BUDGET_KM:.0f} km (CONSTRAINT)\n"
        f"{base_obj}\n"
        f"{base_sat}\n"
        f"{initial_visit}",
        fontsize=8,
    )

    # Refined
    ax = axes[1]
    _draw_base_layers(ax, tpvs=tpvs, base=base, gatepoints=gatepoints,
                      restricted=restricted, atc=atc, dropsonde=dropsonde,
                      satellite=satellite)
    _plot_route(ax, refinement.refined_route, colour="#d62728")
    ax.set_xlim(xmin, xmax); ax.set_ylim(ymin, ymax); ax.set_aspect("equal")
    ax.grid(True, alpha=0.25)
    state = refinement.refined_state
    n_chords_ref = state.total_chords()
    max_chord_ref = state.max_chord_single_tpv()
    n_tpvs_ref = sum(1 for v in state.chords_per_tpv.values() if v)
    ref_sat = _sat_title_line(refinement.refined_route)
    ref_obj = _objective_line(refinement.refined_route)
    ax.set_title(
        f"Plan #{plan_idx}  AFTER (Phase B refined)\n"
        f"n_tpvs={n_tpvs_ref}  total_chords={n_chords_ref}  "
        f"max_chord={max_chord_ref}\n"
        f"cost={refinement.refined_cost_km:.0f}/{BUDGET_KM:.0f} km (CONSTRAINT)\n"
        f"{ref_obj}\n"
        f"{ref_sat}\n"
        f"{refined_visit}",
        fontsize=8,
    )

    legend_handles = [
        Line2D([0], [0], color="#444444", lw=2.4, label="Phase A baseline chord"),
        Line2D([0], [0], color="#d62728", lw=2.4, label="Phase B refined chord"),
        Line2D([0], [0], color="#444444", lw=1.0, ls="--", label="Transit (routed around restricted)"),
        Line2D([0], [0], color="#8800bb", lw=2.8, label="Sat coincidence (flown along track)"),
        Line2D([0], [0], color="#8800bb", lw=1.0, ls="--", alpha=0.55, label="Satellite track (full)"),
        Line2D([0], [0], marker="o", color="#8800bb", ms=8, markerfacecolor="white",
               markeredgewidth=2.0, ls="none", label="Sat entry (pt_a)"),
        Line2D([0], [0], marker="s", color="#8800bb", ms=8, markerfacecolor="white",
               markeredgewidth=2.0, ls="none", label="Sat exit (pt_b)"),
        Line2D([0], [0], marker="*", color="black", ms=10, ls="none", label="BASE"),
        Line2D([0], [0], marker="D", color="#ff7f0e", ms=7, ls="none", label="Gatepoint"),
        Patch(facecolor="#ff6666", alpha=0.40, label="Restricted airspace"),
        Patch(facecolor="#ffe066", alpha=0.30, label="ATC (Gander, x1.35)"),
        Patch(facecolor="#22bb55", alpha=0.10, label="Dropsonde zones"),
    ]
    fig.legend(handles=legend_handles, loc="lower center", ncol=5, fontsize=8,
               bbox_to_anchor=(0.5, -0.01), framealpha=0.95)

    fig.suptitle(
        f"FPO-6 / PB-2 Phase B refinement on plan #{plan_idx} "
        f"(777 / 12 h profile, {refinement.alns_result.iterations_run} ALNS iters "
        f"in {refinement.alns_result.elapsed_sec:.1f} s)",
        fontsize=12,
    )
    plt.tight_layout(rect=(0, 0.04, 1, 0.96))
    plt.savefig(out_path, dpi=140, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved {out_path}")


def main():
    print("[Phase B demo] Loading real data…")
    tpvs, frame = load_tpvs(DEFAULT_DATA_DIR, TPV_FILENAME_PHASE0A)
    restricted = load_restricted(DEFAULT_DATA_DIR, frame)
    atc = load_atc(DEFAULT_DATA_DIR, frame)
    dropsonde = load_dropsonde(DEFAULT_DATA_DIR, frame)
    satellite = load_satellite_track(DEFAULT_DATA_DIR, frame)
    gatepoints = load_gatepoints(DEFAULT_DATA_DIR, frame)
    base = base_km(frame)
    restricted_union = unary_union(restricted) if restricted else None
    atc_union = unary_union(atc) if atc else None
    dropsonde_union = unary_union(dropsonde) if dropsonde else None

    cache_path = "phase_a_archive.pkl"
    if os.path.exists(cache_path):
        print(f"[Phase B demo] Loading cached Phase A result from {cache_path}…")
        with open(cache_path, "rb") as f:
            result = pickle.load(f)
        print(f"[Phase B demo] archive size = {result.archive_size}")
    else:
        print("[Phase B demo] Running Phase A (no cache)…")
        t0 = time.time()
        result = phase_a_envelope(
            tpvs=tpvs,
            base_km=base, gatepoints_km=gatepoints,
            restricted_union=restricted_union, atc_union=atc_union,
            sat_track_xy=satellite, enforce_sat_constraint=True,
            n_chord_options=(1, 2, 3, 4),
            angle_devs_deg=(-5.0, 0.0, 5.0),
            min_spacing_km=100.0,
            budget_km=BUDGET_KM, turn_penalty_km=TURN_PENALTY_KM,
            aircraft_speed_kmh=AIRCRAFT_SPEED_KMH,
            sat_min_length_km=SAT_MIN_LENGTH_KM,
            sat_arc_max_km=SAT_ARC_MAX_KM,
            k_per_bundle=999,
        )
        print(f"[Phase B demo] Phase A took {time.time() - t0:.0f} s; "
              f"archive size = {result.archive_size}")
        with open(cache_path, "wb") as f:
            pickle.dump(result, f)
        print(f"[Phase B demo] Cached Phase A result to {cache_path}")

    # Collect plans in the same ordering as the all-18 figure (subset size,
    # subset id, cost).
    seen = set()
    plans_in_order = []
    for p in result.bundle_b1.plans + result.bundle_b2.plans:
        sig = p.diversity_signature
        if sig in seen:
            continue
        seen.add(sig)
        plans_in_order.append(p)
    labels = [t.label for t in tpvs]
    plans_in_order.sort(key=lambda p: (
        len(p.tpv_indices),
        tuple(sorted(labels[i] for i in p.tpv_indices)),
        p.total_cost_km,
    ))

    print(f"[Phase B demo] {len(plans_in_order)} unique plans surfaced.")

    # PB-3: enumerate sat candidates ONCE so each plan can pick the best
    # (entry, exit, arc_km) at insertion time.
    sat_candidates = enumerate_sat_arc_candidates(
        sat_track_xy=satellite,
        sat_min_length_km=SAT_MIN_LENGTH_KM,
        sat_arc_max_km=SAT_ARC_MAX_KM,
    )
    print(f"[Phase B demo] {len(sat_candidates)} sat (entry, arc) candidates "
          f"enumerated along satellite track")

    for plan_idx in PLAN_INDICES:
        if plan_idx > len(plans_in_order):
            print(f"[Phase B demo] plan #{plan_idx} not in archive, skipping.")
            continue
        plan = plans_in_order[plan_idx - 1]
        print(f"\n[Phase B demo] Refining plan #{plan_idx}: "
              f"n_tpvs={plan.n_tpvs} total={plan.total_chords} "
              f"cost={plan.total_cost_km:.0f} km")
        t0 = time.time()
        refinement = refine_phase_a_plan(
            plan, tpvs,
            base_km=base, gatepoints_km=gatepoints,
            restricted_union=restricted_union, atc_union=atc_union,
            turn_penalty_km=TURN_PENALTY_KM,
            budget_km=BUDGET_KM,
            min_spacing_km=100.0,
            time_budget_sec=TIME_BUDGET_SEC,
            max_iterations=MAX_ITERATIONS,
            rng_seed=plan_idx,
            sat_candidates=sat_candidates,
            sat_track_xy=satellite,
            t_sat_h=T_SAT_H,
            aircraft_speed_kmh=AIRCRAFT_SPEED_KMH,
            parallel_only=True,  # Master 2026-06-10: lock to parallel chords
            freeze_chord_count=True,  # Master 2026-06-10: case identity = subset + chord counts
            dropsonde_union=dropsonde_union,  # Master 2026-06-11: objective 3
        )
        dt = time.time() - t0
        ref_coinc = (refinement.refined_route.cost.coinc_dist_km
                     if refinement.refined_route is not None else 0.0)
        ref_tdep = (refinement.refined_route.cost.t_dep_h
                    if refinement.refined_route is not None else None)
        print(f"  ALNS finished in {dt:.1f} s. "
              f"d_chords = {refinement.chord_delta:+d}, "
              f"d_cost = {refinement.cost_delta_km:+.0f} km, "
              f"sat_coinc = {ref_coinc:.0f} km, "
              f"T_dep = {ref_tdep if ref_tdep is None else f'{ref_tdep:.2f}h'}")
        if dt > PHASE_B_RUNTIME_WARN_SEC:
            print(f"  *** WARNING: Phase B on plan #{plan_idx} took "
                  f"{dt:.0f} s (> {PHASE_B_RUNTIME_WARN_SEC} s threshold). "
                  f"Investigate ALNS / materialise cost. ***")
        out_path = f"11_phase_b_plan_{plan_idx:02d}_before_after.png"
        _render_before_after(
            out_path, plan, refinement,
            tpvs=tpvs, base=base, gatepoints=gatepoints,
            restricted=restricted, atc=atc, dropsonde=dropsonde,
            satellite=satellite, plan_idx=plan_idx,
        )


if __name__ == "__main__":
    main()
