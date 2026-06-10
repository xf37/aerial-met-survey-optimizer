"""Phase B demo: refine Phase A plan #15 (and #18) with PB-2 ALNS.

Master 2026-06-10: "If case 15 produces output, try case 18 too."  This
script runs Phase A inline (~4 min), refines the requested plans, and
renders a side-by-side before-vs-after PNG per plan.
"""

from __future__ import annotations

import os
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


# 777 / 12 h mission profile (matches notebook defaults).
AIRCRAFT_SPEED_KMH = 905.0
TOTAL_FLIGHT_HOURS = 12.0
TURN_TIME_MIN = 7.5
BUDGET_KM = AIRCRAFT_SPEED_KMH * TOTAL_FLIGHT_HOURS
TURN_PENALTY_KM = TURN_TIME_MIN / 60.0 * AIRCRAFT_SPEED_KMH
SAT_MIN_LENGTH_KM = 6.0 / 60.0 * AIRCRAFT_SPEED_KMH
SAT_ARC_MAX_KM = 30.0 / 60.0 * AIRCRAFT_SPEED_KMH

# Master's requested plans (1-based indexing in the sorted-by-subset archive view).
PLAN_INDICES = [15, 18]

# Budget per refinement.
TIME_BUDGET_SEC = 180.0
MAX_ITERATIONS = 4000


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


def _plot_route(ax, route, *, colour, lw_chord=2.4, lw_spacer=1.6, lw_transit=1.0, ls_transit="--"):
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
        else:
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]], ls_transit,
                    color=colour, lw=lw_transit, alpha=0.75, zorder=7.5)


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
    ax.set_title(
        f"Plan #{plan_idx}  BEFORE (Phase A baseline)\n"
        f"n_tpvs={plan.n_tpvs}  total_chords={plan.total_chords}  "
        f"max_chord={plan.max_chord_single_tpv}\n"
        f"cost={refinement.baseline_cost_km:.0f}/{BUDGET_KM:.0f} km\n"
        f"{initial_visit}",
        fontsize=10,
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
    ax.set_title(
        f"Plan #{plan_idx}  AFTER (Phase B refined)\n"
        f"n_tpvs={n_tpvs_ref}  total_chords={n_chords_ref}  "
        f"max_chord={max_chord_ref}\n"
        f"cost={refinement.refined_cost_km:.0f}/{BUDGET_KM:.0f} km   "
        f"(Δ chords {refinement.chord_delta:+d}, Δ cost {refinement.cost_delta_km:+.0f} km)\n"
        f"{refined_visit}",
        fontsize=10,
    )

    legend_handles = [
        Line2D([0], [0], color="#444444", lw=2.4, label="Phase A baseline chord"),
        Line2D([0], [0], color="#d62728", lw=2.4, label="Phase B refined chord"),
        Line2D([0], [0], color="#444444", lw=1.0, ls="--", label="Transit (routed around restricted)"),
        Line2D([0], [0], color="#8800bb", lw=1.0, ls="--", alpha=0.55, label="Satellite track"),
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

    print("[Phase B demo] Running Phase A…")
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
            min_spacing_km=100.0,
            time_budget_sec=TIME_BUDGET_SEC,
            max_iterations=MAX_ITERATIONS,
            rng_seed=plan_idx,
        )
        dt = time.time() - t0
        print(f"  ALNS finished in {dt:.1f} s. "
              f"Δ chords = {refinement.chord_delta:+d}, "
              f"Δ cost = {refinement.cost_delta_km:+.0f} km")
        out_path = f"11_phase_b_plan_{plan_idx:02d}_before_after.png"
        _render_before_after(
            out_path, plan, refinement,
            tpvs=tpvs, base=base, gatepoints=gatepoints,
            restricted=restricted, atc=atc, dropsonde=dropsonde,
            satellite=satellite, plan_idx=plan_idx,
        )


if __name__ == "__main__":
    main()
