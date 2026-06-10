"""Standalone renderer for Phase A snapshots (FPO-6 / PB-0).

Loads a ``.pkl`` snapshot produced by :func:`multi_target_planner.io.save_phase_a_run`
and emits PNG figures *without* re-running the Phase A model.  Master's γ
("二合一") request maps to the ``overview`` style: one grid PNG of the full
archive plus one detail PNG per plan.

Examples
--------
* Grid only::

    python render_envelope.py archive.pkl --style grid -o grid.png

* Detail PNG for plan #5::

    python render_envelope.py archive.pkl --style plan --plan-idx 5 -o plan_5.png

* γ overview (one grid + one per-plan PNG into a directory)::

    python render_envelope.py archive.pkl --style overview --output-dir out/
"""

from __future__ import annotations

import argparse
import os
import sys

import matplotlib
import numpy as np

# Use a non-interactive backend by default; callers can set MPLBACKEND if they
# want an interactive window.
if os.environ.get("MPLBACKEND") is None:
    matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

from multi_target_planner.io import PhaseARun, load_phase_a_run


# ---------------------------------------------------------------------------
# Shared drawing primitives
# ---------------------------------------------------------------------------

TPV_COLORS = ["#1f77b4", "#d62728", "#2ca02c"]


def _viewport_bounds(run: PhaseARun, pad: float = 500.0):
    all_x = np.concatenate(
        [t.vertices_km[:, 0] for t in run.tpvs]
        + [[run.base_km[0]], run.gatepoints_km[:, 0]]
    )
    all_y = np.concatenate(
        [t.vertices_km[:, 1] for t in run.tpvs]
        + [[run.base_km[1]], run.gatepoints_km[:, 1]]
    )
    return (all_x.min() - pad, all_x.max() + pad,
            all_y.min() - pad, all_y.max() + pad)


def _draw_base_layers(ax, run: PhaseARun, *, show_sat: bool = True):
    for poly in run.dropsonde_polygons:
        xs, ys = poly.exterior.xy
        ax.fill(xs, ys, color="#22bb55", alpha=0.10, zorder=2)
    for poly in run.atc_polygons:
        xs, ys = poly.exterior.xy
        ax.fill(xs, ys, color="#ffe066", alpha=0.30, zorder=3)
        ax.plot(xs, ys, color="#b8860b", lw=0.8, zorder=3.5)
    for poly in run.restricted_polygons:
        xs, ys = poly.exterior.xy
        ax.fill(xs, ys, color="#ff6666", alpha=0.40, zorder=4)
        ax.plot(xs, ys, color="#cc0000", lw=0.8, zorder=4.5)
    if show_sat:
        ax.plot(run.sat_track_xy[:, 0], run.sat_track_xy[:, 1], "--",
                color="#8800bb", lw=1.0, alpha=0.6, zorder=5)
    for tpv, col in zip(run.tpvs, TPV_COLORS):
        v = tpv.vertices_km
        closed = np.vstack([v, v[0]])
        ax.fill(v[:, 0], v[:, 1], color=col, alpha=0.18, zorder=6)
        ax.plot(closed[:, 0], closed[:, 1], color=col, lw=1.0, zorder=6.5)
        c = tpv.polygon_km.centroid
        ax.annotate(tpv.label, (c.x, c.y), color=col, fontweight="bold",
                    ha="center", va="center", fontsize=8, zorder=10)
    for i, (gx, gy) in enumerate(run.gatepoints_km):
        ax.plot(gx, gy, marker="D", color="#ff7f0e", ms=7, zorder=8)
    ax.plot(run.base_km[0], run.base_km[1], marker="*", color="black", ms=14, zorder=9)


def _plot_plan_route(ax, plan):
    pts = plan.route.polyline
    kinds = plan.route.seg_kinds
    for k, kind in enumerate(kinds):
        p1, p2 = pts[k], pts[k + 1]
        if kind == "chord":
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]], "-",
                    color="#d62728", lw=2.2, zorder=8.5)
        elif kind == "spacer":
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]], "-",
                    color="#0066cc", lw=1.5, zorder=7.5)
        else:
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]], "--",
                    color="#333333", lw=1.0, alpha=0.85, zorder=7.5)


def _legend_handles():
    return [
        Line2D([0], [0], color="#d62728", lw=2.2, label="TPV survey chord"),
        Line2D([0], [0], color="#0066cc", lw=1.5, label="Inter-chord spacer"),
        Line2D([0], [0], color="#333333", lw=1.0, ls="--",
               label="Transit (auto-routed around restricted)"),
        Line2D([0], [0], color="#8800bb", lw=1.0, ls="--", alpha=0.6,
               label="Satellite track"),
        Line2D([0], [0], marker="*", color="black", ms=10, ls="none",
               label="BASE"),
        Line2D([0], [0], marker="D", color="#ff7f0e", ms=7, ls="none",
               label="Gatepoint"),
        Patch(facecolor="#ff6666", alpha=0.40, label="Restricted airspace"),
        Patch(facecolor="#ffe066", alpha=0.30, label="ATC zone (Gander, x1.35)"),
        Patch(facecolor="#22bb55", alpha=0.10, label="Dropsonde zones"),
    ]


# ---------------------------------------------------------------------------
# Plan collection — every unique Pareto-archive plan surfaced by the bundles
# ---------------------------------------------------------------------------

def _all_archive_plans(run: PhaseARun) -> list:
    """Return every Pareto-archive plan that the bundles surfaced (deduped)."""
    seen, out = set(), []
    for p in run.envelope.bundle_b1.plans + run.envelope.bundle_b2.plans:
        sig = p.diversity_signature
        if sig in seen:
            continue
        seen.add(sig)
        out.append(p)
    return out


# ---------------------------------------------------------------------------
# Style 1 — grid (the entire archive in one figure)
# ---------------------------------------------------------------------------

def render_grid(run: PhaseARun, out_path: str, *, ncols: int | None = None) -> str:
    plans = _all_archive_plans(run)
    if not plans:
        raise ValueError("Archive is empty; nothing to render")
    labels = [t.label for t in run.tpvs]

    plans.sort(key=lambda p: (len(p.tpv_indices),
                              tuple(sorted(labels[i] for i in p.tpv_indices)),
                              p.total_cost_km))

    n_plans = len(plans)
    if ncols is None:
        ncols = max(1, min(6, int(np.ceil(np.sqrt(n_plans * 1.6)))))
    nrows = (n_plans + ncols - 1) // ncols

    fig, axes = plt.subplots(nrows, ncols,
                             figsize=(2.8 * ncols, 2.8 * nrows),
                             squeeze=False)
    xmin, xmax, ymin, ymax = _viewport_bounds(run)
    budget = run.mission_profile.get("BUDGET_KM", 0.0)
    for idx in range(nrows * ncols):
        r, c = divmod(idx, ncols)
        ax = axes[r, c]
        if idx >= n_plans:
            ax.set_visible(False)
            continue
        plan = plans[idx]
        _draw_base_layers(ax, run)
        _plot_plan_route(ax, plan)
        subset = "+".join(labels[i] for i in plan.tpv_indices)
        visit = ", ".join(f"{labels[i]}(x{cs.n_chords})"
                          for i, cs in zip(plan.tpv_indices, plan.chord_choices))
        title = (
            f"#{idx + 1}  {{{subset}}}\n"
            f"n_tpvs={plan.n_tpvs}  total={plan.total_chords}  "
            f"max={plan.max_chord_single_tpv}\n"
            f"cost={plan.total_cost_km:.0f}/{budget:.0f} km\n{visit}"
        )
        ax.set_title(title, fontsize=7)
        ax.set_xlim(xmin, xmax)
        ax.set_ylim(ymin, ymax)
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.2)
        ax.tick_params(axis="both", which="both", labelsize=6)

    fig.legend(handles=_legend_handles(), loc="lower center", ncol=5,
               fontsize=8, bbox_to_anchor=(0.5, -0.005), framealpha=0.95)
    suptitle = (
        f"FPO-6 / PB-0 grid view — {n_plans} plans   "
        f"BUDGET={budget:.0f} km   "
        f"SPEED={run.mission_profile.get('AIRCRAFT_SPEED_KMH', 0):.0f} km/h   "
        f"HOURS={run.mission_profile.get('TOTAL_FLIGHT_HOURS', 0):.0f}\n"
        f"snapshot: {os.path.basename(out_path)}    "
        f"taken: {run.timestamp_iso}"
    )
    fig.suptitle(suptitle, fontsize=11)
    plt.tight_layout(rect=(0, 0.02, 1, 0.96))
    plt.savefig(out_path, dpi=140, bbox_inches="tight")
    plt.close(fig)
    return out_path


# ---------------------------------------------------------------------------
# Style 2 — per-plan detail (Master's γ second half)
# ---------------------------------------------------------------------------

def render_plan_detail(run: PhaseARun, plan_idx: int, out_path: str) -> str:
    plans = _all_archive_plans(run)
    if not 0 <= plan_idx < len(plans):
        raise IndexError(f"plan_idx {plan_idx} out of range [0, {len(plans)})")
    plan = plans[plan_idx]
    labels = [t.label for t in run.tpvs]

    fig, ax = plt.subplots(figsize=(8, 8))
    _draw_base_layers(ax, run)
    _plot_plan_route(ax, plan)
    xmin, xmax, ymin, ymax = _viewport_bounds(run, pad=400.0)
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.25)
    ax.set_xlabel("Easting (km)")
    ax.set_ylabel("Northing (km)")

    subset = "+".join(labels[i] for i in plan.tpv_indices)
    visit = ", ".join(f"{labels[i]}(x{cs.n_chords},ang={cs.angle_dev_deg:+.0f})"
                      for i, cs in zip(plan.tpv_indices, plan.chord_choices))
    cost_pct = plan.total_cost_km / max(1.0, run.mission_profile.get("BUDGET_KM", 1.0)) * 100
    ax.set_title(
        f"Plan #{plan_idx + 1}   {{{subset}}}\n"
        f"n_tpvs={plan.n_tpvs}   total_chords={plan.total_chords}   "
        f"max_chord={plan.max_chord_single_tpv}\n"
        f"cost={plan.total_cost_km:.0f}/{run.mission_profile.get('BUDGET_KM', 0):.0f} km   "
        f"({cost_pct:.0f}% of budget)\n"
        f"{visit}",
        fontsize=11,
    )
    fig.legend(handles=_legend_handles(), loc="lower center", ncol=5,
               fontsize=9, bbox_to_anchor=(0.5, -0.005), framealpha=0.95)
    plt.tight_layout(rect=(0, 0.05, 1, 0.96))
    plt.savefig(out_path, dpi=140, bbox_inches="tight")
    plt.close(fig)
    return out_path


# ---------------------------------------------------------------------------
# Style 3 — overview (Master's γ default: grid + per-plan PNGs)
# ---------------------------------------------------------------------------

def render_overview(run: PhaseARun, out_dir: str) -> list[str]:
    os.makedirs(out_dir, exist_ok=True)
    written: list[str] = []
    written.append(render_grid(run, os.path.join(out_dir, "_grid.png")))
    n_plans = len(_all_archive_plans(run))
    for i in range(n_plans):
        written.append(render_plan_detail(
            run, i, os.path.join(out_dir, f"plan_{i + 1:02d}.png")))
    return written


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="Render PNG figures from a Phase A archive snapshot",
    )
    p.add_argument("archive", help="path to a .pkl snapshot of a PhaseARun")
    p.add_argument("--style", choices=("grid", "plan", "overview"),
                   default="overview",
                   help="grid: single figure of the whole archive; "
                        "plan: detail of one plan (--plan-idx required); "
                        "overview: grid PNG + per-plan PNGs into --output-dir "
                        "(default; matches Master's γ pick).")
    p.add_argument("--plan-idx", type=int, default=0,
                   help="plan index (0-based) when --style plan")
    p.add_argument("--output", "-o", default=None,
                   help="output PNG path (grid / plan styles)")
    p.add_argument("--output-dir", default=None,
                   help="output directory for --style overview")
    p.add_argument("--no-verify-sha1", action="store_true",
                   help="skip the .sha1 digest check at load time")
    return p.parse_args(argv)


def main(argv=None) -> int:
    args = _parse_args(argv)
    run = load_phase_a_run(args.archive, verify_sha1=not args.no_verify_sha1)
    if args.style == "grid":
        out = args.output or args.archive.replace(".pkl", "_grid.png")
        path = render_grid(run, out)
        print(f"Saved {path}")
    elif args.style == "plan":
        out = args.output or args.archive.replace(
            ".pkl", f"_plan_{args.plan_idx + 1:02d}.png")
        path = render_plan_detail(run, args.plan_idx, out)
        print(f"Saved {path}")
    else:  # overview
        out_dir = args.output_dir or args.archive.replace(".pkl", "_panels")
        paths = render_overview(run, out_dir)
        for p in paths:
            print(f"Saved {p}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
