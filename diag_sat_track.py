"""
Quick diagnostic: show the satellite track, all sat candidates, and their
distances to BASE. Answers whether there are arc positions closer to BASE
that the optimizer should have considered.
"""
import pickle
import numpy as np
import matplotlib.pyplot as plt
from math import cos, radians

with open('mission_cache.pkl', 'rb') as f:
    cache = pickle.load(f)

seg_pts  = cache['sat_track']['seg_pts']
cum_arc  = cache['sat_track']['cum_arc']
BASE     = np.asarray(cache['BASE'], float)

# Reconstruct all candidate entry positions and arc midpoints
cands = cache['sat_candidates']       # (pa, pb_min, arc_km) list
variants = cache['sat_arc_variants']  # entry_key -> [(pb, arc_km), ...]

pa_pts   = np.array([c[0] for c in cands])
pb_pts   = np.array([c[1] for c in cands])
mids     = (pa_pts + pb_pts) / 2.0
d_base   = np.linalg.norm(pa_pts - BASE, axis=1)
d_mid    = np.linalg.norm(mids  - BASE, axis=1)

print(f"Satellite track: {len(seg_pts)} vertices, total arc={cum_arc[-1]:.0f} km")
print(f"All candidates: {len(cands)}")
print(f"\nClosest candidate entries to BASE:")
order = np.argsort(d_base)
for i in order[:10]:
    pa, pb, arc = cands[i]
    pa_key = tuple(np.round(pa, 3))
    n_ext = len(variants.get(pa_key, []))
    print(f"  pa=({pa[0]:7.1f},{pa[1]:7.1f})  pb=({pb[0]:7.1f},{pb[1]:7.1f})"
          f"  d_base={d_base[i]:.0f} km  variants={n_ext}")

print(f"\nCurrent solution arc entry (closest match by position):")
# wp8 = (-141.60, -350.46) = pa (sat exit, natural start of arc)
wp8 = np.array([-141.60, -350.46])
wp7 = np.array([-228.95, -766.36])
diffs = np.linalg.norm(pa_pts - wp8, axis=1)
k = int(np.argmin(diffs))
print(f"  pa=({pa_pts[k,0]:.1f},{pa_pts[k,1]:.1f})  d_base={d_base[k]:.0f} km (closest candidate to wp8)")

# ── Plot ──────────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(1, 2, figsize=(16, 8))

# Left: full satellite track + all candidates
ax = axes[0]
ax.set_title('Satellite track + all candidates\n(colour = distance of pa to BASE)', fontsize=9)
ax.plot(seg_pts[:, 0], seg_pts[:, 1], 'gray', lw=1, ls='--', zorder=1,
        label='Satellite track')
sc = ax.scatter(pa_pts[:, 0], pa_pts[:, 1], c=d_base, cmap='RdYlGn_r',
                s=30, zorder=4, label='Candidate entries (pa)')
plt.colorbar(sc, ax=ax, label='Distance pa→BASE (km)')
ax.scatter(*BASE, s=200, marker='*', color='black', zorder=6, label='BASE')
ax.scatter(wp7[0], wp7[1], s=100, color='red', marker='o', zorder=5, label='wp7 (current entry)')
ax.scatter(wp8[0], wp8[1], s=100, color='red', marker='s', zorder=5, label='wp8 (current exit)')
ax.set_aspect('equal'); ax.set_xlabel('East (km)'); ax.set_ylabel('North (km)')
ax.legend(fontsize=7); ax.grid(True, lw=0.3, alpha=0.5)

# Right: cumulative arc position along track vs distance to BASE
ax = axes[1]
ax.set_title('Distance of satellite track to BASE\nvs arc position t (km along track)', fontsize=9)
t_vals = np.linspace(0, float(cum_arc[-1]), 500)
from plan_route_field import arc_interp
pts = np.array([arc_interp(seg_pts, cum_arc, t) for t in t_vals])
d_to_base = np.linalg.norm(pts - BASE, axis=1)
ax.plot(t_vals, d_to_base, 'b-', lw=1.5)
# Mark current arc extent
t_wp8_idx = int(np.argmin(np.linalg.norm(seg_pts - wp8, axis=1)))
t_wp7_idx = int(np.argmin(np.linalg.norm(seg_pts - wp7, axis=1)))
t_wp8 = float(cum_arc[t_wp8_idx])
t_wp7 = float(cum_arc[t_wp7_idx])
ax.axvline(t_wp8, color='red', ls='--', label=f'wp8 t≈{t_wp8:.0f} km')
ax.axvline(t_wp7, color='orange', ls='--', label=f'wp7 t≈{t_wp7:.0f} km')
ax.axhline(np.linalg.norm(wp8-BASE), color='red', ls=':', alpha=0.5)
ax.axhline(np.linalg.norm(wp7-BASE), color='orange', ls=':', alpha=0.5)
# Mark global minimum
t_min_idx = int(np.argmin(d_to_base))
ax.scatter(t_vals[t_min_idx], d_to_base[t_min_idx], s=80, color='green', zorder=5,
           label=f'Closest to BASE: t={t_vals[t_min_idx]:.0f}, d={d_to_base[t_min_idx]:.0f} km')
ax.set_xlabel('Arc position t (km along track)'); ax.set_ylabel('Distance to BASE (km)')
ax.legend(fontsize=8); ax.grid(True, lw=0.3, alpha=0.5)

plt.tight_layout()
plt.savefig('diag_sat_track.png', dpi=150, bbox_inches='tight')
plt.close()
print('\nSaved diag_sat_track.png')
