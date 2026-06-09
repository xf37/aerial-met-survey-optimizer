# Data sources for `multi_target_planner` (FPO-5 / Phase 0a step 0)

Every input file required by the multi-target planner lives in
`C:\nasa-ffp-nurture\test_data\`, the same `DATA_DIR` used by
`07_tpv_route_planning.ipynb` and `precompute_mission.py`.  No synthetic,
dummy, or self-generated input is allowed — Phase 0a mirrors 07's data sources
file-for-file.

If any row below cannot be located on disk during a Phase 0a run, the planner
**stops and reports** rather than substituting a placeholder.

## Master-designated override for Phase 0a

The TPV input for Phase 0a is the **three-TPV** file:

* `Contour_2_14_18_3_3TPVs.shp`  (per Master directive 2026-06-09)

07 uses the single-TPV variant `Contour_2_14_18_3.shp`.  All other inputs are
identical — only the TPV polygon source changes.

## Full input-file mapping

| Role | File (under `C:\nasa-ffp-nurture\test_data\`) | Used by 07 at | Used by `precompute_mission.py` at | Notes |
|---|---|---|---|---|
| TPV contour (Phase 0a override) | `Contour_2_14_18_3_3TPVs.shp` | (new — Phase 0a only) | (new — Phase 0a only) | 3 TPV polygons in one shapefile; lon/lat (EPSG:4326) |
| TPV contour (07 single-TPV) | `Contour_2_14_18_3.shp` | Cell 5, line "Contour_2_14_18_3.shp" | n/a | Kept for back-compat / regression comparison only |
| Restricted airspace (no-fly) | `Restricted_Airspace_Footprint.shp` | Cell 12, `_ra_path` | Line 166 | Filtered to 2000 km of TPV centroid in 07 |
| ATC zone (Gander FIR, soft penalty ×1.35) | `Ganderairtraff_Polygon.shp` | Cell 12, `_gander_path` | Line 170 | No spatial filter |
| Dropsonde deployment zones | `dropsonde_available.shp` | Cell 21, `_drop_path` | Lines 173–176 | Filtered to 2000 km of TPV centroid; passive metric (does not drive routing) |
| Satellite track (EarthCARE overpass 2016-05-16) | `satellite overpass_0516_simplified.shp` | Cell ~17, `_sat_gdf` | Line 185 | Used for sat-coincidence segment and `T_dep` time-axis constraint |
| Mandatory gatepoints | `gatepoints.shp` | Cell 5, `_gp_gdf` | (loaded inline at use site) | Hard constraint — every plan must traverse all gatepoints |

## Hard-coded (not file-based) inputs that must stay consistent with 07

| Constant | Value | Source | Reason it cannot drift |
|---|---|---|---|
| `BASE` (GOOSE BAY airbase) | lon = `-60.43°`, lat = `53.32°` | 07 Cell 5, line "GOOSE BAY airbase" | All transit costs measured from this fixed point. |
| `AIRCRAFT_SPEED_KMH` | 850.0 | 07 Cell 1 / `precompute_mission.py:43` | Drives turn-penalty, satellite arc-length, and time-budget conversions. |
| `FLIGHT_TIME_H` / `TOTAL_BUDGET_KM` | 5.0 h → 4250 km | 07 Cell 1 / `precompute_mission.py:44` | Hard feasibility cap; LOCK 1 forbids infeasible plans. |
| `ATC_PENALTY_FACTOR` | 1.35 | 07 Cell 1 / `precompute_mission.py:45` | Already baked into cached SP edge weights via `seg_atc_cost`. |
| `TURN_PENALTY_KM` | 7.5 min × 850 km/h ≈ 106 | 07 Cell 1 / `precompute_mission.py:46` | Base penalty per sharp turn (Reading B). |
| `TURN_THRESHOLD_DEG` | 30.0 | 07 Cell 1 / `plan_route_field.py:45` | Master 2026-06-09: only Δθ > 30° is penalised. |
| `T_SAT_H` | 4.0 | 07 Cell 17 / `precompute_mission.py:47` | Satellite reference time for `T_dep ≥ 0` constraint. |
| `SAT_MIN_LENGTH_KM` | 6 min × 850 km/h ≈ 85 | 07 / `precompute_mission.py:49` | Hard lower bound on satellite-coincidence segment length. |
| `SAT_ARC_MAX_KM` | 30 min × 850 km/h ≈ 425 | 07 / `precompute_mission.py:51` | Ideal satellite-coincidence length; Phase A surfaces near-ideal candidates. |

## Local frame convention

All downstream code works in a local equirectangular km frame centred on the
TPV-contour centroid (`lon_c`, `lat_c`).  The conversion helper

```python
def wgs84_to_km(lon, lat):
    x = (lon - lon_c) * cos(radians(lat_c)) * 111.32
    y = (lat - lat_c) * 110.54
    return x, y
```

is duplicated verbatim from 07 Cell 5.  When the TPV source changes to
`Contour_2_14_18_3_3TPVs.shp`, `lon_c` / `lat_c` are recomputed from the union
of the three TPV polygons (not from any single one) so the frame remains
centred on the multi-TPV mission area.

## Verifier (D5) data-handling note

Per FPO-3 §3 (R3 from FPO-5 implementation plan), the independent verifier
must reload every shapefile listed above *from disk* and recompute geometry
from scratch.  It **must not** consume any precomputed atomic block, cached SP
matrix, or planner-internal intermediate — otherwise the verifier degenerates
into a "did the cache agree with itself?" check that cannot catch solver
bugs.
