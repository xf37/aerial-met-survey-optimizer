"""Shapefile loaders for the multi-target planner (FPO-5 / Phase 0a).

All inputs come from ``C:\\nasa-ffp-nurture\\test_data\\``, mirroring 07's
``DATA_DIR``. The mapping rationale lives in :file:`DATA_SOURCES.md`.

Two intentional departures from 07's data path:

* **TPV source**: Phase 0a uses ``Contour_2_14_18_3_3TPVs.shp`` (3 closed
  contour LineStrings) instead of 07's single-TPV
  ``Contour_2_14_18_3.shp``. Each LineString becomes a Polygon (the closed
  interior) with :func:`shapely.geometry.Polygon`.
* **Local frame origin**: The equirectangular km frame is centred on the
  *union* centroid of the three TPV polygons, not on a single contour, so
  no TPV gets a privileged origin.

Everything else (BASE = GOOSE BAY, restricted/ATC/dropsonde/satellite paths,
the ``_load_polys_km`` filter idiom) is preserved verbatim from 07 to keep
results comparable.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np
import geopandas as gpd
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union


# Hard-coded constants kept consistent with 07 + plan_route_field.py + precompute_mission.py
DEFAULT_DATA_DIR = r"C:\nasa-ffp-nurture\test_data"

#: GOOSE BAY airbase. Hard-coded in 07 cell 5; keep verbatim per Luc's instruction.
BASE_WGS84_LON = -60.43
BASE_WGS84_LAT = 53.32

#: Filenames mirrored from 07 / precompute_mission.py. Each pairs (role, filename, geometry_kind).
TPV_FILENAME_PHASE0A = "Contour_2_14_18_3_3TPVs.shp"   # multi-TPV, Master 2026-06-09
TPV_FILENAME_07      = "Contour_2_14_18_3.shp"         # single-TPV, kept for regression
RESTRICTED_FILENAME  = "Restricted_Airspace_Footprint.shp"
ATC_FILENAME         = "Ganderairtraff_Polygon.shp"
DROPSONDE_FILENAME   = "dropsonde_available.shp"
SATELLITE_FILENAME   = "satellite overpass_0516_simplified.shp"  # NOTE: 0516 = mission date — confirm with Master if mission date changes
GATEPOINTS_FILENAME  = "gatepoints.shp"

#: Polygon clipping radius (km) for restricted + dropsonde, matching 07 cells 12 and 21.
POLY_FILTER_RADIUS_KM = 2000.0


@dataclass
class LocalFrame:
    """Equirectangular km frame centred on ``(lon_c, lat_c)``."""
    lon_c: float
    lat_c: float

    def to_km(self, lon, lat):
        """Vectorised WGS84 (deg) -> local km."""
        lon = np.asarray(lon, dtype=np.float64)
        lat = np.asarray(lat, dtype=np.float64)
        x = (lon - self.lon_c) * np.cos(np.radians(self.lat_c)) * 111.32
        y = (lat - self.lat_c) * 110.54
        return x, y


@dataclass
class TPV:
    """One TPV polygon in the local km frame plus its index in the source file."""
    label: str
    polygon_km: Polygon
    vertices_km: np.ndarray  # (N, 2)


# ---------------------------------------------------------------------------
# Top-level loaders
# ---------------------------------------------------------------------------

def verify_data_sources(data_dir: str = DEFAULT_DATA_DIR) -> dict:
    """Check that every expected shapefile is present.

    Returns a dict ``{role: (path, exists: bool)}`` so callers can fail loud
    (LOCK: no fallback to synthetic data) without crashing.
    """
    expected = {
        "tpv_phase0a":   TPV_FILENAME_PHASE0A,
        "tpv_07":        TPV_FILENAME_07,
        "restricted":    RESTRICTED_FILENAME,
        "atc":           ATC_FILENAME,
        "dropsonde":     DROPSONDE_FILENAME,
        "satellite":     SATELLITE_FILENAME,
        "gatepoints":    GATEPOINTS_FILENAME,
    }
    report = {}
    for role, fname in expected.items():
        path = os.path.join(data_dir, fname)
        report[role] = (path, os.path.isfile(path))
    return report


def load_tpvs(
    data_dir: str = DEFAULT_DATA_DIR,
    filename: str = TPV_FILENAME_PHASE0A,
    frame: LocalFrame | None = None,
) -> tuple[list[TPV], LocalFrame]:
    """Load TPV contours and convert to Polygons in the local km frame.

    If ``frame`` is None, build one centred on the union centroid of all TPV
    polygons. Returns ``(tpvs, frame)``.
    """
    path = os.path.join(data_dir, filename)
    gdf = gpd.read_file(path).to_crs("EPSG:4326")
    if gdf.empty:
        raise ValueError(f"TPV shapefile {path} is empty")

    # Each feature is a closed contour LineString; convert to Polygon (closed interior).
    wgs_polys = []
    for i, geom in enumerate(gdf.geometry):
        if geom is None or geom.is_empty:
            raise ValueError(f"TPV feature {i} in {path} is empty/null")
        if geom.geom_type == "LineString":
            coords = np.array(geom.coords)
        elif geom.geom_type == "Polygon":
            coords = np.array(geom.exterior.coords)
        else:
            raise ValueError(
                f"TPV feature {i} in {path} has unsupported geom_type "
                f"{geom.geom_type!r}; expected LineString or Polygon"
            )
        wgs_polys.append(Polygon(coords))

    # Build local frame from the centroid of the union of polygons, in WGS84.
    if frame is None:
        union = unary_union(wgs_polys)
        c = union.centroid
        frame = LocalFrame(lon_c=float(c.x), lat_c=float(c.y))

    tpvs: list[TPV] = []
    for i, wgs_poly in enumerate(wgs_polys):
        wgs_coords = np.array(wgs_poly.exterior.coords)
        x, y = frame.to_km(wgs_coords[:, 0], wgs_coords[:, 1])
        verts = np.column_stack([x, y])
        km_poly = Polygon(verts).buffer(0)
        tpvs.append(
            TPV(label=f"TPV-{i+1}", polygon_km=km_poly, vertices_km=verts)
        )
    return tpvs, frame


def load_polygons_km(
    shp_path: str,
    frame: LocalFrame,
    filter_km_radius: float | None = None,
) -> list[Polygon]:
    """Mirror of 07's ``_load_polys_km`` (Cell 5).

    Reads a polygon shapefile (any CRS), reprojects to WGS84, converts to the
    local km frame, optionally clips to a circle around the frame origin.
    """
    gdf = gpd.read_file(shp_path).to_crs("EPSG:4326")
    clip_circle = (
        Point(0, 0).buffer(filter_km_radius) if filter_km_radius else None
    )
    out: list[Polygon] = []
    for geom in gdf.geometry:
        if geom is None or geom.is_empty:
            continue
        parts = list(geom.geoms) if geom.geom_type == "MultiPolygon" else [geom]
        for poly in parts:
            if poly.geom_type != "Polygon":
                continue
            lon_arr = np.array([c[0] for c in poly.exterior.coords])
            lat_arr = np.array([c[1] for c in poly.exterior.coords])
            x_arr, y_arr = frame.to_km(lon_arr, lat_arr)
            km_poly = Polygon(zip(x_arr, y_arr)).buffer(0)
            if not km_poly.is_valid or km_poly.is_empty or km_poly.area < 0.1:
                continue
            if clip_circle is not None:
                if not km_poly.intersects(clip_circle):
                    continue
                km_poly = km_poly.intersection(clip_circle).buffer(0)
                if km_poly.is_empty:
                    continue
            # Intersection can return MultiPolygon / GeometryCollection — flatten.
            if km_poly.geom_type in ("MultiPolygon", "GeometryCollection"):
                for part in km_poly.geoms:
                    if part.geom_type == "Polygon" and part.area >= 0.1:
                        out.append(part)
            elif km_poly.geom_type == "Polygon" and km_poly.area >= 0.1:
                out.append(km_poly)
    return out


def load_restricted(data_dir: str, frame: LocalFrame) -> list[Polygon]:
    return load_polygons_km(
        os.path.join(data_dir, RESTRICTED_FILENAME),
        frame,
        filter_km_radius=POLY_FILTER_RADIUS_KM,
    )


def load_atc(data_dir: str, frame: LocalFrame) -> list[Polygon]:
    # 07 Cell 12 passes no filter for ATC — Gander FIR is a single polygon.
    return load_polygons_km(
        os.path.join(data_dir, ATC_FILENAME),
        frame,
        filter_km_radius=None,
    )


def load_dropsonde(data_dir: str, frame: LocalFrame) -> list[Polygon]:
    return load_polygons_km(
        os.path.join(data_dir, DROPSONDE_FILENAME),
        frame,
        filter_km_radius=POLY_FILTER_RADIUS_KM,
    )


def load_satellite_track(data_dir: str, frame: LocalFrame) -> np.ndarray:
    """Return the satellite ground track as an (N, 2) array in km.

    Mirrors 07's `satellite overpass_0516_simplified.shp` load: a single
    LineString.
    """
    path = os.path.join(data_dir, SATELLITE_FILENAME)
    gdf = gpd.read_file(path).to_crs("EPSG:4326")
    geom = gdf.geometry.iloc[0]
    if geom is None or geom.is_empty:
        raise ValueError(f"Satellite track {path} is empty")
    coords = np.array(geom.coords)
    if coords.shape[0] < 2:
        raise ValueError(f"Satellite track {path} has fewer than 2 vertices")
    x, y = frame.to_km(coords[:, 0], coords[:, 1])
    return np.column_stack([x, y])


def load_gatepoints(data_dir: str, frame: LocalFrame) -> np.ndarray:
    """Return mandatory gatepoints as an (M, 2) array in km."""
    path = os.path.join(data_dir, GATEPOINTS_FILENAME)
    gdf = gpd.read_file(path).to_crs("EPSG:4326")
    pts = []
    for geom in gdf.geometry:
        if geom is None or geom.is_empty:
            continue
        if geom.geom_type != "Point":
            raise ValueError(
                f"Gatepoint feature in {path} has unsupported geom_type "
                f"{geom.geom_type!r}; expected Point"
            )
        x, y = frame.to_km(np.array([geom.x]), np.array([geom.y]))
        pts.append((float(x[0]), float(y[0])))
    if not pts:
        raise ValueError(f"No Point features found in {path}")
    return np.asarray(pts, dtype=np.float64)


def base_km(frame: LocalFrame) -> np.ndarray:
    """BASE (GOOSE BAY) in the local km frame."""
    x, y = frame.to_km(np.array([BASE_WGS84_LON]), np.array([BASE_WGS84_LAT]))
    return np.array([float(x[0]), float(y[0])])
