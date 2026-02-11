# grid_atc.py
from __future__ import annotations
from typing import List, Tuple
import math

def meters_to_latlon(d_north_m: float, d_east_m: float, ref_lat_deg: float, ref_lon_deg: float) -> Tuple[float, float]:
    dlat = d_north_m / 111_320.0
    dlon = d_east_m / (111_320.0 * math.cos(math.radians(ref_lat_deg)))
    return (ref_lat_deg + dlat, ref_lon_deg + dlon)

def grid_offsets(nx: int, ny: int, spacing_m: float) -> List[Tuple[float, float]]:
    """
    Return a serpentine list of (north_m, east_m) waypoints that trace a grid.
    Grid is centered at (0,0). nx columns (east-west), ny rows (north-south).
    """
    assert nx >= 2 and ny >= 2
    east0  = -0.5 * (nx - 1) * spacing_m
    north0 = -0.5 * (ny - 1) * spacing_m

    pts: List[Tuple[float, float]] = []
    for r in range(ny):
        north = north0 + r * spacing_m
        cols = range(nx) if (r % 2 == 0) else range(nx - 1, -1, -1)
        for c in cols:
            east = east0 + c * spacing_m
            pts.append((north, east))
    return pts

async def upload_geofence_rect(sysobj, center_lat: float, center_lon: float, width_m: float, height_m: float) -> None:
    """
    Upload one rectangular INCLUSION geofence around (center_lat, center_lon).
    """
    from mavsdk.geofence import Point, Polygon, GeofenceData, FenceType

    half_w = 0.5 * width_m
    half_h = 0.5 * height_m

    corners_enu = [
        ( half_h,  half_w),
        ( half_h, -half_w),
        (-half_h, -half_w),
        (-half_h,  half_w),
        ( half_h,  half_w),
    ]

    pts_ll = [meters_to_latlon(n, e, center_lat, center_lon) for (n, e) in corners_enu]
    poly_pts = [Point(lat, lon) for (lat, lon) in pts_ll]

    # MAVSDK-Python uses uppercase enum names (INCLUSION/EXCLUSION)
    poly = Polygon(points=poly_pts, fence_type=FenceType.INCLUSION)

    # GeofenceData now takes both polygons and circles
    data = GeofenceData(polygons=[poly], circles=[])
    await sysobj.geofence.upload_geofence(data)


def actions_for_grid(grid_pts: List[Tuple[float, float]], cruise_alt_m: float, start_t: float = 6.0, step_t: float = 4.0) -> List[dict]:
    """
    Turn grid (north,east) points into your runner's goto_offset actions.
    """
    acts = []
    t = start_t
    for (n, e) in grid_pts:
        acts.append({"t": round(t, 1), "cmd": "goto_offset", "north_m": float(n), "east_m": float(e), "alt": float(cruise_alt_m)})
        t += step_t
    return acts

if __name__ == "__main__":
    # helper to print actions you can paste into scenarios.json if you prefer pre-bake
    import json
    nx, ny, spacing = 5, 5, 25.0
    alt = 46.0
    pts = grid_offsets(nx, ny, spacing)
    acts = [{"t": 0.0, "cmd": "arm"}, {"t": 1.0, "cmd": "takeoff", "alt": alt}] + actions_for_grid(pts, alt) + [{"t": 80.0, "cmd": "land"}]
    print(json.dumps(acts, indent=2))
