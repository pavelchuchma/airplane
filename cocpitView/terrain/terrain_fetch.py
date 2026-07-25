#!/usr/bin/env python3
"""One-off downloader of a terrain elevation grid from CUZK DMR 5G.

DMR 5G is the Czech national digital terrain model (open data, ~0.2 m
vertical accuracy). It is served by a public ArcGIS ImageServer which can
resample and reproject on the fly, so the grid is requested directly in
WGS84 at the wanted step - the runtime lookup (terrain.py) then needs no
GIS library at all, just numpy.

The result is a .npz with elevations in int16 decimeters (Bpv heights,
within ~0.4 m of AMSL - negligible against baro accuracy) plus the grid
origin/steps in degrees. Default area: home airfield +- 10 km.

Usage:
    ./.venv/bin/python terrain/terrain_fetch.py
    ./.venv/bin/python terrain/terrain_fetch.py --lat 50.0 --lon 15.0 --radius-km 10
"""

import argparse
import io
import json
import math
import os
import sys
import urllib.parse
import urllib.request

import numpy as np
import tifffile

SERVICE_URL = "https://ags.cuzk.cz/arcgis2/rest/services/dmr5g/ImageServer"
METERS_PER_DEG_LAT = 111_320.0
BAND_ROWS = 1000  # rows per HTTP request (server caps image height at 4100)
NODATA = -32768  # int16 sentinel for "outside DMR 5G coverage"
DEFAULT_OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", "terrain.npz")


def fetch_band(lon_min, lon_max, lat_min, lat_max, width, height):
    params = urllib.parse.urlencode(
        {
            "bbox": f"{lon_min},{lat_min},{lon_max},{lat_max}",
            "bboxSR": 4326,
            "imageSR": 4326,
            "size": f"{width},{height}",
            "format": "tiff",
            "pixelType": "F32",
            "interpolation": "RSP_BilinearInterpolation",
            "f": "image",
        }
    )
    with urllib.request.urlopen(f"{SERVICE_URL}/exportImage?{params}", timeout=120) as resp:
        data = resp.read()
    band = tifffile.imread(io.BytesIO(data))
    if band.shape != (height, width):
        raise RuntimeError(f"server returned {band.shape}, expected {(height, width)}")
    return band


def identify(lat_deg, lon_deg):
    """Server-side point elevation, or None - the reference for self-checks."""
    params = urllib.parse.urlencode(
        {
            "geometry": json.dumps({"x": lon_deg, "y": lat_deg, "spatialReference": {"wkid": 4326}}),
            "geometryType": "esriGeometryPoint",
            "returnGeometry": "false",
            "f": "json",
        }
    )
    with urllib.request.urlopen(f"{SERVICE_URL}/identify?{params}", timeout=30) as resp:
        value = json.load(resp).get("value")
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--lat", type=float, default=50.000000, help="area center latitude (default: home airfield)")
    parser.add_argument("--lon", type=float, default=15.000000, help="area center longitude")
    parser.add_argument("--radius-km", type=float, default=10.0, help="half-size of the square area (default: 10)")
    parser.add_argument("--step-m", type=float, default=5.0, help="grid step in meters (default: 5)")
    parser.add_argument("--out", default=DEFAULT_OUT, help="output file (default: terrain/data/terrain.npz)")
    args = parser.parse_args()

    # One angular step for BOTH axes: exportImage silently rescales the bbox
    # when its aspect ratio does not match the pixel aspect ratio, so pixels
    # must be square in degrees (step_m applies exactly along latitude; along
    # longitude the ground step comes out ~cos(lat) smaller - finer, harmless).
    dpix = args.step_m / METERS_PER_DEG_LAT
    nrows = int(math.ceil(2 * args.radius_km * 1000 / args.step_m))
    ncols = int(math.ceil(nrows / math.cos(math.radians(args.lat))))

    # grid pixel (row, col) center: lat0 - row*dpix, lon0 + col*dpix; row 0 = north edge
    lat_max = args.lat + nrows / 2 * dpix
    lon_min = args.lon - ncols / 2 * dpix
    lat0 = lat_max - dpix / 2
    lon0 = lon_min + dpix / 2

    print(f"Fetching {nrows}x{ncols} grid ({args.step_m} m step, {2 * args.radius_km:.0f}x{2 * args.radius_km:.0f} km) from CUZK DMR 5G...")
    elev = np.empty((nrows, ncols), dtype=np.float32)
    for row in range(0, nrows, BAND_ROWS):
        rows = min(BAND_ROWS, nrows - row)
        band_lat_max = lat_max - row * dpix
        band_lat_min = band_lat_max - rows * dpix
        elev[row : row + rows] = fetch_band(lon_min, lon_min + ncols * dpix, band_lat_min, band_lat_max, ncols, rows)
        print(f"  rows {row + rows}/{nrows}")

    valid = elev > 0  # dataset minimum is ~10 m; 0/negative means no coverage
    elev_dm = np.where(valid, np.round(elev * 10), NODATA).astype(np.int16)
    print(f"Elevation range: {elev[valid].min():.1f} - {elev[valid].max():.1f} m, {(~valid).mean() * 100:.2f}% nodata")

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    np.savez_compressed(
        args.out,
        elev_dm=elev_dm,
        lat0=lat0,
        lon0=lon0,
        dlat=dpix,
        dlon=dpix,
        nodata=NODATA,
        source="CUZK DMR 5G (ags.cuzk.cz), heights Bpv",
    )
    print(f"Saved {args.out}")

    # self-check: compare a few valid grid pixels against server-side point
    # queries - catches any silent bbox rescaling by the server
    print("Verifying against the identify endpoint...")
    rng = np.random.default_rng(0)
    checked = worst = 0
    while checked < 5:
        r, c = int(rng.integers(1, nrows - 1)), int(rng.integers(1, ncols - 1))
        if elev_dm[r, c] == NODATA:
            continue
        ref = identify(lat0 - r * dpix, lon0 + c * dpix)
        if ref is None:
            continue
        diff = abs(elev_dm[r, c] / 10.0 - ref)
        worst = max(worst, diff)
        checked += 1
    print(f"  worst of {checked} points: {worst:.2f} m")
    if worst > 5.0:
        print("ERROR: grid disagrees with the server - do not use this file", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
