"""Terrain elevation lookup from a pre-fetched grid (see cockpitview/terrain_fetch.py).

The grid is a regular WGS84 raster of int16 decimeters, so a query is two
array index computations plus bilinear interpolation - cheap enough to run
every frame. Positions outside the grid (or over nodata pixels) return None
so the HUD can drop the readout instead of showing a wrong number.
"""

import math
import os

import numpy as np


class Terrain:
    def __init__(self, path):
        data = np.load(path)
        self.elev_dm = data["elev_dm"]
        self.lat0 = float(data["lat0"])
        self.lon0 = float(data["lon0"])
        self.dlat = float(data["dlat"])
        self.dlon = float(data["dlon"])
        self.nodata = int(data["nodata"])

    @classmethod
    def load(cls, path):
        """Terrain instance, or None when the file does not exist."""
        return cls(path) if os.path.exists(path) else None

    def elevation_m(self, lat_deg, lon_deg):
        """Terrain elevation AMSL in meters, or None outside the grid."""
        row = (self.lat0 - lat_deg) / self.dlat
        col = (lon_deg - self.lon0) / self.dlon
        r0, c0 = math.floor(row), math.floor(col)
        nr, nc = self.elev_dm.shape
        if not (0 <= r0 < nr - 1 and 0 <= c0 < nc - 1):
            return None
        quad = self.elev_dm[r0 : r0 + 2, c0 : c0 + 2]
        if (quad == self.nodata).any():
            return None
        fr, fc = row - r0, col - c0
        top = quad[0, 0] * (1 - fc) + quad[0, 1] * fc
        bottom = quad[1, 0] * (1 - fc) + quad[1, 1] * fc
        return (top * (1 - fr) + bottom * fr) / 10.0

    def agl_m(self, lat_deg, lon_deg, amsl_m):
        """Height above terrain in meters, or None when terrain is unknown."""
        ground = self.elevation_m(lat_deg, lon_deg)
        if ground is None:
            return None
        return amsl_m - ground
