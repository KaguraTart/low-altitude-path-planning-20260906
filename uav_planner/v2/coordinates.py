"""WGS84/AEQD conversion with explicit vertical datum evidence.

The local horizontal axes are east/north metres in an azimuthal equidistant
projection. The local vertical axis is ``MSL - origin_alt_msl``. AEQD is used
only inside the declared radius, capped at 50 km. No datum conversion is guessed.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Iterable
from datetime import datetime
from typing import Any

from pyproj import CRS, Transformer
from shapely import from_wkt, to_wkt
from shapely.geometry import Point
from shapely.geometry.base import BaseGeometry
from shapely.ops import transform

from uav_planner.errors import PlanningError

from .models import CoordinateRequest, CoordinateResponse, CoordinateSpec, validated_wkt


class CoordinateError(PlanningError):
    """An input coordinate cannot be converted without losing its meaning."""


class CoordinateFrame:
    """Reusable always-XY transforms for one geographic planning scene."""

    def __init__(
        self,
        spec: CoordinateSpec | dict[str, Any],
        terrain_provider: Callable[[float, float, datetime | None], float | None] | None = None,
    ) -> None:
        self.spec = spec if isinstance(spec, CoordinateSpec) else CoordinateSpec.model_validate(spec)
        self.terrain_provider = terrain_provider
        self.local_crs = CRS.from_proj4(
            f"+proj=aeqd +lat_0={self.spec.origin_lat} +lon_0={self.spec.origin_lon} "
            "+datum=WGS84 +units=m +no_defs"
        )
        self._forward = Transformer.from_crs("EPSG:4326", self.local_crs, always_xy=True)
        self._inverse = Transformer.from_crs(self.local_crs, "EPSG:4326", always_xy=True)

    @property
    def context(self) -> dict[str, Any]:
        return {
            "input_horizontal_crs": "EPSG:4326",
            "axis_order": "longitude,latitude,altitude",
            "local_horizontal_crs": self.local_crs.to_string(),
            "local_axis_order": "east_m,north_m,msl_minus_origin_m",
            "origin_lon": self.spec.origin_lon,
            "origin_lat": self.spec.origin_lat,
            "origin_alt_msl": self.spec.origin_alt_msl,
            "max_radius_m": self.spec.max_radius_m,
            "terrain_source": self.spec.terrain_source,
            "geoid_source": self.spec.geoid_source,
            "height_correction_versions": [item.version for item in self.spec.height_corrections],
        }

    def altitude_to_msl(
        self, height: float, reference: str, lon: float, lat: float, at: datetime | None = None,
    ) -> float:
        """Return absolute MSL metres using declared constant datum evidence."""
        _finite(height, "height")
        self._check_geographic(lon, lat)
        if reference == "MSL":
            return float(height)
        if reference == "AGL":
            terrain = self.terrain_provider(lon, lat, at) if self.terrain_provider is not None else None
            if terrain is None:
                terrain = self.spec.terrain_msl_m
            if terrain is None:
                raise CoordinateError("HEIGHT_REFERENCE_UNAVAILABLE", "AGL requires declared terrain_msl_m")
            _finite(terrain, "terrain MSL height")
            return float(height) + terrain
        if reference == "ELLIPSOID":
            if self.spec.geoid_separation_m is None:
                raise CoordinateError("HEIGHT_REFERENCE_UNAVAILABLE", "ELLIPSOID requires declared geoid_separation_m")
            return float(height) - self.spec.geoid_separation_m
        if reference in {"QNE", "QNH"}:
            return float(height) + self._pressure_offset(reference, lon, lat, at)
        raise CoordinateError("INVALID_ALTITUDE_REFERENCE", f"Unsupported altitude reference: {reference}")

    def altitude_to_local(
        self, height: float, reference: str, lon: float, lat: float, at: datetime | None = None,
    ) -> float:
        return self.altitude_to_msl(height, reference, lon, lat, at) - self.spec.origin_alt_msl

    def altitude_from_msl(
        self, height: float, reference: str, lon: float, lat: float, at: datetime | None = None,
    ) -> float:
        """Invert the same datum evidence; pressure offset is MSL minus reported altitude."""
        _finite(height, "height")
        self._check_geographic(lon, lat)
        # The conversion is an additive offset at the declared point and time.
        return float(height) - self.altitude_to_msl(0, reference, lon, lat, at)

    def _pressure_offset(self, reference: str, lon: float, lat: float, at: datetime | None) -> float:
        if at is None or at.tzinfo is None or at.utcoffset() is None:
            raise CoordinateError("HEIGHT_REFERENCE_UNAVAILABLE", f"{reference} requires an aware conversion time")
        candidates = [
            item for item in self.spec.height_corrections
            if item.reference == reference and item.valid_from <= at < item.valid_to
            and (item.geometry_wkt is None or from_wkt(item.geometry_wkt).covers(Point(lon, lat)))
        ]
        if not candidates:
            raise CoordinateError("HEIGHT_REFERENCE_UNAVAILABLE", f"No valid {reference} height correction at the requested location/time")
        if len(candidates) != 1:
            raise CoordinateError("AMBIGUOUS_HEIGHT_CORRECTION", f"Multiple {reference} height corrections apply")
        return candidates[0].offset_m

    def to_local(self, wkt: str, alt_reference: str, at: datetime | None = None) -> BaseGeometry:
        """Convert WGS84 WKT; preserve 2D footprints and every 3D vertex."""
        geometry = self._geometry(wkt)

        def forward(x: Any, y: Any, z: Any = None) -> Any:
            if isinstance(x, Iterable):
                rows = [forward(a, b, None if z is None else c) for a, b, c in zip(x, y, [None] * len(x) if z is None else z)]
                return tuple(list(values) for values in zip(*rows))
            self._check_geographic(x, y)
            east, north = self._forward.transform(x, y, errcheck=True)
            self._check_radius(east, north)
            if z is None:
                return east, north
            return east, north, self.altitude_to_local(z, alt_reference, x, y, at)

        return transform(forward, geometry)

    def to_wgs84(
        self, geometry: BaseGeometry | str, alt_reference: str = "MSL", at: datetime | None = None,
    ) -> str:
        """Convert local geometry to geographic WKT using the requested datum."""
        geometry = self._geometry(geometry) if isinstance(geometry, str) else geometry
        if geometry.is_empty:
            raise CoordinateError("INVALID_COORDINATE", "Geometry must not be empty")

        def inverse(x: Any, y: Any, z: Any = None) -> Any:
            if isinstance(x, Iterable):
                rows = [inverse(a, b, None if z is None else c) for a, b, c in zip(x, y, [None] * len(x) if z is None else z)]
                return tuple(list(values) for values in zip(*rows))
            self._check_radius(x, y)
            lon, lat = self._inverse.transform(x, y, errcheck=True)
            self._check_geographic(lon, lat)
            if z is None:
                return lon, lat
            return lon, lat, self.altitude_from_msl(z + self.spec.origin_alt_msl, alt_reference, lon, lat, at)

        return to_wkt(transform(inverse, geometry), rounding_precision=12, trim=True)

    def convert_batch(self, request: CoordinateRequest) -> CoordinateResponse:
        if request.coordinates != self.spec:
            raise CoordinateError("COORDINATE_CONTEXT_MISMATCH", "Request coordinates differ from this frame")
        if request.direction == "wgs84_to_local":
            wkts = [to_wkt(self.to_local(wkt, request.alt_reference, request.at), rounding_precision=12, trim=True) for wkt in request.geometries_wkt]
        else:
            wkts = [self.to_wgs84(wkt, request.alt_reference, request.at) for wkt in request.geometries_wkt]
        return CoordinateResponse(
            geometries_wkt=wkts, direction=request.direction,
            alt_reference=request.alt_reference, coordinate_context=self.context,
        )

    @staticmethod
    def _geometry(wkt: str) -> BaseGeometry:
        try:
            validated_wkt(wkt)
            geometry = from_wkt(wkt)
            if getattr(geometry, "has_m", False):
                raise ValueError("M coordinates are unsupported; use explicit Z altitude")
            return geometry
        except Exception as exc:
            raise CoordinateError("INVALID_COORDINATE", str(exc)) from exc

    @staticmethod
    def _check_geographic(lon: float, lat: float) -> None:
        _finite(lon, "longitude")
        _finite(lat, "latitude")
        if not -180 <= lon <= 180 or not -90 <= lat <= 90:
            raise CoordinateError("INVALID_COORDINATE", "WGS84 coordinate outside longitude/latitude bounds")

    def _check_radius(self, east: float, north: float) -> None:
        _finite(east, "east")
        _finite(north, "north")
        if math.hypot(east, north) > self.spec.max_radius_m + 1e-6:
            raise CoordinateError("COORDINATE_RANGE_EXCEEDED", "Coordinate lies outside the declared local AEQD radius", {"max_radius_m": self.spec.max_radius_m})


def _finite(value: float, label: str) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise CoordinateError("INVALID_COORDINATE", f"{label} must be finite")


def convert_coordinates(request: CoordinateRequest | dict[str, Any]) -> CoordinateResponse:
    parsed = request if isinstance(request, CoordinateRequest) else CoordinateRequest.model_validate(request)
    return CoordinateFrame(parsed.coordinates).convert_batch(parsed)
