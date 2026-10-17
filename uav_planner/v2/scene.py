"""Compile geographic resources once and check complete swept route segments.

All internal coordinates use the scene's local east/north/metre frame.  Volume
and field intersections are clipped in horizontal geometry, altitude, and time;
neither a thin obstacle nor a short-lived restriction can hide between samples.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from collections import OrderedDict
from datetime import datetime, timedelta
from math import acos, atan2, ceil, degrees, isfinite, pi, sin
from typing import Any

import numpy as np
from shapely import force_2d, from_wkt, intersects_xy
from shapely.geometry import LineString, Point, box
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union
from shapely.prepared import prep
from shapely.strtree import STRtree

from .coordinates import CoordinateFrame
from .models import Scene
from .rules import EPS, RuleSet, as_datetime, get, invalid, violation


def _geometry_intervals(geometry: BaseGeometry, p0: np.ndarray, p1: np.ndarray) -> list[tuple[float, float]]:
    """Closed parameter intervals of a 2D segment inside an exact footprint."""
    if geometry.is_empty:
        return []
    delta = p1[:2] - p0[:2]
    den = float(np.dot(delta, delta))
    if den <= EPS * EPS:
        return [(0.0, 1.0)] if geometry.covers(Point(p0[:2])) else []
    line = LineString([p0[:2], p1[:2]])
    if prep(geometry).covers(line):
        return [(0.0, 1.0)]
    intersection = geometry.intersection(line)
    result: list[tuple[float, float]] = []

    def collect(item: BaseGeometry) -> None:
        if item.is_empty:
            return
        if hasattr(item, "geoms"):
            for part in item.geoms:
                collect(part)
        else:
            coords = np.asarray(item.coords, float)[:, :2]
            params = ((coords - p0[:2]) @ delta) / den
            result.append((max(0.0, float(params.min())), min(1.0, float(params.max()))))

    collect(intersection)
    return _merge(result)


def _merge(intervals: list[tuple[float, float]]) -> list[tuple[float, float]]:
    result: list[tuple[float, float]] = []
    for a, b in sorted(intervals):
        if b < a - EPS:
            continue
        if result and a <= result[-1][1] + EPS:
            result[-1] = (result[-1][0], max(result[-1][1], b))
        else:
            result.append((a, b))
    return result


def _covers(intervals: list[tuple[float, float]], a: float = 0, b: float = 1) -> bool:
    merged = _merge(intervals)
    return any(low <= a + EPS and high >= b - EPS for low, high in merged)


def _clip_scalar(intervals: list[tuple[float, float]], x0: float, x1: float, lo: float, hi: float) -> list[tuple[float, float]]:
    if abs(x1 - x0) <= EPS:
        return intervals if lo - EPS <= x0 <= hi + EPS else []
    a, b = sorted(((lo - x0) / (x1 - x0), (hi - x0) / (x1 - x0)))
    return [(max(low, a), min(high, b)) for low, high in intervals if max(low, a) <= min(high, b) + EPS]


@dataclass
class PreparedVolume:
    id: str
    geometry: BaseGeometry
    min_altitude: float
    max_altitude: float
    alt_reference: str = "MSL"
    buffer_m: float = 0
    valid_from_s: float = -float("inf")
    valid_to_s: float = float("inf")
    source: Any = None
    kind: str = "obstacle"
    prepared: Any = field(init=False)

    def __post_init__(self) -> None:
        self.prepared = prep(self.geometry)

    @property
    def z_min(self) -> float:
        return self.min_altitude

    @property
    def z_max(self) -> float:
        return self.max_altitude

    @property
    def area_m2(self) -> float:
        return float(self.geometry.area)

    @property
    def volume_m3(self) -> float:
        return float(self.geometry.area * max(0, self.max_altitude - self.min_altitude))


@dataclass
class PreparedAirway(PreparedVolume):
    centerline: Any = None
    coordinates: Any = None
    half_width: float = 0
    status: str = "ACTIVE"
    direction: str = "BOTH"


@dataclass
class PreparedCell:
    geometry: BaseGeometry
    min_altitude: float
    max_altitude: float
    values: dict
    valid_from_s: float = -float("inf")
    valid_to_s: float = float("inf")


@dataclass
class PreparedDataset:
    id: str
    kind: str
    version: str
    geometry: BaseGeometry
    cells: list[PreparedCell]
    valid_from_s: float
    valid_to_s: float
    source: Any
    tree: Any = None


class CompiledScene:
    def __init__(self, scene: Scene | dict):
        self.model = scene if isinstance(scene, Scene) else Scene.model_validate(scene)
        self.scene = self.model
        self.frame = CoordinateFrame(self.model.coordinates)
        self.time_origin = self.model.time_origin
        self.bounds = self.model.bounds
        self.resolution = self.model.resolution
        self.horizon_s = self.model.horizon_s
        self.origin_alt_msl = self.model.coordinates.origin_alt_msl
        self.terrain_constant = self.model.coordinates.terrain_msl_m
        self.warnings: list[str] = []
        self.resources: dict[str, Any] = {}
        self.airways: list[PreparedAirway] = []
        self.airway_lines: dict[str, np.ndarray] = {}
        self.obstacles: list[PreparedVolume] = []
        self.zones: list[PreparedVolume] = []
        self.connectors: list[PreparedVolume] = []
        self.datasets: list[PreparedDataset] = []
        self._buffer_cache: OrderedDict[tuple, BaseGeometry] = OrderedDict()
        self._blocked_cache: OrderedDict[tuple, np.ndarray] = OrderedDict()
        self._rule_geometry_cache: OrderedDict[tuple, BaseGeometry] = OrderedDict()
        self._area_cache: dict[str, list[BaseGeometry]] = {}
        self._resource_selection_cache: dict[tuple, list] = {}
        self._compile_datasets()
        self.frame.terrain_provider = self._terrain_at_geographic
        for item in self.model.airways:
            line = self.frame.to_local(item.center_line_wkt, item.alt_reference, self.time_origin)
            xy = force_2d(line)
            airway = PreparedAirway(
                **self._volume_args(item, xy.buffer(item.width_m / 2), "airway"),
                centerline=xy, coordinates=np.asarray(line.coords, float), half_width=item.width_m / 2,
                status=item.status, direction=item.direction,
            )
            self._register(airway)
            self.airways.append(airway)
            self.airway_lines[item.id] = airway.coordinates
        for name, target, kind in (("obstacles", self.obstacles, "obstacle"), ("control_zones", self.zones, "zone"), ("connectors", self.connectors, "connector")):
            for item in getattr(self.model, name):
                geom = self._footprint(item.geometry_wkt)
                if geom.geom_type not in {"Polygon", "MultiPolygon"}:
                    invalid("Volume footprints must be polygons", resource_id=item.id)
                volume = PreparedVolume(**self._volume_args(item, geom, kind))
                self._register(volume)
                target.append(volume)
                if kind == "zone":
                    if get(item, "status", "ACTIVE") == "INACTIVE":
                        volume.valid_to_s = -float("inf")
                    # Geom is authoritative. Metadata never changes the occupied footprint.
                    area = float(geom.area)
                    self.warnings.extend([f"{item.id}: {label} recomputed from canonical geom" for label, given, actual in (("area", get(item, "area"), area), ("volume", get(item, "volume"), area * (item.max_altitude - item.min_altitude))) if given is not None and abs(given - actual) > max(1e-3, abs(actual) * 1e-6)])
                    if "MUSHROOM" in str(get(item, "zoneType", "")).upper():
                        self.warnings.append(f"{item.id}: mushroom volume uses conservative canonical footprint at every height")
        self._compile_grid()
        self._airway_tree = STRtree([a.geometry for a in self.airways])
        self._obstacle_tree = STRtree([a.geometry for a in self.obstacles + self.grid_volumes])
        self._zone_tree = STRtree([a.geometry for a in self.zones])
        self._max_obstacle_buffer = max((v.buffer_m for v in self.obstacles + self.grid_volumes), default=0)
        self._max_zone_buffer = max((v.buffer_m for v in self.zones), default=0)
        self.corridor = unary_union([a.geometry for a in self.airways if a.status == "ACTIVE"])
        self.rules = RuleSet(self, self.model.rules)

    @staticmethod
    def _query(tree: Any, items: list, p0: np.ndarray, p1: np.ndarray, margin: float = 0) -> list:
        if not items:
            return []
        low, high = np.minimum(p0[:2], p1[:2]) - max(margin, EPS), np.maximum(p0[:2], p1[:2]) + max(margin, EPS)
        return [items[int(i)] for i in tree.query(box(low[0], low[1], high[0], high[1]))]

    def _register(self, item: Any) -> None:
        if str(item.id) in self.resources:
            invalid("Resource ids must be unique across resource types", resource_id=item.id)
        self.resources[str(item.id)] = item

    def _seconds(self, value: Any, default: float) -> float:
        dt = as_datetime(value)
        return (dt - self.time_origin).total_seconds() if dt else default

    def _footprint(self, wkt: str) -> BaseGeometry:
        # Footprint Z is extraneous: the explicit vertical datum/limits are authoritative.
        geographic = force_2d(from_wkt(wkt))
        return self.frame.to_local(geographic.wkt, "MSL", self.time_origin)

    def _volume_args(self, item: Any, geometry: BaseGeometry, kind: str) -> dict:
        reference = get(item, "alt_reference", "MSL")
        lo, hi = get(item, "min_altitude"), get(item, "max_altitude")
        # AGL varies by terrain cell and is clipped later against its exact region.
        if reference not in {"AGL", "QNE", "QNH"}:
            geographic = from_wkt(self.frame.to_wgs84(geometry.centroid))
            lo = self.frame.altitude_to_local(lo, reference, geographic.x, geographic.y, self.time_origin)
            hi = self.frame.altitude_to_local(hi, reference, geographic.x, geographic.y, self.time_origin)
        elif reference == "AGL" and self.terrain_constant is None and not self.datasets_for(None, "terrain"):
            invalid("AGL resource requires terrain data", resource_id=get(item, "id"))
        elif reference in {"QNE", "QNH"} and not any(c.reference == reference for c in self.model.coordinates.height_corrections):
            invalid("Pressure altitude resource requires declared correction data", resource_id=get(item, "id"))
        return {"id": str(get(item, "id")), "geometry": geometry, "min_altitude": lo,
                "max_altitude": hi, "alt_reference": reference, "buffer_m": get(item, "buffer_m", 0),
                "valid_from_s": self._seconds(get(item, "valid_from"), -float("inf")),
                "valid_to_s": self._seconds(get(item, "valid_to"), float("inf")), "source": item, "kind": kind}

    def _compile_datasets(self) -> None:
        for dataset in self.model.datasets:
            cells = []
            for item in dataset.cells:
                if item.bounds is not None:
                    bounds = item.bounds
                    geom = box(bounds.x_min, bounds.y_min, bounds.x_max, bounds.y_max)
                    lo, hi = bounds.z_min, bounds.z_max
                else:
                    geom = self._footprint(item.geometry_wkt)
                    lo = -float("inf") if item.min_altitude is None else item.min_altitude - self.origin_alt_msl
                    hi = float("inf") if item.max_altitude is None else item.max_altitude - self.origin_alt_msl
                if not isinstance(item.values, dict):
                    invalid("Field values must be an object", dataset_id=dataset.id)
                allowed_keys = {
                    "terrain": {"elevation_msl_m", "terrain_msl_m", "nodata"},
                    "weather": {"wind_speed_mps", "windSpeed", "precipitation", "precipitation_mm_h", "icing", "nodata"},
                    "signal": {"bands", "signal_strength_dbm", "nodata"},
                    "noise": {"noise_db", "nodata"}, "risk": {"risk", "nodata"},
                    "area": {"adminCode", "gridCode", "code", "nodata"},
                    "grid": {"adminCode", "gridCode", "code", "occupied", "nodata"},
                    "corridor": {"nodata"},
                }[dataset.kind]
                unknown = set(item.values) - allowed_keys
                if unknown:
                    invalid("Unknown field dataset values", dataset_id=dataset.id, fields=sorted(unknown))
                if "nodata" in item.values and not isinstance(item.values["nodata"], bool):
                    invalid("Dataset nodata flag must be boolean", dataset_id=dataset.id)
                if dataset.kind == "terrain" and not item.values.get("nodata", False) and not _finite(item.values.get("elevation_msl_m", item.values.get("terrain_msl_m"))):
                    invalid("Terrain cell requires a finite elevation_msl_m", dataset_id=dataset.id)
                cells.append(PreparedCell(geom, lo, hi, item.values,
                                          self._seconds(get(item, "valid_from"), -float("inf")),
                                          self._seconds(get(item, "valid_to"), float("inf"))))
            geom = self._footprint(dataset.geometry_wkt) if dataset.geometry_wkt else unary_union([c.geometry for c in cells])
            prepared = PreparedDataset(dataset.id, dataset.kind, dataset.version, geom, cells,
                                       self._seconds(dataset.valid_from, -float("inf")),
                                       self._seconds(dataset.valid_to, float("inf")), dataset, STRtree([c.geometry for c in cells]))
            self._register(prepared)
            self.datasets.append(prepared)

    def _compile_grid(self) -> None:
        self.grid_volumes: list[PreparedVolume] = []
        grid = self.model.grid
        if grid is None:
            return
        if grid.crs != "LOCAL_AEQD":
            invalid("Grid CRS must explicitly be LOCAL_AEQD", crs=grid.crs)
        origin, resolution = np.asarray(grid.origin), np.asarray(grid.resolution)

        def add(lo: Any, hi: Any, label: str) -> None:
            self.grid_volumes.append(PreparedVolume(label, box(lo[0], lo[1], hi[0], hi[1]), float(lo[2]), float(hi[2]), kind="grid"))

        for i, index in enumerate(grid.occupied):
            lo = origin + np.asarray(index) * resolution
            add(lo, lo + resolution, f"grid-index-{i}")
        for i, item in enumerate(grid.occupied_ranges):
            add(origin + np.asarray(item.start) * resolution, origin + (np.asarray(item.end) + 1) * resolution, f"grid-range-{i}")
        for i, item in enumerate(grid.cells):
            if item.occupied:
                b = item.bounds
                add((b.x_min, b.y_min, b.z_min), (b.x_max, b.y_max, b.z_max), f"grid-cell-{i}")

    def datasets_for(self, rule: Any = None, kind: str | None = None) -> list[PreparedDataset]:
        refs = set(map(str, get(rule, "resource_ids", [])))
        return [d for d in self.datasets if (kind is None or d.kind == kind) and (not refs or d.id in refs)]

    def volumes_for(self, rule: Any, kind: str) -> list[PreparedVolume]:
        key = (str(get(rule, "id")), kind)
        if key in self._resource_selection_cache:
            return self._resource_selection_cache[key]
        refs = set(map(str, get(rule, "resource_ids", [])))
        candidates = self.obstacles if kind == "BUILDING" else self.zones
        if refs:
            candidates = [r for r in candidates if r.id in refs]
        levels = get(rule, "content", {}).get("zoneLevels") if kind == "ZONE" else None
        result = [r for r in candidates if not levels or str(get(r.source, "level")) in set(map(str, levels))]
        self._resource_selection_cache[key] = result
        return result

    def _rule_geometry(self, key: tuple, factory: Any) -> BaseGeometry:
        if key not in self._rule_geometry_cache:
            self._rule_geometry_cache[key] = factory()
        else:
            self._rule_geometry_cache.move_to_end(key)
        return self._rule_geometry_cache[key]

    def _buffered(self, volume: PreparedVolume, distance: float) -> BaseGeometry:
        key = (volume.id, id(volume), round(distance, 9))
        if key not in self._buffer_cache:
            self._buffer_cache[key] = volume.geometry.buffer(distance) if distance else volume.geometry
        else:
            self._buffer_cache.move_to_end(key)
        return self._buffer_cache[key]

    @staticmethod
    def clearance(profile: Any) -> tuple[float, float]:
        return (get(profile, "radius_m", 0) + get(profile, "safety_margin_m", 0), get(profile, "height_m", 0) / 2 + get(profile, "safety_margin_m", 0))

    def volume_intervals(self, volume: PreparedVolume, p0: np.ndarray, p1: np.ndarray, t0: float, t1: float, *, horizontal: float = 0, vertical: float = 0, inside: bool = False) -> list[tuple[float, float]]:
        # For containment shrink a resource; for collision expand it.
        sign = -1 if inside else 1
        geom = self._buffered(volume, sign * horizontal)
        intervals = _geometry_intervals(geom, p0, p1)
        intervals = _clip_scalar(intervals, t0, t1, volume.valid_from_s, volume.valid_to_s)
        lo, hi = volume.min_altitude - sign * vertical, volume.max_altitude + sign * vertical
        if lo > hi:
            return []
        if volume.alt_reference in {"QNE", "QNH"}:
            output = []
            datum = self.datum_parts(volume.alt_reference, p0, p1, t0, t1)
            covered = [(a, b) for a, b, _ in datum]
            if any(not _covers(covered, a, b) for a, b in intervals):
                invalid("Pressure altitude resource has missing correction coverage at the requested location/time", resource_id=volume.id, reference=volume.alt_reference)
            for a, b, offset in datum:
                overlap = [(max(a, c), min(b, d)) for c, d in intervals if max(a, c) <= min(b, d) + EPS]
                output.extend(_clip_scalar(overlap, p0[2], p1[2], lo + offset - self.origin_alt_msl, hi + offset - self.origin_alt_msl))
            return _merge(output)
        if volume.alt_reference != "AGL":
            return _clip_scalar(intervals, p0[2], p1[2], lo, hi)
        # Clip AGL independently in each terrain region. Unknown terrain cannot establish containment.
        terrain_parts = self.terrain_parts(p0, p1, t0, t1)
        output = []
        for a, b, elevation in terrain_parts:
            overlap = [(max(a, c), min(b, d)) for c, d in intervals if max(a, c) <= min(b, d) + EPS]
            output.extend(_clip_scalar(overlap, p0[2], p1[2], lo + elevation - self.origin_alt_msl, hi + elevation - self.origin_alt_msl))
        return _merge(output)

    def terrain_parts(self, p0: np.ndarray, p1: np.ndarray, t0: float, t1: float) -> list[tuple[float, float, float]]:
        datasets = self.datasets_for(None, "terrain")
        if not datasets:
            return [(0, 1, self.terrain_constant)] if self.terrain_constant is not None else []
        candidates = []
        gaps = []
        for ds in datasets:
            if t1 < ds.valid_from_s or t0 > ds.valid_to_s:
                continue
            for cell in self._query(ds.tree, ds.cells, p0, p1):
                elevation = cell.values.get("elevation_msl_m", cell.values.get("terrain_msl_m"))
                parts = _geometry_intervals(cell.geometry, p0, p1)
                parts = _clip_scalar(parts, t0, t1, max(ds.valid_from_s, cell.valid_from_s), min(ds.valid_to_s, cell.valid_to_s))
                if cell.values.get("nodata", False) or not _finite(elevation):
                    gaps.extend(parts)
                else:
                    candidates.extend((a, b, float(elevation)) for a, b in parts)
        # Resolve overlapping cells to the highest conservative terrain surface;
        # explicit nodata masks remain blocked even over another covering cell.
        if gaps:
            return []
        breaks = sorted({0.0, 1.0, *(a for a, _, _ in candidates), *(b for _, b, _ in candidates), *(a for a, _ in gaps), *(b for _, b in gaps)})
        result = []
        for a, b in zip(breaks, breaks[1:]):
            mid = (a + b) / 2
            if any(c - EPS <= mid <= d + EPS for c, d in gaps):
                continue
            values = [height for c, d, height in candidates if c - EPS <= mid <= d + EPS]
            if values:
                result.append((a, b, max(values)))
        return result

    def datum_parts(self, reference: str, p0: np.ndarray, p1: np.ndarray, t0: float, t1: float) -> list[tuple[float, float, float]]:
        result = []
        for correction in self.model.coordinates.height_corrections:
            if correction.reference != reference:
                continue
            geom = self._footprint(correction.geometry_wkt) if correction.geometry_wkt else box(self.bounds.x_min, self.bounds.y_min, self.bounds.x_max, self.bounds.y_max)
            parts = _geometry_intervals(geom, p0, p1)
            parts = _clip_scalar(parts, t0, t1, self._seconds(correction.valid_from, 0), self._seconds(correction.valid_to, 0))
            result.extend((a, b, correction.offset_m) for a, b in parts)
        for i, (a, b, _) in enumerate(result):
            if any(min(b, d) - max(a, c) > EPS for c, d, _ in result[i + 1:]):
                invalid("Pressure altitude corrections overlap ambiguously", reference=reference)
        return result

    def _terrain_at_geographic(self, lon: float, lat: float, at: datetime | None) -> float | None:
        point = self.frame.to_local(Point(lon, lat).wkt, "MSL", at)
        t = self._seconds(at, 0)
        xyz = np.array([point.x, point.y, 0.0])
        elevations = [v for _, _, v in self.terrain_parts(xyz, xyz, t, t)]
        return max(elevations) if elevations else None

    def rule_parts(self, rule: Any, p0: np.ndarray, p1: np.ndarray, t0: float, t1: float) -> list[tuple[float, float]]:
        active = self.rules.interval(rule, t0, t1)
        if active is None:
            return []
        temporal = _clip_scalar([(0, 1)], t0, t1, active[0], active[1])
        refs = list(map(str, get(rule, "resource_ids", [])))
        kind = get(rule, "rule_type")
        # Dataset and avoidance references describe forbidden/required fields, not rule applicability.
        if not refs or kind in {"AREA", "NO_FLY", "BUILDING", "ZONE", "WEATHER", "SIGNAL", "PRIORITY"}:
            return temporal
        spatial = []
        for ref in refs:
            resource = self.resources[ref]
            if isinstance(resource, PreparedVolume):
                spatial.extend(self.volume_intervals(resource, p0, p1, t0, t1))
            elif isinstance(resource, PreparedDataset):
                spatial.extend(_geometry_intervals(resource.geometry, p0, p1))
        return _merge([(max(a, c), min(b, d)) for a, b in temporal for c, d in spatial if max(a, c) <= min(b, d) + EPS])

    def area_for(self, rule: Any) -> list[BaseGeometry]:
        rid = str(get(rule, "id"))
        if rid in self._area_cache:
            return self._area_cache[rid]
        content = get(rule, "content", {})
        refs = set(map(str, get(rule, "resource_ids", []))) | set(map(str, content.get("zoneIds", [])))
        output = [self.resources[ref].geometry for ref in refs if ref in self.resources]
        missing = refs - self.resources.keys()
        if missing:
            invalid("AREA references unavailable geometries", resource_ids=sorted(missing))
        codes = set(content.get("adminCodes", [])) | set(content.get("gridCodes", []))
        found_codes = set()
        for ds in self.datasets_for(None, "area") + self.datasets_for(None, "grid"):
            for cell in ds.cells:
                matching = {str(cell.values.get(key)) for key in ("adminCode", "gridCode", "code")} & codes
                if matching:
                    found_codes.update(matching)
                    output.append(cell.geometry)
        grid = self.model.grid
        if grid is not None:
            for cell in grid.cells:
                if cell.code in codes:
                    b = cell.bounds
                    found_codes.add(cell.code)
                    output.append(box(b.x_min, b.y_min, b.x_max, b.y_max))
            for index_key, code in grid.codes.items():
                if code not in codes:
                    continue
                try:
                    index = np.asarray([int(v) for v in index_key.replace("/", ",").split(",")], dtype=int)
                    if index.shape != (3,) or np.any(index < 0) or np.any(index >= grid.shape):
                        raise ValueError
                except (TypeError, ValueError):
                    invalid("Grid code index must use 'i,j,k' within grid shape", index=index_key)
                low = np.asarray(grid.origin) + index * np.asarray(grid.resolution)
                high = low + np.asarray(grid.resolution)
                found_codes.add(code)
                output.append(box(low[0], low[1], high[0], high[1]))
        if codes - found_codes:
            invalid("AREA references unavailable administrative/grid codes", codes=sorted(codes - found_codes))
        self._area_cache[rid] = output
        return output

    def area_allowed(self, rule: Any, p0: np.ndarray, p1: np.ndarray, parts: list, profile: Any, t0: float = 0, t1: float = 0) -> bool:
        margin, vertical = self.clearance(profile)
        include = get(rule, "content")["scope"] == "INCLUDE"
        content = get(rule, "content")
        refs = set(map(str, get(rule, "resource_ids", []))) | set(map(str, content.get("zoneIds", [])))
        codes = set(content.get("adminCodes", [])) | set(content.get("gridCodes", []))
        resources = [self.resources[r] for r in refs]
        requires_3d = bool(codes) or any(isinstance(r, PreparedVolume) or isinstance(r, PreparedDataset) and r.cells and not get(r.source, "geometry_wkt") for r in resources)
        if requires_3d:
            intervals = []
            sign = -1 if include else 1

            def add_cell(cell: PreparedCell, valid_from: float, valid_to: float) -> None:
                if include and cell.values.get("nodata", False):
                    return
                geom = cell.geometry.buffer(sign * margin) if margin else cell.geometry
                spatial = _geometry_intervals(geom, p0, p1)
                spatial = _clip_scalar(spatial, p0[2], p1[2], cell.min_altitude - sign * vertical, cell.max_altitude + sign * vertical)
                intervals.extend(_clip_scalar(spatial, t0, t1, valid_from, valid_to))

            for resource in resources:
                if isinstance(resource, PreparedVolume):
                    intervals.extend(self.volume_intervals(resource, p0, p1, t0, t1, horizontal=margin, vertical=vertical, inside=include))
                elif isinstance(resource, PreparedDataset):
                    if get(resource.source, "geometry_wkt"):
                        intervals.extend(_clip_scalar(_geometry_intervals(resource.geometry.buffer(sign * margin) if margin else resource.geometry, p0, p1), t0, t1, resource.valid_from_s, resource.valid_to_s))
                    else:
                        for cell in resource.cells:
                            add_cell(cell, max(resource.valid_from_s, cell.valid_from_s), min(resource.valid_to_s, cell.valid_to_s))
            for dataset in self.datasets_for(None, "area") + self.datasets_for(None, "grid"):
                for cell in dataset.cells:
                    if {str(cell.values.get(k)) for k in ("adminCode", "gridCode", "code")} & codes:
                        add_cell(cell, max(dataset.valid_from_s, cell.valid_from_s), min(dataset.valid_to_s, cell.valid_to_s))
            grid = self.model.grid
            if grid is not None:
                for cell in grid.cells:
                    if cell.code in codes:
                        b = cell.bounds
                        add_cell(PreparedCell(box(b.x_min, b.y_min, b.x_max, b.y_max), b.z_min, b.z_max, {}), -float("inf"), float("inf"))
                for index_key, code in grid.codes.items():
                    if code in codes:
                        index = np.asarray([int(v) for v in index_key.replace("/", ",").split(",")])
                        low = np.asarray(grid.origin) + index * np.asarray(grid.resolution)
                        high = low + np.asarray(grid.resolution)
                        add_cell(PreparedCell(box(low[0], low[1], high[0], high[1]), float(low[2]), float(high[2]), {}), -float("inf"), float("inf"))
            if include:
                return all(_covers(intervals, a, b) for a, b in parts)
            return not any(max(a, c) <= min(b, d) + EPS for a, b in parts for c, d in intervals)
        geom = self._rule_geometry(("area", str(get(rule, "id")), margin), lambda: unary_union(self.area_for(rule)).buffer(-margin if include else margin))
        delta = p1 - p0
        for a, b in parts:
            line = Point((p0 + a * delta)[:2]) if np.linalg.norm((b - a) * delta[:2]) <= EPS else LineString([(p0 + a * delta)[:2], (p0 + b * delta)[:2]])
            if (include and not geom.covers(line)) or (not include and geom.intersects(line)):
                return False
        return True

    def height_part_allowed(self, p0: np.ndarray, p1: np.ndarray, t0: float, t1: float, content: dict) -> bool:
        lo = content.get("minAltitude", content.get("minHeight", -float("inf")))
        hi = content.get("maxAltitude", content.get("maxHeight", float("inf")))
        reference = content.get("altDatum", "MSL")
        if reference == "AGL":
            parts = self.terrain_parts(p0, p1, t0, t1)
            if not _covers([(a, b) for a, b, _ in parts]):
                return False
            for a, b, terrain in parts:
                heights = np.array([p0[2] + a * (p1[2] - p0[2]), p0[2] + b * (p1[2] - p0[2])]) + self.origin_alt_msl - terrain
                if np.min(heights) < lo - EPS or np.max(heights) > hi + EPS:
                    return False
        elif reference in {"QNE", "QNH"}:
            parts = self.datum_parts(reference, p0, p1, t0, t1)
            if not _covers([(a, b) for a, b, _ in parts]):
                return False
            for a, b, offset in parts:
                heights = np.array([p0[2] + a * (p1[2] - p0[2]), p0[2] + b * (p1[2] - p0[2])]) + self.origin_alt_msl - offset
                if np.min(heights) < lo - EPS or np.max(heights) > hi + EPS:
                    return False
        elif reference == "MSL":
            if min(p0[2], p1[2]) + self.origin_alt_msl < lo - EPS or max(p0[2], p1[2]) + self.origin_alt_msl > hi + EPS:
                return False
        else:
            for p, t in ((p0, t0), (p1, t1)):
                geographic = from_wkt(self.frame.to_wgs84(Point(p)))
                height = self.frame.altitude_from_msl(float(p[2]) + self.origin_alt_msl, reference, geographic.x, geographic.y, self.time_origin + timedelta(seconds=float(t)))
                if not lo - EPS <= height <= hi + EPS:
                    return False
        step = content.get("layerStep")
        if step is not None:
            # A height layer is a discrete cruise altitude. Vertical transfer is allowed,
            # but a level leg must use a registered layer anchored at its lower bound.
            if abs(p1[2] - p0[2]) <= EPS:
                anchor = content.get("minHeight", content.get("minAltitude", 0))
                height = p0[2] + self.origin_alt_msl
                if reference == "AGL":
                    layer_heights = [height - terrain for _, _, terrain in self.terrain_parts(p0, p1, t0, t1)]
                elif reference in {"QNE", "QNH"}:
                    layer_heights = [height - offset for _, _, offset in self.datum_parts(reference, p0, p1, t0, t1)]
                elif reference == "ELLIPSOID":
                    # The bounds check above already verifies the declared geoid.
                    layer_heights = [height + self.model.coordinates.geoid_separation_m]
                else:
                    layer_heights = [height]
                return all(abs((value - anchor) / step - round((value - anchor) / step)) < 1e-5 for value in layer_heights)
        return True

    def blocked_indices(self, task: Any, profile: Any, time_s: float = 0) -> np.ndarray:
        """Conservative node-centered 3D raster for native fallback navigation."""
        low = np.array([self.bounds.x_min, self.bounds.y_min, self.bounds.z_min], float)
        high = np.array([self.bounds.x_max, self.bounds.y_max, self.bounds.z_max], float)
        resolution = np.asarray(self.resolution, float)
        # Python integers avoid wrapping a large grid's dimensions or product.
        # NumPy reports the actual allocation/representation failure, if any.
        shape = tuple(ceil((float(hi) - float(lo)) / float(step)) + 1
                      for lo, hi, step in zip(low, high, resolution))
        margin, vertical = self.clearance(profile)
        selected = self.rules.selected(task)
        cache_key = (shape, margin, vertical, round(float(time_s), 6), tuple(str(get(r, "id")) for r in selected), get(task, "route_mode"))
        cache = self._blocked_cache
        if cache_key in cache:
            cache.move_to_end(cache_key)
            return cache[cache_key]
        blocked = np.zeros(shape, dtype=bool)
        shape_array = np.asarray(shape, dtype=np.intp)
        half_horizontal = float(np.linalg.norm(resolution[:2])) / 2
        half_vertical = resolution[2] / 2

        def raster(geometry: BaseGeometry, lo: float, hi: float, target: np.ndarray = blocked) -> None:
            if geometry.is_empty or lo > hi:
                return
            xmin, ymin, xmax, ymax = geometry.bounds
            begin = np.maximum(0, np.floor((np.array([xmin, ymin, lo]) - low) / resolution).astype(np.int64))
            finish = np.minimum(shape_array - 1, np.ceil((np.array([xmax, ymax, hi]) - low) / resolution).astype(np.int64))
            if np.any(begin > finish):
                return
            xi, yi, zi = [np.arange(begin[d], finish[d] + 1) for d in range(3)]
            xx, yy = np.meshgrid(low[0] + xi * resolution[0], low[1] + yi * resolution[1], indexing="ij")
            mask = intersects_xy(geometry, xx, yy)
            # Bounding boxes select candidate indices; exact buffered geometry selects XY.
            z_values = low[2] + zi * resolution[2]
            zi = zi[(z_values >= lo - EPS) & (z_values <= hi + EPS)]
            if len(zi):
                target[np.ix_(xi, yi, zi)] |= mask[:, :, None]

        for volume in self.obstacles + self.zones + self.grid_volumes:
            if not volume.valid_from_s <= time_s <= volume.valid_to_s:
                continue
            geom = self._buffered(volume, margin + volume.buffer_m + half_horizontal)
            if volume.alt_reference == "AGL":
                if self.terrain_constant is not None:
                    terrain_regions = [(geom, self.terrain_constant)]
                else:
                    terrain_regions = [(geom.intersection(c.geometry), float(c.values.get("elevation_msl_m", c.values.get("terrain_msl_m")))) for d in self.datasets_for(None, "terrain") if d.valid_from_s <= time_s <= d.valid_to_s for c in d.cells if _finite(c.values.get("elevation_msl_m", c.values.get("terrain_msl_m"))) and c.valid_from_s <= time_s <= c.valid_to_s]
                for region, elevation in terrain_regions:
                    raster(region, volume.min_altitude + elevation - self.origin_alt_msl - vertical - half_vertical, volume.max_altitude + elevation - self.origin_alt_msl + vertical + half_vertical)
            elif volume.alt_reference in {"QNE", "QNH"}:
                for correction in self.model.coordinates.height_corrections:
                    if correction.reference == volume.alt_reference and self._seconds(correction.valid_from, 0) <= time_s <= self._seconds(correction.valid_to, 0):
                        region = geom.intersection(self._footprint(correction.geometry_wkt)) if correction.geometry_wkt else geom
                        raster(region, volume.min_altitude + correction.offset_m - self.origin_alt_msl - vertical - half_vertical, volume.max_altitude + correction.offset_m - self.origin_alt_msl + vertical + half_vertical)
            else:
                raster(geom, volume.min_altitude - vertical - half_vertical, volume.max_altitude + vertical + half_vertical)
        # Dataset fields must be present and safe in the full node-centered cell.
        for rule in selected:
            if not self.rules.active(rule, time_s):
                continue
            kind = get(rule, "rule_type")
            c = get(rule, "content")
            if kind == "AREA":
                geom = unary_union(self.area_for(rule))
                if c["scope"] == "EXCLUDE":
                    raster(geom.buffer(margin + half_horizontal), low[2], high[2])
                else:
                    allowed = np.zeros_like(blocked)
                    raster(geom.buffer(-margin - half_horizontal), low[2], high[2], allowed)
                    blocked |= ~allowed
            if kind not in {"WEATHER", "SIGNAL", "PRIORITY"} or kind == "PRIORITY" and c["priorityScene"] == "FAST":
                continue
            dataset_kind = kind.lower() if kind != "PRIORITY" else "noise" if c["priorityScene"] == "QUIET" else "risk"
            allowed = np.zeros_like(blocked)
            for ds in self.datasets_for(rule, dataset_kind):
                if not ds.valid_from_s <= time_s <= ds.valid_to_s:
                    continue
                for cell in ds.cells:
                    if not cell.valid_from_s <= time_s <= cell.valid_to_s:
                        continue
                    center = cell.geometry.representative_point()
                    z = float(np.clip((low[2] + high[2]) / 2, cell.min_altitude, cell.max_altitude))
                    point = np.array([center.x, center.y, z])
                    safe = self.priority_data_covered(rule, point, point, time_s, time_s) if kind == "PRIORITY" else self.field_part_allowed(rule, point, point, time_s, time_s, profile)
                    if safe:
                        raster(cell.geometry.buffer(-margin - half_horizontal), max(low[2], cell.min_altitude + vertical + half_vertical), min(high[2], cell.max_altitude - vertical - half_vertical), allowed)
            blocked |= ~allowed
        result = np.argwhere(blocked).astype(np.int64, copy=False)
        cache[cache_key] = result
        return result

    def direction_allowed(self, rule: Any, p0: np.ndarray, p1: np.ndarray) -> bool:
        refs = set(map(str, get(rule, "resource_ids", [])))
        for airway in self.airways:
            if refs and airway.id not in refs:
                continue
            if not airway.geometry.intersects(LineString([p0[:2], p1[:2]])):
                continue
            a, b = airway.centerline.project(Point(p0[:2])), airway.centerline.project(Point(p1[:2]))
            if airway.direction == "REVERSE":
                a, b = b, a
            if b >= a - EPS:
                return True
        return False

    def width_allowed(self, rule: Any, p0: np.ndarray, p1: np.ndarray, parts: list, profile: Any) -> bool:
        refs = set(map(str, get(rule, "resource_ids", [])))
        c = get(rule, "content")
        margin, _ = self.clearance(profile)
        # width is HALF width; bufferWidth is additional mandatory clearance.
        radius = c["width"] - c.get("bufferWidth", 0) - margin
        if radius <= 0:
            return False
        geom = self._rule_geometry(("width", str(get(rule, "id")), margin), lambda: unary_union([a.centerline.buffer(min(a.half_width, c["width"]) - c.get("bufferWidth", 0) - margin) for a in self.airways if (not refs or a.id in refs) and min(a.half_width, c["width"]) > c.get("bufferWidth", 0) + margin]))
        allowed = _geometry_intervals(geom, p0, p1)
        return all(_covers(allowed, a, b) for a, b in parts)

    def rule_volume_collision(self, rule: Any, p0: np.ndarray, p1: np.ndarray, t0: float, t1: float, profile: Any) -> bool:
        kind, content = get(rule, "rule_type"), get(rule, "content")
        horizontal, vertical = self.clearance(profile)
        candidates = self.volumes_for(rule, kind)
        max_extra = content.get("safeDistance", 0) + content.get("bufferRadius", 0)
        tree, collection = (self._obstacle_tree, self.obstacles + self.grid_volumes) if kind == "BUILDING" else (self._zone_tree, self.zones)
        spatial = {id(v) for v in self._query(tree, collection, p0, p1, horizontal + max_extra + (self._max_obstacle_buffer if kind == "BUILDING" else self._max_zone_buffer))}
        for volume in candidates:
            if id(volume) not in spatial:
                continue
            extra = 0
            if kind == "BUILDING":
                extra = content.get("safeDistance", 0)
                if content["avoidMode"] == "BUFFER":
                    extra = max(extra, content.get("bufferRadius", 0))
            if kind != "NO_FLY" or content.get("includeBuffer", False):
                extra += volume.buffer_m
            intervals = self.volume_intervals(volume, p0, p1, t0, t1, horizontal=horizontal + extra, vertical=vertical)
            active = self.rules.interval(rule, t0, t1)
            intervals = _clip_scalar(intervals, t0, t1, *active) if active else []
            if kind == "NO_FLY" and "heightCeiling" in content:
                intervals = _clip_scalar(intervals, p0[2], p1[2], -float("inf"), content["heightCeiling"] - self.origin_alt_msl + vertical)
            if intervals and kind == "BUILDING" and content["avoidMode"] == "CORRIDOR":
                corridor = unary_union([d.geometry for d in self.datasets_for(rule, "corridor")])
                allowed = _geometry_intervals(corridor.buffer(-horizontal), p0, p1)
                if all(_covers(allowed, a, b) for a, b in intervals):
                    continue
            if intervals:
                return True
        return False

    def _field_parts(self, rule: Any, kind: str, p0: np.ndarray, p1: np.ndarray, t0: float, t1: float) -> list[tuple[float, float, dict]]:
        result = []
        for ds in self.datasets_for(rule, kind):
            if t1 < ds.valid_from_s or t0 > ds.valid_to_s:
                continue
            for cell in self._query(ds.tree, ds.cells, p0, p1):
                parts = _geometry_intervals(cell.geometry, p0, p1)
                parts = _clip_scalar(parts, p0[2], p1[2], cell.min_altitude, cell.max_altitude)
                parts = _clip_scalar(parts, t0, t1, max(ds.valid_from_s, cell.valid_from_s), min(ds.valid_to_s, cell.valid_to_s))
                result.extend((a, b, cell.values) for a, b in parts)
        return result

    def field_part_allowed(self, rule: Any, p0: np.ndarray, p1: np.ndarray, t0: float, t1: float, profile: Any) -> bool:
        kind = get(rule, "rule_type")
        c = get(rule, "content")
        parts = self._field_parts(rule, kind.lower(), p0, p1, t0, t1)
        if not _covers([(a, b) for a, b, _ in parts]):
            return False
        for _, _, values in parts:
            if values.get("nodata", False):
                return False
            if kind == "WEATHER":
                wind = values.get("wind_speed_mps", values.get("windSpeed"))
                precipitation = values.get("precipitation", values.get("precipitation_mm_h"))
                icing = values.get("icing")
                if "maxWindSpeed" in c and (not _finite(wind) or wind > c["maxWindSpeed"] + EPS):
                    return False
                if c.get("noPrecipitation") and (not isinstance(precipitation, (bool, int, float)) or precipitation != 0):
                    return False
                if c.get("noIcing") and (not isinstance(icing, (bool, int, float)) or icing != 0):
                    return False
            else:
                supported = {_band(b) for b in get(profile, "supported_bands", [])}
                bands = c.get("bands", list(supported))
                requested = [_band(b) for b in bands if _band(b) in supported]
                if not requested:
                    return False
                if c.get("band_policy", "any") == "all" and set(requested) != {_band(b) for b in bands}:
                    return False
                signals = values.get("bands", values.get("signal_strength_dbm", {}))
                if isinstance(signals, dict):
                    signals = {_band(k): v for k, v in signals.items()}
                    good = [b in signals and _finite(signals[b]) and signals[b] >= c["minSignalStrength"] - EPS for b in requested]
                else:
                    good = [_finite(signals) and signals >= c["minSignalStrength"] - EPS] * len(requested)
                if not (all(good) if c.get("band_policy", "any") == "all" else any(good)):
                    return False
        return True

    def priority_data_covered(self, rule: Any, p0: np.ndarray, p1: np.ndarray, t0: float, t1: float) -> bool:
        kind = "noise" if get(rule, "content")["priorityScene"] == "QUIET" else "risk"
        parts = self._field_parts(rule, kind, p0, p1, t0, t1)
        key = "noise_db" if kind == "noise" else "risk"
        return _covers([(a, b) for a, b, v in parts if not v.get("nodata", False) and _finite(v.get(key))])

    def cost_multiplier(self, point_xyz: Any, time_s: float, task: Any, profile: Any = None) -> float:
        point = np.asarray(point_xyz, float)
        cost = 2.0 if get(task, "route_mode", "strict") == "prefer" and not self.corridor.covers(Point(point[:2])) else 1.0
        for rule in self.rules.selected(task):
            if get(rule, "rule_type") != "PRIORITY" or not self.rules.active(rule, time_s):
                continue
            c = get(rule, "content")
            scene = c["priorityScene"]
            if scene == "FAST":
                cost *= 1 - c["costBonus"]
                continue
            kind, key = ("noise", "noise_db") if scene == "QUIET" else ("risk", "risk")
            parts = self._field_parts(rule, kind, point, point, time_s, time_s)
            values = [v[key] for _, _, v in parts if _finite(v.get(key)) and not v.get("nodata", False)]
            if not values:
                return float("inf")
            # Risk is normalized; noise is mapped monotonically to a nonnegative score.
            score = max(values)
            desirability = max(0.0, 1 - min(1.0, score if kind == "risk" else score / 100))
            cost *= 1 - c["costBonus"] * desirability
        return max(0.001, cost)

    def waypoint_local(self, waypoint: Any, task: Any) -> np.ndarray:
        point = self.frame.to_local(get(waypoint, "wkt"), get(task, "alt_reference"), as_datetime(get(task, "departure_time")))
        return np.asarray(point.coords[0], float)

    def _connector_intervals(self, p0: np.ndarray, p1: np.ndarray, t0: float, t1: float, task: Any, profile: Any, connector_context: dict | None = None) -> list[tuple[float, float]]:
        margin, vertical = self.clearance(profile)
        result = []
        for field_name, endpoint_name in (("start_connector_id", "start_wkt"), ("end_connector_id", "goal_wkt")):
            ref = get(task, field_name)
            if ref is None:
                continue
            connector = self.resources.get(str(ref))
            if not isinstance(connector, PreparedVolume) or connector.kind != "connector":
                invalid("Task connector id does not resolve to a connector", connector_id=ref)
            endpoint = self.frame.to_local(get(task, endpoint_name), get(task, "alt_reference"), as_datetime(get(task, "departure_time")))
            ep = np.asarray(endpoint.coords[0])
            # A connector is authorized only on an actual first/last edge, never an intermediate shortcut.
            is_endpoint_edge = np.linalg.norm(p0 - ep) <= 1e-3 or np.linalg.norm(p1 - ep) <= 1e-3
            if connector_context is not None:
                authorized = connector_context.get(str(ref))
                if authorized is None:
                    continue
            elif not is_endpoint_edge:
                continue
            allowed_ids = set(get(connector.source, "airway_ids", []))
            if allowed_ids and not any(a.id in allowed_ids for a in self.airways):
                invalid("Connector references missing airways", connector_id=ref)
            parts = self.volume_intervals(connector, p0, p1, t0, t1, horizontal=margin, vertical=vertical, inside=True)
            if connector_context is not None:
                a, b = connector_context[str(ref)]
                parts = [(max(a, c), min(b, d)) for c, d in parts if max(a, c) <= min(b, d) + EPS]
            result.extend(parts)
        return result

    def _airway_intervals(self, p0: np.ndarray, p1: np.ndarray, t0: float, t1: float, profile: Any) -> list[tuple[float, float]]:
        margin, vertical = self.clearance(profile)
        covered = []
        for airway in self._query(self._airway_tree, self.airways, p0, p1):
            if airway.status != "ACTIVE":
                continue
            if airway.direction != "BOTH" and np.linalg.norm(p1[:2] - p0[:2]) > EPS:
                a, b = airway.centerline.project(Point(p0[:2])), airway.centerline.project(Point(p1[:2]))
                if (airway.direction == "FORWARD" and b < a - EPS) or (airway.direction == "REVERSE" and b > a + EPS):
                    continue
            covered.extend(self.volume_intervals(airway, p0, p1, t0, t1, horizontal=margin, vertical=vertical, inside=True))
        return _merge(covered)

    def airway_ids_for_segment(self, p0: Any, p1: Any, t0: float, t1: float, profile: Any = None) -> list[str]:
        p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
        return [a.id for a in self._query(self._airway_tree, self.airways, p0, p1)
                if a.status == "ACTIVE" and self.volume_intervals(a, p0, p1, t0, t1)]

    def _corridor_allowed(self, p0: np.ndarray, p1: np.ndarray, t0: float, t1: float, task: Any, profile: Any, connector_context: dict | None = None) -> bool:
        covered = self._airway_intervals(p0, p1, t0, t1, profile)
        covered.extend(self._connector_intervals(p0, p1, t0, t1, task, profile, connector_context))
        return _covers(covered)

    def _connector_contexts(self, points: np.ndarray, times: np.ndarray, task: Any, profile: Any) -> list[dict]:
        contexts: list[dict] = [{} for _ in range(max(0, len(points) - 1))]
        corridor_parts = [self._airway_intervals(points[i], points[i + 1], times[i], times[i + 1], profile) for i in range(len(points) - 1)]
        entries = [(i, a, b) for i, parts in enumerate(corridor_parts) for a, b in parts]
        if not entries:
            return contexts
        first_i, first_a, _ = entries[0]
        last_i, _, last_b = entries[-1]
        for field_name, endpoint_name, indices in (
            ("start_connector_id", "start_wkt", range(first_i + 1)),
            ("end_connector_id", "goal_wkt", range(last_i, len(contexts))),
        ):
            ref = get(task, field_name)
            if not ref:
                continue
            endpoint = self.frame.to_local(get(task, endpoint_name), get(task, "alt_reference"), as_datetime(get(task, "departure_time")))
            actual = points[0] if field_name == "start_connector_id" else points[-1]
            if np.linalg.norm(np.asarray(endpoint.coords[0]) - actual) > 1e-3:
                continue
            connector = self.resources.get(str(ref))
            if not isinstance(connector, PreparedVolume) or connector.kind != "connector":
                invalid("Task connector id does not resolve to a connector", connector_id=ref)
            allowed_ids = set(get(connector.source, "airway_ids", []))
            entry_i = first_i if field_name == "start_connector_id" else last_i
            entry_parameter = first_a if field_name == "start_connector_id" else last_b
            entry_point = points[entry_i] + entry_parameter * (points[entry_i + 1] - points[entry_i])
            if allowed_ids and not any(a.id in allowed_ids and a.geometry.distance(Point(entry_point[:2])) <= EPS for a in self.airways):
                continue
            for i in indices:
                contexts[i][str(ref)] = (0.0, first_a if i == first_i else 1.0) if field_name == "start_connector_id" else (last_b if i == last_i else 0.0, 1.0)
        return contexts

    def _check_geometry(self, p0: Any, p1: Any, t0: float = 0, t1: float = 0, task: Any = None, profile: Any = None, connector_context: dict | None = None, *, include_rules: bool = True) -> list[dict]:
        p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
        errors: list[dict] = []
        horizontal, vertical = self.clearance(profile)
        b = self.bounds
        if any(p[0] - horizontal < b.x_min - EPS or p[0] + horizontal > b.x_max + EPS or p[1] - horizontal < b.y_min - EPS or p[1] + horizontal > b.y_max + EPS or p[2] - vertical < b.z_min - EPS or p[2] + vertical > b.z_max + EPS for p in (p0, p1)):
            errors.append({"rule_id": None, "rule_type": "BOUNDS", "reason": "swept aircraft body exceeds scene bounds"})
        if t0 < -EPS or t1 > self.horizon_s + EPS:
            errors.append({"rule_id": None, "rule_type": "HORIZON", "reason": "segment exceeds scene time horizon"})
        if self.terrain_constant is not None or self.datasets_for(None, "terrain"):
            terrain = self.terrain_parts(p0, p1, t0, t1)
            if not _covers([(a, b) for a, b, _ in terrain]):
                errors.append({"rule_id": None, "rule_type": "TERRAIN_NODATA", "reason": "segment crosses missing or expired terrain data"})
            for a, c, elevation in terrain:
                if min(p0[2] + a * (p1[2] - p0[2]), p0[2] + c * (p1[2] - p0[2])) - vertical + self.origin_alt_msl <= elevation + EPS:
                    errors.append({"rule_id": None, "rule_type": "TERRAIN", "reason": "swept aircraft intersects the terrain surface"})
                    break
        for volume in self._query(self._obstacle_tree, self.obstacles + self.grid_volumes, p0, p1, horizontal + self._max_obstacle_buffer):
            if self.volume_intervals(volume, p0, p1, t0, t1, horizontal=horizontal + volume.buffer_m, vertical=vertical):
                # Explicit BUILDING CORRIDOR may permit traversing an obstacle only
                # when its own rule fully covers this segment in a body-clear corridor.
                corridor_rules = [r for r in self.rules.selected(task) if get(r, "rule_type") == "BUILDING" and get(r, "content").get("avoidMode") == "CORRIDOR" and (not get(r, "resource_ids") or volume.id in get(r, "resource_ids"))]
                if corridor_rules and all(not self.rule_volume_collision(r, p0, p1, t0, t1, profile) for r in corridor_rules):
                    continue
                errors.append({"rule_id": None, "rule_type": "BUILDING" if volume.kind == "obstacle" else "GRID", "reason": "segment intersects an occupied volume", "resource_id": volume.id})
        for volume in self._query(self._zone_tree, self.zones, p0, p1, horizontal + self._max_zone_buffer):
            if self.volume_intervals(volume, p0, p1, t0, t1, horizontal=horizontal + volume.buffer_m, vertical=vertical):
                errors.append({"rule_id": None, "rule_type": "ZONE", "reason": "segment intersects an active control zone", "resource_id": volume.id})
        if get(task, "route_mode", "free") == "strict" and not self._corridor_allowed(p0, p1, t0, t1, task, profile, connector_context):
            errors.append({"rule_id": None, "rule_type": "CORRIDOR", "reason": "swept segment is outside active airway/authorized endpoint connector"})
        # Search edges must respect mandatory geometric/field rules. Dynamic limits
        # are evaluated after the motion model supplies physically meaningful times.
        selected_types = {"HEIGHT_LAYER", "WIDTH", "DIRECTION", "AREA", "NO_FLY", "BUILDING", "ZONE", "WEATHER", "SIGNAL", "PRIORITY"}
        if include_rules:
            errors.extend(self.rules.check_segment(p0, p1, float(t0), float(t1), task, profile, kinds=selected_types))
        return errors

    def check_geometry(self, p0: Any, p1: Any, t0: float = 0, t1: float = 0, task: Any = None, profile: Any = None) -> list[dict]:
        return [_public_violation(v) for v in self._check_geometry(p0, p1, t0, t1, task, profile)]

    def check_path(self, points: Any, times: Any, task: Any, profile: Any, phase: str = "route", *, speed_transient_segments: Any = None) -> list[dict]:
        p, t = np.asarray(points, float), np.asarray(times, float)
        if p.ndim != 2 or p.shape[1] != 3 or t.ndim != 1 or len(p) != len(t) or len(p) < 1 or not np.isfinite(p).all() or not np.isfinite(t).all() or np.any(np.diff(t) < 0):
            invalid("Path requires finite XYZ vertices and nondecreasing matching times")
        errors = []
        lengths = np.linalg.norm(np.diff(p, axis=0), axis=1)
        durations = np.diff(t)
        speed = np.divide(lengths, durations, out=np.full_like(lengths, float("inf")), where=durations > EPS)
        speed[(lengths <= EPS) & (durations <= EPS)] = 0
        # The caller may mark physically generated launch/landing transients. Raw
        # validation remains strict unless the task carries this private context.
        transient_indices = set(speed_transient_segments or [])
        connector_contexts = self._connector_contexts(p, t, task, profile) if get(task, "route_mode") == "strict" else [{} for _ in range(len(p) - 1)]
        if len(p) == 1:
            errors.extend(self._check_geometry(p[0], p[0], t[0], t[0], task, profile))
        for i in range(len(p) - 1):
            errors.extend({**v, "segment": i} for v in self._check_geometry(p[i], p[i + 1], t[i], t[i + 1], task, profile, connector_contexts[i], include_rules=False))
            errors.extend(self.rules.check_segment(p[i], p[i + 1], t[i], t[i + 1], task, profile, i, enforce_min_speed=i not in transient_indices))
            if speed[i] > get(profile, "max_speed_mps", float("inf")) + EPS:
                errors.append({"rule_type": "AIRCRAFT_SPEED", "reason": "segment exceeds aircraft maximum speed", "segment": i})
            climb = (p[i + 1, 2] - p[i, 2]) / durations[i] if durations[i] > EPS else 0
            if climb > get(profile, "climb_speed_mps", float("inf")) + EPS or -climb > get(profile, "descent_speed_mps", float("inf")) + EPS:
                errors.append({"rule_type": "AIRCRAFT_CLIMB", "reason": "segment exceeds aircraft climb or descent speed", "segment": i})
            if lengths[i] <= EPS and durations[i] > EPS and not get(profile, "can_hover", True):
                errors.append({"rule_type": "AIRCRAFT_HOVER", "reason": "aircraft does not support stationary flight", "segment": i})
            for airway in self.airways:
                constraints = get(airway.source, "climbConstraint")
                if constraints is None:
                    continue
                if not self.volume_intervals(airway, p[i], p[i + 1], t[i], t[i + 1]):
                    continue
                max_climb, max_descent = get(constraints, "maxClimbRate"), get(constraints, "maxDescentRate")
                if climb > max_climb + EPS or -climb > max_descent + EPS:
                    errors.append({"rule_type": "AIRWAY_CLIMB", "reason": "segment exceeds this airway's climb constraints", "segment": i, "resource_id": airway.id})
        errors.extend(self.rules.check_path(p, t, task, profile, phase))
        turn_accumulations: dict[str, tuple[float, int]] = {}
        for i in range(1, len(p) - 1):
            a, c = p[i, :2] - p[i - 1, :2], p[i + 1, :2] - p[i, :2]
            la, lc = float(np.linalg.norm(a)), float(np.linalg.norm(c))
            if min(la, lc) <= EPS:
                turn_accumulations.clear()
                continue
            angle = acos(float(np.clip(np.dot(a, c) / (la * lc), -1, 1)))
            radius = float(np.linalg.norm(a + c) / (2 * sin(angle))) if EPS < angle < pi - EPS else float("inf") if angle <= EPS else 0
            if radius + EPS < get(profile, "min_turn_radius_m", 0):
                errors.append({"rule_type": "AIRCRAFT_TURN", "reason": "path curvature exceeds aircraft minimum turn radius", "segment": i})
            for airway in self.airways:
                constraint = get(airway.source, "turnConstraint")
                if constraint is None or not self.volume_intervals(airway, p[i], p[i], t[i], t[i]):
                    turn_accumulations.pop(airway.id, None)
                    continue
                cross = a[0] * c[1] - a[1] * c[0]
                sign = 1 if cross > EPS else -1 if cross < -EPS else 0
                accumulated, previous_sign = turn_accumulations.get(airway.id, (0.0, 0))
                accumulated = accumulated + angle if sign == previous_sign else angle
                if angle < 1e-6:
                    accumulated, sign = 0.0, 0
                turn_accumulations[airway.id] = (accumulated, sign)
                if degrees(accumulated) > get(constraint, "maxTurnAngle", 180) + EPS or radius + EPS < get(constraint, "minTurnRadius", 0):
                    errors.append({"rule_type": "AIRWAY_TURN", "reason": "path violates this airway's turn constraints", "segment": i, "resource_id": airway.id})
        state = get(task, "state")
        remaining_time = get(state, "remaining_flight_time_s")
        remaining_range = get(state, "remaining_range_m")
        budget_time = min(get(profile, "endurance_s", float("inf")), float("inf") if remaining_time is None else remaining_time)
        budget_range = min(get(profile, "max_range_m", float("inf")), float("inf") if remaining_range is None else remaining_range)
        if float(t[-1] - t[0]) > budget_time + EPS or float(lengths.sum()) > budget_range + EPS:
            errors.append({"rule_type": "AIRCRAFT_ENDURANCE", "reason": "route exceeds aircraft capacity or declared remaining flight budget"})
        if get(task, "payload_kg", 0) > get(profile, "payload_kg", 0) + EPS:
            errors.append({"rule_type": "AIRCRAFT_PAYLOAD", "reason": "task payload exceeds aircraft payload capacity"})
        # Report one failure per rule/reason/segment when both search and final checks apply.
        unique = {}
        for error in errors:
            key = (error.get("rule_id"), error.get("rule_type"), error.get("reason"), error.get("segment"), error.get("resource_id"))
            unique[key] = error
        return [_public_violation(v) for v in unique.values()]

    validate_path = check_path


def _finite(value: Any) -> bool:
    return not isinstance(value, bool) and isinstance(value, (int, float)) and isfinite(value)


def _band(value: str) -> str:
    return str(value).upper().replace("HZ", "").replace(" ", "")


def _public_violation(value: dict) -> dict:
    return {"code": str(value.get("rule_type", "CONSTRAINT")), "message": value.get("reason", "constraint violated"),
            "rule_id": value.get("rule_id"), "time_s": value.get("time_s"),
            "details": {k: v for k, v in value.items() if k not in {"reason", "rule_type", "rule_id", "time_s"}}}
