"""Shared temporal boundaries and datum-safe external obstacle trajectories.

Trajectory times are absolute seconds relative to the scene time origin.  A
reported AGL or pressure-altitude segment is linear only while its additive
height datum remains constant.  Exact spatial and temporal partitions establish
that condition before a track is given to the continuous collision checker.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, time, timedelta
from math import isfinite
from typing import TYPE_CHECKING

import numpy as np
from shapely import force_2d, from_wkt
from shapely.geometry import LineString, Point
from shapely.geometry.base import BaseGeometry

from .coordinates import CoordinateError
from .models import Scene, TimedTrajectory
from .rules import LOCAL_TZ

if TYPE_CHECKING:
    from .scene import CompiledScene


def temporal_breakpoints(compiled: CompiledScene, model: Scene) -> tuple[float, ...]:
    """Cache every declared time boundary, including local daily rule windows.

    Explicit timestamps retain their scene-relative value even outside the
    planning horizon.  Recurring wall-clock boundaries are expanded over the
    finite horizon, using the same timezone as TIME rule evaluation.
    """
    cached = getattr(compiled, "_safety_temporal_breakpoints", None)
    if cached is not None and cached[0] is model:
        return cached[1]

    boundaries = {0.0, float(model.horizon_s)}

    def add_timestamp(value: datetime | None) -> None:
        if value is not None:
            boundaries.add(float((value - model.time_origin).total_seconds()))

    for name in ("airways", "connectors", "obstacles", "control_zones", "datasets"):
        for item in getattr(model, name):
            add_timestamp(item.valid_from)
            add_timestamp(item.valid_to)
    for dataset in model.datasets:
        for cell in dataset.cells:
            add_timestamp(cell.valid_from)
            add_timestamp(cell.valid_to)
    for correction in model.coordinates.height_corrections:
        add_timestamp(correction.valid_from)
        add_timestamp(correction.valid_to)

    daily_times: set[time] = set()
    weekdays = False
    for rule in model.rules:
        add_timestamp(rule.effective_start)
        add_timestamp(rule.effective_end)
        if rule.rule_type == "TIME":
            weekdays |= "weekdays" in rule.content
            for key in ("dailyStart", "dailyEnd", "earliestDeparture", "latestArrival"):
                if key in rule.content:
                    daily_times.add(time.fromisoformat(str(rule.content[key])))
    if weekdays:
        daily_times.add(time())
    if daily_times:
        start = model.time_origin.astimezone(LOCAL_TZ)
        finish = (model.time_origin + timedelta(seconds=model.horizon_s)).astimezone(LOCAL_TZ)
        day = start.date()
        while day <= finish.date():
            for daily_time in daily_times:
                at = datetime.combine(day, daily_time, LOCAL_TZ)
                second = float((at - model.time_origin).total_seconds())
                if 0 <= second <= model.horizon_s:
                    boundaries.add(second)
            day += timedelta(days=1)

    result = tuple(sorted(value for value in boundaries if isfinite(value)))
    compiled._safety_temporal_breakpoints = (model, result)
    return result


@dataclass(frozen=True)
class _DatumRegion:
    geometry: BaseGeometry | None
    start: float
    end: float
    offset: float | None


def _datum_regions(compiled: CompiledScene, model: Scene, reference: str) -> list[_DatumRegion]:
    if reference == "AGL":
        datasets = compiled.datasets_for(None, "terrain")
        if not datasets:
            offset = model.coordinates.terrain_ellipsoid_m
            return [] if offset is None else [_DatumRegion(None, -float("inf"), float("inf"), float(offset))]
        result = []
        for dataset in datasets:
            for cell in dataset.cells:
                value = cell.values.get("elevation_ellipsoid_m", cell.values.get("terrain_ellipsoid_m"))
                offset = None if cell.values.get("nodata", False) or value is None else float(value)
                result.append(_DatumRegion(
                    cell.geometry.intersection(dataset.geometry),
                    max(dataset.valid_from_s, cell.valid_from_s),
                    min(dataset.valid_to_s, cell.valid_to_s), offset,
                ))
        return result
    result = []
    for correction in model.coordinates.height_corrections:
        if correction.reference != reference:
            continue
        geometry = None
        if correction.geometry_wkt is not None:
            geometry = compiled.frame.to_local(force_2d(from_wkt(correction.geometry_wkt)).wkt, "EG")
        result.append(_DatumRegion(
            geometry, (correction.valid_from - model.time_origin).total_seconds(),
            (correction.valid_to - model.time_origin).total_seconds(), float(correction.offset_m),
        ))
    return result


def _offset_at(regions: list[_DatumRegion], reference: str, point: np.ndarray, second: float) -> float:
    location = Point(float(point[0]), float(point[1]))
    candidates = [region.offset for region in regions
                  if region.start <= second < region.end
                  and (region.geometry is None or region.geometry.covers(location))]
    if not candidates or any(value is None or not isfinite(value) for value in candidates):
        raise ValueError(f"{reference} dynamic trajectory has missing height datum coverage")
    if reference != "AGL" and len(candidates) != 1:
        raise ValueError(f"{reference} dynamic trajectory has ambiguous height datum coverage")
    # Terrain uses the same conservative highest-surface policy as CompiledScene.
    return max(value for value in candidates if value is not None)


def _spatial_parameters(geometry: BaseGeometry, start: np.ndarray, end: np.ndarray) -> set[float]:
    """Find all exact entry/exit parameters, including isolated point contacts."""
    delta = end[:2] - start[:2]
    denominator = float(delta @ delta)
    if denominator == 0 or geometry.is_empty:
        return set()
    intersection = geometry.intersection(LineString([start[:2], end[:2]]))
    parameters: set[float] = set()

    def collect(part: BaseGeometry) -> None:
        if part.is_empty:
            return
        if hasattr(part, "geoms"):
            for child in part.geoms:
                collect(child)
            return
        for coordinate in part.coords:
            value = float((np.asarray(coordinate[:2]) - start[:2]) @ delta / denominator)
            parameters.add(min(1.0, max(0.0, value)))

    collect(intersection)
    return parameters


def _variable_track(
    compiled: CompiledScene, model: Scene, dynamic: TimedTrajectory,
    regions: list[_DatumRegion], boundaries: tuple[float, ...],
) -> tuple[np.ndarray, np.ndarray]:
    geographic = np.asarray(from_wkt(dynamic.trajectory_wkt).coords, dtype=float)
    # EG here projects XY while retaining each reported altitude as raw data.
    raw = np.asarray(compiled.frame.to_local(dynamic.trajectory_wkt, "EG").coords, dtype=float).copy()
    raw[:, 2] = geographic[:, 2]
    times = np.asarray(dynamic.vertex_times_s, dtype=float)
    reference = dynamic.alt_reference
    offsets = [_offset_at(regions, reference, point, float(second)) for point, second in zip(raw, times)]
    # Every declared vertex is converted at its own absolute scene datetime.
    converted = []
    for coordinate, point, second, offset in zip(geographic, raw, times, offsets):
        at = model.time_origin + timedelta(seconds=float(second))
        height = compiled.frame.altitude_to_local(float(coordinate[2]), reference,
                                                 float(coordinate[0]), float(coordinate[1]), at)
        expected = point[2] + offset - model.coordinates.origin_alt_ellipsoid
        if abs(height - expected) > 1e-7:
            raise ValueError(f"{reference} dynamic trajectory has inconsistent height datum evidence")
        converted.append(np.array([point[0], point[1], height], dtype=float))
    if len(raw) == 1:
        return np.asarray(converted), times.copy()

    points = [converted[0]]
    seconds = [float(times[0])]
    for index, (start, end, t0, t1) in enumerate(zip(raw, raw[1:], times, times[1:])):
        parameters = {0.0, 1.0}
        parameters.update((boundary - t0) / (t1 - t0) for boundary in boundaries if t0 < boundary < t1)
        for region in regions:
            if region.geometry is not None and region.start <= t1 and region.end >= t0:
                parameters.update(_spatial_parameters(region.geometry, start, end))
        ordered = sorted(parameters)
        basis = offsets[index]
        for a, b in zip(ordered, ordered[1:]):
            # Within this exact partition, coverage and offset are constant.
            middle = (a + b) / 2
            point = start + (end - start) * middle
            second = float(t0 + (t1 - t0) * middle)
            if _offset_at(regions, reference, point, second) != basis:
                raise ValueError(f"{reference} dynamic trajectory crosses a discontinuous height datum basis")
            point = start + (end - start) * b
            second = float(t0 + (t1 - t0) * b)
            if _offset_at(regions, reference, point, second) != basis:
                raise ValueError(f"{reference} dynamic trajectory crosses a discontinuous height datum basis")
            if b == 1.0:
                points.append(converted[index + 1])
            else:
                points.append(np.array([point[0], point[1], point[2] + basis - model.coordinates.origin_alt_ellipsoid]))
            seconds.append(second)
    return np.asarray(points, dtype=float), np.asarray(seconds, dtype=float)


def dynamic_tracks(compiled: CompiledScene, model: Scene) -> list[tuple[np.ndarray, np.ndarray]]:
    """Return exactly one fully validated local track per dynamic obstacle.

    Height evidence is checked over every spatial/time partition, including
    boundary contacts.  Gaps, ambiguity, or discontinuous additive offsets are
    input errors; an unsafe obstacle is never silently dropped or interpolated.
    """
    cached = getattr(compiled, "_safety_dynamic_tracks", None)
    if cached is not None and cached[0] is model:
        return cached[1]
    result = []
    regions: dict[str, list[_DatumRegion]] = {}
    boundaries = temporal_breakpoints(compiled, model)
    for dynamic in model.dynamic_obstacles:
        try:
            if dynamic.alt_reference in {"EG", "MSL", "ELLIPSOID"}:
                geometry = compiled.frame.to_local(dynamic.trajectory_wkt, dynamic.alt_reference, model.time_origin)
                track = np.asarray(geometry.coords, dtype=float), np.asarray(dynamic.vertex_times_s, dtype=float)
            else:
                if dynamic.alt_reference not in regions:
                    regions[dynamic.alt_reference] = _datum_regions(compiled, model, dynamic.alt_reference)
                track = _variable_track(compiled, model, dynamic, regions[dynamic.alt_reference], boundaries)
        except (CoordinateError, ValueError) as exc:
            raise ValueError(f"Dynamic obstacle {dynamic.id!r}: {exc}") from exc
        result.append(track)
    compiled._safety_dynamic_tracks = (model, result)
    return result
