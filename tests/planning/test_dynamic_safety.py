"""Exact datum coverage and shared temporal partition regressions."""

from datetime import datetime, timedelta, timezone

import numpy as np
import pytest
from shapely.geometry import LineString, Point, box

from uav_planner.v2.coordinates import CoordinateFrame
from uav_planner.v2.models import Scene
from uav_planner.v2.safety import dynamic_tracks, temporal_breakpoints
from uav_planner.v2.scene import CompiledScene


ORIGIN = datetime(2026, 10, 16, 15, tzinfo=timezone.utc)  # Friday 23:00 in Shanghai.


def _at(second):
    return ORIGIN + timedelta(seconds=second)


def _scene(**updates):
    value = {
        "coordinates": {"origin_lon": 104.78, "origin_lat": 29.34, "origin_alt_msl": 0},
        "bounds": {"x_min": -100, "x_max": 100, "y_min": -100, "y_max": 100,
                   "z_min": -1000, "z_max": 1000},
        "time_origin": ORIGIN, "horizon_s": 100,
    }
    value.update(updates)
    return Scene.model_validate(value)


def _trajectory(reference, times=(1, 9), coordinates=((-10, 0, 10), (10, 0, 10)), **updates):
    frame = CoordinateFrame(_scene().coordinates)
    geometry = Point(coordinates[0]) if len(coordinates) == 1 else LineString(coordinates)
    value = {"id": "dynamic", "trajectory_wkt": frame.to_wgs84(geometry),
             "vertex_times_s": list(times), "alt_reference": reference}
    value.update(updates)
    return value


def _correction(start, end, offset=12, **updates):
    value = {"reference": "QNH", "offset_m": offset, "source": "station",
             "version": f"{start}:{end}", "valid_from": _at(start), "valid_to": _at(end)}
    value.update(updates)
    return value


def _terrain_cell(x0, x1, height=10, **updates):
    value = {"bounds": {"x_min": x0, "x_max": x1, "y_min": -100, "y_max": 100,
                        "z_min": -1000, "z_max": 1000}, "values": {"elevation_msl_m": height}}
    value.update(updates)
    return value


def test_constant_references_keep_one_track_per_obstacle_and_absolute_scene_times():
    model = _scene(dynamic_obstacles=[_trajectory("MSL", times=(10, 20)),
                                     _trajectory("ELLIPSOID", id="ellipsoid", times=(30, 40))])
    model.coordinates.geoid_separation_m = 7
    compiled = CompiledScene(model)
    tracks = dynamic_tracks(compiled, model)
    assert len(tracks) == 2
    assert tracks[0][1].tolist() == [10, 20]
    assert tracks[1][1].tolist() == [30, 40]
    assert tracks[0][0][:, 2].tolist() == [10, 10]
    assert tracks[1][0][:, 2].tolist() == [3, 3]
    assert dynamic_tracks(compiled, model) is tracks


def test_each_pressure_vertex_uses_its_own_datetime_and_same_offset_boundaries_split(monkeypatch):
    model = _scene(coordinates={"origin_lon": 104.78, "origin_lat": 29.34,
                               "height_corrections": [_correction(1, 5), _correction(5, 20)]},
                   dynamic_obstacles=[_trajectory("QNH")])
    compiled = CompiledScene(model)
    convert = compiled.frame.altitude_to_local
    calls = []

    def recording_convert(height, reference, lon, lat, at=None):
        if reference == "QNH":
            calls.append(at)
        return convert(height, reference, lon, lat, at)

    monkeypatch.setattr(compiled.frame, "altitude_to_local", recording_convert)
    points, seconds = dynamic_tracks(compiled, model)[0]
    assert calls == [_at(1), _at(9)]
    assert seconds.tolist() == [1, 5, 9]
    assert np.allclose(points[:, 2], 22)


@pytest.mark.parametrize("corrections", [
    [_correction(0, 5, 12), _correction(5, 20, 20)],
    [_correction(0, 4), _correction(6, 20)],
])
def test_pressure_datum_discontinuity_or_hidden_time_gap_is_rejected(corrections):
    model = _scene(coordinates={"origin_lon": 104.78, "origin_lat": 29.34,
                               "height_corrections": corrections},
                   dynamic_obstacles=[_trajectory("QNH")])
    with pytest.raises(ValueError, match="discontinuous|missing"):
        dynamic_tracks(CompiledScene(model), model)


def test_same_height_terrain_cells_split_track_at_exact_spatial_boundary():
    model = _scene(datasets=[{"id": "terrain", "kind": "terrain", "version": "test-data-1", "cells": [
        _terrain_cell(-100, 0), _terrain_cell(0, 100)]}], dynamic_obstacles=[_trajectory("AGL")])
    points, seconds = dynamic_tracks(CompiledScene(model), model)[0]
    assert seconds.tolist() == pytest.approx([1, 5, 9], abs=1e-6)
    assert points[:, 2].tolist() == [20, 20, 20]
    assert points[1, 0] == pytest.approx(0, abs=1e-7)


@pytest.mark.parametrize("cells", [
    [_terrain_cell(-100, 0), _terrain_cell(0, 100, height=20)],
    [_terrain_cell(-100, -1), _terrain_cell(1, 100)],
    [_terrain_cell(-100, 100), _terrain_cell(-0.01, 0.01, values={"nodata": True})],
])
def test_hidden_terrain_gap_nodata_and_discontinuity_are_rejected(cells):
    model = _scene(coordinates={"origin_lon": 104.78, "origin_lat": 29.34, "terrain_msl_m": 500},
                   datasets=[{"id": "terrain", "kind": "terrain", "version": "test-data-1", "cells": cells}],
                   dynamic_obstacles=[_trajectory("AGL")])
    with pytest.raises(ValueError, match="discontinuous|missing"):
        dynamic_tracks(CompiledScene(model), model)


def test_terrain_dataset_and_cell_temporal_gap_is_rejected_despite_constant_fallback():
    cells = [_terrain_cell(-100, 100, valid_from=_at(0), valid_to=_at(4)),
             _terrain_cell(-100, 100, valid_from=_at(6), valid_to=_at(20))]
    model = _scene(coordinates={"origin_lon": 104.78, "origin_lat": 29.34, "terrain_msl_m": 10},
                   datasets=[{"id": "terrain", "kind": "terrain", "version": "test-data-1", "cells": cells}],
                   dynamic_obstacles=[_trajectory("AGL")])
    with pytest.raises(ValueError, match="missing"):
        dynamic_tracks(CompiledScene(model), model)


def test_spatial_pressure_correction_gap_inside_segment_is_rejected():
    frame = CoordinateFrame(_scene().coordinates)
    corrections = [_correction(0, 20, geometry_wkt=frame.to_wgs84(box(-100, -100, -1, 100))),
                   _correction(0, 20, geometry_wkt=frame.to_wgs84(box(1, -100, 100, 100)))]
    model = _scene(coordinates={"origin_lon": 104.78, "origin_lat": 29.34,
                               "height_corrections": corrections},
                   dynamic_obstacles=[_trajectory("QNH")])
    with pytest.raises(ValueError, match="missing"):
        dynamic_tracks(CompiledScene(model), model)


def test_singleton_dynamic_track_is_kept_and_uses_its_declared_time():
    model = _scene(coordinates={"origin_lon": 104.78, "origin_lat": 29.34,
                               "height_corrections": [_correction(10, 20)]},
                   dynamic_obstacles=[_trajectory("QNH", times=(10,), coordinates=((0, 0, 10),))])
    points, seconds = dynamic_tracks(CompiledScene(model), model)[0]
    assert points.shape == (1, 3)
    assert points[0, 2] == 22
    assert seconds.tolist() == [10]


def test_temporal_breakpoints_cover_resources_cells_corrections_and_local_daily_rules():
    frame = CoordinateFrame(_scene().coordinates)
    model = _scene(horizon_s=86400,
        coordinates={"origin_lon": 104.78, "origin_lat": 29.34,
                     "height_corrections": [_correction(100, 200)]},
        obstacles=[{"id": "wall", "geometry_wkt": frame.to_wgs84(box(-1, -1, 1, 1)),
                    "min_altitude": 0, "max_altitude": 10, "alt_reference": "MSL",
                    "valid_from": _at(2), "valid_to": _at(4)}],
        datasets=[{"id": "terrain", "kind": "terrain", "version": "test-data-1",
                   "valid_from": _at(7), "valid_to": _at(90),
                   "cells": [_terrain_cell(-100, 100, valid_from=_at(8), valid_to=_at(80))]}],
        rules=[{"id": "time", "rule_type": "TIME", "effective_start": _at(3),
                "effective_end": _at(80000), "content": {
                    "dailyStart": "23:45", "dailyEnd": "00:15", "weekdays": [5, 6],
                    "earliestDeparture": "23:15", "latestArrival": "23:30"}}])
    compiled = CompiledScene(model)
    boundaries = temporal_breakpoints(compiled, model)
    assert {0, 2, 3, 4, 7, 8, 80, 90, 100, 200, 900, 1800, 2700,
            3600, 4500, 80000, 86400} <= set(boundaries)
    assert boundaries == tuple(sorted(set(boundaries)))
    assert temporal_breakpoints(compiled, model) is boundaries
