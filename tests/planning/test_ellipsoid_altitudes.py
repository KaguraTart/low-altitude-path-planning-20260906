"""Canonical EG regression checks with nonzero origin and geoid separation."""
from datetime import datetime, timezone

import numpy as np
import pytest
from shapely import from_wkt
from shapely.geometry import Point

from uav_planner.v2.coordinates import CoordinateFrame
from uav_planner.v2.engine import plan_batch_v2, validate_route_v2
from uav_planner.v2.scene import CompiledScene


AT = datetime(2026, 10, 29, tzinfo=timezone.utc)
COORDINATES = {
    "origin_lon": 104.78, "origin_lat": 29.34, "origin_alt_ellipsoid": 300,
    "geoid_separation_m": 28, "geoid_source": "synthetic-nonzero-geoid",
}


def scene():
    return {
        "coordinates": dict(COORDINATES), "time_origin": AT, "horizon_s": 300,
        "bounds": {"x_min": -20, "x_max": 120, "y_min": -20, "y_max": 20,
                   "z_min": 0, "z_max": 200},
    }


def terrain_cell(x0, x1, height):
    return {"bounds": {"x_min": x0, "x_max": x1, "y_min": -20, "y_max": 20,
                       "z_min": 0, "z_max": 200}, "values": {"elevation_ellipsoid_m": height}}


def test_agl_clearance_is_checked_along_route_in_ellipsoid_terrain_cells():
    value = scene()
    value["datasets"] = [{"id": "terrain", "version": "synthetic-v1", "kind": "terrain",
                          "cells": [terrain_cell(-20, 120, 310), terrain_cell(49, 51, 370)]}]
    compiled = CompiledScene(value)
    start, finish = np.array([0, 0, 100]), np.array([100, 0, 100])  # EG 400.
    constraint = {"minHeight": 40, "maxHeight": 100, "altDatum": "AGL"}
    assert compiled.height_part_allowed(start, start, 0, 0, constraint)
    assert compiled.height_part_allowed(finish, finish, 20, 20, constraint)
    assert not compiled.height_part_allowed(start, finish, 0, 20, constraint)
    parts = compiled.terrain_parts(start, finish, 0, 20)
    assert any(height == 370 and a < 0.5 < b for a, b, height in parts)
    assert compiled.frame.altitude_to_local(30, "AGL", COORDINATES["origin_lon"],
                                            COORDINATES["origin_lat"], AT) == 40


@pytest.mark.parametrize("reference,reported_height", [("EG", 380), ("ELLIPSOID", 380), ("MSL", 352)])
def test_planning_and_validation_preserve_requested_reference_over_canonical_eg(reference, reported_height):
    value = scene()
    value["coordinates"]["terrain_ellipsoid_m"] = 310
    frame = CoordinateFrame(value["coordinates"])
    task = {
        "id": "nonzero-datum", "route_mode": "free", "departure_time": AT,
        "alt_reference": reference,
        "start_wkt": frame.to_wgs84(Point(0, 0, 80), reference),
        "goal_wkt": frame.to_wgs84(Point(100, 0, 80), reference),
    }
    batch = plan_batch_v2(scene=value, tasks=[task])
    result = batch["routes"][0]
    assert result["status"] == "planned", result
    assert result["alt_reference"] == reference
    assert result["coordinate_context"] == batch["coordinate_context"]
    assert from_wkt(result["trajectory_wkt"]).coords[0][2] == pytest.approx(reported_height)
    assert batch["coordinate_context"]["canonical_alt_reference"] == "EG"
    assert batch["coordinate_context"]["origin_alt_ellipsoid"] == 300
    checked = validate_route_v2(scene=value, task=task, trajectory_wkt=result["trajectory_wkt"],
                                vertex_times_s=result["vertex_times_s"], motion_segments=result["motion_segments"])
    assert checked["valid"], checked


def test_failed_route_still_declares_input_reference_and_canonical_context():
    value = scene()
    frame = CoordinateFrame(value["coordinates"])
    task = {"id": "failed-datum", "route_mode": "free", "departure_time": AT,
            "alt_reference": "EG", "start_wkt": frame.to_wgs84(Point(0, 0, 80)),
            "goal_wkt": frame.to_wgs84(Point(100, 0, 80))}
    batch = plan_batch_v2(scene=value, tasks=[task], time_budget_s=1e-12)
    result = batch["routes"][0]
    assert result["status"] == "timeout"
    assert result["alt_reference"] == "EG"
    assert result["coordinate_context"]["canonical_alt_reference"] == "EG"
    assert result["coordinate_context"]["origin_alt_ellipsoid"] == 300
