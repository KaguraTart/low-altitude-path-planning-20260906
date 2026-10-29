"""Rule effects, exact boundaries, and missing-data failure contracts."""
from copy import deepcopy
from datetime import datetime, timedelta

import numpy as np
import pytest
from shapely.geometry import LineString, Point, box

from uav_planner.errors import PlanningError
from uav_planner.v2.coordinates import CoordinateFrame
from uav_planner.v2.scene import CompiledScene
from uav_planner.v2.rules import RULE_KEYS, _daily_interval_allowed, separation_violations


ORIGIN = datetime.fromisoformat("2026-10-16T08:00:00+08:00")
COORD = {"origin_lon": 104.0, "origin_lat": 29.0, "origin_alt_ellipsoid": 0}
FRAME = CoordinateFrame(COORD)
PROFILE = {"radius_m": 0, "height_m": 0, "safety_margin_m": 0, "max_speed_mps": 100,
           "climb_speed_mps": 100, "descent_speed_mps": 100, "endurance_s": 1000,
           "max_range_m": 10000, "payload_kg": 5, "supported_bands": ["2.4GHz", "5.8GHz"], "can_hover": True}


def geo(geometry):
    return FRAME.to_wgs84(geometry)


def base_scene(rule_type=None, content=None, refs=None):
    data = {"coordinates": COORD, "bounds": {"x_min": -500, "x_max": 500, "y_min": -500, "y_max": 500, "z_min": -100, "z_max": 500},
            "time_origin": ORIGIN, "rules": [], "datasets": []}
    if rule_type:
        data["rules"] = [{"id": "r", "rule_type": rule_type, "content": content, "resource_ids": refs or []}]
    return data


def task(**kwargs):
    return {"id": "t", "route_mode": "free", "departure_time": ORIGIN, "alt_reference": "EG", "payload_kg": 0,
            "start_wkt": geo(Point(0, 0, 100)), "goal_wkt": geo(Point(100, 0, 100)), **kwargs}


def check(scene, points=None, times=None, **task_kwargs):
    return CompiledScene(scene).check_path(points if points is not None else [[0, 0, 100], [100, 0, 100]], times if times is not None else [0, 20], task(**task_kwargs), PROFILE)


def rule_fails(errors, kind):
    return any(e["code"] == kind and e.get("rule_id") == "r" for e in errors)


def airway(id="a", width=100):
    grid = {"minLon": 103.99, "maxLon": 104.01, "minLat": 28.99, "maxLat": 29.01, "minAlt": 0, "maxAlt": 200}
    return {"id": id, "centerLine": geo(LineString([(-100, 0, 100), (200, 0, 100)])), "widthMeters": width,
            "minAltitude": 0, "maxAltitude": 200, "altReference": "EG", "startGrid": grid, "endGrid": grid}


def zone(id="z", **kwargs):
    return {"id": id, "geom": geo(box(40, -1, 41, 1)), "minAlt": 90, "maxAlt": 110,
            "altReference": "EG", "zoneType": "POLYGON", "level": "A", **kwargs}


def dataset(kind, values, bounds=None, id="d", **kwargs):
    return {"id": id, "kind": kind, "version": "test-data-1", "cells": [{"bounds": bounds or {"x_min": -200, "x_max": 200, "y_min": -200, "y_max": 200, "z_min": -100, "z_max": 500}, "values": values}], **kwargs}


def test_all20_registry_is_complete():
    assert len(RULE_KEYS) == 20


def test_height_layer_bounds_and_terrain_between_vertices():
    scene = base_scene("HEIGHT_LAYER", {"minAltitude": 90, "maxAltitude": 100, "altDatum": "EG"})
    assert not rule_fails(check(scene), "HEIGHT_LAYER")
    assert rule_fails(check(scene, [[0, 0, 100.01], [100, 0, 100.01]]), "HEIGHT_LAYER")
    scene["rules"][0]["content"] = {"minAltitude": 40, "maxAltitude": 200, "altDatum": "AGL"}
    scene["datasets"] = [dataset("terrain", {"elevation_ellipsoid_m": 0}), dataset("terrain", {"elevation_ellipsoid_m": 70}, {"x_min": 49, "x_max": 51, "y_min": -5, "y_max": 5, "z_min": -100, "z_max": 500}, id="hill")]
    assert rule_fails(check(scene), "HEIGHT_LAYER")


@pytest.mark.parametrize("reference", ["QNE", "QNH", "MSL"])
def test_height_layer_step_uses_the_declared_vertical_datum(reference):
    scene = base_scene("HEIGHT_LAYER", {"minHeight": 0, "maxHeight": 200,
                                         "layerStep": 20, "altDatum": reference})
    scene["coordinates"] = {**COORD}
    if reference == "MSL":
        scene["coordinates"]["geoid_separation_m"] = 5
        scene["coordinates"]["geoid_source"] = "test-geoid"
        on_layer_eg = 85
    else:
        scene["coordinates"]["height_corrections"] = [{
            "reference": reference, "offset_m": 5, "source": "test-pressure",
            "version": "test-data-1", "valid_from": ORIGIN,
            "valid_to": ORIGIN + timedelta(hours=1),
        }]
        on_layer_eg = 85
    # EG 80 is a multiple of the step, but its declared-datum height is not.
    assert rule_fails(check(scene, [[0, 0, 80], [100, 0, 80]]), "HEIGHT_LAYER")
    assert not rule_fails(check(scene, [[0, 0, on_layer_eg], [100, 0, on_layer_eg]]), "HEIGHT_LAYER")


def test_terrain_field_overrides_constant_and_nodata_cannot_hide_under_other_cells():
    scene = base_scene()
    scene["coordinates"] = {**COORD, "terrain_ellipsoid_m": 0}
    scene["datasets"] = [dataset("terrain", {"elevation_ellipsoid_m": 120})]
    assert any(e["code"] == "TERRAIN" for e in check(scene))
    scene["datasets"][0]["cells"][0]["values"] = {"elevation_ellipsoid_m": 0}
    scene["datasets"].append(dataset("terrain", {"nodata": True}, {"x_min": 49, "x_max": 51, "y_min": -5, "y_max": 5, "z_min": -100, "z_max": 500}, id="gap"))
    assert any(e["code"] == "TERRAIN_NODATA" for e in check(scene))


def test_width_is_full_width_with_external_buffer():
    scene = base_scene("WIDTH", {"width": 20, "bufferWidth": 5}, ["a"])
    scene["airways"] = [airway()]
    assert not rule_fails(check(scene, [[0, 9.99, 100], [100, 9.99, 100]]), "WIDTH")
    assert rule_fails(check(scene, [[0, 10.01, 100], [100, 10.01, 100]]), "WIDTH")


def test_direction_north_zero_clockwise():
    scene = base_scene("DIRECTION", {"directions": [90]})
    assert not rule_fails(check(scene), "DIRECTION")
    assert rule_fails(check(scene, [[100, 0, 100], [0, 0, 100]]), "DIRECTION")


def test_speed_exact_threshold_and_effective_period():
    scene = base_scene("SPEED", {"minSpeed": 2, "maxSpeed": 5})
    assert not rule_fails(check(scene), "SPEED")
    assert rule_fails(check(scene, times=[0, 19.9]), "SPEED")
    scene["rules"][0]["effective_start"] = ORIGIN + timedelta(seconds=20)
    assert not rule_fails(check(scene, times=[0, 19.9]), "SPEED")
    assert CompiledScene(scene).rules.report(task(), times=[0, 19.9])[0]["status"] == "not_applicable"


def test_time_full_segment_daily_and_cross_midnight_weekday():
    scene = base_scene("TIME", {"dailyStart": "08:00", "dailyEnd": "08:01", "weekdays": [5]})
    assert not rule_fails(check(scene, times=[0, 60]), "TIME")
    assert rule_fails(check(scene, times=[0, 60.01]), "TIME")
    assert _daily_interval_allowed(datetime.fromisoformat("2026-10-16T23:30:00+08:00"), datetime.fromisoformat("2026-10-17T00:30:00+08:00"), {"dailyStart": "23:00", "dailyEnd": "01:00", "weekdays": [5]})


def test_area_exclude_detects_middle_crossing():
    scene = base_scene("AREA", {"scope": "EXCLUDE"}, ["d"])
    scene["datasets"] = [{"id": "d", "kind": "area", "version": "test-data-1", "geometry_wkt": geo(box(49, -1, 51, 1))}]
    assert rule_fails(check(scene), "AREA")
    assert not rule_fails(check(scene, [[0, 2, 100], [100, 2, 100]]), "AREA")


def test_area_grid_codes_respect_vertical_limits():
    scene = base_scene("AREA", {"scope": "INCLUDE", "gridCodes": ["G"]})
    scene["grid"] = {"origin": [0, -10, 90], "resolution": [100, 20, 20], "shape": [1, 1, 1], "codes": {"0,0,0": "G"}}
    assert not rule_fails(check(scene), "AREA")
    assert rule_fails(check(scene, [[0, 0, 111], [100, 0, 111]]), "AREA")
    scene["rules"][0]["content"]["scope"] = "EXCLUDE"
    assert not rule_fails(check(scene, [[0, 0, 111], [100, 0, 111]]), "AREA")


def test_climb_rate_boundary():
    scene = base_scene("CLIMB", {"maxClimbRate": 2, "maxDescentRate": 1})
    assert not rule_fails(check(scene, [[0, 0, 100], [0, 0, 120]], [0, 10]), "CLIMB")
    assert rule_fails(check(scene, [[0, 0, 100], [0, 0, 120.1]], [0, 10]), "CLIMB")


def test_turn_angle_and_radius():
    scene = base_scene("TURN", {"maxTurnAngle": 90, "minTurnRadius": 5})
    assert not rule_fails(check(scene, [[0, 0, 100], [10, 0, 100], [10, 10, 100]], [0, 10, 20]), "TURN")
    assert rule_fails(check(scene, [[0, 0, 100], [1, 0, 100], [1, 1, 100]], [0, 10, 20]), "TURN")


def test_separation_is_continuous_and_boundary_safe():
    p = [[-10, 0, 100], [10, 0, 100]]
    q = [[0, -10, 100], [0, 10, 100]]
    assert separation_violations(p, [0, 1], q, [0, 1], 1, 1)
    assert not separation_violations(p, [0, 1], [[-10, 1, 101], [10, 1, 101]], [0, 1], 1, 1)
    s = CompiledScene(base_scene("SEPARATION", {"horizontalSeparation": 10, "verticalSeparation": 5}))
    assert s.rules.separation(task()) == (10, 5)


def test_no_fly_midsegment_and_temporary_period_required():
    scene = base_scene("NO_FLY", {"noFlyMode": "PERMANENT", "heightCeiling": 110, "includeBuffer": False}, ["z"])
    scene["control_zones"] = [zone()]
    assert rule_fails(check(scene), "NO_FLY")
    assert not rule_fails(check(scene, [[0, 0, 111], [100, 0, 111]]), "NO_FLY")
    scene["rules"][0]["content"]["noFlyMode"] = "TEMPORARY"
    with pytest.raises(PlanningError, match="effective period"):
        CompiledScene(scene)


def test_building_buffer_distance():
    scene = base_scene("BUILDING", {"avoidMode": "BUFFER", "safeDistance": 5, "bufferRadius": 10}, ["b"])
    scene["obstacles"] = [{"id": "b", "geometry_wkt": geo(box(40, 9, 41, 10)), "min_altitude": 90, "max_altitude": 110, "alt_reference": "EG"}]
    assert rule_fails(check(scene), "BUILDING")
    assert not rule_fails(check(scene, [[0, -2, 100], [100, -2, 100]]), "BUILDING")


def test_zone_partial_geometry_not_coverage_threshold():
    scene = base_scene("ZONE", {"zoneLevels": ["A"], "coverageThreshold": 0.99}, ["z"])
    scene["control_zones"] = [zone()]
    assert rule_fails(check(scene), "ZONE")
    assert not rule_fails(check(scene, [[0, 2, 100], [100, 2, 100]]), "ZONE")


def test_weather_threshold_expiration_and_nodata():
    scene = base_scene("WEATHER", {"maxWindSpeed": 10, "noPrecipitation": True, "noIcing": True})
    scene["datasets"] = [dataset("weather", {"wind_speed_mps": 10, "precipitation": False, "icing": False})]
    assert not rule_fails(check(scene), "WEATHER")
    scene["datasets"][0]["cells"][0]["values"]["wind_speed_mps"] = 10.01
    assert rule_fails(check(scene), "WEATHER")
    scene["datasets"][0]["cells"][0]["values"]["wind_speed_mps"] = 10
    scene["datasets"][0]["valid_to"] = ORIGIN + timedelta(seconds=19)
    assert rule_fails(check(scene), "WEATHER")


def test_signal_band_any_all_and_supported_filter():
    scene = base_scene("SIGNAL", {"minSignalStrength": -85, "bands": ["2.4G", "5.8G"]})
    scene["datasets"] = [dataset("signal", {"bands": {"2.4G": -85, "5.8G": -90}})]
    assert not rule_fails(check(scene), "SIGNAL")
    scene["rules"][0]["content"]["band_policy"] = "all"
    assert rule_fails(check(scene), "SIGNAL")


def test_priority_quiet_field_cost_and_missing_requirement():
    scene = base_scene("PRIORITY", {"priorityScene": "QUIET", "costBonus": 0.5})
    with pytest.raises(PlanningError, match="dataset"):
        CompiledScene(scene)
    scene["datasets"] = [dataset("noise", {"noise_db": 20})]
    compiled = CompiledScene(scene)
    assert compiled.cost_multiplier([0, 0, 100], 0, task(), PROFILE) == pytest.approx(0.6)
    assert not rule_fails(check(scene), "PRIORITY")
    scene["datasets"][0]["cells"][0]["values"] = {"nodata": True}
    assert rule_fails(check(scene), "PRIORITY")


def test_hover_only_marked_waypoints_and_exact_duration():
    scene = base_scene("HOVER", {"maxHoverTime": 10, "defaultHoverTime": 5})
    wp = {"wkt": geo(Point(0, 0, 100)), "hover_s": 10}
    assert not rule_fails(check(scene, [[0, 0, 100], [0, 0, 100]], [0, 10], waypoints=[wp]), "HOVER")
    assert rule_fails(check(scene, [[0, 0, 100], [0, 0, 100]], [0, 10.01], waypoints=[wp]), "HOVER")
    assert rule_fails(check(scene, [[0, 0, 100], [0, 0, 100]], [0, 1]), "HOVER")
    assert rule_fails(check(scene, [[0, 0, 100], [0, 0, 100], [0, 0, 100]], [0, 6, 12], waypoints=[wp]), "HOVER")
    assert not rule_fails(check(scene, [[0, 0, 100], [0, 0, 100]], [0, 5], waypoints=[{"wkt": wp["wkt"], "default_hover": True}]), "HOVER")


def test_endurance_capacity_floor_and_remaining_budget_are_separate():
    scene = base_scene("ENDURANCE", {"minEndurance": 1000, "maxRange": 100})
    assert not rule_fails(check(scene), "ENDURANCE")
    assert rule_fails(check(scene, state={"remaining_flight_time_s": 19}), "ENDURANCE")
    assert rule_fails(check(scene, [[0, 0, 100], [100.01, 0, 100]]), "ENDURANCE")
    scene["rules"][0]["content"]["minEndurance"] = 1000.1
    assert rule_fails(check(scene), "ENDURANCE")


def test_payload_capacity_and_actual_mass():
    scene = base_scene("PAYLOAD", {"maxPayloadKg": 5})
    assert not rule_fails(check(scene, payload_kg=5), "PAYLOAD")
    assert rule_fails(check(scene, payload_kg=5.01), "PAYLOAD")
    scene["rules"][0]["content"]["maxPayloadKg"] = 5.01
    assert rule_fails(check(scene), "PAYLOAD")


def test_waypoint_count_and_interval_use_business_waypoints():
    scene = base_scene("WAYPOINT", {"maxWaypoints": 3, "minWaypointInterval": 50})
    assert not rule_fails(check(scene, waypoints=[{"wkt": geo(Point(50, 0, 100))}]), "WAYPOINT")
    assert rule_fails(check(scene, waypoints=[{"wkt": geo(Point(49, 0, 100))}]), "WAYPOINT")


def test_rtl_low_battery_requires_upstream_state_and_eg_altitude():
    scene = base_scene("RTL", {"rtlAltitude": 100, "rtlOnLowBattery": True, "lowBatteryPercent": 20})
    compiled = CompiledScene(scene)
    with pytest.raises(PlanningError, match="upstream"):
        compiled.rules.rtl_requirements(task(), PROFILE)
    state = {"battery_percent": 20, "remaining_flight_time_s": 100, "remaining_range_m": 1000, "home_wkt": geo(Point(0, 0, 100))}
    assert compiled.rules.rtl_requirements(task(state=state), PROFILE)["required"]
    errors = compiled.check_path([[0, 0, 99], [100, 0, 99]], [0, 20], task(state=state), PROFILE, phase="rtl")
    assert rule_fails(errors, "RTL")


@pytest.mark.parametrize("kind,content", [("SPEED", {"maxSpeeed": 10}), ("SIGNAL", {"bands": ["2.4G"]}), ("TIME", {"dailyStart": "08:00"}), ("WIDTH", {"width": True})])
def test_unknown_missing_or_mistyped_rule_content_is_invalid(kind, content):
    with pytest.raises(PlanningError):
        CompiledScene(base_scene(kind, content))


def test_strict_corridor_body_width_status_direction_and_prefer_penalty():
    scene = base_scene()
    scene["airways"] = [airway(width=20)]
    compiled = CompiledScene(scene)
    assert not compiled.check_path([[0, 9.99, 100], [100, 9.99, 100]], [0, 20], task(route_mode="strict"), PROFILE)
    assert compiled.check_path([[0, 10.01, 100], [100, 10.01, 100]], [0, 20], task(route_mode="strict"), PROFILE)
    assert compiled.cost_multiplier([0, 11, 100], 0, task(route_mode="prefer")) == 2
    scene["airways"][0]["direction"] = "FORWARD"
    assert CompiledScene(scene).check_path([[100, 0, 100], [0, 0, 100]], [0, 20], task(route_mode="strict"), PROFILE)


def test_segment_vertical_and_time_clipping_are_simultaneous():
    scene = base_scene()
    scene["control_zones"] = [zone(effective_start=ORIGIN + timedelta(seconds=8), effective_end=ORIGIN + timedelta(seconds=9))]
    compiled = CompiledScene(scene)
    assert compiled.check_path([[0, 0, 100], [100, 0, 100]], [0, 20], task(), PROFILE)
    scene["control_zones"][0]["effective_start"] = ORIGIN + timedelta(seconds=9)
    scene["control_zones"][0]["effective_end"] = ORIGIN + timedelta(seconds=10)
    assert not CompiledScene(scene).check_path([[0, 0, 100], [100, 0, 100]], [0, 20], task(), PROFILE)


def test_connector_authorizes_split_entry_prefix_and_not_intermediate_shortcuts():
    scene = base_scene()
    item = airway(width=20)
    item["centerLine"] = geo(LineString([(0, 0, 100), (200, 0, 100)]))
    scene["airways"] = [item]
    scene["connectors"] = [{"id": "entry", "geometry_wkt": geo(box(-110, -15, 10, 15)), "min_altitude": 0,
                            "max_altitude": 200, "alt_reference": "EG", "airway_ids": ["a"]}]
    compiled = CompiledScene(scene)
    t = task(route_mode="strict", start_wkt=geo(Point(-100, 0, 100)), start_connector_id="entry")
    assert not compiled.check_path([[-100, 0, 100], [-80, 0, 100], [-30, 0, 100], [0, 0, 100], [100, 0, 100]], [0, 10, 20, 30, 40], t, PROFILE)
    # Once the route has entered the airway, returning through the entry connector is forbidden.
    errors = compiled.check_path([[-100, 0, 100], [0, 0, 100], [-50, 0, 100], [100, 0, 100]], [0, 10, 20, 40], t, PROFILE)
    assert any(e["code"] == "CORRIDOR" for e in errors)


def test_single_point_path_and_grid_raster_include_thin_obstacle():
    scene = base_scene()
    scene["resolution"] = [10, 10, 10]
    scene["obstacles"] = [{"id": "thin", "geometry_wkt": geo(box(49.9, -1, 50.1, 1)), "min_altitude": 95, "max_altitude": 105, "alt_reference": "EG"}]
    compiled = CompiledScene(scene)
    assert not compiled.check_path([[0, 0, 100]], [0], task(), PROFILE)
    blocked = compiled.blocked_indices(task(), PROFILE, 0)
    assert np.any(np.all(blocked == [55, 50, 20], axis=1))  # node (50,0,100)
    assert compiled.blocked_indices(task(), PROFILE, 0) is blocked


def test_per_airway_climb_and_turn_limits_apply_independently():
    scene = base_scene()
    scene["airways"] = [airway()]
    scene["airways"][0]["climbConstraint"] = {"maxClimbRate": 1, "maxDescentRate": 1}
    scene["airways"][0]["turnConstraint"] = {"maxTurnAngle": 45, "minTurnRadius": 5}
    assert any(e["code"] == "AIRWAY_CLIMB" for e in check(scene, [[0, 0, 100], [10, 0, 120]], [0, 10]))
    assert any(e["code"] == "AIRWAY_TURN" for e in check(scene, [[0, 0, 100], [10, 0, 100], [10, 10, 100]], [0, 10, 20]))


def test_forecast_cells_and_pressure_corrections_cover_whole_time_interval():
    scene = base_scene("WEATHER", {"maxWindSpeed": 10})
    scene["datasets"] = [dataset("weather", {"wind_speed_mps": 5})]
    scene["datasets"][0]["cells"][0]["valid_to"] = ORIGIN + timedelta(seconds=10)
    assert rule_fails(check(scene), "WEATHER")
    scene = base_scene("HEIGHT_LAYER", {"minAltitude": 90, "maxAltitude": 110, "altDatum": "QNH"})
    scene["coordinates"] = {**COORD, "height_corrections": [{"reference": "QNH", "offset_m": 0, "source": "test", "version": "test-data-1", "valid_from": ORIGIN, "valid_to": ORIGIN + timedelta(seconds=10)}]}
    assert rule_fails(check(scene), "HEIGHT_LAYER")
    scene = base_scene()
    scene["coordinates"] = {**COORD, "height_corrections": [{"reference": "QNE", "offset_m": 0, "source": "test", "version": "test-data-1", "valid_from": ORIGIN + timedelta(hours=1), "valid_to": ORIGIN + timedelta(hours=2)}]}
    scene["obstacles"] = [{"id": "wall", "geometry_wkt": geo(box(40, -10, 60, 10)), "min_altitude": 0, "max_altitude": 300, "alt_reference": "QNE"}]
    with pytest.raises(PlanningError, match="missing correction coverage"):
        check(scene)


def test_turn_limit_cannot_be_bypassed_by_dense_arc_sampling():
    scene = base_scene("TURN", {"maxTurnAngle": 45, "minTurnRadius": 5})
    angles = np.linspace(0, np.pi / 2, 20)
    points = np.column_stack((10 * np.cos(angles), 10 * np.sin(angles), np.full(len(angles), 100)))
    assert rule_fails(check(scene, points, np.arange(len(points))), "TURN")


def test_rule_reports_match_public_violation_contract():
    scene = CompiledScene(base_scene("SPEED", {"maxSpeed": 4}))
    errors = scene.check_path([[0, 0, 100], [100, 0, 100]], [0, 20], task(), PROFILE)
    assert set(errors[0]) == {"code", "message", "rule_id", "time_s", "details"}
    report = scene.rules.report(task(), errors, times=[0, 20])[0]
    assert report["status"] == "failed"
    assert report["rule_id"] == "r"


@pytest.mark.parametrize("kind,content",[
    ("HEIGHT_LAYER",{"minHeight":0,"altDatum":[]}),
    ("DIRECTION",{"direction":{}}),
    ("AREA",{"scope":[]}),
    ("AREA",{"scope":"EXCLUDE","zoneIds":[{}]}),
])
def test_malformed_enum_and_reference_types_are_input_errors(kind,content):
    with pytest.raises(PlanningError) as caught:
        CompiledScene(base_scene(kind,content))
    assert caught.value.code=="INVALID_INPUT"
