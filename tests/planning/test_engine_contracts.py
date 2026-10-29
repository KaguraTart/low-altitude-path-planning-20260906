"""Execution checks for the 2026-10-28 ENG-06/ENG-07 rule contract."""
from datetime import datetime

import numpy as np
import pytest
from pydantic import ValidationError
from shapely.geometry import Point, Polygon, box

from server.v2 import example_request, capabilities
from uav_planner.errors import PlanningError
from uav_planner.v2.coordinates import CoordinateFrame
from uav_planner.v2.engine import plan_single_v2
from uav_planner.v2.models import Rule, Scene, Task
from uav_planner.v2.rules import _daily_interval_allowed
from uav_planner.v2.scene import CompiledScene


def setup(rule):
    request = example_request()
    request["scene"]["rules"] = [rule]
    request["task"]["route_mode"] = "free"
    frame = CoordinateFrame(request["scene"]["coordinates"])
    return request, frame


def test_rulebo_aliases_and_platform_local_dates():
    rule = Rule(id="r", type="SPEED", constraints={"maxSpeed": 5},
                effective_start="2026-10-29 08:00:00", effective_end="")
    assert rule.rule_type == "SPEED" and rule.content == {"maxSpeed": 5}
    assert rule.effective_start.isoformat() == "2026-10-29T08:00:00+08:00"
    assert rule.effective_end is None
    assert "constraints" not in rule.model_dump()


@pytest.mark.parametrize("scope", ["BBOX", "WKT"])
def test_area_geometry_changes_continuous_validation(scope):
    request, frame = setup({"id": "area", "rule_type": "AREA", "content": {"scope": scope}})
    polygon = frame.to_wgs84(box(50, 50, 400, 150))
    from shapely import from_wkt
    content = request["scene"]["rules"][0]["content"]
    if scope == "WKT":
        content["areaWkt"] = polygon
    else:
        a, b, c, d = from_wkt(polygon).bounds
        content.update(minLon=a, minLat=b, maxLon=c, maxLat=d)
    compiled = CompiledScene(request["scene"])
    rule = compiled.model.rules[0]
    profile = {"radius_m": 0, "height_m": 0, "safety_margin_m": 0}
    assert compiled.area_allowed(rule, np.array([100,100,80]), np.array([300,100,80]), [(0,1)], profile)
    assert not compiled.area_allowed(rule, np.array([100,100,80]), np.array([450,100,80]), [(0,1)], profile)


def test_scoped_building_geometry_blocks_between_sparse_vertices():
    request, frame = setup({"id": "building", "rule_type": "BUILDING", "content": {"avoidMode": "STRICT", "safeDistance": 2}})
    geometry = frame.to_wgs84(Polygon([(199,90,60),(201,90,60),(201,110,100),(199,110,100),(199,90,60)]))
    request["scene"]["rules"][0]["scope"] = {"grid": [{"geometry": geometry}]}
    compiled = CompiledScene(request["scene"])
    task = Task.model_validate(request["task"])
    profile = {"radius_m": 0, "height_m": 0, "safety_margin_m": 0}
    assert any(e["rule_id"] == "building" for e in compiled.check_geometry([100,100,80], [300,100,80], 0, 40, task, profile))
    assert not compiled.check_geometry([100,150,80], [300,150,80], 0, 40, task, profile)
    # Envelope contact must be checked even when the centerline misses scope.
    assert any(e["rule_id"] == "building" for e in compiled.check_geometry([100,111,80], [300,111,80], 0, 40, task, profile))


def test_scope_grid_index_requires_geometry_context_and_resolves_cell():
    rule = {"id": "building", "rule_type": "BUILDING", "content": {"avoidMode": "STRICT"}, "scope": {"grid": [{"x": 1,"y": 0,"z": 0}]}}
    request, _ = setup(rule)
    with pytest.raises(PlanningError, match="scene.grid"):
        CompiledScene(request["scene"])
    request["scene"]["grid"] = {"origin": [100,90,60], "resolution": [100,20,40], "shape": [3,1,1]}
    compiled = CompiledScene(request["scene"])
    assert compiled.rule_scopes["building"][0].geometry.bounds == (200,90,300,110)


def test_weather_scope_does_not_require_coverage_outside_its_region():
    request, frame = setup({"id": "wind", "rule_type": "WEATHER", "content": {"maxWindSpeed": 5}})
    bounds = {"x_min": 100,"x_max": 300,"y_min": 90,"y_max": 110,"z_min": 60,"z_max": 100}
    geometry = frame.to_wgs84(Polygon([(100,90,60),(300,90,60),(300,110,100),(100,110,100),(100,90,60)]))
    request["scene"]["rules"][0]["scope"] = {"grid": [{"geometry": geometry}]}
    request["scene"]["datasets"] = [{"id":"wind-data","kind":"weather","version":"1","cells":[{"bounds":bounds,"values":{"wind_speed_mps":10}}]}]
    compiled = CompiledScene(request["scene"])
    task = Task.model_validate(request["task"])
    assert compiled.rules.check_segment(np.array([100,100,80]),np.array([300,100,80]),0,40,task,{})
    assert not compiled.rules.check_segment(np.array([100,150,80]),np.array([300,150,80]),0,40,task,{})


def test_priority_scope_changes_cost_only_inside_region():
    request, frame = setup({"id":"priority","rule_type":"PRIORITY","content":{"priorityScene":"PLANNING","costBonus":.25}})
    request["scene"]["rules"][0]["scope"] = {"geom":[{"type":"AIRSPACE","boundary_2d":frame.to_wgs84(box(100,90,300,110))}]}
    compiled = CompiledScene(request["scene"])
    task = Task.model_validate(request["task"])
    assert compiled.cost_multiplier([200,100,80],0,task) == .75
    assert compiled.cost_multiplier([200,150,80],0,task) == 1


def test_engine_ids_and_profile_admission_affect_planning():
    request = example_request()
    request["scene"]["airways"][0]["allowed_profile_ids"] = ["other-aircraft"]
    result = plan_single_v2(**request)
    assert result["status"] != "planned" and result["engine_id"] == "ENG-06"
    request["scene"]["airways"][0]["allowed_profile_ids"] = ["dji_mini_4_pro"]
    result = plan_single_v2(**request)
    assert result["status"] == "planned"
    request["task"]["route_mode"] = "free"
    assert plan_single_v2(**request)["engine_id"] == "ENG-07"
    with pytest.raises(ValidationError, match="ENG-06"):
        Task.model_validate({**request["task"], "engine_id":"ENG-06"})


def test_unbound_defaults_and_bound_values_are_reported():
    request = example_request()
    request["scene"]["rules"] = []
    compiled = CompiledScene(request["scene"])
    selected = {r.rule_type:r.content for r in compiled.rules.selected(request["task"])}
    assert selected["CLIMB"] == {"maxClimbRate":5,"maxDescentRate":3}
    assert selected["TURN"] == {"maxTurnAngle":90,"minTurnRadius":10}
    assert {r["rule_id"] for r in plan_single_v2(**request)["rule_report"]} == {"default:CLIMB","default:TURN"}
    request["scene"]["rules"] = [{"id":"turn","rule_type":"TURN","content":{"minTurnRadius":20}}]
    assert CompiledScene(request["scene"]).rules.selected(request["task"])[0].content["minTurnRadius"] == 20


def test_sunday_is_zero_and_via_count_excludes_endpoints():
    sunday = datetime.fromisoformat("2026-11-01T09:00:00+08:00")
    assert _daily_interval_allowed(sunday,sunday,{"weekdays":[0]})
    assert not _daily_interval_allowed(sunday,sunday,{"weekdays":[1]})
    request, _ = setup({"id":"via","rule_type":"WAYPOINT","content":{"maxWaypoints":0}})
    assert plan_single_v2(**request)["status"] == "planned"
    assert capabilities()["canonical_alt_reference"] == "EG"


@pytest.mark.parametrize("content", [{"scope":"BBOX","minLon":2}, {"scope":"WKT","areaWkt":"POINT (104 29)"}])
def test_invalid_area_parameters_are_rejected(content):
    request, _ = setup({"id":"area","rule_type":"AREA","content":content})
    with pytest.raises(PlanningError):
        CompiledScene(request["scene"])


def test_width_buffer_checks_outer_protection_without_reducing_flight_width():
    request, frame = setup({"id":"width","rule_type":"WIDTH","content":{"width":40,"bufferWidth":10}})
    request["scene"]["obstacles"] = [{"id":"outer","geometry_wkt":frame.to_wgs84(box(200,125,210,127)),
        "min_altitude":60,"max_altitude":100,"alt_reference":"EG"}]
    compiled = CompiledScene(request["scene"])
    rule = compiled.model.rules[0]
    profile = {"radius_m":0,"height_m":0,"safety_margin_m":0}
    assert not compiled.width_allowed(rule,np.array([100,100,80]),np.array([300,100,80]),[(0,1)],profile)
    request["scene"]["rules"][0]["content"]["bufferWidth"] = 0
    compiled = CompiledScene(request["scene"])
    assert compiled.width_allowed(compiled.model.rules[0],np.array([100,119,80]),np.array([300,119,80]),[(0,1)],profile)


def test_scoped_corridor_does_not_exempt_unrelated_building():
    request, frame = setup({"id":"passage","rule_type":"BUILDING","content":{"avoidMode":"CORRIDOR"}})
    unrelated = frame.to_wgs84(Polygon([(100,300,60),(150,300,60),(150,310,100),(100,310,100),(100,300,60)]))
    request["scene"]["rules"][0]["scope"] = {"grid":[{"geometry":unrelated}]}
    request["scene"]["datasets"] = [{"id":"passage-data","kind":"corridor","version":"1","geometry_wkt":frame.to_wgs84(box(100,300,150,310))}]
    request["scene"]["obstacles"] = [{"id":"building","geometry_wkt":frame.to_wgs84(box(200,90,210,110)),"min_altitude":60,"max_altitude":100,"alt_reference":"EG"}]
    compiled = CompiledScene(request["scene"])
    errors = compiled.check_geometry([100,100,80],[300,100,80],0,40,Task.model_validate(request["task"]),{})
    assert any(e["code"] == "BUILDING" for e in errors)


def test_width_checks_scoped_obstacles_and_clips_its_own_scope():
    request, frame = setup({"id":"width","rule_type":"WIDTH","content":{"width":40,"bufferWidth":10}})
    geometry = frame.to_wgs84(Polygon([(250,125,60),(260,125,60),(260,127,100),(250,127,100),(250,125,60)]))
    request["scene"]["rules"].append({"id":"obstacle","rule_type":"BUILDING","content":{"avoidMode":"STRICT"},"scope":{"grid":[{"geometry":geometry}]}})
    profile = {"radius_m":0,"height_m":0,"safety_margin_m":0}
    compiled = CompiledScene(request["scene"])
    points = np.array([[100,100,80],[300,100,80]])
    assert not compiled.width_allowed(compiled.model.rules[0],*points,[(0,1)],profile)
    request["scene"]["rules"][0]["scope"] = {"grid":[{"geometry":frame.to_wgs84(Polygon([(100,90,60),(150,90,60),(150,110,100),(100,110,100),(100,90,60)]))}]}
    compiled = CompiledScene(request["scene"])
    rule = compiled.model.rules[0]
    parts = compiled.rule_parts(rule,*points,0,40)
    assert compiled.width_allowed(rule,*points,parts,profile,0,40)


def test_prefer_does_not_bypass_aircraft_admission():
    request = example_request()
    request["task"]["route_mode"] = "prefer"
    request["scene"]["airways"][0]["allowed_profile_ids"] = ["other"]
    route = plan_single_v2(**request)
    assert route["status"] != "planned"  # Both endpoints are in the prohibited corridor.


def test_scoped_separation_is_rejected_instead_of_applied_globally():
    with pytest.raises(ValidationError, match="scope.grid"):
        Rule(id="separation",rule_type="SEPARATION",content={"horizontalSeparation":100},scope={"grid":[{"x":0,"y":0,"z":0}]})


@pytest.mark.parametrize("patch", [
    {"valid_from":"2020-01-01T00:00:00Z","valid_to":"2020-01-02T00:00:00Z"},
    {"cells":[{"bounds":{"x_min":100,"x_max":300,"y_min":90,"y_max":110,"z_min":10,"z_max":20},"values":{}}]},
    {"cells":[{"bounds":{"x_min":100,"x_max":300,"y_min":90,"y_max":110,"z_min":60,"z_max":100},"values":{"nodata":True}}]},
])
def test_scoped_corridor_requires_valid_time_height_and_data(patch):
    request, frame = setup({"id":"passage","rule_type":"BUILDING","content":{"avoidMode":"CORRIDOR"}})
    geometry = frame.to_wgs84(Polygon([(200,90,60),(210,90,60),(210,110,100),(200,110,100),(200,90,60)]))
    request["scene"]["rules"][0]["scope"] = {"grid":[{"geometry":geometry}]}
    request["scene"]["datasets"] = [{"id":"passage-data","kind":"corridor","version":"1","geometry_wkt":frame.to_wgs84(box(100,90,300,110)),**patch}]
    compiled = CompiledScene(request["scene"])
    assert any(e.get("rule_id") == "passage" for e in compiled.check_geometry([100,100,80],[300,100,80],0,40,Task.model_validate(request["task"]),{}))


def test_width_respects_scoped_no_fly_height_ceiling():
    request, frame = setup({"id":"width","rule_type":"WIDTH","content":{"width":40,"bufferWidth":10}})
    geometry = frame.to_wgs84(Polygon([(200,125,60),(210,125,60),(210,127,100),(200,127,100),(200,125,60)]))
    request["scene"]["rules"].append({"id":"no-fly","rule_type":"NO_FLY","content":{"noFlyMode":"STANDARD","heightCeiling":70},"scope":{"geom":[{"type":"NO_FLY_ZONE","geom":geometry}]}})
    compiled = CompiledScene(request["scene"])
    assert compiled.width_allowed(compiled.model.rules[0],np.array([100,100,80]),np.array([300,100,80]),[(0,1)],{})


def test_corridor_cells_do_not_extend_declared_dataset_footprint():
    request, frame = setup({"id":"passage","rule_type":"BUILDING","content":{"avoidMode":"CORRIDOR"}})
    geometry = frame.to_wgs84(Polygon([(200,90,60),(210,90,60),(210,110,100),(200,110,100),(200,90,60)]))
    request["scene"]["rules"][0]["scope"] = {"grid":[{"geometry":geometry}]}
    request["scene"]["datasets"] = [{"id":"passage-data","kind":"corridor","version":"1","geometry_wkt":frame.to_wgs84(box(100,300,150,310)),"cells":[{"bounds":{"x_min":100,"x_max":300,"y_min":90,"y_max":110,"z_min":60,"z_max":100},"values":{}}]}]
    compiled = CompiledScene(request["scene"])
    assert any(e.get("rule_id") == "passage" for e in compiled.check_geometry([100,100,80],[300,100,80],0,40,Task.model_validate(request["task"]),{}))
