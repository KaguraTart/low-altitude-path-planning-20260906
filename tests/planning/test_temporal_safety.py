"""Motion proof, ordered passage and timestamped datum regressions."""
from copy import deepcopy
from datetime import datetime, timedelta

import pytest
from shapely import from_wkt
from shapely.geometry import LineString, Point, box

from server.v2 import example_request
from uav_planner.v2.coordinates import CoordinateFrame
from uav_planner.v2.engine import plan_single_v2, validate_route_v2


def request():
    value=example_request()
    value["scene"]["airways"]=[]
    value["task"]["route_mode"]="free"
    value["task"]["aircraft"]={"radius_m":.01,"height_m":.01,"safety_margin_m":0}
    return value,CoordinateFrame(value["scene"]["coordinates"])


def at(value, seconds):
    return (datetime.fromisoformat(value["scene"]["time_origin"])+timedelta(seconds=seconds)).isoformat()


def validate(value, route, **updates):
    return validate_route_v2(**value,trajectory_wkt=route["trajectory_wkt"],
        vertex_times_s=route["vertex_times_s"],motion_segments=route["motion_segments"],**updates)


def test_timed_thin_wall_is_checked_on_accelerating_motion():
    value,frame=request()
    value["scene"]["obstacles"]=[{"id":"thin-wall","geometry_wkt":frame.to_wgs84(box(104.7,90,105.1,110)),
        "min_altitude":70,"max_altitude":90,"alt_reference":"EG","valid_from":at(value,2.15),"valid_to":at(value,2.25)}]
    route=plan_single_v2(**value)
    assert route["status"]!="planned",route


def test_rule_activating_during_acceleration_limits_analytic_peak():
    value,_=request()
    value["scene"]["rules"]=[{"id":"slow","rule_type":"SPEED","content":{"maxSpeed":3.5},
        "effective_start":at(value,2.15),"effective_end":at(value,2.25)}]
    route=plan_single_v2(**value)
    assert route["status"]=="planned",route
    assert max(p["profile"]["peak_speed_mps"] for p in route["motion_segments"])<=3.5


def test_moving_wkt_requires_analytic_motion_model():
    value,_=request();route=plan_single_v2(**value)
    check=validate_route_v2(**value,trajectory_wkt=route["trajectory_wkt"],vertex_times_s=route["vertex_times_s"])
    assert not check["valid"]
    assert any(v["code"]=="MOTION_MODEL_REQUIRED" for v in check["violations"])
    assert validate(value,route)["valid"]


def test_tampered_motion_cannot_be_certified_by_plausible_wkt():
    value,_=request();route=plan_single_v2(**value)
    route["motion_segments"][0]["profile"]["accel_time_s"]*=.5
    check=validate(value,route)
    assert not check["valid"]
    assert any(v["code"]=="MOTION_MODEL" for v in check["violations"])


def test_motion_vertices_must_follow_certified_time_law():
    value,frame=request();route=plan_single_v2(**value)
    coordinates=list(from_wkt(route["trajectory_wkt"]).coords)
    coordinates[1]=tuple(from_wkt(frame.to_wgs84(Point(110,100,80))).coords[0])
    route["trajectory_wkt"]=LineString(coordinates).wkt
    check=validate(value,route)
    assert not check["valid"]
    assert any(v["code"]=="MOTION_MODEL" for v in check["violations"])


def test_repeated_waypoint_windows_use_ordered_first_encounters():
    value,frame=request()
    value["task"]["goal_wkt"]=frame.to_wgs84(Point(500,100,80))
    p=frame.to_wgs84(Point(200,100,80));q=frame.to_wgs84(Point(300,100,80))
    value["task"]["waypoints"]=[{"wkt":p,"earliest":at(value,50)},
        {"wkt":q,"latest":at(value,40)},{"wkt":p}]
    assert plan_single_v2(**value)["status"]!="planned"


def test_validator_rejects_missing_ordered_waypoint():
    value,frame=request();route=plan_single_v2(**value)
    value["task"]["waypoints"]=[{"wkt":frame.to_wgs84(Point(200,200,80))}]
    check=validate(value,route)
    assert not check["valid"]
    assert any(v["code"]=="WAYPOINT" for v in check["violations"])


def test_rtl_home_uses_state_datum_and_rtl_transit_is_checked():
    value,frame=request()
    value["scene"]["coordinates"].update(geoid_separation_m=100,geoid_source="test-geoid")
    value["scene"]["rules"]=[{"id":"rtl","rule_type":"RTL","content":{
        "rtlAltitude":100,"rtlOnLowBattery":True,"lowBatteryPercent":20}}]
    value["task"]["state"]={"at":at(value,0),"battery_percent":10,"remaining_flight_time_s":1000,
        "remaining_range_m":5000,"home_wkt":frame.to_wgs84(Point(500,100,80)),"alt_reference":"ELLIPSOID"}
    route=plan_single_v2(**value)
    assert route["status"]=="planned",route
    assert from_wkt(route["goal_wkt"]).z==pytest.approx(80)
    assert from_wkt(route["trajectory_wkt"]).coords[-1][2]==pytest.approx(80)
    assert validate(value,route)["valid"]
    lower=deepcopy(value)
    lower["task"]["goal_wkt"]=frame.to_wgs84(Point(500,100,80))
    lower["scene"]["rules"]=[];lower["task"].pop("state")
    direct=plan_single_v2(**lower)
    check=validate(value,direct)
    assert not check["valid"]
    assert any(v["code"]=="RTL" for v in check["violations"])


def test_pressure_output_vertices_and_keypoints_use_their_timestamp():
    value,frame=request()
    value["scene"]["coordinates"]["height_corrections"]=[
        {"reference":"QNE","offset_m":0,"source":"meteo","version":"one","valid_from":at(value,0),"valid_to":at(value,50)},
        {"reference":"QNE","offset_m":100,"source":"meteo","version":"two","valid_from":at(value,50),"valid_to":at(value,3600)}]
    value["task"]["alt_reference"]="QNE"
    route=plan_single_v2(**value)
    assert route["status"]=="planned",route
    for coordinate,t in zip(from_wkt(route["trajectory_wkt"]).coords,route["vertex_times_s"]):
        assert coordinate[2]==pytest.approx(80 if t<50 else -20)
    for key in route["keypoints"]:
        assert from_wkt(key["wkt"]).z==pytest.approx(80 if key["time_s"]<50 else -20)
    assert validate(value,route)["valid"]


def test_minimum_speed_permits_only_certified_launch_and_landing_phases():
    value,_=request()
    value["scene"]["rules"]=[{"id":"minimum","rule_type":"SPEED","content":{"minSpeed":3}}]
    route=plan_single_v2(**value)
    assert route["status"]=="planned",route
    assert validate(value,route)["valid"]
    damaged=deepcopy(route)
    damaged["motion_segments"][0]["start_speed_mps"]=1
    check=validate(value,damaged)
    assert not check["valid"]
    assert any(v["code"]=="MOTION_MODEL" for v in check["violations"])


def test_stationary_point_does_not_require_a_motion_model():
    value,_=request();value["task"]["goal_wkt"]=value["task"]["start_wkt"]
    route=plan_single_v2(**value)
    assert route["status"]=="planned",route
    check=validate_route_v2(**value,trajectory_wkt=route["trajectory_wkt"],vertex_times_s=route["vertex_times_s"])
    assert check["valid"],check


def test_arc_wkt_must_keep_the_certified_spatial_approximation():
    from uav_planner.v2.engine import _limits, _result, registry, register_scene
    from uav_planner.v2.models import AircraftProfile, Task
    from uav_planner.v2.motion import build_motion
    value,frame=request()
    value["task"]["goal_wkt"]=frame.to_wgs84(Point(900,300,80))
    value["task"]["aircraft"]["min_turn_radius_m"]=10
    model,compiled,_,_=registry.get(register_scene(value["scene"])["scene_id"])
    task=Task.model_validate(value["task"]);profile=AircraftProfile.model_validate(value["task"]["aircraft"])
    motion=build_motion([[100,100,80],[500,100,80],[500,300,80],[900,300,80]],_limits(compiled,task,profile))
    route=_result(compiled,model,task,profile,motion)
    assert validate(value,route)["valid"]
    arc=next(p for p in route["motion_segments"] if p["kind"]=="arc")
    rows=list(from_wkt(route["trajectory_wkt"]).coords)
    kept=[i for i,t in enumerate(route["vertex_times_s"]) if not arc["start_time_s"]<t<arc["end_time_s"]]
    route["trajectory_wkt"]=LineString([rows[i] for i in kept]).wkt
    route["vertex_times_s"]=[route["vertex_times_s"][i] for i in kept]
    check=validate(value,route)
    assert not check["valid"]
    assert any(v["code"]=="MOTION_MODEL" and "approximation" in v["message"] for v in check["violations"])


def test_dynamic_clearance_combines_obstacle_separation_and_aircraft_size():
    value,frame=request();value["task"].pop("aircraft")
    value["scene"]["dynamic_obstacles"]=[{"id":"wide","trajectory_wkt":frame.to_wgs84(
        LineString([(500,122,80),(500.001,122,80)])),"vertex_times_s":[0,300],"alt_reference":"EG",
        "horizontal_separation_m":20}]
    route=plan_single_v2(**value)
    if route["status"]=="planned":
        assert validate(value,route)["valid"]
