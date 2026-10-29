"""Integration regressions for geographic contracts and real native planning."""
from copy import deepcopy

import numpy as np
import pytest
from shapely import from_wkt
from shapely.geometry import Point, Polygon

from server.v2 import example_request
from uav_planner import PlanningError
from uav_planner.v2.coordinates import CoordinateFrame
from uav_planner.v2.engine import plan_batch_v2, plan_single_v2, register_scene, validate_route_v2
from uav_planner.v2.motion import MotionError, build_motion


def test_exact_endpoint_and_departure_and_complete_time_law():
    request=example_request()
    result=plan_single_v2(**request)
    assert result["status"]=="planned",result
    coordinates=list(from_wkt(result["trajectory_wkt"]).coords)
    assert np.allclose(coordinates[0],from_wkt(request["task"]["start_wkt"]).coords[0],atol=1e-10)
    assert np.allclose(coordinates[-1],from_wkt(request["task"]["goal_wkt"]).coords[0],atol=1e-10)
    assert result["vertex_times_s"][0]==0
    assert result["motion_segments"][0]["profile"]["accel_time_s"]>0
    assert result["validation"]["valid"]


def test_default_strict_is_not_silently_free():
    request=example_request();request["task"].pop("route_mode")
    request["scene"]["airways"]=[]
    with pytest.raises(PlanningError,match="strict"):
        plan_single_v2(**request)


def test_scene_reference_and_inline_are_equivalent_without_answer_cache():
    request=example_request()
    scene_id=register_scene(request["scene"])["scene_id"]
    first=plan_single_v2(scene_id=scene_id,task=request["task"])
    second=plan_single_v2(scene_id=scene_id,task=request["task"])
    assert first==second
    assert first["trajectory_wkt"]==plan_single_v2(**request)["trajectory_wkt"]


def test_midsegment_obstacle_blocks_straight_line_and_is_not_ignored():
    request=example_request()
    frame=CoordinateFrame(request["scene"]["coordinates"])
    request["scene"]["obstacles"]=[{"id":"wall","geometry_wkt":frame.to_wgs84(Polygon([(600,0),(700,0),(700,300),(600,300)])),
        "min_altitude":0,"max_altitude":300,"alt_reference":"EG"}]
    result=plan_single_v2(**request)
    assert result["status"]=="failed",result


def test_explicit_free_works_without_airways():
    request=example_request();request["scene"]["airways"]=[];request["task"]["route_mode"]="free"
    assert plan_single_v2(**request)["status"]=="planned"


def test_shared_corridor_is_rescheduled_and_independently_verified():
    request=example_request()
    tasks=[dict(request["task"],id=f"shared-{i}") for i in range(10)]
    result=plan_batch_v2(scene=request["scene"],tasks=tasks)
    assert result["success_count"]==10,result
    assert result["conflicts"]==[]
    departures=sorted(route["vertex_times_s"][0] for route in result["routes"])
    assert len(set(departures))==10


def test_supplied_validation_rejects_shifted_endpoint():
    request=example_request();result=plan_single_v2(**request)
    route=from_wkt(result["trajectory_wkt"])
    points=list(route.coords);points[-1]=(points[-1][0]+.00001,*points[-1][1:])
    from shapely.geometry import LineString
    check=validate_route_v2(scene=request["scene"],task=request["task"],trajectory_wkt=LineString(points).wkt,
        vertex_times_s=result["vertex_times_s"])
    assert not check["valid"]
    assert any(v["code"]=="ENDPOINT" for v in check["violations"])


def test_analytic_acceleration_and_radius_geometry():
    limits={"max_speed_mps":12,"cruise_speed_mps":6,"climb_speed_mps":3,"descent_speed_mps":3,
        "acceleration_mps2":2,"deceleration_mps2":2,"max_yaw_rate_deg_s":60,"min_turn_radius_m":10}
    motion=build_motion([[0,0,50],[100,0,50],[100,100,50]],limits)
    arc=next(p for p in motion.primitives if p.kind=="arc")
    assert arc.radius==10
    for distance in np.linspace(0,arc.length,20):
        assert np.linalg.norm((arc.position(distance)-arc.center)[:2])==pytest.approx(10)
    assert motion.points[0].tolist()==[0,0,50]
    assert motion.points[-1].tolist()==[100,100,50]
    assert np.all(np.diff(motion.times)>0)
    points,times=motion.collision_samples()
    assert len(points)>len(motion.points)
    assert np.all(np.diff(times)>0)


def test_positive_turn_radius_cannot_be_replaced_by_hover():
    limits={"max_speed_mps":12,"cruise_speed_mps":6,"climb_speed_mps":3,"descent_speed_mps":3,
        "acceleration_mps2":2,"deceleration_mps2":2,"max_yaw_rate_deg_s":60,"min_turn_radius_m":20}
    with pytest.raises(MotionError,match="radius"):
        build_motion([[0,0,50],[5,0,50],[5,5,50]],limits)


def test_altitude_conversion_missing_evidence_is_request_error():
    request=example_request();request["task"]["alt_reference"]="QNE"
    with pytest.raises(PlanningError,match="correction"):
        plan_single_v2(**request)


def test_centerline_endpoint_outside_supplied_grid_is_rejected():
    request=example_request()
    request["scene"]["airways"][0]["endGrid"]["maxAlt"]=50
    with pytest.raises(PlanningError,match="endGrid"):
        plan_single_v2(**request)


def test_mutually_conflicting_external_tracks_are_both_reserved():
    request=example_request();frame=CoordinateFrame(request["scene"]["coordinates"])
    from shapely.geometry import LineString
    request["scene"]["dynamic_obstacles"]=[
        {"id":"first","trajectory_wkt":frame.to_wgs84(LineString([(650,250,80),(650,100,80)])),"vertex_times_s":[0,200],"alt_reference":"EG"},
        {"id":"second","trajectory_wkt":frame.to_wgs84(LineString([(650,250,80),(650,100,80),(650,100,80)])),"vertex_times_s":[0,1,300],"alt_reference":"EG"}]
    result=plan_single_v2(**request)
    if result["status"]=="planned":
        assert result["vertex_times_s"][0]>200
        check=validate_route_v2(**request,trajectory_wkt=result["trajectory_wkt"],vertex_times_s=result["vertex_times_s"],
                                motion_segments=result["motion_segments"])
        assert check["valid"],check


def test_raw_validator_checks_time_window_and_upstream_obstacles():
    request=example_request();result=plan_single_v2(**request)
    request["task"]["departure_time"]="2026-10-16T09:01:00+08:00"
    request["task"]["latest_arrival"]="2026-10-16T09:01:01+08:00"
    check=validate_route_v2(**request,trajectory_wkt=result["trajectory_wkt"],vertex_times_s=result["vertex_times_s"])
    assert not check["valid"]
    assert any(v["code"]=="TIME" for v in check["violations"])


def test_strict_route_can_adjust_reference_centerline_altitude():
    from shapely.geometry import box
    request=example_request();frame=CoordinateFrame(request["scene"]["coordinates"])
    request["scene"]["obstacles"]=[{"id":"full-width-wall","geometry_wkt":frame.to_wgs84(box(400,0,800,200)),
        "min_altitude":50,"max_altitude":100,"alt_reference":"EG"}]
    result=plan_single_v2(**request)
    assert result["status"]=="planned",result
    rows=frame.to_local(result["trajectory_wkt"],"EG").coords
    assert max(p[2] for p in rows)>105.3
    check=validate_route_v2(**request,trajectory_wkt=result["trajectory_wkt"],vertex_times_s=result["vertex_times_s"],
                            motion_segments=result["motion_segments"])
    assert check["valid"],check


def test_airway_climb_limit_constrains_unsampled_motion_peak():
    request=example_request();frame=CoordinateFrame(request["scene"]["coordinates"])
    request["task"]["goal_wkt"]=frame.to_wgs84(Point(100,100,82))
    request["scene"]["airways"][0]["climbConstraint"]={"maxClimbRate":1.5,"maxDescentRate":1.5}
    result=plan_single_v2(**request)
    assert result["status"]=="planned",result
    assert max(p["profile"]["peak_speed_mps"] for p in result["motion_segments"])<=1.5
    check=validate_route_v2(**request,trajectory_wkt=result["trajectory_wkt"],vertex_times_s=result["vertex_times_s"],
                            motion_segments=result["motion_segments"])
    assert check["valid"],check


@pytest.mark.parametrize("content",[
    {"earliestDeparture":"09:01"},
    {"dailyStart":"09:01","dailyEnd":"09:10"},
])
def test_time_rules_delay_launch_into_permitted_windows(content):
    request=example_request()
    request["scene"]["rules"]=[{"id":"opening","rule_type":"TIME","content":content}]
    request["task"]["latest_arrival"]="2026-10-16T09:10:00+08:00"
    result=plan_single_v2(**request)
    assert result["status"]=="planned",result
    assert result["vertex_times_s"][0]>=60
    check=validate_route_v2(**request,trajectory_wkt=result["trajectory_wkt"],vertex_times_s=result["vertex_times_s"],
                            motion_segments=result["motion_segments"])
    assert check["valid"],check


def test_collision_retry_uses_a_later_disjoint_time_window():
    request=example_request()
    request["scene"]["rules"]=[{"id":"temporary-time","rule_type":"TIME","content":{"earliestDeparture":"09:10"},
        "effective_start":"2026-10-16T09:03:07+08:00","effective_end":"2026-10-16T09:04:10+08:00"}]
    tasks=[dict(request["task"],id=f"time-{i}") for i in range(2)]
    result=plan_batch_v2(scene=request["scene"],tasks=tasks)
    assert result["success_count"]==2,result
    assert not result["conflicts"]
    assert max(route["vertex_times_s"][0] for route in result["routes"])>250
