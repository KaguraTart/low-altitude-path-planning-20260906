"""Airway handoffs are annotations on the actual analytic trajectory."""
from datetime import datetime, timedelta

import numpy as np
import pytest
from shapely import from_wkt
from shapely.geometry import LineString, Point

from server.v2 import example_request
from uav_planner.v2.coordinates import CoordinateFrame
from uav_planner.v2.engine import _limits, _result, plan_single_v2
from uav_planner.v2.models import AircraftProfile, Task
from uav_planner.v2.motion import build_motion
from uav_planner.v2.scene import CompiledScene


def setup():
    value=example_request();frame=CoordinateFrame(value["scene"]["coordinates"])
    def airway(id,x0,x1):
        grid={"minLon":104.76,"maxLon":104.80,"minLat":29.32,"maxLat":29.34,"minAlt":0,"maxAlt":300}
        return {"id":id,"centerLine":frame.to_wgs84(LineString([(x0,100,80),(x1,100,80)])),
            "widthMeters":100,"minAltitude":50,"maxAltitude":150,"altReference":"EG",
            "startGrid":grid,"endGrid":grid}
    value["scene"]["airways"]=[airway("a",100,650),airway("b",600,1200)]
    value["task"]["aircraft"]={"radius_m":.01,"height_m":.01,"safety_margin_m":0}
    return value,frame


def test_overlapping_collinear_airways_switch_only_when_previous_coverage_ends():
    value,frame=setup();result=plan_single_v2(**value)
    assert result["status"]=="planned",result
    events=[k for k in result["keypoints"] if "AIRWAY_SWITCH" in k["types"]]
    assert len(events)==1
    point=np.asarray(frame.to_local(events[0]["wkt"],"EG").coords[0])
    compiled=CompiledScene(value["scene"]);a=compiled.resources["a"]
    usable=compiled._buffered(a,-.01)
    assert point[0]==pytest.approx(usable.bounds[2],abs=1e-5)
    primitive=result["motion_segments"][0]
    assert set(primitive["airway_ids"])=={"a","b"}
    assert events[0]["time_s"] not in result["vertex_times_s"]
    assert point[1:]==pytest.approx([100,80],abs=1e-6)


def test_switch_timestamp_uses_acceleration_law_and_grid_codes():
    value,frame=setup()
    # End the first airway's usable footprint during the first launch phase.
    value["scene"]["airways"][0]["centerLine"]=frame.to_wgs84(LineString([(100,100,80),(101,100,80)]))
    value["scene"]["airways"][0]["widthMeters"]=2
    value["scene"]["airways"][1]["centerLine"]=frame.to_wgs84(LineString([(100,100,80),(1200,100,80)]))
    value["scene"]["grid"]={"origin":[50,50,0],"resolution":[10,10,10],"shape":[150,50,30],
        "codes":{"5,5,8":"handoff-cell"}}
    result=plan_single_v2(**value)
    assert result["status"]=="planned",result
    event=next(k for k in result["keypoints"] if "AIRWAY_SWITCH" in k["types"])
    point=np.asarray(frame.to_local(event["wkt"],"EG").coords[0])
    compiled=CompiledScene(value["scene"])
    expected=compiled._buffered(compiled.resources["a"],-.01).bounds[2]
    assert point[0]==pytest.approx(expected,abs=1e-6)
    assert event["time_s"]==pytest.approx(np.sqrt(point[0]-100),abs=1e-5)
    # Internal search indices retain Scene bounds/resolution; the code uses
    # the separately supplied upstream GridSpec origin/resolution.
    assert event["grid_indices"]==[4,4,8]
    assert event["grid_code"]=="handoff-cell"


def test_airway_validity_boundary_triggers_handoff_at_actual_position():
    value,frame=setup();origin=datetime.fromisoformat(value["scene"]["time_origin"])
    value["scene"]["airways"][0]["valid_to"]=(origin+timedelta(seconds=50)).isoformat()
    value["scene"]["airways"][1]["centerLine"]=frame.to_wgs84(LineString([(100,100,80),(1200,100,80)]))
    result=plan_single_v2(**value)
    assert result["status"]=="planned",result
    event=next(k for k in result["keypoints"] if "AIRWAY_SWITCH" in k["types"])
    assert event["time_s"]==pytest.approx(50)
    point=frame.to_local(event["wkt"],"EG").coords[0]
    assert point[0]==pytest.approx(391,abs=1e-5)


def test_coincident_waypoint_and_switch_merge_event_types_and_keep_identity():
    value,frame=setup();value["scene"]["airways"][0]["valid_to"]="2026-10-16T09:00:50+08:00"
    value["scene"]["airways"][1]["centerLine"]=frame.to_wgs84(LineString([(100,100,80),(1200,100,80)]))
    value["task"]["waypoints"]=[{"wkt":frame.to_wgs84(Point(391,100,80))}]
    result=plan_single_v2(**value)
    assert result["status"]=="planned",result
    events=[k for k in result["keypoints"] if abs(k["time_s"]-50)<1e-6]
    assert len(events)==1
    assert events[0]["types"]==["AIRWAY_SWITCH","WAYPOINT"]
    assert events[0]["waypoint_index"]==0


def test_one_airway_route_keeps_existing_keypoints_without_switches():
    value,_=setup();value["scene"]["airways"]=value["scene"]["airways"][1:]
    value["task"]["route_mode"]="free"
    result=plan_single_v2(**value)
    assert result["status"]=="planned",result
    assert not any("AIRWAY_SWITCH" in k["types"] for k in result["keypoints"])


def test_two_business_waypoint_identities_survive_coincident_event_merging():
    value,frame=setup();value["scene"]["airways"][0]["valid_to"]="2026-10-16T09:00:50+08:00"
    value["scene"]["airways"][1]["centerLine"]=frame.to_wgs84(LineString([(100,100,80),(1200,100,80)]))
    value["task"]["waypoints"]=[{"wkt":frame.to_wgs84(Point(391,100,80))}]*2
    result=plan_single_v2(**value)
    assert result["status"]=="planned",result
    events=[k for k in result["keypoints"] if abs(k["time_s"]-50)<1e-6]
    assert len(events)==2
    assert {event["waypoint_index"] for event in events}=={0,1}
    assert sum("AIRWAY_SWITCH" in event["types"] for event in events)==1


def test_unusable_reverse_airway_is_not_selected_as_a_handoff_source():
    value,frame=setup();value["scene"]["airways"][0]["direction"]="REVERSE"
    value["scene"]["airways"][1]["centerLine"]=frame.to_wgs84(LineString([(100,100,80),(1200,100,80)]))
    result=plan_single_v2(**value)
    assert result["status"]=="planned",result
    assert not any("AIRWAY_SWITCH" in k["types"] for k in result["keypoints"])


def test_arc_switch_is_on_actual_circle_and_preserves_exported_motion():
    value,frame=setup();value["task"]["route_mode"]="free"
    value["task"]["goal_wkt"]=frame.to_wgs84(Point(400,400,80))
    value["task"]["aircraft"]["min_turn_radius_m"]=40
    value["scene"]["airways"][0]["centerLine"]=frame.to_wgs84(LineString([(100,100,80),(300,100,80)]))
    value["scene"]["airways"][1]["centerLine"]=frame.to_wgs84(LineString([(300,100,80),(300,400,80)]))
    value["scene"]["airways"][0]["widthMeters"]=20
    value["scene"]["airways"][1]["widthMeters"]=100
    compiled=CompiledScene(value["scene"]);task=Task.model_validate(value["task"])
    profile=AircraftProfile.model_validate(value["task"]["aircraft"])
    motion=build_motion([[100,100,80],[300,100,80],[300,400,80],[400,400,80]],_limits(compiled,task,profile))
    before=motion.times.copy();result=_result(compiled,compiled.model,task,profile,motion)
    events=[k for k in result["keypoints"] if "AIRWAY_SWITCH" in k["types"]]
    assert events
    event=events[0];point=np.asarray(frame.to_local(event["wkt"],"EG").coords[0])
    arc=next(p for p in motion.primitives if p.kind=="arc")
    assert np.linalg.norm(point[:2]-arc.center[:2])==pytest.approx(arc.radius,abs=1e-6)
    assert point[1]==pytest.approx(109.99,abs=1e-5)
    assert motion.times==pytest.approx(before)
    assert result["vertex_times_s"]==pytest.approx(before)
