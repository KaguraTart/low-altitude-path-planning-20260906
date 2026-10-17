"""Public validation cannot hide dynamics behind a sparse WKT representation."""
import math

from shapely.geometry import LineString, Point

from server.v2 import example_request
from uav_planner.v2.coordinates import CoordinateFrame
from uav_planner.v2.engine import validate_route_v2


def validate_primitive(primitive, rules=()):
    value = example_request()
    value["scene"]["airways"] = []
    value["scene"]["rules"] = list(rules)
    value["task"]["route_mode"] = "free"
    frame = CoordinateFrame(value["scene"]["coordinates"])
    value["task"]["start_wkt"] = frame.to_wgs84(Point(primitive["start"]))
    value["task"]["goal_wkt"] = frame.to_wgs84(Point(primitive["end"]))
    return validate_route_v2(
        **value,
        trajectory_wkt=frame.to_wgs84(LineString([primitive["start"], primitive["end"]])),
        vertex_times_s=[primitive["start_time_s"], primitive["end_time_s"]],
        motion_segments=[primitive],
    )


def proof_codes(result):
    return {issue["code"] for violation in result["violations"]
            for issue in violation.get("details", {}).get("violations", [])}


def test_sparse_wkt_does_not_hide_excessive_analytic_turn():
    duration = math.pi * .05 / 2 / .02
    primitive = {
        "kind": "arc", "start": [100.05, 100, 80], "end": [100, 100.05, 80],
        "center": [100, 100, 80], "radius_m": .05, "sweep_angle_deg": 90,
        "length_m": math.pi * .05 / 2, "start_time_s": 0, "end_time_s": duration,
        "start_speed_mps": .02, "end_speed_mps": .02, "peak_speed_mps": .02,
        "acceleration_mps2": 0, "deceleration_mps2": 0,
        "accel_time_s": 0, "cruise_time_s": duration, "decel_time_s": 0,
    }
    result = validate_primitive(primitive, [{"id": "turn", "rule_type": "TURN",
                                           "content": {"maxTurnAngle": 60, "minTurnRadius": 0}}])
    assert not result["valid"]
    assert "TURN_ANGLE" in proof_codes(result)


def test_sparse_wkt_does_not_hide_short_excessive_acceleration():
    primitive = {
        "kind": "line", "start": [100, 100, 80], "end": [101, 100, 80],
        "length_m": 1, "start_time_s": 0, "end_time_s": 1.000000001,
        "start_speed_mps": 0, "end_speed_mps": 0, "peak_speed_mps": 1,
        "acceleration_mps2": 1e9, "deceleration_mps2": 1e9,
        "accel_time_s": 1e-9, "cruise_time_s": .999999999, "decel_time_s": 1e-9,
    }
    result = validate_primitive(primitive)
    assert not result["valid"]
    assert {"ACCELERATION", "DECELERATION"} <= proof_codes(result)
