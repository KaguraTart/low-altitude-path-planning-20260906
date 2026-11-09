"""ENG-10: hard-constraint itemisation, honest metrics, and plan ranking."""
from __future__ import annotations

import copy

import pytest
from shapely import from_wkt

from server.v2 import example_request
from uav_planner.errors import ErrorCode, PlanningError
from uav_planner.v2.engine import plan_single_v2
from uav_planner.v2.evaluation import compare_plans_v2, evaluate_v2


def planned(request):
    route = plan_single_v2(**request)
    assert route["status"] == "planned", route.get("message")
    return route


def payload(request, route, **extra):
    value = dict(task=request["task"], scene=request["scene"],
                 trajectory_wkt=route["trajectory_wkt"],
                 vertex_times_s=route["vertex_times_s"],
                 motion_segments=route["motion_segments"])
    value.update(extra)
    return value


def with_rules(request, rules):
    scene = copy.deepcopy(request["scene"])
    scene["rules"] = rules
    return scene


def first_vertex(route):
    return from_wkt(route["trajectory_wkt"]).coords[0]


DISTANCE_ONLY = {"weights": {"distance": 1, "time": 0, "energy": 0, "risk": 0}}


def test_valid_route_reports_safety_and_available_metrics():
    request = example_request()
    route = planned(request)
    result = evaluate_v2(**payload(request, route))
    assert result["valid"] is True
    assert result["rankable"] is True
    assert result["engine_id"] == "ENG-10"
    assert result["metrics"]["distance_m"]["available"] is True
    assert result["metrics"]["distance_m"]["value"] == pytest.approx(route["metrics"]["distance_m"])
    assert result["metrics"]["flight_time_s"]["available"] is True
    assert result["metrics"]["wait_time_s"]["available"] is True
    assert result["safety"]["violations"] == []
    assert {check["status"] for check in result["safety"]["checks"]} <= {
        "applied", "not_applicable", "inactive", "unsupported"}


def test_missing_energy_and_risk_are_unavailable_not_zero():
    request = example_request()
    route = planned(request)
    result = evaluate_v2(**payload(request, route))
    energy = result["metrics"]["energy_wh"]
    risk = result["metrics"]["risk"]
    assert energy["available"] is False and energy["value"] is None
    assert "energy_model" in energy["reason"]
    assert risk["available"] is False and risk["value"] is None
    assert "risk" in risk["reason"]


def test_energy_metric_uses_the_supplied_model():
    request = example_request()
    route = planned(request)
    objectives = {"weights": {"distance": 0, "time": 0, "energy": 1, "risk": 0},
                  "energy_model": {"mass_kg": 1.5, "hover_power_w": 350,
                                   "cruise_power_w": 500, "climb_efficiency": 0.7}}
    result = evaluate_v2(**payload(request, route), objectives=objectives)
    assert result["metrics"]["energy_wh"]["available"] is True
    assert result["metrics"]["energy_wh"]["value"] > 0
    assert result["score"]["value"] == pytest.approx(result["metrics"]["energy_wh"]["value"])


def test_positively_weighted_unavailable_metric_blocks_ranking():
    request = example_request()
    route = planned(request)
    result = evaluate_v2(**payload(request, route),
                         objectives={"weights": {"distance": 1, "time": 0, "energy": 0, "risk": 1}})
    assert result["valid"] is True
    assert result["score"]["available"] is False
    assert result["score"]["value"] is None
    assert result["rankable"] is False
    assert result["exclusion_code"] == "MISSING_METRIC"


def test_no_fly_violation_reports_point_threshold_and_reason():
    request = example_request()
    route = planned(request)
    lon, lat, z = first_vertex(route)
    rule = {"id": "rule-NO_FLY", "rule_type": "NO_FLY",
            "content": {"noFlyMode": "STANDARD", "heightCeiling": 300, "includeBuffer": True},
            "scope": {"geom": [{"type": "NO_FLY_ZONE",
                                "geom": (f"POLYGON Z(({lon-.002} {lat-.002} 0,{lon+.002} {lat-.002} 0,"
                                         f"{lon+.002} {lat+.002} 300,{lon-.002} {lat+.002} 300,"
                                         f"{lon-.002} {lat-.002} 0))")}]}}
    result = evaluate_v2(task=request["task"], scene=with_rules(request, [rule]),
                         trajectory_wkt=route["trajectory_wkt"],
                         vertex_times_s=route["vertex_times_s"],
                         motion_segments=route["motion_segments"])
    assert result["valid"] is False
    failed = [check for check in result["safety"]["checks"]
              if check["status"] == "failed" and check["code"] == "NO_FLY"]
    assert failed, result["safety"]["checks"]
    check = failed[0]
    assert check["rule_id"] == "rule-NO_FLY"
    assert check["point_wkt"] and check["point_wkt"].startswith("POINT Z")
    assert check["time_s"] is not None
    assert check["threshold"]["heightCeiling"] == 300
    assert check["reason"]
    assert result["exclusion_code"] == "HARD_CONSTRAINT_FAILED"


def test_height_layer_violation_is_itemised():
    request = example_request()
    route = planned(request)
    rule = {"id": "rule-HEIGHT_LAYER", "rule_type": "HEIGHT_LAYER",
            "content": {"minAltitude": 0, "maxAltitude": 1, "altDatum": "EG"}}
    result = evaluate_v2(task=request["task"], scene=with_rules(request, [rule]),
                         trajectory_wkt=route["trajectory_wkt"],
                         vertex_times_s=route["vertex_times_s"],
                         motion_segments=route["motion_segments"])
    assert result["valid"] is False
    check = next(item for item in result["safety"]["checks"] if item["code"] == "HEIGHT_LAYER")
    assert check["status"] == "failed"
    assert check["threshold"]["maxAltitude"] == 1
    assert check["actual"]["altitude_m"] > 1


def test_endurance_violation_is_itemised():
    request = example_request()
    route = planned(request)
    rule = {"id": "rule-ENDURANCE", "rule_type": "ENDURANCE",
            "content": {"maxRange": 1, "minEndurance": 1}}
    result = evaluate_v2(task=request["task"], scene=with_rules(request, [rule]),
                         trajectory_wkt=route["trajectory_wkt"],
                         vertex_times_s=route["vertex_times_s"],
                         motion_segments=route["motion_segments"])
    assert result["valid"] is False
    check = next(item for item in result["safety"]["checks"] if item["code"] == "ENDURANCE")
    assert check["status"] == "failed"
    assert check["threshold"]["maxRange"] == 1


def test_compare_excludes_hard_failed_plan():
    request = example_request()
    route = planned(request)
    lon, lat, _ = first_vertex(route)
    blocked = with_rules(request, [{
        "id": "rule-NO_FLY", "rule_type": "NO_FLY",
        "content": {"noFlyMode": "STANDARD", "heightCeiling": 300, "includeBuffer": True},
        "scope": {"geom": [{"type": "NO_FLY_ZONE",
                            "geom": (f"POLYGON Z(({lon-.002} {lat-.002} 0,{lon+.002} {lat-.002} 0,"
                                     f"{lon+.002} {lat+.002} 300,{lon-.002} {lat+.002} 300,"
                                     f"{lon-.002} {lat-.002} 0))")}]}}])
    plans = [
        {"plan_id": "plan-safe", "task": request["task"], "trajectory_wkt": route["trajectory_wkt"],
         "vertex_times_s": route["vertex_times_s"], "motion_segments": route["motion_segments"]},
        {"plan_id": "plan-blocked", "scene": blocked, "task": request["task"],
         "trajectory_wkt": route["trajectory_wkt"],
         "vertex_times_s": route["vertex_times_s"], "motion_segments": route["motion_segments"]},
    ]
    result = compare_plans_v2(request_id="compare-1", plans=plans, objectives=DISTANCE_ONLY,
                              scene=request["scene"])
    assert result["ranking"] == ["plan-safe"]
    assert result["winner"] == "plan-safe"
    assert result["scene_mode"] == "per_plan"
    excluded = next(item for item in result["plans"] if item["plan_id"] == "plan-blocked")
    assert excluded["rankable"] is False
    assert excluded["exclusion_code"] == "HARD_CONSTRAINT_FAILED"
    assert excluded["exclusion_reason"]


def test_compare_tie_breaks_on_plan_id():
    request = example_request()
    route = planned(request)
    plans = [{"plan_id": plan_id, "task": request["task"], "trajectory_wkt": route["trajectory_wkt"],
              "vertex_times_s": route["vertex_times_s"], "motion_segments": route["motion_segments"]}
             for plan_id in ("plan-b", "plan-a")]
    result = compare_plans_v2(request_id="compare-tie", plans=plans, objectives=DISTANCE_ONLY,
                              scene=request["scene"])
    assert result["scene_mode"] == "shared"
    assert result["ranking"] == ["plan-a", "plan-b"]
    assert result["winner"] == "plan-a"


def test_compare_requires_two_plans():
    request = example_request()
    route = planned(request)
    plans = [{"plan_id": "only", "task": request["task"], "trajectory_wkt": route["trajectory_wkt"],
              "vertex_times_s": route["vertex_times_s"], "motion_segments": route["motion_segments"]}]
    with pytest.raises(PlanningError) as caught:
        compare_plans_v2(request_id="one", plans=plans, objectives=DISTANCE_ONLY, scene=request["scene"])
    assert caught.value.code == ErrorCode.INVALID_INPUT


def test_compare_rejects_duplicate_plan_ids():
    request = example_request()
    route = planned(request)
    plans = [{"plan_id": "same", "task": request["task"], "trajectory_wkt": route["trajectory_wkt"],
              "vertex_times_s": route["vertex_times_s"], "motion_segments": route["motion_segments"]}
             for _ in range(2)]
    with pytest.raises(PlanningError) as caught:
        compare_plans_v2(request_id="dup", plans=plans, objectives=DISTANCE_ONLY, scene=request["scene"])
    assert caught.value.code == ErrorCode.INVALID_INPUT


def test_evaluate_requires_exactly_one_scene_source():
    request = example_request()
    route = planned(request)
    with pytest.raises(PlanningError) as caught:
        evaluate_v2(task=request["task"], trajectory_wkt=route["trajectory_wkt"],
                    vertex_times_s=route["vertex_times_s"],
                    motion_segments=route["motion_segments"])
    assert caught.value.code == ErrorCode.INVALID_INPUT
