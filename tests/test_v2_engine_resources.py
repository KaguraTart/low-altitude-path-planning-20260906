"""Caller-controlled planning deadlines and complete safety certificates."""
import math
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from server.v2 import example_request
from uav_planner import PlanningError
from uav_planner.v2 import engine, native
from uav_planner.v2.models import AircraftProfile, Scene, Task
from uav_planner.v2.scene import CompiledScene


@pytest.mark.parametrize("budget", ["omitted", None, 600])
def test_planning_can_continue_beyond_old_default_deadline(monkeypatch, budget):
    request = example_request()
    clock = [100.0]
    original = engine._path_for_leg

    def slow_leg(*args, **kwargs):
        clock[0] += 20
        return original(*args, **kwargs)

    monkeypatch.setattr(engine.time, "perf_counter", lambda: clock[0])
    monkeypatch.setattr(engine, "_path_for_leg", slow_leg)
    kwargs = {} if budget == "omitted" else {"time_budget_s": budget}
    assert engine.plan_single_v2(**request, **kwargs)["status"] == "planned"


def test_explicit_expired_deadline_is_a_timeout_without_native_call(monkeypatch):
    request = example_request()
    clock = [100.0]
    original = engine._path_for_leg

    def slow_leg(*args, **kwargs):
        clock[0] += 20
        return original(*args, **kwargs)

    def unexpected_schedule(*args, **kwargs):
        pytest.fail("Expired deadline reached native scheduling")

    monkeypatch.setattr(engine.time, "perf_counter", lambda: clock[0])
    monkeypatch.setattr(engine, "_path_for_leg", slow_leg)
    monkeypatch.setattr(native, "schedule_paths", unexpected_schedule)
    result = engine.plan_single_v2(**request, time_budget_s=5)
    assert result["status"] == "timeout"
    assert result["code"] == "TIME_BUDGET_EXCEEDED"


@pytest.mark.parametrize("budget", [0, -1, True, "10", math.inf, math.nan, 10**1000])
def test_sdk_budget_must_be_positive_finite_or_none(budget):
    with pytest.raises(PlanningError, match="positive finite"):
        engine.plan_single_v2(**example_request(), time_budget_s=budget)


def test_native_deadline_helper_distinguishes_unlimited_expired_and_large(monkeypatch):
    monkeypatch.setattr(engine.time, "perf_counter", lambda: 100)
    assert engine._native_deadline_ms(math.inf) == 0
    assert engine._native_deadline_ms(101) == 1000
    assert engine._native_deadline_ms(1e12) > 1e12
    assert engine._native_deadline_ms(sys.float_info.max) == sys.float_info.max
    with pytest.raises(TimeoutError):
        engine._native_deadline_ms(100)


def test_batch_above_old_count_limit_still_checks_unique_ids():
    value = example_request()
    with pytest.raises(PlanningError, match="unique ids"):
        engine.plan_batch_v2(scene=value["scene"], tasks=[value["task"]] * 10001)


def test_grid_above_old_cell_limit_reaches_native_without_iteration_cap(monkeypatch):
    value = example_request()
    value["scene"]["bounds"]["x_max"] = 100000
    value["scene"]["bounds"]["y_max"] = 100000
    model = Scene.model_validate(value["scene"])
    compiled = SimpleNamespace(model=model, blocked_indices=lambda *args: [])
    calls = []

    def search(*args, **kwargs):
        calls.append(kwargs)
        return {"status": "ok", "path": [args[0], args[1]]}

    monkeypatch.setattr(native, "grid_search", search)
    path = engine._grid_path(compiled, np.array([0, 0, 80]), np.array([100, 0, 80]),
                             Task.model_validate(value["task"]), AircraftProfile(), 0, math.inf)
    assert len(path) == 2
    assert calls == [{"deadline_ms": 0.0}]


def test_strict_altitude_fallback_searches_past_old_sixty_four_limit(monkeypatch):
    value = example_request()
    value["scene"]["resolution"][2] = 1
    compiled = CompiledScene(value["scene"])
    task = Task.model_validate(value["task"])
    start = np.array([100, 100, 80.0])
    goal = np.array([1200, 100, 80.0])
    attempts = []

    def roadmap(*args):
        altitude = args[-1] if len(args) == 8 else None
        if altitude is not None:
            attempts.append(altitude)
        return np.array([start, goal]) if len(attempts) == 65 else None

    monkeypatch.setattr(engine, "_geometry_ok", lambda *args: False)
    monkeypatch.setattr(engine, "_roadmap", roadmap)
    assert engine._path_for_leg(compiled, start, goal, task, AircraftProfile(), 0, math.inf) is not None
    assert len(attempts) == 65


def test_cost_floor_cache_retains_more_than_old_one_hundred_twenty_eight_entries():
    rules = SimpleNamespace(
        selected=lambda task: [SimpleNamespace(id=task.id, rule_type="PRIORITY",
            content={"priorityScene": "FAST", "costBonus": .1})],
        active=lambda rule, at: True,
    )
    compiled = SimpleNamespace(rules=rules)
    for i in range(129):
        assert engine._cost_floor(compiled, SimpleNamespace(id=f"rule-{i}"), 0) == .9
    assert len(compiled._cost_floor_cache) == 129
    assert ("rule-0",) in compiled._cost_floor_cache


@pytest.mark.parametrize("status", ["conflict_limit", "deadline_exceeded"])
def test_incomplete_final_conflict_proof_never_returns_success(monkeypatch, status):
    monkeypatch.setattr(native, "continuous_conflicts", lambda *args, **kwargs: {
        "status": status, "conflicts": [], "checked_pairs": 0,
    })
    result = engine.plan_single_v2(**example_request())
    assert result["status"] == ("timeout" if status == "deadline_exceeded" else "failed")
    code = "TIME_BUDGET_EXCEEDED" if status == "deadline_exceeded" else "VALIDATION_INCOMPLETE"
    assert result["code"] == code


def test_validator_rejects_incomplete_upstream_conflict_proof(monkeypatch):
    value = example_request()
    route = engine.plan_single_v2(**value)
    value["scene"]["dynamic_obstacles"] = [{
        "id": "upstream", "trajectory_wkt": value["task"]["start_wkt"],
        "vertex_times_s": [1000], "alt_reference": "MSL",
    }]
    monkeypatch.setattr(native, "continuous_conflicts", lambda *args, **kwargs: {
        "status": "conflict_limit", "conflicts": [], "checked_pairs": 0,
    })
    check = engine.validate_route_v2(**value, trajectory_wkt=route["trajectory_wkt"],
        vertex_times_s=route["vertex_times_s"], motion_segments=route["motion_segments"])
    assert not check["valid"]
    assert any(item["code"] == "VALIDATION_INCOMPLETE" for item in check["violations"])


def test_validation_input_above_old_sample_and_primitive_limits_reaches_geometry(monkeypatch):
    value = example_request()

    class ReachedGeometry(Exception):
        pass

    def geometry(*args):
        raise ReachedGeometry

    monkeypatch.setattr(engine, "_timed_local", geometry)
    with pytest.raises(ReachedGeometry):
        engine.validate_route_v2(**value, trajectory_wkt=value["task"]["start_wkt"],
            vertex_times_s=np.arange(100001), motion_segments=[None] * 100001)


@pytest.mark.parametrize("target", ["uav_planner.v2.scene.CompiledScene", "uav_planner.v2.engine.build_motion"])
def test_engine_allocation_failure_is_not_a_planning_or_input_failure(monkeypatch, target):
    value = example_request()
    value["scene"]["horizon_s"] = 123456

    def out_of_memory(*args, **kwargs):
        raise MemoryError("test allocation")

    monkeypatch.setattr(target, out_of_memory)
    with pytest.raises(MemoryError, match="test allocation"):
        engine.plan_single_v2(**value)
