"""Acceptance fixtures are immutable, independently feasible, and contentious."""
from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest

from uav_planner.v2 import native
from uav_planner.v2.models import RULE_TYPES, Scene
from uav_planner.v2.scene import CompiledScene

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("benchmark_v2", ROOT / "tools" / "benchmark_v2.py")
assert SPEC and SPEC.loader
benchmark = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(benchmark)


@pytest.fixture(scope="module", params=benchmark.SUITES)
def workload(request):
    return benchmark.load_frozen(request.param)[0]


def test_frozen_generation_is_reproducible_and_schema_valid(workload):
    data = benchmark.canonical_bytes(benchmark.generate_suite(workload["suite"]))
    assert data == (benchmark.FROZEN_DIRECTORY / f"{workload['suite']}.json").read_bytes()
    model = Scene.model_validate(workload["scene"])
    assert len(workload["tasks"]) == len(workload["reference_trajectories"]) == 1000
    assert len({task["id"] for task in workload["tasks"]}) == 1000
    assert model.bounds.x_max - model.bounds.x_min == 5000
    assert model.bounds.y_max - model.bounds.y_min == 5000
    assert (model.bounds.z_min, model.bounds.z_max) == (0, 300)
    assert model.resolution == (25, 25, 10)
    assert model.horizon_s == 3600
    assert len(model.profiles) == 4
    assert len(model.obstacles) <= 1000 and len(model.control_zones) <= 20
    assert len(model.airways) <= 200
    assert all(len(task["waypoints"]) <= 2 for task in workload["tasks"])
    assert workload["construction"]["reference_uses_planner"] is False
    assert workload["construction"]["ground_wait_occupies_airspace"] is False


def test_frozen_hash_tampering_is_rejected(tmp_path):
    manifest = json.loads((benchmark.FROZEN_DIRECTORY / "manifest.json").read_bytes())
    (tmp_path / "manifest.json").write_bytes(benchmark.canonical_bytes(manifest))
    for filename in manifest["files"]:
        (tmp_path / filename).write_bytes((benchmark.FROZEN_DIRECTORY / filename).read_bytes())
    (tmp_path / "strict.json").write_bytes((tmp_path / "strict.json").read_bytes() + b" ")
    with pytest.raises(ValueError, match="hash mismatch"):
        benchmark.load_frozen("strict", tmp_path)


def test_freeze_refuses_changed_existing_files(tmp_path):
    (tmp_path / "strict.json").write_text("changed", encoding="utf-8")
    with pytest.raises(ValueError, match="refusing overwrite"):
        benchmark.freeze(tmp_path)
    assert (tmp_path / "strict.json").read_text(encoding="utf-8") == "changed"
    assert not (tmp_path / "mixed.json").exists()


@pytest.mark.skipif(not native.available(), reason="required C++ extension is not built")
def test_all_reference_pair_schedules_are_feasible_and_originals_conflict(workload):
    references = workload["reference_trajectories"]
    points = [np.asarray(reference["local_points"]) for reference in references]
    times = [np.asarray(reference["vertex_times_s"]) for reference in references]
    assert all(np.all(np.diff(time) > 0) and time[-1] < 3500 for time in times)
    check = native.continuous_conflicts(points, times, 12, 12, max_conflicts=100000)
    assert check["conflicts"] == []
    initial_times = [time - time[0] for time in times]
    initial = native.continuous_conflicts(points, initial_times, 12, 12, max_conflicts=100000)
    conflicted = {conflict[key] for conflict in initial["conflicts"] for key in ("first", "second")}
    assert len(conflicted) == 1000


@pytest.mark.skipif(not native.available(), reason="required C++ extension is not built")
def test_mixed_all_twenty_rules_have_executed_effects():
    workload, _ = benchmark.load_frozen("mixed")
    compiled = CompiledScene(workload["scene"])
    effects = benchmark.validate_rule_witnesses(workload, compiled)
    assert set(effects) == set(RULE_TYPES)
    assert effects["PRIORITY"]["quiet_cost"] < effects["PRIORITY"]["noisy_cost"]
    assert effects["RTL"]["required"]
    triggered = [task for task in workload["tasks"] if task["state"]["battery_percent"] == 10]
    assert len(triggered) == 10
    assert all(task["goal_wkt"] != task["state"]["home_wkt"] for task in triggered)


def test_analytic_reference_obeys_aircraft_acceleration_and_speed(workload):
    profiles = {profile["id"]: profile for profile in workload["scene"]["profiles"]}
    for task, reference in zip(workload["tasks"], workload["reference_trajectories"]):
        profile = profiles[task["profile_id"]]
        for primitive in reference["motion_primitives"]:
            duration = primitive["end_time_s"] - primitive["start_time_s"]
            assert duration > 0
            if primitive["kind"] == "yaw":
                continue
            peak = primitive["peak_speed_mps"]
            ta, tc, td = (primitive[name] for name in ("accel_time_s", "cruise_time_s", "decel_time_s"))
            assert peak <= profile["max_speed_mps"]
            assert peak / ta <= profile["acceleration_mps2"] + 1e-9
            assert peak / td <= profile["deceleration_mps2"] + 1e-9
            assert ta + tc + td == pytest.approx(duration)
            assert peak * (ta / 2 + tc + td / 2) == pytest.approx(primitive["length_m"])
            delta = np.asarray(primitive["end"]) - np.asarray(primitive["start"])
            vertical_speed = abs(delta[2]) / primitive["length_m"] * peak
            assert vertical_speed <= profile["climb_speed_mps"]
            if workload["suite"] == "mixed":
                assert peak * np.linalg.norm(delta[:2]) / primitive["length_m"] <= 4 + 1e-9
                assert vertical_speed <= 1 + 1e-9
