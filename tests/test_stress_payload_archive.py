"""Guard complete archives against mutation, omitted routes, and timing drift."""
from __future__ import annotations

import hashlib
import json
import random
from dataclasses import asdict

import pytest

from src.models.obstacles import DynamicObstacle, ObstacleSet, StaticObstacle
from src.models.space import SpaceTimeConfig
from src.models.task import PlannedRoute, PlanningExplanation, PlanningResult, TrajectoryPoint
from tools import run_stress_test as stress
from tools.planning_archive import snapshot_input_files


@pytest.mark.parametrize("dispatch", [False, True])
def test_scale_archive_retains_original_inputs_and_every_result_field(tmp_path, monkeypatch, dispatch):
    monkeypatch.setenv("GITHUB_SHA", "a" * 40)
    config = SpaceTimeConfig(x_max=1000, y_max=1000, z_max=500)
    obstacles = ObstacleSet(
        static_obstacles=[StaticObstacle("tower", "Tower", "tower", 500, 500, 0)],
        dynamic_obstacles=[DynamicObstacle("moving", "Moving", "uav", [(100, 100, 100, 0), (200, 100, 100, 10)])],
    )
    seed = 17
    original_tasks, original_performances = stress.generate_random_tasks(2, config, obstacles, random.Random(seed + 2))
    result = PlanningResult(
        request_id="stress-2", success_count=1, failed_count=1, total_planning_time_ms=250,
        routes=[
            PlannedRoute(
                "route-good", original_tasks[0].task_id, original_tasks[0].aircraft_id,
                trajectory=[TrajectoryPoint(1.23456, 2, 100, 0, vx=3.45678, vy=1, vz=2)],
                explanation=PlanningExplanation(summary="Full explanation", notes=["Preserve this note"]),
            ),
            PlannedRoute("route-failed", original_tasks[1].task_id, original_tasks[1].aircraft_id,
                         status="failed", message="No feasible route"),
        ],
        conflicts=[{"route_a": "route-good", "route_b": "route-failed", "time": 5}],
    )
    clock = [0.0]
    monkeypatch.setattr(stress.time, "time", lambda: clock[0])
    real_write = stress.write_json
    real_archive = stress.archive_planning_result

    def slow_write(*args, **kwargs):
        clock[0] += 50
        return real_write(*args, **kwargs)

    def slow_archive(*args, **kwargs):
        clock[0] += 70
        return real_archive(*args, **kwargs)

    monkeypatch.setattr(stress, "write_json", slow_write)
    monkeypatch.setattr(stress, "archive_planning_result", slow_archive)

    class Planner:
        def __init__(self, **kwargs):
            pass

        def plan_batch(self, tasks, performances, **kwargs):
            # Dispatch may mutate a task; the persisted input must precede it.
            tasks[0].waypoints[0].x = -999
            performances[tasks[0].aircraft_id].physical.weight_kg = 999
            clock[0] += 3
            return result

    monkeypatch.setattr(stress, "ParallelPlanner", Planner)
    scale = stress.run_scale(2, config, obstacles, seed, 1, 100,
                             dispatch=dispatch, verbose=False, archive_directory=tmp_path)
    stem = f"{'dispatch' if dispatch else 'full'}_n000002_seed17"
    captured = json.loads((tmp_path / f"{stem}_input.json").read_text())
    assert captured["tasks"] == [asdict(task) for task in original_tasks]
    assert captured["aircraft_performances"] == {key: asdict(value) for key, value in original_performances.items()}
    assert captured["config"] == asdict(config)
    assert captured["obstacles"]["dynamic_obstacles"][0]["trajectory"][1] == [200, 100, 100, 10]
    assert captured["parameters"]["seed"] == seed
    assert captured["parameters"]["effective_seed"] == seed + 2
    assert captured["parameters"]["dispatch"] is dispatch
    assert captured["git_sha"] == "a" * 40
    output = json.loads((tmp_path / f"{stem}_output.json").read_text())
    assert output == result.summary()
    assert len(output["routes"]) == 2
    assert output["routes"][1]["message"] == "No feasible route"
    state = json.loads((tmp_path / f"{stem}_output_state.json").read_text())
    assert state == asdict(result)
    assert state["conflicts"] == result.conflicts
    assert state["routes"][0]["trajectory"][0]["vx"] == 3.45678
    assert state["routes"][0]["trajectory"][0]["x"] == 1.23456
    assert scale.wall_clock_time_s == 3
    manifest = json.loads((tmp_path / f"{stem}_manifest.json").read_text())
    for field in ("input", "output", "output_state"):
        record = manifest[field]
        data = (tmp_path / record["path"]).read_bytes()
        assert len(data) == record["bytes"]
        assert hashlib.sha256(data).hexdigest() == record["sha256"]


def test_source_snapshot_preserves_exact_input_bytes_and_revision(tmp_path, monkeypatch):
    monkeypatch.setenv("GITHUB_SHA", "b" * 40)
    source, target = tmp_path / "source", tmp_path / "archive"
    source.mkdir()
    original = b'{"tasks": [], "unparsed_value": 1.000000000000001}\n'
    (source / "tasks.json").write_bytes(original)
    manifest = snapshot_input_files(source, target, {"seed": 17}, filenames=("tasks.json", "dynamic_obstacles.json"))
    assert (target / "tasks.json").read_bytes() == original
    assert manifest["git_sha"] == "b" * 40
    assert manifest["arguments"] == {"seed": 17}
    assert manifest["files"][0]["sha256"] == hashlib.sha256(original).hexdigest()
    assert manifest["missing_files"] == ["dynamic_obstacles.json"]
    assert not (target / "dynamic_obstacles.json").exists()
