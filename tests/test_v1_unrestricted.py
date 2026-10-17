"""v1 search defaults, explicit budgets and allocation failures."""

import sys
from types import SimpleNamespace

import pytest

from src.models.aircraft import AircraftPerformance, FlightTask, Waypoint
from src.models.obstacles import ObstacleSet
from src.models.space import SpaceTimeConfig
from src.planners.parallel_planner import ParallelPlanner
from src.planners.route_optimizer import RouteOptimizer
from src.planners.spacetime_astar import SpaceTimeAStar
from tools import run_stress_test as stress
from uav_planner.pipeline import run_single_task


@pytest.fixture
def small_scene():
    config = SpaceTimeConfig(
        x_min=0, x_max=60, y_min=0, y_max=30, z_min=0, z_max=30,
        t_min=0, t_max=30, dx=10, dy=10, dz=10, dt=1,
    )
    obstacles = ObstacleSet()
    performance = AircraftPerformance(aircraft_id="test-uav")
    task = FlightTask(
        task_id="test-budget", task_name="search-budget", aircraft_id="test-uav",
        waypoints=[
            Waypoint(wp_id="s", x=5, y=5, z=15, wp_type="start"),
            Waypoint(wp_id="g", x=45, y=5, z=15, wp_type="end"),
        ],
    )
    return config, obstacles, performance, task


@pytest.mark.parametrize("budget", [None, 50])
def test_single_pipeline_applies_budget_to_initial_and_optimization_search(
    small_scene, monkeypatch, budget,
):
    config, obstacles, performance, task = small_scene
    actual_plan = SpaceTimeAStar.plan
    observed = []

    def record_plan(self, *args, **kwargs):
        observed.append(self.max_iterations)
        return actual_plan(self, *args, **kwargs)

    monkeypatch.setattr(SpaceTimeAStar, "plan", record_plan)
    options = {} if budget is None else {"max_iterations": budget}
    route, _, _ = run_single_task(
        task, config, obstacles, performance, optimize=True, verbose=False, **options,
    )

    assert route.status == "planned"
    assert len(observed) >= 2
    assert all(value == budget for value in observed)


def test_optimizer_honors_explicit_iteration_budget(small_scene):
    config, obstacles, performance, task = small_scene
    limited = RouteOptimizer(config, obstacles, performance, max_iterations=1)
    unrestricted = RouteOptimizer(config, obstacles, performance)

    assert limited.optimize(task).status == "failed"
    assert unrestricted.optimize(task).status == "planned"


def test_threaded_batch_propagates_memory_error(small_scene, monkeypatch):
    config, obstacles, _, _ = small_scene
    planner = ParallelPlanner(config, obstacles, max_workers=1)

    def allocation_failure(*args, **kwargs):
        raise MemoryError("allocation failed")

    monkeypatch.setattr(planner, "_plan_one", allocation_failure)
    task = SimpleNamespace(task_id="test-memory", aircraft_id="test-uav")
    with pytest.raises(MemoryError, match="allocation failed"):
        planner._plan_multithread([task], {})


def test_stress_stops_on_memory_error(monkeypatch, tmp_path, input_dir):
    def allocation_failure(*args, **kwargs):
        raise MemoryError("allocation failed")

    monkeypatch.setattr(stress, "run_scale", allocation_failure)
    monkeypatch.setattr(sys, "argv", [
        "run_stress_test.py", "--data-dir", input_dir, "--scales", "1",
        "--output", str(tmp_path),
    ])
    with pytest.raises(MemoryError, match="allocation failed"):
        stress.main()
    assert not (tmp_path / "STRESS_TEST_RESULTS.md").exists()
