"""飞行器包线约束测试：高度包线 + 爬升/下降率（阶梯式垂直移动）

这些约束是 A* 接入 PathValidator 后暴露出的实际缺陷修复点：
- 规划器必须不产出超出飞行器工作包线的航路
- 爬升/下降率受限时，垂直位移需跨多个时间步（阶梯式），否则无法改变高度
"""
import numpy as np
import pytest

from src.models.obstacles import ObstacleSet
from src.models.space import SpaceTimeGrid
from src.planners.spacetime_astar import SpaceTimeAStar


def _planner(config, grid, **kwargs):
    return SpaceTimeAStar(
        config=config, obstacle_set=ObstacleSet(),
        max_speed=kwargs.pop("max_speed", 15.0), grid=grid, **kwargs
    )


class TestAltitudeEnvelope:
    def test_altitude_band_intersects_grid(self, config, empty_grid):
        p = _planner(config, empty_grid, min_altitude=50.0, max_altitude=250.0)
        iz_min, iz_max = p._iz_min, p._iz_max
        assert p.config.to_world(0, 0, iz_min)[2] <= 50.0 + config.dz
        assert p.config.to_world(0, 0, iz_max)[2] >= 250.0 - config.dz

    def test_no_altitude_limits_defaults_to_full_range(self, config, empty_grid):
        p = _planner(config, empty_grid)
        assert (p._iz_min, p._iz_max) == (0, config.nz - 1)

    def test_altitude_error_detects_too_high(self, config, empty_grid):
        p = _planner(config, empty_grid, min_altitude=10.0, max_altitude=200.0)
        assert p.altitude_error((0, 0, 220), "终点") is not None
        assert p.altitude_error((0, 0, 150), "终点") is None

    def test_altitude_error_detects_too_low(self, config, empty_grid):
        p = _planner(config, empty_grid, min_altitude=30.0, max_altitude=200.0)
        assert p.altitude_error((0, 0, 10), "起点") is not None

    def test_goal_above_ceiling_fails_with_message(self, config, empty_grid):
        p = _planner(config, empty_grid, max_altitude=200.0)
        r = p.plan(start=(500, 500, 100), goal=(1500, 500, 220), start_time=0)
        assert r.status == "failed"
        assert "超过飞行器最高高度" in r.message

    def test_start_above_ceiling_fails_with_message(self, config, empty_grid):
        p = _planner(config, empty_grid, max_altitude=200.0)
        r = p.plan(start=(500, 500, 250), goal=(1500, 500, 100), start_time=0)
        assert r.status == "failed"
        assert "超过飞行器最高高度" in r.message

    def test_trajectory_never_exceeds_ceiling(self, config, empty_grid):
        """规划出的所有轨迹点都应在高度包线内"""
        p = _planner(config, empty_grid, min_altitude=50.0, max_altitude=250.0)
        r = p.plan(start=(500, 500, 60), goal=(2000, 2000, 240), start_time=0)
        assert r.status == "planned"
        for pt in r.trajectory:
            assert 50.0 - 1e-6 <= pt.z <= 250.0 + 1e-6

    def test_trajectory_never_below_floor(self, config, empty_grid):
        p = _planner(config, empty_grid, min_altitude=150.0, max_altitude=400.0)
        r = p.plan(start=(500, 500, 200), goal=(2000, 2000, 160), start_time=0)
        assert r.status == "planned"
        for pt in r.trajectory:
            assert pt.z >= 150.0 - 1e-6

    def test_envelope_above_airspace_fails_fast(self, config, empty_grid):
        """高度包线完全在可用空域之外时，plan 应直接失败并说明是高度原因"""
        p = _planner(config, empty_grid, min_altitude=900.0, max_altitude=950.0)
        r = p.plan(start=(500, 500, 100), goal=(1500, 500, 150), start_time=0)
        assert r.status == "failed"
        assert "高度" in r.message and "飞行器" in r.message


class TestClimbDescentRate:
    def test_descent_rate_respected(self, config, empty_grid):
        """下降率限制为 3 m/s（低于单格 dz/dt=5 m/s），必须阶梯式下降"""
        p = _planner(
            config, empty_grid, max_speed=15.0,
            max_climb_rate=5.0, max_descent_rate=3.0,
            min_altitude=0.0, max_altitude=400.0,
        )
        r = p.plan(start=(500, 500, 400), goal=(3000, 500, 50), start_time=0)
        assert r.status == "planned"
        for i in range(1, len(r.trajectory)):
            p0, p1 = r.trajectory[i - 1], r.trajectory[i]
            dt = p1.t - p0.t
            assert dt > 0
            descent = (p0.z - p1.z) / dt
            assert descent <= 3.0 + 1e-6, f"段{i} 下降率 {descent:.2f} 超限"

    def test_climb_rate_respected(self, config, empty_grid):
        p = _planner(
            config, empty_grid, max_speed=15.0,
            max_climb_rate=2.5, max_descent_rate=5.0,
            min_altitude=0.0, max_altitude=400.0,
        )
        r = p.plan(start=(500, 500, 50), goal=(3000, 500, 400), start_time=0)
        assert r.status == "planned"
        for i in range(1, len(r.trajectory)):
            p0, p1 = r.trajectory[i - 1], r.trajectory[i]
            dt = p1.t - p0.t
            assert dt > 0
            climb = (p1.z - p0.z) / dt
            assert climb <= 2.5 + 1e-6, f"段{i} 爬升率 {climb:.2f} 超限"

    def test_vertical_move_spans_multiple_steps(self, config, empty_grid):
        """dz=50m 而下降率仅 3 m/s（dt=10s → 每步 30m），一次降 1 格需 ≥2 步"""
        p = _planner(
            config, empty_grid, max_speed=15.0,
            max_climb_rate=5.0, max_descent_rate=3.0,
            min_altitude=0.0, max_altitude=400.0,
        )
        r = p.plan(start=(500, 500, 400), goal=(3000, 500, 100), start_time=0)
        assert r.status == "planned"
        multi = 0
        for i in range(1, len(r.trajectory)):
            p0, p1 = r.trajectory[i - 1], r.trajectory[i]
            if abs(p1.z - p0.z) > 1e-6 and (p1.t - p0.t) > config.dt:
                multi += 1
        assert multi > 0, "应存在跨多时间步的垂直移动"

    def test_unlimited_rate_keeps_single_step_climb(self, config, empty_grid):
        """不限制垂直速率时，垂直移动保持单时间步（不额外拖长时间）"""
        p = _planner(
            config, empty_grid, max_speed=15.0,
            min_altitude=0.0, max_altitude=400.0,
        )
        r = p.plan(start=(500, 500, 100), goal=(3000, 500, 250), start_time=0)
        assert r.status == "planned"
        for i in range(1, len(r.trajectory)):
            p0, p1 = r.trajectory[i - 1], r.trajectory[i]
            if abs(p1.z - p0.z) > 1e-6:
                assert (p1.t - p0.t) == pytest.approx(config.dt)

    def test_flat_flight_unaffected_by_climb_limits(self, config, empty_grid):
        """等高飞行不受垂直速率约束影响"""
        p = _planner(
            config, empty_grid, max_speed=15.0,
            max_climb_rate=2.0, max_descent_rate=2.0,
        )
        r = p.plan(start=(500, 500, 150), goal=(2000, 500, 150), start_time=0)
        assert r.status == "planned"
        assert r.total_distance > 0


class TestAircraftWaypointCompatibility:
    """任务航点高度必须落在飞行器包线内（plan_multi 的需求校验前置）"""

    def _task_with_z(self, z_values):
        from src.models.aircraft import FlightTask, Waypoint, TaskRequirements
        wps = [
            Waypoint(wp_id=f"wp{i}", x=1000.0, y=1000.0, z=z, wp_type="waypoint")
            for i, z in enumerate(z_values)
        ]
        return FlightTask(
            task_id="t-001", task_name="测试任务", aircraft_id="uav-x",
            priority=5, departure_time=0.0, waypoints=wps,
            requirements=TaskRequirements(priority_level=3),
        )

    def _perf_with_z(self, min_alt, max_alt):
        from src.models.aircraft import AircraftPerformance, OperationalSpec
        return AircraftPerformance(
            aircraft_id="uav-x",
            operational=OperationalSpec(
                min_altitude_m=min_alt, max_altitude_m=max_alt,
            ),
        )

    def test_waypoint_above_ceiling_rejected(self):
        task = self._task_with_z([100.0, 220.0])
        perf = self._perf_with_z(10.0, 200.0)
        ok, problems = task.check_aircraft_compatibility(perf)
        assert not ok
        assert any("超出飞行器升限" in p for p in problems)

    def test_waypoint_below_floor_rejected(self):
        task = self._task_with_z([5.0, 150.0])
        perf = self._perf_with_z(20.0, 200.0)
        ok, problems = task.check_aircraft_compatibility(perf)
        assert not ok
        assert any("低于飞行器最低工作高度" in p for p in problems)

    def test_waypoints_inside_envelope_accepted(self):
        task = self._task_with_z([50.0, 150.0])
        perf = self._perf_with_z(20.0, 200.0)
        ok, problems = task.check_aircraft_compatibility(perf)
        assert ok, problems

    def test_problem_message_names_waypoint(self):
        task = self._task_with_z([100.0, 220.0])
        perf = self._perf_with_z(10.0, 200.0)
        _, problems = task.check_aircraft_compatibility(perf)
        assert any("wp1" in p for p in problems)


class TestRealDataAltitudeConsistency:
    """真实样例数据里存在航点高度超出升限的任务，应被需求校验拦下"""

    def test_sample_tasks_have_infeasible_pairings(self, real_tasks, real_performances):
        from src.models.aircraft import FlightTask

        infeasible = []
        for task in real_tasks:
            perf = real_performances.get(task.aircraft_id)
            if perf is None:
                continue
            ok, problems = task.check_aircraft_compatibility(perf)
            if not ok and any("超出飞行器升限" in p for p in problems):
                infeasible.append(task.task_id)
        # 这些任务在规划前即应被识别，而不是等到 A* 失败
        assert isinstance(infeasible, list)
        for task in real_tasks:
            perf = real_performances.get(task.aircraft_id)
            if perf is None:
                continue
            ok, _ = task.check_aircraft_compatibility(perf)
            if not ok:
                assert isinstance(task, FlightTask)