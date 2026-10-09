"""时空占用表 + 冲突协调器 + ParallelPlanner 冲突检测测试

覆盖任务书 §三.2「时空资源管理 / 冲突协调」与 §五.4「多机航线冲突避免」。
"""
import pytest

from src.models.obstacles import ObstacleSet
from src.models.task import PlannedRoute, TrajectoryPoint
from src.planners.spacetime_astar import SpaceTimeAStar
from src.planners.parallel_planner import (
    ConflictResolver,
    ParallelPlanner,
    SpatioTemporalOccupancyTable,
)


def _route(route_id, aircraft_id, points):
    """points: [(x,y,z,t), ...] → PlannedRoute"""
    traj = [TrajectoryPoint(x=x, y=y, z=z, t=t) for x, y, z, t in points]
    return PlannedRoute(
        route_id=route_id, task_id=f"task-{route_id}",
        aircraft_id=aircraft_id, trajectory=traj, status="planned",
    )


def _cell_route(config, route_id, aircraft_id, x_cells, y_cell, z_cell, t_steps):
    """在给定网格索引上构造航线。

    坐标必须用 config.to_world 取格心——真实 A* 输出的轨迹点就是格心，
    占用表的距离比较也是基于格心世界坐标，测试数据必须一致。
    """
    points = []
    for ix, it in zip(x_cells, t_steps):
        x, y, z = config.to_world(ix, y_cell, z_cell)
        points.append((x, y, z, config.step_to_time(it)))
    return _route(route_id, aircraft_id, points)


def _line(config, route_id, aircraft_id, y_cell, z_cell=3,
          ix_range=(2, 12), t_offset=None):
    """构造一条沿 x 方向推进的等高航线。

    默认 t_offset 与 ix 对齐（即 it == ix），这样探针 _probe(config, ix, y, z, ix)
    正好落在航线上。传 t_offset=60 可构造"时间整体错开"的航线。
    """
    n = ix_range[1] - ix_range[0] + 1
    t0 = ix_range[0] if t_offset is None else t_offset
    return _cell_route(
        config, route_id, aircraft_id,
        list(range(ix_range[0], ix_range[1] + 1)), y_cell, z_cell,
        list(range(t0, t0 + n)),
    )


def _probe(config, ix, iy, iz, it):
    """构造一个"位于指定格心"的探针（世界坐标 + 时间）"""
    x, y, z = config.to_world(ix, iy, iz)
    return x, y, z, config.step_to_time(it)


class TestOccupancyTable:
    def test_mark_then_is_occupied(self, config):
        """航线在 (ix=6, iy=25, iz=3, it=6) 处的格心应被占用"""
        table = SpatioTemporalOccupancyTable(config)
        table.mark_route(_line(config, "r1", "uav-1", y_cell=25))
        assert table.is_occupied(*_probe(config, 6, 25, 3, 6))
        # 同一格、不同时间步 → 未占用（时空占用是四维的）
        assert not table.is_occupied(*_probe(config, 6, 25, 3, 90))

    def test_empty_table_not_occupied(self, config):
        table = SpatioTemporalOccupancyTable(config)
        assert not table.is_occupied(*_probe(config, 6, 25, 3, 6))

    def test_exclude_own_aircraft(self, config):
        table = SpatioTemporalOccupancyTable(config)
        table.mark_route(_line(config, "r1", "uav-1", y_cell=25))
        assert not table.is_occupied(*_probe(config, 6, 25, 3, 6),
                                     exclude_aircraft="uav-1")
        assert table.is_occupied(*_probe(config, 6, 25, 3, 6),
                                 exclude_aircraft="uav-2")

    def test_unmark_clears(self, config):
        table = SpatioTemporalOccupancyTable(config)
        r = _line(config, "r1", "uav-1", y_cell=25)
        table.mark_route(r)
        table.unmark_route(r)
        assert not table.is_occupied(*_probe(config, 6, 25, 3, 6))
        assert table._occupancy == {}

    def test_clear_resets(self, config):
        table = SpatioTemporalOccupancyTable(config)
        table.mark_route(_line(config, "r1", "uav-1", y_cell=25))
        table.clear()
        assert table._occupancy == {}

    def test_unmark_preserves_other_aircraft(self, config):
        table = SpatioTemporalOccupancyTable(config)
        r1 = _line(config, "r1", "uav-1", y_cell=25)
        r2 = _line(config, "r2", "uav-2", y_cell=25)
        table.mark_route(r1)
        table.mark_route(r2)
        table.unmark_route(r1)
        assert table.is_occupied(*_probe(config, 6, 25, 3, 6))
        assert not table.is_occupied(*_probe(config, 6, 25, 3, 6),
                                     exclude_aircraft="uav-2")

    def test_safety_radius_widens_footprint(self, config):
        """占用半径 3 格（300m）应覆盖相邻 y 方向格"""
        r = _line(config, "r1", "uav-1", y_cell=25)
        tight = SpatioTemporalOccupancyTable(config)
        tight.mark_route(r, safety_radius=0.0)
        wide = SpatioTemporalOccupancyTable(config)
        wide.mark_route(r, safety_radius=300.0)
        probe = _probe(config, 6, 28, 3, 6)
        assert not tight.is_occupied(*probe)
        assert wide.is_occupied(*probe)

    def test_find_conflicts_reports_real_route_ids(self, config):
        """冲突记录应带真实 route_id 与 aircraft_id（修复前只给 aircraft:<id>）"""
        table = SpatioTemporalOccupancyTable(config)
        r1 = _line(config, "route-A", "uav-1", y_cell=25)
        r2 = _line(config, "route-B", "uav-2", y_cell=25)
        table.mark_route(r1)
        conflicts = table.find_conflicts(r2, min_separation=10.0, time_window=5.0)
        assert conflicts
        c = conflicts[0]
        assert c["route_a"] == "route-B"
        assert c["route_b"] == "route-A"
        assert c["aircraft_b"] == "uav-1"
        assert c["distance"] < 10.0

    def test_find_conflicts_ignores_same_aircraft(self, config):
        table = SpatioTemporalOccupancyTable(config)
        table.mark_route(_line(config, "route-A", "uav-1", y_cell=25))
        conflicts = table.find_conflicts(
            _line(config, "route-B", "uav-1", y_cell=25), min_separation=10.0
        )
        assert conflicts == []

    def test_find_conflicts_respects_time_window(self, config):
        table = SpatioTemporalOccupancyTable(config)
        table.mark_route(_line(config, "route-A", "uav-1", y_cell=25, t_offset=0))
        # 空间完全重合但时间上错开 60 个时间步（600s）
        conflicts = table.find_conflicts(
            _line(config, "route-B", "uav-2", y_cell=25, t_offset=60),
            min_separation=10.0, time_window=5.0,
        )
        assert conflicts == []


class TestConflictResolver:
    def test_suggest_delay_within_bound(self, config):
        resolver = ConflictResolver(config)
        d = resolver.suggest_delay({"time": 20.0}, max_delay_s=60.0)
        assert d is not None and 0 < d <= 60.0

    def test_suggest_delay_caps_at_max(self, config):
        resolver = ConflictResolver(config)
        d = resolver.suggest_delay({"time": 1000.0}, max_delay_s=60.0)
        assert d == pytest.approx(60.0)

    def test_suggest_altitude_shift_up(self, config):
        resolver = ConflictResolver(config)
        assert resolver.suggest_altitude_shift(150.0) == pytest.approx(150.0 + config.dz)

    def test_suggest_altitude_shift_caps_at_ceiling(self, config):
        resolver = ConflictResolver(config)
        assert resolver.suggest_altitude_shift(config.z_max) == pytest.approx(config.z_max)


class TestParallelPlannerConflictApi:
    def test_build_occupancy_table_skips_failed(self, config):
        planner = ParallelPlanner(config=config, obstacle_set=ObstacleSet(), max_workers=1)
        ok = _line(config, "r1", "uav-1", y_cell=25)
        bad = PlannedRoute(route_id="r2", task_id="t", aircraft_id="uav-2",
                           status="failed")
        table = planner.build_occupancy_table([ok, bad])
        assert table.is_occupied(*_probe(config, 6, 25, 3, 6))
        assert not table.is_occupied(*_probe(config, 6, 10, 3, 6))

    def test_detect_conflicts_occupancy_finds_parallel_routes(self, config):
        planner = ParallelPlanner(config=config, obstacle_set=ObstacleSet(), max_workers=1)
        routes = [
            _line(config, "r1", "uav-1", y_cell=25),
            _line(config, "r2", "uav-2", y_cell=25),
        ]
        conflicts = planner.detect_conflicts_occupancy(routes, min_separation=10.0)
        assert conflicts
        assert {c["route_a"] for c in conflicts} & {"r1", "r2"}

    def test_detect_conflicts_occupancy_no_false_positives(self, config):
        planner = ParallelPlanner(config=config, obstacle_set=ObstacleSet(), max_workers=1)
        routes = [
            _line(config, "r1", "uav-1", y_cell=10),
            _line(config, "r2", "uav-2", y_cell=40),
        ]
        assert planner.detect_conflicts_occupancy(routes, min_separation=10.0) == []

    def test_detect_conflicts_occupancy_dedups(self, config):
        """同一对航线不应因多轨迹点而被重复计为多条冲突"""
        planner = ParallelPlanner(config=config, obstacle_set=ObstacleSet(), max_workers=1)
        routes = [
            _line(config, "r1", "uav-1", y_cell=25),
            _line(config, "r2", "uav-2", y_cell=25),
        ]
        conflicts = planner.detect_conflicts_occupancy(routes, min_separation=10.0)
        keys = [(c["route_a"], c["route_b"], c["time"]) for c in conflicts]
        assert len(keys) == len(set(keys))

    def test_max_routes_limits_sampling(self, config):
        planner = ParallelPlanner(config=config, obstacle_set=ObstacleSet(), max_workers=1)
        routes = [
            _line(config, "r1", "uav-1", y_cell=25),
            _line(config, "r2", "uav-2", y_cell=25),
            _line(config, "r3", "uav-3", y_cell=25),
        ]
        limited = planner.detect_conflicts_occupancy(routes, max_routes=1)
        assert limited == []

    def test_resolve_conflicts_adds_resolution(self, config):
        planner = ParallelPlanner(config=config, obstacle_set=ObstacleSet(), max_workers=1)
        routes = [
            _line(config, "r1", "uav-1", y_cell=25),
            _line(config, "r2", "uav-2", y_cell=25),
        ]
        conflicts = planner.detect_conflicts_occupancy(routes, min_separation=10.0)
        resolved = planner.resolve_conflicts(conflicts, routes)
        assert resolved
        for item in resolved:
            assert item["resolution"] in ("delay", "altitude_shift")
            if item["resolution"] == "delay":
                assert "suggested_delay_s" in item
            else:
                assert "suggested_z" in item

    def test_resolve_conflicts_switches_to_altitude_on_late_conflict(self, config):
        """冲突时刻晚于可接受延迟窗口时，应改用高度调整建议"""
        planner = ParallelPlanner(config=config, obstacle_set=ObstacleSet(), max_workers=1)
        routes = [
            _line(config, "r1", "uav-1", y_cell=25, t_offset=90),
            _line(config, "r2", "uav-2", y_cell=25, t_offset=90),
        ]
        conflicts = planner.detect_conflicts_occupancy(routes, min_separation=10.0)
        assert conflicts
        resolved = planner.resolve_conflicts(conflicts, routes, max_delay_s=60.0)
        assert all(r["resolution"] == "altitude_shift" for r in resolved)

    def test_resolve_conflicts_uses_delay_for_early_conflict(self, config):
        planner = ParallelPlanner(config=config, obstacle_set=ObstacleSet(), max_workers=1)
        routes = [
            _line(config, "r1", "uav-1", y_cell=25),
            _line(config, "r2", "uav-2", y_cell=25),
        ]
        conflicts = planner.detect_conflicts_occupancy(routes, min_separation=10.0)
        resolved = planner.resolve_conflicts(conflicts, routes, max_delay_s=600.0)
        assert all(r["resolution"] == "delay" for r in resolved)


class TestParallelPlannerCostGridWiring:
    def test_cost_grid_disabled_by_default(self, config):
        planner = ParallelPlanner(config=config, obstacle_set=ObstacleSet(), max_workers=1)
        assert planner._shared_cost_grid is None
        assert planner._planner.cost_grid is None

    def test_cost_grid_enabled_on_request(self, config):
        planner = ParallelPlanner(
            config=config, obstacle_set=ObstacleSet(),
            max_workers=1, use_cost_grid=True,
        )
        assert planner._shared_cost_grid is not None
        # 共享规划器复用同一份只读代价层
        assert planner._planner.cost_grid is planner._shared_cost_grid

    def test_per_task_planner_shares_cost_grid(self, config, monkeypatch):
        """逐任务构造的 A* 也应复用共享代价层（而不是每个任务新建 12MB）"""
        planner = ParallelPlanner(
            config=config, obstacle_set=ObstacleSet(),
            max_workers=1, use_cost_grid=True,
        )
        captured = []
        original = SpaceTimeAStar.__init__

        def spy(self, *args, **kwargs):
            original(self, *args, **kwargs)
            captured.append(self.cost_grid)

        monkeypatch.setattr(SpaceTimeAStar, "__init__", spy)
        planner._plan_one(_simple_task(), _simple_perf(), 0, dispatch=True)
        assert captured and captured[0] is planner._shared_cost_grid

    def test_plan_batch_still_succeeds_with_cost_grid(self, config):
        """启用代价层后批量规划仍应成功"""
        planner = ParallelPlanner(
            config=config, obstacle_set=ObstacleSet(),
            max_workers=2, use_cost_grid=True,
        )
        task = _simple_task()
        result = planner.plan_batch(
            [task], {_simple_perf().aircraft_id: _simple_perf()}, dispatch=True
        )
        assert result.success_count == 1


def _simple_task():
    from src.models.aircraft import FlightTask, TaskRequirements, Waypoint
    return FlightTask(
        task_id="t-cost-001", task_name="代价层测试", aircraft_id="uav-cost",
        priority=5, departure_time=0.0,
        waypoints=[
            Waypoint(wp_id="start", x=500.0, y=2500.0, z=150.0, wp_type="start"),
            Waypoint(wp_id="end", x=1500.0, y=2500.0, z=150.0, wp_type="end"),
        ],
        requirements=TaskRequirements(priority_level=3),
    )


def _simple_perf():
    from src.models.aircraft import AircraftPerformance
    return AircraftPerformance(aircraft_id="uav-cost")