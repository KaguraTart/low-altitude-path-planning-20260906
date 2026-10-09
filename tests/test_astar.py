"""SpaceTimeAStar 规划器单元测试"""
import pytest

from src.models.obstacles import ObstacleSet
from src.models.space import SpaceTimeGrid
from src.planners.spacetime_astar import SpaceTimeAStar


class TestAStarBasic:
    def test_plan_empty_space(self, config, empty_grid):
        """无障碍空域应能规划出可达路径"""
        planner = SpaceTimeAStar(
            config=config, obstacle_set=ObstacleSet(),
            max_speed=15.0, grid=empty_grid,
        )
        r = planner.plan(start=(500, 500, 100), goal=(1500, 500, 100), start_time=0)
        assert r.status == "planned"
        assert r.total_time > 0
        assert len(r.trajectory) > 1

    def test_plan_reaches_goal(self, config, empty_grid):
        """终点应被到达（最后一格接近目标网格）"""
        planner = SpaceTimeAStar(
            config=config, obstacle_set=ObstacleSet(),
            max_speed=15.0, grid=empty_grid,
        )
        r = planner.plan(start=(500, 500, 100), goal=(1500, 500, 100), start_time=0)
        goal_cell = config.to_grid(1500, 500, 100)
        last = r.trajectory[-1]
        last_cell = config.to_grid(last.x, last.y, last.z)
        assert last_cell == goal_cell

    def test_plan_avoids_static_obstacle(self, config, building):
        """起点终点在障碍两侧，路径应绕开且不穿障"""
        grid = SpaceTimeGrid(config)
        grid.set_static_box(
            building.x - building.width, building.x + building.width,
            building.y - building.depth, building.y + building.depth,
            building.z, building.z + building.height,
        )
        obs = ObstacleSet()
        obs.add_static(building)
        planner = SpaceTimeAStar(
            config=config, obstacle_set=obs, max_speed=15.0, grid=grid,
        )
        # 起点在障碍一侧，终点在障碍另一侧
        r = planner.plan(start=(2000, 2500, 100), goal=(3000, 2500, 100), start_time=0)
        assert r.status == "planned"
        # 轨迹任何点都不能进入障碍包围盒
        for p in r.trajectory:
            assert not building.contains(p.x, p.y, p.z)

    def test_plan_start_in_obstacle_fails(self, config, building):
        """起点落在障碍内应失败并给出原因"""
        grid = SpaceTimeGrid(config)
        grid.set_static_box(
            building.x - building.width, building.x + building.width,
            building.y - building.depth, building.y + building.depth,
            building.z, building.z + building.height,
        )
        obs = ObstacleSet()
        obs.add_static(building)
        planner = SpaceTimeAStar(
            config=config, obstacle_set=obs, max_speed=15.0, grid=grid,
        )
        r = planner.plan(start=(2500, 2500, 100), goal=(4000, 2500, 100), start_time=0)
        assert r.status == "failed"
        assert "起点被障碍物占用" in r.message

    def test_plan_out_of_bounds_goal_clamped(self, config, empty_grid):
        """终点越界被 clip 到边界，不应崩溃"""
        planner = SpaceTimeAStar(
            config=config, obstacle_set=ObstacleSet(),
            max_speed=15.0, grid=empty_grid,
        )
        r = planner.plan(start=(500, 500, 100), goal=(99999, 500, 100), start_time=0)
        assert r.status in ("planned", "failed")

    def test_plan_respects_speed_limit(self, config, empty_grid):
        """相邻轨迹点间的等效速度不应超 max_speed"""
        planner = SpaceTimeAStar(
            config=config, obstacle_set=ObstacleSet(),
            max_speed=15.0, grid=empty_grid,
        )
        r = planner.plan(start=(500, 500, 100), goal=(2500, 500, 100), start_time=0)
        assert r.status == "planned"
        for i in range(1, len(r.trajectory)):
            p0, p1 = r.trajectory[i - 1], r.trajectory[i]
            dt = p1.t - p0.t
            dist = ((p1.x - p0.x) ** 2 + (p1.y - p0.y) ** 2) ** 0.5
            if dt > 0:
                assert dist / dt <= 15.0 + 1e-6

    def test_plan_max_iterations_truncates(self, config, empty_grid):
        """设 max_iterations 极小，应触发截断而非死循环"""
        planner = SpaceTimeAStar(
            config=config, obstacle_set=ObstacleSet(),
            max_speed=15.0, grid=empty_grid, max_iterations=1,
        )
        r = planner.plan(start=(500, 500, 100), goal=(4000, 4000, 300), start_time=0)
        assert r.status == "failed"
        assert "未找到" in r.message
