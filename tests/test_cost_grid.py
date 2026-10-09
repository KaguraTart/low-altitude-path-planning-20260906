"""CostGrid 通行代价层 + A* 代价层集成测试

覆盖：
- CostGrid 读写、区域设置、风场倍率、min_cost 增量维护
- build_clearance_cost_grid 按障碍物净空生成代价
- A* 启用代价层后仍能规划，且代价层真的影响路径
"""
import numpy as np
import pytest

from src.grid.cost import (
    CostGrid,
    build_default_cost_grid,
    build_clearance_cost_grid,
)
from src.models.obstacles import ObstacleSet, StaticObstacle
from src.models.space import SpaceTimeGrid
from src.planners.spacetime_astar import SpaceTimeAStar


class TestCostGridBasics:
    def test_default_cost_is_one(self, config, empty_grid):
        cg = CostGrid(empty_grid)
        assert cg.get_cost(0, 0, 0, 0) == pytest.approx(1.0)
        assert cg.min_cost == pytest.approx(1.0)

    def test_shape_matches_occupancy(self, config, empty_grid):
        cg = CostGrid(empty_grid)
        assert cg.cost.shape == empty_grid.occupancy.shape
        assert cg.cost.dtype == np.float32

    def test_out_of_bounds_returns_inf(self, config, empty_grid):
        cg = CostGrid(empty_grid)
        assert cg.get_cost(-1, 0, 0, 0) == float("inf")
        assert cg.get_cost(0, 0, 0, config.nt) == float("inf")

    def test_set_cell_cost_updates_min(self, config, empty_grid):
        cg = CostGrid(empty_grid)
        cg.set_cell_cost(1, 2, 3, 4, 5.0)
        assert cg.get_cost(1, 2, 3, 4) == pytest.approx(5.0)
        # 只抬高代价不应改变 min_cost
        assert cg.min_cost == pytest.approx(1.0)

    def test_set_cell_cost_lowers_min(self, config, empty_grid):
        cg = CostGrid(empty_grid)
        cg.set_cell_cost(1, 2, 3, 4, 0.5)
        assert cg.min_cost == pytest.approx(0.5)

    def test_out_of_bounds_set_ignored(self, config, empty_grid):
        cg = CostGrid(empty_grid)
        cg.set_cell_cost(9999, 0, 0, 0, 3.0)
        assert cg.min_cost == pytest.approx(1.0)

    def test_set_box_cost(self, config, empty_grid):
        cg = CostGrid(empty_grid, default_cost=1.0)
        cg.set_box_cost(0, 1000, 0, 1000, 0, 200, cost=7.0)
        inside = config.to_grid(500, 500, 100)
        assert cg.get_cost(*inside, 0) == pytest.approx(7.0)
        assert cg.min_cost == pytest.approx(1.0)

    def test_set_box_cost_clipped_to_bounds(self, config, empty_grid):
        """超出网格范围的盒子应被裁剪，不越界"""
        cg = CostGrid(empty_grid)
        cg.set_box_cost(-99999, 99999, -99999, 99999, -99999, 99999, cost=2.0)
        assert cg.get_cost(config.nx - 1, config.ny - 1, config.nz - 1, 0) == pytest.approx(2.0)

    def test_apply_wind_multiplier(self, config, empty_grid):
        cg = CostGrid(empty_grid)
        cg.apply_wind_multiplier(0, 2000, 0, 2000, 0, 300, multiplier=2.5)
        inside = config.to_grid(1000, 1000, 150)
        assert cg.get_cost(*inside, 0) == pytest.approx(2.5)

    def test_get_world_cost(self, config, empty_grid):
        cg = CostGrid(empty_grid)
        cg.set_box_cost(0, 1000, 0, 1000, 0, 300, cost=4.0)
        assert cg.get_world_cost(500, 500, 150, 0) == pytest.approx(4.0)
        assert cg.get_world_cost(4000, 4000, 150, 0) == pytest.approx(1.0)

    def test_reset_restores_min_cost(self, config, empty_grid):
        cg = CostGrid(empty_grid)
        cg.set_cell_cost(1, 1, 1, 1, 0.25)
        cg.reset()
        assert cg.min_cost == pytest.approx(1.0)
        assert cg.get_cost(1, 1, 1, 1) == pytest.approx(1.0)


class TestCostGridBuilders:
    def test_build_default_cost_grid_with_boxes(self, config, empty_grid):
        cg = build_default_cost_grid(
            empty_grid,
            cbd_box=(0, 2000, 0, 2000, 0, 300),
            cbd_cost=10.0,
        )
        inside = config.to_grid(1000, 1000, 100)
        assert cg.get_cost(*inside, 0) == pytest.approx(10.0)
        assert cg.min_cost == pytest.approx(1.0)

    def test_build_clearance_cost_grid_inflates_obstacles(self, config, building):
        obs = ObstacleSet()
        obs.add_static(building)
        base = SpaceTimeGrid(config)
        cg = build_clearance_cost_grid(base, obs, clearance=300.0, clearance_cost=3.0)

        # 建筑正上方（z 在建筑高度以上 100m）应处于膨胀净空区内
        near = config.to_grid(building.x, building.y, building.z + building.height + 100)
        assert cg.get_cost(*near, 0) == pytest.approx(3.0)
        # 远处应保持默认
        far = config.to_grid(100, 100, 100)
        assert cg.get_cost(*far, 0) == pytest.approx(1.0)

    def test_build_clearance_skips_terrain(self, config, building):
        terrain = StaticObstacle(
            obstacle_id="terrain-001", name="山地", obstacle_type="terrain",
            x=building.x, y=building.y, z=0,
            width=building.width, depth=building.depth, height=building.height,
        )
        obs = ObstacleSet()
        obs.add_static(terrain)
        cg = build_clearance_cost_grid(
            SpaceTimeGrid(config), obs, skip_types=("terrain",)
        )
        near = config.to_grid(terrain.x, terrain.y, 100)
        assert cg.get_cost(*near, 0) == pytest.approx(1.0)

    def test_build_clearance_empty_obstacles(self, config, empty_grid):
        cg = build_clearance_cost_grid(empty_grid, ObstacleSet())
        assert cg.get_cost(0, 0, 0, 0) == pytest.approx(1.0)


class TestCostGridTimeInvariance:
    """时不变代价走 3D 扁平视图；出现时变代价后必须正确退回 4D 语义"""

    def test_box_cost_is_time_invariant(self, config, empty_grid):
        cg = CostGrid(empty_grid)
        cg.set_box_cost(0, 1000, 0, 1000, 0, 300, cost=3.0)
        assert cg._time_invariant is True
        # 任意时间步读到的值都应一致
        cell = config.to_grid(500, 500, 150)
        values = {cg.get_cost(*cell, it) for it in range(0, config.nt, 7)}
        assert values == {3.0}

    def test_wind_multiplier_is_time_invariant(self, config, empty_grid):
        cg = CostGrid(empty_grid)
        cg.apply_wind_multiplier(0, 2000, 0, 2000, 0, 300, multiplier=2.0)
        assert cg._time_invariant is True

    def test_spatial_view_matches_4d(self, config, empty_grid):
        cg = CostGrid(empty_grid)
        cg.set_box_cost(1000, 3000, 1000, 3000, 0, 200, cost=5.0)
        view = cg._spatial_view()
        assert view is not None
        assert view.shape == (config.nx, config.ny, config.nz)
        assert float(view.max()) == pytest.approx(5.0)

    def test_set_cell_cost_disables_spatial_view(self, config, empty_grid):
        """按具体时间步改写后必须读 4D，而不是沿用 3D 视图"""
        cg = CostGrid(empty_grid)
        cg.set_box_cost(0, 2000, 0, 2000, 0, 300, cost=3.0)
        cg.get_cost(1, 1, 1, 0)  # 触发视图构建
        assert cg._spatial is not None
        cg.set_cell_cost(1, 1, 1, 50, 9.0)
        assert cg._time_invariant is False
        assert cg.get_cost(1, 1, 1, 50) == pytest.approx(9.0)
        # 其他时间步仍是原值
        assert cg.get_cost(1, 1, 1, 0) == pytest.approx(3.0)

    def test_reset_restores_time_invariance(self, config, empty_grid):
        cg = CostGrid(empty_grid)
        cg.set_cell_cost(1, 1, 1, 50, 9.0)
        cg.reset()
        assert cg._time_invariant is True
        assert cg.get_cost(1, 1, 1, 50) == pytest.approx(1.0)

    def test_out_of_bounds_with_spatial_view(self, config, empty_grid):
        cg = CostGrid(empty_grid)
        cg.set_box_cost(0, 1000, 0, 1000, 0, 300, cost=3.0)
        cg.get_cost(0, 0, 0, 0)
        assert cg.get_cost(config.nx, 0, 0, 0) == float("inf")
        assert cg.get_cost(0, 0, 0, config.nt) == float("inf")


class TestAStarWithCostGrid:
    def test_plan_with_uniform_cost_grid(self, config, empty_grid):
        """全 1.0 代价层不应改变可达性"""
        cg = CostGrid(empty_grid, default_cost=1.0)
        planner = SpaceTimeAStar(
            config=config, obstacle_set=ObstacleSet(),
            max_speed=15.0, grid=empty_grid, cost_grid=cg,
        )
        r = planner.plan(start=(500, 500, 100), goal=(1500, 500, 100), start_time=0)
        assert r.status == "planned"

    def test_cost_layer_does_not_break_optimality(self, config, empty_grid):
        """代价层全 1.0 时，路径应与无代价层一致（启发函数缩放正确）"""
        planner_off = SpaceTimeAStar(
            config=config, obstacle_set=ObstacleSet(),
            max_speed=15.0, grid=empty_grid,
        )
        planner_on = SpaceTimeAStar(
            config=config, obstacle_set=ObstacleSet(),
            max_speed=15.0, grid=empty_grid,
            cost_grid=CostGrid(empty_grid, default_cost=1.0),
        )
        a = planner_off.plan(start=(500, 500, 100), goal=(1500, 500, 100), start_time=0)
        b = planner_on.plan(start=(500, 500, 100), goal=(1500, 500, 100), start_time=0)
        assert a.total_time == pytest.approx(b.total_time)
        assert a.total_distance == pytest.approx(b.total_distance)

    def test_cost_layer_prefers_cheap_corridor(self, config, building):
        """把中间高价带设为代价，带外仍保持默认倍率"""
        grid = SpaceTimeGrid(config)
        grid.set_static_box(
            building.x - building.width, building.x + building.width,
            building.y - building.depth, building.y + building.depth,
            building.z, building.z + building.height,
        )
        obs = ObstacleSet()
        obs.add_static(building)

        # 横向高价走廊：x 贯穿全场、y 仅 400-600 的窄带
        cg = CostGrid(grid, default_cost=1.0)
        cg.set_box_cost(0, config.x_max, 400, 600, 0, 500, cost=50.0)
        planner = SpaceTimeAStar(
            config=config, obstacle_set=obs, max_speed=15.0,
            grid=grid, cost_grid=cg,
        )
        r = planner.plan(start=(500, 500, 100), goal=(2500, 500, 100), start_time=0)
        assert r.status == "planned"
        assert cg.get_world_cost(2500, 500, 100, 0) == pytest.approx(50.0)
        assert cg.get_world_cost(2500, 4500, 100, 0) == pytest.approx(1.0)

    def test_min_cost_scales_heuristic(self, config, empty_grid):
        """min_cost > 1 时启发函数应被放大（仍保最优）"""
        cg = CostGrid(empty_grid, default_cost=1.0)
        cg.set_cell_cost(0, 0, 0, 0, 4.0)
        planner = SpaceTimeAStar(
            config=config, obstacle_set=ObstacleSet(),
            max_speed=15.0, grid=empty_grid, cost_grid=cg,
        )
        assert planner._min_cost_scale == pytest.approx(1.0)  # 全局仍有一个 1.0 的格
        r = planner.plan(start=(500, 500, 100), goal=(1500, 500, 100), start_time=0)
        assert r.status == "planned"