"""主链路接线回归测试

这些测试锁定"曾经写了但没接线"的模块确实进入了执行路径：
- PathValidator 被 plan_single / plan_multi 调用
- data_fusion 参与输入加载
- 时空占用表参与冲突检测
- CostGrid 参与 A* 搜索
- PlanningExplanation 出现在输出里

通过源码级断言 + 端到端运行双重保证，避免"模块存在但入口没用"的回归。
"""
import inspect
import os
import runpy
import subprocess
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _src(name):
    with open(os.path.join(ROOT, name), encoding="utf-8") as f:
        return f.read()


class TestEntryPointWiring:
    @pytest.mark.parametrize("script", ["plan_single.py", "plan_multi.py"])
    def test_uses_path_validator(self, script):
        """两个入口都应导入并使用 PathValidator"""
        src = _src(script)
        assert "from src.validator import PathValidator" in src
        assert "PathValidator(" in src
        assert "validate_route" in src

    @pytest.mark.parametrize("script", ["plan_single.py", "plan_multi.py"])
    def test_uses_fused_loader(self, script):
        src = _src(script)
        assert "load_obstacle_set_fused" in src
        assert "data_quality" in src

    @pytest.mark.parametrize("script", ["plan_single.py", "plan_multi.py"])
    def test_writes_validation_output(self, script):
        src = _src(script)
        assert "write_validation" in src

    def test_plan_multi_uses_occupancy_table(self):
        src = _src("plan_multi.py")
        assert "detect_conflicts_occupancy" in src
        assert "resolve_conflicts" in src

    def test_plan_single_builds_cost_grid_directly(self):
        """单机侧自建代价层（只有一个规划器，无需共享）"""
        src = _src("plan_single.py")
        assert "build_clearance_cost_grid" in src
        assert "cost_grid=cost_grid" in src

    def test_plan_multi_delegates_cost_grid_to_parallel_planner(self):
        """多机侧由 ParallelPlanner 构建共享代价层（跨线程只读复用）"""
        src = _src("plan_multi.py")
        assert "use_cost_grid" in src
        assert "ParallelPlanner(" in src

    def test_plan_single_exposes_cost_grid_flags(self):
        src = _src("plan_single.py")
        assert "--no-cost-grid" in src
        assert "--clearance" in src
        assert "--no-validate" in src

    def test_plan_multi_exposes_new_flags(self):
        src = _src("plan_multi.py")
        for flag in ("--cost-grid", "--validate", "--conflict-check",
                     "--min-separation", "--conflict-sample"):
            assert flag in src or flag.replace("--cost-grid", "--no-cost-grid") in src

    def test_no_module_defines_unused_validator(self):
        """src.validator 应被入口引用，而不是只有测试引用"""
        srcs = [_src(f"plan_single.py"), _src("plan_multi.py"),
                _src("run_stress_test.py")]
        assert any("PathValidator" in s for s in srcs)


class TestValidatorIsWiredInRuntime:
    def test_plan_single_end_to_end_is_valid(self, tmp_path):
        """端到端跑一次单机规划，验证结果应为合法且产出验证文件"""
        env = dict(os.environ)
        proc = subprocess.run(
            [sys.executable, "plan_single.py", "--output-dir", str(tmp_path)],
            cwd=ROOT, capture_output=True, text=True, env=env, timeout=600,
        )
        out = proc.stdout
        assert "合法性" in out, out[-2000:]
        files = os.listdir(tmp_path)
        assert any(f.startswith("validation_") for f in files), files
        assert any(f.startswith("report_") for f in files), files

    def test_plan_multi_end_to_end_validates(self, tmp_path):
        env = dict(os.environ)
        proc = subprocess.run(
            [sys.executable, "plan_multi.py", "--output-dir", str(tmp_path),
             "--conflict-check", "--validate", "--workers", "2"],
            cwd=ROOT, capture_output=True, text=True, env=env, timeout=900,
        )
        out = proc.stdout
        assert "合法性" in out, out[-2000:]
        assert os.path.exists(os.path.join(tmp_path, "validation.json"))

    def test_planned_routes_pass_validation(self, tmp_path):
        """核心回归：规划成功且合法的任务，其航线不应有任何 ERROR 级问题。

        接入 PathValidator 之前，A* 会为超出飞行器升限/升降率的航段输出
        'planned' 状态，验证模块从未运行因此无人发现。这里确保该类缺陷不再
        静默通过。
        """
        from src.io.input_loader import (
            load_config, load_obstacle_set_fused, load_aircraft_performances,
            load_tasks,
        )
        from src.planners.parallel_planner import ParallelPlanner
        from src.validator import PathValidator

        inp = os.path.join(ROOT, "input")
        config = load_config(os.path.join(inp, "config.json"))
        obstacles, _ = load_obstacle_set_fused(
            [os.path.join(inp, "static_obstacles.json")],
            [os.path.join(inp, "dynamic_obstacles.json")],
        )
        perfs = load_aircraft_performances(os.path.join(inp, "aircraft_config.json"))
        tasks = load_tasks(os.path.join(inp, "tasks.json"))

        planner = ParallelPlanner(
            config=config, obstacle_set=obstacles,
            max_workers=2, use_cost_grid=True,
        )
        result = planner.plan_batch(tasks, perfs, dispatch=False)

        validator = PathValidator(obstacle_set=obstacles)
        for route in result.routes:
            if route.status != "planned":
                continue
            perf = perfs.get(route.aircraft_id)
            v = validator.validate_route(route, performance=perf)
            errors = [i for i in v.issues if i.severity.value == "error"]
            assert not errors, (
                f"{route.route_id} ({route.task_id}) 存在 ERROR 级问题: "
                + "; ".join(e.message for e in errors[:3])
            )

    def test_only_feasible_tasks_are_planned(self):
        """航点高度超出升限的任务应在需求校验阶段被排除，而非规划失败"""
        from src.io.input_loader import load_aircraft_performances, load_tasks

        inp = os.path.join(ROOT, "input")
        perfs = load_aircraft_performances(os.path.join(inp, "aircraft_config.json"))
        tasks = load_tasks(os.path.join(inp, "tasks.json"))
        rejected = set()
        for task in tasks:
            perf = perfs.get(task.aircraft_id)
            if perf is None:
                continue
            ok, problems = task.check_aircraft_compatibility(perf)
            if not ok and any("超出飞行器升限" in p for p in problems):
                rejected.add(task.task_id)
        # 样例数据里 task-003 / task-004 的航点高度超出对应机型升限
        assert {"task-003", "task-004"} <= rejected


class TestMultiWaypointRiskMetrics:
    """多航点航线的风险指标必须在拼接后重算（回归测试）

    历史缺陷：分段规划时每个 seg_route 各自算了 risk，但拼接后的整条航路
    是新的 PlannedRoute 对象，risk 字段停留在默认值 0.0，导致报告里
    "最近障碍距离" 恒为 0.00，与实际路径严重不符。
    """

    def _three_waypoint_task(self):
        from src.models.aircraft import FlightTask, TaskRequirements, Waypoint
        return FlightTask(
            task_id="t-mw-001", task_name="多航点", aircraft_id="uav-mw",
            priority=5, departure_time=0.0,
            waypoints=[
                Waypoint(wp_id="start", x=500.0, y=1000.0, z=150.0, wp_type="start"),
                Waypoint(wp_id="mid", x=2500.0, y=1000.0, z=150.0, wp_type="via"),
                Waypoint(wp_id="end", x=2500.0, y=4000.0, z=150.0, wp_type="end"),
            ],
            requirements=TaskRequirements(priority_level=3),
        )

    def test_multi_waypoint_risk_is_computed(self, config):
        from src.models.obstacles import ObstacleSet, StaticObstacle
        from src.planners.parallel_planner import ParallelPlanner
        from src.models.aircraft import AircraftPerformance

        obstacles = ObstacleSet()
        obstacles.add_static(StaticObstacle(
            obstacle_id="off-route", name="off", obstacle_type="building",
            x=100.0, y=4800.0, z=0.0, width=40.0, depth=40.0, height=100.0,
        ))
        perf = AircraftPerformance(aircraft_id="uav-mw")
        planner = ParallelPlanner(
            config=config, obstacle_set=obstacles, max_workers=1,
        )
        route = planner._plan_one(self._three_waypoint_task(), perf, 0, dispatch=False)
        assert route.status == "planned"
        assert len(route.trajectory) > 1
        # 拼接后必须已算出最近障碍距离（>0 表示确实做了 AABB 距离计算）
        assert route.min_obstacle_distance > 0.0

    def test_multi_waypoint_risk_matches_single_shot(self, config):
        """同一路径下，多航点拼接与单段规划应得到一致的风险指标"""
        from src.models.obstacles import ObstacleSet
        from src.planners.parallel_planner import ParallelPlanner
        from src.models.aircraft import AircraftPerformance
        from src.planners.spacetime_astar import SpaceTimeAStar

        perf = AircraftPerformance(aircraft_id="uav-mw")
        planner = ParallelPlanner(
            config=config, obstacle_set=ObstacleSet(), max_workers=1,
        )
        route = planner._plan_one(self._three_waypoint_task(), perf, 0, dispatch=False)
        assert route.status == "planned"

        # 把拼接后的轨迹单独交给 A* 重算风险，应得到同样的值
        first, last = route.trajectory[0], route.trajectory[-1]
        astar = SpaceTimeAStar(
            config=config, obstacle_set=ObstacleSet(), max_speed=perf.max_speed,
        )
        single = astar.plan(
            start=(first.x, first.y, first.z),
            goal=(last.x, last.y, last.z),
            start_time=first.t,
        )
        assert single.status == "planned"
        assert route.min_obstacle_distance == pytest.approx(
            single.min_obstacle_distance, abs=1e-6
        )


class TestModuleReachability:
    def test_all_public_modules_importable(self):
        """所有 src 子包应可导入（防止相对导入断裂）"""
        import importlib
        for mod in (
            "src.models.space", "src.models.obstacles", "src.models.aircraft",
            "src.models.task", "src.grid.cost", "src.planners.spacetime_astar",
            "src.planners.route_optimizer", "src.planners.parallel_planner",
            "src.validator.path_validator", "src.data_fusion.obstacle_fusion",
            "src.io.input_loader", "src.io.output_writer",
            "src.visualization.viewer",
        ):
            importlib.import_module(mod)

    def test_grid_exports_clearance_builder(self):
        import src.grid as g
        assert hasattr(g, "build_clearance_cost_grid")
        assert hasattr(g, "build_default_cost_grid")

    def test_planners_package_imports_without_error(self):
        import src.planners.parallel_planner as pp
        assert hasattr(pp, "SpatioTemporalOccupancyTable")
        assert hasattr(pp, "ConflictResolver")
        assert hasattr(pp.ParallelPlanner, "detect_conflicts_occupancy")
        assert hasattr(pp.ParallelPlanner, "build_occupancy_table")
        assert hasattr(pp.ParallelPlanner, "resolve_conflicts")