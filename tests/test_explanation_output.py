"""结构化规划解释 (PlanningExplanation) + 输出序列化测试

PlanningExplanation 之前只存在于数据模型里，从未被任何规划入口填充，
本文件验证它现在确实被 RouteOptimizer 写出并进入 JSON 输出。
"""
import json
import os

import pytest

from src.io.output_writer import write_validation, write_route
from src.models.obstacles import ObstacleSet
from src.planners.route_optimizer import RouteOptimizer


def _optimizer(config, obstacle_set, perf):
    return RouteOptimizer(config, obstacle_set, perf)


def _task(config, obj=ObstacleSet(), perf=None):
    from src.models.aircraft import FlightTask, TaskRequirements, Waypoint
    return FlightTask(
        task_id="task-exp-001", task_name="解释测试", aircraft_id="uav-exp",
        priority=5, departure_time=0.0,
        waypoints=[
            Waypoint(wp_id="start", x=500.0, y=2500.0, z=150.0, wp_type="start"),
            Waypoint(wp_id="end", x=2500.0, y=2500.0, z=150.0, wp_type="end"),
        ],
        requirements=TaskRequirements(priority_level=3),
        objective_weights={"distance": 0.25, "time": 0.25,
                           "risk": 0.25, "energy": 0.25},
    )


def _count(config, obstacle_set, points, perf=None, radius=50.0):
    from src.models.task import TrajectoryPoint
    opt = _optimizer(config, obstacle_set, perf)
    traj = [TrajectoryPoint(x=x, y=y, z=z, t=t) for x, y, z, t in points]
    return opt._count_avoidances(traj, radius=radius)


class TestCountAvoidances:
    def test_zero_when_no_obstacles(self, config, perf):
        assert _count(config, ObstacleSet(), [(500, 2500, 150, 0)], perf) == 0

    def test_zero_for_empty_trajectory(self, config, perf):
        assert _count(config, ObstacleSet(), [], perf) == 0

    def test_counts_nearby_static(self, config, perf, building):
        """轨迹点距建筑 < 50m 时应计入"""
        pts = [(building.x, building.y, building.z + building.height + 10, 0.0)]
        assert _count(config, _obs(building), pts, perf) == 1

    def test_ignores_distant_static(self, config, perf, building):
        pts = [(building.x + 1000.0, building.y, 150.0, 0.0)]
        assert _count(config, _obs(building), pts, perf) == 0

    def test_counts_distinct_obstacles_once(self, config, perf, building):
        """多个轨迹点贴近同一障碍物只计 1 次"""
        pts = [
            (building.x, building.y, building.z + building.height + 10, 0.0),
            (building.x, building.y, building.z + building.height + 12, 10.0),
            (building.x, building.y, building.z + building.height + 11, 20.0),
        ]
        assert _count(config, _obs(building), pts, perf) == 1

    def test_radius_is_respected(self, config, perf, building):
        """扩大 radius 后应计入更多障碍物"""
        from src.models.obstacles import StaticObstacle
        far = StaticObstacle(
            obstacle_id="far", name="far", obstacle_type="building",
            x=building.x + 300.0, y=building.y, z=0.0,
            width=80.0, depth=80.0, height=200.0,
        )
        obs = ObstacleSet()
        obs.add_static(building)
        obs.add_static(far)
        pts = [(building.x, building.y, 150.0, 0.0)]
        assert _count(obs and config, obs, pts, perf, radius=50.0) == 1
        assert _count(config, obs, pts, perf, radius=400.0) == 2

    def test_counts_nearby_dynamic(self, config, perf):
        from src.models.obstacles import DynamicObstacle
        dyn = DynamicObstacle(
            obstacle_id="dyn-1", name="dyn", obstacle_type="uav",
            trajectory=[(1000.0, 2500.0, 150.0, 0.0),
                        (2000.0, 2500.0, 150.0, 600.0)],
            safety_radius=20.0,
        )
        obs = ObstacleSet()
        obs.add_dynamic(dyn)
        # 距动态障碍中心 30m（< 50m 半径）→ 计入
        assert _count(config, obs, [(1030.0, 2500.0, 150.0, 0.0)], perf) == 1
        assert _count(config, obs, [(1600.0, 2500.0, 150.0, 0.0)], perf) == 0


def _obs(obstacle):
    obs = ObstacleSet()
    obs.add_static(obstacle)
    return obs


class TestPlanningExplanation:
    def test_optimize_populates_explanation(self, config, perf):
        opt = _optimizer(config, ObstacleSet(), perf)
        route = opt.optimize(_task(config), route_id="opt-exp")
        assert route.status == "planned"
        assert route.explanation is not None

    def test_explanation_has_score_and_breakdown(self, config, perf):
        opt = _optimizer(config, ObstacleSet(), perf)
        e = opt.optimize(_task(config)).explanation
        assert e.score >= 0.0
        assert set(e.objective_breakdown) == {"distance", "time", "risk", "energy"}

    def test_breakdown_sums_to_score(self, config, perf):
        """四个目标的加权贡献之和应等于综合得分"""
        opt = _optimizer(config, ObstacleSet(), perf)
        e = opt.optimize(_task(config)).explanation
        assert sum(e.objective_breakdown.values()) == pytest.approx(e.score, abs=1e-6)

    def test_performance_margin_keys(self, config, perf):
        opt = _optimizer(config, ObstacleSet(), perf)
        e = opt.optimize(_task(config)).explanation
        assert set(e.performance_margin) == {
            "speed_ratio", "climb_ratio", "descent_ratio", "turn_rate_ratio"
        }

    def test_performance_margin_within_limits(self, config, perf):
        """优化后应满足性能约束，各项比例不应显著超 1.0"""
        opt = _optimizer(config, ObstacleSet(), perf)
        e = opt.optimize(_task(config)).explanation
        for key, ratio in e.performance_margin.items():
            assert ratio <= 1.15, f"{key}={ratio:.3f} 明显超限"

    def test_summary_is_non_empty(self, config, perf):
        opt = _optimizer(config, ObstacleSet(), perf)
        e = opt.optimize(_task(config)).explanation
        assert e.summary.strip()
        assert "得分" in e.summary

    def test_notes_flag_raw_fallback(self, config, perf):
        """平滑后碰撞回退时应给出对应说明"""
        from src.models.obstacles import StaticObstacle
        # 在航线正中间放一堵墙，迫使平滑路径与静态障碍冲突
        obs = ObstacleSet()
        obs.add_static(StaticObstacle(
            obstacle_id="wall", name="wall", obstacle_type="building",
            x=1500.0, y=2500.0, z=100.0, width=10.0, depth=300.0, height=100.0,
        ))
        opt = _optimizer(config, obs, perf)
        route = opt.optimize(_task(config))
        if route.algorithm.endswith("raw-fallback"):
            assert any("回退" in n for n in route.explanation.notes)

    def test_explanation_to_dict_is_json_serializable(self, config, perf):
        opt = _optimizer(config, ObstacleSet(), perf)
        e = opt.optimize(_task(config)).explanation
        json.dumps(e.to_dict(), ensure_ascii=False)

    def test_explanation_appears_in_route_json(self, config, perf, tmp_path):
        opt = _optimizer(config, ObstacleSet(), perf)
        route = opt.optimize(_task(config))
        path = write_route(route, str(tmp_path))
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        assert data["explanation"] is not None
        assert "objective_breakdown" in data["explanation"]
        assert "performance_margin" in data["explanation"]

    def test_message_still_populated_for_backcompat(self, config, perf):
        """route.message 仍应保留可读摘要（报告章节依赖它）"""
        opt = _optimizer(config, ObstacleSet(), perf)
        route = opt.optimize(_task(config))
        assert "综合得分" in route.message

    def test_explanation_notes_flag_high_risk(self, config, perf):
        """贴近障碍物导致高风险时应给出复核建议"""
        from src.models.obstacles import StaticObstacle
        obs = ObstacleSet()
        obs.add_static(StaticObstacle(
            obstacle_id="lowbuild", name="low", obstacle_type="building",
            x=2500.0, y=2500.0, z=0.0, width=60.0, depth=60.0, height=60.0,
        ))
        opt = _optimizer(config, obs, perf)
        e = opt.optimize(_task(config)).explanation
        assert isinstance(e.notes, list)


class TestWriteValidation:
    def _validation(self, is_valid=True, n_issues=1):
        from src.validator import ValidationResult, ValidationIssue, Severity
        v = ValidationResult(is_valid=is_valid)
        for i in range(n_issues):
            v.add(ValidationIssue(
                severity=Severity.WARNING, code=f"CODE_{i}",
                message=f"问题 {i}", trajectory_index=i,
                time=1.5, position=(1.0, 2.0, 3.0),
            ))
        v.summary = {"n_warnings": n_issues}
        return v

    def test_writes_json(self, tmp_path):
        v = self._validation()
        path = write_validation(v, str(tmp_path), "validation.json")
        assert os.path.exists(path)
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        assert data["is_valid"] is True
        assert data["issue_count"] == 1
        assert data["summary_text"]

    def test_serializes_issue_fields(self, tmp_path):
        v = self._validation()
        path = write_validation(v, str(tmp_path), "validation.json")
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        issue = data["issues"][0]
        assert issue["severity"] == "warning"
        assert issue["code"] == "CODE_0"
        assert issue["position"] == [1.0, 2.0, 3.0]

    def test_extra_fields_merged(self, tmp_path):
        v = self._validation()
        path = write_validation(
            v, str(tmp_path), "validation.json",
            extra={"task_id": "t-1", "data_quality": {"registered_sources": 3}},
        )
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        assert data["task_id"] == "t-1"
        assert data["data_quality"]["registered_sources"] == 3

    def test_invalid_result_recorded(self, tmp_path):
        v = self._validation(is_valid=False, n_issues=0)
        path = write_validation(v, str(tmp_path), "validation.json")
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        assert data["is_valid"] is False


class TestReportSections:
    def test_report_includes_explanation(self, config, perf, tmp_path):
        from src.io.output_writer import generate_report
        from src.models.task import PlanningResult
        opt = _optimizer(config, ObstacleSet(), perf)
        route = opt.optimize(_task(config))
        result = PlanningResult(
            request_id="rep-001", routes=[route],
            success_count=1, failed_count=0,
        )
        path = generate_report(
            result, {"网格": "50x50x10"}, {"static_count": 0},
            str(tmp_path), validation=None,
        )
        text = open(path, encoding="utf-8").read()
        assert "规划解释" in text
        assert "性能裕度" in text

    def test_report_includes_validation_section(self, config, perf, tmp_path):
        from src.io.output_writer import generate_report
        from src.models.task import PlanningResult
        from src.validator import ValidationResult, ValidationIssue, Severity
        v = ValidationResult(is_valid=False)
        v.add(ValidationIssue(
            severity=Severity.ERROR, code="SPEED_EXCEEDED", message="超速",
        ))
        v.summary = {"n_errors": 1}
        opt = _optimizer(config, ObstacleSet(), perf)
        route = opt.optimize(_task(config))
        result = PlanningResult(
            request_id="rep-002", routes=[route],
            success_count=1, failed_count=0,
        )
        path = generate_report(
            result, {"网格": "50x50x10"}, {"static_count": 0},
            str(tmp_path), validation=v, data_quality={"registered_sources": 3},
        )
        text = open(path, encoding="utf-8").read()
        assert "合法性验证" in text
        assert "SPEED_EXCEEDED" in text
        assert "多源数据融合质量" in text