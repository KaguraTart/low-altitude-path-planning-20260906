"""
uav_planner SDK 契约测试

重点验证「对外契约」而非算法正确性（算法正确性由既有 200+ 测试覆盖）：

- 返回值必须是**纯 JSON 结构**，无 numpy 标量、无 NaN/Infinity；
- 参数错误的错误码稳定；
- 多航点必须真正生效（曾经因为只取起终点而静默丢掉中间经停）。
"""

import json
import math
import os

import pytest

from uav_planner import (
    DEFAULT_CONFIG,
    ErrorCode,
    PlanningError,
    dumps_safe,
    get_version,
    plan_batch,
    plan_from_dir,
    plan_single,
    sanitize,
    validate_route,
)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
INPUT_DIR = os.path.join(ROOT, "input")


def _no_json_artifacts(obj):
    """递归断言：无 numpy 标量、无非有限浮点数"""
    if isinstance(obj, dict):
        for v in obj.values():
            _no_json_artifacts(v)
    elif isinstance(obj, (list, tuple)):
        for v in obj:
            _no_json_artifacts(v)
    elif isinstance(obj, float):
        assert math.isfinite(obj), f"出现非有限浮点数: {obj}"
    elif isinstance(obj, (str, int, bool)) or obj is None:
        pass
    else:
        pytest.fail(f"返回值含非原生类型 {type(obj).__name__}: {obj!r}")


# ---------------------------------------------------------------- 序列化契约

class TestSerialization:
    def test_sanitize_converts_numpy(self):
        import numpy as np

        out = sanitize({"a": np.float64(1.5), "b": np.int64(3), "c": np.bool_(True)})
        assert type(out["a"]) is float
        assert type(out["b"]) is int
        assert type(out["c"]) is bool

    def test_sanitize_replaces_non_finite(self):
        out = sanitize({"nan": float("nan"), "inf": float("inf"), "ok": 1.0})
        assert out["nan"] is None
        assert out["inf"] is None
        assert out["ok"] == 1.0

    def test_dumps_safe_rejects_nan(self):
        """经过 sanitize 后不应再有 NaN；allow_nan=False 是最后一道防线"""
        assert json.loads(dumps_safe({"x": float("nan")})) == {"x": None}
        with pytest.raises(ValueError):
            json.dumps(float("nan"), allow_nan=False)


# ---------------------------------------------------------------- plan_single

class TestPlanSingle:
    def test_basic_call(self):
        r = plan_single({"x": 500, "y": 500, "z": 100}, {"x": 2500, "y": 2500, "z": 150})
        assert r["status"] == "planned"
        assert r["route"]["metrics"]["total_distance_m"] > 0
        assert r["validation"]["is_valid"] is True
        _no_json_artifacts(r)

    def test_accepts_list_points(self):
        r = plan_single([500, 500, 100], [1500, 500, 100])
        assert r["status"] == "planned"

    def test_defaults_match_documented_config(self):
        v = get_version()
        assert v["defaults"]["config"]["x_max"] == DEFAULT_CONFIG["x_max"]
        # 默认网格 50x50x10x120
        assert v["defaults"]["grid_cells"] == 50 * 50 * 10 * 120

    def test_obstacle_avoidance(self):
        """障碍物横在航线上，规划结果不应穿过它"""
        r = plan_single(
            {"x": 500, "y": 1500, "z": 100},
            {"x": 2500, "y": 1500, "z": 100},
            static_obstacles=[{
                "id": "b1", "name": "楼", "type": "building",
                "x": 1500, "y": 1500, "z": 0,
                "width": 80, "depth": 80, "height": 200,
            }],
        )
        assert r["status"] == "planned"
        for p in r["route"]["trajectory"]:
            inside = (abs(p["x"] - 1500) <= 80 and abs(p["y"] - 1500) <= 80
                      and 0 <= p["z"] <= 200)
            assert not inside, f"轨迹点 {p} 落在障碍物内"

    def test_planning_failure_is_not_exception(self):
        """规划不出来属于业务失败：正常返回 status=failed，不抛异常"""
        r = plan_single(
            {"x": 100, "y": 100, "z": 100},
            {"x": 200, "y": 200, "z": 100},
            static_obstacles=[{
                "id": "big", "name": "巨墙", "type": "building",
                "x": 2500, "y": 2500, "z": 0,
                "width": 3000, "depth": 3000, "height": 400,
            }],
        )
        assert r["status"] == "failed"
        assert r["route"]["message"]

    def test_validate_false_skips_validation(self):
        r = plan_single({"x": 500, "y": 500, "z": 100},
                        {"x": 1500, "y": 500, "z": 100}, validate=False)
        assert r["validation"] is None


class TestPlanSingleWaypoints:
    """多航点必须真正经过中间经停点"""

    def test_waypoints_included(self):
        wps = [
            {"id": "start", "x": 500, "y": 500, "z": 100, "type": "start"},
            {"id": "wp1", "x": 500, "y": 4000, "z": 120, "type": "via"},
            {"id": "end", "x": 4000, "y": 4000, "z": 150, "type": "end"},
        ]
        r = plan_single(waypoints=wps)
        assert r["status"] == "planned"

        traj = r["route"]["trajectory"]
        # 轨迹里必须存在靠近 wp1 的点，否则说明经停被丢掉了
        near_wp1 = [p for p in traj if abs(p["x"] - 500) < 300 and abs(p["y"] - 4000) < 300]
        assert near_wp1, "轨迹未经过中间经停点 wp1"

    def test_waypoints_longer_than_direct(self):
        """
        经停航线应显著长于两点直飞——这正是「丢掉经停」会被发现的地方。

        注意选点要真的构成绕行：A* 走 6 邻域（曼哈顿）移动，两点"直飞"
        本身就是阶梯状路径，若经停点落在起终点包围盒内，曼哈顿距离与直飞
        完全相同，断言不出任何东西。这里让经停点把路线拉出一个明显的折返。
        """
        direct = plan_single({"x": 1000, "y": 1000, "z": 100},
                             {"x": 1000, "y": 3000, "z": 150})
        via = plan_single(waypoints=[
            {"id": "s", "x": 1000, "y": 1000, "z": 100, "type": "start"},
            {"id": "w", "x": 3000, "y": 3000, "z": 120, "type": "via"},
            {"id": "e", "x": 1000, "y": 3000, "z": 150, "type": "end"},
        ])
        d_direct = direct["route"]["metrics"]["total_distance_m"]
        d_via = via["route"]["metrics"]["total_distance_m"]
        # 直飞曼哈顿 ≈ 2000m，绕行 ≈ 6000m
        assert d_via > d_direct * 1.5, f"直飞 {d_direct}m，绕行 {d_via}m"

    def test_waypoint_visit_order(self):
        """经停点必须在到达终点之前被经过，不能只是「路径上恰好路过」"""
        r = plan_single(waypoints=[
            {"id": "s", "x": 1000, "y": 1000, "z": 100, "type": "start"},
            {"id": "w", "x": 3000, "y": 3000, "z": 120, "type": "via"},
            {"id": "e", "x": 1000, "y": 3000, "z": 150, "type": "end"},
        ])
        traj = r["route"]["trajectory"]

        def nearest_idx(tx, ty):
            return min(range(len(traj)),
                       key=lambda i: (traj[i]["x"] - tx) ** 2 + (traj[i]["y"] - ty) ** 2)

        i_start = nearest_idx(1000, 1000)
        i_via = nearest_idx(3000, 3000)
        i_end = nearest_idx(1000, 3000)
        assert i_start <= i_via <= i_end, \
            f"访问顺序不对: start@{i_start}, via@{i_via}, end@{i_end}"

    def test_waypoints_and_start_are_mutually_exclusive(self):
        with pytest.raises(PlanningError) as ei:
            plan_single({"x": 0, "y": 0, "z": 50}, waypoints=[
                {"x": 0, "y": 0, "z": 50}, {"x": 100, "y": 100, "z": 50},
            ])
        assert ei.value.code == ErrorCode.INVALID_INPUT

    def test_single_waypoint_rejected(self):
        with pytest.raises(PlanningError) as ei:
            plan_single(waypoints=[{"x": 0, "y": 0, "z": 50}])
        assert ei.value.code == ErrorCode.INVALID_INPUT


# ---------------------------------------------------------------- 入参校验

class TestInputValidation:
    def test_missing_start_goal(self):
        with pytest.raises(PlanningError) as ei:
            plan_single()
        assert ei.value.code == ErrorCode.INVALID_INPUT

    def test_point_missing_field(self):
        with pytest.raises(PlanningError) as ei:
            plan_single({"x": 1, "y": 2}, {"x": 3, "y": 4, "z": 5})
        assert ei.value.code == ErrorCode.INVALID_INPUT
        assert "z" in ei.value.message

    def test_point_wrong_type(self):
        with pytest.raises(PlanningError) as ei:
            plan_single("不是点", {"x": 1, "y": 2, "z": 3})
        assert ei.value.code == ErrorCode.INVALID_INPUT

    def test_config_zero_resolution(self):
        with pytest.raises(PlanningError) as ei:
            plan_single({"x": 1, "y": 2, "z": 3}, {"x": 4, "y": 5, "z": 6},
                        config={"dx": 0})
        assert ei.value.code == ErrorCode.INVALID_INPUT
        assert "dx" in ei.value.message

    def test_config_inverted_range(self):
        with pytest.raises(PlanningError) as ei:
            plan_single({"x": 1, "y": 2, "z": 3}, {"x": 4, "y": 5, "z": 6},
                        config={"x_min": 5000, "x_max": 0})
        assert ei.value.code == ErrorCode.INVALID_INPUT

    def test_negative_start_time(self):
        with pytest.raises(PlanningError) as ei:
            plan_single({"x": 1, "y": 2, "z": 3}, {"x": 4, "y": 5, "z": 6},
                        start_time=-1)
        assert ei.value.code == ErrorCode.INVALID_INPUT

    def test_error_to_dict_shape(self):
        with pytest.raises(PlanningError) as ei:
            plan_single()
        payload = ei.value.to_dict()
        assert set(payload["error"]) == {"code", "message", "detail"}


# ---------------------------------------------------------------- plan_batch

class TestPlanBatch:
    def _tasks(self):
        return [
            {"id": "t1", "aircraft_id": "a1", "priority": 5, "waypoints": [
                {"id": "s", "x": 500, "y": 500, "z": 100, "type": "start"},
                {"id": "e", "x": 2500, "y": 2500, "z": 150, "type": "end"}]},
            {"id": "t2", "aircraft_id": "a2", "priority": 3, "waypoints": [
                {"id": "s", "x": 4000, "y": 4000, "z": 100, "type": "start"},
                {"id": "e", "x": 1000, "y": 1000, "z": 150, "type": "end"}]},
        ]

    def test_batch_ok(self):
        r = plan_batch(self._tasks(), aircraft=[{"id": "a1"}, {"id": "a2"}], max_workers=2)
        assert r["status"] == "planned"
        assert r["success_count"] + r["failed_count"] == 2
        assert "conflicts" in r
        _no_json_artifacts(r)

    def test_missing_aircraft_raises(self):
        with pytest.raises(PlanningError) as ei:
            plan_batch(self._tasks(), aircraft=[{"id": "a1"}])
        assert ei.value.code == ErrorCode.AIRCRAFT_NOT_FOUND
        assert "a2" in ei.value.detail["missing"]

    def test_aircraft_as_id_keyed_dict(self):
        r = plan_batch(self._tasks(),
                       aircraft={"a1": {"id": "a1"}, "a2": {"id": "a2"}},
                       max_workers=2)
        assert r["success_count"] == 2

    def test_empty_tasks_rejected(self):
        with pytest.raises(PlanningError) as ei:
            plan_batch([], aircraft=[{"id": "a1"}])
        assert ei.value.code == ErrorCode.INVALID_INPUT

    def test_dispatch_mode(self):
        r = plan_batch(self._tasks(), aircraft=[{"id": "a1"}, {"id": "a2"}],
                       max_workers=2, dispatch=True)
        assert r["success_count"] == 2


# ---------------------------------------------------------------- validate_route

class TestValidateRoute:
    def _straight(self):
        return {
            "route_id": "ext-1",
            "trajectory": [
                {"x": 0, "y": 0, "z": 50, "t": 0},
                {"x": 100, "y": 0, "z": 50, "t": 10},
                {"x": 200, "y": 0, "z": 50, "t": 20},
            ],
        }

    def test_valid_external_route(self):
        r = validate_route(self._straight())
        assert r["is_valid"] is True
        assert r["issue_count"] == 0
        assert "summary_text" in r
        _no_json_artifacts(r)

    def test_detects_obstacle_collision(self):
        route = {
            "route_id": "ext-bad",
            "trajectory": [{"x": 2500, "y": 2500, "z": 100, "t": 0},
                           {"x": 3000, "y": 2500, "z": 100, "t": 10}],
        }
        r = validate_route(route, static_obstacles=[{
            "id": "b1", "name": "楼", "type": "building",
            "x": 2500, "y": 2500, "z": 0,
            "width": 80, "depth": 80, "height": 200,
        }])
        assert r["is_valid"] is False
        assert any(i["code"] == "STATIC_OBSTACLE_COLLISION" for i in r["issues"])

    def test_empty_trajectory_invalid(self):
        r = validate_route({"trajectory": []})
        assert r["is_valid"] is False
        assert any(i["code"] == "EMPTY_TRAJECTORY" for i in r["issues"])

    def test_missing_trajectory_raises(self):
        with pytest.raises(PlanningError) as ei:
            validate_route({"route_id": "x"})
        assert ei.value.code == ErrorCode.INVALID_INPUT

    def test_point_missing_time_raises(self):
        with pytest.raises(PlanningError) as ei:
            validate_route({"trajectory": [{"x": 0, "y": 0, "z": 0}]})
        assert ei.value.code == ErrorCode.INVALID_INPUT


# ---------------------------------------------------------------- plan_from_dir

@pytest.mark.skipif(not os.path.isdir(INPUT_DIR), reason="缺少 input/ 目录")
class TestPlanFromDir:
    def test_batch_mode(self):
        r = plan_from_dir(INPUT_DIR, limit=3, max_workers=2)
        assert r["status"] == "planned"
        assert r["success_count"] >= 1
        assert "data_quality" in r
        _no_json_artifacts(r)

    def test_single_mode_uses_full_waypoints(self):
        """单机模式必须走完整航点，否则中间经停会被静默丢掉"""
        r = plan_from_dir(INPUT_DIR, mode="single")
        assert r["status"] == "planned"
        assert r["task_id"]

    def test_missing_dir_raises(self):
        with pytest.raises(PlanningError) as ei:
            plan_from_dir("/definitely/not/here")
        assert ei.value.code == ErrorCode.INVALID_INPUT

    def test_unknown_task_raises(self):
        with pytest.raises(PlanningError) as ei:
            plan_from_dir(INPUT_DIR, task_id="no-such-task")
        assert ei.value.code == ErrorCode.TASK_NOT_FOUND
