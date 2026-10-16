"""端到端多机冲突避免回归测试（任务书 §五.4「多机航线冲突避免」）

背景：接入前 A\\* 从不消费 `SpatioTemporalOccupancyTable`，冲突只有"事后检测"，
规划器对"两机在同一格同一秒重叠"照样返回 planned、exit 0。本文件锁定修复后的
三件事：

1. **真的避开了**：对向穿越场景在开启避让后残余冲突为 0；
2. **关掉也没撒谎**：`conflict_avoidance=False` 时仍然如实报告检出的冲突
   （"未检查" ≠ "无冲突"）；
3. **时间窗语义不回退**：占用表按 `time_to_step`（向下取整）登记、
   `step_to_time`（步中心）回读，两者不同源，历史上曾把相差一整个 dt 的
   两段轨迹判成 0m 冲突，导致避让循环空转。

交叉验证一律另写一套**独立于本项目检测代码**的最小距离计算，避免
"检测器和验证器共享同一个 bug"。
"""
import os
import random
import sys

import pytest
import numpy as np

from src.models.aircraft import AircraftPerformance, FlightTask, Waypoint
from src.models.obstacles import ObstacleSet
from src.planners.parallel_planner import (
    ParallelPlanner,
    SpatioTemporalOccupancyTable,
)
from src.models.task import PlannedRoute, TrajectoryPoint
from src.validator.path_validator import PathValidator

MIN_SEP = 30.0

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# 入口脚本已从仓库根移入 tools/（根目录只保留交付物）；统一从这里解析路径
TOOLS = os.path.join(ROOT, "tools")

# 测试复用 run_stress_test.py 里的随机任务生成器；该脚本现在位于 tools/，
# 因此把 tools/ 加入模块搜索路径，保证 `import run_stress_test` 可用。
if TOOLS not in sys.path:
    sys.path.insert(0, TOOLS)


def _tool_path(name):
    """返回 tools/ 下入口脚本的绝对路径"""
    return os.path.join(TOOLS, name)


# ---------------------------------------------------------------------------
# 场景构造
# ---------------------------------------------------------------------------
def _task(i, x0, x1, y=4850.0, z=75.0, departure=0.0):
    """沿 y=常数 直线飞行的两点任务"""
    return FlightTask(
        task_id=f"task-{i}", task_name=f"head-on-{i}",
        aircraft_id=f"uav-{i}",
        waypoints=[
            Waypoint(wp_id="start", x=float(x0), y=y, z=z, wp_type="start"),
            Waypoint(wp_id="end", x=float(x1), y=y, z=z, wp_type="end"),
        ],
        priority=5, task_type="patrol", departure_time=departure,
        objective_weights={"distance": 0.4, "time": 0.3, "risk": 0.2, "energy": 0.1},
    )


def _performances(n):
    return {f"uav-{i}": AircraftPerformance(aircraft_id=f"uav-{i}") for i in range(n)}


def _plan_crossing(config, conflict_avoidance, n_pairs=4, min_sep=MIN_SEP):
    """n_pairs 组对向穿越，分处不同高度平行航线。

    每条车道 y = 1050 + 300k（300m 间距 > min_sep），车道之间互不干扰，
    车道内两机反向穿越必然在中点重叠。整体确定性强、规划极快
    （相比随机批次，不会撞上 p95≈12s 的病态 A* 实例）。
    """
    planner = ParallelPlanner(
        config=config, obstacle_set=ObstacleSet(),
        max_workers=4, use_cost_grid=False,
    )
    tasks, n = [], 0
    for k in range(n_pairs):
        y = 1050.0 + 300.0 * k
        tasks.append(_task(n, 500.0, 4500.0, y=y)); n += 1
        tasks.append(_task(n, 4500.0, 500.0, y=y)); n += 1
    return planner.plan_batch(
        tasks, _performances(n), dispatch=False,
        conflict_avoidance=conflict_avoidance, min_separation=min_sep,
    )


# ---------------------------------------------------------------------------
# 独立交叉验证：不复用项目里的任何检测/校验代码
# ---------------------------------------------------------------------------
def _min_same_time_distance(route_a, route_b):
    """Independent exact relative-motion oracle; timestamps need not align."""
    best = None
    for a0,a1 in zip(route_a.trajectory,route_a.trajectory[1:]):
        for b0,b1 in zip(route_b.trajectory,route_b.trajectory[1:]):
            lo,hi=max(a0.t,b0.t),min(a1.t,b1.t)
            if hi<lo or a1.t<=a0.t or b1.t<=b0.t:
                continue
            a=np.array([a0.x,a0.y,a0.z]);b=np.array([b0.x,b0.y,b0.z])
            va=(np.array([a1.x,a1.y,a1.z])-a)/(a1.t-a0.t)
            vb=(np.array([b1.x,b1.y,b1.z])-b)/(b1.t-b0.t)
            relative=a+va*(lo-a0.t)-b-vb*(lo-b0.t)
            velocity=va-vb
            speed2=float(np.dot(velocity,velocity))
            offset=float(np.clip(-np.dot(relative,velocity)/speed2,0,hi-lo)) if speed2 else 0
            distance=float(np.linalg.norm(relative+velocity*offset))
            best=distance if best is None else min(best,distance)
    return best


def _min_over_all_pairs(routes):
    """所有"时间上确实共存"的航线对里的最小距离。

    Returns:
        (min_distance, n_pairs_compared)。n_pairs_compared 用于防止
        断言变成空集通过——若为 0 说明这次检查根本没覆盖到任何一对。
    """
    planned = [r for r in routes if r.status == "planned"]
    best, checked = float("inf"), 0
    for i in range(len(planned)):
        for j in range(i + 1, len(planned)):
            d = _min_same_time_distance(planned[i], planned[j])
            if d is None:
                continue
            checked += 1
            best = min(best, d)
    return best, checked


def _collision_issues(validator, routes):
    result = validator.validate_multi_routes(routes)
    return [i for i in result.issues if i.code == "MULTI_UAV_COLLISION"]


# ---------------------------------------------------------------------------
# 1. 对向穿越：避让必须真的生效
# ---------------------------------------------------------------------------
class TestHeadOnAvoidance:
    def test_scenario_is_a_genuine_conflict_without_avoidance(self, config):
        """反证：关闭避让时该场景确实撞在一起（否则下面的"0 冲突"没有意义）"""
        res = _plan_crossing(config, False, n_pairs=4)
        assert res.success_count == 8, "四条车道两两对向，八机都应规划成功"
        assert res.has_conflicts, "关闭避让时必须检出冲突"
        assert min(c["distance"] for c in res.conflicts) == 0.0
        d, n = _min_over_all_pairs(res.routes)
        assert n >= 4, "至少四对车道存在时间共存"
        assert d == 0.0

    def test_avoidance_yields_zero_residual_conflicts(self, config):
        res = _plan_crossing(config, True, n_pairs=4)
        assert res.success_count == 8
        assert res.conflicts == []
        assert not res.has_conflicts

    def test_avoidance_min_distance_meets_separation(self, config):
        """用独立实现复核：同一时刻的最小距离 ≥ 间隔要求"""
        res = _plan_crossing(config, True, n_pairs=4)
        d, n = _min_over_all_pairs(res.routes)
        assert n >= 4, "交叉验证必须真正覆盖到车道对"
        assert d >= MIN_SEP

    def test_avoidance_result_passes_path_validator(self, config):
        """校验器（走占用表快路径）也不能报出碰撞"""
        res = _plan_crossing(config, True, n_pairs=4)
        validator = PathValidator(
            obstacle_set=ObstacleSet(), min_separation=MIN_SEP, config=config,
        )
        assert _collision_issues(validator, res.routes) == []

    def test_disabled_avoidance_still_reports_conflicts(self, config):
        """关掉避让 ≠ 关掉检测：报告里必须留下残余冲突"""
        res = _plan_crossing(config, False, n_pairs=4)
        assert res.conflicts, "关闭避让时 plan_batch 仍须如实返回冲突"
        validator = PathValidator(
            obstacle_set=ObstacleSet(), min_separation=MIN_SEP, config=config,
        )
        assert _collision_issues(validator, res.routes), "校验器也应报出同一处碰撞"


# ---------------------------------------------------------------------------
# 2. 时间窗语义：occupancy 登记用 time_to_step，回读用 step_to_time
# ---------------------------------------------------------------------------
def _cell_route(config, route_id, aircraft_id, ix, iy, iz, it):
    """在指定格心、指定时间步上放一个轨迹点"""
    x, y, z = config.to_world(ix, iy, iz)
    return PlannedRoute(
        route_id=route_id, task_id=f"task-{route_id}", aircraft_id=aircraft_id,
        trajectory=[TrajectoryPoint(x=x, y=y, z=z, t=config.step_to_time(it))],
        status="planned",
    )


class TestTimeWindowSemantics:
    def test_same_cell_adjacent_steps_is_not_conflict(self, config):
        """回归：相差一整步（10s）的两机在同一格不得判为冲突。

        这曾是真 bug——find_conflicts 的时间步区间没按 step_to_time 的
        "+0.5dt 步中心" 约定反解，把窗外一步纳入，报出 distance=0 的假
        冲突，避让循环因此反复重规划却"路径未变"（该格本来就没被占用）。
        """
        table = SpatioTemporalOccupancyTable(config, min_separation=MIN_SEP)
        table.mark_route(_cell_route(config, "r-b", "uav-b", 10, 25, 3, 49))
        probed = _cell_route(config, "r-a", "uav-a", 10, 25, 3, 48)
        assert table.find_conflicts(probed, min_separation=MIN_SEP) == []

    def test_same_cell_same_step_is_conflict(self, config):
        """同一格同一时间步必须报出（0m 距离）"""
        table = SpatioTemporalOccupancyTable(config, min_separation=MIN_SEP)
        table.mark_route(_cell_route(config, "r-b", "uav-b", 10, 25, 3, 48))
        probed = _cell_route(config, "r-a", "uav-a", 10, 25, 3, 48)
        conflicts = table.find_conflicts(probed, min_separation=MIN_SEP)
        assert len(conflicts) == 1
        assert conflicts[0]["distance"] == 0.0
        assert conflicts[0]["route_b"] == "r-b"

    def test_adjacent_cell_within_separation_is_conflict(self, config):
        """空间上贴近但同一时间步 → 距离 < min_separation 时仍算冲突"""
        table = SpatioTemporalOccupancyTable(config, min_separation=MIN_SEP)
        # 相邻格中心距 = dx = 100m > 30m，应判为不冲突
        table.mark_route(_cell_route(config, "r-b", "uav-b", 11, 25, 3, 48))
        probed = _cell_route(config, "r-a", "uav-a", 10, 25, 3, 48)
        assert table.find_conflicts(probed, min_separation=MIN_SEP) == []

    def test_time_window_covers_non_centered_timestamp(self, config):
        """不在步中心的时间戳（如起飞时间偏移）也要落进窗口内"""
        table = SpatioTemporalOccupancyTable(config, min_separation=MIN_SEP)
        # t = t_min + 0.4*dt → time_to_step 落到 step 0
        t_off = config.t_min + 0.4 * config.dt
        x, y, z = config.to_world(10, 25, 3)
        other = PlannedRoute(
            route_id="r-b", task_id="task-b", aircraft_id="uav-b",
            trajectory=[TrajectoryPoint(x=x, y=y, z=z, t=config.step_to_time(0))],
            status="planned",
        )
        table.mark_route(other)
        probed = PlannedRoute(
            route_id="r-a", task_id="task-a", aircraft_id="uav-a",
            trajectory=[TrajectoryPoint(x=x, y=y, z=z, t=t_off)],
            status="planned",
        )
        conflicts = table.find_conflicts(probed, min_separation=MIN_SEP)
        assert len(conflicts) == 1, "偏移时间戳应与所在时间步的占用比对上"


# ---------------------------------------------------------------------------
# 3. 随机批次：残余冲突必须归零
# ---------------------------------------------------------------------------
class TestRandomBatchSafety:
    @pytest.mark.parametrize("n", [16])
    def test_random_batch_zero_residual_conflicts(self, config, n):
        """随机批次（含多航点任务）开启避让后残余冲突必须为 0"""
        from run_stress_test import generate_random_tasks

        obstacle_set, _ = _load_obstacles()
        rng = random.Random(20260906)
        tasks, performances = generate_random_tasks(
            n, config, obstacle_set, rng, departure_window=600.0,
        )
        planner = ParallelPlanner(
            config=config, obstacle_set=obstacle_set, max_workers=4,
            use_cost_grid=True,
        )
        res = planner.plan_batch(
            tasks, performances, dispatch=True,
            conflict_avoidance=True, min_separation=MIN_SEP,
        )
        assert res.conflicts == [], f"残余冲突 {len(res.conflicts)} 对: {res.conflicts[:3]}"
        d, checked = _min_over_all_pairs(res.routes)
        assert checked >= 1, "独立交叉验证至少要覆盖一对航线"
        assert d >= MIN_SEP
        if res.success_count >= 2:
            validator = PathValidator(
                obstacle_set=obstacle_set, min_separation=MIN_SEP, config=config,
            )
            assert _collision_issues(validator, res.routes) == []

    def test_denser_batch_off_still_detects(self, config):
        """多车道同时穿越 + 关闭避让 → 检测器必须报出冲突。

        用确定性场景而非随机批次：随机小批次往往不产生冲突，
        断言会因种子/规模漂移而失去意义。
        """
        res = _plan_crossing(config, False, n_pairs=6)
        assert res.success_count == 12
        assert res.conflicts
        d, checked = _min_over_all_pairs(res.routes)
        assert checked >= 6
        assert d < MIN_SEP


def _load_obstacles():
    from src.io.input_loader import load_obstacle_set_fused

    static = os.path.join(ROOT, "input", "static_obstacles.json")
    dynamic = os.path.join(ROOT, "input", "dynamic_obstacles.json")
    return load_obstacle_set_fused([static], [dynamic])


# ---------------------------------------------------------------------------
# 4. 入口退出码：残余冲突必须让 CI 感知
# ---------------------------------------------------------------------------
class TestEntryPointExitCode:
    def test_plan_multi_exits_2_on_residual_conflicts(self):
        with open(_tool_path("plan_multi.py"), encoding="utf-8") as f:
            src = f.read()
        assert "if conflicts:" in src
        assert "sys.exit(2)" in src
        assert "--no-avoid-conflicts" in src

    def test_plan_multi_reports_residual_conflicts_not_prefix_sample(self):
        with open(_tool_path("plan_multi.py"), encoding="utf-8") as f:
            src = f.read()
        # 报告的是 plan_batch 返回的全量残余冲突，而非前缀切片
        assert "result.conflicts" in src
        assert "n_residual_conflicts" in src

    def test_stress_test_reports_safety_rate(self):
        """压测必须同时报成功率与安全率——两者正交"""
        with open(_tool_path("run_stress_test.py"), encoding="utf-8") as f:
            src = f.read()
        assert "def safety_rate" in src
        assert "residual_conflict_count" in src
        assert "unsafe_route_count" in src


# ---------------------------------------------------------------------------
# 5. 真实进程级回归：退出码是 CI 唯一能感知到"撞了"的通道
# ---------------------------------------------------------------------------
_HEAD_ON_TASKS = """{
  "tasks": [
    {
      "id": "task-A", "name": "对向穿越A", "aircraft_id": "uav-001",
      "type": "patrol", "priority": 9, "departure_time": 0,
      "objective_weights": {"distance": 0.4, "time": 0.3, "risk": 0.2, "energy": 0.1},
      "waypoints": [
        {"id": "start", "x": 500, "y": 4850, "z": 75, "type": "start"},
        {"id": "end", "x": 4500, "y": 4850, "z": 75, "type": "end"}
      ],
      "requirements": {"payload_kg": 0.1, "required_endurance_s": 900.0,
        "required_range_m": 6000.0, "required_max_speed_mps": 5.0,
        "min_ceiling_m": 80.0, "priority_level": 5, "service_type": "patrol"}
    },
    {
      "id": "task-B", "name": "对向穿越B", "aircraft_id": "uav-002",
      "type": "patrol", "priority": 5, "departure_time": 0,
      "objective_weights": {"distance": 0.4, "time": 0.3, "risk": 0.2, "energy": 0.1},
      "waypoints": [
        {"id": "start", "x": 4500, "y": 4850, "z": 75, "type": "start"},
        {"id": "end", "x": 500, "y": 4850, "z": 75, "type": "end"}
      ],
      "requirements": {"payload_kg": 0.1, "required_endurance_s": 900.0,
        "required_range_m": 6000.0, "required_max_speed_mps": 5.0,
        "min_ceiling_m": 80.0, "priority_level": 3, "service_type": "patrol"}
    }
  ]
}"""


def _run_plan_multi(tmp_path, *extra):
    import json
    import shutil
    import subprocess

    in_dir = tmp_path / "input"
    in_dir.mkdir(exist_ok=True)
    for name in ("config.json", "static_obstacles.json",
                 "dynamic_obstacles.json", "aircraft_config.json"):
        shutil.copy(os.path.join(ROOT, "input", name), in_dir / name)
    with open(in_dir / "tasks.json", "w", encoding="utf-8") as f:
        f.write(_HEAD_ON_TASKS)

    out_dir = tmp_path / "out"
    cmd = [
        sys.executable, _tool_path("plan_multi.py"),
        "--input-dir", str(in_dir), "--output-dir", str(out_dir),
        "--conflict-check", "--min-separation", "30", *extra,
    ]
    proc = subprocess.run(
        cmd, capture_output=True, text=True, timeout=300,
        cwd=ROOT, env={**os.environ, "PYTHONPATH": ROOT},
    )
    return proc, out_dir


class TestPlanMultiExitCodeSubprocess:
    def test_exits_2_when_avoidance_disabled(self, tmp_path):
        """真跑一次进程：关掉避让 → 检出 0m 重叠 → 必须 exit 2"""
        import json

        proc, out_dir = _run_plan_multi(tmp_path, "--no-avoid-conflicts")
        assert proc.returncode == 2, (
            f"期望退出码 2，实际 {proc.returncode}\n"
            f"stdout:\n{proc.stdout[-2000:]}\nstderr:\n{proc.stderr[-2000:]}"
        )
        assert "仍有 1 对未消解" in proc.stdout

        with open(out_dir / "conflicts.json", encoding="utf-8") as f:
            report = json.load(f)
        assert report["avoidance_enabled"] is False
        assert report["n_residual_conflicts"] >= 1
        assert min(c["distance"] for c in report["conflicts"]) == 0.0

    def test_exits_0_when_avoidance_enabled(self, tmp_path):
        """默认开启避让 → 残余冲突 0 → exit 0"""
        import json

        proc, out_dir = _run_plan_multi(tmp_path)
        assert proc.returncode == 0, (
            f"期望退出码 0，实际 {proc.returncode}\n"
            f"stdout:\n{proc.stdout[-2000:]}\nstderr:\n{proc.stderr[-2000:]}"
        )
        assert "✓ 无残余冲突" in proc.stdout

        with open(out_dir / "conflicts.json", encoding="utf-8") as f:
            report = json.load(f)
        assert report["avoidance_enabled"] is True
        assert report["n_residual_conflicts"] == 0
        assert report["conflicts"] == []
