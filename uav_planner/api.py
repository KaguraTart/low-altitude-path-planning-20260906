"""
对外公共 API

这是算法对外承诺的**稳定契约**。所有函数：

- 入参一律是纯 ``dict`` / ``list`` / 数字，不需要调用方 import ``src.*`` 下的任何类；
- 返回值一律是纯 JSON 可序列化结构，可直接 ``json.dumps`` 或作为 HTTP 响应体；
- 参数错误抛 :class:`~uav_planner.errors.PlanningError`（带机器可读错误码）；
  规划不出路径属于**业务失败**，不抛异常，正常返回 ``status="failed"``。

内部实现复用 CLI 已验证的代码路径（``plan_single.plan_single_task``、
``plan_multi.plan_batch``），而不是另写一套编排逻辑——这样「命令行跑出来的」
和「接口调用出来的」结果天然一致，不会随版本漂移。

字段级说明见 ``docs/API.md``。
"""

from __future__ import annotations

import os
import time
from typing import Any, Dict, List, Optional

from src.io.input_loader import (
    load_aircraft_performances,
    load_config,
    load_obstacle_set_fused,
    load_tasks,
)
from src.models.space import SpaceTimeGrid
from src.grid.cost import build_clearance_cost_grid
from src.validator import PathValidator

from .adapters import (
    DEFAULT_AIRCRAFT,
    DEFAULT_CONFIG,
    build_obstacle_set,
    build_task_from_points,
    build_task_from_waypoints,
    parse_aircraft,
    parse_aircraft_map,
    parse_config,
    parse_point,
    parse_tasks,
    parse_waypoints,
)
from .errors import ErrorCode, PlanningError
from .serializer import (
    result_to_dict,
    route_to_dict,
    sanitize,
    validation_to_dict,
)

__version__ = "1.0.0"

#: 本接口封装的算法组件（用于 /version 与文档）
ALGORITHMS = {
    "astar": "多维度时空 A* 搜索（x, y, z, t 四维状态）",
    "optimizer": "多目标航线优化（航程/时间/风险/能耗加权）",
    "parallel": "大规模并行规划 + 多机冲突避免",
    "validator": "路径合法性验证（7 维检查）",
}


# ---------------------------------------------------------------- 内部工具

def _build_cost_grid(config, obstacle_set, clearance: float, clearance_cost: float):
    """构建通行代价层；与占用层正交（占用决定「不能走」，代价决定「更费力」）"""
    base_grid = SpaceTimeGrid(config)
    return build_clearance_cost_grid(
        base_grid, obstacle_set,
        clearance=clearance, clearance_cost=clearance_cost,
    )


def _check_positive(value: float, field: str) -> float:
    if not (float(value) > 0):
        raise PlanningError(
            ErrorCode.INVALID_INPUT,
            f"{field} 必须大于 0，实际 {value}",
            {"field": field, "got": value},
        )
    return float(value)


def _check_non_negative(value: float, field: str) -> float:
    if float(value) < 0:
        raise PlanningError(
            ErrorCode.INVALID_INPUT,
            f"{field} 不能为负，实际 {value}",
            {"field": field, "got": value},
        )
    return float(value)


# ---------------------------------------------------------------- 公共 API

def plan_single(
    start=None,
    goal=None,
    *,
    waypoints=None,
    config: Optional[dict] = None,
    static_obstacles=(),
    dynamic_obstacles=(),
    aircraft: Optional[dict] = None,
    start_time: float = 0.0,
    priority: int = 5,
    objective_weights: Optional[dict] = None,
    optimize: bool = True,
    cost_grid: bool = True,
    clearance: float = 200.0,
    clearance_cost: float = 3.0,
    max_iterations: Optional[int] = None,
    validate: bool = True,
    task_id: str = "task-001",
) -> Dict[str, Any]:
    """
    单条航线的完整规划：时空 A* → 多目标优化 → 合法性验证

    航线用两种方式给出，**二选一**：

    - ``start`` + ``goal``：两点直飞，最简用法；
    - ``waypoints``：完整航点序列，支持经停点（``type="via"``）及其
      ``hover_time`` / 时间窗。多航点会分段规划，结果与只给起终点**不同**，
      所以有中间航点时必须用这种方式传，否则中间的经停会被静默丢掉。

    Args:
        start: 起点，``{"x":..,"y":..,"z":..}`` 或 ``[x, y, z]``，单位米
        goal: 终点，格式同 ``start``
        waypoints: 完整航点序列，每项见 ``docs/API.md`` §2.6；
                   与 ``start``/``goal`` 互斥
        config: 时空网格配置；``None`` 用 :data:`~uav_planner.adapters.DEFAULT_CONFIG`
        static_obstacles: 静态障碍物列表（AABB 包围盒）
        dynamic_obstacles: 动态障碍物列表（轨迹 + 安全半径）
        aircraft: 飞行器性能；``None`` 用默认 15 m/s 通用四旋翼
        start_time: 起飞时刻，秒
        priority: 调度优先级（1-10），影响批量规划时的排序
        objective_weights: 多目标权重，键为 distance/time/risk/energy，和须为 1.0
        optimize: 是否做多目标航线优化（关闭则只返回 A\* 原始结果）
        cost_grid: 是否启用通行代价层（偏好远离建筑群的航路）
        clearance: 障碍物净空膨胀半径，米
        clearance_cost: 净空区内通行代价倍率
        max_iterations: A\* 迭代预算；None 不限制次数
        validate: 是否附带路径合法性验证结果
        task_id: 任务标识，会回填到返回结构里

    Returns:
        见 ``docs/API.md`` §3.1。关键字段：
        - ``status``: ``"planned"`` / ``"failed"``
        - ``route``: 航线详情（指标 + 轨迹点 + 规划解释）
        - ``validation``: 验证结果；``validate=False`` 或规划失败时为 ``None``
        - ``warnings``: 飞行器与任务需求不匹配等非致命告警
    """
    cfg = parse_config(config)
    obstacle_set = build_obstacle_set(static_obstacles, dynamic_obstacles)
    perf = parse_aircraft(aircraft)
    _check_non_negative(start_time, "start_time")
    _check_positive(clearance, "clearance")
    _check_positive(clearance_cost, "clearance_cost")
    if max_iterations is not None:
        _check_positive(max_iterations, "max_iterations")

    if waypoints is not None:
        if start is not None or goal is not None:
            raise PlanningError(
                ErrorCode.INVALID_INPUT,
                "waypoints 与 start/goal 互斥，只能给出其中一种",
                {"hint": "多航点场景请只传 waypoints"},
            )
        task = build_task_from_waypoints(
            parse_waypoints(waypoints),
            task_id=task_id, aircraft_id=perf.aircraft_id,
            start_time=start_time, priority=priority,
            objective_weights=objective_weights,
        )
    else:
        if start is None or goal is None:
            raise PlanningError(
                ErrorCode.INVALID_INPUT,
                "必须提供 start 与 goal，或改传 waypoints",
                {"start_given": start is not None, "goal_given": goal is not None},
            )
        task = build_task_from_points(
            parse_point(start, "start"), parse_point(goal, "goal"),
            task_id=task_id, aircraft_id=perf.aircraft_id,
            start_time=start_time, priority=priority,
            objective_weights=objective_weights,
        )

    ok, msg = task.validate()
    if not ok:
        raise PlanningError(
            ErrorCode.INVALID_INPUT, f"任务定义非法: {msg}", {"task_id": task_id}
        )

    cost_grid_obj = (
        _build_cost_grid(cfg, obstacle_set, clearance, clearance_cost)
        if cost_grid else None
    )

    # 编排逻辑来自本包的 pipeline 模块（原先定义在命令行脚本里，
    # 依赖方向倒挂；现已下沉到 SDK，接口层可独立交付）
    from .pipeline import run_single_task

    t0 = time.time()
    try:
        final_route, all_routes, warnings = run_single_task(
            task, cfg, obstacle_set, perf,
            optimize=optimize,
            max_iterations=int(max_iterations) if max_iterations is not None else None,
            cost_grid=cost_grid_obj,
            verbose=False,
        )
    except ValueError as e:
        # plan_single_task 对非法任务抛 ValueError，翻译成带错误码的调用错误
        raise PlanningError(ErrorCode.INVALID_INPUT, str(e), {"task_id": task_id}) from e
    elapsed_ms = (time.time() - t0) * 1000

    validation = None
    if validate and final_route.status == "planned":
        validator = PathValidator(obstacle_set=obstacle_set)
        validation = validator.validate_route(final_route, performance=perf)

    return sanitize({
        "status": final_route.status,
        "request_id": f"single-{task_id}",
        "task_id": task_id,
        "aircraft_id": perf.aircraft_id,
        "algorithm": final_route.algorithm or "SpaceTimeAStar",
        "elapsed_ms": round(elapsed_ms, 2),
        "route": route_to_dict(final_route),
        "route_count": len(all_routes),
        "validation": validation_to_dict(validation),
        "warnings": list(warnings),
    })


def plan_batch(
    tasks,
    *,
    aircraft=None,
    config: Optional[dict] = None,
    static_obstacles=(),
    dynamic_obstacles=(),
    max_workers: int = 4,
    dispatch: bool = False,
    conflict_avoidance: bool = True,
    min_separation: float = 10.0,
    cost_grid: bool = True,
    clearance: float = 200.0,
    clearance_cost: float = 3.0,
    max_iterations: Optional[int] = None,
    request_id: Optional[str] = None,
    validate_tasks: bool = False,
) -> Dict[str, Any]:
    """
    多任务批量并行规划，含多机冲突避免

    Args:
        tasks: 任务列表，每项含 ``id`` / ``aircraft_id`` / ``waypoints`` 等
        aircraft: 飞行器列表或 ``{id: {...}}`` 字典；任务引用的 ID 必须存在
        config: 时空网格配置
        static_obstacles / dynamic_obstacles: 障碍物
        max_workers: 并行线程数
        dispatch: 调度模式。``True`` 时跳过 risk/heading 等附加字段的计算
                  （实测省约 8%），适用于只看可行性与时长的调度场景
        conflict_avoidance: 是否做多机冲突避免。关闭后 **仍会如实报告**
                            检测到的冲突，不会把「没检查」包装成「无冲突」
        min_separation: 多机最小安全间隔，米
        cost_grid / clearance / clearance_cost: 通行代价层参数
        max_iterations: 单任务 A\* 迭代预算；None 不限制次数
        request_id: 请求标识，缺省由任务数生成
        validate_tasks: 是否先做飞行器需求兼容性校验并剔除不兼容任务

    Returns:
        见 ``docs/API.md`` §3.2
    """
    cfg = parse_config(config)
    obstacle_set = build_obstacle_set(static_obstacles, dynamic_obstacles)
    perfs = parse_aircraft_map(aircraft)
    task_objs = parse_tasks(tasks)

    _check_positive(max_workers, "max_workers")
    _check_positive(min_separation, "min_separation")
    if max_iterations is not None:
        _check_positive(max_iterations, "max_iterations")

    # 任务引用的飞行器必须存在，否则规划到一半才炸，错误定位成本很高
    missing = sorted({t.aircraft_id for t in task_objs if t.aircraft_id not in perfs})
    if missing:
        raise PlanningError(
            ErrorCode.AIRCRAFT_NOT_FOUND,
            f"以下飞行器未在 aircraft 中定义: {', '.join(missing)}",
            {"missing": missing, "available": sorted(perfs.keys())},
        )

    warnings: List[str] = []
    if validate_tasks:
        from .pipeline import validate_tasks_with_aircraft

        task_objs, incompatible = validate_tasks_with_aircraft(task_objs, perfs)
        for task_id, problems in incompatible:
            warnings.append(f"任务 {task_id} 与飞行器不兼容: {'; '.join(problems)}")
        if not task_objs:
            raise PlanningError(
                ErrorCode.PLANNING_FAILED,
                "所有任务都与飞行器需求不兼容，无可规划任务",
                {"incompatible_count": len(incompatible)},
            )

    from .pipeline import run_batch as _run_batch

    t0 = time.time()
    result, _planner = _run_batch(
        task_objs, perfs, cfg, obstacle_set,
        dispatch=dispatch,
        workers=int(max_workers),
        max_iterations=int(max_iterations) if max_iterations is not None else None,
        use_cost_grid=bool(cost_grid),
        conflict_avoidance=conflict_avoidance,
        min_separation=float(min_separation),
        clearance=float(clearance),
        clearance_cost=float(clearance_cost),
    )
    elapsed_ms = (time.time() - t0) * 1000

    out = result_to_dict(result)
    out["status"] = "planned" if result.success_count > 0 else "failed"
    out["request_id"] = request_id or out.get("request_id") or f"batch-{len(task_objs)}"
    out["elapsed_ms"] = round(elapsed_ms, 2)
    out["warnings"] = warnings
    if warnings:
        # 有任务被剔除时，原始任务数才是调用方关心的分母
        out["submitted_count"] = len(tasks) if isinstance(tasks, (list, tuple)) else None
    return sanitize(out)


def plan_from_dir(
    input_dir: str,
    *,
    task_id: Optional[str] = None,
    limit: Optional[int] = None,
    mode: str = "batch",
    **options,
) -> Dict[str, Any]:
    """
    从数据目录（``config.json`` / ``*_obstacles.json`` / ``aircraft_config.json`` /
    ``tasks.json``）加载并规划

    这是把既有命令行流程「一键接口化」的入口：与 ``plan_single.py`` /
    ``plan_multi.py`` 读同一批文件、走同一条代码路径。

    Args:
        input_dir: 输入目录
        task_id: 只规划指定任务；``limit`` 与它互斥
        limit: 只取前 N 个任务（按文件顺序），便于联调时快速验证
        mode: ``"batch"``（多机批量，默认）或 ``"single"``（取优先级最高的单个任务）
        **options: 透传给 :func:`plan_single` 或 :func:`plan_batch` 的其余参数

    Returns:
        与所选 ``mode`` 对应的返回结构
    """
    if not os.path.isdir(input_dir):
        raise PlanningError(
            ErrorCode.INVALID_INPUT,
            f"输入目录不存在: {input_dir}",
            {"input_dir": input_dir},
        )

    def _path(name: str) -> str:
        return os.path.join(input_dir, name)

    config_path = _path("config.json")
    if not os.path.exists(config_path):
        raise PlanningError(
            ErrorCode.INVALID_INPUT,
            f"缺少配置文件: {config_path}",
            {"expected": config_path},
        )

    cfg = load_config(config_path)
    # 与 plan_single.py / plan_multi.py 一致地走多源融合加载（3.25 模块三）：
    # 统一格式、跨源去重、按 source 优先级占位。用简单加载器会得到不同的
    # 障碍集合，进而规划出不同航线——接口结果必须与 CLI 一致。
    obstacle_set, data_quality = load_obstacle_set_fused(
        [_path("static_obstacles.json")],
        [_path("dynamic_obstacles.json")],
    )
    tasks = load_tasks(_path("tasks.json"))

    if task_id is not None:
        tasks = [t for t in tasks if t.task_id == task_id]
        if not tasks:
            raise PlanningError(
                ErrorCode.TASK_NOT_FOUND,
                f"任务 {task_id} 不存在",
                {"task_id": task_id},
            )
    elif limit is not None:
        tasks = tasks[: int(limit)]

    # plan_batch 需要 dict 形式的 tasks / aircraft；这里从已加载的
    # dataclass 反向取原始文件内容，避免为了调用 SDK 而重造一份数据。
    import json

    with open(_path("tasks.json"), encoding="utf-8") as f:
        raw_tasks = json.load(f)
    with open(_path("aircraft_config.json"), encoding="utf-8") as f:
        raw_aircraft = json.load(f)

    keep = {t.task_id for t in tasks}
    raw_items = raw_tasks.get("tasks", raw_tasks) if isinstance(raw_tasks, dict) else raw_tasks
    raw_items = [it for it in raw_items if it.get("id", it.get("task_id")) in keep]

    config_dict = {
        k: getattr(cfg, k)
        for k in ("x_min", "x_max", "y_min", "y_max", "z_min", "z_max",
                  "dx", "dy", "dz", "t_min", "t_max", "dt")
    }
    static_dicts = [
        {"id": o.obstacle_id, "name": o.name, "type": o.obstacle_type,
         "x": o.x, "y": o.y, "z": o.z,
         "width": o.width, "depth": o.depth, "height": o.height,
         "source": o.source}
        for o in obstacle_set.static_obstacles
    ]
    dynamic_dicts = [
        {"id": o.obstacle_id, "name": o.name, "type": o.obstacle_type,
         "safety_radius": o.safety_radius, "source": o.source,
         "trajectory": [
             {"x": px, "y": py, "z": pz, "t": pt} for px, py, pz, pt in o.trajectory
         ]}
        for o in obstacle_set.dynamic_obstacles
    ]

    if mode == "single":
        if not tasks:
            raise PlanningError(ErrorCode.TASK_NOT_FOUND, "没有可规划的任务", {})
        # 与 plan_single.py 一致：按业务优先级 + 调度优先级选最高
        task = max(tasks, key=lambda t: (t.requirements.priority_level, t.priority))
        perf = None
        aircraft_items = (
            raw_aircraft.get("aircraft", raw_aircraft)
            if isinstance(raw_aircraft, dict) else raw_aircraft
        )
        for item in aircraft_items:
            if item.get("id", item.get("aircraft_id")) == task.aircraft_id:
                perf = item
                break
        # 传完整航点而非起终点：中间经停点丢了会得到一条更短但**不合要求**
        # 的航线，而调用方从结果上看不出来（里程只差几米）。
        raw_task = next(
            (it for it in raw_items
             if it.get("id", it.get("task_id")) == task.task_id),
            None,
        )
        if not raw_task or not raw_task.get("waypoints"):
            raise PlanningError(
                ErrorCode.TASK_NOT_FOUND,
                f"任务 {task.task_id} 缺少航点定义",
                {"task_id": task.task_id},
            )
        options.setdefault("max_iterations", None)
        outcome = plan_single(
            waypoints=raw_task["waypoints"],
            config=config_dict,
            static_obstacles=static_dicts,
            dynamic_obstacles=dynamic_dicts,
            aircraft=perf,
            start_time=task.departure_time,
            priority=task.priority,
            objective_weights=task.objective_weights,
            task_id=task.task_id,
            **options,
        )
        outcome["data_quality"] = data_quality
        return outcome

    options.setdefault("max_iterations", None)
    # 与 plan_multi.py CLI 对齐：CLI 会先用 validate_tasks_with_aircraft 剔除
    # 与飞行器需求不兼容的任务再规划。这里默认保持一致，使同一批文件
    # 经 CLI 与经接口调用的结果相同；被剔除的任务在 warnings 中如实列出。
    options.setdefault("validate_tasks", True)
    outcome = plan_batch(
        raw_items,
        aircraft=raw_aircraft,
        config=config_dict,
        static_obstacles=static_dicts,
        dynamic_obstacles=dynamic_dicts,
        **options,
    )
    outcome["data_quality"] = data_quality
    return outcome


def validate_route(
    route: dict,
    *,
    aircraft: Optional[dict] = None,
    static_obstacles=(),
    dynamic_obstacles=(),
    min_separation: float = 10.0,
) -> Dict[str, Any]:
    """
    校验一条已有航线的合法性（连续性 / 障碍 / 性能 / 高度 / 时空占用）

    用于**校验外部系统给出的航线**，而不只是校验本算法自己产出的航线——
    这正是联调场景里对方最需要的能力。

    Args:
        route: 航线结构，至少含 ``trajectory``：
               ``[{"x":..,"y":..,"z":..,"t":..}, ...]``；可选 ``route_id`` /
               ``aircraft_id`` / ``total_distance`` / ``total_time``
        aircraft: 飞行器性能，用于速度/爬升/高度约束检查
        static_obstacles / dynamic_obstacles: 障碍物
        min_separation: 多机最小间隔（仅多航线校验时生效）

    Returns:
        见 ``docs/API.md`` §3.4（验证结果）
    """
    if not isinstance(route, dict):
        raise PlanningError(
            ErrorCode.INVALID_INPUT,
            f"route 必须是对象，实际 {type(route).__name__}",
            {"type": type(route).__name__},
        )
    traj = route.get("trajectory")
    if not isinstance(traj, (list, tuple)):
        raise PlanningError(
            ErrorCode.INVALID_INPUT,
            "route.trajectory 必须是数组",
            {"got": type(traj).__name__},
        )

    from src.models.task import PlannedRoute, TrajectoryPoint

    points = []
    for i, p in enumerate(traj):
        if not isinstance(p, dict):
            raise PlanningError(
                ErrorCode.INVALID_INPUT,
                f"route.trajectory[{i}] 必须是对象",
                {"index": i, "got": type(p).__name__},
            )
        missing = [k for k in ("x", "y", "z", "t") if k not in p]
        if missing:
            raise PlanningError(
                ErrorCode.INVALID_INPUT,
                f"route.trajectory[{i}] 缺少字段: {', '.join(missing)}",
                {"index": i, "missing": missing},
            )
        points.append(TrajectoryPoint(
            x=float(p["x"]), y=float(p["y"]), z=float(p["z"]), t=float(p["t"]),
            heading=float(p.get("heading", 0.0)),
        ))

    planned = PlannedRoute(
        route_id=str(route.get("route_id", "external-route")),
        task_id=str(route.get("task_id", "external-task")),
        aircraft_id=str(route.get("aircraft_id", "external-uav")),
        trajectory=points,
        status=str(route.get("status", "planned")),
    )
    # 允许调用方带上自报的里程/时长，用于与轨迹实际长度对比
    planned.total_distance = float(route.get("total_distance", 0.0) or 0.0)
    planned.total_time = float(route.get("total_time", 0.0) or 0.0)

    obstacle_set = build_obstacle_set(static_obstacles, dynamic_obstacles)
    perf = parse_aircraft(aircraft) if aircraft is not None else None

    validator = PathValidator(
        obstacle_set=obstacle_set if (
            obstacle_set.static_obstacles or obstacle_set.dynamic_obstacles
        ) else None,
        min_separation=float(min_separation),
    )
    t0 = time.time()
    result = validator.validate_route(planned, performance=perf)
    elapsed_ms = (time.time() - t0) * 1000

    out = validation_to_dict(result)
    out["route_id"] = planned.route_id
    out["elapsed_ms"] = round(elapsed_ms, 2)
    return sanitize(out)


def get_version() -> Dict[str, Any]:
    """返回接口版本、封装的算法组件与默认参数（供对方确认对接版本）"""
    return {
        "api_version": __version__,
        "algorithms": dict(ALGORITHMS),
        "defaults": {
            "config": dict(DEFAULT_CONFIG),
            "aircraft": {
                "max_speed_mps": DEFAULT_AIRCRAFT.get("max_speed_mps", 15.0),
                "model": DEFAULT_AIRCRAFT.get("model"),
            },
            "grid_cells": (
                int((DEFAULT_CONFIG["x_max"] - DEFAULT_CONFIG["x_min"]) / DEFAULT_CONFIG["dx"])
                * int((DEFAULT_CONFIG["y_max"] - DEFAULT_CONFIG["y_min"]) / DEFAULT_CONFIG["dy"])
                * int((DEFAULT_CONFIG["z_max"] - DEFAULT_CONFIG["z_min"]) / DEFAULT_CONFIG["dz"])
                * int((DEFAULT_CONFIG["t_max"] - DEFAULT_CONFIG["t_min"]) / DEFAULT_CONFIG["dt"])
            ),
        },
    }
