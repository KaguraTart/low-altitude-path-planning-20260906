"""
外部入参 → 内部数据模型 的适配层

职责：把调用方传来的**纯 JSON 结构**（dict / list / 数字）翻译成 ``src.models``
下的 dataclass，并在翻译过程中做参数校验，尽早抛出带错误码的
:class:`~uav_planner.errors.PlanningError`。

字段映射**不在此处重复实现**——统一复用 ``src.io.input_loader`` 中已存在的
``parse_*_from_dict`` 函数（那些函数同时服务于文件加载路径），
保证「从文件读」和「从接口传」两条路的 schema 永远一致。
"""

from __future__ import annotations

from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple, Union

from src.io.input_loader import (
    parse_aircraft_from_dict,
    parse_dynamic_obstacle_from_dict,
    parse_static_obstacle_from_dict,
    parse_task_from_dict,
    parse_waypoint_from_dict,
)
from src.models.aircraft import AircraftPerformance, FlightTask, Waypoint
from src.models.obstacles import DynamicObstacle, ObstacleSet, StaticObstacle
from src.models.space import SpaceTimeConfig

from .errors import ErrorCode, PlanningError


# ---------------------------------------------------------------- 默认值

#: 不传 ``config`` 时使用的默认空域，与项目 ``input/config.json`` 保持一致：
#: 5000m × 5000m × 500m，分辨率 100/100/50m，时间窗 0-1200s，步长 10s
#: → 50 × 50 × 10 × 120 = 300 万个时空格
DEFAULT_CONFIG: Dict[str, float] = {
    "x_min": 0.0, "x_max": 5000.0,
    "y_min": 0.0, "y_max": 5000.0,
    "z_min": 0.0, "z_max": 500.0,
    "dx": 100.0, "dy": 100.0, "dz": 50.0,
    "t_min": 0.0, "t_max": 1200.0,
    "dt": 10.0,
}

#: 不传 ``aircraft`` 时的默认飞行器包线（15 m/s 通用四旋翼）
DEFAULT_AIRCRAFT: Dict[str, Any] = {
    "id": "default-uav",
    "model": "generic-quadcopter",
    "category": "general",
}

PointLike = Union[Dict[str, float], Sequence[float]]


# ---------------------------------------------------------------- 基础校验

def _require(condition: bool, message: str, detail: Optional[dict] = None):
    """断言入参合法，否则抛 INVALID_INPUT"""
    if not condition:
        raise PlanningError(ErrorCode.INVALID_INPUT, message, detail)


def parse_point(point: PointLike, field: str) -> Tuple[float, float, float]:
    """
    归一化一个空间点为 ``(x, y, z)``

    同时接受字典 ``{"x":..,"y":..,"z":..}`` 与序列 ``[x, y, z]``，
    单位一律为米。缺字段或非数字会抛 ``INVALID_INPUT``。
    """
    if isinstance(point, dict):
        missing = [k for k in ("x", "y", "z") if k not in point]
        _require(
            not missing,
            f"{field} 缺少字段: {', '.join(missing)}",
            {"field": field, "missing": missing, "got": sorted(point.keys())},
        )
        raw = (point["x"], point["y"], point["z"])
    elif isinstance(point, (list, tuple)):
        _require(
            len(point) == 3,
            f"{field} 需要 3 个分量 [x, y, z]，实际 {len(point)} 个",
            {"field": field, "length": len(point)},
        )
        raw = tuple(point)
    else:
        raise PlanningError(
            ErrorCode.INVALID_INPUT,
            f"{field} 类型不支持，应为 dict 或 [x,y,z]，实际 {type(point).__name__}",
            {"field": field, "type": type(point).__name__},
        )

    try:
        xyz = (float(raw[0]), float(raw[1]), float(raw[2]))
    except (TypeError, ValueError) as e:
        raise PlanningError(
            ErrorCode.INVALID_INPUT,
            f"{field} 分量必须是数字: {e}",
            {"field": field, "got": list(raw)},
        ) from e

    for name, value in zip(("x", "y", "z"), xyz):
        _require(
            value == value and abs(value) != float("inf"),
            f"{field}.{name} 必须是有限数值，实际 {value}",
            {"field": f"{field}.{name}", "got": value},
        )
    return xyz


def parse_config(config: Optional[dict]) -> SpaceTimeConfig:
    """
    构造 :class:`SpaceTimeConfig`；``None`` 时用 :data:`DEFAULT_CONFIG`

    校验分辨率必须为正、范围必须递增——否则网格维度会算出 0 或负数，
    A\* 会静默搜不到任何路径，那种失败很难排查。
    """
    data = dict(DEFAULT_CONFIG if config is None else config)

    _require(isinstance(data, dict), "config 必须是对象", {"got": type(data).__name__})

    for key in ("dx", "dy", "dz", "dt"):
        _require(key in data, f"config 缺少字段 {key}", {"field": key})
        _require(
            float(data[key]) > 0,
            f"config.{key} 必须大于 0，实际 {data[key]}",
            {"field": key, "got": data[key]},
        )

    for axis in ("x", "y", "z", "t"):
        lo, hi = f"{axis}_min", f"{axis}_max"
        _require(lo in data and hi in data, f"config 缺少 {lo} 或 {hi}", {"field": lo})
        _require(
            float(data[lo]) < float(data[hi]),
            f"config.{lo} 必须小于 {hi}，实际 {data[lo]} >= {data[hi]}",
            {"field": lo, "min": data[lo], "max": data[hi]},
        )

    try:
        return SpaceTimeConfig(**{k: float(v) for k, v in data.items()})
    except TypeError as e:
        raise PlanningError(
            ErrorCode.INVALID_INPUT,
            f"config 含未知字段或类型错误: {e}",
            {"got": sorted(data.keys())},
        ) from e


def _materialize(items: Union[None, dict, Iterable[Any]], key: str) -> List[dict]:
    """兼容 ``[{...}]`` 与 ``{"<key>": [{...}]}`` 两种写法"""
    if items is None:
        return []
    if isinstance(items, dict):
        inner = items.get(key, [])
        return list(inner) if isinstance(inner, (list, tuple)) else [inner]
    if isinstance(items, (list, tuple)):
        return list(items)
    raise PlanningError(
        ErrorCode.INVALID_INPUT,
        f"{key} 应为数组或对象，实际 {type(items).__name__}",
        {"field": key, "type": type(items).__name__},
    )


def parse_static_obstacles(items) -> List[StaticObstacle]:
    """解析静态障碍物列表（每项见 API 文档 §2.2）"""
    out = []
    for i, item in enumerate(_materialize(items, "static_obstacles")):
        try:
            out.append(parse_static_obstacle_from_dict(item))
        except (KeyError, TypeError, ValueError) as e:
            raise PlanningError(
                ErrorCode.INVALID_INPUT,
                f"static_obstacles[{i}] 解析失败: {e}",
                {"index": i, "got": item},
            ) from e
    return out


def parse_dynamic_obstacles(items) -> List[DynamicObstacle]:
    """解析动态障碍物列表（每项见 API 文档 §2.3）"""
    out = []
    for i, item in enumerate(_materialize(items, "dynamic_obstacles")):
        try:
            out.append(parse_dynamic_obstacle_from_dict(item))
        except (KeyError, TypeError, ValueError) as e:
            raise PlanningError(
                ErrorCode.INVALID_INPUT,
                f"dynamic_obstacles[{i}] 解析失败: {e}",
                {"index": i, "got": item},
            ) from e
    return out


def build_obstacle_set(static_items, dynamic_items) -> ObstacleSet:
    """组装 :class:`ObstacleSet`（静态 AABB + 动态轨迹）"""
    obstacle_set = ObstacleSet()
    for obs in parse_static_obstacles(static_items):
        obstacle_set.add_static(obs)
    for obs in parse_dynamic_obstacles(dynamic_items):
        obstacle_set.add_dynamic(obs)
    return obstacle_set


def parse_aircraft(aircraft: Optional[dict]) -> AircraftPerformance:
    """解析单个飞行器；``None`` 时用 :data:`DEFAULT_AIRCRAFT`"""
    item = dict(DEFAULT_AIRCRAFT if aircraft is None else aircraft)
    _require(isinstance(item, dict), "aircraft 必须是对象", {"got": type(item).__name__})
    try:
        return parse_aircraft_from_dict(item)
    except (KeyError, TypeError, ValueError) as e:
        raise PlanningError(
            ErrorCode.INVALID_INPUT, f"aircraft 解析失败: {e}", {"got": item}
        ) from e


def parse_aircraft_map(aircraft) -> Dict[str, AircraftPerformance]:
    """
    解析飞行器集合，返回 ``{aircraft_id: AircraftPerformance}``

    接受四种写法：
    - ``None`` → 空表
    - ``{"aircraft": [{...}, ...]}`` → 带键包装（与 ``aircraft_config.json`` 同形）
    - ``{"id": ..., ...}`` → 单个飞行器对象
    - ``{"uav-001": {...}, "uav-002": {...}}`` → 以键为 ID 的字典
    - ``[{...}, {...}]`` → 列表
    """
    if aircraft is None:
        return {}

    # 带键包装：{"aircraft": [...]}。注意要先于「单机对象」判断，
    # 因为配置文件顶层既没有 id 也没有 aircraft_id。
    if isinstance(aircraft, dict) and "aircraft" in aircraft and not (
        "id" in aircraft or "aircraft_id" in aircraft
    ):
        aircraft = aircraft["aircraft"]

    if isinstance(aircraft, dict):
        # 区分「单个飞行器对象」与「以 ID 为键的字典」
        if "id" in aircraft or "aircraft_id" in aircraft:
            perf = parse_aircraft(aircraft)
            return {perf.aircraft_id: perf}
        out: Dict[str, AircraftPerformance] = {}
        for key, item in aircraft.items():
            if not isinstance(item, dict):
                raise PlanningError(
                    ErrorCode.INVALID_INPUT,
                    f"aircraft[{key}] 应是对象，实际 {type(item).__name__}",
                    {"key": key, "type": type(item).__name__},
                )
            merged = dict(item)
            merged.setdefault("id", key)
            perf = parse_aircraft(merged)
            out[perf.aircraft_id] = perf
        return out

    if isinstance(aircraft, (list, tuple)):
        out = {}
        for i, item in enumerate(aircraft):
            try:
                perf = parse_aircraft_from_dict(item)
            except (KeyError, TypeError, ValueError) as e:
                raise PlanningError(
                    ErrorCode.INVALID_INPUT,
                    f"aircraft[{i}] 解析失败: {e}",
                    {"index": i, "got": item},
                ) from e
            out[perf.aircraft_id] = perf
        return out

    raise PlanningError(
        ErrorCode.INVALID_INPUT,
        f"aircraft 应为数组或对象，实际 {type(aircraft).__name__}",
        {"type": type(aircraft).__name__},
    )


def build_task_from_points(
    start: Tuple[float, float, float],
    goal: Tuple[float, float, float],
    *,
    task_id: str = "task-001",
    aircraft_id: str = "default-uav",
    start_time: float = 0.0,
    priority: int = 5,
    objective_weights: Optional[dict] = None,
) -> FlightTask:
    """由起终点两点构造一个最小 :class:`FlightTask`（供 plan_single 使用）"""
    return FlightTask(
        task_id=task_id,
        task_name=task_id,
        aircraft_id=aircraft_id,
        waypoints=[
            Waypoint(wp_id="start", x=start[0], y=start[1], z=start[2], wp_type="start"),
            Waypoint(wp_id="end", x=goal[0], y=goal[1], z=goal[2], wp_type="end"),
        ],
        priority=priority,
        departure_time=start_time,
        objective_weights=objective_weights or {
            "distance": 0.3, "time": 0.3, "risk": 0.2, "energy": 0.2
        },
    )


def parse_waypoints(items) -> List[Waypoint]:
    """解析航点序列（保留 hover_time / 时间窗 / 类型）"""
    out = []
    for i, item in enumerate(_materialize(items, "waypoints")):
        try:
            out.append(parse_waypoint_from_dict(item, i))
        except (KeyError, TypeError, ValueError) as e:
            raise PlanningError(
                ErrorCode.INVALID_INPUT,
                f"waypoints[{i}] 解析失败: {e}",
                {"index": i, "got": item},
            ) from e
    _require(len(out) >= 2, f"waypoints 至少需要起点和终点，实际 {len(out)} 个",
             {"count": len(out)})
    return out


def build_task_from_waypoints(
    waypoints: List[Waypoint],
    *,
    task_id: str = "task-001",
    aircraft_id: str = "default-uav",
    start_time: float = 0.0,
    priority: int = 5,
    objective_weights: Optional[dict] = None,
) -> FlightTask:
    """
    由完整航点序列构造 :class:`FlightTask`

    经停点（``wp_type="via"`` / ``"task"``）会带上各自的 ``hover_time`` 与
    时间窗，规划器按顺序分段规划——这与只传起终点得到的结果**不同**，
    所以调用方给了中间航点就必须走这条路。
    """
    return FlightTask(
        task_id=task_id,
        task_name=task_id,
        aircraft_id=aircraft_id,
        waypoints=list(waypoints),
        priority=priority,
        departure_time=start_time,
        objective_weights=objective_weights or {
            "distance": 0.3, "time": 0.3, "risk": 0.2, "energy": 0.2
        },
    )


def parse_tasks(items) -> List[FlightTask]:
    """解析任务列表（每项见 API 文档 §2.5）"""
    tasks = []
    for i, item in enumerate(_materialize(items, "tasks")):
        try:
            tasks.append(parse_task_from_dict(item))
        except (KeyError, TypeError, ValueError) as e:
            raise PlanningError(
                ErrorCode.INVALID_INPUT,
                f"tasks[{i}] 解析失败: {e}",
                {"index": i, "got": item},
            ) from e
    _require(bool(tasks), "tasks 不能为空", {"count": 0})
    return tasks
