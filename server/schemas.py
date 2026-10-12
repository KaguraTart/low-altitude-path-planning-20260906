"""
HTTP 请求/响应模型（Pydantic v2）

**职责边界**：这些模型只做「外层形状」校验（字段是否存在、类型是否对），
用来生成 Swagger 文档、让联调方在 `/docs` 上直接看到完整字段表。

**语义校验不在这里重复实现**——取值范围、互斥关系、权重和是否为 1、
障碍物是否合法等，统一由 ``uav_planner`` SDK 判定并抛出带错误码的
``PlanningError``。两处各写一份校验必然会随版本漂移，最终以哪一处为准
都说不清；所以这里只保证「能被解析成对象」，其余交给唯一的事实来源。

字段含义与单位见 ``docs/API_INTERFACE.md``。
"""

from __future__ import annotations

import warnings
from typing import Any, Dict, List, Optional, Union

from pydantic import BaseModel, ConfigDict, Field

# PlanSingleRequest.validate 会与 Pydantic 基类上的同名属性重名，Pydantic 因此
# 发出 UserWarning。这里保留字段名 `validate` 并定向静音：接口字段名必须与
# SDK 函数参数逐字一致，否则对接方要在两套名字之间做映射，得不偿失。
warnings.filterwarnings(
    "ignore",
    message=r'Field name "validate" .* shadows an attribute',
    category=UserWarning,
)


# ---------------------------------------------------------------- 基础结构

class Point(BaseModel):
    """空间点，单位米"""
    model_config = ConfigDict(extra="forbid")

    x: float = Field(..., description="X 坐标（米）")
    y: float = Field(..., description="Y 坐标（米）")
    z: float = Field(..., description="Z 坐标（米）")


class TrajectoryPoint(BaseModel):
    """轨迹点：位置 + 时间戳"""
    x: float = Field(..., description="X 坐标（米）")
    y: float = Field(..., description="Y 坐标（米）")
    z: float = Field(..., description="Z 坐标（米）")
    t: float = Field(..., description="时间（秒）")
    heading: float = Field(0.0, description="航向角（度），可选")


class SpaceTimeConfig(BaseModel):
    """时空网格配置：定义空域范围、分辨率与时间窗"""

    x_min: float = Field(0.0, description="X 最小值（米）")
    x_max: float = Field(5000.0, description="X 最大值（米）")
    y_min: float = Field(0.0, description="Y 最小值（米）")
    y_max: float = Field(5000.0, description="Y 最大值（米）")
    z_min: float = Field(0.0, description="Z 最小值（米，地面）")
    z_max: float = Field(500.0, description="Z 最大值（米，空域顶）")
    dx: float = Field(100.0, description="X 方向分辨率（米/格），须 > 0")
    dy: float = Field(100.0, description="Y 方向分辨率（米/格），须 > 0")
    dz: float = Field(50.0, description="Z 方向分辨率（米/格），须 > 0")
    t_min: float = Field(0.0, description="规划窗口起点（秒）")
    t_max: float = Field(1200.0, description="规划窗口终点（秒）")
    dt: float = Field(10.0, description="时间步长（秒/步），须 > 0")


class StaticObstacle(BaseModel):
    """静态障碍物：AABB 长方体包围盒"""
    model_config = ConfigDict(extra="forbid")

    id: str = Field(..., description="障碍物唯一 ID")
    name: str = Field("unnamed", description="显示名")
    type: str = Field("other", description="building / tower / terrain / other")
    x: float = Field(..., description="中心 X（米）")
    y: float = Field(..., description="中心 Y（米）")
    z: float = Field(0.0, description="底部 Z（米）")
    width: float = Field(20.0, description="X 方向半宽（米）")
    depth: float = Field(20.0, description="Y 方向半深（米）")
    height: float = Field(50.0, description="高度（米，自 z 向上）")
    source: str = Field("api", description="数据源标识")


class DynamicObstacle(BaseModel):
    """动态障碍物：轨迹点线性插值 + 安全半径"""
    model_config = ConfigDict(extra="forbid")

    id: str = Field(..., description="障碍物唯一 ID")
    name: str = Field("unnamed", description="显示名")
    type: str = Field("other", description="uav / vehicle / bird_flock / other")
    safety_radius: float = Field(10.0, description="安全避让半径（米）")
    source: str = Field("api", description="数据源标识")
    trajectory: List[TrajectoryPoint] = Field(..., description="轨迹点列表（按时间递增）")


class Waypoint(BaseModel):
    """航点"""
    model_config = ConfigDict(extra="forbid")

    id: Optional[str] = Field(None, description="航点 ID，缺省自动编号")
    x: float = Field(..., description="X 坐标（米）")
    y: float = Field(..., description="Y 坐标（米）")
    z: float = Field(..., description="Z 坐标（米）")
    type: str = Field("via", description="start / via / end / task")
    earliest_arrival: Optional[float] = Field(None, description="最早到达时间（秒）")
    latest_arrival: Optional[float] = Field(None, description="最晚到达时间（秒）")
    hover_time: float = Field(0.0, description="悬停时间（秒）")
    task_description: str = Field("", description="任务描述")


class Task(BaseModel):
    """飞行任务"""
    model_config = ConfigDict(extra="forbid")

    id: str = Field(..., description="任务唯一 ID")
    name: Optional[str] = Field(None, description="任务名")
    aircraft_id: str = Field(..., description="执行该任务的飞行器 ID，必须存在于 aircraft")
    type: str = Field("other", description="delivery / inspection / patrol / surveillance / other")
    priority: int = Field(5, description="调度优先级 1-10（10 最高）")
    departure_time: float = Field(0.0, description="期望起飞时间（秒）")
    objective_weights: Optional[Dict[str, float]] = Field(
        None, description="多目标权重 {distance,time,risk,energy}，和须为 1.0"
    )
    requirements: Optional[Dict[str, Any]] = Field(
        None, description="任务需求约束 {payload_kg, required_endurance_s, ...}"
    )
    waypoints: List[Waypoint] = Field(..., description="航点序列（至少起点和终点）")


class RouteInput(BaseModel):
    """待校验的航线（可由外部系统给出，不必是本算法产出）"""
    model_config = ConfigDict(extra="forbid")

    route_id: str = Field("external-route", description="航线 ID")
    task_id: str = Field("external-task", description="任务 ID")
    aircraft_id: str = Field("external-uav", description="飞行器 ID")
    status: str = Field("planned", description="planned / failed / partial")
    total_distance: float = Field(0.0, description="自报总里程（米），可选")
    total_time: float = Field(0.0, description="自报总时长（秒），可选")
    trajectory: List[TrajectoryPoint] = Field(..., description="轨迹点序列")


# ---------------------------------------------------------------- 请求体

class PlanSingleRequest(BaseModel):
    """POST /api/v1/plan/single 请求体"""

    start: Optional[Point] = Field(None, description="起点；与 waypoints 二选一")
    goal: Optional[Point] = Field(None, description="终点；与 waypoints 二选一")
    waypoints: Optional[List[Waypoint]] = Field(
        None, description="完整航点序列（含中间经停）；与 start/goal 互斥"
    )
    config: Optional[SpaceTimeConfig] = Field(None, description="时空网格配置，缺省用默认值")
    static_obstacles: List[StaticObstacle] = Field(default_factory=list)
    dynamic_obstacles: List[DynamicObstacle] = Field(default_factory=list)
    aircraft: Optional[Dict[str, Any]] = Field(
        None, description="飞行器性能，缺省用 15 m/s 通用四旋翼"
    )
    start_time: float = Field(0.0, description="起飞时刻（秒）")
    priority: int = Field(5, description="调度优先级 1-10")
    objective_weights: Optional[Dict[str, float]] = Field(
        None, description="多目标权重，和须为 1.0"
    )
    optimize: bool = Field(True, description="是否做多目标航线优化")
    cost_grid: bool = Field(True, description="是否启用通行代价层")
    clearance: float = Field(200.0, description="净空膨胀半径（米）")
    clearance_cost: float = Field(3.0, description="净空区通行代价倍率")
    max_iterations: int = Field(2_000_000, description="A* 最大迭代次数")
    validate: bool = Field(True, description="是否附带路径合法性验证")
    task_id: str = Field("task-001", description="任务标识")


class PlanBatchRequest(BaseModel):
    """POST /api/v1/plan/batch 与 POST /api/v1/jobs 请求体"""

    tasks: List[Task] = Field(..., description="任务列表")
    aircraft: Union[List[Dict[str, Any]], Dict[str, Any], None] = Field(
        None, description="飞行器列表，或 {id: {...}} 字典；任务引用的 ID 必须存在"
    )
    config: Optional[SpaceTimeConfig] = None
    static_obstacles: List[StaticObstacle] = Field(default_factory=list)
    dynamic_obstacles: List[DynamicObstacle] = Field(default_factory=list)
    max_workers: int = Field(4, description="并行线程数")
    dispatch: bool = Field(
        False, description="调度模式：跳过 risk/heading 计算（约省 8%），适用于只看可行性"
    )
    conflict_avoidance: bool = Field(True, description="是否做多机冲突避免")
    min_separation: float = Field(10.0, description="多机最小安全间隔（米）")
    cost_grid: bool = Field(True, description="是否启用通行代价层")
    clearance: float = Field(200.0, description="净空膨胀半径（米）")
    clearance_cost: float = Field(3.0, description="净空区通行代价倍率")
    max_iterations: int = Field(200_000, description="单任务 A* 最大迭代次数")
    request_id: Optional[str] = Field(None, description="请求标识")
    validate_tasks: bool = Field(
        False,
        description="是否先做飞行器需求兼容性校验并剔除不兼容任务（"
                    "plan_from_dir 默认开启以与 CLI 对齐，此处默认关闭以如实报告失败）",
    )


class ValidateRequest(BaseModel):
    """POST /api/v1/validate 请求体"""

    route: RouteInput = Field(..., description="待校验的航线")
    aircraft: Optional[Dict[str, Any]] = Field(
        None, description="飞行器性能，用于速度/爬升/高度约束检查"
    )
    static_obstacles: List[StaticObstacle] = Field(default_factory=list)
    dynamic_obstacles: List[DynamicObstacle] = Field(default_factory=list)
    min_separation: float = Field(10.0, description="多机最小间隔（米）")


# ---------------------------------------------------------------- 响应体

class HealthResponse(BaseModel):
    """GET /api/v1/health 响应"""
    status: str = Field(..., description="ok 表示服务就绪")
    service: str = Field(..., description="服务名")
    api_version: str = Field(..., description="接口版本")
    uptime_s: float = Field(..., description="服务已运行时长（秒）")


class JobSubmitResponse(BaseModel):
    """POST /api/v1/jobs 响应"""
    job_id: str = Field(..., description="作业 ID，用于轮询 GET /api/v1/jobs/{job_id}")
    status: str = Field(..., description="pending / running / succeeded / failed")
    poll_url: str = Field(..., description="轮询地址")


class JobStatusResponse(BaseModel):
    """GET /api/v1/jobs/{job_id} 响应"""
    job_id: str
    status: str = Field(..., description="pending / running / succeeded / failed")
    created_at: str = Field(..., description="提交时间（ISO 8601）")
    started_at: Optional[str] = Field(None, description="开始执行时间")
    finished_at: Optional[str] = Field(None, description="结束时间")
    elapsed_ms: Optional[float] = Field(None, description="执行耗时（毫秒）")
    result: Optional[Dict[str, Any]] = Field(None, description="成功时的规划结果")
    error: Optional[Dict[str, Any]] = Field(None, description="失败时的错误信封")
