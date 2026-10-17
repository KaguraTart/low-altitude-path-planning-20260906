"""
REST HTTP 服务（FastAPI）

把 ``uav_planner`` SDK 薄薄包一层，让不会/不想用 Python 的对接方也能调用，
并自带 Swagger 交互文档（``/docs``）供联调时直接在网页上试。

设计约定：
- **请求体字段与 SDK 函数参数逐字一致**，用 ``model_dump()`` 直接展开成
  kwargs，不做二次命名。对接方看文档时只需记住一套名字。
- **错误统一信封** ``{"error": {"code", "message", "detail"}}``，并映射到
  合适的 HTTP 状态码（见 ``_error_status``）。
- **响应保证是合法 JSON**：交给 SDK 的 ``sanitize`` 处理过，不含
  ``NaN`` / ``Infinity`` 字面量，对方的解析器不会因此报错。

启动请用仓库根目录的 ``serve.py``（它会打印局域网地址）。
"""

from __future__ import annotations

import time
from asyncio import gather, to_thread
from contextlib import asynccontextmanager
from typing import Any, Dict

from fastapi import FastAPI, Request
from fastapi.exceptions import RequestValidationError
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

from uav_planner import (
    ErrorCode,
    PlanningError,
    get_version,
    plan_batch,
    plan_single,
    validate_route,
)
from uav_planner.v2.native import NativeUnavailableError

from .jobs import JobRegistry
from .schemas import (
    HealthResponse,
    JobStatusResponse,
    JobSubmitResponse,
    PlanBatchRequest,
    PlanSingleRequest,
    ValidateRequest,
)

# PlanningError 的错误码 → HTTP 状态码
_ERROR_STATUS: Dict[str, int] = {
    ErrorCode.INVALID_INPUT: 400,        # 请求本身有问题，改参数可解决
    ErrorCode.TASK_NOT_FOUND: 404,
    ErrorCode.AIRCRAFT_NOT_FOUND: 404,
    ErrorCode.PLANNING_FAILED: 422,      # 请求合法但无法完成规划
    ErrorCode.TIMEOUT: 504,
    ErrorCode.INTERNAL_ERROR: 500,
    ErrorCode.JOB_QUEUE_FULL: 429,
    ErrorCode.RESOURCE_EXHAUSTED: 503,
}

_STARTED_AT = time.time()
_jobs = JobRegistry(max_workers=2)


@asynccontextmanager
async def _lifespan(application: FastAPI):
    from .v2 import jobs as v2_jobs

    _jobs.start()
    v2_jobs.start()
    try:
        yield
    finally:
        await gather(to_thread(_jobs.shutdown, wait=True), to_thread(v2_jobs.shutdown, wait=True))

# ---------------------------------------------------------------- 安全说明

# 服务**不做认证**。暴露范围请自行控制：
# - 本地开发：绑 127.0.0.1
# - 内网联调：绑 0.0.0.0，但确保在可信网段内
# - 公网：务必在前面加一层边缘鉴权（Cloudflare Access / 反向代理 Basic Auth 等），
#         并限制可用资源。详见 internal/docs/DEPLOYMENT.md

app = FastAPI(
    title="低空无人机路径规划算法接口",
    version="2.0.0",
    lifespan=_lifespan,
    description=(
        "多维时空 A* 路径规划 + 多目标航线优化 + 路径合法性验证。\n\n"
        "- 单任务规划：`POST /api/v1/plan/single`\n"
        "- 多机批量规划：`POST /api/v1/plan/batch`\n"
        "- 航线合法性校验：`POST /api/v1/validate`\n"
        "- 大批量异步作业：`POST /api/v1/jobs` + `GET /api/v1/jobs/{job_id}`\n\n"
        "字段含义、单位与调用示例见仓库 `docs/API.md`。"
    ),
)

# 联调期放开跨域，方便对接方前端直接调。生产部署请收紧为具体来源。
# 本服务不做认证与限流，暴露范围请自行控制，见文件顶部的安全说明。
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=False,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ---------------------------------------------------------------- 异常处理

@app.exception_handler(PlanningError)
async def _planning_error_handler(request: Request, exc: PlanningError):
    status = _ERROR_STATUS.get(exc.code, 400 if request.url.path.startswith("/api/v2/") else 500)
    return JSONResponse(status_code=status, content=exc.to_dict())


@app.exception_handler(RequestValidationError)
async def _validation_error_handler(request: Request, exc: RequestValidationError):
    """把 FastAPI 的校验错误也套进统一信封，对方的错误处理逻辑只需写一套"""
    return JSONResponse(
        status_code=400,
        content={
            "error": {
                "code": ErrorCode.INVALID_INPUT,
                "message": "请求体校验失败",
                "detail": {"errors": [
                    {
                        "field": ".".join(str(p) for p in e.get("loc", [])),
                        "reason": e.get("msg", ""),
                    }
                    for e in exc.errors()
                ]},
            }
        },
    )


@app.exception_handler(NativeUnavailableError)
async def _native_unavailable_handler(request: Request, exc: NativeUnavailableError):
    return JSONResponse(status_code=503, content={
        "error": {"code": ErrorCode.NATIVE_UNAVAILABLE, "message": str(exc), "detail": {}}
    })


@app.exception_handler(MemoryError)
async def _memory_error_handler(request: Request, exc: MemoryError):
    return JSONResponse(status_code=503, content={
        "error": {"code": ErrorCode.RESOURCE_EXHAUSTED,
                  "message": "Memory allocation failed",
                  "detail": {"type": type(exc).__name__}}
    })


@app.exception_handler(Exception)
async def _unhandled_handler(request: Request, exc: Exception):  # pragma: no cover
    return JSONResponse(
        status_code=500,
        content={
            "error": {
                "code": ErrorCode.INTERNAL_ERROR,
                "message": f"服务内部错误: {exc}",
                "detail": {"type": type(exc).__name__},
            }
        },
    )


# ---------------------------------------------------------------- 基础端点

@app.get("/api/v1/health", response_model=HealthResponse, tags=["基础"])
async def health():
    """健康检查：服务是否就绪、已运行多久、异步作业积压情况"""
    return HealthResponse(
        status="ok",
        service="uav-path-planning-api",
        api_version=get_version()["api_version"],
        uptime_s=round(time.time() - _STARTED_AT, 1),
    ).model_dump()


@app.get("/api/v1/version", tags=["基础"])
async def version():
    """接口版本、封装的算法组件与默认参数"""
    return get_version()


@app.get("/api/v1/health/jobs", tags=["基础"])
async def jobs_health():
    """异步作业队列统计（pending/running/succeeded/failed）"""
    return _jobs.stats()


# ---------------------------------------------------------------- 规划端点

@app.post("/api/v1/plan/single", tags=["规划"])
async def api_plan_single(req: PlanSingleRequest) -> Dict[str, Any]:
    """
    单条航线规划：时空 A* → 多目标优化 → 合法性验证

    航线可用 ``start`` + ``goal`` 两点给出，或用 ``waypoints`` 给出完整
    航点序列（含中间经停）。**有中间经停时必须用 waypoints**，只给起终点
    会丢掉经停点，规划出一条更短但不合要求的航线。
    """
    from starlette.concurrency import run_in_threadpool
    return await run_in_threadpool(plan_single, **req.model_dump(exclude_none=True))


@app.post("/api/v1/plan/batch", tags=["规划"])
async def api_plan_batch(req: PlanBatchRequest) -> Dict[str, Any]:
    """
    多任务批量并行规划（同步）

    适合中小批量联调。**大批量请改用** ``POST /api/v1/jobs``：
    实测 10000 个任务约需 437 秒，同步请求会被客户端超时掐断。
    """
    from starlette.concurrency import run_in_threadpool
    return await run_in_threadpool(plan_batch, **req.model_dump(exclude_none=True))


@app.post("/api/v1/validate", tags=["验证"])
async def api_validate(req: ValidateRequest) -> Dict[str, Any]:
    """
    校验一条航线的合法性（连续性 / 障碍 / 性能 / 高度 / 时空占用）

    航线可由外部系统给出，不限于本算法产出——这是联调时验证对方航线的主要手段。
    """
    from starlette.concurrency import run_in_threadpool
    return await run_in_threadpool(validate_route, **req.model_dump(exclude_none=True))


# ---------------------------------------------------------------- 异步作业

@app.post("/api/v1/jobs", response_model=JobSubmitResponse, status_code=202, tags=["异步作业"])
async def submit_job(req: PlanBatchRequest):
    """
    提交批量规划作业，立即返回 ``job_id``（HTTP 202）

    请求体与 ``POST /api/v1/plan/batch`` 完全相同，只是改为后台执行。
    用 ``GET /api/v1/jobs/{job_id}`` 轮询进度与结果。
    """
    kwargs = req.model_dump(exclude_none=True)
    job_id = _jobs.submit(plan_batch, **kwargs)
    return JobSubmitResponse(
        job_id=job_id,
        status="pending",
        poll_url=f"/api/v1/jobs/{job_id}",
    ).model_dump()


@app.get("/api/v1/jobs/{job_id}", response_model=JobStatusResponse, tags=["异步作业"])
async def get_job(job_id: str):
    """
    查询作业状态；``status`` 为 ``succeeded`` 时 ``result`` 里是完整的规划结果

    作业表在进程内存中，服务重启后历史作业不可查；多 worker 部署时轮询
    可能落到没有该作业的进程。联调阶段足够，生产需换持久化存储。
    """
    rec = _jobs.get(job_id)
    if rec is None:
        raise PlanningError(
            ErrorCode.TASK_NOT_FOUND,
            f"作业 {job_id} 不存在（可能已随服务重启丢失，或已被淘汰）",
            {"job_id": job_id},
        )
    return rec


# ---------------------------------------------------------------- 示例

@app.get("/api/v1/examples/single", tags=["示例"])
async def example_single() -> Dict[str, Any]:
    """
    返回一份可直接使用的 ``/api/v1/plan/single`` 示例请求体

    联调起步用：取下来直接 POST 回去就能看到完整响应结构。
    """
    return {
        "task_id": "demo-001",
        "start": {"x": 500, "y": 500, "z": 100},
        "goal": {"x": 2500, "y": 2500, "z": 150},
        "static_obstacles": [
            {
                "id": "building-001",
                "name": "中心大厦",
                "type": "building",
                "x": 1500, "y": 1500, "z": 0,
                "width": 80, "depth": 80, "height": 200,
                "source": "demo",
            }
        ],
        "dynamic_obstacles": [
            {
                "id": "uav-intruder-001",
                "name": "闯入无人机",
                "type": "uav",
                "safety_radius": 15,
                "source": "demo",
                "trajectory": [
                    {"x": 1000, "y": 2000, "z": 100, "t": 0},
                    {"x": 2000, "y": 2000, "z": 100, "t": 60},
                ],
            }
        ],
        "optimize": True,
        "validate": True,
    }


@app.get("/api/v1/examples/batch", tags=["示例"])
async def example_batch() -> Dict[str, Any]:
    """返回一份可直接使用的 ``/api/v1/plan/batch``（或 ``/jobs``）示例请求体"""
    return {
        "request_id": "demo-batch-001",
        "aircraft": [
            {
                "id": "uav-001",
                "model": "DJI-Matrice-300",
                "category": "heavy_payload",
                "performance": {"max_speed_mps": 18.0, "max_climb_rate_mps": 6.0},
                "operational": {"min_altitude_m": 10.0, "max_altitude_m": 300.0},
            },
            {
                "id": "uav-002",
                "model": "DJI-Mavic-3E",
                "category": "inspection",
                "performance": {"max_speed_mps": 15.0},
                "operational": {"min_altitude_m": 10.0, "max_altitude_m": 200.0},
            },
        ],
        "tasks": [
            {
                "id": "task-001",
                "name": "医疗物资配送",
                "aircraft_id": "uav-001",
                "priority": 9,
                "departure_time": 0,
                "waypoints": [
                    {"id": "start", "x": 500, "y": 500, "z": 100, "type": "start"},
                    {"id": "end", "x": 2500, "y": 2500, "z": 150, "type": "end"},
                ],
            },
            {
                "id": "task-002",
                "name": "应急侦察",
                "aircraft_id": "uav-002",
                "priority": 5,
                "departure_time": 0,
                "waypoints": [
                    {"id": "start", "x": 4000, "y": 4000, "z": 100, "type": "start"},
                    {"id": "wp1", "x": 3000, "y": 3000, "z": 120, "type": "via"},
                    {"id": "end", "x": 1000, "y": 1000, "z": 150, "type": "end"},
                ],
            },
        ],
        "max_workers": 4,
        "conflict_avoidance": True,
    }


# Geographic v2 is a separate contract; v1 routes remain available.
from .v2 import router as v2_router
app.include_router(v2_router)
