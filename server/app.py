"""FastAPI service for current geographic planning and asynchronous jobs."""
from __future__ import annotations

from asyncio import to_thread
from contextlib import asynccontextmanager

from fastapi import FastAPI, Request
from fastapi.exceptions import RequestValidationError
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

from uav_planner import ErrorCode, PlanningError
from uav_planner.v2.native import NativeUnavailableError

from .v2 import router as v2_router


_ERROR_STATUS: dict[str, int] = {
    ErrorCode.INVALID_INPUT: 400,
    ErrorCode.TASK_NOT_FOUND: 404,
    ErrorCode.AIRCRAFT_NOT_FOUND: 404,
    ErrorCode.PLANNING_FAILED: 422,
    ErrorCode.TIMEOUT: 504,
    ErrorCode.INTERNAL_ERROR: 500,
    ErrorCode.JOB_QUEUE_FULL: 429,
    ErrorCode.RESOURCE_EXHAUSTED: 503,
}


@asynccontextmanager
async def _lifespan(application: FastAPI):
    from .v2 import jobs

    jobs.start()
    try:
        yield
    finally:
        await to_thread(jobs.shutdown, wait=True)


app = FastAPI(
    title="低空无人机路径规划算法接口",
    version="2.0.0",
    lifespan=_lifespan,
    description=(
        "WGS84/WKT 地理坐标路径规划、连续时空避碰与航线验证。\n\n"
        "- 场景登记：`POST /api/v2/scenes`\n"
        "- 单任务规划：`POST /api/v2/plan/single`\n"
        "- 多机批量规划：`POST /api/v2/plan/batch`\n"
        "- 航线合法性校验：`POST /api/v2/validate`\n"
        "- 异步作业：`POST /api/v2/jobs` + `GET /api/v2/jobs/{job_id}`\n\n"
        "字段含义、单位与调用示例见仓库 `docs/API.md`。"
    ),
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=False,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.exception_handler(PlanningError)
async def _planning_error_handler(request: Request, exc: PlanningError):
    status = _ERROR_STATUS.get(exc.code, 400)
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



app.include_router(v2_router)
