"""Typed geographic API. Sync handlers run outside the ASGI event loop."""
from __future__ import annotations

from datetime import datetime, timezone
from typing import Literal

from fastapi import APIRouter
from pydantic import Field, JsonValue, model_validator
from shapely.geometry import LineString, Point

from uav_planner import ErrorCode, PlanningError
from uav_planner.v2.coordinates import CoordinateFrame, convert_coordinates
from uav_planner.v2.engine import (plan_batch_v2, plan_single_v2, register_scene,
                                   registry, validate_route_v2)
from uav_planner.v2.models import (
    AircraftProfile, BatchResult, CoordinateRequest, CoordinateResponse, Number,
    Positive, RouteResult, RouteValidation, RuleReport, RULE_TYPES, Scene, StrictModel, MotionSegment,
    Task, WKT,
)
from uav_planner.v2 import native
from .jobs import JobRegistry

class ErrorBody(StrictModel):
    code: str
    message: str
    detail: dict[str,JsonValue] = Field(default_factory=dict)


class ErrorEnvelope(StrictModel):
    error: ErrorBody


router=APIRouter(prefix="/api/v2",tags=["v2 WKT / WGS84"],responses={
    "4XX":{"model":ErrorEnvelope,"description":"Invalid input or unavailable resource"},
    400:{"model":ErrorEnvelope,"description":"Input or referenced constraints are invalid"},
    404:{"model":ErrorEnvelope,"description":"Unknown scene or job"},
    429:{"model":ErrorEnvelope,"description":"Job admission or retention budget reached"},
    500:{"model":ErrorEnvelope,"description":"Unexpected service error"},
    503:{"model":ErrorEnvelope,"description":"Native planning engine unavailable"},
})
jobs=JobRegistry(max_workers=1,max_jobs=32)


class SceneSelection(StrictModel):
    scene_id: str | None = None
    scene: Scene | None = None

    @model_validator(mode="after")
    def one_scene(self):
        if (self.scene_id is None)==(self.scene is None):
            raise ValueError("Provide exactly one of scene_id or scene")
        return self


class SingleRequest(SceneSelection):
    task: Task
    time_budget_s: Positive = Field(default=10,le=300)


class BatchRequest(SceneSelection):
    tasks: list[Task] = Field(min_length=1,max_length=10000)
    request_id: str | None = None
    time_budget_s: Positive = Field(default=10,le=300)


class ValidateRequest(SceneSelection):
    task: Task
    trajectory_wkt: WKT
    vertex_times_s: list[Number] = Field(min_length=1,max_length=100000)
    motion_segments: list[MotionSegment] | None = Field(default=None,max_length=100000)


class ValidationResponse(RouteValidation):
    rule_report: list[RuleReport]


class SceneResponse(StrictModel):
    scene_id: str
    scene_version: str
    status: Literal["ready"]
    compilation_ms: Number
    coordinate_context: dict[str,JsonValue]
    warnings: list[JsonValue]
    counts: dict[str,int]


class JobSubmitted(StrictModel):
    job_id: str
    status: Literal["pending"]
    poll_url: str


class JobError(StrictModel):
    code: str
    message: str
    detail: dict[str,JsonValue] = Field(default_factory=dict)


class JobResult(StrictModel):
    job_id: str
    status: Literal["pending","running","succeeded","failed"]
    created_at: str
    started_at: str | None = None
    finished_at: str | None = None
    elapsed_ms: Number | None = None
    result: BatchResult | None = None
    error: JobError | None = None


class HealthResponse(StrictModel):
    status: Literal["ok","degraded"]
    api_version: str
    native_available: bool
    jobs: dict[str,int]


class VersionResponse(StrictModel):
    api_version: str
    engine: dict[str,JsonValue]
    default_profile: AircraftProfile


class Capabilities(StrictModel):
    api_version: str
    route_modes: list[str]
    default_route_mode: str
    rule_types: list[str]
    altitude_references: list[str]
    geometric_tolerance_m: Number
    max_scene_radius_m: Number
    grid_formats: list[str]
    scene_lifecycle: str
    scene_limits: dict[str,int]
    max_registered_scenes: int
    max_registered_scene_bytes: int


@router.get("/health",response_model=HealthResponse)
def health():
    return {"status":"ok" if native.available() else "degraded","api_version":"2.0.0",
            "native_available":native.available(),"jobs":jobs.stats()}


@router.get("/health/jobs")
def health_jobs() -> dict[str,int]:
    return jobs.stats()


@router.get("/version",response_model=VersionResponse)
def version():
    return {"api_version":"2.0.0","engine":native.engine_info(),"default_profile":AircraftProfile()}


@router.get("/capabilities",response_model=Capabilities)
def capabilities():
    from uav_planner.v2.budgets import SCENE_LIMITS
    return {"api_version":"2.0.0","route_modes":["strict","prefer","free"],"default_route_mode":"strict",
            "rule_types":list(RULE_TYPES),"altitude_references":["MSL","AGL","ELLIPSOID","QNE","QNH"],
            "geometric_tolerance_m":.1,"max_scene_radius_m":50000,
            "grid_formats":["regular_sparse_indices","regular_index_ranges","explicit_bounds_cells"],
            "scene_limits":SCENE_LIMITS,"max_registered_scenes":registry.max_scenes,
            "max_registered_scene_bytes":registry.max_bytes,
            "scene_lifecycle":"Process-local immutable scenes; register again after restart; task answers are not cached"}


@router.post("/scenes",response_model=SceneResponse)
def create_scene(request: Scene):
    return register_scene(request)


@router.get("/scenes/{scene_id}",response_model=SceneResponse)
def get_scene(scene_id: str):
    return registry.describe(scene_id)


@router.post("/coordinates/convert",response_model=CoordinateResponse)
def coordinates(request: CoordinateRequest):
    return convert_coordinates(request)


@router.post("/plan/single",response_model=RouteResult)
def single(request: SingleRequest):
    """WKT single flight. route_mode defaults to strict; all heights are explicit."""
    return plan_single_v2(**request.model_dump(exclude_none=True))


@router.post("/plan/batch",response_model=BatchResult)
def batch(request: BatchRequest):
    return plan_batch_v2(**request.model_dump(exclude_none=True))


@router.post("/validate",response_model=ValidationResponse)
def validate(request: ValidateRequest):
    return validate_route_v2(**request.model_dump(exclude_none=True))


@router.post("/jobs",response_model=JobSubmitted,status_code=202)
def submit(request: BatchRequest):
    job_id=jobs.submit(plan_batch_v2,**request.model_dump(exclude_none=True))
    return {"job_id":job_id,"status":"pending","poll_url":f"/api/v2/jobs/{job_id}"}


@router.get("/jobs/{job_id}",response_model=JobResult)
def job(job_id: str):
    result=jobs.get(job_id)
    if result is None:
        raise PlanningError(ErrorCode.TASK_NOT_FOUND,"Unknown or expired v2 job",{"job_id":job_id})
    return result


def example_request() -> dict:
    context={"origin_lon":104.77,"origin_lat":29.33,"terrain_msl_m":0,"terrain_source":"example-flat-terrain-v1"}
    frame=CoordinateFrame(context)
    start,goal=Point(100,100,80),Point(1200,100,80)
    line=frame.to_wgs84(LineString([start,goal]))
    def grid(point):
        lon,lat,_=frame._inverse.transform(point.x,point.y,point.z)
        return {"minLon":lon-.0001,"maxLon":lon+.0001,"minLat":lat-.0001,"maxLat":lat+.0001,"minAlt":50,"maxAlt":150}
    return {"scene":{"coordinates":context,"bounds":{"x_min":0,"x_max":1500,"y_min":0,"y_max":500,"z_min":0,"z_max":300},
                     "time_origin":"2026-10-16T09:00:00+08:00","resolution":[25,25,10],"horizon_s":3600,
                     "airways":[{"id":"demo-airway","centerLine":line,"widthMeters":100,"minAltitude":50,"maxAltitude":150,
                                 "altReference":"MSL","startGrid":grid(start),"endGrid":grid(goal)}]},
            "task":{"id":"demo-v2-001","start_wkt":frame.to_wgs84(start),"goal_wkt":frame.to_wgs84(goal),
                    "departure_time":"2026-10-16T09:00:00+08:00","alt_reference":"MSL","route_mode":"strict"}}


@router.get("/examples/single",response_model=SingleRequest)
def example_single():
    return example_request()


@router.get("/examples/batch",response_model=BatchRequest)
def example_batch():
    example=example_request()
    task=example.pop("task")
    second=dict(task,id="demo-v2-002")
    return {**example,"request_id":"demo-v2-batch","tasks":[task,second]}
