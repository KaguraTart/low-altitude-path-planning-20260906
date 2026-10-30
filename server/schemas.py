"""HTTP contracts for the current WGS84/WKT planning service."""
from __future__ import annotations

from typing import Literal

from pydantic import Field, JsonValue, model_validator

from uav_planner.v2.models import (
    AircraftProfile, BatchResult, ExecutionOptions, MotionSegment, Number, Positive, RouteValidation,
    RuleReport, Scene, StrictModel, Task, WKT, Timestamp, NetworkRoute,
)


class ErrorBody(StrictModel):
    code: str
    message: str
    detail: dict[str,JsonValue] = Field(default_factory=dict)


class ErrorEnvelope(StrictModel):
    error: ErrorBody


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
    time_budget_s: Positive | None = None


class BatchRequest(SceneSelection):
    execution: ExecutionOptions = Field(default_factory=ExecutionOptions)
    tasks: list[Task] = Field(min_length=1)
    request_id: str | None = None
    time_budget_s: Positive | None = None


class ValidateRequest(SceneSelection):
    initial_velocity_xyz_mps: tuple[Number, Number, Number] = (0, 0, 0)
    network_route: NetworkRoute | None = None
    task: Task
    trajectory_wkt: WKT
    vertex_times_s: list[Number] = Field(min_length=1)
    motion_segments: list[MotionSegment] | None = None


class ValidationResponse(RouteValidation):
    rule_report: list[RuleReport]


class SceneResponse(StrictModel):
    network_version: str | None = None
    scene_type: Literal["LOW_ALTITUDE"] = "LOW_ALTITUDE"
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
    incremental_replanning_supported: bool
    initial_velocity_validation_supported: bool
    replanning_engine_ids: list[Literal["ENG-07"]]
    objective_optimization_supported: bool
    objective_dimensions: list[Literal["distance", "time", "energy", "risk"]]
    cost_layer_bases: list[Literal["distance", "time"]]
    search_algorithms: list[Literal["ASTAR", "DIJKSTRA"]]
    grid_neighbors: list[Literal[6, 18, 26]]
    airway_graph_supported: bool
    network_analysis_supported: bool
    traversed_grids_supported: bool
    batch_execution_supported: bool
    contract_version: str
    engine_rules: dict[str,list[str]]
    canonical_alt_reference: Literal["EG"]
    api_version: str
    route_modes: list[str]
    default_route_mode: str
    rule_types: list[str]
    altitude_references: list[str]
    geometric_tolerance_m: Number
    max_scene_radius_m: Number | None
    grid_formats: list[str]
    scene_lifecycle: str
    scene_limits: dict[str,int | None]
    max_registered_scenes: int | None
    max_registered_scene_bytes: int | None


class NetworkAnalyzeRequest(SceneSelection):
    at: Timestamp | None = None


class NetworkAnalysis(StrictModel):
    version: str
    nodes: list[str]
    active_edges: list[str]
    weak_components: list[list[str]]
    strong_components: list[list[str]]
    inactive_edge_ids: list[str]


from uav_planner.v2.replanning import ReplanRequest, ReplanResponse
