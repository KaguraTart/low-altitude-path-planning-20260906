"""Public SDK for WGS84/WKT scenes, planning, and continuous route validation.

``plan_single``, ``plan_batch``, and ``validate_route`` use the same geographic
contract as their explicit ``_v2`` names and the HTTP service. Results are JSON
compatible dictionaries; imports defer engine loading until a call is made.
"""
from __future__ import annotations

from typing import Any

from .errors import ErrorCode, PlanningError

__version__ = "2.0.0"


def register_scene(scene: Any) -> dict[str, Any]:
    from .v2.engine import register_scene as implementation
    return implementation(scene)


def plan_single_v2(*, task: Any, scene_id: str | None = None,
                   scene: Any = None, time_budget_s: float | None = None) -> dict[str, Any]:
    from .v2.engine import plan_single_v2 as implementation
    return implementation(task=task, scene_id=scene_id, scene=scene, time_budget_s=time_budget_s)


def plan_batch_v2(*, tasks: Any, scene_id: str | None = None,
                  scene: Any = None, request_id: str | None = None,
                  time_budget_s: float | None = None, execution: Any = None) -> dict[str, Any]:
    from .v2.engine import plan_batch_v2 as implementation
    return implementation(tasks=tasks, scene_id=scene_id, scene=scene,
                          request_id=request_id, time_budget_s=time_budget_s, execution=execution)


def validate_route_v2(*, task: Any, trajectory_wkt: str, vertex_times_s: Any,
                      motion_segments: Any = None, network_route: Any = None, scene_id: str | None = None,
                      scene: Any = None, initial_velocity_xyz_mps: Any = (0, 0, 0)) -> dict[str, Any]:
    from .v2.engine import validate_route_v2 as implementation
    return implementation(task=task, trajectory_wkt=trajectory_wkt,
                          vertex_times_s=vertex_times_s, motion_segments=motion_segments, network_route=network_route,
                          scene_id=scene_id, scene=scene, initial_velocity_xyz_mps=initial_velocity_xyz_mps)


def replan_v2(**kwargs: Any) -> dict[str, Any]:
    from .v2.replanning import replan_v2 as implementation
    return implementation(**kwargs)


def analyze_network_v2(*, scene_id: str | None = None, scene: Any = None,
                       at: Any = None) -> dict[str, Any]:
    from .v2.network import analyze_network_v2 as implementation
    return implementation(scene_id=scene_id, scene=scene, at=at)


def convert_coordinates(request: Any) -> dict[str, Any]:
    from .v2.coordinates import convert_coordinates as implementation
    return implementation(request).model_dump(mode="json")


def get_version() -> dict[str, Any]:
    from .v2.native import engine_info
    from .v2.profiles import default_profile
    return {"api_version": __version__, "engine": engine_info(),
            "default_profile": default_profile().model_dump(mode="json")}


plan_single = plan_single_v2
plan_batch = plan_batch_v2
validate_route = validate_route_v2

__all__ = [
    "register_scene", "plan_single", "plan_batch", "validate_route",
    "plan_single_v2", "plan_batch_v2", "validate_route_v2", "convert_coordinates",
    "replan_v2", "analyze_network_v2", "get_version", "ErrorCode", "PlanningError", "__version__",
]
