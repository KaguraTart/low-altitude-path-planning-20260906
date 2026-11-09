"""Typed WGS84/WKT planning contracts and coordinate conversion helpers."""

from .coordinates import CoordinateError, CoordinateFrame, convert_coordinates
from .models import (
    RULE_TYPES,
    ObjectiveConfig, ObjectiveWeights, ObjectiveNormalization, EnergyModel,
    ObjectiveCostLayer, ObjectiveValue, RouteObjectives, OptimizationReport,
    ExecutionReport,
    ExecutionOptions,
    TraversedGrid,
    SearchReport,
    SearchOptions,
    NetworkRoute,
    NetworkSelection,
    NetworkEdge,
    NetworkNode,
    AirwayGraph,
    AircraftProfile,
    AircraftState,
    Airway,
    BatchResult,
    Bounds3D,
    Connector,
    ControlZone,
    CoordinateRequest,
    CoordinateResponse,
    CoordinateSpec,
    Dataset,
    EvaluationMetrics,
    EvaluationResult,
    FieldCell,
    GridSpec,
    HardConstraintCheck,
    HeightCorrection,
    MetricValue,
    PlanCandidate,
    PlanComparison,
    RouteResult,
    Rule,
    SafetyEvaluation,
    Scene,
    Task,
    TimedTrajectory,
    Volume,
    Waypoint,
)
from .profiles import DJI_MINI_4_PRO, default_profile, resolve_profile

__all__ = [
    "ObjectiveConfig", "ObjectiveWeights", "ObjectiveNormalization", "EnergyModel",
    "ObjectiveCostLayer", "ObjectiveValue", "RouteObjectives", "OptimizationReport",
    "AirwayGraph",
    "NetworkNode",
    "NetworkEdge",
    "NetworkSelection",
    "NetworkRoute",
    "SearchOptions",
    "SearchReport",
    "TraversedGrid",
    "ExecutionOptions",
    "ExecutionReport",
    "RULE_TYPES", "AircraftProfile", "AircraftState", "Airway", "BatchResult",
    "Bounds3D", "Connector", "ControlZone", "CoordinateError", "CoordinateFrame",
    "CoordinateRequest", "CoordinateResponse", "CoordinateSpec", "Dataset",
    "DJI_MINI_4_PRO", "EvaluationMetrics", "EvaluationResult", "FieldCell",
    "GridSpec", "HardConstraintCheck", "HeightCorrection", "MetricValue",
    "PlanCandidate", "PlanComparison", "RouteResult", "Rule", "SafetyEvaluation",
    "Scene", "Task", "TimedTrajectory", "Volume", "Waypoint",
    "convert_coordinates", "default_profile", "resolve_profile",
]

from .replanning import CurrentState, ReplanEvent, ReplanTrigger, ReplanRequest, ReplanResponse, replan_v2
__all__ += ["CurrentState", "ReplanEvent", "ReplanTrigger", "ReplanRequest", "ReplanResponse", "replan_v2"]

from .evaluation import compare_plans_v2, evaluate_v2
__all__ += ["evaluate_v2", "compare_plans_v2"]
