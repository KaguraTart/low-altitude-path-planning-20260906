"""ENG-10 path safety validation and multi-plan evaluation.

Hard-constraint judgement is delegated to :func:`engine.validate_route_v2`, so
evaluation can never disagree with planning about whether a route is legal.
This module adds three things on top of it: per-constraint results with the
offending point, threshold and reason; metrics that mark missing inputs as
unavailable instead of zero; and a deterministic ranking of candidate plans.

Unavailable metrics are never substituted with zero. A plan whose hard
constraints fail, or whose positively weighted metrics are incomplete, is
reported as not rankable rather than being scored on partial data.
"""
from __future__ import annotations

import math
from datetime import datetime, timezone

import numpy as np

from uav_planner.errors import ErrorCode, PlanningError

from . import engine, objectives as objective_module
from .models import (
    EvaluationMetrics, EvaluationResult, HardConstraintCheck, MetricValue,
    ObjectiveConfig, ObjectiveNormalization, ObjectiveWeights, PlanComparison,
    SafetyEvaluation, Task, Violation,
)

AIRCRAFT_CODES = {"AIRCRAFT_SPEED", "AIRCRAFT_CLIMB", "AIRCRAFT_TURN", "AIRCRAFT_HOVER"}
_RULE_THRESHOLD_KEYS = {
    "NO_FLY": ("noFlyMode", "heightCeiling", "includeBuffer"),
    "ZONE": ("zoneLevels", "coverageThreshold"),
    "HEIGHT_LAYER": ("minHeight", "maxHeight", "minAltitude", "maxAltitude", "altDatum"),
    "SEPARATION": ("horizontalSeparation", "verticalSeparation"),
    "ENDURANCE": ("maxRange", "minEndurance"),
    "PAYLOAD": ("maxPayloadKg",),
    "SPEED": ("minSpeed", "maxSpeed", "cruiseSpeed"),
    "CLIMB": ("maxClimbRate", "maxDescentRate"),
    "TURN": ("maxTurnAngle", "minTurnRadius"),
    "WEATHER": ("maxWindSpeed", "noPrecipitation", "noIcing", "minVisibility"),
    "SIGNAL": ("minSignalStrength", "bands", "band_policy"),
}
_EPS = 1e-9


def _get(item, key, default=None):
    return item.get(key, default) if isinstance(item, dict) else getattr(item, key, default)


def _metric(value: float | None, unit: str, reason: str | None = None) -> dict:
    available = value is not None
    return {"value": None if value is None else float(value), "unit": unit,
            "available": available, "reason": None if available else reason}


def _unavailable(unit: str, reason: str) -> dict:
    return _metric(None, unit, reason)


def _scene_context(scene_id, scene):
    if (scene_id is None) == (scene is None):
        raise PlanningError(ErrorCode.INVALID_INPUT, "Provide exactly one of scene_id or scene")
    if scene is not None:
        scene_id = engine.register_scene(scene)["scene_id"]
    model, compiled, version, _ = engine.registry.get(scene_id)
    return scene_id, model, compiled, version


def _rule_index(compiled):
    return {str(rule.id): rule for rule in list(compiled.rules.items) + list(compiled.rules.defaults)}


def _rule_threshold(rule) -> dict:
    content = getattr(rule, "content", None) or {}
    keys = _RULE_THRESHOLD_KEYS.get(str(getattr(rule, "rule_type", "")), tuple(content))
    return {key: content[key] for key in keys if key in content}


def _point_at(compiled, model, task, points, times, violation) -> tuple[str | None, float | None]:
    """Map a violation's segment or time to an exported WGS84 point."""
    details = violation.get("details") or {}
    second = violation.get("time_s")
    if second is None and details.get("time_s") is not None:
        second = details["time_s"]
    index = details.get("segment")
    if index is None and second is not None:
        index = int(np.argmin(np.abs(np.asarray(times, float) - float(second))))
    if index is None:
        return None, None if second is None else float(second)
    index = int(index)
    if not 0 <= index < len(points):
        return None, None if second is None else float(second)
    moment = float(times[index])
    wkt = engine._timed_geographic(compiled, points[index:index + 1], [moment],
                                   task.alt_reference, model)
    return wkt, moment


def _segment_actual(points, times, index) -> dict:
    """Physical quantities of one travelled segment, used as the actual value."""
    points = np.asarray(points, float)
    times = np.asarray(times, float)
    if index is None or not 0 <= index < len(points) - 1:
        return {}
    delta = points[index + 1] - points[index]
    duration = float(times[index + 1] - times[index])
    horizontal = float(np.hypot(delta[0], delta[1]))
    speed = float(np.linalg.norm(delta) / duration) if duration > _EPS else math.inf
    climb = float(delta[2] / duration) if duration > _EPS else math.inf
    actual = {"speed_mps": speed, "climb_speed_mps": climb,
              "descent_speed_mps": max(0.0, -climb), "length_m": float(np.linalg.norm(delta)),
              "altitude_m": float(points[index][2])}
    if index >= 1:
        previous = points[index] - points[index - 1]
        if np.linalg.norm(previous) > _EPS and horizontal > _EPS:
            cross = abs(previous[0] * delta[1] - previous[1] * delta[0])
            a, b = float(np.linalg.norm(previous)), horizontal
            c = float(np.hypot(*(points[index + 1] - points[index - 1])[:2]))
            actual["turn_radius_m"] = 0.0 if cross <= 1e-12 * a * b else a * b * c / (2 * cross)
    return actual


def _aircraft_threshold(compiled, task, profile) -> dict:
    limits = engine._limits(compiled, task, profile)
    return {key: limits[key] for key in
            ("max_speed_mps", "climb_speed_mps", "descent_speed_mps",
             "min_turn_radius_m", "max_turn_angle_deg") if key in limits}


def _check(report, compiled, model, task, points, times, violations, thresholds) -> dict:
    rule_id = str(report["rule_id"])
    matching = [v for v in violations if str(v.get("rule_id")) == rule_id]
    point, moment = (None, None)
    if matching:
        point, moment = _point_at(compiled, model, task, points, times, matching[0])
    actual = {}
    if matching:
        actual = _segment_actual(points, times, (matching[0].get("details") or {}).get("segment"))
    return {
        "code": report["rule_type"],
        "category": "rule",
        "rule_type": report["rule_type"],
        "rule_id": rule_id,
        "status": report["status"],
        "point_wkt": point,
        "time_s": moment,
        "threshold": thresholds.get(rule_id, {}),
        "actual": actual,
        "reason": report.get("message", ""),
    }


def _unmatched_checks(compiled, model, task, points, times, violations, profile) -> list[dict]:
    """Aircraft, motion and separation failures that carry no rule id."""
    grouped: dict[str, list[dict]] = {}
    for item in violations:
        if item.get("rule_id"):
            continue
        grouped.setdefault(str(item.get("code", "CONSTRAINT")), []).append(item)
    limits = _aircraft_threshold(compiled, task, profile)
    checks = []
    for code, items in grouped.items():
        first = items[0]
        point, moment = _point_at(compiled, model, task, points, times, first)
        index = (first.get("details") or {}).get("segment")
        category = ("aircraft" if code in AIRCRAFT_CODES else
                    "motion" if code.startswith("MOTION") else
                    "separation" if code in {"SEPARATION", "VALIDATION_INCOMPLETE"} else "scene")
        threshold = limits if category == "aircraft" else {}
        checks.append({
            "code": code,
            "category": category,
            "rule_type": None,
            "rule_id": None,
            "status": "failed",
            "point_wkt": point,
            "time_s": moment,
            "threshold": threshold,
            "actual": _segment_actual(points, times, index),
            "reason": "; ".join(str(item.get("message", "constraint violated")) for item in items),
        })
    return checks


def _wait_time(points, times) -> float:
    points, times = np.asarray(points, float), np.asarray(times, float)
    if len(points) < 2:
        return 0.0
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    return float(np.sum(np.diff(times)[lengths <= _EPS]))


def _metrics(compiled, task, points, times, motion, config) -> dict:
    distance = float(motion.distance) if motion is not None else float(
        np.sum(np.linalg.norm(np.diff(np.asarray(points, float), axis=0), axis=1)))
    flight = float(times[-1] - times[0]) if len(times) else 0.0
    wait = _wait_time(points, times)
    energy = _unavailable("Wh", "energy_model was not supplied, so energy cannot be evaluated")
    if config is not None and config.energy_model is not None:
        if motion is None:
            energy = _unavailable("Wh", "analytic motion_segments are required for energy evaluation")
        else:
            try:
                evaluated = objective_module.evaluate(compiled, config, motion)
            except objective_module.ObjectiveCoverageError as exc:
                energy = _unavailable("Wh", str(exc))
            else:
                value = evaluated["values"].get("energy")
                energy = (_metric(value["raw"], "Wh") if value is not None
                          else _unavailable("Wh", "energy could not be integrated"))
    risk_datasets = compiled.datasets_for(None, "risk")
    if not risk_datasets:
        risk = _unavailable("risk*s", "scene has no versioned risk datasets")
    elif motion is None:
        risk = _unavailable("risk*s", "analytic motion_segments are required for risk integration")
    else:
        try:
            risk = _metric(objective_module.field_integral(motion, risk_datasets, "risk", "time"), "risk*s")
        except objective_module.ObjectiveCoverageError as exc:
            risk = _unavailable("risk*s", str(exc))
    return {"distance_m": _metric(distance, "m"), "flight_time_s": _metric(flight, "s"),
            "wait_time_s": _metric(wait, "s"), "energy_wh": energy, "risk": risk}


def _objective_config(value) -> ObjectiveConfig | None:
    if value is None:
        return None
    try:
        return value if isinstance(value, ObjectiveConfig) else ObjectiveConfig.model_validate(value)
    except ValueError as exc:
        raise PlanningError(ErrorCode.INVALID_INPUT, str(exc)) from exc


def _score(metrics, config) -> dict:
    weights = config.weights if config is not None else ObjectiveWeights(distance=1)
    scales = config.normalization if config is not None else ObjectiveNormalization()
    lookup = {"distance": (metrics["distance_m"], scales.distance_m),
              "time": (metrics["flight_time_s"], scales.time_s),
              "energy": (metrics["energy_wh"], scales.energy_wh),
              "risk": (metrics["risk"], scales.risk_s)}
    total = 0.0
    for name, weight in weights.model_dump().items():
        if not weight:
            continue
        metric, scale = lookup[name]
        if not metric["available"]:
            return _unavailable("dimensionless",
                                f"weighted objective {name} is unavailable: {metric['reason']}")
        total += weight * metric["value"] / scale
    if not math.isfinite(total):
        return _unavailable("dimensionless", "weighted score is not finite")
    return _metric(total, "dimensionless")


def evaluate_v2(*, task, trajectory_wkt, vertex_times_s, motion_segments=None,
                network_route=None, initial_velocity_xyz_mps=(0, 0, 0),
                scene_id=None, scene=None, objectives=None, plan_id="plan-001") -> dict:
    """Validate one route and evaluate it against the ENG-10 contract."""
    scene_id, model, compiled, version = _scene_context(scene_id, scene)
    parsed_task = Task.model_validate(task)
    checked = engine.validate_route_v2(
        task=task, trajectory_wkt=trajectory_wkt, vertex_times_s=vertex_times_s,
        motion_segments=motion_segments, network_route=network_route,
        scene_id=scene_id, scene=None, initial_velocity_xyz_mps=initial_velocity_xyz_mps)
    times = np.asarray(vertex_times_s, float)
    points = engine._timed_local(compiled, trajectory_wkt, times, parsed_task.alt_reference, model)
    violations = list(checked["violations"])
    profile = engine._profile(model, parsed_task)
    rules = _rule_index(compiled)
    thresholds = {rule_id: _rule_threshold(rule) for rule_id, rule in rules.items()}
    checks = [_check(report, compiled, model, parsed_task, points, times, violations, thresholds)
              for report in checked["rule_report"]]
    checks.extend(_unmatched_checks(compiled, model, parsed_task, points, times, violations, profile))
    motion = None
    if motion_segments:
        try:
            motion = engine._reconstruct_motion(motion_segments, points, times)
        except (ValueError, TypeError, KeyError):
            motion = None
    config = _objective_config(objectives)
    metrics = _metrics(compiled, parsed_task, points, times, motion, config)
    score = _score(metrics, config)
    valid = bool(checked["valid"])
    rankable = valid and score["available"]
    if valid and not score["available"]:
        exclusion_code, exclusion_reason = "MISSING_METRIC", score["reason"]
    elif not valid:
        exclusion_code, exclusion_reason = "HARD_CONSTRAINT_FAILED", "; ".join(
            sorted({item.get("message", "constraint violated") for item in violations})) or "constraint failed"
    else:
        exclusion_code, exclusion_reason = None, None
    result = {
        "plan_id": plan_id, "scene_id": scene_id, "scene_version": version,
        "engine_id": "ENG-10", "valid": valid,
        "safety": {"valid": valid, "checks": checks,
                   "violations": [Violation.model_validate(_violation(item)) for item in violations]},
        "metrics": EvaluationMetrics.model_validate(metrics),
        "score": MetricValue.model_validate(score),
        "rankable": rankable, "exclusion_code": exclusion_code,
        "exclusion_reason": exclusion_reason,
    }
    return EvaluationResult.model_validate(result).model_dump(mode="json")


def _violation(item: dict) -> dict:
    return {"code": str(item.get("code", "CONSTRAINT")),
            "message": str(item.get("message", "constraint violated")),
            "rule_id": item.get("rule_id"),
            "time_s": item.get("time_s") or (item.get("details") or {}).get("time_s"),
            "details": {key: value for key, value in (item.get("details") or {}).items()
                        if key != "time_s"}}


def compare_plans_v2(*, request_id, plans, objectives, scene_id=None, scene=None) -> dict:
    """Evaluate candidate plans and rank the ones that are safe and complete."""
    if len(plans) < 2:
        raise PlanningError(ErrorCode.INVALID_INPUT, "compare requires at least two plans")
    seen: set[str] = set()
    results = []
    per_plan_scene = False
    for candidate in plans:
        plan_id = str(_get(candidate, "plan_id"))
        if plan_id in seen:
            raise PlanningError(ErrorCode.INVALID_INPUT, "plan_id must be unique", {"plan_id": plan_id})
        seen.add(plan_id)
        own_scene = _get(candidate, "scene") or _get(candidate, "scene_id")
        per_plan_scene = per_plan_scene or bool(own_scene)
        results.append(evaluate_v2(
            task=_get(candidate, "task"), trajectory_wkt=_get(candidate, "trajectory_wkt"),
            vertex_times_s=_get(candidate, "vertex_times_s"),
            motion_segments=_get(candidate, "motion_segments"),
            network_route=_get(candidate, "network_route"),
            initial_velocity_xyz_mps=_get(candidate, "initial_velocity_xyz_mps", (0, 0, 0)),
            scene_id=_get(candidate, "scene_id") or scene_id,
            scene=_get(candidate, "scene") or scene,
            objectives=objectives, plan_id=plan_id))
    rankable = sorted((r for r in results if r["rankable"]),
                      key=lambda item: (item["score"]["value"], item["plan_id"]))
    ranking = [item["plan_id"] for item in rankable]
    config = _objective_config(objectives)
    comparison = {
        "request_id": request_id,
        "evaluated_at": datetime.now(timezone.utc).isoformat(),
        "scene_mode": "per_plan" if per_plan_scene else "shared",
        "ranking": ranking,
        "winner": ranking[0] if ranking else None,
        "plans": results,
        "weights": config.weights,
        "normalization": config.normalization,
        "notes": ["Ranking contains only plans that pass every hard constraint and provide every positively weighted metric.",
                  "Excluded plans are reported with exclusion_code and exclusion_reason; they are never scored as zero."],
    }
    return PlanComparison.model_validate(comparison).model_dump(mode="json")
