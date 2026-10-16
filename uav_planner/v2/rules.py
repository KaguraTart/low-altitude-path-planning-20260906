"""Strict ALL20 rule parsing and continuous route rule evaluation.

Rule dictionaries are deliberately closed: a misspelled constraint is an input
error rather than an accidentally unconstrained flight.
"""
from __future__ import annotations

from datetime import datetime, time, timedelta, timezone
from math import acos, atan2, degrees, isfinite, pi, sin
from typing import Any, Iterable, TYPE_CHECKING
from zoneinfo import ZoneInfo

import numpy as np

from uav_planner.errors import ErrorCode, PlanningError

if TYPE_CHECKING:
    from .scene import CompiledScene

EPS = 1e-7
LOCAL_TZ = ZoneInfo("Asia/Shanghai")
RULE_KEYS = {
    "HEIGHT_LAYER": {"minHeight", "maxHeight", "layerStep", "minAltitude", "maxAltitude", "altDatum"},
    "WIDTH": {"width", "bufferWidth"},
    "DIRECTION": {"direction", "directions"},
    "SPEED": {"minSpeed", "maxSpeed", "cruiseSpeed"},
    "TIME": {"dailyStart", "dailyEnd", "weekdays", "earliestDeparture", "latestArrival"},
    "AREA": {"scope", "adminCodes", "gridCodes", "zoneIds"},
    "CLIMB": {"maxClimbRate", "maxDescentRate"},
    "TURN": {"maxTurnAngle", "minTurnRadius"},
    "SEPARATION": {"verticalSeparation", "horizontalSeparation"},
    "NO_FLY": {"noFlyMode", "heightCeiling", "includeBuffer"},
    "BUILDING": {"avoidMode", "safeDistance", "bufferRadius"},
    "ZONE": {"zoneLevels", "coverageThreshold"},
    "WEATHER": {"maxWindSpeed", "noPrecipitation", "noIcing"},
    "SIGNAL": {"minSignalStrength", "bands", "band_policy"},
    "PRIORITY": {"priorityScene", "costBonus"},
    "HOVER": {"maxHoverTime", "defaultHoverTime"},
    "ENDURANCE": {"maxRange", "minEndurance"},
    "PAYLOAD": {"maxPayloadKg"},
    "WAYPOINT": {"maxWaypoints", "minWaypointInterval"},
    "RTL": {"rtlAltitude", "rtlOnLowBattery", "lowBatteryPercent"},
}


def get(obj: Any, key: str, default: Any = None) -> Any:
    return obj.get(key, default) if isinstance(obj, dict) else getattr(obj, key, default)


def invalid(message: str, **detail: Any) -> None:
    raise PlanningError(ErrorCode.INVALID_INPUT, message, detail)


def as_datetime(value: Any) -> datetime | None:
    if value is None:
        return None
    result = datetime.fromisoformat(value.replace("Z", "+00:00")) if isinstance(value, str) else value
    if not isinstance(result, datetime) or result.tzinfo is None or result.utcoffset() is None:
        invalid("Rule timestamps must include a timezone", value=str(value))
    return result


def _number(content: dict, key: str, *, lo: float | None = 0, hi: float | None = None) -> None:
    if key not in content:
        return
    val = content[key]
    if isinstance(val, bool) or not isinstance(val, (int, float)) or not isfinite(val):
        invalid("Rule value must be a finite number", field=key)
    if (lo is not None and val < lo) or (hi is not None and val > hi):
        invalid("Rule value is outside its allowed range", field=key, value=val)


def validate_rule(rule: Any) -> None:
    kind, content = get(rule, "rule_type"), get(rule, "content")
    if kind not in RULE_KEYS:
        invalid("Unknown ALL20 rule type", rule_id=get(rule, "id"), rule_type=kind)
    if not isinstance(content, dict) or not content:
        invalid("Rule content must contain a constraint", rule_id=get(rule, "id"))
    unknown = set(content) - RULE_KEYS[kind]
    if unknown:
        invalid("Unknown rule content fields", rule_id=get(rule, "id"), fields=sorted(unknown))
    for key, value in content.items():
        if key in {"includeBuffer", "noPrecipitation", "noIcing", "rtlOnLowBattery"}:
            if not isinstance(value, bool):
                invalid("Rule boolean field has wrong type", field=key)
        elif key in {"directions", "weekdays", "bands", "adminCodes", "gridCodes", "zoneIds", "zoneLevels"}:
            if not isinstance(value, list) or not value:
                invalid("Rule list field must be nonempty", field=key)
        elif key not in {"altDatum", "direction", "dailyStart", "dailyEnd", "earliestDeparture", "latestArrival", "scope", "noFlyMode", "avoidMode", "priorityScene", "band_policy"}:
            _number(content, key, lo=None if key in {"minSignalStrength", "minHeight", "maxHeight", "minAltitude", "maxAltitude"} else 0)
    for lower, upper in (("minHeight", "maxHeight"), ("minAltitude", "maxAltitude"), ("minSpeed", "maxSpeed")):
        if lower in content and upper in content and content[lower] > content[upper]:
            invalid("Rule lower bound exceeds upper bound", field=lower)
    required = {
        "WIDTH": {"width"}, "AREA": {"scope"}, "NO_FLY": {"noFlyMode"},
        "BUILDING": {"avoidMode"}, "SIGNAL": {"minSignalStrength"},
        "PRIORITY": {"priorityScene", "costBonus"}, "PAYLOAD": {"maxPayloadKg"},
        "RTL": {"rtlAltitude", "rtlOnLowBattery", "lowBatteryPercent"},
    }.get(kind, set())
    if not required <= content.keys():
        invalid("Rule missing required fields", rule_id=get(rule, "id"), fields=sorted(required - content.keys()))
    meaningful = {
        "HEIGHT_LAYER": {"minHeight", "maxHeight", "minAltitude", "maxAltitude"},
        "DIRECTION": {"direction", "directions"}, "TIME": {"dailyStart", "dailyEnd", "earliestDeparture", "latestArrival", "weekdays"},
        "WEATHER": {"maxWindSpeed", "noPrecipitation", "noIcing"},
        "ZONE": {"zoneLevels", "coverageThreshold"},
    }.get(kind)
    if meaningful and not meaningful.intersection(content):
        invalid("Rule has no executable constraint", rule_id=get(rule, "id"))
    enum_values = {
        "altDatum": {"MSL", "AGL", "ELLIPSOID", "QNE", "QNH"},
        "direction": {"ONE_WAY", "BIDIRECTIONAL"}, "scope": {"INCLUDE", "EXCLUDE"},
        "noFlyMode": {"PERMANENT", "TEMPORARY"}, "avoidMode": {"BUFFER", "CORRIDOR"},
        "priorityScene": {"LOW_RISK", "FAST", "QUIET"}, "band_policy": {"any", "all"},
    }
    for key, choices in enum_values.items():
        if key in content and (not isinstance(content[key],str) or content[key] not in choices):
            invalid("Unknown rule enum value", field=key, value=content[key])
    for key in ("dailyStart", "dailyEnd", "earliestDeparture", "latestArrival"):
        if key in content:
            try:
                value = content[key]
                if not isinstance(value, str) or len(value) != 5 or value[2] != ":":
                    raise ValueError
                time.fromisoformat(value)
            except (ValueError, TypeError):
                invalid("Daily time must use HH:mm", field=key)
    if ("dailyStart" in content) != ("dailyEnd" in content):
        invalid("dailyStart and dailyEnd must be specified together")
    if content.get("defaultHoverTime", 0) > content.get("maxHoverTime", float("inf")):
        invalid("defaultHoverTime exceeds maxHoverTime")
    for key in ("coverageThreshold", "costBonus"):
        _number(content, key, hi=1)
    _number(content, "lowBatteryPercent", hi=100)
    _number(content, "maxTurnAngle", hi=180)
    for key in ("layerStep", "width", "maxSpeed", "cruiseSpeed", "maxRange", "minEndurance"):
        if key in content and content[key] <= 0:
            invalid("Rule value must be positive", field=key)
    for key in ("maxWaypoints",):
        if key in content and (isinstance(content[key], bool) or int(content[key]) != content[key] or content[key] < 2):
            invalid("maxWaypoints must be an integer of at least two")
    for key in ("minHeight", "maxHeight", "minAltitude", "maxAltitude"):
        _number(content, key, lo=None)
    if "weekdays" in content and any(isinstance(n, bool) or not isinstance(n, int) or not 1 <= n <= 7 for n in content["weekdays"]):
        invalid("weekdays must contain ISO weekday numbers 1 through 7")
    if "directions" in content and any(isinstance(n, bool) or not isinstance(n, (float, int)) or not isfinite(n) or not 0 <= n < 360 for n in content["directions"]):
        invalid("directions must contain angles in [0, 360)")
    for key in ("bands", "adminCodes", "gridCodes", "zoneLevels", "zoneIds"):
        if key in content and any(not isinstance(s, str) or not s for s in content[key]):
            invalid("Rule field must contain nonempty strings", field=key)
    start, end = as_datetime(get(rule, "effective_start")), as_datetime(get(rule, "effective_end"))
    if start and end and start > end:
        invalid("Rule effective period is reversed", rule_id=get(rule, "id"))
    if kind == "NO_FLY" and content["noFlyMode"] == "TEMPORARY" and (not start or not end):
        invalid("Temporary no-fly rules require an explicit effective period", rule_id=get(rule, "id"))


def violation(rule: Any, reason: str, *, segment: int | None = None, **detail: Any) -> dict:
    result = {"rule_id": get(rule, "id"), "rule_type": get(rule, "rule_type"), "reason": reason, **detail}
    if segment is not None:
        result["segment"] = segment
    return result


class RuleSet:
    def __init__(self, scene: CompiledScene, rules: Iterable[Any]):
        self.scene = scene
        self.items = list(rules)
        seen: set[str] = set()
        for rule in self.items:
            validate_rule(rule)
            rid = str(get(rule, "id"))
            if rid in seen:
                invalid("Duplicate rule id", rule_id=rid)
            seen.add(rid)
            refs = [str(x) for x in get(rule, "resource_ids", [])]
            missing = [x for x in refs if x not in scene.resources]
            if missing:
                invalid("Rule references missing resources", rule_id=rid, resource_ids=missing)
            self._validate_resources(rule)

    def _validate_resources(self, rule: Any) -> None:
        kind, content = get(rule, "rule_type"), get(rule, "content")
        if kind in {"WEATHER", "SIGNAL"} or (kind == "PRIORITY" and content["priorityScene"] in {"LOW_RISK", "QUIET"}):
            dataset_kind = {"WEATHER": "weather", "SIGNAL": "signal"}.get(kind, "noise" if content.get("priorityScene") == "QUIET" else "risk")
            if not self.scene.datasets_for(rule, dataset_kind):
                invalid("Rule requires a versioned spatial dataset", rule_id=get(rule, "id"), dataset_kind=dataset_kind)
            for dataset in self.scene.datasets_for(rule, dataset_kind):
                if not dataset.cells:
                    invalid("Rule requires dataset field cells", rule_id=get(rule, "id"), dataset_id=dataset.id)
                for cell in dataset.cells:
                    values = cell.values
                    if values.get("nodata", False):
                        continue
                    required_values = []
                    if kind == "WEATHER":
                        if "maxWindSpeed" in content:
                            required_values.append(("wind_speed_mps", values.get("wind_speed_mps", values.get("windSpeed")), "number"))
                        if content.get("noPrecipitation"):
                            required_values.append(("precipitation", values.get("precipitation", values.get("precipitation_mm_h")), "weather_flag"))
                        if content.get("noIcing"):
                            required_values.append(("icing", values.get("icing"), "weather_flag"))
                    elif kind == "SIGNAL":
                        signal = values.get("bands", values.get("signal_strength_dbm"))
                        if isinstance(signal, dict):
                            if not signal or not all(isinstance(v, (int, float)) and not isinstance(v, bool) and isfinite(v) for v in signal.values()):
                                invalid("Signal bands require finite dBm values", dataset_id=dataset.id)
                        else:
                            required_values.append(("bands or signal_strength_dbm", signal, "number"))
                    else:
                        key = "noise_db" if dataset_kind == "noise" else "risk"
                        required_values.append((key, values.get(key), "number"))
                    for name, value, value_type in required_values:
                        good = isinstance(value, (int, float)) and isfinite(value) and not isinstance(value, bool)
                        if value_type == "weather_flag":
                            good = isinstance(value, bool) or good and value >= 0
                        if not good:
                            invalid("Rule field dataset is missing a required value", rule_id=get(rule, "id"), dataset_id=dataset.id, field=name)
        if kind == "HEIGHT_LAYER" and content.get("altDatum", "MSL") == "AGL":
            if self.scene.terrain_constant is None and not self.scene.datasets_for(None, "terrain"):
                invalid("AGL height rules require terrain data", rule_id=get(rule, "id"))
        if kind == "WIDTH" and not self.scene.airways:
            invalid("WIDTH rule requires an airway", rule_id=get(rule, "id"))
        if kind == "DIRECTION" and content.get("direction") == "ONE_WAY" and not content.get("directions"):
            if not self.scene.airways:
                invalid("ONE_WAY requires an airway centerline or explicit directions", rule_id=get(rule, "id"))
        if kind == "AREA" and not self.scene.area_for(rule):
            invalid("AREA rule requires resolvable area resources", rule_id=get(rule, "id"))
        if kind in {"NO_FLY", "ZONE", "BUILDING"} and not self.scene.volumes_for(rule, kind):
            invalid("Avoidance rule requires matching geometry resources", rule_id=get(rule, "id"))
        if kind == "BUILDING" and content["avoidMode"] == "CORRIDOR":
            if not self.scene.datasets_for(rule, "corridor"):
                invalid("BUILDING CORRIDOR requires explicit traversable corridor geometry", rule_id=get(rule, "id"))

    def selected(self, task: Any = None) -> list[Any]:
        ids = get(task, "rule_ids")
        if ids is None:
            return self.items
        known = {str(get(r, "id")) for r in self.items}
        missing = set(map(str, ids)) - known
        if missing:
            invalid("Task references missing rules", rule_ids=sorted(missing))
        return [r for r in self.items if str(get(r, "id")) in set(map(str, ids))]

    def interval(self, rule: Any, t0: float, t1: float) -> tuple[float, float] | None:
        start, end = as_datetime(get(rule, "effective_start")), as_datetime(get(rule, "effective_end"))
        low = (start - self.scene.time_origin).total_seconds() if start else -float("inf")
        high = (end - self.scene.time_origin).total_seconds() if end else float("inf")
        a, b = max(t0, low), min(t1, high)
        return (a, b) if a <= b + EPS else None

    def active(self, rule: Any, t: float) -> bool:
        return self.interval(rule, t, t) is not None

    def separation(self, task: Any = None) -> tuple[float, float]:
        h = v = 0.0
        departure = as_datetime(get(task, "departure_time")) or self.scene.time_origin
        t0 = (departure - self.scene.time_origin).total_seconds()
        for rule in self.selected(task):
            if get(rule, "rule_type") == "SEPARATION":
                if self.interval(rule, t0, self.scene.horizon_s) is None:
                    continue
                c = get(rule, "content")
                h, v = max(h, c.get("horizontalSeparation", 0)), max(v, c.get("verticalSeparation", 0))
        return h, v

    def limits(self, task: Any, profile: Any) -> dict:
        """Search hints; final validation still gates every rule by time and scope."""
        result = {"max_speed_mps": get(profile, "max_speed_mps", 10), "min_speed_mps": 0,
                  "cruise_speed_mps": get(profile, "cruise_speed_mps", 8),
                  "climb_speed_mps": get(profile, "climb_speed_mps", 3),
                  "descent_speed_mps": get(profile, "descent_speed_mps", 3),
                  "max_yaw_rate_deg_s": get(profile, "max_yaw_rate_deg_s", 60),
                  "acceleration_mps2": get(profile, "acceleration_mps2", 2),
                  "deceleration_mps2": get(profile, "deceleration_mps2", 2),
                  "max_turn_angle_deg": 180,
                  "min_turn_radius_m": get(profile, "min_turn_radius_m", 0)}
        departure = as_datetime(get(task, "departure_time")) or self.scene.time_origin
        t = (departure - self.scene.time_origin).total_seconds()
        for rule in self.selected(task):
            if not self.active(rule, t):
                continue
            c, kind = get(rule, "content"), get(rule, "rule_type")
            if kind == "SPEED":
                result["max_speed_mps"] = min(result["max_speed_mps"], c.get("maxSpeed", float("inf")))
                result["min_speed_mps"] = max(result["min_speed_mps"], c.get("minSpeed", 0))
                result["cruise_speed_mps"] = min(result["cruise_speed_mps"], c.get("cruiseSpeed", float("inf")))
            if kind == "CLIMB":
                result["climb_speed_mps"] = min(result["climb_speed_mps"], c.get("maxClimbRate", float("inf")))
                result["descent_speed_mps"] = min(result["descent_speed_mps"], c.get("maxDescentRate", float("inf")))
            if kind == "TURN":
                result["min_turn_radius_m"] = max(result["min_turn_radius_m"], c.get("minTurnRadius", 0))
                result["max_turn_angle_deg"] = min(result["max_turn_angle_deg"], c.get("maxTurnAngle", 180))
        result["cruise_speed_mps"] = min(result["cruise_speed_mps"], result["max_speed_mps"])
        return result

    def rtl_requirements(self, task: Any, profile: Any = None) -> dict | None:
        state = get(task, "state")
        battery = get(state, "battery_percent")
        departure = as_datetime(get(task, "departure_time")) or self.scene.time_origin
        t = (departure - self.scene.time_origin).total_seconds()
        applicable = [r for r in self.selected(task) if get(r, "rule_type") == "RTL" and self.active(r, t)]
        if not applicable:
            return None
        if any(get(r, "content")["rtlOnLowBattery"] for r in applicable):
            if state is None or battery is None or get(state, "remaining_flight_time_s") is None or get(state, "remaining_range_m") is None or not get(state, "home_wkt"):
                invalid("RTL low-battery planning requires upstream battery, remaining flight time/range and home position", task_id=get(task, "id"))
        altitude = max(get(r, "content")["rtlAltitude"] for r in applicable)
        triggers = [r for r in applicable if get(r, "content")["rtlOnLowBattery"] and battery is not None and battery <= get(r, "content")["lowBatteryPercent"]]
        return {"required": bool(triggers), "rtl_altitude_m": altitude, "rule_ids": [get(r, "id") for r in applicable], "reserve_required": any(get(r, "content")["rtlOnLowBattery"] for r in applicable)}

    rtl_required = rtl_requirements

    def check_segment(self, p0: np.ndarray, p1: np.ndarray, t0: float, t1: float, task: Any, profile: Any, segment: int = 0, enforce_min_speed: bool = True, kinds: set | None = None) -> list[dict]:
        errors: list[dict] = []
        dt = t1 - t0
        delta = p1 - p0
        speed = float(np.linalg.norm(delta)) / dt if dt > EPS else (float("inf") if np.linalg.norm(delta) > EPS else 0)
        climb = float(delta[2]) / dt if dt > EPS else 0
        heading = degrees(atan2(delta[1], delta[0])) % 360
        for rule in self.selected(task):
            if kinds is not None and get(rule, "rule_type") not in kinds:
                continue
            active = self.interval(rule, t0, t1)
            if active is None:
                continue
            kind, c = get(rule, "rule_type"), get(rule, "content")
            parts = self.scene.rule_parts(rule, p0, p1, t0, t1)
            if not parts:
                continue
            reason = None
            if kind == "SPEED" and speed > EPS:
                if (enforce_min_speed and speed < c.get("minSpeed", 0) - EPS) or speed > c.get("maxSpeed", float("inf")) + EPS:
                    reason = "segment speed is outside allowed limits"
            elif kind == "CLIMB":
                if climb > c.get("maxClimbRate", float("inf")) + EPS or -climb > c.get("maxDescentRate", float("inf")) + EPS:
                    reason = "segment climb or descent rate exceeds limit"
            elif kind == "HEIGHT_LAYER":
                for a, b in parts:
                    if not self.scene.height_part_allowed(p0 + a * delta, p0 + b * delta, t0 + a * dt, t0 + b * dt, c):
                        reason = "segment altitude is outside its allowed height layer or terrain is missing"
                        break
            elif kind == "DIRECTION" and np.linalg.norm(delta[:2]) > EPS:
                if c.get("directions") and not any(abs((heading - h + 180) % 360 - 180) <= 1e-4 for h in c["directions"]):
                    reason = "segment heading is not in allowed East-zero counterclockwise directions"
                elif c.get("direction") == "ONE_WAY" and not c.get("directions") and not self.scene.direction_allowed(rule, p0, p1):
                    reason = "segment travels against the referenced airway direction"
            elif kind == "WIDTH":
                if not self.scene.width_allowed(rule, p0, p1, parts, profile):
                    reason = "segment exceeds the reduced airway half width"
            elif kind == "TIME":
                if not all(_daily_interval_allowed(self.scene.time_origin + timedelta(seconds=t0 + a * dt), self.scene.time_origin + timedelta(seconds=t0 + b * dt), c) for a, b in parts):
                    reason = "segment crosses a disallowed daily time or weekday"
            elif kind == "AREA":
                if not self.scene.area_allowed(rule, p0, p1, parts, profile, t0, t1):
                    reason = "segment violates the included or excluded area boundary"
            elif kind in {"NO_FLY", "BUILDING", "ZONE"}:
                if self.scene.rule_volume_collision(rule, p0, p1, t0, t1, profile):
                    reason = "segment intersects an active forbidden volume"
            elif kind in {"WEATHER", "SIGNAL"}:
                for a, b in parts:
                    if not self.scene.field_part_allowed(rule, p0 + a * delta, p0 + b * delta, t0 + a * dt, t0 + b * dt, profile):
                        reason = "segment crosses unsafe or missing spatial field data"
                        break
            elif kind == "PRIORITY" and c["priorityScene"] in {"QUIET", "LOW_RISK"}:
                if not all(self.scene.priority_data_covered(rule, p0 + a * delta, p0 + b * delta, t0 + a * dt, t0 + b * dt) for a, b in parts):
                    reason = "priority cost field has a nodata gap"
            if reason:
                errors.append(violation(rule, reason, segment=segment))
        return errors

    def check_path(self, points: np.ndarray, times: np.ndarray, task: Any, profile: Any, phase: str = "route") -> list[dict]:
        errors: list[dict] = []
        if not len(points):
            return errors
        lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
        elapsed, distance = float(times[-1] - times[0]), float(lengths.sum())
        state = get(task, "state")
        remaining = get(state, "remaining_flight_time_s", None)
        if remaining is None:
            remaining = get(profile, "endurance_s", float("inf"))
        task_wps = list(get(task, "waypoints", []))
        for rule in self.selected(task):
            if self.interval(rule, float(times[0]), float(times[-1])) is None:
                continue
            c, kind = get(rule, "content"), get(rule, "rule_type")
            if get(rule, "resource_ids") and kind not in {"AREA", "NO_FLY", "BUILDING", "ZONE", "WEATHER", "SIGNAL", "PRIORITY"}:
                if len(points) == 1:
                    scoped = self.scene.rule_parts(rule, points[0], points[0], float(times[0]), float(times[0]))
                else:
                    scoped = any(self.scene.rule_parts(rule, points[i], points[i + 1], float(times[i]), float(times[i + 1])) for i in range(len(points) - 1))
                if not scoped:
                    continue
            reason = None
            if kind == "ENDURANCE":
                if get(profile, "endurance_s", 0) + EPS < c.get("minEndurance", 0):
                    reason = "aircraft endurance capacity is below the rule minimum"
                elif distance > min(c.get("maxRange", float("inf")), get(state, "remaining_range_m") if get(state, "remaining_range_m") is not None else float("inf")) + EPS or elapsed > remaining + EPS:
                    reason = "route exceeds range or remaining endurance budget"
            elif kind == "PAYLOAD":
                capacity = get(profile, "payload_kg", 0)
                if capacity + EPS < c["maxPayloadKg"] or get(task, "payload_kg", 0) > min(capacity, c["maxPayloadKg"]) + EPS:
                    reason = "aircraft capacity or task payload violates payload requirements"
            elif kind == "WAYPOINT":
                # Business waypoints, including start and goal; search/smoothing nodes are not user waypoints.
                wps = [points[0]] + [self.scene.waypoint_local(w, task) for w in task_wps] + [points[-1]]
                if len(wps) > c.get("maxWaypoints", float("inf")):
                    reason = "task waypoint count exceeds limit"
                elif any(np.linalg.norm(b - a) + EPS < c.get("minWaypointInterval", 0) for a, b in zip(wps, wps[1:])):
                    reason = "adjacent task waypoints are too close"
            elif kind == "HOVER":
                i = 0
                while i < len(lengths):
                    if lengths[i] > EPS:
                        i += 1
                        continue
                    j = i + 1
                    while j < len(lengths) and np.linalg.norm(points[j + 1] - points[i]) <= EPS:
                        j += 1
                    active = self.interval(rule, float(times[i]), float(times[j]))
                    duration = active[1] - active[0] if active is not None else 0
                    if get(rule, "resource_ids") and not self.scene.rule_parts(rule, points[i], points[j], float(times[i]), float(times[j])):
                        duration = 0
                    if duration <= EPS:
                        i = j
                        continue
                    marked = [w for w in task_wps if np.linalg.norm(self.scene.waypoint_local(w, task) - points[i]) <= 1e-3 and (get(w, "hover_s", 0) > 0 or get(w, "hover", False) or get(w, "default_hover", False))]
                    declared = max((get(w, "hover_s", 0) or c.get("defaultHoverTime", 0) for w in marked), default=0)
                    if not marked or duration > c.get("maxHoverTime", float("inf")) + EPS or duration > declared + EPS:
                        reason = "hover is unmarked or exceeds the permitted waypoint duration"
                        break
                    i = j
            elif kind == "TURN" and len(points) >= 3:
                accumulated_angle, previous_sign = 0.0, 0
                for i in range(1, len(points) - 1):
                    if not self.active(rule, float(times[i])) or not self.scene.rule_parts(rule, points[i - 1], points[i + 1], float(times[i - 1]), float(times[i + 1])):
                        accumulated_angle, previous_sign = 0.0, 0
                        continue
                    a, b = points[i, :2] - points[i - 1, :2], points[i + 1, :2] - points[i, :2]
                    la, lb = np.linalg.norm(a), np.linalg.norm(b)
                    if min(la, lb) < EPS:
                        continue
                    angle = acos(float(np.clip(np.dot(a, b) / (la * lb), -1, 1)))
                    cross = a[0] * b[1] - a[1] * b[0]
                    sign = 1 if cross > EPS else -1 if cross < -EPS else 0
                    if angle < 1e-6:
                        accumulated_angle, previous_sign = 0.0, 0
                    else:
                        accumulated_angle = accumulated_angle + angle if sign == previous_sign else angle
                        previous_sign = sign
                    radius = float(np.linalg.norm(a + b) / (2 * sin(angle))) if angle > EPS and angle < pi - EPS else (float("inf") if angle <= EPS else 0)
                    if degrees(accumulated_angle) > c.get("maxTurnAngle", 180) + EPS or radius + EPS < c.get("minTurnRadius", 0):
                        reason = "route turn angle or geometric radius violates turn constraints"
                        break
            elif kind == "TIME":
                departure = (self.scene.time_origin + timedelta(seconds=float(times[0]))).astimezone(LOCAL_TZ)
                arrival = (self.scene.time_origin + timedelta(seconds=float(times[-1]))).astimezone(LOCAL_TZ)
                if "earliestDeparture" in c and departure.time().replace(tzinfo=None) < time.fromisoformat(c["earliestDeparture"]):
                    reason = "departure precedes the allowed local departure time"
                if "latestArrival" in c and arrival.time().replace(tzinfo=None) > time.fromisoformat(c["latestArrival"]):
                    reason = "arrival exceeds the allowed local arrival time"
            elif kind == "RTL" and phase == "rtl":
                # Root builds the return route and raises to this MSL altitude before transit.
                moving = [i for i in range(len(lengths)) if np.linalg.norm(points[i + 1, :2] - points[i, :2]) > EPS]
                if any(not self.scene.height_part_allowed(points[i], points[i + 1], times[i], times[i + 1], {"minAltitude": c["rtlAltitude"], "altDatum": "MSL"}) for i in moving):
                    reason = "return transit is below the required RTL altitude"
            if reason:
                errors.append(violation(rule, reason))
        return errors

    def report(self, task: Any = None, violations: Iterable[dict] = (), times: Iterable[float] | None = None) -> list[dict]:
        failures = list(violations)
        if times is None:
            departure = as_datetime(get(task, "departure_time")) or self.scene.time_origin
            t0 = t1 = (departure - self.scene.time_origin).total_seconds()
        else:
            arr = list(times)
            t0, t1 = (min(arr), max(arr)) if arr else (0, 0)
        selected = {str(get(r, "id")) for r in self.selected(task)}
        result = []
        for rule in self.items:
            rid = get(rule, "id")
            failed = [v.get("reason", v.get("message", "rule violation")) for v in failures if str(v.get("rule_id")) == str(rid)]
            enabled = str(rid) in selected and self.interval(rule, t0, t1) is not None
            reasons = failed or ([] if enabled else ["outside effective period or not selected"])
            result.append({"rule_id": rid, "rule_type": get(rule, "rule_type"), "status": "failed" if failed else "applied" if enabled else "not_applicable", "message": "; ".join(reasons), "details": {"reasons": reasons, "dataset_versions": {d.id: d.version for d in self.scene.datasets_for(rule)}}})
        return result


def _daily_interval_allowed(start: datetime, end: datetime, content: dict) -> bool:
    """Cover a complete interval by permitted local daily windows, including midnight."""
    start, end = start.astimezone(LOCAL_TZ), end.astimezone(LOCAL_TZ)
    weekdays = set(content.get("weekdays", range(1, 8)))
    if "dailyStart" not in content:
        day = start.date()
        while day <= end.date():
            if day.isoweekday() not in weekdays:
                return False
            day += timedelta(days=1)
        return True
    begin, finish = time.fromisoformat(content["dailyStart"]), time.fromisoformat(content["dailyEnd"])
    windows = []
    day = start.date() - timedelta(days=1)
    while day <= end.date():
        if day.isoweekday() in weekdays:
            a = datetime.combine(day, begin, LOCAL_TZ)
            b = datetime.combine(day + timedelta(days=finish <= begin), finish, LOCAL_TZ)
            windows.append((a, b))
        day += timedelta(days=1)
    cursor = start
    for a, b in windows:
        if b < cursor:
            continue
        if a > cursor:
            return False
        cursor = max(cursor, b)
        if cursor >= end:
            return True
    return False


def separation_violations(points: Any, times: Any, other_points: Any, other_times: Any, horizontal_m: float, vertical_m: float) -> list[dict]:
    """Exact piecewise-linear simultaneous proximity; no time sampling holes."""
    p, q = np.asarray(points, float), np.asarray(other_points, float)
    pt, qt = np.asarray(times, float), np.asarray(other_times, float)
    for i in range(len(p) - 1):
        for j in range(len(q) - 1):
            a, b = max(pt[i], qt[j]), min(pt[i + 1], qt[j + 1])
            if b < a or min(pt[i + 1] - pt[i], qt[j + 1] - qt[j]) <= EPS:
                continue
            vp = (p[i + 1] - p[i]) / (pt[i + 1] - pt[i])
            vq = (q[j + 1] - q[j]) / (qt[j + 1] - qt[j])
            r = p[i] + vp * (a - pt[i]) - q[j] - vq * (a - qt[j])
            v = vp - vq
            low, high = 0.0, float(b - a)
            if abs(v[2]) <= EPS:
                if abs(r[2]) >= vertical_m - EPS:
                    continue
            else:
                za, zb = sorted(((-vertical_m - r[2]) / v[2], (vertical_m - r[2]) / v[2]))
                low, high = max(low, za), min(high, zb)
            if high < low:
                continue
            vv = float(np.dot(v[:2], v[:2]))
            best = float(np.clip(-np.dot(r[:2], v[:2]) / vv, low, high)) if vv > EPS else low
            if np.linalg.norm(r[:2] + best * v[:2]) < horizontal_m - EPS:
                return [{"rule_id": None, "rule_type": "SEPARATION", "reason": "simultaneous routes violate continuous separation", "time_s": float(a + best), "segment": i, "other_segment": j}]
    return []
