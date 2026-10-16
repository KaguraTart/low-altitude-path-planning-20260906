#!/usr/bin/env python3
"""Frozen, constructive v2 acceptance workloads and a measured SDK/HTTP runner.

The generator does not import or call the planner, scheduler, or motion builder.
Its reference flights stop at each business waypoint and use an independently
implemented analytic trapezoid. Ground waiting is outside airborne occupancy.
The deliberately conservative 160 s launch slots prove feasibility; they are
neither planner answers nor performance targets.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import platform
import random
import resource
import statistics
import sys
import time
import urllib.request
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any

import numpy as np
from shapely.geometry import LineString, Point, box
from shapely.ops import unary_union

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

SEED = 20261016
SUITES = ("strict", "mixed", "free")
FROZEN_DIRECTORY = ROOT / "input" / "v2_benchmarks"
TIME_ORIGIN = datetime(2026, 10, 16, 1, tzinfo=timezone.utc)
COORDINATES = {
    "origin_lon": 104.78, "origin_lat": 29.34, "origin_alt_msl": 0,
    "max_radius_m": 10000, "terrain_msl_m": 0,
    "terrain_source": "synthetic-flat-msl-v1",
}
HORIZONTAL_SEPARATION = 12.0
VERTICAL_SEPARATION = 12.0


def canonical_bytes(value: Any) -> bytes:
    """One stable UTF-8 representation, also used for every frozen digest."""
    return (json.dumps(value, sort_keys=True, separators=(",", ":"),
                       ensure_ascii=False, allow_nan=False) + "\n").encode("utf-8")


def sha256(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def timestamp(seconds: float) -> str:
    return (TIME_ORIGIN + timedelta(seconds=seconds)).isoformat()


def _frame() -> Any:
    # Coordinate conversion is shared public infrastructure, not route search.
    from uav_planner.v2.coordinates import CoordinateFrame
    return CoordinateFrame(COORDINATES)


def _profile(index: int) -> dict[str, Any]:
    return {
        "id": f"synthetic-{index}", "name": f"Synthetic engineering profile {index}",
        "max_speed_mps": [10, 11, 12, 13][index],
        "cruise_speed_mps": [5, 6, 7, 8][index],
        "climb_speed_mps": [2, 2.5, 3, 3.5][index],
        "descent_speed_mps": [2, 2.5, 3, 3.5][index],
        "acceleration_mps2": [1.5, 1.8, 2.0, 2.2][index],
        "deceleration_mps2": [1.5, 1.8, 2.0, 2.2][index],
        "max_yaw_rate_deg_s": 60, "radius_m": [.3, .4, .6, .8][index],
        "height_m": [.2, .3, .4, .5][index], "safety_margin_m": 5,
        "min_turn_radius_m": 0, "endurance_s": 1800,
        "max_range_m": 5000, "payload_kg": [.25, .5, 1, 2][index],
        "rtl_altitude_m": 100, "can_hover": True,
    }


def analytic_reference(anchors: list[list[float]], profile: dict[str, Any],
                       launch: float, mixed: bool) -> dict[str, Any]:
    """Construct rest-to-rest straight legs and physical stationary yaw turns.

    Acceleration and braking are independently integrated here. Phase vertices
    preserve exact times; analytic primitives retain the continuous time law.
    No engine or production motion code is used.
    """
    points: list[list[float]] = [list(anchors[0])]
    times = [float(launch)]
    primitives: list[dict[str, Any]] = []
    now = float(launch)
    for index, (start, end) in enumerate(zip(anchors, anchors[1:])):
        a, b = np.asarray(start, float), np.asarray(end, float)
        delta = b - a
        length = float(np.linalg.norm(delta))
        horizontal = float(np.linalg.norm(delta[:2]))
        cruise = min(profile["cruise_speed_mps"], 4.0 if mixed else math.inf)
        cap = cruise * length / horizontal if horizontal else math.inf
        if abs(delta[2]) > 1e-9:
            vertical = min(profile["climb_speed_mps"] if delta[2] > 0 else profile["descent_speed_mps"], 1.0 if mixed else math.inf)
            cap = min(cap, vertical * length / abs(delta[2]))
        acceleration = profile["acceleration_mps2"]
        deceleration = profile["deceleration_mps2"]
        peak = min(cap, math.sqrt(2 * length / (1 / acceleration + 1 / deceleration)))
        ta, td = peak / acceleration, peak / deceleration
        sa, sd = peak * ta / 2, peak * td / 2
        tc = max(0.0, (length - sa - sd) / peak)
        duration = ta + tc + td
        primitive = {
            "kind": "line", "start": start, "end": end,
            "start_time_s": now, "end_time_s": now + duration,
            "length_m": length, "peak_speed_mps": peak,
            "acceleration_mps2": acceleration, "deceleration_mps2": deceleration,
            "accel_time_s": ta, "cruise_time_s": tc, "decel_time_s": td,
        }
        primitives.append(primitive)
        for fraction, offset in ((sa / length, ta), ((length - sd) / length, ta + tc), (1.0, duration)):
            if now + offset > times[-1] + 1e-9:
                points.append((a + fraction * delta).tolist())
                times.append(now + offset)
        now += duration
        if index + 2 < len(anchors):
            following = np.asarray(anchors[index + 2], float) - b
            if horizontal > 1e-8 and np.linalg.norm(following[:2]) > 1e-8:
                angle = math.degrees(math.acos(float(np.clip(np.dot(delta[:2], following[:2]) / horizontal / np.linalg.norm(following[:2]), -1, 1))))
                if angle > 1e-6:
                    dwell = angle / profile["max_yaw_rate_deg_s"]
                    primitives.append({"kind": "yaw", "start": end, "end": end,
                                       "start_time_s": now, "end_time_s": now + dwell})
                    now += dwell
                    points.append(list(end)); times.append(now)
    return {"local_points": points, "vertex_times_s": times, "motion_primitives": primitives}


def _geo(frame: Any, geometry: Any) -> str:
    return frame.to_wgs84(geometry, "MSL", TIME_ORIGIN)


def _grid(frame: Any, point: list[float]) -> dict[str, float]:
    lon, lat = frame._inverse.transform(point[0], point[1])
    return {"minLon": lon - .00001, "maxLon": lon + .00001,
            "minLat": lat - .00001, "maxLat": lat + .00001,
            "minAlt": point[2] - 10, "maxAlt": point[2] + 10}


def _mixed_rules() -> list[dict[str, Any]]:
    definitions = {
        "HEIGHT_LAYER": ({"minHeight": 60, "maxHeight": 140, "altDatum": "MSL"}, []),
        "WIDTH": ({"width": 20, "bufferWidth": 1}, []),
        "DIRECTION": ({"direction": "ONE_WAY"}, []),
        "SPEED": ({"maxSpeed": 4, "cruiseSpeed": 4}, []),
        "TIME": ({"dailyStart": "09:00", "dailyEnd": "10:00", "weekdays": [5], "latestArrival": "09:59"}, []),
        "AREA": ({"scope": "INCLUDE"}, ["allowed-route-bands"]),
        "CLIMB": ({"maxClimbRate": 1, "maxDescentRate": 1}, []),
        "TURN": ({"maxTurnAngle": 100, "minTurnRadius": 0}, []),
        "SEPARATION": ({"horizontalSeparation": 12, "verticalSeparation": 12}, []),
        "NO_FLY": ({"noFlyMode": "PERMANENT", "includeBuffer": True}, ["no-fly-0"]),
        "BUILDING": ({"avoidMode": "BUFFER", "safeDistance": 8}, []),
        "ZONE": ({"zoneLevels": ["restricted"], "coverageThreshold": 0}, ["zone-0"]),
        "WEATHER": ({"maxWindSpeed": 8, "noPrecipitation": True, "noIcing": True}, ["weather-v1"]),
        "SIGNAL": ({"minSignalStrength": -85, "bands": ["2.4GHz", "5.8GHz"], "band_policy": "all"}, ["signal-v1"]),
        "PRIORITY": ({"priorityScene": "QUIET", "costBonus": .8}, ["noise-v1"]),
        "HOVER": ({"maxHoverTime": 1.6, "defaultHoverTime": 1.5}, []),
        "ENDURANCE": ({"maxRange": 600, "minEndurance": 1200}, []),
        "PAYLOAD": ({"maxPayloadKg": .1}, []),
        "WAYPOINT": ({"maxWaypoints": 4, "minWaypointInterval": 10}, []),
        "RTL": ({"rtlAltitude": 100, "rtlOnLowBattery": True, "lowBatteryPercent": 20}, []),
    }
    return [{"id": f"rule-{kind}", "rule_type": kind, "content": content,
             "resource_ids": resources} for kind, (content, resources) in definitions.items()]


def generate_suite(suite: str) -> dict[str, Any]:
    if suite not in SUITES:
        raise ValueError(f"Unknown suite: {suite}")
    frame, randomizer = _frame(), random.Random(SEED)
    mixed = suite == "mixed"
    scene: dict[str, Any] = {
        "coordinates": COORDINATES,
        "bounds": {"x_min": 0, "x_max": 5000, "y_min": 0, "y_max": 5000, "z_min": 0, "z_max": 300},
        "resolution": [25, 25, 10], "horizon_s": 3600,
        "time_origin": TIME_ORIGIN.isoformat(), "airways": [], "obstacles": [],
        "control_zones": [], "rules": _mixed_rules() if mixed else [], "datasets": [],
        "profiles": [_profile(i) for i in range(4)],
    }
    tasks, references, groups, route_bands, bad_bounds = [], [], [], [], []
    for group in range(100):
        column, row = group % 10, group // 10
        x, y = column * 500 + 70, row * 500 + 180
        length = randomizer.randrange(300, 381, 10)
        anchors: list[list[float]] = [[x, y, 100], [x + length, y, 100]]
        category = "shared-straight"
        if mixed and 10 <= group < 20:
            anchors = [[x, y, 80], [x, y, 100], [x + length, y, 100]]
            category = "vertical-climb"
        elif mixed and 20 <= group < 30:
            anchors = [[x, y, 100], [x + length / 2, y, 100], [x + length, y, 100]]
            category = "business-waypoint"
        elif mixed and 90 <= group < 100:
            anchors = [[x, y, 100], [x + length - 80, y, 100], [x + length - 80, y + 80, 100]]
            category = "right-angle-yaw-hover"
        profile = scene["profiles"][group % 4]
        airway_id = f"shared-{group:03d}"
        line = LineString(anchors)
        scene["airways"].append({
            "id": airway_id, "centerLine": _geo(frame, line), "widthMeters": 60,
            "minAltitude": 60, "maxAltitude": 140, "altReference": "MSL",
            "startGrid": _grid(frame, anchors[0]), "endGrid": _grid(frame, anchors[-1]),
            "direction": "FORWARD", "status": "ACTIVE",
        })
        route_bands.append(line.buffer(35))
        bad = [column * 500 + 210, row * 500 + 350, 40,
               column * 500 + 250, row * 500 + 390, 160]
        bad_bounds.append(bad)
        scene["obstacles"].append({
            "id": f"building-{group}", "geometry_wkt": _geo(frame, box(bad[0], bad[1], bad[3], bad[4])),
            "min_altitude": 40, "max_altitude": 160, "alt_reference": "MSL", "buffer_m": 2,
        })
        if group < 10:
            for zone_kind, shift, level in (("no-fly", 60, "prohibited"), ("zone", 120, "restricted")):
                scene["control_zones"].append({
                    "id": f"{zone_kind}-{group}", "geom": _geo(frame, box(bad[0] + shift, bad[1], bad[3] + shift, bad[4])),
                    "minAlt": 40, "maxAlt": 160, "altReference": "MSL",
                    "zoneType": zone_kind, "level": level,
                })
        group_ids = []
        for member in range(10):
            task_id = f"{suite}-{group:03d}-{member:02d}"
            route_mode = "strict" if suite == "strict" else "free" if suite == "free" else ("strict", "prefer", "free")[group % 3]
            task = {
                "id": task_id, "start_wkt": _geo(frame, Point(anchors[0])),
                "goal_wkt": _geo(frame, Point(anchors[-1])),
                "departure_time": timestamp(0), "latest_arrival": timestamp(3500),
                "alt_reference": "MSL", "route_mode": route_mode, "profile_id": profile["id"],
                "payload_kg": .1, "priority": 0,
                "waypoints": [{"wkt": _geo(frame, Point(p)), "hover_s": 1.5 if category == "right-angle-yaw-hover" else 0}
                              for p in anchors[1:-1]],
            }
            if mixed:
                task["state"] = {"battery_percent": 80, "remaining_flight_time_s": 1700,
                                 "remaining_range_m": 4500, "home_wkt": task["goal_wkt"], "alt_reference": "MSL"}
                if group == 80:
                    task["state"]["battery_percent"] = 10
                    task["goal_wkt"] = _geo(frame, Point([anchors[-1][0], anchors[-1][1] + 80, 100]))
            reference = analytic_reference(anchors, profile, member * 160, mixed)
            reference.update({"task_id": task_id, "group_id": airway_id,
                              "effective_goal_wkt": task["state"]["home_wkt"] if mixed and group == 80 else task["goal_wkt"],
                              "trajectory_wkt": _geo(frame, LineString(reference["local_points"]))})
            tasks.append(task); references.append(reference); group_ids.append(task_id)
        groups.append({"id": airway_id, "task_ids": group_ids, "anchors_local": anchors,
                       "category": category, "reference_launch_slot_s": 160})
    if mixed:
        extent = {"x_min": 0, "x_max": 5000, "y_min": 0, "y_max": 5000, "z_min": 0, "z_max": 300}
        for kind, safe, unsafe in (
            ("weather", {"wind_speed_mps": 2, "precipitation": 0, "icing": False}, {"wind_speed_mps": 15, "precipitation": 1, "icing": True}),
            ("signal", {"bands": {"2.4GHz": -60, "5.8GHz": -65}}, {"bands": {"2.4GHz": -105, "5.8GHz": -110}}),
            ("noise", {"noise_db": 30}, {"noise_db": 95}),
        ):
            cells = [{"bounds": extent, "values": safe}]
            for xmin, ymin, zmin, xmax, ymax, zmax in bad_bounds:
                cells.append({"bounds": {"x_min": xmin, "x_max": xmax, "y_min": ymin, "y_max": ymax, "z_min": zmin, "z_max": zmax}, "values": unsafe})
            scene["datasets"].append({"id": f"{kind}-v1", "kind": kind, "version": "frozen-20261016-v1",
                                      "valid_from": timestamp(0), "valid_to": timestamp(3600), "cells": cells})
        scene["datasets"].append({"id": "allowed-route-bands", "kind": "area", "version": "frozen-20261016-v1",
                                  "geometry_wkt": _geo(frame, unary_union(route_bands)), "cells": []})
    result = {"format_version": 1, "seed": SEED, "suite": suite, "scene": scene,
              "tasks": tasks, "reference_trajectories": references,
              "construction": {
                  "algorithm": "100 geographically separated shared corridors, ten concurrent requests each; independent rest-to-rest analytic legs and 160s launch slots",
                  "groups": groups, "ground_wait_occupies_airspace": False,
                  "initial_departure_s": 0, "minimum_initial_conflicted_task_count": 1000,
                  "body_safety_margin_m": 5, "reference_horizontal_separation_m": HORIZONTAL_SEPARATION,
                  "reference_vertical_separation_m": VERTICAL_SEPARATION,
                  "reference_uses_planner": False,
              }}
    if mixed:
        result["rule_effect_witnesses"] = _witnesses(result)
    return result


def _witnesses(workload: dict[str, Any]) -> list[dict[str, Any]]:
    """Executable counterexamples prevent ALL20 coverage being mere registration."""
    a, b = [70, 180, 100], [370, 180, 100]
    cases: dict[str, dict[str, Any]] = {
        "HEIGHT_LAYER": {"points": [[70, 180, 200], [370, 180, 200]], "times": [0, 100]},
        "WIDTH": {"points": [[70, 200, 100], [370, 200, 100]], "times": [0, 100]},
        "DIRECTION": {"points": [b, a], "times": [0, 100]},
        "SPEED": {"points": [a, b], "times": [0, 30]},
        "TIME": {"points": [a, b], "times": [3550, 3650]},
        "AREA": {"points": [[70, 300, 100], [370, 300, 100]], "times": [0, 100]},
        "CLIMB": {"points": [[70, 180, 80], [70, 180, 100]], "times": [0, 2]},
        "TURN": {"points": [a, [200, 180, 100], [100, 180, 100]], "times": [0, 50, 100]},
        "SEPARATION": {"kind": "pair", "task_ids": [workload["tasks"][0]["id"], workload["tasks"][1]["id"]]},
        "NO_FLY": {"points": [[280, 370, 100], [300, 370, 100]], "times": [0, 20]},
        "BUILDING": {"points": [[195, 370, 100], [205, 370, 100]], "times": [0, 20]},
        "ZONE": {"points": [[340, 370, 100], [360, 370, 100]], "times": [0, 20]},
        "WEATHER": {"points": [[215, 370, 100], [245, 370, 100]], "times": [0, 20]},
        "SIGNAL": {"points": [[215, 370, 100], [245, 370, 100]], "times": [0, 20]},
        "PRIORITY": {"kind": "cost", "quiet_point": [200, 180, 100], "noisy_point": [230, 370, 100]},
        "HOVER": {"points": [a, a, b], "times": [0, 5, 105]},
        "ENDURANCE": {"points": [a, [800, 180, 100]], "times": [0, 300]},
        "PAYLOAD": {"points": [a, b], "times": [0, 100], "task_patch": {"payload_kg": .2}},
        "WAYPOINT": {"points": [a, b], "times": [0, 100], "task_patch": {"waypoints": [{"local": [75, 180, 100]}]}},
        "RTL": {"kind": "rtl", "task_patch": {"state": {"battery_percent": 10, "remaining_flight_time_s": 1700,
                                                                      "remaining_range_m": 4500, "home_wkt": workload["tasks"][0]["goal_wkt"], "alt_reference": "MSL"}}},
    }
    return [{"rule_type": kind, "task_id": workload["tasks"][0]["id"], "kind": "violation", **case}
            for kind, case in cases.items()]


def freeze(directory: Path = FROZEN_DIRECTORY) -> dict[str, Any]:
    """Create once; changed generated bytes or modified frozen files are fatal."""
    generated = {f"{suite}.json": canonical_bytes(generate_suite(suite)) for suite in SUITES}
    manifest = {"format_version": 1, "seed": SEED, "frozen_at": "2026-10-16T01:00:00+00:00",
                "canonicalization": "UTF-8, sorted keys, compact separators, newline, no NaN",
                "files": {name: {"sha256": sha256(data), "bytes": len(data), "task_count": 1000}
                          for name, data in generated.items()}}
    generated["manifest.json"] = canonical_bytes(manifest)
    directory.mkdir(parents=True, exist_ok=True)
    for name, data in generated.items():
        destination = directory / name
        if destination.exists() and destination.read_bytes() != data:
            raise ValueError(f"Frozen benchmark differs: {destination}; refusing overwrite")
    for name, data in generated.items():
        destination = directory / name
        if not destination.exists():
            with destination.open("xb") as handle:
                handle.write(data)
    return manifest


def load_frozen(suite: str, directory: Path = FROZEN_DIRECTORY) -> tuple[dict[str, Any], str]:
    manifest_bytes = (directory / "manifest.json").read_bytes()
    manifest = json.loads(manifest_bytes)
    if canonical_bytes(manifest) != manifest_bytes or manifest["seed"] != SEED:
        raise ValueError("Noncanonical or wrong-seed benchmark manifest")
    for name, expected in manifest["files"].items():
        data = (directory / name).read_bytes()
        if sha256(data) != expected["sha256"] or len(data) != expected["bytes"]:
            raise ValueError(f"Frozen benchmark hash mismatch: {name}")
    raw = (directory / f"{suite}.json").read_bytes()
    workload = json.loads(raw)
    if canonical_bytes(workload) != raw:
        raise ValueError("Benchmark JSON is not canonical")
    return workload, sha256(raw)


def _initial_paths(workload: dict[str, Any]) -> tuple[list[np.ndarray], list[np.ndarray]]:
    paths, times = [], []
    for reference in workload["reference_trajectories"]:
        paths.append(np.asarray(reference["local_points"], float))
        timing = np.asarray(reference["vertex_times_s"], float)
        times.append(timing - timing[0])
    return paths, times


def validate_rule_witnesses(workload: dict[str, Any], compiled: Any) -> dict[str, Any]:
    from uav_planner.v2.models import Task
    from uav_planner.v2 import native
    tasks = {task["id"]: task for task in workload["tasks"]}
    checked: dict[str, Any] = {}
    for witness in workload.get("rule_effect_witnesses", []):
        kind = witness["rule_type"]
        task_value = copy.deepcopy(tasks[witness["task_id"]])
        task_value.update(witness.get("task_patch", {}))
        task_value["rule_ids"] = [f"rule-{kind}"]
        for waypoint in task_value.get("waypoints", []):
            if "local" in waypoint:
                waypoint["wkt"] = _geo(compiled.frame, Point(waypoint.pop("local")))
        task = Task.model_validate(task_value)
        profile = next(p for p in compiled.scene.profiles if p.id == task.profile_id)
        if witness["kind"] == "cost":
            quiet = compiled.cost_multiplier(witness["quiet_point"], 0, task, profile)
            noisy = compiled.cost_multiplier(witness["noisy_point"], 0, task, profile)
            passed, detail = quiet < noisy, {"quiet_cost": quiet, "noisy_cost": noisy}
        elif witness["kind"] == "rtl":
            detail = compiled.rules.rtl_requirements(task, profile)
            passed = detail["required"] and detail["rtl_altitude_m"] == 100
        elif witness["kind"] == "pair":
            paths, timings = _initial_paths(workload)
            detail = native.continuous_conflicts(paths[:2], timings[:2], HORIZONTAL_SEPARATION, VERTICAL_SEPARATION)
            passed = bool(detail["conflicts"])
            detail = {"conflict_count": len(detail["conflicts"])}
        else:
            points, timing = np.asarray(witness["points"], float), np.asarray(witness["times"], float)
            detail = compiled.rules.check_path(points, timing, task, profile)
            detail += [item for i in range(len(points) - 1) for item in
                       compiled.rules.check_segment(points[i], points[i + 1], timing[i], timing[i + 1], task, profile, i)]
            passed = any(item.get("rule_type") == kind for item in detail)
        if not passed:
            raise AssertionError(f"ALL20 benchmark witness has no effect: {kind}: {detail}")
        checked[kind] = detail
    if workload["suite"] == "mixed" and len(checked) != 20:
        raise AssertionError("Mixed benchmark must execute twenty effective witnesses")
    return checked


def validate_reference(workload: dict[str, Any]) -> dict[str, Any]:
    """Check every constructive reference and continuous pair occupancy."""
    from uav_planner.v2.models import Scene, Task
    from uav_planner.v2.scene import CompiledScene
    from uav_planner.v2 import native
    model = Scene.model_validate(workload["scene"])
    compiled = CompiledScene(model)
    tasks = {task["id"]: Task.model_validate(task) for task in workload["tasks"]}
    paths, timings = [], []
    if len(tasks) != 1000 or len(workload["reference_trajectories"]) != 1000:
        raise AssertionError("Each suite needs exactly 1000 different feasible tasks")
    for reference in workload["reference_trajectories"]:
        task = tasks[reference["task_id"]]
        profile = next(p for p in model.profiles if p.id == task.profile_id)
        points = np.asarray(reference["local_points"], float)
        times = np.asarray(reference["vertex_times_s"], float)
        errors = compiled.check_path(points, times, task, profile)
        if errors:
            raise AssertionError(f"Invalid constructive reference {task.id}: {errors[:4]}")
        local = np.asarray(compiled.frame.to_local(reference["trajectory_wkt"], "MSL", TIME_ORIGIN).coords)
        if not np.allclose(local, points, atol=.0001, rtol=0):
            raise AssertionError(f"Reference geographic/local mismatch: {task.id}")
        departure = (task.departure_time - TIME_ORIGIN).total_seconds()
        latest = (task.latest_arrival - TIME_ORIGIN).total_seconds() if task.latest_arrival else model.horizon_s
        if times[0] < departure or times[-1] > latest:
            raise AssertionError(f"Reference violates task time window: {task.id}")
        paths.append(points); timings.append(times)
    conflicts = native.continuous_conflicts(paths, timings, HORIZONTAL_SEPARATION, VERTICAL_SEPARATION)
    if conflicts["conflicts"]:
        raise AssertionError(f"Constructive references collide: {conflicts['conflicts'][:2]}")
    original_paths, original_times = _initial_paths(workload)
    initial = native.continuous_conflicts(original_paths, original_times, HORIZONTAL_SEPARATION,
                                         VERTICAL_SEPARATION, max_conflicts=100000)
    conflicted = {c[key] for c in initial["conflicts"] for key in ("first", "second")}
    if len(conflicted) < 200:
        raise AssertionError("Benchmark is insufficiently contentious: fewer than 200 conflicted tasks")
    witnesses = validate_rule_witnesses(workload, compiled)
    return {"all_references_valid": True, "validated_reference_count": len(paths),
            "reference_conflict_count": 0, "initial_conflicted_task_count": len(conflicted),
            "initial_conflict_count": len(initial["conflicts"]), "rule_effect_witnesses": witnesses}


def sample_motion_segments(route: dict[str, Any], error_m: float = .02) -> tuple[np.ndarray, np.ndarray]:
    """Independently sample exported analytic primitives with chord error bound.

    The returned cover is tested at separation + 2*error, covering both paths'
    interpolation error. It does not trust route export phase chords at turns or
    under acceleration, and does not call the production collision sampler.
    """
    points, times = [], []
    for segment in route.get("motion_segments", []):
        start, end = np.asarray(segment["start"], float), np.asarray(segment["end"], float)
        t0, t1 = segment["start_time_s"], segment["end_time_s"]
        duration = t1 - t0
        data = segment.get("profile", {})
        peak = segment.get("peak_speed_mps")
        peak = data.get("peak_speed_mps", peak) or 0
        acceleration = segment.get("acceleration_mps2") or 0
        deceleration = segment.get("deceleration_mps2") or data.get("deceleration_mps2", 0)
        ta = segment.get("accel_time_s") or data.get("accel_time_s", 0)
        tc = segment.get("cruise_time_s") or data.get("cruise_time_s", 0)
        v0 = segment.get("start_speed_mps") or 0
        length = segment.get("length_m", 0)
        radius = segment.get("radius_m") or 0
        bound = max(acceleration, deceleration) + (peak * peak / radius if radius else 0)
        step = min(2.0, math.sqrt(8 * error_m / max(bound, 1e-9)))
        samples = sorted(set([0.0, duration, min(ta, duration), min(ta + tc, duration)] +
                             [i * step for i in range(1, math.ceil(duration / step))]))
        for offset in samples:
            if times and t0 + offset <= times[-1] + 1e-9:
                continue
            if offset <= ta:
                distance = v0 * offset + .5 * acceleration * offset * offset
            elif offset <= ta + tc:
                distance = (v0 + peak) * ta * .5 + peak * (offset - ta)
            else:
                q = offset - ta - tc
                distance = (v0 + peak) * ta * .5 + peak * tc + peak * q - .5 * deceleration * q * q
            fraction = min(1, max(0, distance / length)) if length > 1e-9 else 0
            point = start + fraction * (end - start)
            if segment["kind"] == "arc":
                center = np.asarray(segment["center"], float)
                angle = math.atan2(start[1] - center[1], start[0] - center[0])
                sweep = data.get("sweep_angle_deg", segment.get("sweep_angle_deg", 0))
                angle += fraction * math.radians(sweep)
                point[:2] = center[:2] + radius * np.array([math.cos(angle), math.sin(angle)])
            points.append(point); times.append(t0 + offset)
    if not points:
        raise AssertionError(f"Missing analytic motion segments for planned route {route['task_id']}")
    return np.asarray(points), np.asarray(times)


def certify_exported_motion(route: dict[str, Any], limits: dict[str, float]) -> dict[str, Any]:
    """Rebuild a native certificate from public JSON, independent of Motion."""
    from uav_planner.v2 import native
    primitives, starts = [], []
    for segment in route["motion_segments"]:
        data = segment.get("profile", {})
        primitives.append({
            "kind": segment["kind"], "start": segment["start"], "end": segment["end"],
            "center": segment.get("center"), "radius": segment.get("radius_m") or 0,
            "length": segment["length_m"],
            "duration": segment["end_time_s"] - segment["start_time_s"],
            "v0": segment.get("start_speed_mps") or 0, "v1": segment.get("end_speed_mps") or 0,
            "peak": data.get("peak_speed_mps", segment.get("peak_speed_mps")) or 0,
            "accel": segment.get("acceleration_mps2") or 0,
            "decel": data.get("deceleration_mps2", segment.get("deceleration_mps2")) or 0,
            "ta": data.get("accel_time_s", segment.get("accel_time_s")) or 0,
            "tc": data.get("cruise_time_s", segment.get("cruise_time_s")) or 0,
            "td": data.get("decel_time_s", segment.get("decel_time_s")) or 0,
            "sweep": math.radians(data.get("sweep_angle_deg", segment.get("sweep_angle_deg")) or 0),
        })
        starts.append(segment["start_time_s"])
    certificate = native.validate_motion(primitives, limits, starts=starts)
    if not certificate["valid"]:
        raise AssertionError(f"Exported motion certificate failed: {route['task_id']}: {certificate['violations']}")
    return certificate


def validate_result(workload: dict[str, Any], result: dict[str, Any], count: int,
                    scene_id: str | None = None) -> dict[str, Any]:
    from uav_planner.v2 import native
    from uav_planner.v2.engine import registry, register_scene
    from uav_planner.v2.models import Task
    if scene_id is None:
        scene_id = register_scene(workload["scene"])["scene_id"]
    model, compiled, _, _ = registry.get(scene_id)
    tasks = {t["id"]: Task.model_validate(t) for t in workload["tasks"][:count]}
    routes = result["routes"]
    if (len(routes) != count or result["success_count"] != count or result["failed_count"] != 0
            or {r["task_id"] for r in routes} != set(tasks)):
        failures = [{"task_id": r["task_id"], "code": r.get("code"), "message": r.get("message"),
                     "validation": r.get("validation")} for r in routes if r["status"] != "planned"]
        raise AssertionError(f"Acceptance requires every requested route, success {result['success_count']}/{count}: {failures[:4]}")
    paths, timings = [], []
    for route in routes:
        task = tasks[route["task_id"]]
        profile = next(p for p in model.profiles if p.id == task.profile_id)
        if route["status"] != "planned" or not route["validation"]["valid"]:
            raise AssertionError(f"Route not validated: {task.id}")
        certify_exported_motion(route, compiled.rules.limits(task, profile))
        points = np.asarray(compiled.frame.to_local(route["trajectory_wkt"], task.alt_reference, task.departure_time).coords)
        times = np.asarray(route["vertex_times_s"], float)
        rtl = compiled.rules.rtl_requirements(task, profile)
        errors = compiled.check_path(points, times, task, profile,
                                     phase="rtl" if rtl and rtl["required"] else "route")
        if errors:
            raise AssertionError(f"Independent route revalidation failed: {task.id}: {errors[:3]}")
        expected_start = np.asarray(compiled.frame.to_local(task.start_wkt, task.alt_reference, task.departure_time).coords[0])
        effective_goal = task.state.home_wkt if rtl and rtl["required"] else task.goal_wkt
        expected_end = np.asarray(compiled.frame.to_local(effective_goal, task.alt_reference, task.departure_time).coords[0])
        if np.linalg.norm(points[0] - expected_start) > .01 or np.linalg.norm(points[-1] - expected_end) > .01:
            raise AssertionError(f"Route endpoints differ from task: {task.id}")
        departure = (task.departure_time - TIME_ORIGIN).total_seconds()
        latest = (task.latest_arrival - TIME_ORIGIN).total_seconds() if task.latest_arrival else model.horizon_s
        earliest = (task.earliest_arrival - TIME_ORIGIN).total_seconds() if task.earliest_arrival else 0
        if times[0] < departure - 1e-7 or times[-1] > latest + 1e-7 or times[-1] < earliest - 1e-7:
            raise AssertionError(f"Route violates task time window: {task.id}")
        sampled, sampled_times = sample_motion_segments(route)
        paths.append(sampled); timings.append(sampled_times)
    active_profiles = [next(p for p in model.profiles if p.id == task.profile_id) for task in tasks.values()]
    h = max([2 * (p.radius_m + p.safety_margin_m) for p in active_profiles] +
            [compiled.rules.separation(task)[0] for task in tasks.values()])
    v = max([p.height_m + 2 * p.safety_margin_m for p in active_profiles] +
            [compiled.rules.separation(task)[1] for task in tasks.values()])
    pair_check = native.continuous_conflicts(paths, timings, h + .04, v + .04)
    if pair_check["conflicts"] or result.get("conflicts"):
        raise AssertionError(f"Independent continuous conflicts: {pair_check['conflicts'][:3]}")
    return {"validated_route_count": count, "continuous_conflict_count": 0,
            "native_motion_certificate_count": count,
            "collision_chord_error_m": .02, "horizontal_separation_checked_m": h + .04,
            "vertical_separation_checked_m": v + .04,
            "distance_m": sum(r["metrics"]["distance_m"] for r in routes),
            "flight_time_s": sum(r["metrics"]["flight_time_s"] for r in routes),
            "wait_time_s": sum(r["metrics"]["wait_time_s"] for r in routes),
            "total_route_vertices": sum(len(r["vertex_times_s"]) for r in routes)}


def _http_json(base_url: str, path: str, body: Any, timeout: float) -> dict[str, Any]:
    request = urllib.request.Request(base_url.rstrip("/") + path, canonical_bytes(body),
                                     {"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.load(response)


def run_suite(suite: str, repeats: int, scales: list[int], output: Path,
              time_budget_s: float = 120, base_url: str | None = None,
              directory: Path = FROZEN_DIRECTORY) -> dict[str, Any]:
    from uav_planner.v2 import native
    from uav_planner.v2.engine import plan_batch_v2, register_scene
    workload, digest = load_frozen(suite, directory)
    reference = validate_reference(workload)
    started = time.perf_counter()
    registered = (_http_json(base_url, "/api/v2/scenes", workload["scene"], time_budget_s) if base_url
                  else register_scene(workload["scene"]))
    registration_ms = (time.perf_counter() - started) * 1000
    scene_id = registered["scene_id"]
    local_scene_id = register_scene(workload["scene"])["scene_id"] if base_url else scene_id
    service_version=None
    if base_url:
        with urllib.request.urlopen(base_url.rstrip("/")+"/api/v2/version",timeout=10) as response:
            service_version=json.load(response)
    output.mkdir(parents=True, exist_ok=True)
    report: dict[str, Any] = {
        "suite": suite, "seed": SEED, "input_sha256": digest,
        "frozen_manifest_sha256": sha256((directory / "manifest.json").read_bytes()),
        "mode": "http" if base_url else "sdk", "scene_registration_ms": registration_ms,
        "scene_version": registered.get("scene_version"), "native": native.engine_info(),
        "service_version":service_version,
        "reference_validation": reference, "runs": [], "summary": {},
        "environment": {"python": sys.version, "platform": platform.platform(),
                        "cpu_count": __import__("os").cpu_count(),
                        "timed_boundary": "request creation + plan_batch_v2 + complete JSON serialization; scene preloaded; independent validation excluded"},
    }
    report_path = output / f"{suite}-report.json"
    for count in scales:
        if count not in (10, 100, 1000):
            raise ValueError("Accepted scales are 10, 100, and 1000")
        for repeat in range(repeats):
            start = time.perf_counter()
            request = {"scene_id": scene_id, "tasks": workload["tasks"][:count],
                       "time_budget_s": time_budget_s, "request_id": f"frozen-{suite}-{count}-{repeat}"}
            result = (_http_json(base_url, "/api/v2/plan/batch", request, time_budget_s + 10) if base_url
                      else plan_batch_v2(**request))
            serialized = canonical_bytes(result)
            elapsed_ms = (time.perf_counter() - start) * 1000
            run = {"scale": count, "repeat": repeat, "elapsed_ms": elapsed_ms,
                   "server_stage_timings_ms": result.get("stage_timings_ms", {}),
                   "success_count": result["success_count"], "failed_count": result["failed_count"],
                   "serialized_bytes": len(serialized), "result_sha256": sha256(serialized),
                   "peak_rss_mib": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024}
            try:
                run["validation"] = validate_result(workload, result, count, local_scene_id)
                run["accepted"] = True
            except (AssertionError, ValueError) as exc:
                run["accepted"] = False; run["error"] = str(exc)
                (output / f"{suite}-{count}-{repeat}-failed.json").write_bytes(serialized)
            report["runs"].append(run)
            report_path.write_bytes(canonical_bytes(report))
            print(json.dumps({"suite": suite, **run}, ensure_ascii=False), flush=True)
        runs = [run for run in report["runs"] if run["scale"] == count]
        durations = [run["elapsed_ms"] for run in runs]
        report["summary"][str(count)] = {
            "runs": len(runs), "accepted_runs": sum(run["accepted"] for run in runs),
            "median_ms": statistics.median(durations), "min_ms": min(durations), "max_ms": max(durations),
            "p95_ms": float(np.percentile(durations, 95)), "peak_rss_mib": max(run["peak_rss_mib"] for run in runs),
            "all_under_10s": all(run["elapsed_ms"] < 10000 and run["accepted"] for run in runs),
        }
        report_path.write_bytes(canonical_bytes(report))
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--freeze", action="store_true", help="Generate canonical inputs once, refusing any changed overwrite")
    parser.add_argument("--suite", choices=SUITES, action="append")
    parser.add_argument("--repeats", type=int, default=10)
    parser.add_argument("--scales", nargs="+", type=int, default=[10, 100, 1000])
    parser.add_argument("--output", type=Path, default=ROOT / "output" / "v2_benchmarks")
    parser.add_argument("--input-directory", type=Path, default=FROZEN_DIRECTORY)
    parser.add_argument("--time-budget-s", type=float, default=120)
    parser.add_argument("--base-url", help="Optional HTTP API origin; SDK is the default")
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args()
    if args.freeze:
        print(json.dumps(freeze(args.input_directory), indent=2)); return 0
    if args.repeats < 1:
        parser.error("--repeats must be positive")
    accepted = True
    for suite in args.suite or list(SUITES):
        if args.validate_only:
            workload, digest = load_frozen(suite, args.input_directory)
            print(json.dumps({"suite": suite, "sha256": digest, **validate_reference(workload)}, ensure_ascii=False))
        else:
            report = run_suite(suite, args.repeats, args.scales, args.output,
                               args.time_budget_s, args.base_url, args.input_directory)
            accepted &= all(run["accepted"] and (run["scale"] != 1000 or run["elapsed_ms"] < 10000)
                            for run in report["runs"])
    return 0 if accepted else 1


if __name__ == "__main__":
    raise SystemExit(main())
