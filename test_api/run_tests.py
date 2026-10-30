#!/usr/bin/env python3
"""Test the v2 service and save complete HTTP records and reports."""
from __future__ import annotations

import argparse
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import sys
import time
from typing import Any, Callable

if __package__:
    from .client import HTTPClient, normalize_base_url
    from .reporting import write_report
else:
    from client import HTTPClient, normalize_base_url
    from reporting import write_report


ROOT = Path(__file__).resolve().parent
OPERATIONS = (
    ("GET", "/api/v2/health"), ("GET", "/api/v2/health/jobs"),
    ("GET", "/api/v2/version"), ("GET", "/api/v2/capabilities"),
    ("GET", "/api/v2/examples/single"), ("GET", "/api/v2/examples/batch"),
    ("POST", "/api/v2/scenes"), ("GET", "/api/v2/scenes/{scene_id}"),
    ("POST", "/api/v2/coordinates/convert"), ("POST", "/api/v2/plan/single"),
    ("POST", "/api/v2/plan/batch"), ("POST", "/api/v2/validate"),
    ("POST", "/api/v2/jobs"), ("GET", "/api/v2/jobs/{job_id}"),
    ("POST", "/api/v2/networks/analyze"), ("POST", "/api/v2/plan/replan"),
)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def verify_data(data_dir: Path) -> dict[str, Any]:
    """Verify every declared fixture byte count and SHA-256."""
    manifest = load_json(data_dir / "manifest.json")
    require(bool(manifest.get("files")), "Data manifest has no files")
    declared: set[str] = set()
    for item in manifest["files"]:
        relative = item["path"]
        path = (data_dir / relative).resolve()
        require(path.is_relative_to(data_dir.resolve()), f"Fixture path leaves data directory: {relative}")
        require(relative not in declared, f"Duplicate fixture path: {relative}")
        declared.add(relative)
        content = path.read_bytes()
        require(len(content) == item["bytes"], f"Fixture byte count differs: {relative}")
        require(hashlib.sha256(content).hexdigest() == item["sha256"], f"Fixture SHA-256 differs: {relative}")
        if path.suffix == ".json":
            json.loads(content)
    for example in manifest.get("examples", []):
        require(example["path"] in declared, f"Example is missing from file manifest: {example['path']}")
    for feature in manifest.get("features", []):
        require(feature["path"] in declared, f"Feature is missing from file manifest: {feature['path']}")
    for benchmark in manifest.get("benchmarks", []):
        for name in ("scene", "tasks"):
            require(benchmark[name] in declared, f"Benchmark is missing from file manifest: {benchmark[name]}")
    return manifest


def wkt_vertices(value: str) -> list[tuple[float, float, float]]:
    """Read the POINT Z and LINESTRING Z formats used by service routes."""
    require(isinstance(value, str), "Trajectory WKT must be a string")
    match = re.fullmatch(r"\s*(POINT|LINESTRING)\s+Z\s*\(([^()]+)\)\s*", value, re.IGNORECASE)
    require(match is not None, "Route must contain POINT Z or LINESTRING Z")
    coordinates = []
    for vertex in match.group(2).split(","):
        numbers = tuple(float(number) for number in vertex.split())
        require(len(numbers) == 3 and all(math.isfinite(number) for number in numbers),
                "Each WKT vertex must have three finite coordinates")
        coordinates.append(numbers)
    require(len(coordinates) == 1 if match.group(1).upper() == "POINT" else len(coordinates) >= 2,
            "WKT vertex count does not match its geometry type")
    return coordinates


def check_route(route: dict[str, Any], task: dict[str, Any]) -> None:
    require(route.get("task_id") == task["id"], f"Route task ID differs: {task['id']}")
    require(route.get("alt_reference") == task["alt_reference"], "Route altitude reference differs from task")
    require(route.get("coordinate_context", {}).get("canonical_alt_reference") == "EG", "Route internal altitude reference is not EG")
    expected_engine = "ENG-07" if task.get("route_mode", "strict") == "free" else "ENG-06"
    require(route.get("engine_id") == expected_engine, "Route engine differs from route mode")
    require(route.get("status") == "planned", f"Task {task['id']} did not produce a planned route")
    require(route.get("validation", {}).get("valid") is True, f"Task {task['id']} validation failed")
    require(not route["validation"].get("violations"), f"Task {task['id']} has violations")
    vertices = wkt_vertices(route.get("trajectory_wkt"))
    times = route.get("vertex_times_s", [])
    require(len(vertices) == len(times), f"Task {task['id']} vertex and time counts differ")
    require(all(isinstance(value, (int, float)) and math.isfinite(value) and value >= 0 for value in times),
            f"Task {task['id']} has invalid vertex times")
    require(all(left < right for left, right in zip(times, times[1:])), f"Task {task['id']} vertex times must increase")
    for actual, expected in ((vertices[0], wkt_vertices(route["start_wkt"])[0]),
                             (vertices[-1], wkt_vertices(route["goal_wkt"])[0])):
        require(all(abs(left - right) <= tolerance for left, right, tolerance in
                    zip(actual, expected, (1e-8, 1e-8, 1e-5))), f"Task {task['id']} trajectory endpoint differs")
    expected_goal = task["goal_wkt"]
    if route.get("action", "mission") == "return_to_home":
        expected_goal = (task.get("state") or {}).get("home_wkt")
        require(isinstance(expected_goal, str), f"Task {task['id']} return-to-home route requires state.home_wkt")
    for actual, expected in ((vertices[0], wkt_vertices(task["start_wkt"])[0]),
                             (vertices[-1], wkt_vertices(expected_goal)[0])):
        require(all(abs(left - right) <= tolerance for left, right, tolerance in
                    zip(actual, expected, (1e-8, 1e-8, 1e-5))), f"Task {task['id']} endpoint differs from its input")
    segments = route.get("motion_segments", [])
    require(bool(segments), f"Task {task['id']} has no motion segments")
    previous_end = None
    for segment in segments:
        start, end = segment["start_time_s"], segment["end_time_s"]
        require(isinstance(start, (int, float)) and isinstance(end, (int, float))
                and math.isfinite(start) and math.isfinite(end) and 0 <= start <= end,
                f"Task {task['id']} has an invalid motion interval")
        if previous_end is not None:
            require(abs(start - previous_end) <= 1e-5, f"Task {task['id']} motion timing has a gap")
        previous_end = end
        if segment.get("points"):
            require(len(segment["points"]) == len(segment.get("times_s", [])),
                    f"Task {task['id']} motion point and time counts differ")
    require(abs(segments[0]["start_time_s"] - times[0]) <= 1e-5 and
            abs(segments[-1]["end_time_s"] - times[-1]) <= 1e-5,
            f"Task {task['id']} exported times do not cover its motion")


def check_batch(result: dict[str, Any], tasks: list[dict[str, Any]]) -> None:
    expected = [task["id"] for task in tasks]
    require(len(set(expected)) == len(expected), "Input task IDs must be unique")
    routes = result.get("routes", [])
    actual = [route.get("task_id") for route in routes]
    require(len(actual) == len(expected) and len(set(actual)) == len(actual) and set(actual) == set(expected),
            "Batch route IDs are missing, duplicated, or unknown")
    require(result.get("success_count") == len(tasks), "Batch success count differs from task count")
    require(result.get("failed_count") == 0, "Batch contains failed routes")
    require(result.get("conflicts") == [], "Batch contains route conflicts")
    by_id = {task["id"]: task for task in tasks}
    for route in routes:
        check_route(route, by_id[route["task_id"]])


class TestRun:
    """Run independent interface checks and collect their actual results."""

    def __init__(self, client: HTTPClient, data_dir: Path, poll_timeout: float) -> None:
        self.client = client
        self.data_dir = data_dir
        self.poll_timeout = poll_timeout
        self.report: dict[str, Any] = {"base_url": client.base_url, "started_at": utc_now(),
                                      "finished_at": None, "status": "running", "service_version": None,
                                      "checks": [], "coverage": [], "benchmarks": [],
                                      "exchanges": client.exchanges, "summary": {}}

    def check(self, name: str, function: Callable[[], Any], *, method: str | None = None,
              path: str | None = None) -> Any:
        start = time.perf_counter()
        item: dict[str, Any] = {"name": name}
        if method:
            item["method"] = method
        if path:
            item["path"] = path
        try:
            result = function()
            item.update(status="passed", message="检查通过。")
            return result
        except Exception as exc:
            item.update(status="failed", message=str(exc), error_type=type(exc).__name__)
            return None
        finally:
            item["elapsed_ms"] = round((time.perf_counter() - start) * 1000, 3)
            self.report["checks"].append(item)
            print(f"[{item['status']}] {name}: {item['message']}", flush=True)

    def get(self, path: str) -> Any:
        return self.client.request("GET", path)

    def post(self, path: str, body: Any) -> Any:
        return self.client.request("POST", path, body)

    def fixture(self, name: str) -> Any:
        return load_json(self.data_dir / "examples" / f"{name}.json")

    def validate(self, source: dict[str, Any], route: dict[str, Any]) -> None:
        payload = {key: source[key] for key in ("scene", "scene_id", "task") if key in source}
        payload.update({key: route[key] for key in ("trajectory_wkt", "vertex_times_s", "motion_segments")})
        if "initial_velocity_xyz_mps" in route:
            payload["initial_velocity_xyz_mps"] = route["initial_velocity_xyz_mps"]
        if route.get("network_route") is not None:
            payload["network_route"] = route["network_route"]
        response = self.post("/api/v2/validate", payload)
        require(response.get("valid") is True and not response.get("violations"), "Independent route validation failed")

    def _health(self) -> None:
        result = self.get("/api/v2/health")
        require(result.get("status") == "ok" and result.get("native_available") is True,
                "Health status or native engine status failed")

    def _job_health(self) -> None:
        result = self.get("/api/v2/health/jobs")
        counts = [result.get(name) for name in ("pending", "running", "succeeded", "failed")]
        require(all(type(value) is int and value >= 0 for value in counts), "Job counts must be nonnegative integers")
        require(sum(counts) == result.get("total"), "Job counts do not sum to total")

    def _version(self) -> None:
        result = self.get("/api/v2/version")
        self.report["service_version"] = result
        require(result.get("api_version") == "2.0.0", "Service API version differs from 2.0.0")
        require(bool(result.get("engine", {}).get("git_sha")), "Service revision is missing")

    def _capabilities(self) -> None:
        result = self.get("/api/v2/capabilities")
        require(result.get("canonical_alt_reference") == "EG" and result.get("contract_version") == "2026-10-30", "Service contract or altitude reference differs")
        require(set(result.get("engine_rules", {})) == {"ENG-06", "ENG-07"}, "Engine rule allocation is missing")
        require(set(result.get("route_modes", [])) == {"strict", "prefer", "free"}, "Route modes differ")
        require(len(set(result.get("rule_types", []))) == 20, "Rule types do not contain 20 unique entries")
        require(all(result.get(key) is True for key in ("airway_graph_supported", "network_analysis_supported",
                    "traversed_grids_supported", "batch_execution_supported", "objective_optimization_supported",
                    "incremental_replanning_supported", "initial_velocity_validation_supported")), "Current engine features are missing")
        require(set(result.get("search_algorithms", [])) == {"ASTAR", "DIJKSTRA"}
                and set(result.get("grid_neighbors", [])) == {6,18,26}, "Current search options differ")

    def _openapi(self) -> None:
        spec = self.get("/openapi.json")
        operations = {(method.upper(), path) for path, item in spec.get("paths", {}).items()
                      for method in item if method.upper() in {"GET", "POST", "PUT", "PATCH", "DELETE"}}
        require(operations == set(OPERATIONS), f"OpenAPI operations differ from the {len(OPERATIONS)} v2 operations")

    def _swagger(self) -> None:
        page = self.client.request("GET", "/docs", as_text=True)
        require("SwaggerUIBundle" in page and "/openapi.json" in page, "Swagger page does not load the OpenAPI document")

    def _resolved_task(self, source: dict[str, Any]) -> dict[str, Any]:
        task = deepcopy(source["task"])
        scene = source.get("scene", {})
        for label in ("start", "goal"):
            if task.get(label + "_wkt"):
                continue
            if task.get("network"):
                require(task["alt_reference"] == "EG", "Graph handoff fixture uses EG heights")
                node_id = task["network"][label + "_node_id"]
                nodes = {node["id"]: node for node in scene["airway_graph"]["nodes"]}
                task[label + "_wkt"] = nodes[node_id]["point_wkt"]
            elif task.get(label + "_grid") is not None:
                grid = scene["grid"]
                center = [origin + (index + .5) * resolution for origin, index, resolution in
                          zip(grid["origin"], task[label + "_grid"], grid["resolution"])]
                point = "POINT Z (" + " ".join(str(value) for value in center) + ")"
                converted = self.post("/api/v2/coordinates/convert", {
                    "coordinates": scene["coordinates"], "direction": "local_to_wgs84",
                    "alt_reference": task["alt_reference"], "at": task["departure_time"],
                    "geometries_wkt": [point]})
                task[label + "_wkt"] = converted["geometries_wkt"][0]
            else:
                raise AssertionError(f"Task {task['id']} has no {label} selector")
        return task

    def _single(self, source: dict[str, Any]) -> dict[str, Any]:
        expected_task = self._resolved_task(source)
        result = self.post("/api/v2/plan/single", source)
        check_route(result, expected_task)
        self.validate(source, result)
        return result

    def _network_feature(self) -> None:
        result = self._single(load_json(self.data_dir / "features/network_request.json"))
        trace = result.get("network_route", {})
        require(trace.get("node_ids") == ["A", "B", "D"] and trace.get("edge_ids") == ["ab", "bd"],
                "Weighted graph route differs from the fixture optimum")
        require(trace.get("total_cost") == 2, "Graph raw weight sum differs")

    def _network_analysis(self) -> None:
        source = load_json(self.data_dir / "features/network_analyze_request.json")
        result = self.post("/api/v2/networks/analyze", source)
        require(result.get("version") == source["scene"]["airway_graph"]["version"], "Graph version differs")
        require(["E"] in result.get("weak_components", []), "Isolated graph node is missing")
        require(set(result.get("active_edges", [])) == {"ab", "bd", "ac", "cd"}, "Active graph edges differ")

    def _grid_feature(self) -> None:
        result = self._single(load_json(self.data_dir / "features/search_request.json"))
        search = result.get("search_report", {})
        require(search.get("algorithm") == "DIJKSTRA" and search.get("neighbors") == 6,
                "Grid search configuration differs")
        require(search.get("grid_level") == 12 and search.get("expanded_nodes", 0) > 0,
                "Grid search level or expansion count differs")
        require(result.get("grid_context", {}).get("level") == 12, "Exported grid level differs")
        grids = result.get("traversed_grids", [])
        require(len(grids) >= 2, "Complete traversed grid sequence is missing")
        require(grids[0].get("grid_code") == "start-cell" and grids[-1].get("grid_code") == "goal-cell",
                "Upstream grid codes are missing")
        for left, right in zip(grids, grids[1:]):
            require(abs(left["exit_time_s"] - right["entry_time_s"]) <= 1e-5, "Traversed grid times have a gap")

    def _objectives_feature(self) -> None:
        source = load_json(self.data_dir / "features/objectives_request.json")
        primary = self._single(source)
        routes = [primary, *primary.get("alternatives", [])]
        require(len(routes) >= 2, "Objective fixture did not produce competing alternatives")
        report = primary.get("optimization", {})
        require(report.get("scope") == "validated_candidates" and report.get("status") == "generator_exhausted",
                "Objective candidate generator did not finish")
        require(report.get("returned_solutions") == len(routes) and report.get("primary_executed_only") is True,
                "Objective solution report differs")
        signatures = set()
        vectors = []
        scores = []
        for route in routes:
            check_route(route, source["task"])
            if route is not primary:
                self.validate(source, route)
            signature = (route["trajectory_wkt"], tuple(route["vertex_times_s"]))
            require(signature not in signatures, "Duplicate objective alternative")
            signatures.add(signature)
            objective = route.get("objectives", {})
            values = objective.get("values", {})
            require(set(values) == {"distance", "time", "energy", "risk", "north_cost", "south_cost"},
                    "Objective dimensions are missing")
            require(values["energy"]["unit"] == "Wh" and values["risk"]["unit"] == "risk*s",
                    "Objective energy or risk unit differs")
            require(objective.get("dataset_versions") == {item["id"]: item["version"] for item in source["scene"]["datasets"]},
                    "Objective dataset versions differ")
            score = 0.0
            for value in values.values():
                require(all(type(value[key]) in (int, float) and math.isfinite(value[key])
                            for key in ("raw", "normalized", "normalization", "weight"))
                        and value["raw"] >= 0 and value["normalization"] > 0 and value["weight"] >= 0,
                        "Objective values or weights are invalid")
                require(math.isclose(value["normalized"], value["raw"] / value["normalization"], abs_tol=1e-8),
                        "Objective normalization differs")
                score += value["normalized"] * value["weight"]
            require(math.isclose(score, objective.get("weighted_score", math.inf), abs_tol=1e-8), "Objective total score differs")
            scores.append(score)
            vectors.append([values[key]["normalized"] for key in sorted(values)])
        require(scores[0] <= min(scores) + 1e-8, "Primary route does not minimize returned objective scores")
        require(all(not (all(x <= y+1e-8 for x,y in zip(left,right)) and any(x < y-1e-8 for x,y in zip(left,right)))
                    for i,left in enumerate(vectors) for j,right in enumerate(vectors) if i != j),
                "Returned objective alternatives contain a dominated route")
        primary_values = primary["objectives"]["values"]
        require(any(route["objectives"]["values"]["distance"]["raw"] < primary_values["distance"]["raw"]
                    and route["objectives"]["values"]["risk"]["raw"] > primary_values["risk"]["raw"]
                    for route in routes[1:]), "Objective fixture did not expose distance/risk tradeoff")

    def _replan_feature(self) -> None:
        source = load_json(self.data_dir / "features/replan_request.json")
        result = self.post("/api/v2/plan/replan", source)
        require(result.get("decision") == "replanned" and result.get("new_search_calls", 0) >= 1,
                "Changed obstacle did not cause incremental recomputation")
        require(result.get("reused_distance_m", 0) > 0 and result.get("reused_primitives", 0) > 0,
                "Replanning did not retain its safe prefix")
        route = result.get("route", {})
        check_route(route, source["remaining_task"])
        velocity = source["current_state"]["velocity_xyz_mps"]
        require(all(abs(a-b) <= 1e-8 for a,b in zip(route.get("initial_velocity_xyz_mps", []), velocity))
                and len(route.get("initial_velocity_xyz_mps", [])) == 3, "Replanning lost the current velocity")
        validation_source = {"scene": source["scene"], "task": source["remaining_task"]}
        self.validate(validation_source, route)
        wrong_initial = {**validation_source, **{key: route[key] for key in
                         ("trajectory_wkt", "vertex_times_s", "motion_segments")}, "initial_velocity_xyz_mps": [0,0,0]}
        require(self.post("/api/v2/validate", wrong_initial).get("valid") is False,
                "Independent validation accepted the wrong declared initial velocity")
        unchanged = deepcopy(source)
        unchanged["scene"] = unchanged["previous_scene"]
        unchanged["event"] = {"id": "unchanged-route-demo", "type": "aircraft_state",
                              "at": unchanged["current_state"]["at"], "old_value": 90, "new_value": 85}
        reused = self.post("/api/v2/plan/replan", unchanged)
        require(reused.get("decision") == "reused" and reused.get("new_search_calls") == 0,
                "Unchanged suffix was not reused without search")
        require(reused.get("reused_distance_m", 0) > 0, "Unchanged suffix reuse report is empty")
        check_route(reused["route"], unchanged["remaining_task"])
        self.validate({"scene": unchanged["scene"], "task": unchanged["remaining_task"]}, reused["route"])

    def _parallel_feature(self) -> None:
        source = self.fixture("batch_request")
        source["execution"] = {"workers": 2, "batch_size": 2}
        result = self._batch(source)
        report = result.get("execution_report", {})
        require(report.get("requested_workers") == 2 and 1 <= report.get("actual_workers", 0) <= 2,
                "Candidate worker report differs")
        require(report.get("batch_size") == 2 and report.get("batches") == 1 and report.get("candidate_count") == 2,
                "Candidate group report differs")
        for task in source["tasks"]:
            route = next(route for route in result["routes"] if route["task_id"] == task["id"])
            self.validate({"scene": source["scene"], "task": task}, route)

    def _batch(self, source: dict[str, Any]) -> dict[str, Any]:
        result = self.post("/api/v2/plan/batch", source)
        check_batch(result, source["tasks"])
        return result

    def _scene_reference(self) -> None:
        source = self.fixture("single_strict_request")
        created = self.post("/api/v2/scenes", source["scene"])
        require(created.get("status") == "ready", "Registered scene is not ready")
        scene_id = created["scene_id"]
        detail = self.get("/api/v2/scenes/" + scene_id)
        require(detail["scene_id"] == scene_id and detail["scene_version"] == created["scene_version"],
                "Scene lookup does not match the registered scene")
        self._single({"scene_id": scene_id, "task": source["task"]})

    def _coordinates(self) -> None:
        source = self.fixture("coordinates_to_local_request")
        converted = self.post("/api/v2/coordinates/convert", source)
        require(converted.get("direction") == "wgs84_to_local", "Coordinate direction differs")
        require(len(converted.get("geometries_wkt", [])) == len(source["geometries_wkt"]),
                "Coordinate geometry count differs")
        restored = self.post("/api/v2/coordinates/convert", {
            **source, "direction": "local_to_wgs84", "geometries_wkt": converted["geometries_wkt"]})
        require(restored.get("direction") == "local_to_wgs84", "Return coordinate direction differs")
        for before, after in zip(source["geometries_wkt"], restored["geometries_wkt"]):
            left, right = wkt_vertices(before), wkt_vertices(after)
            require(len(left) == len(right), "Coordinate round trip changes vertex count")
            require(all(abs(a - b) <= tolerance for lpoint, rpoint in zip(left, right)
                        for a, b, tolerance in zip(lpoint, rpoint, (1e-8, 1e-8, 1e-5))),
                    "Coordinate round trip differs by more than 1e-8 degrees or 1e-5 metres")

    def _recorded_validation(self) -> None:
        response = self.post("/api/v2/validate", self.fixture("validate_request"))
        require(response.get("valid") is True and not response.get("violations"), "Fixture route validation failed")

    def _async(self) -> None:
        source = self.fixture("batch_request")
        source["execution"] = {"workers": 2, "batch_size": 2}
        submitted = self.client.request("POST", "/api/v2/jobs", source, expected_status=202)
        require(submitted.get("status") == "pending", "Submitted job is not pending")
        job_id = submitted["job_id"]
        path = f"/api/v2/jobs/{job_id}"
        require(submitted.get("poll_url") == path, "Job poll URL differs from job ID")
        deadline = time.monotonic() + self.poll_timeout
        while True:
            result = self.get(path)
            require(result.get("job_id") == job_id, "Polled job ID differs")
            if result.get("status") == "succeeded":
                check_batch(result["result"], source["tasks"])
                return
            require(result.get("status") in {"pending", "running"}, f"Job failed: {result.get('error')}")
            require(time.monotonic() < deadline, f"Job did not finish in {self.poll_timeout:g} seconds")
            time.sleep(min(0.5, max(0, deadline - time.monotonic())))

    def _error(self, method: str, path: str, body: Any, status: int, code: str) -> None:
        response = self.client.request(method, path, body, expected_status=status)
        require(response.get("error", {}).get("code") == code, f"Expected error code {code}")

    def examples(self) -> None:
        for name, path, action in (
            ("服务健康", "/api/v2/health", self._health),
            ("作业状态统计", "/api/v2/health/jobs", self._job_health),
            ("服务版本", "/api/v2/version", self._version),
            ("接口能力", "/api/v2/capabilities", self._capabilities),
            ("OpenAPI 文档", "/openapi.json", self._openapi),
            ("Swagger 页面", "/docs", self._swagger),
        ):
            self.check(name, action, method="GET", path=path)
        def remote_single() -> None:
            self._single(self.get("/api/v2/examples/single"))
        def remote_batch() -> None:
            self._batch(self.get("/api/v2/examples/batch"))
        self.check("在线单机示例及验证", remote_single)
        self.check("在线批量示例", remote_batch)
        for mode in ("strict", "prefer", "free"):
            self.check(f"{mode} 单机规划及验证", lambda mode=mode: self._single(self.fixture(f"single_{mode}_request")))
        self.check("完整内嵌批量请求", lambda: self._batch(self.fixture("batch_request")))
        self.check("场景登记、查询及引用规划", self._scene_reference)
        self.check("坐标双向转换", self._coordinates)
        self.check("归档运动段独立验证", self._recorded_validation)
        self.check("异步提交及结果轮询", self._async)
        invalid = deepcopy(self.fixture("single_strict_request"))
        invalid["unknown_field"] = True
        self.check("未知字段返回 400", lambda: self._error("POST", "/api/v2/plan/single", invalid, 400, "INVALID_INPUT"))
        self.check("未知场景返回 404", lambda: self._error("GET", "/api/v2/scenes/test-api-missing-scene", None, 404, "TASK_NOT_FOUND"))
        self.check("未知作业返回 404", lambda: self._error("GET", "/api/v2/jobs/test-api-missing-job", None, 404, "TASK_NOT_FOUND"))
        self.check("航路图权重规划及验证", self._network_feature)
        self.check("航路图连通性分析", self._network_analysis)
        self.check("网格端点、搜索配置及完整网格序列", self._grid_feature)
        self.check("批量并行与统一避让", self._parallel_feature)
        self.check("多目标、能耗及自定义代价方案比较", self._objectives_feature)
        self.check("增量重规划、零搜索复用及初始速度验证", self._replan_feature)

    def benchmark(self, config: dict[str, Any]) -> None:
        suite = config["suite"]
        tasks = load_json(self.data_dir / config["tasks"])
        require(len(tasks) == config["task_count"], f"{suite} fixture task count differs")
        created = self.post("/api/v2/scenes", load_json(self.data_dir / config["scene"]))
        require(created.get("status") == "ready", f"{suite} scene is not ready")
        payload = {"scene_id": created["scene_id"], "tasks": tasks, "request_id": f"test-api-{suite}"}
        started = time.perf_counter()
        result = self.post("/api/v2/plan/batch", payload)
        elapsed_ms = round((time.perf_counter() - started) * 1000, 3)
        response_path = self.client.exchanges[-1].get("response_path")
        item = {"suite": suite, "task_count": len(tasks), "success_count": result.get("success_count"),
                "failed_count": result.get("failed_count"), "conflict_count": len(result.get("conflicts", [])),
                "elapsed_ms": elapsed_ms, "response_path": response_path, "status": "failed"}
        self.report["benchmarks"].append(item)
        check_batch(result, tasks)
        route = result["routes"][0]
        task = next(task for task in tasks if task["id"] == route["task_id"])
        self.validate({"scene_id": created["scene_id"], "task": task}, route)
        item["status"] = "passed"

    def finish(self) -> dict[str, Any]:
        checks = self.report["checks"]
        self.report["finished_at"] = utc_now()
        coverage = []
        for method, template in OPERATIONS:
            pattern = re.sub(r"\\\{[^}]+\\\}", "[^/]+", re.escape(template))
            matches = [item for item in self.client.exchanges if item["method"] == method and
                       re.fullmatch(pattern, item["path"]) and item.get("http_status", 500) < 300]
            coverage.append({"method": method, "path": template,
                             "status": "passed" if any(item.get("status") == "passed" for item in matches) else "failed"})
        self.report["coverage"] = coverage
        passed = sum(item["status"] == "passed" for item in checks)
        failed = len(checks) - passed
        self.report["summary"] = {"passed": passed, "failed": failed,
                                  "operation_count": len(OPERATIONS),
                                  "covered_operations": sum(item["status"] == "passed" for item in coverage)}
        self.report["status"] = "passed" if failed == 0 and all(item["status"] == "passed" for item in coverage) else "failed"
        return self.report


def positive_seconds(value: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("Use a positive finite number of seconds")
    return number


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-url", default="https://api.kaguratart.com")
    parser.add_argument("--output", type=Path, help="Write records and reports to a new directory")
    parser.add_argument("--timeout", type=positive_seconds, default=120, help="HTTP timeout in seconds")
    parser.add_argument("--poll-timeout", type=positive_seconds, default=300, help="Job polling timeout in seconds")
    parser.add_argument("--examples-only", action="store_true", help="Run interface examples; omit the three 1000-task batches")
    parser.add_argument("--verify-data", action="store_true", help="Verify fixture SHA-256 values and exit")
    args = parser.parse_args(argv)
    try:
        manifest = verify_data(ROOT / "data")
        if args.verify_data:
            print(f"数据校验通过：{len(manifest['files'])} 个文件。")
            return 0
        base_url = normalize_base_url(args.base_url)
        output = args.output or ROOT / "results" / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        output.mkdir(parents=True, exist_ok=False)
    except (OSError, ValueError, AssertionError) as exc:
        print(f"测试启动失败：{exc}", file=sys.stderr)
        return 1
    run = TestRun(HTTPClient(base_url, output, args.timeout), ROOT / "data", args.poll_timeout)
    try:
        run.examples()
        if not args.examples_only:
            for config in manifest["benchmarks"]:
                run.check(f"{config['suite']} 1000 任务规划及验证", lambda config=config: run.benchmark(config))
    except Exception as exc:
        def failed_run() -> None:
            raise exc
        run.check("执行测试", failed_run)
    report = run.finish()
    (output / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
                                      encoding="utf-8")
    report_path = write_report(report, output)
    print(f"报告：{report_path}")
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
