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
        require(set(result.get("route_modes", [])) == {"strict", "prefer", "free"}, "Route modes differ")
        require(len(set(result.get("rule_types", []))) == 20, "Rule types do not contain 20 unique entries")

    def _openapi(self) -> None:
        spec = self.get("/openapi.json")
        operations = {(method.upper(), path) for path, item in spec.get("paths", {}).items()
                      for method in item if method.upper() in {"GET", "POST", "PUT", "PATCH", "DELETE"}}
        require(operations == set(OPERATIONS), "OpenAPI operations differ from the 14 v2 operations")

    def _swagger(self) -> None:
        page = self.client.request("GET", "/docs", as_text=True)
        require("SwaggerUIBundle" in page and "/openapi.json" in page, "Swagger page does not load the OpenAPI document")

    def _single(self, source: dict[str, Any]) -> dict[str, Any]:
        result = self.post("/api/v2/plan/single", source)
        check_route(result, source["task"])
        self.validate(source, result)
        return result

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
