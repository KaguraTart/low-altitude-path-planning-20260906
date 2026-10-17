"""The delivery kit runs independently and records real HTTP exchanges."""
from contextlib import contextmanager
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import shutil
import subprocess
import sys
from threading import Thread
from urllib.parse import urlsplit

import pytest

from test_api.client import HTTPClient, USER_AGENT, normalize_base_url
from test_api.run_tests import OPERATIONS, check_batch, check_route, main, verify_data


PROJECT = Path(__file__).resolve().parents[2]
KIT = PROJECT / "test_api"


def read_example(name):
    return json.loads((KIT / "data" / "examples" / f"{name}.json").read_text(encoding="utf-8"))


def route_for(task):
    start = task["start_wkt"].partition("(")[2].rstrip(") ")
    goal = task["goal_wkt"].partition("(")[2].rstrip(") ")
    return {"task_id": task["id"], "status": "planned", "route_mode": task.get("route_mode", "strict"),
            "start_wkt": task["start_wkt"], "goal_wkt": task["goal_wkt"],
            "trajectory_wkt": f"LINESTRING Z ({start}, {goal})", "vertex_times_s": [0, 10],
            "motion_segments": [{"kind": "line", "start_time_s": 0, "end_time_s": 10,
                                 "start": [0, 0, 80], "end": [100, 0, 80]}],
            "validation": {"valid": True, "violations": []}}


class FakeService:
    def __init__(self):
        self.requests = []
        self.unknown_scene = "test-api-missing-scene"
        self.omit_last_route = False
        self.job_failed = False
        self.batch = None
        self.coordinate_input = None

    def batch_result(self, body):
        routes = [route_for(task) for task in body["tasks"]]
        if self.omit_last_route:
            routes = routes[:-1]
        return {"routes": routes, "success_count": len(body["tasks"]), "failed_count": 0, "conflicts": []}

    def dispatch(self, method, path, body, headers):
        self.requests.append({"method": method, "path": path, "body": body,
                              "user_agent": headers.get("User-Agent"), "accept": headers.get("Accept")})
        if path == "/rejected":
            return 403, b'{"error":{"code":"REJECTED"},"body":"complete response"}'
        if path == "/docs":
            return 200, b'<html>SwaggerUIBundle({url:"/openapi.json"})</html>'
        if path == "/openapi.json":
            paths = {}
            for operation_method, operation_path in OPERATIONS:
                paths.setdefault(operation_path, {})[operation_method.lower()] = {}
            return 200, {"openapi": "3.1.0", "paths": paths}
        if path == "/api/v2/health":
            return 200, {"status": "ok", "native_available": True}
        if path == "/api/v2/health/jobs":
            return 200, {"pending": 0, "running": 0, "succeeded": 0, "failed": 0, "total": 0}
        if path == "/api/v2/version":
            return 200, {"api_version": "2.0.0", "engine": {"git_sha": "f" * 40}}
        if path == "/api/v2/capabilities":
            return 200, {"route_modes": ["strict", "prefer", "free"], "rule_types": list(range(20))}
        if path == "/api/v2/examples/single":
            return 200, read_example("single_strict_request")
        if path == "/api/v2/examples/batch":
            return 200, read_example("batch_request")
        if path == "/api/v2/scenes" or path == "/api/v2/scenes/fixture-scene":
            return 200, {"status": "ready", "scene_id": "fixture-scene", "scene_version": "one"}
        if path == "/api/v2/coordinates/convert":
            if body["direction"] == "wgs84_to_local":
                self.coordinate_input = body["geometries_wkt"]
                result = ["POINT Z (100 100 80)" for _ in self.coordinate_input]
            else:
                result = self.coordinate_input
            return 200, {"geometries_wkt": result, "direction": body["direction"]}
        if path == "/api/v2/plan/single":
            if "unknown_field" in body:
                return 400, {"error": {"code": "INVALID_INPUT"}}
            return 200, route_for(body["task"])
        if path == "/api/v2/plan/batch":
            return 200, self.batch_result(body)
        if path == "/api/v2/validate":
            return 200, {"valid": True, "violations": []}
        if path == "/api/v2/jobs":
            self.batch = body
            return 202, {"job_id": "fixture-job", "status": "pending", "poll_url": "/api/v2/jobs/fixture-job"}
        if path == "/api/v2/jobs/fixture-job":
            if self.job_failed:
                return 200, {"job_id": "fixture-job", "status": "failed", "error": {"code": "RESOURCE_EXHAUSTED"}}
            return 200, {"job_id": "fixture-job", "status": "succeeded", "result": self.batch_result(self.batch)}
        return 404, {"error": {"code": "TASK_NOT_FOUND"}}


@contextmanager
def service():
    api = FakeService()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            self.respond()

        def do_POST(self):
            self.respond()

        def respond(self):
            raw = self.rfile.read(int(self.headers.get("Content-Length", "0")))
            body = json.loads(raw) if raw else None
            status, value = api.dispatch(self.command, urlsplit(self.path).path, body, self.headers)
            content = value if isinstance(value, bytes) else json.dumps(value).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(content)))
            self.end_headers()
            self.wfile.write(content)

        def log_message(self, format, *args):
            pass

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield api, f"http://127.0.0.1:{server.server_port}"
    finally:
        server.shutdown()
        server.server_close()
        thread.join()


def test_delivery_kit_covers_all_operations_and_keeps_payloads(tmp_path):
    output = tmp_path / "records"
    with service() as (api, url):
        assert main(["--base-url", url + "/docs#/", "--examples-only", "--output", str(output)]) == 0
    report = json.loads((output / "report.json").read_text())
    assert report["status"] == "passed"
    assert report["summary"] == {"passed": 19, "failed": 0, "operation_count": 14, "covered_operations": 14}
    assert (output / "REPORT.md").is_file()
    assert all(item["user_agent"] == USER_AGENT and item["accept"] == "application/json" for item in api.requests)
    batch_requests = [item for item in api.requests if item["path"] == "/api/v2/plan/batch"]
    assert batch_requests[-1]["body"] == read_example("batch_request")
    validation_requests = [item for item in api.requests if item["path"] == "/api/v2/validate"]
    assert all(item["body"]["motion_segments"] for item in validation_requests)
    for exchange in report["exchanges"]:
        assert (output / exchange["request_path"]).is_file()
        assert (output / exchange["response_path"]).stat().st_size == exchange["output"]["bytes"]


def test_failed_batch_produces_nonzero_status_and_complete_report(tmp_path):
    output = tmp_path / "records"
    with service() as (api, url):
        api.omit_last_route = True
        assert main(["--base-url", url, "--examples-only", "--output", str(output)]) == 1
    report = json.loads((output / "report.json").read_text())
    assert report["status"] == "failed"
    assert any("IDs are missing" in item["message"] for item in report["checks"] if item["status"] == "failed")
    assert (output / "REPORT.md").is_file()


def test_http_failure_preserves_the_exact_response(tmp_path):
    with service() as (_, url):
        client = HTTPClient(url, tmp_path)
        with pytest.raises(AssertionError, match="received 403"):
            client.request("GET", "/rejected")
    exchange = client.exchanges[0]
    assert exchange["status"] == "failed" and exchange["http_status"] == 403
    assert (tmp_path / exchange["response_path"]).read_bytes() == (
        b'{"error":{"code":"REJECTED"},"body":"complete response"}')


def test_failed_async_job_is_a_failed_check(tmp_path):
    output = tmp_path / "records"
    with service() as (api, url):
        api.job_failed = True
        assert main(["--base-url", url, "--examples-only", "--output", str(output)]) == 1
    report = json.loads((output / "report.json").read_text())
    check = next(item for item in report["checks"] if item["name"] == "异步提交及结果轮询")
    assert check["status"] == "failed"
    assert "RESOURCE_EXHAUSTED" in check["message"]


def test_batch_requires_complete_unique_routes():
    tasks = read_example("batch_request")["tasks"]
    result = FakeService().batch_result({"tasks": tasks})
    result["routes"][-1] = result["routes"][0]
    with pytest.raises(AssertionError, match="duplicated"):
        check_batch(result, tasks)


def test_route_must_reach_the_input_endpoint():
    task = read_example("single_strict_request")["task"]
    task["state"] = {"home_wkt": task["start_wkt"]}
    wrong = {**task, "goal_wkt": task["start_wkt"]}
    route = route_for(wrong)
    route["action"] = "mission"
    with pytest.raises(AssertionError, match="endpoint differs from its input"):
        check_route(route, task)


def test_return_to_home_route_reaches_the_input_home():
    task = read_example("single_strict_request")["task"]
    home = "POINT Z (104.772 29.331 80)"
    task["state"] = {"battery_percent": 10, "home_wkt": home}
    route = route_for({**task, "goal_wkt": home})
    route["action"] = "return_to_home"
    check_route(route, task)


def test_return_to_home_route_requires_the_input_home():
    task = read_example("single_strict_request")["task"]
    route = route_for(task)
    route["action"] = "return_to_home"
    with pytest.raises(AssertionError, match="requires state.home_wkt"):
        check_route(route, task)


def test_route_vertex_times_must_increase():
    task = read_example("single_strict_request")["task"]
    route = route_for(task)
    route["vertex_times_s"] = [0, 0]
    with pytest.raises(AssertionError, match="vertex times must increase"):
        check_route(route, task)


def test_standalone_copy_runs_from_another_directory(tmp_path):
    copied = tmp_path / "delivery" / "test_api"
    shutil.copytree(KIT, copied, ignore=shutil.ignore_patterns("results", "__pycache__"))
    completed = subprocess.run([sys.executable, str(copied / "run_tests.py"), "--verify-data"],
                               cwd=tmp_path, capture_output=True, text=True, timeout=30)
    assert completed.returncode == 0, completed.stderr
    assert "数据校验通过" in completed.stdout


def test_fixture_tamper_is_detected(tmp_path):
    data = tmp_path / "data"
    data.mkdir()
    (data / "sample.json").write_text("{}", encoding="utf-8")
    (data / "manifest.json").write_text(json.dumps({"files": [{"path": "sample.json", "bytes": 2, "sha256": "0" * 64}]}))
    with pytest.raises(AssertionError, match="SHA-256 differs"):
        verify_data(data)


@pytest.mark.parametrize("value", ["https://example.com", "https://example.com/docs#/", "http://127.0.0.1:18000/docs"])
def test_origin_and_swagger_urls_are_accepted(value):
    assert normalize_base_url(value) == value.split("/docs")[0]


@pytest.mark.parametrize("value", ["example.com", "file:///tmp/data", "https://name:secret@example.com", "https://example.com/other"])
def test_invalid_base_url_is_rejected(value):
    with pytest.raises(ValueError):
        normalize_base_url(value)
