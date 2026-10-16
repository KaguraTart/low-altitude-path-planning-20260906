"""Complete replayable benchmark archives, including failed exchanges."""
from copy import deepcopy
from io import BytesIO
import json
from pathlib import Path
import sys
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit

import pytest

from tools import benchmark_v2 as benchmark
from uav_planner import ErrorCode, PlanningError
from uav_planner.v2 import engine


class Response(BytesIO):
    status = 200
    headers = {"Content-Type": "application/json"}


@pytest.fixture
def small_run(monkeypatch, tmp_path):
    workload = {"scene": {
        "coordinates": {"origin_lon": 104, "origin_lat": 29, "terrain_msl_m": 0},
        "bounds": {"x_min": -500, "x_max": 500, "y_min": -500, "y_max": 500, "z_min": 0, "z_max": 300},
        "time_origin": benchmark.timestamp(0),
        "datasets": [{"id": "terrain", "kind": "terrain", "version": "v1", "cells": [{
            "bounds": {"x_min": -500, "x_max": 500, "y_min": -500, "y_max": 500, "z_min": 0, "z_max": 300},
            "values": {"elevation_msl_m": 0}}]}]},
                "tasks": [{"id": f"task-{i}", "waypoints": [{"wkt": "POINT Z (1 2 3)"}]} for i in range(10)]}
    result = {"success_count": 10, "failed_count": 0, "conflicts": [], "routes": [
        {"task_id": task["id"], "status": "planned", "trajectory_wkt": "POINT Z (1 2 3)",
         "motion_segments": [{"kind": "line", "profile": {"peak_speed_mps": 3}}],
         "validation": {"valid": True}, "all_details": {"values": list(range(20))}}
        for task in workload["tasks"]]}
    directory = tmp_path / "frozen"
    directory.mkdir()
    (directory / "manifest.json").write_bytes(b'{"frozen":true}\n')
    state = {"workload": workload, "result": result, "registrations": [], "requests": [],
             "validations": [], "directory": directory, "output": tmp_path / "archive",
             "real_register": engine.register_scene}
    monkeypatch.setattr(benchmark, "load_frozen", lambda *args: (deepcopy(workload), "frozen-digest"))
    monkeypatch.setattr(benchmark, "validate_reference", lambda value: {"valid": True})
    def register(value):
        state["registrations"].append(deepcopy(value))
        return {"scene_id": "registered-scene", "scene_version": "version-one", "warnings": ["完整元数据"]}
    def plan(**request):
        state["requests"].append(deepcopy(request))
        return deepcopy(state["result"])
    def validate(workload, response, count, scene_id):
        state["validations"].append(deepcopy(response))
        assert response["success_count"] == count, "Independent validation rejected failed routes"
        return {"validated_route_count": count}
    monkeypatch.setattr(engine, "register_scene", register)
    monkeypatch.setattr(engine, "plan_batch_v2", plan)
    monkeypatch.setattr(benchmark, "validate_result", validate)
    return state


def run(state, **kwargs):
    return benchmark.run_suite("strict", 1, [10], state["output"], directory=state["directory"], **kwargs)


def read_payload(state, descriptor):
    raw = (state["output"] / descriptor["path"]).read_bytes()
    assert descriptor["bytes"] == len(raw)
    assert descriptor["sha256"] == benchmark.sha256(raw)
    return raw


def test_sdk_success_archives_full_requests_responses_and_replayable_scene(small_run):
    report = run(small_run)
    registration = report["scene_registration"]["payloads"]
    scene = json.loads(read_payload(small_run, registration["input"]))
    assert scene == small_run["workload"]["scene"]
    assert small_run["real_register"](scene)["scene_version"]
    assert json.loads(read_payload(small_run, registration["output"]))["scene_id"] == "registered-scene"
    payloads = report["runs"][0]["payloads"]
    request = json.loads(read_payload(small_run, payloads["input"]))
    assert request == small_run["requests"][0]
    assert request["scene_id"] == "registered-scene"
    assert json.loads(read_payload(small_run, payloads["output"])) == small_run["result"]
    assert payloads["output"]["representation"] == "canonical_json"
    assert payloads["output"]["wire_sha256"] is None
    assert payloads["output"]["canonical_sha256"] == report["runs"][0]["result_sha256"]
    assert len(small_run["registrations"]) == len(small_run["validations"]) == 1


def http_boundary(monkeypatch, state, *, failure=None, scene_failure=False):
    raw_result = json.dumps(state["result"], ensure_ascii=False, indent=2).encode() + b"  \n"
    raw_error = b'{ "error": {"code":"INVALID_INPUT", "detail":{"complete":[1,2,3]}} }\n'
    def urlopen(request, timeout):
        if isinstance(request, str):
            path = urlsplit(request).path
        else:
            path = urlsplit(request.full_url).path
        if path == "/api/v2/scenes":
            assert json.loads(request.data) == state["workload"]["scene"]
            if scene_failure:
                raise HTTPError(request.full_url, 400, "Bad scene", {"X-Trace": "scene-error"}, BytesIO(raw_error))
            return Response(b'{ "scene_id": "registered-scene", "scene_version": "version-one" }\n')
        if path == "/api/v2/version":
            return Response(b'{"api_version":"2.0.0","engine":{"git_sha":"revision"}}')
        assert path == "/api/v2/plan/batch"
        if failure == "http":
            raise HTTPError(request.full_url, 409, "Conflict", {"X-Trace": "batch-error"}, BytesIO(raw_error))
        if failure == "malformed":
            return Response(b"<html>Complete proxy failure</html>")
        if failure == "transport":
            raise URLError("Complete connection failure")
        return Response(raw_result)
    monkeypatch.setattr(benchmark.urllib.request, "urlopen", urlopen)
    return raw_result, raw_error


def test_http_archives_exact_wire_bytes_and_distinct_canonical_hash(small_run, monkeypatch):
    raw, _ = http_boundary(monkeypatch, small_run)
    report = run(small_run, base_url="http://candidate.invalid")
    payloads = report["runs"][0]["payloads"]
    assert read_payload(small_run, payloads["output"]) == raw
    assert payloads["http_status"] == 200
    assert payloads["output"]["wire_sha256"] == benchmark.sha256(raw)
    assert payloads["output"]["canonical_sha256"] == benchmark.sha256(benchmark.canonical_bytes(small_run["result"]))
    assert payloads["output"]["wire_sha256"] != payloads["output"]["canonical_sha256"]
    assert read_payload(small_run, report["service_version_payloads"]["output"])
    assert small_run["validations"] == [small_run["result"]]


def test_failed_route_response_is_still_archived_and_validated(small_run):
    small_run["result"].update(success_count=0, failed_count=10)
    report = run(small_run, functional_only=True)
    failed = report["runs"][0]
    assert not failed["accepted"]
    assert not report["accepted"]
    assert json.loads(read_payload(small_run, failed["payloads"]["output"])) == small_run["result"]
    assert small_run["validations"]


def test_http_error_keeps_complete_body_status_and_request(small_run, monkeypatch):
    _, error = http_boundary(monkeypatch, small_run, failure="http")
    report = run(small_run, base_url="http://candidate.invalid")
    failed = report["runs"][0]
    assert not failed["accepted"]
    assert read_payload(small_run, failed["payloads"]["output"]) == error
    assert failed["payloads"]["http_status"] == 409
    assert failed["payloads"]["http_headers"]["X-Trace"] == "batch-error"
    assert json.loads(read_payload(small_run, failed["payloads"]["input"]))["tasks"] == small_run["workload"]["tasks"]


def test_registration_error_also_has_replayable_archive(small_run, monkeypatch):
    _, error = http_boundary(monkeypatch, small_run, scene_failure=True)
    report = run(small_run, base_url="http://candidate.invalid")
    assert not report["accepted"]
    registration = report["scene_registration"]
    assert not registration["accepted"]
    assert registration["payloads"]["http_status"] == 400
    assert read_payload(small_run, registration["payloads"]["output"]) == error
    assert json.loads(read_payload(small_run, registration["payloads"]["input"])) == small_run["workload"]["scene"]


def test_malformed_response_preserves_raw_body(small_run, monkeypatch):
    http_boundary(monkeypatch, small_run, failure="malformed")
    report = run(small_run, base_url="http://candidate.invalid")
    entry = report["runs"][0]
    assert not entry["accepted"]
    assert read_payload(small_run, entry["payloads"]["output"]) == b"<html>Complete proxy failure</html>"
    assert entry["payloads"]["output"]["canonical_sha256"] is None


def test_transport_error_is_not_mislabeled_as_a_wire_response(small_run, monkeypatch):
    http_boundary(monkeypatch, small_run, failure="transport")
    report = run(small_run, base_url="http://candidate.invalid")
    entry = report["runs"][0]
    assert not entry["accepted"]
    assert entry["payloads"]["http_status"] is None
    descriptor = entry["payloads"]["output"]
    assert descriptor["representation"] == "client_error"
    assert descriptor["wire_sha256"] is None
    assert "Complete connection failure" in json.loads(read_payload(small_run, descriptor))["client_error"]["message"]


def test_sdk_exception_is_archived_as_an_explicit_exception(small_run, monkeypatch):
    def failure(**kwargs):
        raise PlanningError(ErrorCode.INVALID_INPUT, "Complete input failure", {"all": [1, 2, 3]})
    monkeypatch.setattr(engine, "plan_batch_v2", failure)
    report = run(small_run)
    entry = report["runs"][0]
    body = json.loads(read_payload(small_run, entry["payloads"]["output"]))
    assert body["error"]["detail"] == {"all": [1, 2, 3]}
    assert entry["payloads"]["output"]["representation"] == "sdk_exception"
    assert not entry["accepted"]


def test_repeat_refuses_overwrite_before_another_service_call(small_run):
    run(small_run)
    before = {path: path.read_bytes() for path in small_run["output"].rglob("*") if path.is_file()}
    with pytest.raises(FileExistsError, match="overwrite|exists"):
        run(small_run)
    assert all(path.read_bytes() == data for path, data in before.items())
    assert len(small_run["registrations"]) == len(small_run["requests"]) == 1


def test_every_repeat_has_distinct_complete_payloads_with_one_registration(small_run):
    report = benchmark.run_suite("strict", 2, [10], small_run["output"], directory=small_run["directory"])
    assert len(small_run["registrations"]) == 1
    assert len(report["runs"]) == len(small_run["validations"]) == 2
    assert report["completed"] and report["expected_run_count"] == 2
    for index, entry in enumerate(report["runs"], 1):
        payloads = entry["payloads"]
        assert payloads["input"]["path"] == f"payloads/strict_n10_run{index:02d}_input.json"
        assert payloads["output"]["path"] == f"payloads/strict_n10_run{index:02d}_output.json"
        assert json.loads(read_payload(small_run, payloads["input"])) == small_run["requests"][index - 1]
        assert json.loads(read_payload(small_run, payloads["output"])) == small_run["result"]


def test_partial_prior_payload_also_refuses_overwrite(small_run):
    payloads = small_run["output"] / "payloads"
    payloads.mkdir(parents=True)
    previous = payloads / "strict_n10_run01_output.json"
    previous.write_bytes(b"prior evidence")
    with pytest.raises(FileExistsError, match="overwrite"):
        run(small_run)
    assert previous.read_bytes() == b"prior evidence"
    assert not small_run["registrations"]


def test_incomplete_progress_report_never_claims_acceptance():
    partial = {"scene_registration": {"accepted": True}, "expected_run_count": 2,
               "runs": [{"accepted": True, "scale": 1000, "elapsed_ms": 5}]}
    assert not benchmark._report_accepted(partial, functional_only=False)
    assert not benchmark._report_accepted(partial, functional_only=True)


def test_payload_writes_happen_after_timing_stops(small_run, monkeypatch):
    ticks = []
    def clock():
        ticks.append(len(ticks))
        return len(ticks) / 100
    monkeypatch.setattr(benchmark.time, "perf_counter", clock)
    original = Path.open
    writes = []
    def opening(path, mode="r", *args, **kwargs):
        if "payloads" in path.parts and "x" in mode:
            writes.append((path.name, len(ticks)))
        return original(path, mode, *args, **kwargs)
    monkeypatch.setattr(Path, "open", opening)
    run(small_run)
    assert writes
    assert all(tick >= (4 if "_n10_" in name else 2) for name, tick in writes)


@pytest.mark.parametrize(("functional", "expected"), [(False, 1), (True, 0)])
def test_cli_keeps_default_ten_second_gate(monkeypatch, tmp_path, functional, expected):
    def suite(*args, **kwargs):
        return {"scene_registration": {"accepted": True},
                "runs": [{"accepted": True, "scale": 1000, "elapsed_ms": 10001}]}
    monkeypatch.setattr(benchmark, "run_suite", suite)
    argv = ["benchmark_v2.py", "--suite", "strict", "--repeats", "1", "--output", str(tmp_path)]
    if functional:
        argv.append("--functional-only")
    monkeypatch.setattr(sys, "argv", argv)
    assert benchmark.main() == expected
