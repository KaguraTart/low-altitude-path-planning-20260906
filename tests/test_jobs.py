"""Bounded background admission, retention, and application shutdown."""
from importlib import import_module
from threading import Event
from time import monotonic, sleep

import pytest
from fastapi.testclient import TestClient

from server.jobs import JobRegistry
from server.v2 import example_request
from uav_planner import ErrorCode, PlanningError
from uav_planner.v2.native import NativeUnavailableError


def finished(registry, job_id):
    deadline = monotonic() + 5
    while monotonic() < deadline:
        record = registry.get(job_id)
        if record is None or record["status"] in {"succeeded", "failed"}:
            return record
        sleep(.005)
    pytest.fail("Background job did not finish")


def test_queue_rejects_overload_without_retaining_more_requests():
    registry = JobRegistry(max_workers=1, max_jobs=2)
    gate = Event()
    try:
        registry.submit(gate.wait)
        registry.submit(gate.wait)
        with pytest.raises(PlanningError) as error:
            registry.submit(gate.wait)
        assert error.value.code == ErrorCode.JOB_QUEUE_FULL
        assert registry.stats()["total"] == 2
    finally:
        gate.set()
        registry.shutdown(wait=True)


def test_completion_evicts_excess_history_and_result_bytes():
    registry = JobRegistry(max_workers=1, max_jobs=3, max_completed_bytes=2000)
    try:
        first = registry.submit(lambda: "x" * 1500)
        assert finished(registry, first)["status"] == "succeeded"
        second = registry.submit(lambda: "y" * 1500)
        assert finished(registry, second)["status"] == "succeeded"
        assert registry.get(first) is None
        assert registry.stats()["total"] == 1
    finally:
        registry.shutdown(wait=True)


def test_large_result_remains_pollable_as_explicit_failure():
    registry = JobRegistry(max_workers=1, max_jobs=2, max_completed_bytes=100)
    try:
        job_id = registry.submit(lambda: "x" * 1000)
        result = finished(registry, job_id)
        assert result["status"] == "failed"
        assert result["error"]["code"] == ErrorCode.JOB_RESULT_TOO_LARGE
        assert result["result"] is None
    finally:
        registry.shutdown(wait=True)


def test_completed_history_remains_bounded_without_a_later_submission():
    registry = JobRegistry(max_workers=1, max_jobs=2)
    try:
        ids = []
        for _ in range(3):
            job_id = registry.submit(lambda: None)
            ids.append(job_id)
            assert finished(registry, job_id)["status"] == "succeeded"
        assert registry.get(ids[0]) is None
        assert registry.stats()["total"] == 2
    finally:
        registry.shutdown(wait=True)


def test_missing_native_engine_is_explicit_in_job_error():
    registry = JobRegistry(max_workers=1, max_jobs=2)
    def unavailable():
        raise NativeUnavailableError("Compiled v2 engine is unavailable")
    try:
        record = finished(registry, registry.submit(unavailable))
        assert record["status"] == "failed"
        assert record["error"]["code"] == ErrorCode.NATIVE_UNAVAILABLE
    finally:
        registry.shutdown(wait=True)


def test_shutdown_cancels_pending_work_and_rejects_admission():
    registry = JobRegistry(max_workers=1, max_jobs=2)
    running = Event()
    gate = Event()
    def block():
        running.set()
        gate.wait()
    try:
        registry.submit(block)
        assert running.wait(5)
        queued = registry.submit(lambda: pytest.fail("Cancelled work executed"))
        registry.shutdown(wait=False)
        assert registry.get(queued)["error"]["code"] == "JOB_CANCELLED"
        with pytest.raises(PlanningError, match="shutting down"):
            registry.submit(lambda: None)
    finally:
        gate.set()
        registry.shutdown(wait=True)


def test_http_job_overload_is_429(monkeypatch):
    module = import_module("server.v2")
    application = import_module("server.app").app
    registry = JobRegistry(max_workers=1, max_jobs=1)
    gate = Event()
    try:
        registry.submit(gate.wait)
        monkeypatch.setattr(module, "jobs", registry)
        value = example_request()
        value["tasks"] = [value.pop("task")]
        response = TestClient(application).post("/api/v2/jobs", json=value)
        assert response.status_code == 429
        assert response.json()["error"]["code"] == ErrorCode.JOB_QUEUE_FULL
    finally:
        gate.set()
        registry.shutdown(wait=True)


def test_application_lifespan_drains_both_registries(monkeypatch):
    module = import_module("server.app")
    v2 = import_module("server.v2")
    calls = []
    class Registry:
        def start(self):
            calls.append((self, "start"))
        def shutdown(self, wait=False):
            calls.append((self, wait))
    first, second = Registry(), Registry()
    monkeypatch.setattr(module, "_jobs", first)
    monkeypatch.setattr(v2, "jobs", second)
    with TestClient(module.app):
        assert (first, "start") in calls
        assert (second, "start") in calls
    assert (first, True) in calls
    assert (second, True) in calls


def test_missing_native_engine_returns_503(monkeypatch):
    module = import_module("server.v2")
    application = import_module("server.app").app
    def unavailable(**kwargs):
        raise NativeUnavailableError("Compiled v2 engine is unavailable")
    monkeypatch.setattr(module, "plan_single_v2", unavailable)
    response = TestClient(application).post("/api/v2/plan/single", json=example_request())
    assert response.status_code == 503
    assert response.json()["error"]["code"] == ErrorCode.NATIVE_UNAVAILABLE
