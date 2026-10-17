"""Background jobs retain complete results until the process stops."""
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


def test_queue_accepts_more_than_the_previous_admission_limit():
    registry = JobRegistry(max_workers=1)
    gate = Event()
    try:
        ids = [registry.submit(gate.wait) for _ in range(201)]
        assert registry.stats()["total"] == len(ids)
        assert all(registry.get(jid) is not None for jid in ids)
        gate.set()
        assert finished(registry, ids[-1])["status"] == "succeeded"
    finally:
        gate.set()
        registry.shutdown(wait=True)


def test_completed_history_keeps_all_results():
    registry = JobRegistry(max_workers=1)
    try:
        result = {"values": list(range(1000))}
        ids = [registry.submit(lambda: result) for _ in range(201)]
        assert finished(registry, ids[-1])["status"] == "succeeded"
        assert registry.stats()["succeeded"] == len(ids)
        assert all(registry.get(jid)["result"] == result for jid in ids)
    finally:
        registry.shutdown(wait=True)


def test_memory_failure_is_pollable():
    registry = JobRegistry(max_workers=1)
    def exhausted():
        raise MemoryError("allocation failed")
    try:
        record = finished(registry, registry.submit(exhausted))
        assert record["status"] == "failed"
        assert record["result"] is None
        assert record["error"]["code"] == ErrorCode.RESOURCE_EXHAUSTED
        assert record["error"]["detail"]["type"] == "MemoryError"
    finally:
        registry.shutdown(wait=True)


@pytest.mark.parametrize("failure", [MemoryError, RuntimeError])
def test_submission_failure_does_not_leave_a_pending_record(monkeypatch, failure):
    registry = JobRegistry(max_workers=1)
    def fail_submit(*args, **kwargs):
        raise failure("submit failed")
    try:
        monkeypatch.setattr(registry._executor, "submit", fail_submit)
        with pytest.raises(failure):
            registry.submit(lambda: None)
        assert registry.stats()["total"] == 0
    finally:
        registry.shutdown(wait=True)


def test_missing_native_engine_is_explicit_in_job_error():
    registry = JobRegistry(max_workers=1)
    def unavailable():
        raise NativeUnavailableError("Compiled v2 engine is unavailable")
    try:
        record = finished(registry, registry.submit(unavailable))
        assert record["status"] == "failed"
        assert record["error"]["code"] == ErrorCode.NATIVE_UNAVAILABLE
    finally:
        registry.shutdown(wait=True)


def test_shutdown_cancels_pending_work_and_rejects_admission():
    registry = JobRegistry(max_workers=1)
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


def test_http_job_submission_during_shutdown_is_429(monkeypatch):
    module = import_module("server.v2")
    application = import_module("server.app").app
    registry = JobRegistry(max_workers=1)
    registry.shutdown(wait=True)
    monkeypatch.setattr(module, "jobs", registry)
    value = example_request()
    value["tasks"] = [value.pop("task")]
    response = TestClient(application).post("/api/v2/jobs", json=value)
    assert response.status_code == 429
    assert response.json()["error"]["code"] == ErrorCode.JOB_QUEUE_FULL


def test_application_lifespan_drains_current_registry(monkeypatch):
    module = import_module("server.app")
    v2 = import_module("server.v2")
    calls = []
    class Registry:
        def start(self):
            calls.append((self, "start"))
        def shutdown(self, wait=False):
            calls.append((self, wait))
    registry = Registry()
    monkeypatch.setattr(v2, "jobs", registry)
    with TestClient(module.app):
        assert calls == [(registry, "start")]
    assert calls == [(registry, "start"), (registry, True)]
    assert not hasattr(module, "_jobs")


def test_missing_native_engine_returns_503(monkeypatch):
    module = import_module("server.v2")
    application = import_module("server.app").app
    def unavailable(**kwargs):
        raise NativeUnavailableError("Compiled v2 engine is unavailable")
    monkeypatch.setattr(module, "plan_single_v2", unavailable)
    response = TestClient(application).post("/api/v2/plan/single", json=example_request())
    assert response.status_code == 503
    assert response.json()["error"]["code"] == ErrorCode.NATIVE_UNAVAILABLE


def test_memory_failure_returns_503_for_current_api(monkeypatch):
    module = import_module("server.app")
    v2 = import_module("server.v2")
    def exhausted(**kwargs):
        raise MemoryError("allocation failed")
    monkeypatch.setattr(v2, "plan_single_v2", exhausted)
    client = TestClient(module.app)
    response = client.post("/api/v2/plan/single", json=example_request())
    assert response.status_code == 503
    assert response.json()["error"]["code"] == ErrorCode.RESOURCE_EXHAUSTED
    assert response.json()["error"]["detail"]["type"] == "MemoryError"
