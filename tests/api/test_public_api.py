"""Public HTTP and package-root SDK expose one geographic planning contract."""
import json
from time import monotonic, sleep

import pytest
from fastapi.testclient import TestClient

import uav_planner as sdk
from server.app import app
from server import schemas, v2


@pytest.fixture
def client():
    with TestClient(app) as value:
        yield value


def test_openapi_and_swagger_expose_only_current_routes(client):
    response = client.get("/openapi.json")
    assert response.status_code == 200
    schema = response.json()
    assert schema["paths"]
    assert all(path.startswith("/api/v2/") for path in schema["paths"])
    assert {"/api/v2/scenes", "/api/v2/coordinates/convert", "/api/v2/plan/single",
            "/api/v2/plan/batch", "/api/v2/validate", "/api/v2/jobs"} <= schema["paths"].keys()
    assert schemas.SingleRequest is v2.SingleRequest
    assert schemas.BatchRequest is v2.BatchRequest
    assert schemas.ValidateRequest is v2.ValidateRequest
    docs = client.get("/docs")
    assert docs.status_code == 200
    assert "SwaggerUIBundle" in docs.text
    assert "/openapi.json" in docs.text


def test_current_health_version_and_sdk_metadata_match(client):
    health = client.get("/api/v2/health")
    assert health.status_code == 200
    assert health.json()["status"] == "ok"
    assert health.json()["native_available"] is True
    assert client.get("/api/v2/version").json() == sdk.get_version()
    assert set(client.get("/api/v2/health/jobs").json()) >= {
        "pending", "running", "succeeded", "failed", "total",
    }


def test_http_single_example_round_trips_with_analytic_validation(client):
    example = client.get("/api/v2/examples/single").json()
    result = client.post("/api/v2/plan/single", json=example)
    assert result.status_code == 200
    route = result.json()
    assert route["status"] == "planned"
    assert route["validation"]["valid"]
    assert route["metrics"]["distance_m"] > 0
    check = client.post("/api/v2/validate", json={"scene": example["scene"], "task": example["task"],
        "trajectory_wkt": route["trajectory_wkt"], "vertex_times_s": route["vertex_times_s"],
        "motion_segments": route["motion_segments"]})
    assert check.status_code == 200
    assert check.json()["valid"]


def test_http_current_batch_example_completes_as_a_job(client):
    example = client.get("/api/v2/examples/batch").json()
    submitted = client.post("/api/v2/jobs", json=example)
    assert submitted.status_code == 202
    poll_url = submitted.json()["poll_url"]
    assert poll_url.startswith("/api/v2/jobs/")
    deadline = monotonic() + 5
    while monotonic() < deadline:
        response = client.get(poll_url)
        assert response.status_code == 200
        job = response.json()
        if job["status"] in {"succeeded", "failed"}:
            break
        sleep(.005)
    else:
        pytest.fail("Current API job did not finish")
    assert job["status"] == "succeeded", job
    assert job["result"]["success_count"] == len(example["tasks"])
    assert job["result"]["conflicts"] == []


def test_package_root_aliases_use_current_scene_and_motion_contract():
    assert sdk.plan_single is sdk.plan_single_v2
    assert sdk.plan_batch is sdk.plan_batch_v2
    assert sdk.validate_route is sdk.validate_route_v2
    example = v2.example_request()
    scene_id = sdk.register_scene(example["scene"])["scene_id"]
    route = sdk.plan_single(scene_id=scene_id, task=example["task"])
    assert route["status"] == "planned"
    assert route == sdk.plan_single_v2(scene_id=scene_id, task=example["task"])
    parameters = {"scene_id": scene_id, "task": example["task"],
        "trajectory_wkt": route["trajectory_wkt"], "vertex_times_s": route["vertex_times_s"],
        "motion_segments": route["motion_segments"]}
    assert sdk.validate_route(**parameters)["valid"]
    assert sdk.validate_route_v2(**parameters)["valid"]
    tasks = [example["task"], {**example["task"], "id": "sdk-second"}]
    assert sdk.plan_batch(scene_id=scene_id, tasks=tasks)["success_count"] == 2
    assert sdk.plan_batch_v2(scene_id=scene_id, tasks=tasks)["success_count"] == 2
    converted = sdk.convert_coordinates({"coordinates": example["scene"]["coordinates"],
        "direction": "wgs84_to_local", "alt_reference": "EG",
        "geometries_wkt": [example["task"]["start_wkt"]]})
    assert isinstance(converted, dict)
    assert converted["geometries_wkt"][0].startswith("POINT Z")
    json.dumps(route, allow_nan=False)
