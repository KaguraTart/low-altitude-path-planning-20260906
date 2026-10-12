"""
REST 服务端点测试

用 FastAPI 的 TestClient（不占端口、不需要真的起服务），覆盖：
- 全部端点的正常路径
- 统一错误信封与 HTTP 状态码映射
- 响应必须是合法 JSON（无 NaN/Infinity 字面量）
- 异步作业的提交与轮询
"""

import json
import math
import time

import pytest

fastapi = pytest.importorskip("fastapi", reason="未安装 fastapi")

from fastapi.testclient import TestClient  # noqa: E402

from server.app import app  # noqa: E402


@pytest.fixture(scope="module")
def client():
    return TestClient(app)


def _assert_clean_json(payload):
    """响应必须是严格合法 JSON：不允许 NaN/Infinity 字面量，也不允许 numpy 类型"""
    text = json.dumps(payload)
    assert "NaN" not in text and "Infinity" not in text

    def walk(o):
        if isinstance(o, dict):
            for v in o.values():
                walk(v)
        elif isinstance(o, list):
            for v in o:
                walk(v)
        elif isinstance(o, float):
            assert math.isfinite(o)
        elif isinstance(o, (str, int, bool)) or o is None:
            pass
        else:
            pytest.fail(f"响应含非原生类型 {type(o).__name__}")

    walk(payload)


# ---------------------------------------------------------------- 基础端点

class TestBasicEndpoints:
    def test_health(self, client):
        r = client.get("/api/v1/health")
        assert r.status_code == 200
        body = r.json()
        assert body["status"] == "ok"
        assert body["uptime_s"] >= 0

    def test_version(self, client):
        r = client.get("/api/v1/version")
        assert r.status_code == 200
        body = r.json()
        assert body["api_version"]
        assert "astar" in body["algorithms"]
        # 默认网格 50x50x10x120 = 300 万格
        assert body["defaults"]["grid_cells"] == 3_000_000

    def test_jobs_health(self, client):
        r = client.get("/api/v1/health/jobs")
        assert r.status_code == 200
        assert "total" in r.json()

    def test_openapi_schema_available(self, client):
        """Swagger 文档是联调的主要入口，必须能生成"""
        r = client.get("/openapi.json")
        assert r.status_code == 200
        paths = r.json()["paths"]
        for p in ("/api/v1/plan/single", "/api/v1/plan/batch",
                  "/api/v1/validate", "/api/v1/jobs"):
            assert p in paths


# ---------------------------------------------------------------- 规划端点

class TestPlanSingleEndpoint:
    def test_example_roundtrip(self, client):
        """取示例请求体直接打回去，应得到完整结构——这是对接方最快的起步方式"""
        ex = client.get("/api/v1/examples/single").json()
        r = client.post("/api/v1/plan/single", json=ex)
        assert r.status_code == 200
        body = r.json()
        assert body["status"] == "planned"
        assert body["route"]["metrics"]["total_distance_m"] > 0
        assert body["validation"]["is_valid"] is True
        _assert_clean_json(body)

    def test_minimal_request(self, client):
        r = client.post("/api/v1/plan/single", json={
            "start": {"x": 500, "y": 500, "z": 100},
            "goal": {"x": 1500, "y": 500, "z": 100},
        })
        assert r.status_code == 200
        assert r.json()["status"] == "planned"

    def test_waypoints_request(self, client):
        r = client.post("/api/v1/plan/single", json={
            "waypoints": [
                {"id": "s", "x": 500, "y": 500, "z": 100, "type": "start"},
                {"id": "w", "x": 500, "y": 4000, "z": 120, "type": "via"},
                {"id": "e", "x": 4000, "y": 4000, "z": 150, "type": "end"},
            ]
        })
        assert r.status_code == 200
        assert r.json()["status"] == "planned"

    def test_semantic_error_returns_400_envelope(self, client):
        """缺 start/goal：FastAPI 形状校验通过，由 SDK 判定为 INVALID_INPUT"""
        r = client.post("/api/v1/plan/single", json={"task_id": "x"})
        assert r.status_code == 400
        err = r.json()["error"]
        assert err["code"] == "INVALID_INPUT"
        assert err["message"]

    def test_shape_error_returns_400_envelope(self, client):
        """类型错误（z 不是数字）：由 FastAPI 拦下，也要套同一信封"""
        r = client.post("/api/v1/plan/single", json={
            "start": {"x": 1, "y": 2, "z": "not-a-number"},
            "goal": {"x": 3, "y": 4, "z": 5},
        })
        assert r.status_code == 400
        assert r.json()["error"]["code"] == "INVALID_INPUT"

    def test_planning_failure_still_200(self, client):
        """规划不出来是业务结果，不是 HTTP 错误"""
        r = client.post("/api/v1/plan/single", json={
            "start": {"x": 100, "y": 100, "z": 100},
            "goal": {"x": 200, "y": 200, "z": 100},
            "static_obstacles": [{
                "id": "wall", "name": "墙", "type": "building",
                "x": 2500, "y": 2500, "z": 0,
                "width": 3000, "depth": 3000, "height": 400, "source": "test",
            }],
        })
        assert r.status_code == 200
        assert r.json()["status"] == "failed"


class TestPlanBatchEndpoint:
    def test_example_roundtrip(self, client):
        ex = client.get("/api/v1/examples/batch").json()
        r = client.post("/api/v1/plan/batch", json=ex)
        assert r.status_code == 200
        body = r.json()
        assert body["success_count"] >= 1
        assert "conflicts" in body
        _assert_clean_json(body)

    def test_missing_aircraft_returns_404(self, client):
        ex = client.get("/api/v1/examples/batch").json()
        ex["tasks"] = [ex["tasks"][0]]
        ex["tasks"][0]["aircraft_id"] = "does-not-exist"
        r = client.post("/api/v1/plan/batch", json=ex)
        assert r.status_code == 404
        err = r.json()["error"]
        assert err["code"] == "AIRCRAFT_NOT_FOUND"
        assert "does-not-exist" in err["detail"]["missing"]


class TestValidateEndpoint:
    def test_valid_route(self, client):
        r = client.post("/api/v1/validate", json={
            "route": {
                "route_id": "ext-1",
                "trajectory": [
                    {"x": 0, "y": 0, "z": 50, "t": 0},
                    {"x": 100, "y": 0, "z": 50, "t": 10},
                ],
            }
        })
        assert r.status_code == 200
        assert r.json()["is_valid"] is True

    def test_collision_detected(self, client):
        r = client.post("/api/v1/validate", json={
            "route": {
                "route_id": "ext-bad",
                "trajectory": [
                    {"x": 2500, "y": 2500, "z": 100, "t": 0},
                    {"x": 3000, "y": 2500, "z": 100, "t": 10},
                ],
            },
            "static_obstacles": [{
                "id": "b1", "name": "楼", "type": "building",
                "x": 2500, "y": 2500, "z": 0,
                "width": 80, "depth": 80, "height": 200, "source": "test",
            }],
        })
        assert r.status_code == 200
        body = r.json()
        assert body["is_valid"] is False
        assert any(i["code"] == "STATIC_OBSTACLE_COLLISION" for i in body["issues"])

    def test_missing_trajectory_400(self, client):
        r = client.post("/api/v1/validate", json={"route": {"route_id": "x"}})
        assert r.status_code == 400
        assert r.json()["error"]["code"] == "INVALID_INPUT"


# ---------------------------------------------------------------- 异步作业

class TestJobs:
    def _wait(self, client, job_id, timeout=60.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            body = client.get(f"/api/v1/jobs/{job_id}").json()
            if body["status"] in ("succeeded", "failed"):
                return body
            time.sleep(0.2)
        pytest.fail(f"作业 {job_id} 在 {timeout}s 内未完成")

    def test_submit_and_poll(self, client):
        ex = client.get("/api/v1/examples/batch").json()
        r = client.post("/api/v1/jobs", json=ex)
        assert r.status_code == 202
        submitted = r.json()
        assert submitted["job_id"]
        assert submitted["poll_url"].endswith(submitted["job_id"])

        body = self._wait(client, submitted["job_id"])
        assert body["status"] == "succeeded", body.get("error")
        assert body["result"]["success_count"] >= 1
        assert body["elapsed_ms"] is not None
        _assert_clean_json(body)

    def test_job_failure_records_error_envelope(self, client):
        """后台作业失败时，错误要能被轮询方看到，而不是只打在 stderr"""
        ex = client.get("/api/v1/examples/batch").json()
        ex["tasks"] = [ex["tasks"][0]]
        ex["tasks"][0]["aircraft_id"] = "nope"
        r = client.post("/api/v1/jobs", json=ex)
        body = self._wait(client, r.json()["job_id"])
        assert body["status"] == "failed"
        assert body["error"]["code"] == "AIRCRAFT_NOT_FOUND"

    def test_unknown_job_404(self, client):
        r = client.get("/api/v1/jobs/deadbeef")
        assert r.status_code == 404
        assert r.json()["error"]["code"] == "TASK_NOT_FOUND"


# ---------------------------------------------------------------- 示例端点

class TestExamples:
    def test_single_example_is_usable(self, client):
        body = client.get("/api/v1/examples/single").json()
        assert "start" in body and "goal" in body

    def test_batch_example_is_usable(self, client):
        body = client.get("/api/v1/examples/batch").json()
        assert len(body["tasks"]) >= 1
        ids = {a["id"] for a in body["aircraft"]}
        for t in body["tasks"]:
            assert t["aircraft_id"] in ids, "示例里任务引用的飞机必须存在"
