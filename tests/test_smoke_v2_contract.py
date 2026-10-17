"""Deployed smoke checks reject revision, schema, and Swagger drift."""
from copy import deepcopy
import hashlib
from io import BytesIO
import json
from urllib.error import HTTPError
from urllib.parse import urlsplit

import pytest

from tools import smoke_v2


SHA = "a" * 40
BASE = "http://candidate.invalid"


class FakeAPI:
    """HTTP boundary double; all smoke flows still consume actual JSON/HTML."""

    def __init__(self):
        self.sha = SHA
        self.native_available = True
        self.distance = 100
        self.retired_status = 404
        self.docs_status = 200
        self.docs = "<html><script>SwaggerUIBundle({url: '/openapi.json'})</script></html>"
        self.paths = []
        self.spec = {"openapi": "3.1.0", "paths": {"/api/v2/plan/single": {
            "post": {"responses": {"200": {"content": {"application/json": {
                "schema": {"$ref": "#/components/schemas/RouteResult"}}}}}}}},
            "components": {"schemas": {"RouteResult": {"type": "object"}}}}
        self.point = "POINT Z (104.77 29.33 80)"
        self.sample = {"scene": {"coordinates": {}}, "task": {
            "start_wkt": self.point, "departure_time": "2026-10-16T09:00:00+08:00"}}

    def __call__(self, request, timeout):
        assert timeout > 0
        path = urlsplit(request.full_url).path
        self.paths.append(path)
        body = json.loads(request.data) if request.data is not None else None
        if path.startswith("/api/v1/"):
            if self.retired_status != 200:
                raise HTTPError(request.full_url,self.retired_status,"Retired",{},BytesIO(b'{"detail":"Not Found"}'))
            return BytesIO(b'{"status":"ok"}')
        if path == "/docs":
            if self.docs_status != 200:
                raise HTTPError(request.full_url, self.docs_status, "Unavailable", {}, BytesIO())
            return BytesIO(self.docs.encode())
        if path == "/api/v2/plan/single" and "route_mode_typo" in body:
            raise HTTPError(request.full_url, 400, "Invalid input", {},
                            BytesIO(b'{"error":{"code":"INVALID_INPUT"}}'))
        if path == "/api/v2/coordinates/convert":
            result = {"geometries_wkt": ["POINT Z (100 100 80)" if body["direction"] == "wgs84_to_local" else self.point]}
        else:
            responses = {
                "/api/v2/health": {"status": "ok", "native_available": self.native_available},
                "/api/v2/version": {"api_version": "2.0.0", "engine": {"git_sha": self.sha}},
                "/openapi.json": self.spec,
                "/api/v2/examples/single": self.sample,
                "/api/v2/plan/single": {"status": "planned", "validation": {"valid": True},
                                         "metrics": {"distance_m": self.distance},
                                         "trajectory_wkt": self.point, "vertex_times_s": [0], "motion_segments": []},
                "/api/v2/validate": {"valid": True},
                "/api/v2/scenes": {"scene_id": "scene-one", "scene_version": "v1"},
                "/api/v2/scenes/scene-one": {"scene_version": "v1"},
                "/api/v2/examples/batch": {"tasks": [{"id": "one"}, {"id": "two"}]},
                "/api/v2/plan/batch": {"success_count": 2, "conflicts": []},
                "/api/v2/jobs": {"job_id": "job-one", "poll_url": "/api/v2/jobs/job-one"},
                "/api/v2/jobs/job-one": {"status": "succeeded", "result": {"success_count": 2}},
                "/api/v2/capabilities": {"rule_types": list(range(20))},
            }
            result = responses[path]
        return BytesIO(json.dumps(result).encode())


@pytest.fixture
def api(monkeypatch):
    value = FakeAPI()
    monkeypatch.setattr(smoke_v2.urllib.request, "urlopen", value)
    return value


def snapshot(tmp_path, spec):
    path = tmp_path / "openapi.json"
    path.write_text(json.dumps(spec, sort_keys=True, indent=2), encoding="utf-8")
    return path


def test_expected_revision_snapshot_and_swagger_accept_correct_service(api, tmp_path):
    result = smoke_v2.smoke(BASE, expected_git_sha=SHA, openapi_snapshot=snapshot(tmp_path, api.spec))
    assert result["status"] == "passed"
    assert result["native_available"] is True
    assert result["swagger_ui"] is True
    assert result["openapi_snapshot_matches"] is True
    assert "/docs" in api.paths


def test_default_smoke_checks_current_service(api):
    assert smoke_v2.smoke(BASE)["status"] == "passed"


def test_wrong_revision_is_rejected(api):
    with pytest.raises(AssertionError, match="revision"):
        smoke_v2.smoke(BASE, expected_git_sha="b" * 40)


def test_v2_example_must_have_positive_distance(api):
    api.distance = 0
    with pytest.raises(AssertionError, match="positive distance"):
        smoke_v2.smoke(BASE)


@pytest.mark.parametrize("status", [200, 503])
def test_retired_routes_must_return_404(api, status):
    api.retired_status = status
    with pytest.raises(AssertionError, match="v1"):
        smoke_v2.smoke(BASE)


@pytest.mark.parametrize("path", ["/api/v1/health", "/status"])
def test_openapi_must_only_contain_v2_routes(api, path):
    api.spec["paths"][path] = {"get": {}}
    with pytest.raises(AssertionError, match="only v2"):
        smoke_v2.smoke(BASE)


def test_http_archive_preserves_full_payloads_and_expected_errors(api, tmp_path):
    archive = tmp_path / "http"
    result = smoke_v2.smoke(BASE, archive_directory=archive)
    assert result["retired_v1_routes_absent"] is True
    manifest = json.loads((archive / "manifest.json").read_text())
    assert manifest["status"] == "passed"
    assert sum(case["http_status"] == 404 for case in manifest["exchanges"]) == 10
    assert any(case["http_status"] == 400 for case in manifest["exchanges"])
    for case in manifest["exchanges"]:
        for side in ("input", "output"):
            record = case[side]
            data = (archive / record["path"]).read_bytes()
            assert len(data) == record["bytes"]
            assert hashlib.sha256(data).hexdigest() == record["sha256"]
        if case["endpoint"] == "/docs":
            assert (archive / case["output"]["path"]).read_text() == api.docs


def test_failed_contract_archive_keeps_observed_payloads(api, tmp_path):
    api.distance = 0
    archive = tmp_path / "http"
    with pytest.raises(AssertionError, match="positive distance"):
        smoke_v2.smoke(BASE, archive_directory=archive)
    manifest = json.loads((archive / "manifest.json").read_text())
    assert manifest["status"] == "failed"
    actual = manifest["exchanges"][-1]
    assert json.loads((archive / actual["output"]["path"]).read_bytes())["metrics"]["distance_m"] == 0


def test_nested_openapi_drift_is_rejected(api, tmp_path):
    expected = deepcopy(api.spec)
    expected["components"]["schemas"]["RouteResult"]["required"] = ["validation"]
    with pytest.raises(AssertionError, match="OpenAPI"):
        smoke_v2.smoke(BASE, openapi_snapshot=snapshot(tmp_path, expected))


@pytest.mark.parametrize("html", ["<html>Unavailable</html>",
                                 "<script>SwaggerUIBundle({url: '/wrong.json'})</script>"])
def test_swagger_must_reference_runtime_openapi(api, html):
    api.docs = html
    with pytest.raises(AssertionError, match="Swagger"):
        smoke_v2.smoke(BASE)


def test_unavailable_swagger_is_rejected(api):
    api.docs_status = 503
    with pytest.raises(HTTPError):
        smoke_v2.smoke(BASE)


@pytest.mark.parametrize("native", [False, 1, "true"])
def test_native_availability_must_be_json_true(api, native):
    api.native_available = native
    with pytest.raises(AssertionError, match="native"):
        smoke_v2.smoke(BASE)


def test_cli_revision_failure_returns_nonzero(api, capsys):
    assert smoke_v2.main(["--base-url", BASE, "--expected-git-sha", "b" * 40]) == 1
    assert "revision" in capsys.readouterr().err


def test_cli_accepts_both_contract_options(api, tmp_path, capsys):
    assert smoke_v2.main(["--base-url", BASE, "--expected-git-sha", SHA,
                          "--openapi-snapshot", str(snapshot(tmp_path, api.spec))]) == 0
    assert json.loads(capsys.readouterr().out)["status"] == "passed"
