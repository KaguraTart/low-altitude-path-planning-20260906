"""Exercise all current v2 service endpoints through HTTP."""
from __future__ import annotations

import argparse
import hashlib
from io import BytesIO
import json
from pathlib import Path
import re
import sys
import time
import urllib.error
import urllib.request
from typing import Any

from shapely import from_wkt


class HTTPArchive:
    """Store exact request/response bytes before interpreting their contents."""

    def __init__(self, directory: str | Path, base_url: str):
        self.directory=Path(directory)
        self.directory.mkdir(parents=True,exist_ok=False)
        self.manifest={"format_version":1,"base_url":base_url,"status":"recording","exchanges":[]}

    def write(self, name: str, data: bytes) -> dict:
        with (self.directory/name).open("xb") as stream:
            stream.write(data)
        return {"path":name,"bytes":len(data),"sha256":hashlib.sha256(data).hexdigest()}

    def begin(self, path: str, data: bytes | None, as_text: bool) -> dict:
        method="GET" if data is None else "POST"
        stem=f"{len(self.manifest['exchanges'])+1:02d}_{method.lower()}_{re.sub(r'[^A-Za-z0-9_-]', '_', path.strip('/'))}"
        record={"method":method,"endpoint":path,"stem":stem,"as_text":as_text,
                "input":self.write(stem+"_input.json",data if data is not None else json.dumps({
                    "method":method,"endpoint":path,"body":None}).encode()),
                "input_representation":"http_json_body" if data is not None else "request_descriptor"}
        self.manifest["exchanges"].append(record)
        self.save()
        return record

    def complete(self, record: dict, data: bytes, status: int) -> None:
        record["output"]=self.write(record["stem"]+"_output"+(".html" if record["as_text"] else ".json"),data)
        record["http_status"]=status
        record["output_representation"]="http_response_bytes"
        self.save()

    def save(self, status: str | None = None) -> None:
        if status is not None:
            self.manifest["status"]=status
        (self.directory/"manifest.json").write_text(json.dumps(self.manifest,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")


def request(base_url: str, path: str, body: Any = None, *, as_text: bool = False,
            archive: HTTPArchive | None = None) -> Any:
    data=None if body is None else json.dumps(body,allow_nan=False).encode()
    req=urllib.request.Request(base_url.rstrip("/")+path,data,{"Content-Type":"application/json"})
    record=archive.begin(path,data,as_text) if archive is not None else None
    try:
        with urllib.request.urlopen(req,timeout=30) as response:
            raw=response.read()
            status=getattr(response,"status",200)
    except urllib.error.HTTPError as exc:
        raw=exc.read()
        if archive is not None:
            archive.complete(record,raw,exc.code)
        raise urllib.error.HTTPError(exc.url,exc.code,exc.reason,exc.headers,BytesIO(raw)) from exc
    except OSError as exc:
        if archive is not None:
            record["transport_error"]={"type":type(exc).__name__,"message":str(exc)}
            archive.save()
        raise
    if archive is not None:
        archive.complete(record,raw,status)
    return raw.decode("utf-8") if as_text else json.loads(raw)


def smoke(base_url: str, *, expected_git_sha: str | None = None,
          openapi_snapshot: str | Path | None = None,
          archive_directory: str | Path | None = None) -> dict:
    archive=HTTPArchive(archive_directory,base_url) if archive_directory is not None else None
    try:
        result=_smoke(base_url,expected_git_sha=expected_git_sha,openapi_snapshot=openapi_snapshot,archive=archive)
    except Exception:
        if archive is not None:
            archive.save("failed")
        raise
    if archive is not None:
        archive.save("passed")
    return result


def _smoke(base_url: str, *, expected_git_sha: str | None,
           openapi_snapshot: str | Path | None, archive: HTTPArchive | None) -> dict:
    def call(path: str, body: Any = None, *, as_text: bool = False) -> Any:
        return request(base_url,path,body,as_text=as_text,archive=archive)
    results={}
    health=call("/api/v2/health")
    assert health["status"]=="ok" and health["native_available"] is True, "v2 native engine must be explicitly available"
    job_health=call("/api/v2/health/jobs")
    job_statuses=("pending","running","succeeded","failed")
    assert all(type(job_health.get(name)) is int and job_health[name]>=0
               for name in (*job_statuses,"total")), "Job health must report nonnegative integer counts"
    assert job_health["total"]==sum(job_health[name] for name in job_statuses), "Job health counts must sum to total"
    version=call("/api/v2/version")
    if expected_git_sha is not None:
        actual_sha=version["engine"].get("git_sha")
        assert actual_sha==expected_git_sha, f"Service revision mismatch: expected {expected_git_sha}, got {actual_sha}"
    spec=call("/openapi.json")
    assert spec["paths"] and all(path.startswith("/api/v2/") for path in spec["paths"]), "OpenAPI must expose only v2 routes"
    if openapi_snapshot is not None:
        expected=json.loads(Path(openapi_snapshot).read_text(encoding="utf-8"))
        assert spec==expected, f"Runtime OpenAPI differs from snapshot {openapi_snapshot}"
    assert spec["paths"]["/api/v2/plan/single"]["post"]["responses"]["200"]["content"]["application/json"]["schema"]["$ref"].endswith("RouteResult")
    swagger=call("/docs",as_text=True)
    assert "SwaggerUIBundle" in swagger and re.search(r'''["']?url["']?\s*:\s*["']/openapi\.json["']''',swagger), "Swagger UI must reference /openapi.json"
    sample=call("/api/v2/examples/single")
    conversion={"coordinates":sample["scene"]["coordinates"],"alt_reference":"MSL",
                "at":sample["task"]["departure_time"],"geometries_wkt":[sample["task"]["start_wkt"]],
                "direction":"wgs84_to_local"}
    local=call("/api/v2/coordinates/convert",conversion)
    point=from_wkt(local["geometries_wkt"][0])
    assert max(abs(a-b) for a,b in zip(point.coords[0],(100,100,80)))<1e-5
    roundtrip=call("/api/v2/coordinates/convert",{
        **conversion,"direction":"local_to_wgs84","geometries_wkt":local["geometries_wkt"]})
    assert from_wkt(roundtrip["geometries_wkt"][0]).equals_exact(from_wkt(sample["task"]["start_wkt"]),1e-10)
    result=call("/api/v2/plan/single",sample)
    assert result["status"]=="planned" and result["validation"]["valid"],result
    assert result["metrics"]["distance_m"]>0, "v2 example route must have positive distance"
    validation={**sample,"trajectory_wkt":result["trajectory_wkt"],"vertex_times_s":result["vertex_times_s"],
                "motion_segments":result["motion_segments"]}
    validation.pop("time_budget_s",None)
    check=call("/api/v2/validate",validation)
    assert check["valid"],check
    scene=call("/api/v2/scenes",sample["scene"])
    assert call("/api/v2/scenes/"+scene["scene_id"])["scene_version"]==scene["scene_version"]
    batch=call("/api/v2/examples/batch")
    completed=call("/api/v2/plan/batch",batch)
    assert completed["success_count"]==len(batch["tasks"]) and not completed["conflicts"],completed
    submitted=call("/api/v2/jobs",batch)
    deadline=time.monotonic()+30
    while time.monotonic()<deadline:
        status=call(submitted["poll_url"])
        if status["status"] in {"succeeded","failed"}:
            break
        time.sleep(.1)
    else:
        raise AssertionError("Asynchronous job did not finish within 30s")
    assert status["status"]=="succeeded" and status["result"]["success_count"]==len(batch["tasks"]),status
    capabilities=call("/api/v2/capabilities")
    assert len(capabilities["rule_types"])==20
    bad={**sample,"route_mode_typo":"free"}
    try:
        call("/api/v2/plan/single",bad)
    except urllib.error.HTTPError as exc:
        assert exc.code==400
        error=json.load(exc)
        assert error["error"]["code"]=="INVALID_INPUT"
    else:
        raise AssertionError("Unknown constraint field was silently accepted")
    results.update(status="passed",base_url=base_url,api_version=version["api_version"],
                   git_sha=version["engine"].get("git_sha"),coordinate_roundtrip=True,
                   native_available=True,openapi_paths=len(spec["paths"]),scene_id=scene["scene_id"],
                   swagger_ui=True,job_health_checked=True,
                   batch_success_count=completed["success_count"],async_job_id=submitted["job_id"],
                   checked_at=time.strftime("%Y-%m-%dT%H:%M:%SZ",time.gmtime()))
    if openapi_snapshot is not None:
        results["openapi_snapshot_matches"]=True
    return results


def main(argv: list[str] | None = None) -> int:
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-url",default="http://127.0.0.1:18001")
    parser.add_argument("--expected-git-sha",help="Require the deployed /api/v2/version revision to match exactly")
    parser.add_argument("--openapi-snapshot",type=Path,help="Require runtime OpenAPI to equal this JSON snapshot")
    parser.add_argument("--archive-dir",type=Path,help="Preserve complete HTTP inputs and outputs in a new directory")
    args=parser.parse_args(argv)
    try:
        result=smoke(args.base_url,expected_git_sha=args.expected_git_sha,openapi_snapshot=args.openapi_snapshot,
                     archive_directory=args.archive_dir)
    except (AssertionError,OSError,ValueError,KeyError,TypeError) as exc:
        print(f"Smoke failed: {exc}",file=sys.stderr)
        return 1
    print(json.dumps(result,ensure_ascii=False,indent=2))
    return 0


if __name__=="__main__":
    raise SystemExit(main())
