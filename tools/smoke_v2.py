"""Exercise deployed v1/v2 contracts, including real asynchronous job polling."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import sys
import time
import urllib.error
import urllib.request
from typing import Any

from shapely import from_wkt


def request(base_url: str, path: str, body: Any = None, *, as_text: bool = False) -> Any:
    data=None if body is None else json.dumps(body,allow_nan=False).encode()
    req=urllib.request.Request(base_url.rstrip("/")+path,data,{"Content-Type":"application/json"})
    with urllib.request.urlopen(req,timeout=30) as response:
        return response.read().decode("utf-8") if as_text else json.load(response)


def smoke(base_url: str, *, expected_git_sha: str | None = None,
          openapi_snapshot: str | Path | None = None) -> dict:
    results={}
    assert request(base_url,"/api/v1/health")["status"]=="ok"
    health=request(base_url,"/api/v2/health")
    assert health["status"]=="ok" and health["native_available"] is True, "v2 native engine must be explicitly available"
    version=request(base_url,"/api/v2/version")
    if expected_git_sha is not None:
        actual_sha=version["engine"].get("git_sha")
        assert actual_sha==expected_git_sha, f"Service revision mismatch: expected {expected_git_sha}, got {actual_sha}"
    spec=request(base_url,"/openapi.json")
    if openapi_snapshot is not None:
        expected=json.loads(Path(openapi_snapshot).read_text(encoding="utf-8"))
        assert spec==expected, f"Runtime OpenAPI differs from snapshot {openapi_snapshot}"
    assert spec["paths"]["/api/v2/plan/single"]["post"]["responses"]["200"]["content"]["application/json"]["schema"]["$ref"].endswith("RouteResult")
    swagger=request(base_url,"/docs",as_text=True)
    assert "SwaggerUIBundle" in swagger and re.search(r'''["']?url["']?\s*:\s*["']/openapi\.json["']''',swagger), "Swagger UI must reference /openapi.json"
    old=request(base_url,"/api/v1/examples/single")
    old_result=request(base_url,"/api/v1/plan/single",old)
    assert old_result["status"]=="planned",old_result
    assert old_result["route"]["metrics"]["total_distance_m"]>0, "v1 planned route must have positive distance"
    sample=request(base_url,"/api/v2/examples/single")
    conversion={"coordinates":sample["scene"]["coordinates"],"alt_reference":"MSL",
                "at":sample["task"]["departure_time"],"geometries_wkt":[sample["task"]["start_wkt"]],
                "direction":"wgs84_to_local"}
    local=request(base_url,"/api/v2/coordinates/convert",conversion)
    point=from_wkt(local["geometries_wkt"][0])
    assert max(abs(a-b) for a,b in zip(point.coords[0],(100,100,80)))<1e-5
    roundtrip=request(base_url,"/api/v2/coordinates/convert",{
        **conversion,"direction":"local_to_wgs84","geometries_wkt":local["geometries_wkt"]})
    assert from_wkt(roundtrip["geometries_wkt"][0]).equals_exact(from_wkt(sample["task"]["start_wkt"]),1e-10)
    result=request(base_url,"/api/v2/plan/single",sample)
    assert result["status"]=="planned" and result["validation"]["valid"],result
    validation={**sample,"trajectory_wkt":result["trajectory_wkt"],"vertex_times_s":result["vertex_times_s"],
                "motion_segments":result["motion_segments"]}
    validation.pop("time_budget_s",None)
    check=request(base_url,"/api/v2/validate",validation)
    assert check["valid"],check
    scene=request(base_url,"/api/v2/scenes",sample["scene"])
    assert request(base_url,"/api/v2/scenes/"+scene["scene_id"])["scene_version"]==scene["scene_version"]
    batch=request(base_url,"/api/v2/examples/batch")
    completed=request(base_url,"/api/v2/plan/batch",batch)
    assert completed["success_count"]==len(batch["tasks"]) and not completed["conflicts"],completed
    submitted=request(base_url,"/api/v2/jobs",batch)
    deadline=time.monotonic()+30
    while time.monotonic()<deadline:
        status=request(base_url,submitted["poll_url"])
        if status["status"] in {"succeeded","failed"}:
            break
        time.sleep(.1)
    else:
        raise AssertionError("Asynchronous job did not finish within 30s")
    assert status["status"]=="succeeded" and status["result"]["success_count"]==len(batch["tasks"]),status
    capabilities=request(base_url,"/api/v2/capabilities")
    assert len(capabilities["rule_types"])==20
    bad={**sample,"route_mode_typo":"free"}
    try:
        request(base_url,"/api/v2/plan/single",bad)
    except urllib.error.HTTPError as exc:
        assert exc.code==400
        error=json.load(exc)
        assert error["error"]["code"]=="INVALID_INPUT"
    else:
        raise AssertionError("Unknown constraint field was silently accepted")
    results.update(status="passed",base_url=base_url,api_version=version["api_version"],
                   git_sha=version["engine"].get("git_sha"),coordinate_roundtrip=True,
                   native_available=True,openapi_paths=len(spec["paths"]),scene_id=scene["scene_id"],
                   swagger_ui=True,
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
    args=parser.parse_args(argv)
    try:
        result=smoke(args.base_url,expected_git_sha=args.expected_git_sha,openapi_snapshot=args.openapi_snapshot)
    except (AssertionError,OSError,ValueError,KeyError,TypeError) as exc:
        print(f"Smoke failed: {exc}",file=sys.stderr)
        return 1
    print(json.dumps(result,ensure_ascii=False,indent=2))
    return 0


if __name__=="__main__":
    raise SystemExit(main())
