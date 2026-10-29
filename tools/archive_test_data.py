#!/usr/bin/env python3
"""Capture complete HTTP examples and frozen workloads in a reusable data bundle."""
from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
import time
import urllib.error
import urllib.request
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def encoded(value: object) -> bytes:
    return (json.dumps(value, ensure_ascii=False, allow_nan=False, indent=2) + "\n").encode()


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def verify_archive(directory: Path) -> dict:
    """Verify every listed byte sequence and prevent references outside the bundle."""
    directory = directory.resolve()
    manifest = json.loads((directory / "manifest.json").read_bytes())
    actual = {p.relative_to(directory).as_posix() for p in directory.rglob("*")
              if p.is_file() and p != directory / "manifest.json"}
    if actual != {item["path"] for item in manifest["files"]}:
        raise ValueError("Archive file inventory mismatch")
    for item in manifest["files"]:
        path = (directory / item["path"]).resolve()
        if not path.is_relative_to(directory):
            raise ValueError("Archive file escapes its bundle")
        data = path.read_bytes()
        if len(data) != item["bytes"] or digest(data) != item["sha256"]:
            raise ValueError(f"Archive integrity mismatch: {item['path']}")
        if path.suffix == ".json":
            json.loads(data)
    return manifest


class Recorder:
    def __init__(self, base_url: str, directory: Path):
        self.base_url = base_url.rstrip("/")
        self.directory = directory
        directory.mkdir(parents=True, exist_ok=False)
        self.manifest = {
            "format_version": 1,
            "captured_at": datetime.now(timezone.utc).isoformat(),
            "purpose": "functional_replay_capture",
            "base_url": self.base_url,
            "status": "recording",
            "collector_sha256": digest(Path(__file__).read_bytes()),
            "cases": [], "benchmarks": [], "files": [],
        }

    def write(self, relative: str, data: bytes) -> None:
        path = self.directory / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("xb") as stream:
            stream.write(data)

    def exchange(self, name: str, method: str, endpoint: str, body=None,
                 expected_status: int = 200) -> dict:
        raw_input = None if body is None else encoded(body)
        input_path, output_path = f"inputs/{name}_input.json", f"outputs/{name}_output.json"
        self.write(input_path, raw_input if raw_input is not None else encoded({
            "method": method, "endpoint": endpoint, "body": None}))
        request = urllib.request.Request(self.base_url + endpoint, raw_input,
                                         {"Content-Type": "application/json"}, method=method)
        start = time.perf_counter()
        try:
            response = urllib.request.urlopen(request, timeout=150)
        except urllib.error.HTTPError as exc:
            response = exc
        with response:
            data, status = response.read(), response.status
        elapsed_ms = (time.perf_counter() - start) * 1000
        self.write(output_path, data)
        self.manifest["cases"].append({
            "name": name, "method": method, "endpoint": endpoint,
            "input": input_path, "output": output_path,
            "input_representation": "http_json_body" if raw_input is not None else "request_descriptor",
            "output_representation": "http_response_bytes", "http_status": status,
            "elapsed_ms": elapsed_ms,
        })
        if status != expected_status:
            raise RuntimeError(f"{name}: HTTP {status}; complete response saved at {output_path}")
        return json.loads(data)

    def async_case(self, version: str, request: dict) -> None:
        submitted = self.exchange(f"{version}_async_submit", "POST", f"/api/{version}/jobs",
                                  request, expected_status=202)
        deadline = time.monotonic() + 120
        for index in range(1, 1201):
            result = self.exchange(f"{version}_async_poll{index:03d}", "GET", submitted["poll_url"])
            if result["status"] in {"succeeded", "failed"}:
                assert result["status"] == "succeeded", result
                assert result["result"]["success_count"] == len(request["tasks"]), result
                return
            if time.monotonic() >= deadline:
                break
            time.sleep(.1)
        raise RuntimeError(f"{version} asynchronous capture timed out")

    def finish(self, status: str) -> None:
        self.manifest["status"] = status
        self.manifest["files"] = []
        for path in sorted(self.directory.rglob("*")):
            if path.is_file() and path != self.directory / "manifest.json":
                data = path.read_bytes()
                self.manifest["files"].append({"path": path.relative_to(self.directory).as_posix(),
                                               "bytes": len(data), "sha256": digest(data)})
        # The manifest is updated during this capture; payload files never overwrite.
        (self.directory / "manifest.json").write_bytes(encoded(self.manifest))


def capture(base_url: str, directory: Path, container: str | None, repeats: int = 1) -> dict:
    from tools.benchmark_v2 import SUITES, run_suite

    recorder = Recorder(base_url, directory)
    try:
        version = recorder.exchange("service_version", "GET", "/api/v2/version")
        environment = {"service_version": version,
                       "source_revision": subprocess.check_output(
                           ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()}
        if container:
            info = json.loads(subprocess.check_output(["docker", "inspect", container], text=True))[0]
            environment["container"] = {
                "image_id": info["Image"], "revision": info["Config"]["Labels"].get(
                    "org.opencontainers.image.revision"),
                "nano_cpus": info["HostConfig"]["NanoCpus"],
                "memory_bytes": info["HostConfig"]["Memory"],
                "memory_swap_bytes": info["HostConfig"]["MemorySwap"],
            }
        recorder.write("environment.json", encoded(environment))
        recorder.exchange("service_health", "GET", "/api/v2/health")
        recorder.exchange("service_capabilities", "GET", "/api/v2/capabilities")
        for source in sorted((ROOT / "input" / "engine_contract_20261029").rglob("*.json")):
            recorder.write("source_inputs/" + source.relative_to(ROOT / "input").as_posix(),
                           source.read_bytes())
        single_v2 = recorder.exchange("v2_single_example", "GET", "/api/v2/examples/single")
        strict_result = None
        for mode in ("strict", "prefer", "free"):
            request = {**single_v2, "task": {**single_v2["task"], "route_mode": mode}}
            result = recorder.exchange(f"v2_single_{mode}", "POST", "/api/v2/plan/single", request)
            assert result["status"] == "planned" and result["validation"]["valid"], result
            if mode == "strict":
                strict_result = result
        batch_v2 = recorder.exchange("v2_batch_example", "GET", "/api/v2/examples/batch")
        result = recorder.exchange("v2_batch", "POST", "/api/v2/plan/batch", batch_v2)
        assert result["success_count"] == len(batch_v2["tasks"]) and not result["conflicts"], result
        recorder.async_case("v2", batch_v2)
        conversion = {"coordinates": single_v2["scene"]["coordinates"], "alt_reference": "EG",
                      "at": single_v2["task"]["departure_time"],
                      "geometries_wkt": [single_v2["task"]["start_wkt"]], "direction": "wgs84_to_local"}
        local = recorder.exchange("v2_coordinates_to_local", "POST", "/api/v2/coordinates/convert", conversion)
        recorder.exchange("v2_coordinates_to_wgs84", "POST", "/api/v2/coordinates/convert",
                          {**conversion, "direction": "local_to_wgs84", "geometries_wkt": local["geometries_wkt"]})
        validation = {**single_v2, "trajectory_wkt": strict_result["trajectory_wkt"],
                      "vertex_times_s": strict_result["vertex_times_s"],
                      "motion_segments": strict_result["motion_segments"]}
        validation.pop("time_budget_s", None)
        result = recorder.exchange("v2_validate", "POST", "/api/v2/validate", validation)
        assert result["valid"], result

        for suite in SUITES:
            report = run_suite(suite, repeats, [1000], directory / "benchmarks", 120, base_url,
                               directory / "source_inputs" / "v2_benchmarks", functional_only=True)
            assert (report["accepted"] and report["completed"]
                    and len(report["runs"]) == repeats
                    and all(run["accepted"] for run in report["runs"])), suite
            recorder.manifest["benchmarks"].append({
                "suite": suite, "task_count": 1000, "repeats": repeats,
                "report": f"benchmarks/{suite}-report.json", "input_sha256": report["input_sha256"]})
        recorder.finish("passed")
    except Exception:
        recorder.finish("failed")
        raise
    return verify_archive(directory)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-url", default="http://127.0.0.1:18000")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--container", help="Record the local service's actual Docker resource limits")
    parser.add_argument("--repeats", type=int, default=1)
    parser.add_argument("--verify", type=Path, help="Check an existing bundle without contacting a service")
    args = parser.parse_args(argv)
    if args.repeats < 1:
        parser.error("--repeats must be positive")
    if args.verify:
        manifest = verify_archive(args.verify)
        print(json.dumps({"status": "verified", "files": len(manifest["files"])}, indent=2))
        return 0
    directory = args.output or ROOT / "tests" / "data" / "recordings" / (
        datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "_http_functional")
    manifest = capture(args.base_url, directory, args.container, args.repeats)
    print(json.dumps({"status": manifest["status"], "directory": str(directory),
                      "cases": len(manifest["cases"]), "benchmarks": manifest["benchmarks"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
