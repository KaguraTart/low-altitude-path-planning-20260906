"""Recorded requests and complete responses remain intact and self contained."""
import json
from pathlib import Path

import pytest

from tools.archive_test_data import Recorder, verify_archive

DATA = Path(__file__).parent / "data" / "recordings"


def test_archive_detects_modified_response_and_missing_inventory(tmp_path):
    folder = tmp_path / "recording"
    recorder = Recorder("http://test.invalid", folder)
    recorder.write("outputs/example_output.json", b'{"routes": [1, 2]}')
    recorder.finish("passed")
    assert verify_archive(folder)["status"] == "passed"
    (folder / "outputs/example_output.json").write_bytes(b'{"routes": [1]}')
    with pytest.raises(ValueError, match="integrity"):
        verify_archive(folder)
    (folder / "outputs/example_output.json").unlink()
    with pytest.raises(ValueError, match="inventory"):
        verify_archive(folder)


def test_archive_preserves_nested_source_manifest_and_refuses_overwrite(tmp_path):
    recorder = Recorder("http://test.invalid", tmp_path / "recording")
    recorder.write("source_inputs/v2_benchmarks/manifest.json", b'{"seed": 20261016}')
    with pytest.raises(FileExistsError):
        recorder.write("source_inputs/v2_benchmarks/manifest.json", b'{}')
    recorder.finish("passed")
    manifest = verify_archive(recorder.directory)
    assert len(manifest["files"]) == 1


@pytest.mark.parametrize("manifest_path", sorted(DATA.glob("*/manifest.json")), ids=lambda p: p.parent.name)
def test_complete_recorded_requests_and_responses(manifest_path):
    directory = manifest_path.parent
    manifest = verify_archive(directory)
    assert manifest["status"] == "passed"
    assert manifest["purpose"] == "functional_replay_capture"
    assert {case["name"] for case in manifest["cases"]} >= {
        "v1_single", "v1_batch", "v2_single_strict", "v2_single_prefer", "v2_single_free",
        "v2_batch", "v2_coordinates_to_local", "v2_coordinates_to_wgs84", "v2_validate"}
    assert {suite["suite"] for suite in manifest["benchmarks"]} == {"strict", "mixed", "free"}
    for suite in manifest["benchmarks"]:
        report = json.loads((directory / suite["report"]).read_bytes())
        assert report["accepted"] and report["completed"]
        assert len(report["runs"]) == suite["repeats"] == 1
        registration = report["scene_registration"]["payloads"]
        scene = json.loads((directory / "benchmarks" / registration["input"]["path"]).read_bytes())
        assert "coordinates" in scene and "airways" in scene
        for run in report["runs"]:
            payloads = run["payloads"]
            request = json.loads((directory / "benchmarks" / payloads["input"]["path"]).read_bytes())
            response = json.loads((directory / "benchmarks" / payloads["output"]["path"]).read_bytes())
            assert run["scale"] == len(request["tasks"]) == len(response["routes"]) == 1000
            assert response["success_count"] == 1000 and response["failed_count"] == 0
            assert not response["conflicts"]
            assert {t["id"] for t in request["tasks"]} == {r["task_id"] for r in response["routes"]}
            assert all(route["status"] == "planned" and route["validation"]["valid"]
                       and route["motion_segments"] for route in response["routes"])
            assert run["accepted"] and run["validation"]["validated_route_count"] == 1000
