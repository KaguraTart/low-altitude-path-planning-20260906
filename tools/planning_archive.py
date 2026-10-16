"""Keep reproducible CLI inputs and complete planning results beside reports."""
from __future__ import annotations

import hashlib
import json
import os
import subprocess
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

from src.io.output_writer import write_planning_result
from src.models.task import PlanningResult


def git_revision() -> str:
    """Use the checked-out CI revision, falling back to the local checkout."""
    for name in ("GITHUB_SHA", "UAV_GIT_SHA"):
        revision = os.environ.get(name, "")
        if revision and revision != "unknown":
            return revision
    result = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=Path(__file__).resolve().parents[1],
        capture_output=True, text=True, check=False,
    )
    return result.stdout.strip() if result.returncode == 0 else "unknown"


def write_json(path: Path, value: Any) -> dict[str, Any]:
    path.parent.mkdir(parents=True, exist_ok=True)
    data = (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n").encode("utf-8")
    path.write_bytes(data)
    return file_record(path)


def file_record(path: Path) -> dict[str, Any]:
    data = path.read_bytes()
    return {"path": path.name, "bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()}


def snapshot_input_files(
    input_directory: str | Path, output_directory: str | Path,
    arguments: dict[str, Any], *, filenames: Iterable[str] = (
        "config.json", "static_obstacles.json", "dynamic_obstacles.json",
        "aircraft_config.json", "tasks.json",
    ),
) -> dict[str, Any]:
    """Copy exact source bytes before planning, including parameters and revision."""
    destination = Path(output_directory)
    destination.mkdir(parents=True, exist_ok=True)
    files = []
    missing_files = []
    for name in filenames:
        source = Path(input_directory) / name
        if not source.exists():
            # Fused obstacle loading permits absent optional sources; preserve
            # that fact instead of adding a new input requirement or fake data.
            missing_files.append(name)
            continue
        target = destination / name
        target.write_bytes(source.read_bytes())
        files.append(file_record(target))
    manifest = {
        "format_version": 1, "captured_at": datetime.now(timezone.utc).isoformat(),
        "git_sha": git_revision(), "arguments": arguments, "files": files,
        "missing_files": missing_files,
    }
    write_json(destination / "manifest.json", manifest)
    return manifest


def archive_planning_result(
    result: PlanningResult, directory: str | Path, stem: str,
) -> dict[str, Any]:
    """Retain the public serializer and unrounded model fields separately.

    The existing serializer includes every route and its explanation, but rounds
    coordinates and omits velocity components and conflict details. The model
    snapshot preserves those fields without changing the public output format.
    """
    destination = Path(directory)
    destination.mkdir(parents=True, exist_ok=True)
    output = Path(write_planning_result(result, str(destination), f"{stem}_output.json"))
    state = destination / f"{stem}_output_state.json"
    write_json(state, asdict(result))
    return {
        "output": file_record(output), "output_state": file_record(state),
        "output_format": "PlanningResult.summary", "output_state_format": "dataclasses.asdict",
    }
