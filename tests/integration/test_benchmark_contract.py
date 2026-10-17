"""Acceptance fixtures are immutable, independently feasible, and contentious."""
from __future__ import annotations

import importlib.util
import json
import math
import re
from pathlib import Path
from typing import Any

import numpy as np
import pytest
from shapely import from_wkt, get_coordinates

from uav_planner.v2 import native
from uav_planner.v2.models import RULE_TYPES, Scene
from uav_planner.v2.scene import CompiledScene

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("benchmark_v2", ROOT / "tools" / "benchmark_v2.py")
assert SPEC and SPEC.loader
benchmark = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(benchmark)

WKT_PREFIX = re.compile(
    r"^\s*(?:POINT|LINESTRING|POLYGON|MULTIPOINT|MULTILINESTRING|MULTIPOLYGON|GEOMETRYCOLLECTION)"
    r"(?:\s+(?:ZM|Z|M))?\s*(?:\(|EMPTY\b)",
    re.IGNORECASE,
)
FLOAT_ABSOLUTE_TOLERANCE = 1e-9
WKT_ABSOLUTE_TOLERANCE = 2e-11


def _geometry_structure(geometry: Any) -> tuple:
    """Retain component/ring order as well as coordinate dimension and type."""
    signature = (geometry.geom_type, geometry.has_z, getattr(geometry, "has_m", False))
    if hasattr(geometry, "geoms"):
        return (*signature, tuple(_geometry_structure(part) for part in geometry.geoms))
    if geometry.geom_type == "Polygon":
        return (*signature, len(geometry.exterior.coords),
                tuple(len(ring.coords) for ring in geometry.interiors))
    return (*signature, len(geometry.coords))


def assert_portable_generation_equal(expected: Any, actual: Any, path: str = "$") -> None:
    """Allow only tiny regeneration roundoff; frozen bytes remain authoritative."""
    __tracebackhide__ = True
    # NumPy float64 is emitted as a JSON floating-point number by the generator.
    same_json_float_type = isinstance(expected, float) and isinstance(actual, float)
    if type(expected) is not type(actual) and not same_json_float_type:
        raise AssertionError(f"{path}: type differs ({type(expected).__name__} vs {type(actual).__name__})")
    if isinstance(expected, dict):
        if expected.keys() != actual.keys():
            raise AssertionError(f"{path}: object keys differ")
        for key in expected:
            assert_portable_generation_equal(expected[key], actual[key], f"{path}.{key}")
    elif isinstance(expected, list):
        if len(expected) != len(actual):
            raise AssertionError(f"{path}: list lengths differ ({len(expected)} vs {len(actual)})")
        for index, (left, right) in enumerate(zip(expected, actual)):
            assert_portable_generation_equal(left, right, f"{path}[{index}]")
    elif isinstance(expected, float):
        if not math.isfinite(expected) or not math.isfinite(actual) or abs(expected - actual) > FLOAT_ABSOLUTE_TOLERANCE:
            raise AssertionError(f"{path}: numeric difference exceeds {FLOAT_ABSOLUTE_TOLERANCE:g}")
    elif isinstance(expected, str) and WKT_PREFIX.match(expected) and WKT_PREFIX.match(actual):
        try:
            left, right = from_wkt(expected), from_wkt(actual)
        except Exception as exc:
            raise AssertionError(f"{path}: invalid WKT ({type(exc).__name__})") from None
        if _geometry_structure(left) != _geometry_structure(right):
            raise AssertionError(f"{path}: WKT type, dimension, or ordered component/ring structure differs")
        left_coords = get_coordinates(left, include_z=left.has_z)
        right_coords = get_coordinates(right, include_z=right.has_z)
        if left_coords.shape != right_coords.shape:
            raise AssertionError(f"{path}: WKT coordinate array shapes differ")
        differences = np.abs(left_coords - right_coords)
        if not np.isfinite(differences).all() or np.any(differences > WKT_ABSOLUTE_TOLERANCE):
            index = tuple(int(i) for i in np.argwhere(~np.isfinite(differences) | (differences > WKT_ABSOLUTE_TOLERANCE))[0])
            raise AssertionError(f"{path}: WKT coordinate{index} difference exceeds {WKT_ABSOLUTE_TOLERANCE:g}")
    elif expected != actual:
        raise AssertionError(f"{path}: scalar value differs")


@pytest.fixture(scope="module", params=benchmark.SUITES)
def workload(request):
    return benchmark.load_frozen(request.param)[0]


def test_frozen_generation_is_reproducible_and_schema_valid(workload):
    # workload was loaded through the mandatory exact-byte SHA-256 guard above.
    # PROJ/GEOS builds may round the final WKT decimal differently on CI.
    assert_portable_generation_equal(workload, benchmark.generate_suite(workload["suite"]))
    model = Scene.model_validate(workload["scene"])
    assert len(workload["tasks"]) == len(workload["reference_trajectories"]) == 1000
    assert len({task["id"] for task in workload["tasks"]}) == 1000
    assert model.bounds.x_max - model.bounds.x_min == 5000
    assert model.bounds.y_max - model.bounds.y_min == 5000
    assert (model.bounds.z_min, model.bounds.z_max) == (0, 300)
    assert model.resolution == (25, 25, 10)
    assert model.horizon_s == 3600
    assert len(model.profiles) == 4
    assert len(model.obstacles) <= 1000 and len(model.control_zones) <= 20
    assert len(model.airways) <= 200
    assert all(len(task["waypoints"]) <= 2 for task in workload["tasks"])
    assert workload["construction"]["reference_uses_planner"] is False
    assert workload["construction"]["ground_wait_occupies_airspace"] is False


def test_portable_comparison_accepts_only_tiny_projection_and_numeric_roundoff():
    expected = {"scene": {"wkt": "LINESTRING Z (104.78 29.34 100, 104.79 29.35 101)",
                          "profile": {"speed": 4.0}, "seed": 20261016,
                          "enabled": True, "modes": ["strict", "free"]}}
    actual = {"scene": {"wkt": "LINESTRING Z (104.780000000001 29.34 100, 104.79 29.349999999999 101)",
                        "profile": {"speed": 4.0000000005}, "seed": 20261016,
                        "enabled": True, "modes": ["strict", "free"]}}
    assert_portable_generation_equal(expected, actual)


@pytest.mark.parametrize(("expected", "actual", "location"), [
    ({"scene": {"wkt": "POINT Z (104.78 29.34 100)"}},
     {"scene": {"wkt": "POINT Z (104.78000001 29.34 100)"}}, r"\$\.scene\.wkt"),
    ({"scene": {"wkt": "POINT Z (104.78 29.34 100)"}},
     {"scene": {"wkt": "POINT Z (104.78 29.34 100.00000001)"}}, r"\$\.scene\.wkt"),
    ({"scene": {"wkt": "LINESTRING Z (104.78 29.34 100, 104.79 29.35 101)"}},
     {"scene": {"wkt": "LINESTRING Z (104.79 29.35 101, 104.78 29.34 100)"}}, r"\$\.scene\.wkt"),
    ({"scene": {"wkt": "POINT Z (104.78 29.34 100)"}},
     {"scene": {"wkt": "POINT (104.78 29.34)"}}, r"\$\.scene\.wkt"),
    ({"scene": {"wkt": "POINT Z (104.78 29.34 100)"}},
     {"scene": {"wkt": "MULTIPOINT Z ((104.78 29.34 100))"}}, r"\$\.scene\.wkt"),
    ({"scene": {"resolution": [25.0, 25.0, 10.0]}},
     {"scene": {"resolution": [25.00001, 25.0, 10.0]}}, r"\$\.scene\.resolution\[0\]"),
    ({"scene": {"horizon_s": 3600}}, {"scene": {"horizon_s": 3599}}, r"\$\.scene\.horizon_s"),
    ({"scene": {"horizon_s": 3600}}, {"scene": {"horizon_s": 3600.0}}, r"\$\.scene\.horizon_s"),
    ({"scene": {"enabled": True}}, {"scene": {"enabled": 1}}, r"\$\.scene\.enabled"),
    ({"scene": {"mode": "strict"}}, {"scene": {"mode": "prefer"}}, r"\$\.scene\.mode"),
    ({"scene": {"modes": ["strict", "free"]}},
     {"scene": {"modes": ["free", "strict"]}}, r"\$\.scene\.modes\[0\]"),
    ({"scene": {"mode": "strict"}}, {"scene": {}}, r"\$\.scene"),
])
def test_portable_comparison_rejects_meaningful_or_structural_changes(expected, actual, location):
    with pytest.raises(AssertionError, match=location) as caught:
        assert_portable_generation_equal(expected, actual)
    assert len(str(caught.value)) < 180


def test_frozen_hash_tampering_is_rejected(tmp_path):
    manifest = json.loads((benchmark.FROZEN_DIRECTORY / "manifest.json").read_bytes())
    (tmp_path / "manifest.json").write_bytes(benchmark.canonical_bytes(manifest))
    for filename in manifest["files"]:
        (tmp_path / filename).write_bytes((benchmark.FROZEN_DIRECTORY / filename).read_bytes())
    (tmp_path / "strict.json").write_bytes((tmp_path / "strict.json").read_bytes() + b" ")
    with pytest.raises(ValueError, match="hash mismatch"):
        benchmark.load_frozen("strict", tmp_path)


def test_freeze_refuses_changed_existing_files(tmp_path):
    (tmp_path / "strict.json").write_text("changed", encoding="utf-8")
    with pytest.raises(ValueError, match="refusing overwrite"):
        benchmark.freeze(tmp_path)
    assert (tmp_path / "strict.json").read_text(encoding="utf-8") == "changed"
    assert not (tmp_path / "mixed.json").exists()


@pytest.mark.skipif(not native.available(), reason="required C++ extension is not built")
def test_all_reference_pair_schedules_are_feasible_and_originals_conflict(workload):
    references = workload["reference_trajectories"]
    points = [np.asarray(reference["local_points"]) for reference in references]
    times = [np.asarray(reference["vertex_times_s"]) for reference in references]
    assert all(np.all(np.diff(time) > 0) and time[-1] < 3500 for time in times)
    check = native.continuous_conflicts(points, times, 12, 12, max_conflicts=100000)
    assert check["conflicts"] == []
    initial_times = [time - time[0] for time in times]
    initial = native.continuous_conflicts(points, initial_times, 12, 12, max_conflicts=100000)
    conflicted = {conflict[key] for conflict in initial["conflicts"] for key in ("first", "second")}
    assert len(conflicted) == 1000


@pytest.mark.skipif(not native.available(), reason="required C++ extension is not built")
def test_mixed_all_twenty_rules_have_executed_effects():
    workload, _ = benchmark.load_frozen("mixed")
    compiled = CompiledScene(workload["scene"])
    effects = benchmark.validate_rule_witnesses(workload, compiled)
    assert set(effects) == set(RULE_TYPES)
    assert effects["PRIORITY"]["quiet_cost"] < effects["PRIORITY"]["noisy_cost"]
    assert effects["RTL"]["required"]
    triggered = [task for task in workload["tasks"] if task["state"]["battery_percent"] == 10]
    assert len(triggered) == 10
    assert all(task["goal_wkt"] != task["state"]["home_wkt"] for task in triggered)


def test_analytic_reference_obeys_aircraft_acceleration_and_speed(workload):
    profiles = {profile["id"]: profile for profile in workload["scene"]["profiles"]}
    for task, reference in zip(workload["tasks"], workload["reference_trajectories"]):
        profile = profiles[task["profile_id"]]
        for primitive in reference["motion_primitives"]:
            duration = primitive["end_time_s"] - primitive["start_time_s"]
            assert duration > 0
            if primitive["kind"] == "yaw":
                continue
            peak = primitive["peak_speed_mps"]
            ta, tc, td = (primitive[name] for name in ("accel_time_s", "cruise_time_s", "decel_time_s"))
            assert peak <= profile["max_speed_mps"]
            assert peak / ta <= profile["acceleration_mps2"] + 1e-9
            assert peak / td <= profile["deceleration_mps2"] + 1e-9
            assert ta + tc + td == pytest.approx(duration)
            assert peak * (ta / 2 + tc + td / 2) == pytest.approx(primitive["length_m"])
            delta = np.asarray(primitive["end"]) - np.asarray(primitive["start"])
            vertical_speed = abs(delta[2]) / primitive["length_m"] * peak
            assert vertical_speed <= profile["climb_speed_mps"]
            if workload["suite"] == "mixed":
                assert peak * np.linalg.norm(delta[:2]) / primitive["length_m"] <= 4 + 1e-9
                assert vertical_speed <= 1 + 1e-9
