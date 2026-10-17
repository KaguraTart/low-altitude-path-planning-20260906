"""Scene registration has no count, geometry, grid, or estimated-memory quota."""
from copy import deepcopy
from types import SimpleNamespace

import pytest
from shapely.geometry import LineString

from server.v2 import example_request
from uav_planner.v2.budgets import SCENE_LIMITS
from uav_planner.v2.coordinates import CoordinateFrame
from uav_planner.v2.engine import SceneRegistry


@pytest.fixture
def lightweight_compile(monkeypatch):
    """Exercise registration without compiling thousands of duplicate geometries."""
    captured = []

    def compile_scene(model):
        captured.append(model)
        return SimpleNamespace(frame=SimpleNamespace(context={}), warnings=[])

    monkeypatch.setattr("uav_planner.v2.scene.CompiledScene", compile_scene)
    monkeypatch.setattr("uav_planner.v2.safety.temporal_breakpoints", lambda *args: [])
    return captured


def test_scene_limit_metadata_keeps_existing_keys_with_no_limit():
    assert set(SCENE_LIMITS) == {
        "airways", "obstacles", "control_zones", "connectors", "rules", "profiles",
        "datasets", "geometry_vertices", "field_cells", "grid_entries", "grid_nodes",
        "estimated_scene_bytes",
    }
    assert all(limit is None for limit in SCENE_LIMITS.values())


def test_geometry_above_old_vertex_limit_reaches_compilation(lightweight_compile):
    scene = example_request()["scene"]
    frame = CoordinateFrame(scene["coordinates"])
    scene["airways"][0]["centerLine"] = frame.to_wgs84(
        LineString([(100 + i * .022, 100, 80) for i in range(50001)])
    )
    assert SceneRegistry().register(scene)["status"] == "ready"
    assert len(lightweight_compile) == 1


def test_scene_object_counts_above_old_limits_reach_compilation(lightweight_compile):
    scene = example_request()["scene"]
    point = example_request()["task"]["start_wkt"]
    volume = {"geometry_wkt": point, "min_altitude": 0, "max_altitude": 100,
              "alt_reference": "MSL"}
    templates = {
        "airways": (scene["airways"][0], 201),
        "obstacles": (volume, 1001),
        "connectors": (volume, 1001),
        "control_zones": ({"geom": point, "minAlt": 0, "maxAlt": 100,
                           "zoneType": "POINT", "altReference": "MSL"}, 21),
        "rules": ({"rule_type": "TIME", "content": {}}, 1001),
        "profiles": ({}, 33),
        "datasets": ({"kind": "risk", "version": "v1"}, 101),
    }
    for field, (template, count) in templates.items():
        scene[field] = [{**template, "id": f"{field}-{i}"} for i in range(count)]
    assert SceneRegistry().register(scene)["status"] == "ready"
    assert len(lightweight_compile[0].rules) == 1001


def test_large_sparse_grid_does_not_require_dense_allocation(lightweight_compile):
    scene = example_request()["scene"]
    scene["grid"] = {"origin": [0, 0, 0], "resolution": [1, 1, 1],
                     "shape": [10**9, 10**9, 10**9], "occupied": [[0, 0, 0]]}
    assert SceneRegistry().register(scene)["status"] == "ready"
    assert lightweight_compile[0].grid.shape == (10**9,) * 3


def test_registry_retains_more_than_eight_scenes_without_size_estimation(monkeypatch):
    def unexpected_estimate(scene):
        pytest.fail("Registration must not estimate and reject scene memory")

    monkeypatch.setattr("uav_planner.v2.budgets.validate_scene_budget", unexpected_estimate)
    registry = SceneRegistry()
    scene = example_request()["scene"]
    ids = []
    for horizon in range(90000, 90010):
        value = deepcopy(scene)
        value["horizon_s"] = horizon
        ids.append(registry.register(value)["scene_id"])
    assert registry.max_scenes is None
    assert registry.max_bytes is None
    assert len(set(ids)) == 10
    assert all(registry.get(scene_id) for scene_id in ids)
    assert registry.register(value)["scene_id"] == ids[-1]
