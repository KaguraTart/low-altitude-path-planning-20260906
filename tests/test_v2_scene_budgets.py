"""Resource guards reject excessive scenes before compiling GEOS/index caches."""
from copy import deepcopy

import pytest
from shapely.geometry import LineString

from server.v2 import example_request
from uav_planner import PlanningError
from uav_planner.v2.coordinates import CoordinateFrame
from uav_planner.v2.engine import SceneRegistry


def test_geometry_vertex_limit_is_checked_before_compilation(monkeypatch):
    scene=example_request()["scene"];frame=CoordinateFrame(scene["coordinates"])
    scene["airways"][0]["centerLine"]=frame.to_wgs84(LineString([(100+i*.022,100,80) for i in range(50001)]))
    def forbidden_compile(*args):
        pytest.fail("Oversized scene reached geometric compilation")
    monkeypatch.setattr("uav_planner.v2.scene.CompiledScene",forbidden_compile)
    with pytest.raises(PlanningError,match="resource budget"):
        SceneRegistry().register(scene)


def test_scene_registry_evicts_least_recently_used_context():
    registry=SceneRegistry(max_scenes=2)
    scene=example_request()["scene"]
    first=registry.register(scene)["scene_id"]
    second_scene=deepcopy(scene);second_scene["horizon_s"]=3500
    second=registry.register(second_scene)["scene_id"]
    registry.get(first)
    third_scene=deepcopy(scene);third_scene["horizon_s"]=3400
    registry.register(third_scene)
    assert registry.get(first)
    with pytest.raises(PlanningError,match="Scene not found"):
        registry.get(second)


def test_single_scene_cannot_exceed_registry_byte_budget():
    with pytest.raises(PlanningError,match="memory budget"):
        SceneRegistry(max_bytes=100).register(example_request()["scene"])
