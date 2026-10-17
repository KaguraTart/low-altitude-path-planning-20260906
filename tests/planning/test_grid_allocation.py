"""Grid expansion uses available memory without fixed scene-size cutoffs."""
from math import prod

import pytest
from shapely.geometry import Point

from server.v2 import example_request
from uav_planner.v2.coordinates import CoordinateFrame
from uav_planner.v2.models import AircraftProfile, Task
from uav_planner.v2.scene import CompiledScene


def scene_and_task(*, extent=159, spacing=1):
    sample = example_request()
    scene = sample["scene"]
    scene["bounds"] = {"x_min": 0, "x_max": extent, "y_min": 0, "y_max": extent,
                       "z_min": 0, "z_max": extent}
    scene["resolution"] = [spacing] * 3
    scene["airways"] = []
    frame = CoordinateFrame(scene["coordinates"])
    task = Task.model_validate({**sample["task"], "route_mode": "free",
        "start_wkt": frame.to_wgs84(Point(extent / 4, extent / 2, extent / 2)),
        "goal_wkt": frame.to_wgs84(Point(extent * .75, extent / 2, extent / 2))})
    return CompiledScene(scene), task


def test_grid_expansion_can_exceed_four_million_nodes():
    compiled, task = scene_and_task()
    indices = compiled.blocked_indices(task, AircraftProfile())
    assert indices.shape == (0, 3)
    assert compiled.blocked_indices(task, AircraftProfile()) is indices


def test_large_shape_does_not_wrap_and_real_allocation_error_propagates(monkeypatch):
    compiled, task = scene_and_task(extent=100, spacing=1e-8)
    requested = []
    def unavailable_memory(shape, **kwargs):
        requested.append(shape)
        raise MemoryError("allocation failed")
    monkeypatch.setattr("uav_planner.v2.scene.np.zeros", unavailable_memory)
    with pytest.raises(MemoryError, match="allocation failed"):
        compiled.blocked_indices(task, AircraftProfile())
    assert len(requested) == 1
    assert all(isinstance(value, int) for value in requested[0])
    assert requested[0] == (10_000_000_001,) * 3
    assert prod(requested[0]) > 2 ** 63


def test_reusable_grid_contexts_are_not_evicted_after_sixteen_entries():
    compiled, task = scene_and_task(extent=1)
    first = compiled.blocked_indices(task, AircraftProfile(), 0)
    for second in range(1, 20):
        compiled.blocked_indices(task, AircraftProfile(), second)
    assert compiled.blocked_indices(task, AircraftProfile(), 0) is first
