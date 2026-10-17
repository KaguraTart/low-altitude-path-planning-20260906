"""Sampling keeps its accuracy beyond former limits and exposes allocation errors."""
import numpy as np
import pytest

from uav_planner.v2.motion import MotionError, build_motion


def limits(**overrides):
    return {"max_speed_mps": 12, "cruise_speed_mps": 6,
            "climb_speed_mps": 3, "descent_speed_mps": 3,
            "acceleration_mps2": 2, "deceleration_mps2": 2,
            "max_yaw_rate_deg_s": 60, "min_turn_radius_m": 0,
            **overrides}


def test_collision_cover_is_complete_above_former_sample_limit():
    motion = build_motion([[0, 0, 80], [150000, 0, 80]], limits())
    points, times = motion.collision_samples()
    assert len(points) > 100_000
    assert len(points) == len(times)
    np.testing.assert_allclose(points[[0, -1]], [[0, 0, 80], [150000, 0, 80]])
    assert times[0] == motion.times[0] and times[-1] == motion.times[-1]
    assert np.all(np.diff(times) > 0)


def test_arc_export_keeps_every_vertex_above_former_limit():
    motion = build_motion([[0, 0, 80], [100, 0, 80], [100, 100, 80]],
                          limits(min_turn_radius_m=10), tolerance=1e-10)
    assert len(motion.points) > 100_000
    np.testing.assert_allclose(motion.points[[0, -1]], [[0, 0, 80], [100, 100, 80]])
    assert motion.events[-1] == (len(motion.points) - 1, ["GOAL"])


def test_collision_allocation_error_is_not_converted_to_a_candidate_failure(monkeypatch):
    motion = build_motion([[0, 0, 80], [10, 0, 80]], limits())
    def allocation_failure(*args):
        raise MemoryError("allocation failed")
    monkeypatch.setattr(motion.primitives[0], "position", allocation_failure)
    with pytest.raises(MemoryError, match="allocation failed"):
        motion.collision_samples()


@pytest.mark.parametrize("error", [0, -1, float("nan"), float("inf")])
def test_collision_sampling_requires_a_positive_finite_error(error):
    motion = build_motion([[0, 0, 80], [10, 0, 80]], limits())
    with pytest.raises(MotionError, match="finite and positive"):
        motion.collision_samples(error)
