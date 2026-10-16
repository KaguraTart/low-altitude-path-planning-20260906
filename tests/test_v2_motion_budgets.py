"""Analytic sampling rejects unreasonable allocations before creating a mesh."""
import pytest

from uav_planner.v2.motion import MotionError, build_motion


def limits(**overrides):
    return {"max_speed_mps": 12, "cruise_speed_mps": 6,
            "climb_speed_mps": 3, "descent_speed_mps": 3,
            "acceleration_mps2": 2, "deceleration_mps2": 2,
            "max_yaw_rate_deg_s": 60, "min_turn_radius_m": 0,
            **overrides}


def test_legal_slow_profile_cannot_allocate_half_a_billion_collision_samples():
    motion = build_motion([[0, 0, 80], [1000, 0, 80]],
                          limits(max_speed_mps=1e-6, cruise_speed_mps=1e-6))
    assert motion.times[-1] > 1e8
    with pytest.raises(MotionError, match="sample budget"):
        motion.collision_samples()


def test_excessive_arc_export_resolution_is_rejected_before_allocation():
    with pytest.raises(MotionError, match="Exported motion sample budget"):
        build_motion([[0, 0, 80], [100, 0, 80], [100, 100, 80]],
                     limits(min_turn_radius_m=10), tolerance=1e-10)


@pytest.mark.parametrize("error", [0, -1, float("nan"), float("inf")])
def test_collision_sampling_requires_a_positive_finite_error(error):
    motion = build_motion([[0, 0, 80], [10, 0, 80]], limits())
    with pytest.raises(MotionError, match="finite and positive"):
        motion.collision_samples(error)
