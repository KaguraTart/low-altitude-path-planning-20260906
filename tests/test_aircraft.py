"""AircraftPerformance / TaskRequirements / FlightTask 单元测试"""
import pytest

from src.models.aircraft import (
    AircraftPerformance,
    FlightTask,
    PerformanceSpec,
    PhysicalSpec,
    TaskRequirements,
    Waypoint,
)


class TestPerformanceValidation:
    def test_is_speed_valid_horizontal(self):
        perf = AircraftPerformance(aircraft_id="u")
        assert perf.is_speed_valid(10, 0, 0)     # 水平 10 < max 15
        assert not perf.is_speed_valid(20, 0, 0)  # 水平 20 > max 15

    def test_is_speed_valid_climb(self):
        perf = AircraftPerformance(aircraft_id="u")
        assert perf.is_speed_valid(0, 0, 4)
        assert not perf.is_speed_valid(0, 0, 6)   # > max_climb 5

    def test_is_speed_valid_descent(self):
        perf = AircraftPerformance(aircraft_id="u")
        assert perf.is_speed_valid(0, 0, -2)
        assert not perf.is_speed_valid(0, 0, -4)  # > max_descent 3

    def test_is_acceleration_valid(self):
        perf = AircraftPerformance(aircraft_id="u")
        assert perf.is_acceleration_valid(1, 1, 1)
        assert not perf.is_acceleration_valid(10, 0, 0)  # > max_acc 3

    def test_is_altitude_valid(self):
        perf = AircraftPerformance(aircraft_id="u")
        assert perf.is_altitude_valid(100)
        assert not perf.is_altitude_valid(5)    # < min 10
        assert not perf.is_altitude_valid(350)  # > max 300

    def test_min_turn_radius_override(self):
        perf = AircraftPerformance(
            aircraft_id="u", performance=PerformanceSpec(min_turn_radius_m=10.0)
        )
        assert perf.is_turn_radius_valid(15)
        assert not perf.is_turn_radius_valid(5)   # < min 10
        # 默认 0 = 不约束
        default = AircraftPerformance(aircraft_id="u")
        assert default.is_turn_radius_valid(1)


class TestEstimateEnergy:
    def test_flat_distance_positive(self):
        """纯水平巡航能耗 > 0，且随距离增长"""
        perf = AircraftPerformance(aircraft_id="u")
        e1 = perf.estimate_energy(100, 0)
        e2 = perf.estimate_energy(200, 0)
        assert e1 > 0
        assert e2 > e1

    def test_climb_costs_more_than_descent(self):
        """爬升能耗系数(3) > 下降(0.5)"""
        perf = AircraftPerformance(aircraft_id="u")
        climb = perf.estimate_energy(0, 50)
        descend = perf.estimate_energy(0, -50)
        assert climb > descend


class TestAircraftSummary:
    def test_summary_fields(self):
        perf = AircraftPerformance(
            aircraft_id="uav-x", model="M300",
            category="heavy_payload",
            physical=PhysicalSpec(weight_kg=6.3, payload_capacity_kg=2.7),
        )
        s = perf.summary()
        assert s["aircraft_id"] == "uav-x"
        assert s["model"] == "M300"
        assert s["weight_kg"] == 6.3
        assert s["payload_kg"] == 2.7


class TestTaskRequirements:
    def test_requirements_defaults(self):
        req = TaskRequirements()
        assert req.payload_kg == 0.0
        assert req.priority_level == 1

    def test_aircraft_compatibility_ok(self):
        task = FlightTask(
            task_id="t1", task_name="t1", aircraft_id="u",
            waypoints=[
                Waypoint(wp_id="s", x=0, y=0, z=50, wp_type="start"),
                Waypoint(wp_id="e", x=100, y=100, z=50, wp_type="end"),
            ],
            requirements=TaskRequirements(payload_kg=1.0),
        )
        perf = AircraftPerformance(
            aircraft_id="u",
            physical=PhysicalSpec(payload_capacity_kg=2.0),
        )
        ok, problems = task.check_aircraft_compatibility(perf)
        assert ok
        assert problems == []

    def test_aircraft_compatibility_payload_too_heavy(self):
        task = FlightTask(
            task_id="t1", task_name="t1", aircraft_id="u",
            waypoints=[
                Waypoint(wp_id="s", x=0, y=0, z=50, wp_type="start"),
                Waypoint(wp_id="e", x=100, y=100, z=50, wp_type="end"),
            ],
            requirements=TaskRequirements(payload_kg=5.0),
        )
        perf = AircraftPerformance(
            aircraft_id="u",
            physical=PhysicalSpec(payload_capacity_kg=2.0),
        )
        ok, problems = task.check_aircraft_compatibility(perf)
        assert not ok
        assert any("载荷不足" in p for p in problems)

    def test_aircraft_compatibility_speed(self):
        task = FlightTask(
            task_id="t1", task_name="t1", aircraft_id="u",
            waypoints=[
                Waypoint(wp_id="s", x=0, y=0, z=50, wp_type="start"),
                Waypoint(wp_id="e", x=100, y=100, z=50, wp_type="end"),
            ],
            requirements=TaskRequirements(required_max_speed_mps=20.0),
        )
        perf = AircraftPerformance(aircraft_id="u", max_speed=15.0)
        ok, problems = task.check_aircraft_compatibility(perf)
        assert not ok
        assert any("最大速度不足" in p for p in problems)


class TestFlightTaskValidate:
    def test_valid_task(self):
        task = FlightTask(
            task_id="t", task_name="t", aircraft_id="u",
            waypoints=[
                Waypoint(wp_id="s", x=0, y=0, z=50, wp_type="start"),
                Waypoint(wp_id="e", x=100, y=100, z=50, wp_type="end"),
            ],
            priority=5,
        )
        ok, msg = task.validate()
        assert ok

    def test_insufficient_waypoints(self):
        task = FlightTask(
            task_id="t", task_name="t", aircraft_id="u",
            waypoints=[Waypoint(wp_id="only", x=0, y=0, z=50, wp_type="start")],
        )
        ok, msg = task.validate()
        assert not ok
        assert "至少需要起点和终点" in msg

    def test_priority_out_of_range(self):
        task = FlightTask(
            task_id="t", task_name="t", aircraft_id="u",
            waypoints=[
                Waypoint(wp_id="s", x=0, y=0, z=50, wp_type="start"),
                Waypoint(wp_id="e", x=100, y=100, z=50, wp_type="end"),
            ],
            priority=11,
        )
        ok, _ = task.validate()
        assert not ok

    def test_weight_sum_not_one(self):
        task = FlightTask(
            task_id="t", task_name="t", aircraft_id="u",
            waypoints=[
                Waypoint(wp_id="s", x=0, y=0, z=50, wp_type="start"),
                Waypoint(wp_id="e", x=100, y=100, z=50, wp_type="end"),
            ],
            objective_weights={"distance": 0.5, "time": 0.5, "risk": 0.1, "energy": 0.1},
        )
        ok, msg = task.validate()
        assert not ok
        assert "权重之和" in msg