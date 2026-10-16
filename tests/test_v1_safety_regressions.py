"""Adversarial v1 safety regressions, independent of the optional native engine."""
import pytest

from src.models.aircraft import AircraftPerformance, FlightTask, PerformanceSpec, Waypoint
from src.models.obstacles import DynamicObstacle, ObstacleSet, StaticObstacle
from src.models.space import SpaceTimeConfig, SpaceTimeGrid
from src.models.task import PlannedRoute, TrajectoryPoint
from src.planners.occupancy import SpatioTemporalOccupancyTable
from src.planners.parallel_planner import ParallelPlanner
from src.planners.route_optimizer import RouteOptimizer
from src.planners.spacetime_astar import SpaceTimeAStar
from src.validator.path_validator import PathValidator


def cfg(**kwargs):
    return SpaceTimeConfig(x_max=200, y_max=200, z_max=100, dx=20, dy=20, dz=20,
                           t_max=300, dt=10, **kwargs)


def route(points, rid="r", aid="u"):
    result = PlannedRoute(route_id=rid, task_id="t", aircraft_id=aid, status="planned",
                          trajectory=[TrajectoryPoint(x=x, y=y, z=z, t=t) for x,y,z,t in points])
    result.compute_metrics()
    return result


def test_astar_preserves_exact_endpoints_and_departure():
    planner = SpaceTimeAStar(cfg(), max_speed=15, min_altitude=10, max_altitude=60)
    result = planner.plan((21, 31, 55), (141, 31, 55), start_time=1.25)
    assert result.status == "planned"
    assert [(p.x,p.y,p.z) for p in (result.trajectory[0],result.trajectory[-1])] == [(21,31,55),(141,31,55)]
    assert result.trajectory[0].t == 1.25
    assert all(10 <= p.z <= 60 for p in result.trajectory)


def test_astar_does_not_use_out_of_envelope_cell_center():
    config = SpaceTimeConfig(x_max=200,y_max=200,z_max=100,dx=20,dy=20,dz=50,t_max=300,dt=10)
    result = SpaceTimeAStar(config,min_altitude=50,max_altitude=60).plan((21,31,55),(141,31,55))
    assert result.status == "failed" or all(50 <= p.z <= 60 for p in result.trajectory)


def test_astar_rejects_outside_coordinates_instead_of_clamping():
    result = SpaceTimeAStar(cfg()).plan((-1, 31, 55), (141,31,55))
    assert result.status == "failed"


def test_astar_checks_connector_speed_and_collision():
    config = cfg()
    grid = SpaceTimeGrid(config)
    # The connector must spend real time reaching the center, without jumping.
    result = SpaceTimeAStar(config,grid=grid,max_speed=1).plan((1,1,41),(11,11,41),start_time=1)
    if result.status == "planned":
        for a,b in zip(result.trajectory,result.trajectory[1:]):
            assert ((b.x-a.x)**2+(b.y-a.y)**2+(b.z-a.z)**2)**.5 <= (b.t-a.t) + 1e-6


@pytest.mark.parametrize("with_config", [False,True])
def test_continuous_edge_swap_is_a_conflict(with_config):
    a = route([(10,30,50,5),(30,30,50,15)],"a","a")
    b = route([(30,30,50,5),(10,30,50,15)],"b","b")
    table = SpatioTemporalOccupancyTable(cfg())
    table.mark_route(a)
    conflicts = table.find_conflicts(b,min_separation=1,time_window=0)
    assert conflicts and conflicts[0]["distance"] == pytest.approx(0)
    assert conflicts[0]["time"] == pytest.approx(10)
    validation = PathValidator(config=cfg() if with_config else None,min_separation=1,time_window=0).validate_multi_routes([a,b])
    assert not validation.is_valid


def test_continuous_window_keeps_separate_flights_separate():
    a=route([(10,30,50,5),(30,30,50,15)],"a","a")
    b=route([(30,30,50,35),(10,30,50,45)],"b","b")
    table=SpatioTemporalOccupancyTable(cfg()); table.mark_route(a)
    assert table.find_conflicts(b,min_separation=1,time_window=5) == []


def test_off_center_positions_are_compared_exactly():
    a=route([(19,30,50,5)],"a","a"); b=route([(21,30,50,5)],"b","b")
    table=SpatioTemporalOccupancyTable(cfg()); table.mark_route(a)
    assert table.find_conflicts(b,min_separation=3,time_window=0)


def test_dynamic_obstacle_sweeps_intermediate_cells_and_times():
    config=cfg(); grid=SpaceTimeGrid(config)
    grid.set_dynamic_obstacle([(10,30,50,5),(150,30,50,75)],radius=0)
    assert not grid.is_free(*config.to_grid(70,30,50),config.time_to_step(35))
    assert grid.is_free(*config.to_grid(70,30,50),config.time_to_step(155))


def test_validator_checks_segment_static_obstacles():
    obstacles=ObstacleSet(); obstacles.add_static(StaticObstacle(obstacle_id="wall",name="wall",obstacle_type="building",x=80,y=30,z=40,width=1,depth=5,height=20))
    result=PathValidator(obstacle_set=obstacles).validate_route(route([(10,30,50,5),(150,30,50,75)]))
    assert not result.is_valid


def test_validator_checks_continuous_dynamic_collision():
    obstacles=ObstacleSet(); obstacles.add_dynamic(DynamicObstacle(obstacle_id="moving",name="moving",obstacle_type="uav",trajectory=[(30,30,50,5),(10,30,50,15)],safety_radius=1))
    result=PathValidator(obstacle_set=obstacles).validate_route(route([(10,30,50,5),(30,30,50,15)]))
    assert not result.is_valid


def test_full_minimum_separation_is_hard_requirement():
    a=route([(10,30,50,5)],"a","a"); b=route([(17.5,30,50,5)],"b","b")
    assert not PathValidator(min_separation=10,time_window=0).validate_multi_routes([a,b]).is_valid


def test_minimum_turn_radius_cannot_be_certified_by_chord_circle():
    perf=AircraftPerformance("u",performance=PerformanceSpec(min_turn_radius_m=1))
    result=PathValidator().validate_route(route([(10,30,50,5),(30,30,50,25),(30,50,50,45)]),perf)
    assert not result.is_valid
    assert any(i.code=="TURN_RADIUS_EXCEEDED" for i in result.issues)


def test_nonmonotonic_time_is_error_without_performance():
    assert not PathValidator().validate_route(route([(10,30,50,5),(20,30,50,5)])).is_valid


def test_optimizer_does_not_retime_wait_or_patch_coordinates():
    opt=RouteOptimizer(cfg(),ObstacleSet(),AircraftPerformance("u"))
    original=route([(10,30,50,5),(30,30,50,15),(30,30,50,35),(50,30,50,45)]).trajectory
    candidate=opt._enforce_performance_constraints(opt._smooth_trajectory(original))
    assert [(p.x,p.y,p.z,p.t) for p in candidate] == [(p.x,p.y,p.z,p.t) for p in original]
    fast=route([(10,30,50,5),(150,30,50,6)]).trajectory
    assert [(p.x,p.y,p.z,p.t) for p in opt._enforce_performance_constraints(fast)] == [(p.x,p.y,p.z,p.t) for p in fast]


def test_optimizer_checks_raw_fallback_before_reporting_success(monkeypatch):
    opt=RouteOptimizer(cfg(),ObstacleSet(),AircraftPerformance("u"))
    unsafe=route([(10,30,50,5),(150,30,50,6)])
    monkeypatch.setattr(opt.astar,"plan",lambda **kwargs:unsafe)
    task=FlightTask("t","t","u",waypoints=[Waypoint("a",10,30,50),Waypoint("b",150,30,50)],departure_time=5)
    assert opt.optimize(task).status == "failed"


def test_optimizer_preserves_schedule_during_geometry_smoothing():
    opt = RouteOptimizer(cfg(), ObstacleSet(), AircraftPerformance("u"))
    original = route([(10,30,50,5),(30,30,50,25),(50,30,50,45),(70,30,50,65)]).trajectory
    smoothed = opt._smooth_trajectory(original)
    assert smoothed[0].t == 5 and smoothed[-1].t == 65
    for point in original:
        assert any((p.x,p.y,p.z,p.t) == (point.x,point.y,point.z,point.t) for p in smoothed)


def test_optimizer_uses_valid_raw_fallback_when_geometry_cuts_wall(monkeypatch):
    obstacles=ObstacleSet()
    obstacles.add_static(StaticObstacle(obstacle_id="wall",name="wall",obstacle_type="building",
                                        x=30,y=10,z=40,width=2,depth=2,height=20))
    opt=RouteOptimizer(cfg(),obstacles,AircraftPerformance("u"))
    original=route([(10,10,50,5),(10,50,50,25),(50,50,50,45),(50,10,50,65)])
    monkeypatch.setattr(opt.astar,"plan",lambda **kwargs:original)
    monkeypatch.setattr(opt,"_smooth_trajectory",lambda trajectory:[trajectory[0],trajectory[-1]])
    task=FlightTask("t","t","u",waypoints=[Waypoint("a",10,10,50),Waypoint("b",50,10,50)],departure_time=5)
    result=opt.optimize(task)
    assert result.status == "planned"
    assert "raw-fallback" in result.algorithm
    assert [(p.x,p.y,p.z,p.t) for p in result.trajectory] == [(p.x,p.y,p.z,p.t) for p in original.trajectory]


def test_swept_registration_unmark_and_exclusion_are_complete():
    config=cfg(); table=SpatioTemporalOccupancyTable(config)
    original=route([(10,30,50,5),(150,30,50,75)])
    table.mark_route(original)
    cell=(*config.to_grid(70,30,50),config.time_to_step(35))
    assert table.to_dense_mask(dilate=False)[cell]
    assert not table.to_dense_mask(exclude_route=original,dilate=False).any()
    table.unmark_route(original)
    assert not table.to_dense_mask(dilate=False).any()
    assert table.find_conflicts(original,time_window=0) == []


def test_deceleration_limit_is_enforced_separately():
    perf=AircraftPerformance("u",performance=PerformanceSpec(max_acceleration_mps2=10,max_deceleration_mps2=.1))
    result=PathValidator().validate_route(route([(10,30,50,5),(30,30,50,15),(31,30,50,25)]),perf)
    assert not result.is_valid
    assert any(i.code=="DECELERATION_EXCEEDED" for i in result.issues)


@pytest.mark.parametrize("with_config", [False,True])
def test_multi_validator_does_not_certify_malformed_planned_routes(with_config):
    bad=route([(float("nan"),30,50,5)],"bad","bad")
    good=route([(10,30,50,5)],"good","good")
    assert not PathValidator(config=cfg() if with_config else None).validate_multi_routes([bad,good]).is_valid


def test_zero_vertical_rate_does_not_allow_grid_altitude_change():
    planner=SpaceTimeAStar(cfg(),max_climb_rate=0,max_descent_rate=0,max_iterations=100)
    assert planner.plan((10,30,50),(10,30,70),start_time=5).status == "failed"


@pytest.mark.parametrize("avoid_conflicts", [True, False])
def test_exhausted_batch_conflicts_are_not_executable_success(avoid_conflicts):
    tasks = [
        FlightTask("cross-a", "cross-a", "a", departure_time=5,
                   waypoints=[Waypoint("start", 10, 30, 50), Waypoint("end", 150, 30, 50)]),
        FlightTask("cross-b", "cross-b", "b", departure_time=5,
                   waypoints=[Waypoint("start", 150, 30, 50), Waypoint("end", 10, 30, 50)]),
        FlightTask("safe", "safe", "c", departure_time=5,
                   waypoints=[Waypoint("start", 10, 130, 50), Waypoint("end", 150, 130, 50)]),
    ]
    planner = ParallelPlanner(cfg(), ObstacleSet(), max_workers=1)
    result = planner.plan_batch(
        tasks, {aid: AircraftPerformance(aid) for aid in ("a", "b", "c")},
        conflict_avoidance=avoid_conflicts, max_conflict_rounds=0, min_separation=1,
    )
    assert result.conflicts, "The raw unresolved conflict must remain available for diagnosis"
    by_task = {item.task_id: item for item in result.routes}
    assert by_task["safe"].status == "planned"
    if avoid_conflicts:
        assert result.success_count == 1 and result.failed_count == 2
        for task_id in ("cross-a", "cross-b"):
            assert by_task[task_id].status == "failed"
            assert "未解决" in by_task[task_id].message
            assert "冲突" in by_task[task_id].message
    else:
        assert result.success_count == 3 and result.failed_count == 0
        assert all(item.status == "planned" for item in result.routes)
    for task in tasks:
        trajectory = by_task[task.task_id].trajectory
        assert trajectory, "Preserve the raw trajectory even when execution is rejected"
        assert (trajectory[0].x, trajectory[0].y, trajectory[0].z, trajectory[0].t) == (
            task.start.x, task.start.y, task.start.z, task.departure_time)
        assert (trajectory[-1].x, trajectory[-1].y, trajectory[-1].z) == (
            task.end.x, task.end.y, task.end.z)
