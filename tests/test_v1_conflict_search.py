"""Conflict repair uses the scene horizon instead of fixed retry counts."""
from types import SimpleNamespace

from src.models.aircraft import AircraftPerformance, FlightTask, Waypoint
from src.models.obstacles import ObstacleSet
from src.models.space import SpaceTimeConfig
from src.models.task import PlannedRoute, TrajectoryPoint
from src.planners.parallel_planner import ParallelPlanner


def setup_planner():
    config = SpaceTimeConfig(x_max=100, y_max=100, z_max=100, t_max=120, dt=10)
    return ParallelPlanner(config, ObstacleSet(), max_workers=1)


def task():
    return FlightTask(task_id='t', task_name='repair', aircraft_id='u', waypoints=[Waypoint('s', 10, 10, 50),
                                 Waypoint('e', 20, 10, 50)])


def route(index, time):
    return PlannedRoute(route_id=f'route-{index:03d}', task_id='t', aircraft_id='u', status='planned',
                        trajectory=[TrajectoryPoint(x=10, y=10, z=50, t=time)])


def test_departure_search_continues_past_six_retries(monkeypatch):
    planner = setup_planner()
    attempts = []
    def plan_one(*args, **kwargs):
        start = kwargs.get('start_time', 0)
        attempts.append(start)
        if start >= 80:
            return route(0, start)
        return PlannedRoute(route_id='route-000', task_id='t', aircraft_id='u', status='failed')
    monkeypatch.setattr(planner, '_plan_one', plan_one)
    table = SimpleNamespace(to_dense_mask=lambda **kwargs: None)
    result = planner._replan_avoiding(task(), AircraftPerformance('u'), 0, False, table)
    assert result.status == 'planned'
    assert attempts[-1] == 80


def test_departure_search_stops_at_scene_horizon(monkeypatch):
    planner = setup_planner()
    attempts = []
    def plan_one(*args, **kwargs):
        attempts.append(kwargs.get('start_time', 0))
        return PlannedRoute(route_id='route-000', task_id='t', aircraft_id='u', status='failed')
    monkeypatch.setattr(planner, '_plan_one', plan_one)
    result = planner._replan_avoiding(task(), AircraftPerformance('u'), 0, False,
                                    SimpleNamespace(to_dense_mask=lambda **kwargs: None))
    assert result.status == 'failed'
    assert attempts == list(range(0, 120, 10))


def test_repair_continues_past_three_rounds(monkeypatch):
    planner = setup_planner()
    first, second = route(0, 0), route(1, 0)
    def conflicts(routes, **kwargs):
        return [{'route_a': 'route-000', 'route_b': 'route-001'}] if routes[1].trajectory[0].t < 4 else []
    monkeypatch.setattr(planner, 'detect_conflicts_occupancy', conflicts)
    monkeypatch.setattr(planner, '_replan_avoiding',
                        lambda **kwargs: route(1, kwargs['old_route'].trajectory[0].t + 1))
    routes, remaining, count = planner.avoid_conflicts([first, second], [task(), task()], {'u': AircraftPerformance('u')})
    assert count == 4
    assert remaining == []
    assert routes[1].trajectory[0].t == 4


def test_repair_stops_when_schedule_repeats(monkeypatch):
    planner = setup_planner()
    monkeypatch.setattr(planner, 'detect_conflicts_occupancy',
                        lambda *args, **kwargs: [{'route_a': 'route-000', 'route_b': 'route-001'}])
    monkeypatch.setattr(planner, '_replan_avoiding',
                        lambda **kwargs: route(1, 1 - kwargs['old_route'].trajectory[0].t))
    _, remaining, count = planner.avoid_conflicts([route(0, 0), route(1, 0)], [task(), task()], {'u': AircraftPerformance('u')})
    assert remaining
    assert count == 2
