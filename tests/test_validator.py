"""PathValidator 路径合法性验证模块单元测试"""
import pytest

from src.models.aircraft import AircraftPerformance
from src.models.obstacles import ObstacleSet
from src.models.task import PlannedRoute, TrajectoryPoint
from src.validator.path_validator import PathValidator, Severity


def _route(pts, route_id="r1", aircraft_id="uav-1", status="planned"):
    """由 (x,y,z,t) 元组列表构造 PlannedRoute"""
    return PlannedRoute(
        route_id=route_id,
        task_id="t1",
        aircraft_id=aircraft_id,
        status=status,
        trajectory=[TrajectoryPoint(x=x, y=y, z=z, t=t) for x, y, z, t in pts],
    )


class TestValidateSingleRoute:
    def test_valid_route_no_errors(self):
        """合法航线（直线匀速、高度合规）应无错误"""
        r = _route(
            [(0, 0, 50, 0), (100, 0, 50, 10), (200, 0, 50, 20)],
            route_id="r-ok",
        )
        v = PathValidator().validate_route(r, performance=AircraftPerformance(aircraft_id="u"))
        assert v.is_valid
        assert not v.has_errors()
        assert v.summary["trajectory_points"] == 3

    def test_empty_trajectory_error(self):
        """轨迹为空应判定不合法"""
        r = _route([], route_id="r-empty")
        v = PathValidator().validate_route(r)
        assert not v.is_valid
        assert any(i.code == "EMPTY_TRAJECTORY" for i in v.issues)

    def test_static_obstacle_collision(self, building):
        """轨迹点落在静态障碍内应报错"""
        obs = ObstacleSet()
        obs.add_static(building)
        r = _route([(2500, 2500, 100, 0), (3000, 2500, 100, 10)], route_id="r-collide")
        v = PathValidator(obstacle_set=obs).validate_route(r)
        assert not v.is_valid
        assert any(i.code == "STATIC_OBSTACLE_COLLISION" for i in v.issues)

    def test_static_obstacle_proximity_warning(self, building):
        """接近障碍但未撞上应是 WARNING 而非 ERROR"""
        obs = ObstacleSet()
        obs.add_static(building)
        # 距障碍表面 3m（< 默认 safety_margin 5m）
        r = _route([(2583, 2500, 100, 0), (3000, 2500, 100, 10)], route_id="r-near")
        v = PathValidator(obstacle_set=obs).validate_route(r)
        assert not v.has_errors()
        assert any(i.code == "STATIC_OBSTACLE_PROXIMITY" and i.severity == Severity.WARNING
                   for i in v.issues)

    def test_altitude_too_low(self):
        """低于最低飞行高度应报错"""
        r = _route([(0, 0, 2, 0), (100, 0, 2, 10)], route_id="r-low")
        v = PathValidator().validate_route(r, performance=AircraftPerformance(aircraft_id="u"))
        assert not v.is_valid
        assert any(i.code == "ALTITUDE_TOO_LOW" for i in v.issues)


class TestValidateMultiRoutes:
    def test_no_conflict_when_apart(self):
        """时间/空间都分离的航线应无冲突"""
        a = _route([(0, 0, 50, 0), (100, 0, 50, 10)], route_id="ra", aircraft_id="uav-a")
        b = _route([(0, 500, 50, 0), (100, 500, 50, 10)], route_id="rb", aircraft_id="uav-b")
        v = PathValidator().validate_multi_routes([a, b])
        assert v.is_valid
        assert v.summary["n_conflicts"] == 0

    def test_conflict_when_same_place_time(self):
        """两机同时同地应检测出冲突"""
        a = _route([(100, 100, 50, 10)], route_id="ra", aircraft_id="uav-a")
        b = _route([(100, 100, 50, 10)], route_id="rb", aircraft_id="uav-b")
        v = PathValidator().validate_multi_routes([a, b])
        assert not v.is_valid
        assert any(i.code == "MULTI_UAV_COLLISION" for i in v.issues)

    def test_same_aircraft_not_conflict(self):
        """同一架飞机的多段航线不视为冲突"""
        a = _route([(100, 100, 50, 10)], route_id="ra", aircraft_id="uav-a")
        b = _route([(100, 100, 50, 10)], route_id="rb", aircraft_id="uav-a")
        v = PathValidator().validate_multi_routes([a, b])
        assert v.is_valid

    def test_failed_routes_skipped(self):
        """status != planned 的航线应被跳过"""
        a = _route([(100, 100, 50, 10)], route_id="ra", aircraft_id="uav-a")
        b = _route([(100, 100, 50, 10)], route_id="rb", aircraft_id="uav-b", status="failed")
        v = PathValidator().validate_multi_routes([a, b])
        assert v.is_valid
