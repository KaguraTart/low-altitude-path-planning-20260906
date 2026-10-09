"""StaticObstacle / DynamicObstacle / ObstacleSet 单元测试"""
import pytest

from src.models.obstacles import (
    DynamicObstacle,
    ObstacleSet,
    StaticObstacle,
)


class TestStaticObstacle:
    def test_contains_inside(self, building):
        assert building.contains(2500, 2500, 100) is True
        # 顶部边缘（z+height）
        assert building.contains(2500, 2500, 200) is True
        # 底部
        assert building.contains(2500, 2500, 0) is True

    def test_contains_outside_x(self, building):
        assert building.contains(2500 + 81, 2500, 100) is False  # 超出半宽

    def test_contains_outside_y(self, building):
        assert building.contains(2500, 2500 + 81, 100) is False

    def test_contains_above_height(self, building):
        """超过高度 z+height 即不受障碍影响"""
        assert building.contains(2500, 2500, 201) is False

    def test_cylinder_radius(self, building):
        assert building.to_cylinder_radius() == max(80, 80)


class TestDynamicObstacle:
    def _make(self):
        return DynamicObstacle(
            obstacle_id="dyn1", name="闯入者", obstacle_type="uav",
            trajectory=[(0, 0, 50, 0), (100, 0, 50, 10), (200, 0, 50, 20)],
            safety_radius=10,
        )

    def test_position_at_start(self):
        d = self._make()
        assert d.position_at(0) == (0, 0, 50)

    def test_position_at_end(self):
        d = self._make()
        assert d.position_at(20) == (200, 0, 50)

    def test_position_at_before_first_clamps(self):
        d = self._make()
        assert d.position_at(-5) == (0, 0, 50)

    def test_position_at_after_last_clamps(self):
        d = self._make()
        assert d.position_at(99) == (200, 0, 50)

    def test_position_at_interpolates(self):
        """t=5 时应在 (50,0,50)（线性插值）"""
        d = self._make()
        p = d.position_at(5)
        assert abs(p[0] - 50) < 1e-6

    def test_collides_inside_radius(self):
        d = self._make()
        # t=10 时位于 (100,0,50)，距离 5 < safety_radius 10
        assert d.collides(105, 0, 50, 10)

    def test_collides_outside_radius(self):
        d = self._make()
        assert not d.collides(200, 0, 50, 10)


class TestObstacleSet:
    def test_is_static_blocked(self, building):
        obs = ObstacleSet()
        obs.add_static(building)
        assert obs.is_static_blocked(2500, 2500, 100) is True
        assert obs.is_static_blocked(100, 100, 100) is False

    def test_is_dynamic_blocked(self):
        obs = ObstacleSet()
        obs.add_dynamic(DynamicObstacle(
            obstacle_id="d", name="d", obstacle_type="uav",
            trajectory=[(100, 100, 50, 0), (100, 100, 50, 10)],
            safety_radius=5,
        ))
        assert obs.is_dynamic_blocked(102, 100, 50, 5) is True
        assert obs.is_dynamic_blocked(200, 100, 50, 5) is False

    def test_is_blocked_combined(self, building):
        obs = ObstacleSet()
        obs.add_static(building)
        assert obs.is_blocked(2500, 2500, 100, 30) is True

    def test_summary(self, building):
        obs = ObstacleSet()
        obs.add_static(building)
        obs.add_dynamic(DynamicObstacle(
            obstacle_id="d", name="d", obstacle_type="uav",
            trajectory=[(0, 0, 50, 0), (10, 10, 50, 10)],
        ))
        s = obs.summary()
        assert s["static_count"] == 1
        assert s["dynamic_count"] == 1