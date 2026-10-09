"""SpaceTimeConfig / SpaceTimeGrid / NoFlyZone / AirspaceBoundary 单元测试"""
import pytest

from src.models.space import (
    AirspaceBoundary,
    NoFlyZone,
    SpaceTimeGrid,
)


# ---------- SpaceTimeConfig 网格维度 ----------

class TestGridDimensions:
    def test_dimensions(self, config):
        """nx/ny/nz/nt 由空间范围和分辨率推导（dx=100,dz=50,t_max=1200,dt=10）"""
        assert config.nx == 50   # 5000/100
        assert config.ny == 50
        assert config.nz == 10   # 500/50
        assert config.nt == 120  # 1200/10

    def test_total_cells(self, config):
        assert config.nx * config.ny * config.nz * config.nt == 3_000_000


# ---------- 坐标转换 ----------

class TestCoordinateConversion:
    def test_to_grid_origin(self, config):
        """原点映射到 (0,0,0)"""
        assert config.to_grid(0.0, 0.0, 0.0) == (0, 0, 0)

    def test_to_grid_max_corner(self, config):
        """最大角映射到最后一格（clip 生效）"""
        ix, iy, iz = config.to_grid(4999.9, 4999.9, 499.9)
        assert ix == config.nx - 1
        assert iy == config.ny - 1
        assert iz == config.nz - 1

    def test_to_grid_out_of_bounds_clamped(self, config):
        """越界坐标被钳制到边界"""
        ix, iy, iz = config.to_grid(-100, 99999, 1000)
        assert ix == 0
        assert iy == config.ny - 1
        assert iz == config.nz - 1

    def test_grid_world_roundtrip(self, config):
        """网格 → 世界 → 网格 往返一致（返回格中心）"""
        ix, iy, iz = 10, 25, 3
        x, y, z = config.to_world(ix, iy, iz)
        ix2, iy2, iz2 = config.to_grid(x, y, z)
        assert (ix, iy, iz) == (ix2, iy2, iz2)

    def test_time_step_roundtrip(self, config):
        """时间步 → 时间 → 时间步 往返一致"""
        step = 37
        t = config.step_to_time(step)
        assert config.time_to_step(t) == step


# ---------- NoFlyZone ----------

class TestNoFlyZone:
    def test_contains_inside(self):
        zone = NoFlyZone(zone_id="z1", name="机场", center_x=1000, center_y=1000,
                         radius=200, z_min=0, z_max=300)
        assert zone.contains(1100, 1000, 100) is True

    def test_contains_outside_radius(self):
        zone = NoFlyZone(zone_id="z1", name="机场", center_x=1000, center_y=1000,
                         radius=200, z_min=0, z_max=300)
        assert zone.contains(2000, 1000, 100) is False

    def test_contains_outside_z(self):
        """超过禁飞区高度即不受约束"""
        zone = NoFlyZone(zone_id="z1", name="机场", center_x=1000, center_y=1000,
                         radius=200, z_min=0, z_max=100)
        assert zone.contains(1000, 1000, 200) is False


# ---------- AirspaceBoundary ----------

class TestAirspaceBoundary:
    def test_contains(self):
        b = AirspaceBoundary(x_min=0, x_max=5000, y_min=0, y_max=5000,
                             z_min=0, z_max=500)
        assert b.contains(2500, 2500, 250) is True
        assert b.contains(6000, 2500, 250) is False
        assert b.contains(2500, 2500, 600) is False


# ---------- SpaceTimeGrid 障碍标记 ----------

class TestSpaceTimeGrid:
    def test_empty_all_free(self, empty_grid):
        """空网格所有格空闲"""
        assert empty_grid.is_free(0, 0, 0, 0)
        assert empty_grid.is_free(49, 49, 9, 119)
        assert empty_grid.occupancy_count() == 0

    def test_out_of_bounds_not_free(self, empty_grid):
        """越界视为占用"""
        assert not empty_grid.is_free(-1, 0, 0, 0)
        assert not empty_grid.is_free(0, 0, 0, 120)  # nt=120 越界

    def test_set_static_obstacle_fills_cells(self, config, empty_grid):
        """标记球形静态障碍后，占用格数 > 0"""
        empty_grid.set_static_obstacle(2500, 2500, 250, radius=50)
        assert empty_grid.occupancy_count() > 0
        ix, iy, iz = config.to_grid(2500, 2500, 250)
        assert not empty_grid.is_free(ix, iy, iz, 0)

    def test_set_static_box(self, config, empty_grid):
        """长方体标记占用区域内所有时间步"""
        empty_grid.set_static_box(2400, 2600, 2400, 2600, 0, 100)
        count = empty_grid.occupancy_count()
        assert count > 0
        # 区域外仍然空闲
        ix_out, iy_out, iz_out = config.to_grid(100, 100, 100)
        assert empty_grid.is_free(ix_out, iy_out, iz_out, 0)

    def test_set_dynamic_obstacle(self, config, empty_grid):
        """动态障碍只在轨迹时间步占用"""
        trajectory = [(1000, 1000, 100, 50)]
        empty_grid.set_dynamic_obstacle(trajectory, radius=0)
        it = config.time_to_step(50)
        assert not empty_grid.is_free(
            *config.to_grid(1000, 1000, 100), it
        )