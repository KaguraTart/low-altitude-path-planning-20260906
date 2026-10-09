"""pytest 共享 fixtures：项目根目录加入 sys.path，提供常用测试对象"""
import os
import sys

import pytest

# 项目根目录加入 path，保证 `import src.xxx` 可用
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from src.models.space import SpaceTimeConfig, SpaceTimeGrid
from src.models.obstacles import StaticObstacle, ObstacleSet
from src.models.aircraft import AircraftPerformance


@pytest.fixture
def config():
    """默认时空网格配置（与 input/config.json 一致）"""
    cfg_path = os.path.join(ROOT, "input", "config.json")
    if os.path.exists(cfg_path):
        import json
        with open(cfg_path, encoding="utf-8") as f:
            return SpaceTimeConfig(**json.load(f))
    return SpaceTimeConfig()


@pytest.fixture
def empty_grid(config):
    """无障碍的空网格"""
    return SpaceTimeGrid(config)


@pytest.fixture
def obstacle_set():
    """空障碍物集合"""
    return ObstacleSet()


@pytest.fixture
def building():
    """中心大厦（底部中心 2500,2500,0，半宽/半深 80，高 200）"""
    return StaticObstacle(
        obstacle_id="building-001",
        name="中心大厦",
        obstacle_type="building",
        x=2500, y=2500, z=0,
        width=80, depth=80, height=200,
        source="test",
    )


@pytest.fixture
def perf():
    """默认飞行器参数"""
    return AircraftPerformance(aircraft_id="test-uav")


@pytest.fixture(scope="session")
def input_dir():
    """input/ 目录（真实样例数据）"""
    return os.path.join(ROOT, "input")


@pytest.fixture(scope="session")
def real_obstacle_set(input_dir):
    """多源融合后的真实障碍物集合"""
    from src.io.input_loader import load_obstacle_set_fused
    obstacle_set, _ = load_obstacle_set_fused(
        [os.path.join(input_dir, "static_obstacles.json")],
        [os.path.join(input_dir, "dynamic_obstacles.json")],
    )
    return obstacle_set


@pytest.fixture(scope="session")
def real_performances(input_dir):
    """真实飞行器配置"""
    from src.io.input_loader import load_aircraft_performances
    return load_aircraft_performances(
        os.path.join(input_dir, "aircraft_config.json")
    )


@pytest.fixture(scope="session")
def real_tasks(input_dir):
    """真实任务列表"""
    from src.io.input_loader import load_tasks
    return load_tasks(os.path.join(input_dir, "tasks.json"))
