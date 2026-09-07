"""grid 包 - 网格模块（任务书 §十一 要求 flight_path_planning_algorithm/grid/）

提供：
- SpaceTimeGrid: 四维时空占用网格（重导出自 src.models.space）
- CostGrid: 网格通行代价

任务书要求"建立三维GridMap，包括网格坐标、占用状态、通行代价"。
占用和代价正交：占用 = 能不能走，代价 = 走起来多费力。
"""
from ..models.space import SpaceTimeGrid
from .cost import CostGrid, build_default_cost_grid

__all__ = ["SpaceTimeGrid", "CostGrid", "build_default_cost_grid"]