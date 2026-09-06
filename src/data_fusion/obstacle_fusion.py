"""
多源规划数据融合 (3.25 模块三)

将来自不同数据源的障碍物数据（静态建筑物、动态飞行器轨迹、
禁飞区、地形等）进行统一融合，生成规划引擎可用的统一障碍物集合。

支持:
- 多源数据格式统一
- 数据去重与合并
- 冲突检测与优先级处理
- 数据质量评估
"""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple

import numpy as np

from ..models.obstacles import StaticObstacle, DynamicObstacle, ObstacleSet


class ObstacleDataFusion:
    """
    多源障碍物数据融合器

    将不同来源、不同格式的障碍物数据融合为统一的 ObstacleSet。
    """

    def __init__(self):
        self.sources: Dict[str, dict] = {}
        self.warnings: List[str] = []

    def register_source(self, source_id: str, source_type: str, priority: int = 5):
        """
        注册数据源

        Args:
            source_id: 数据源唯一标识
            source_type: 数据类型 ("static_buildings", "dynamic_uav", "no_fly_zone", "terrain", "other")
            priority: 优先级 (1-10, 10最高)，冲突时高优先级覆盖低优先级
        """
        self.sources[source_id] = {
            "type": source_type,
            "priority": priority,
            "static_count": 0,
            "dynamic_count": 0,
        }

    def add_static_from_dict(self, source_id: str, data: dict) -> Optional[StaticObstacle]:
        """从字典添加静态障碍物"""
        try:
            obs = StaticObstacle(
                obstacle_id=data.get("id", data.get("obstacle_id", "unknown")),
                name=data.get("name", "unnamed"),
                obstacle_type=data.get("type", data.get("obstacle_type", "other")),
                x=float(data["x"]),
                y=float(data["y"]),
                z=float(data.get("z", 0)),
                width=float(data.get("width", 20.0)),
                depth=float(data.get("depth", 20.0)),
                height=float(data.get("height", 50.0)),
                source=source_id,
            )
            if source_id in self.sources:
                self.sources[source_id]["static_count"] += 1
            return obs
        except (KeyError, ValueError, TypeError) as e:
            self.warnings.append(f"静态障碍物数据解析失败 (source={source_id}): {e}")
            return None

    def add_dynamic_from_dict(self, source_id: str, data: dict) -> Optional[DynamicObstacle]:
        """从字典添加动态障碍物"""
        try:
            trajectory = []
            for point in data.get("trajectory", []):
                trajectory.append((
                    float(point["x"]),
                    float(point["y"]),
                    float(point["z"]),
                    float(point["t"]),
                ))

            obs = DynamicObstacle(
                obstacle_id=data.get("id", data.get("obstacle_id", "unknown")),
                name=data.get("name", "unnamed"),
                obstacle_type=data.get("type", data.get("obstacle_type", "other")),
                trajectory=trajectory,
                safety_radius=float(data.get("safety_radius", 10.0)),
                source=source_id,
            )
            if source_id in self.sources:
                self.sources[source_id]["dynamic_count"] += 1
            return obs
        except (KeyError, ValueError, TypeError) as e:
            self.warnings.append(f"动态障碍物数据解析失败 (source={source_id}): {e}")
            return None

    def fuse(
        self,
        static_data: Dict[str, List[dict]],
        dynamic_data: Dict[str, List[dict]],
    ) -> ObstacleSet:
        """
        融合多源数据

        Args:
            static_data: {source_id: [obstacle_dict, ...]}
            dynamic_data: {source_id: [obstacle_dict, ...]}

        Returns:
            ObstacleSet: 融合后的障碍物集合
        """
        obstacle_set = ObstacleSet()
        self.warnings = []

        # 按优先级排序数据源
        all_sources = set(list(static_data.keys()) + list(dynamic_data.keys()))
        sorted_sources = sorted(
            all_sources,
            key=lambda s: self.sources.get(s, {}).get("priority", 5),
            reverse=True,
        )

        # 融合静态障碍物（去重：相同位置 + 相同类型视为重复）
        seen_static = set()
        for source_id in sorted_sources:
            for data in static_data.get(source_id, []):
                obs = self.add_static_from_dict(source_id, data)
                if obs is None:
                    continue
                key = (round(obs.x, -1), round(obs.y, -1), obs.obstacle_type)
                if key in seen_static:
                    self.warnings.append(
                        f"静态障碍物去重: {obs.obstacle_id} (source={source_id}) 与已有障碍物位置重叠"
                    )
                    continue
                seen_static.add(key)
                obstacle_set.add_static(obs)

        # 融合动态障碍物（按 ID 去重）
        seen_dynamic = set()
        for source_id in sorted_sources:
            for data in dynamic_data.get(source_id, []):
                obs = self.add_dynamic_from_dict(source_id, data)
                if obs is None:
                    continue
                if obs.obstacle_id in seen_dynamic:
                    self.warnings.append(
                        f"动态障碍物去重: {obs.obstacle_id} (source={source_id}) 已存在"
                    )
                    continue
                seen_dynamic.add(obs.obstacle_id)
                obstacle_set.add_dynamic(obs)

        return obstacle_set

    def quality_report(self) -> dict:
        """生成数据质量报告"""
        return {
            "registered_sources": len(self.sources),
            "source_details": self.sources,
            "warnings": self.warnings,
            "warning_count": len(self.warnings),
        }
