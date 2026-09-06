"""
输出数据写入器

将规划结果写入 JSON 文件，支持:
- 单条航线输出
- 批量规划结果输出
- 航点指令输出
- 规划报告生成
"""

from __future__ import annotations

import json
import os
from datetime import datetime
from typing import Dict, List, Optional

from ..models.task import PlannedRoute, PlanningResult


def ensure_output_dir(output_dir: str) -> str:
    """确保输出目录存在"""
    os.makedirs(output_dir, exist_ok=True)
    return output_dir


def write_route(route: PlannedRoute, output_dir: str, filename: Optional[str] = None) -> str:
    """
    写入单条航线结果

    Returns:
        输出文件路径
    """
    ensure_output_dir(output_dir)
    if filename is None:
        filename = f"{route.route_id}.json"
    filepath = os.path.join(output_dir, filename)

    with open(filepath, "w", encoding="utf-8") as f:
        json.dump(route.to_dict(), f, ensure_ascii=False, indent=2)

    return filepath


def write_planning_result(result: PlanningResult, output_dir: str, filename: Optional[str] = None) -> str:
    """
    写入批量规划结果

    Returns:
        输出文件路径
    """
    ensure_output_dir(output_dir)
    if filename is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"planning_result_{result.request_id}_{timestamp}.json"
    filepath = os.path.join(output_dir, filename)

    with open(filepath, "w", encoding="utf-8") as f:
        json.dump(result.summary(), f, ensure_ascii=False, indent=2)

    return filepath


def write_waypoints(waypoints: List[dict], output_dir: str, filename: str) -> str:
    """
    写入航点指令文件

    Returns:
        输出文件路径
    """
    ensure_output_dir(output_dir)
    filepath = os.path.join(output_dir, filename)

    output = {
        "generated_at": datetime.now().isoformat(),
        "waypoint_count": len(waypoints),
        "waypoints": waypoints,
    }

    with open(filepath, "w", encoding="utf-8") as f:
        json.dump(output, f, ensure_ascii=False, indent=2)

    return filepath


def generate_report(
    result: PlanningResult,
    config_info: dict,
    obstacle_info: dict,
    output_dir: str,
    filename: Optional[str] = None,
) -> str:
    """
    生成规划报告（Markdown 格式）

    Returns:
        报告文件路径
    """
    ensure_output_dir(output_dir)
    if filename is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"report_{result.request_id}_{timestamp}.md"
    filepath = os.path.join(output_dir, filename)

    lines = [
        f"# 路径规划报告 - {result.request_id}",
        "",
        f"**生成时间**: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        "",
        "## 1. 规划概览",
        "",
        f"- 总任务数: {len(result.routes)}",
        f"- 成功: {result.success_count}",
        f"- 失败: {result.failed_count}",
        f"- 总规划耗时: {result.total_planning_time_ms:.2f} ms",
        f"- 平均单任务耗时: {result.total_planning_time_ms / max(1, len(result.routes)):.2f} ms",
        "",
        "## 2. 环境配置",
        "",
    ]

    for key, value in config_info.items():
        lines.append(f"- {key}: {value}")

    lines.extend([
        "",
        "## 3. 障碍物信息",
        "",
    ])
    for key, value in obstacle_info.items():
        lines.append(f"- {key}: {value}")

    lines.extend([
        "",
        "## 4. 航线详情",
        "",
    ])

    for i, route in enumerate(result.routes):
        lines.append(f"### 4.{i+1} {route.route_id} ({route.task_id})")
        lines.append("")
        lines.append(f"- 状态: {route.status}")
        if route.message:
            lines.append(f"- 说明: {route.message}")
        lines.append(f"- 算法: {route.algorithm}")
        lines.append(f"- 规划耗时: {route.planning_time_ms:.2f} ms")
        lines.append(f"- 总航程: {route.total_distance:.2f} m")
        lines.append(f"- 总时间: {route.total_time:.2f} s")
        lines.append(f"- 估算能耗: {route.total_energy:.2f}")
        lines.append(f"- 最大风险: {route.max_risk:.4f}")
        lines.append(f"- 平均风险: {route.avg_risk:.4f}")
        lines.append(f"- 最近障碍距离: {route.min_obstacle_distance:.2f} m")
        lines.append(f"- 轨迹点数: {len(route.trajectory)}")
        lines.append("")

    with open(filepath, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))

    return filepath
