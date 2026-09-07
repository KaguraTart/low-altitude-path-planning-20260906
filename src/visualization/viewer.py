"""
可视化展示程序

支持两种可视化模式:
1. Matplotlib 3D 静态图（适合论文/报告）
2. Plotly 交互式 HTML（适合演示/交互探索）

展示内容:
- 三维空间中的规划航线
- 静态障碍物（建筑物等）
- 动态障碍物轨迹
- 起点/终点/航点
- 时间维度动画（Plotly）
- 风险热力图
"""

from __future__ import annotations

import os
from typing import Dict, List, Optional, Tuple

import numpy as np

from ..models.space import SpaceTimeConfig
from ..models.obstacles import ObstacleSet, StaticObstacle, DynamicObstacle
from ..models.task import PlannedRoute, PlanningResult


class RouteVisualizer:
    """航线可视化器"""

    def __init__(
        self,
        config: SpaceTimeConfig,
        obstacle_set: Optional[ObstacleSet] = None,
    ):
        self.config = config
        self.obstacle_set = obstacle_set

        # 颜色方案
        self.route_colors = [
            "#1f77b4", "#ff7f0e", "#2ca02c", "#d62728",
            "#9467bd", "#8c564b", "#e377c2", "#7f7f7f",
            "#bcbd22", "#17becf",
        ]
        self.static_color = "#8B4513"  # 棕色 - 建筑物
        self.dynamic_color = "#FF4500"  # 橙红 - 动态障碍
        self.start_color = "#00FF00"  # 绿色 - 起点
        self.end_color = "#FF0000"  # 红色 - 终点

    def plot_matplotlib_3d(
        self,
        routes: List[PlannedRoute],
        output_path: str,
        title: str = "低空无人机路径规划可视化",
        show_obstacles: bool = True,
        show_dynamic: bool = True,
    ) -> str:
        """
        生成 Matplotlib 3D 静态图

        Args:
            routes: 规划航线列表
            output_path: 输出图片路径
            title: 图表标题
            show_obstacles: 是否显示静态障碍物
            show_dynamic: 是否显示动态障碍物

        Returns:
            输出文件路径
        """
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection

        fig = plt.figure(figsize=(16, 12))
        ax = fig.add_subplot(111, projection="3d")

        # 绘制静态障碍物
        if show_obstacles and self.obstacle_set:
            for obs in self.obstacle_set.static_obstacles:
                self._draw_static_obstacle_mpl(ax, obs)

        # 绘制动态障碍物轨迹
        if show_dynamic and self.obstacle_set:
            for obs in self.obstacle_set.dynamic_obstacles:
                traj = np.array(obs.trajectory)
                if len(traj) > 0:
                    ax.plot(
                        traj[:, 0], traj[:, 1], traj[:, 2],
                        color=self.dynamic_color, linewidth=2, linestyle="--",
                        alpha=0.7, label=f"动态障碍: {obs.name}",
                    )
                    ax.scatter(
                        [traj[0, 0]], [traj[0, 1]], [traj[0, 2]],
                        color=self.dynamic_color, s=50, marker="x",
                    )

        # 绘制航线
        for i, route in enumerate(routes):
            if route.status != "planned" or not route.trajectory:
                continue

            color = self.route_colors[i % len(self.route_colors)]
            traj = np.array([[p.x, p.y, p.z] for p in route.trajectory])

            ax.plot(
                traj[:, 0], traj[:, 1], traj[:, 2],
                color=color, linewidth=2.5,
                label=f"{route.route_id} ({route.task_id})",
            )

            # 起点
            ax.scatter(
                [traj[0, 0]], [traj[0, 1]], [traj[0, 2]],
                color=self.start_color, s=100, marker="o", edgecolors="black", linewidths=1.5,
                zorder=5,
            )
            # 终点
            ax.scatter(
                [traj[-1, 0]], [traj[-1, 1]], [traj[-1, 2]],
                color=self.end_color, s=100, marker="s", edgecolors="black", linewidths=1.5,
                zorder=5,
            )

            # 航点标记（每隔几个点标一个）
            if len(traj) > 5:
                step = max(1, len(traj) // 8)
                for j in range(0, len(traj), step):
                    ax.scatter(
                        [traj[j, 0]], [traj[j, 1]], [traj[j, 2]],
                        color=color, s=30, marker="^", alpha=0.6,
                    )

        # 设置坐标轴
        ax.set_xlabel("X (m)", fontsize=12)
        ax.set_ylabel("Y (m)", fontsize=12)
        ax.set_zlabel("Z (m)", fontsize=12)
        ax.set_title(title if title.isascii() else "Low-Altitude UAV Path Planning 3D Visualization", fontsize=14, fontweight="bold")

        ax.set_xlim(self.config.x_min, self.config.x_max)
        ax.set_ylim(self.config.y_min, self.config.y_max)
        ax.set_zlim(self.config.z_min, self.config.z_max)

        # 图例
        if len(routes) <= 10:
            ax.legend(loc="upper left", fontsize=8, bbox_to_anchor=(1.0, 1.0))

        ax.view_init(elev=25, azim=45)
        plt.tight_layout()

        os.makedirs(os.path.dirname(output_path) if os.path.dirname(output_path) else ".", exist_ok=True)
        plt.savefig(output_path, dpi=150, bbox_inches="tight")
        plt.close()

        return output_path

    def _draw_static_obstacle_mpl(self, ax, obs: StaticObstacle):
        """绘制静态障碍物（长方体）"""
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection

        x0, x1 = obs.x - obs.width, obs.x + obs.width
        y0, y1 = obs.y - obs.depth, obs.y + obs.depth
        z0, z1 = obs.z, obs.z + obs.height

        vertices = [
            [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0)],
            [(x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)],
            [(x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)],
            [(x0, y1, z0), (x1, y1, z0), (x1, y1, z1), (x0, y1, z1)],
            [(x0, y0, z0), (x0, y1, z0), (x0, y1, z1), (x0, y0, z1)],
            [(x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1)],
        ]

        poly = Poly3DCollection(vertices, alpha=0.3, facecolor=self.static_color, edgecolors="black")
        ax.add_collection3d(poly)

    def generate_plotly_html(
        self,
        routes: List[PlannedRoute],
        output_path: str,
        title: str = "低空无人机路径规划交互式可视化",
        show_obstacles: bool = True,
        show_dynamic: bool = True,
    ) -> str:
        """
        生成 Plotly 交互式 HTML 可视化

        Args:
            routes: 规划航线列表
            output_path: 输出 HTML 路径
            title: 图表标题
            show_obstacles: 是否显示静态障碍物
            show_dynamic: 是否显示动态障碍物

        Returns:
            输出文件路径
        """
        import plotly.graph_objects as go

        fig = go.Figure()

        # 静态障碍物
        if show_obstacles and self.obstacle_set:
            for obs in self.obstacle_set.static_obstacles:
                x0, x1 = obs.x - obs.width, obs.x + obs.width
                y0, y1 = obs.y - obs.depth, obs.y + obs.depth
                z0, z1 = obs.z, obs.z + obs.height

                fig.add_trace(go.Mesh3d(
                    x=[x0, x1, x1, x0, x0, x1, x1, x0],
                    y=[y0, y0, y1, y1, y0, y0, y1, y1],
                    z=[z0, z0, z0, z0, z1, z1, z1, z1],
                    i=[7, 0, 0, 0, 4, 4, 6, 6, 4, 0, 3, 2],
                    j=[3, 4, 1, 2, 5, 6, 5, 2, 0, 1, 6, 3],
                    k=[0, 7, 2, 3, 6, 7, 1, 1, 5, 5, 7, 6],
                    opacity=0.3,
                    color=self.static_color,
                    name=f"{obs.name} ({obs.obstacle_type})",
                    hovertemplate=f"<b>{obs.name}</b><br>类型: {obs.obstacle_type}<br>位置: ({obs.x:.0f}, {obs.y:.0f}, {obs.z:.0f})<br>尺寸: {obs.width*2:.0f}x{obs.depth*2:.0f}x{obs.height:.0f}m<extra></extra>",
                ))

        # 动态障碍物
        if show_dynamic and self.obstacle_set:
            for obs in self.obstacle_set.dynamic_obstacles:
                traj = np.array(obs.trajectory)
                if len(traj) > 0:
                    fig.add_trace(go.Scatter3d(
                        x=traj[:, 0], y=traj[:, 1], z=traj[:, 2],
                        mode="lines+markers",
                        line=dict(color=self.dynamic_color, width=4, dash="dash"),
                        marker=dict(size=4, color=traj[:, 3], colorscale="Hot"),
                        name=f"动态障碍: {obs.name}",
                        hovertemplate=f"<b>{obs.name}</b><br>时间: %{{marker.color:.1f}}s<br>位置: (%{{x:.1f}}, %{{y:.1f}}, %{{z:.1f}})<extra></extra>",
                    ))

        # 航线
        for i, route in enumerate(routes):
            if route.status != "planned" or not route.trajectory:
                continue

            color = self.route_colors[i % len(self.route_colors)]
            traj = np.array([[p.x, p.y, p.z, p.t, p.heading] for p in route.trajectory])

            # 航线
            fig.add_trace(go.Scatter3d(
                x=traj[:, 0], y=traj[:, 1], z=traj[:, 2],
                mode="lines+markers",
                line=dict(color=color, width=5),
                marker=dict(size=3, color=traj[:, 3], colorscale="Viridis"),
                name=f"{route.route_id} - {route.task_id}",
                hovertemplate=(
                    f"<b>{route.route_id}</b><br>"
                    f"任务: {route.task_id}<br>"
                    f"时间: %{{marker.color:.1f}}s<br>"
                    f"位置: (%{{x:.1f}}, %{{y:.1f}}, %{{z:.1f}})<br>"
                    f"航程: {route.total_distance:.1f}m | 时间: {route.total_time:.1f}s"
                    f"<extra></extra>"
                ),
            ))

            # 起点
            fig.add_trace(go.Scatter3d(
                x=[traj[0, 0]], y=[traj[0, 1]], z=[traj[0, 2]],
                mode="markers",
                marker=dict(size=8, color=self.start_color, symbol="circle", line=dict(width=2, color="black")),
                name=f"{route.route_id} 起点",
                hovertemplate=f"<b>起点</b><br>({traj[0,0]:.1f}, {traj[0,1]:.1f}, {traj[0,2]:.1f})<extra></extra>",
            ))

            # 终点
            fig.add_trace(go.Scatter3d(
                x=[traj[-1, 0]], y=[traj[-1, 1]], z=[traj[-1, 2]],
                mode="markers",
                marker=dict(size=8, color=self.end_color, symbol="diamond", line=dict(width=2, color="black")),
                name=f"{route.route_id} 终点",
                hovertemplate=f"<b>终点</b><br>({traj[-1,0]:.1f}, {traj[-1,1]:.1f}, {traj[-1,2]:.1f})<extra></extra>",
            ))

        # 布局
        fig.update_layout(
            title=dict(text=title, font=dict(size=18)),
            scene=dict(
                xaxis_title="X (m)",
                yaxis_title="Y (m)",
                zaxis_title="Z (m)",
                xaxis=dict(range=[self.config.x_min, self.config.x_max]),
                yaxis=dict(range=[self.config.y_min, self.config.y_max]),
                zaxis=dict(range=[self.config.z_min, self.config.z_max]),
                aspectmode="cube",
            ),
            width=1200,
            height=800,
            legend=dict(
                x=0.02, y=0.98,
                bgcolor="rgba(255,255,255,0.8)",
                bordercolor="gray",
                borderwidth=1,
            ),
            margin=dict(l=0, r=0, t=40, b=0),
        )

        os.makedirs(os.path.dirname(output_path) if os.path.dirname(output_path) else ".", exist_ok=True)
        fig.write_html(output_path, include_plotlyjs="cdn")

        return output_path

    def plot_risk_heatmap(
        self,
        route: PlannedRoute,
        output_path: str,
        height_slice: Optional[float] = None,
    ) -> str:
        """
        生成风险热力图（2D 俯视图）

        Args:
            route: 规划航线
            output_path: 输出路径
            height_slice: 高度切片 (None 则取航线平均高度)

        Returns:
            输出文件路径
        """
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        if not route.trajectory:
            return output_path

        if height_slice is None:
            height_slice = np.mean([p.z for p in route.trajectory])

        # 构建 2D 风险网格
        nx = 100
        ny = 100
        risk_grid = np.zeros((ny, nx))

        x_coords = np.linspace(self.config.x_min, self.config.x_max, nx)
        y_coords = np.linspace(self.config.y_min, self.config.y_max, ny)

        for i, x in enumerate(x_coords):
            for j, y in enumerate(y_coords):
                risk = 0.0
                if self.obstacle_set:
                    for obs in self.obstacle_set.static_obstacles:
                        dx = x - obs.x
                        dy = y - obs.y
                        dist = np.sqrt(dx * dx + dy * dy)
                        if dist < obs.to_cylinder_radius() + 100:
                            risk = max(risk, 1.0 - dist / (obs.to_cylinder_radius() + 100))
                risk_grid[j, i] = risk

        fig, ax = plt.subplots(figsize=(12, 10))
        im = ax.imshow(
            risk_grid,
            extent=[self.config.x_min, self.config.x_max, self.config.y_min, self.config.y_max],
            origin="lower",
            cmap="YlOrRd",
            alpha=0.7,
            vmin=0,
            vmax=1,
        )

        # 绘制航线
        traj = np.array([[p.x, p.y] for p in route.trajectory])
        ax.plot(traj[:, 0], traj[:, 1], "b-", linewidth=2.5, label=f"{route.route_id}")
        ax.plot(traj[0, 0], traj[0, 1], "go", markersize=12, label="起点")
        ax.plot(traj[-1, 0], traj[-1, 1], "rs", markersize=12, label="终点")

        # 绘制静态障碍物
        if self.obstacle_set:
            for obs in self.obstacle_set.static_obstacles:
                rect = plt.Rectangle(
                    (obs.x - obs.width, obs.y - obs.depth),
                    obs.width * 2, obs.depth * 2,
                    linewidth=1, edgecolor="black", facecolor="none", linestyle="--",
                )
                ax.add_patch(rect)

        ax.set_xlabel("X (m)", fontsize=12)
        ax.set_ylabel("Y (m)", fontsize=12)
        ax.set_title(f"Risk Heatmap (alt={height_slice:.0f}m) - {route.route_id}", fontsize=14)
        ax.legend(fontsize=10)
        plt.colorbar(im, ax=ax, label="风险值")

        os.makedirs(os.path.dirname(output_path) if os.path.dirname(output_path) else ".", exist_ok=True)
        plt.savefig(output_path, dpi=150, bbox_inches="tight")
        plt.close()

        return output_path
