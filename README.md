# 低空路径规划调度 20260906

> 全空间无人体系建设项目 — 路径规划与导航算法
>
> **3.25 多维时空大规模并行规划算法** & **3.26 智能飞行线路规划算法**

## 项目简介

本项目实现低空无人机（UAV）在复杂城市环境中的多维时空路径规划与智能航线优化，支持静态/动态障碍物避障、多飞行器并行规划、多目标航线优化，以及三维可视化展示。

### 对应需求

| 子项目 | 模块 | 状态 |
|--------|------|------|
| 3.25 多维时空大规模并行规划 | 多维时空约束建模 | ✅ |
| 3.25 | 静态与动态避障 | ✅ |
| 3.25 | 多源规划数据融合 | ✅ |
| 3.25 | 大规模并行规划引擎 | ✅ |
| 3.25 | 秒级性能优化与规模验证 | ✅ |
| 3.26 智能飞行线路规划 | 地理管制数据融合（气象不做） | ✅ |
| 3.26 | 飞行器性能与任务约束建模 | ✅ |
| 3.26 | 多目标航线优化 | ✅ |
| 3.26 | 航点生成与结果解释 | ✅ |
| 3.26 | 规划服务封装与综合评测 | ✅ |

## 快速开始

### 环境要求

- Python 3.11+
- 依赖：numpy, scipy, matplotlib, plotly, pydantic

### 安装

```bash
pip install -r requirements.txt
```

### 运行

```bash
# 完整规划（A* + 多目标优化 + 可视化）
python run_planning.py

# 仅时空 A* 规划
python run_planning.py --mode spacetime

# 仅多目标优化
python run_planning.py --mode optimize

# 批量并行规划
python run_planning.py --mode batch

# 指定数据/输出目录
python run_planning.py --data-dir ./data --output-dir ./output
```

### 查看可视化

运行后在 `output/` 目录生成：
- `routes_3d.png` — 3D 静态图
- `routes_interactive.html` — 交互式 3D 可视化（浏览器打开）
- `risk_heatmap.png` — 风险热力图

## 项目结构

```
├── run_planning.py              # 主运行脚本
├── requirements.txt             # Python 依赖
├── data/                        # 模拟输入数据
│   ├── config.json              # 时空网格配置
│   ├── static_obstacles.json    # 静态障碍物（12个建筑物/塔）
│   ├── dynamic_obstacles.json   # 动态障碍物（4个移动目标）
│   ├── aircraft_config.json     # 飞行器性能配置（5架）
│   └── tasks.json               # 飞行任务（5个）
├── src/
│   ├── models/                  # 数据模型
│   │   ├── space.py             # 多维时空网格
│   │   ├── obstacles.py         # 静态/动态障碍物
│   │   ├── aircraft.py          # 飞行器性能 & 任务
│   │   └── task.py              # 轨迹 & 规划结果
│   ├── planners/                # 规划算法
│   │   ├── spacetime_astar.py   # 多维时空 A*（3.25核心）
│   │   ├── parallel_planner.py  # 大规模并行引擎（3.25）
│   │   └── route_optimizer.py   # 多目标航线优化（3.26）
│   ├── data_fusion/             # 数据融合
│   │   └── obstacle_fusion.py   # 多源障碍物融合
│   ├── io/                      # 输入输出
│   │   ├── input_loader.py      # 数据加载
│   │   └── output_writer.py     # 结果写入 & 报告
│   └── visualization/           # 可视化
│       └── viewer.py            # 3D可视化 + 热力图
├── docs/                        # 文档
│   ├── input_output_spec.md     # 输入输出格式规范
│   └── algorithm_design.md      # 算法设计说明
└── output/                      # 规划输出（运行后生成）
```

## 核心算法

### 3.25 多维时空 A*

在 `(x, y, z, t)` 四维时空网格中搜索无碰撞路径：
- **状态**：`(ix, iy, iz, it)` 网格坐标
- **动作**：6 邻域空间移动 + 等待
- **启发**：曼哈顿距离 × 时间步长（可采纳）
- **避障**：静态障碍物（全时段占用）+ 动态障碍物（时段占用）

### 3.26 多目标航线优化

四目标加权优化（航程/时间/风险/能耗）：
1. 时空 A* 生成初始路径
2. 三次样条插值平滑
3. 碰撞检测与回退
4. 性能约束修正（速度/爬升率/转弯率）
5. 航点提取与结果解释

## 输入输出格式

详见 [docs/input_output_spec.md](docs/input_output_spec.md)

- **输入**：config.json, static_obstacles.json, dynamic_obstacles.json, aircraft_config.json, tasks.json
- **输出**：规划结果 JSON, 航点指令 JSON, 规划报告 MD, 可视化图片/HTML

## 性能

| 指标 | 数值 |
|------|------|
| 网格规模 | 50×50×10×120 |
| 单任务规划 | < 10ms |
| 5 任务批量 | ~60ms |
| 规划成功率 | 100%（测试场景） |

## 开发人员

- 李奥勇（算法开发）
- 排期：2026/09/01 — 2026/09/20

## 许可证

MIT License
