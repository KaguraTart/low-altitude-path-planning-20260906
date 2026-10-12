# 接口文档（API Reference）

> 本文档是算法**对外契约的唯一权威来源**：函数名、参数、返回结构、错误码、内部变量。
>
> 联调步骤与可直接执行的命令见 [`API_QUICKSTART.md`](API_QUICKSTART.md)。
> 早期文档已移至 [`deprecated/`](deprecated/)，仅作历史参考。

**接口版本**：1.0.0 　|　**更新日期**：2026-10-12

---

## 0. 背景与作用

### 这个项目是什么

本项目是「自贡低空横向无人机路径调度」课题的算法实现，对应任务书 **3.25 多维时空大规模并行规划算法** 与 **3.26 智能飞行线路规划算法** 两个模块。

核心能力是把一段低空空域（默认 5000m × 5000m × 500m、时间窗 1200 秒）建模成
**四维时空网格** `(x, y, z, t)`，在其中有约束地搜索无人机航线，并给出：

- **可行航线**：避开静态建筑 / 塔杆、绕开动态障碍物（其他飞行器、鸟群）的安全半径；
- **多机协同**：批量规划时保证任意两条航线满足最小安全间隔，冲突在规划闭环内消化；
- **多目标权衡**：在航程、时间、风险、能耗四个目标间按权重折中；
- **合法性验证**：对航线做 7 个维度的检查（连续性、障碍、性能、高度、时空占用等）；
- **规划解释**：说明这条航线为什么这样走、避障几次、性能裕度还剩多少。

### 为什么需要这份接口

在此之前，算法只有**命令行脚本**这一种使用方式（`plan_single.py` / `plan_multi.py`），
调用方必须「起子进程 → 等它跑完 → 去 `output/` 目录里翻 JSON 文件」。
这带来三个实际困难：

1. **无法被其他系统集成**——别人的调度平台没法直接调用算法，只能解析文件；
2. **字段契约不明确**——输入输出格式散落在代码和早期文档里，联调时对方只能读源码猜；
3. **无法在线验证**——没有可访问的服务端点，对方没法确认自己构造的请求是否被正确接受。

本次改动把算法封装成**两层接口**，解决上述问题：

| 层 | 解决什么 |
|----|----------|
| **Python SDK**（`uav_planner`） | 对方是 Python 项目时直接 `import` 调用，输入输出都是纯 JSON，不需要了解内部 `dataclass` |
| **REST 服务**（`server/` + `serve.py`） | 对方是 Java / Go / 前端 / 其他语言时走 HTTP；自带 Swagger 交互文档，**不写代码就能在网页上试调用** |

### 本文档的作用

本文档是**对接双方共同依据的契约**：

- 对接方按本文档构造请求、解析响应、处理错误；
- 算法方按本文档维护接口，任何破坏性改动都必须在本文档体现并升版本号；
- 出现争议时，以本文档描述的行为为准（本文档中的字段表、错误码、常量值均已与
  实现逐项核对，见文末「文档一致性」）。

**本文档不含**：算法原理（见 [`deprecated/algorithm_design.md`](deprecated/algorithm_design.md)，
其算法章节仍然有效）、命令行脚本用法（见 [`../README.md`](../README.md)）。

---

## 目录

0. [背景与作用](#0-背景与作用)
1. [接口总览](#1-接口总览)
2. [输入数据结构](#2-输入数据结构)
3. [Python SDK 函数](#3-python-sdk-函数)
4. [返回结构](#4-返回结构)
5. [错误码](#5-错误码)
6. [HTTP 接口](#6-http-接口)
7. [内部变量](#7-内部变量)
8. [单位与约定](#8-单位与约定)

---

## 1. 接口总览

算法以**两层**对外提供，两层共用同一份实现：

```
┌──────────────────────────────────────────────────────────┐
│  ① REST HTTP 服务（server/，端口 8000）                   │
│     POST /api/v1/plan/single      POST /api/v1/plan/batch │
│     POST /api/v1/validate         POST /api/v1/jobs       │
│     GET  /docs  ← Swagger 交互文档                        │
└───────────────────────────┬──────────────────────────────┘
                            │  薄封装，字段名逐字一致
┌───────────────────────────▼──────────────────────────────┐
│  ② Python SDK（uav_planner/）                             │
│     plan_single()  plan_batch()  plan_from_dir()          │
│     validate_route()  get_version()                       │
└───────────────────────────┬──────────────────────────────┘
                            │
┌───────────────────────────▼──────────────────────────────┐
│  ③ 算法内核（src/）                                        │
│     时空 A* · 多目标优化 · 并行规划 · 冲突避免 · 路径验证  │
└──────────────────────────────────────────────────────────┘
```

**选哪一层**：

| 场景 | 用哪层 |
|------|--------|
| 对方是 Python 项目 | 直接用 SDK，`pip install` / `import uav_planner` |
| 对方是 Java / Go / 前端 / 其他语言 | 用 REST 服务 |
| 联调期快速验证字段 | 打开 `http://<ip>:8000/docs` 在网页上直接试 |

**两条核心设计约定**：

1. **接口只进出纯 JSON 结构**（dict / list / str / number / bool / null），
   不要求调用方 import 任何内部类，返回值也不会带 numpy 标量或 `NaN`。
2. **请求体字段名与 SDK 函数参数名逐字一致**。HTTP 层用
   `model_dump()` 直接展开成 kwargs，不做二次命名——对接方只需记住一套名字。

---

## 2. 输入数据结构

### 2.1 `Point` — 空间点

| 字段 | 类型 | 单位 | 必填 | 默认 | 说明 |
|------|------|------|------|------|------|
| `x` | float | 米 | ✅ | — | X 坐标 |
| `y` | float | 米 | ✅ | — | Y 坐标 |
| `z` | float | 米 | ✅ | — | Z 坐标（高度） |

可简写为数组 `[x, y, z]`。

### 2.2 `SpaceTimeConfig` — 时空网格配置

**不传则使用默认值**（与仓库 `input/config.json` 一致）。

| 字段 | 类型 | 单位 | 默认 | 说明 |
|------|------|------|------|------|
| `x_min` / `x_max` | float | 米 | 0 / 5000 | X 轴范围 |
| `y_min` / `y_max` | float | 米 | 0 / 5000 | Y 轴范围 |
| `z_min` / `z_max` | float | 米 | 0 / 500 | Z 轴范围（高度） |
| `dx` / `dy` / `dz` | float | 米/格 | 100 / 100 / 50 | 空间分辨率，**须 > 0** |
| `t_min` / `t_max` | float | 秒 | 0 / 1200 | 规划时间窗 |
| `dt` | float | 秒/步 | 10 | 时间步长，**须 > 0** |

**约束**（违反抛 `INVALID_INPUT`）：
- `dx, dy, dz, dt > 0`；
- `x_min < x_max`，`y_min < y_max`，`z_min < z_max`，`t_min < t_max`；
- 网格规模 `nx×ny×nz×nt` 建议 ≤ 1000 万格（默认配置为 300 万格，占用约 2.9 MB）。

### 2.3 `StaticObstacle` — 静态障碍物

AABB 长方体包围盒。占据范围：
`[x-width, x+width] × [y-depth, y+depth] × [z, z+height]`

| 字段 | 类型 | 单位 | 必填 | 默认 | 说明 |
|------|------|------|------|------|------|
| `id` | string | — | ✅ | — | 唯一 ID |
| `name` | string | — | | `"unnamed"` | 显示名 |
| `type` | string | — | | `"other"` | `building` / `tower` / `terrain` / `other` |
| `x` | float | 米 | ✅ | — | 中心 X |
| `y` | float | 米 | ✅ | — | 中心 Y |
| `z` | float | 米 | | 0 | 底部 Z |
| `width` | float | 米 | | 20 | X 方向**半宽**（不是全长） |
| `depth` | float | 米 | | 20 | Y 方向**半深**（不是全深） |
| `height` | float | 米 | | 50 | 高度（自 `z` 向上） |
| `source` | string | — | | `"api"` | 数据源标识 |

> ⚠️ `width` / `depth` 是**半宽 / 半深**。一个 160m×160m 的建筑应填
> `width=80, depth=80`。这是最容易填错的两个字段。

### 2.4 `DynamicObstacle` — 动态障碍物

| 字段 | 类型 | 单位 | 必填 | 默认 | 说明 |
|------|------|------|------|------|------|
| `id` | string | — | ✅ | — | 唯一 ID |
| `name` | string | — | | `"unnamed"` | 显示名 |
| `type` | string | — | | `"other"` | `uav` / `vehicle` / `bird_flock` / `other` |
| `safety_radius` | float | 米 | | 10 | 安全避让半径 |
| `source` | string | — | | `"api"` | 数据源标识 |
| `trajectory` | array | — | ✅ | — | 轨迹点列表，按 `t` 递增 |

轨迹点在任意时刻的位置由相邻两点**线性插值**得到；查询时刻早于首点或晚于末点时，取端点值（不外推）。

### 2.5 `TrajectoryPoint` — 轨迹点

| 字段 | 类型 | 单位 | 必填 | 默认 | 说明 |
|------|------|------|------|------|------|
| `x` / `y` / `z` | float | 米 | ✅ | — | 位置 |
| `t` | float | 秒 | ✅ | — | 时间戳 |
| `heading` | float | 度 | | 0 | 航向角（仅输出时有意义） |

### 2.6 `Waypoint` — 航点

| 字段 | 类型 | 单位 | 必填 | 默认 | 说明 |
|------|------|------|------|------|------|
| `id` | string | — | | 自动编号 | 航点 ID |
| `x` / `y` / `z` | float | 米 | ✅ | — | 位置 |
| `type` | string | — | | `"via"` | `start` / `via` / `end` / `task` |
| `earliest_arrival` | float | 秒 | | null | 最早到达时间 |
| `latest_arrival` | float | 秒 | | null | 最晚到达时间 |
| `hover_time` | float | 秒 | | 0 | 到点后悬停时长 |
| `task_description` | string | — | | `""` | 任务描述 |

### 2.7 `Task` — 飞行任务

| 字段 | 类型 | 单位 | 必填 | 默认 | 说明 |
|------|------|------|------|------|------|
| `id` | string | — | ✅ | — | 任务唯一 ID |
| `name` | string | — | | null | 任务名 |
| `aircraft_id` | string | — | ✅ | — | 执行飞行器 ID，**必须**在 `aircraft` 中存在 |
| `type` | string | — | | `"other"` | `delivery` / `inspection` / `patrol` / `surveillance` |
| `priority` | int | — | | 5 | 调度优先级 1-10（10 最高） |
| `departure_time` | float | 秒 | | 0 | 期望起飞时间 |
| `objective_weights` | object | — | | 见下 | 多目标权重，**和须为 1.0** |
| `requirements` | object | — | | null | 任务需求约束，见下 |
| `waypoints` | array | — | ✅ | — | 航点序列，**至少 2 个** |

**`objective_weights` 默认值**：

| 键 | 含义 | 默认 |
|----|------|------|
| `distance` | 总航程最短 | 0.3 |
| `time` | 总飞行时间最短 | 0.3 |
| `risk` | 距障碍物风险最低 | 0.2 |
| `energy` | 估算能耗最低 | 0.2 |

**`requirements` 字段**（批量规划做 `validate_tasks=true` 时用于剔除不兼容任务）：

| 字段 | 类型 | 单位 | 默认 | 说明 |
|------|------|------|------|------|
| `payload_kg` | float | kg | 0 | 需要的载荷 |
| `required_endurance_s` | float | 秒 | 0 | 需要的最小续航 |
| `required_range_m` | float | 米 | 0 | 需要的最小航程 |
| `required_max_speed_mps` | float | m/s | 0 | 需要的最小最大速度 |
| `min_ceiling_m` | float | 米 | 0 | 需要的最低飞行高度 |
| `delivery_window` | [float, float] | 秒 | null | 投递时间窗 |
| `priority_level` | int (1-5) | — | 1 | 业务优先级 |
| `service_type` | string | — | `"delivery"` | 服务类型 |

### 2.8 `Aircraft` — 飞行器

**不传则使用默认包线**（最大速度 15 m/s 的通用四旋翼）。支持嵌套分组，也兼容旧版扁平字段。

| 分组 | 字段 | 类型 | 单位 | 默认 |
|------|------|------|------|------|
| 顶层 | `id` | string | — | 必填 |
| | `model` | string | — | `generic-quadcopter` |
| | `category` | string | — | `general` |
| `physical` | `weight_kg` | float | kg | 1.0 |
| | `payload_capacity_kg` | float | kg | 0.5 |
| | `wingspan_m` / `length_m` / `width_m` / `height_m` | float | 米 | 0.5 / 0.5 / 0.5 / 0.2 |
| `performance` | `max_speed_mps` | float | m/s | 15 |
| | `max_climb_rate_mps` | float | m/s | 5 |
| | `max_descent_rate_mps` | float | m/s | 3 |
| | `max_turn_rate_dps` | float | 度/秒 | 45 |
| | `max_acceleration_mps2` | float | m/s² | 3 |
| | `min_turn_radius_m` | float | 米 | 0（0 = 不约束） |
| `endurance` | `max_flight_time_s` | float | 秒 | 1800 |
| | `max_range_m` | float | 米 | 15000 |
| `operational` | `min_altitude_m` | float | 米 | 10 |
| | `max_altitude_m` | float | 米 | 300 |
| `safety` | `safety_margin_m` | float | 米 | 5 |

> **`operational.min_altitude_m` / `max_altitude_m` 是硬约束**：航点高度超出
> 升限会导致该任务规划失败（`终点高度 220.0m 超过飞行器最高高度 200.0m`）。
> 这是数据配置问题，不是算法缺陷。

### 2.9 `Route` — 待校验航线（仅 `validate_route` 输入用）

| 字段 | 类型 | 必填 | 默认 | 说明 |
|------|------|------|------|------|
| `route_id` | string | | `"external-route"` | 航线 ID |
| `task_id` | string | | `"external-task"` | 任务 ID |
| `aircraft_id` | string | | `"external-uav"` | 飞行器 ID |
| `status` | string | | `"planned"` | `planned` / `failed` / `partial` |
| `total_distance` | float | | 0 | 自报里程（米），用于与轨迹实际长度对照 |
| `total_time` | float | | 0 | 自报时长（秒） |
| `trajectory` | array | ✅ | — | 轨迹点序列（`TrajectoryPoint`） |

---

## 3. Python SDK 函数

安装（三选一，详见 [`API_QUICKSTART.md`](API_QUICKSTART.md#35-python-调用)）：

```bash
pip install -e ".[server]"                    # 从源码（仓库根目录）
pip install uav_path_planning-1.0.0-*.whl     # 从 wheel
export PYTHONPATH=/path/to/repo:$PYTHONPATH   # 不安装，临时加路径
```

```python
import uav_planner  # 或 from uav_planner import plan_single, plan_batch, ...
```

可选 extras：`[server]`（REST 服务）、`[viz]`（可视化）、`[test]`（测试）、`[all]`（全部）。

### 3.1 `plan_single(start, goal, *, waypoints=None, ...)` → `dict`

单条航线的完整规划：时空 A\* → 多目标优化 → 合法性验证。

**起终点二选一**（同时给出会抛 `INVALID_INPUT`）：
- `start` + `goal`：两点直飞；
- `waypoints`：完整航点序列。**有中间经停点时必须用这种方式传**，只给起终点
  会丢掉经停，规划出一条更短但不合要求的航线，且从返回结果上看不出来。

| 参数 | 类型 | 必填 | 默认 | 说明 |
|------|------|------|------|------|
| `start` | `Point` \| `[x,y,z]` | 二选一 | — | 起点 |
| `goal` | `Point` \| `[x,y,z]` | 二选一 | — | 终点 |
| `waypoints` | `list[Waypoint]` | 二选一 | — | 完整航点序列，与上两者互斥 |
| `config` | `SpaceTimeConfig` | | 默认网格 | 时空网格配置 |
| `static_obstacles` | `list[StaticObstacle]` | | `[]` | 静态障碍物 |
| `dynamic_obstacles` | `list[DynamicObstacle]` | | `[]` | 动态障碍物 |
| `aircraft` | `Aircraft` | | 默认包线 | 飞行器性能 |
| `start_time` | float | | `0.0` | 起飞时刻（秒） |
| `priority` | int | | `5` | 调度优先级 1-10 |
| `objective_weights` | object | | 默认权重 | 多目标权重，和须为 1.0 |
| `optimize` | bool | | `True` | 是否做多目标航线优化 |
| `cost_grid` | bool | | `True` | 是否启用通行代价层 |
| `clearance` | float | | `200.0` | 净空膨胀半径（米） |
| `clearance_cost` | float | | `3.0` | 净空区通行代价倍率 |
| `max_iterations` | int | | `2000000` | A\* 最大迭代次数 |
| `validate` | bool | | `True` | 是否附带合法性验证结果 |
| `task_id` | string | | `"task-001"` | 任务标识 |

**返回**：见 [§4.1](#41-单任务返回plan_single)。

### 3.2 `plan_batch(tasks, *, aircraft, ...)` → `dict`

多任务批量并行规划，含多机冲突避免。

| 参数 | 类型 | 必填 | 默认 | 说明 |
|------|------|------|------|------|
| `tasks` | `list[Task]` | ✅ | — | 任务列表，不能为空 |
| `aircraft` | `list[Aircraft]` \| `{id: Aircraft}` | ✅ | — | 飞行器集合 |
| `config` | `SpaceTimeConfig` | | 默认网格 | 时空网格配置 |
| `static_obstacles` / `dynamic_obstacles` | array | | `[]` | 障碍物 |
| `max_workers` | int | | `4` | 并行线程数 |
| `dispatch` | bool | | `False` | 调度模式：跳过 risk/heading 计算（约省 8%） |
| `conflict_avoidance` | bool | | `True` | 多机冲突避免 |
| `min_separation` | float | | `10.0` | 多机最小安全间隔（米） |
| `cost_grid` / `clearance` / `clearance_cost` | | | `True` / `200.0` / `3.0` | 通行代价层 |
| `max_iterations` | int | | `200000` | 单任务 A\* 最大迭代次数 |
| `request_id` | string | | 自动 | 请求标识 |
| `validate_tasks` | bool | | `False` | 是否先剔除飞行器需求不兼容的任务 |

**`validate_tasks` 该选哪个**：

- `False`（默认）：所有任务都尝试规划，不兼容的任务以 `status="failed"` 返回并
  在 `message` 里说明原因。**不丢数据**，接口调用推荐。
- `True`：预先剔除不兼容任务，被剔除的记入 `warnings`。与 `plan_multi.py`
  命令行行为一致，`plan_from_dir` 默认开启。

**返回**：见 [§4.2](#42-批量返回plan_batch)。

> **性能提示**：实测 10000 个任务约需 **437 秒**。批量超过约 500 个任务时，
> 请走 HTTP 异步作业接口 `POST /api/v1/jobs`，否则客户端会超时。

### 3.3 `validate_route(route, *, aircraft=None, ...)` → `dict`

校验一条**已有**航线的合法性。航线可以来自外部系统，不限于本算法产出——
这是联调时验证对方航线的主要手段。

| 参数 | 类型 | 必填 | 默认 | 说明 |
|------|------|------|------|------|
| `route` | `Route` | ✅ | — | 待校验航线，至少含 `trajectory` |
| `aircraft` | `Aircraft` | | null | 提供则检查速度 / 爬升 / 高度约束 |
| `static_obstacles` / `dynamic_obstacles` | array | | `[]` | 障碍物 |
| `min_separation` | float | | `10.0` | 多机最小间隔（米） |

**返回**：见 [§4.4](#44-验证结果)。

### 3.4 `plan_from_dir(input_dir, *, task_id=None, limit=None, mode="batch", **options)` → `dict`

从数据目录加载并规划——把既有命令行流程一键接口化，读同一批文件、走同一条代码路径。

| 参数 | 类型 | 必填 | 默认 | 说明 |
|------|------|------|------|------|
| `input_dir` | string | ✅ | — | 含 `config.json` / `*_obstacles.json` / `aircraft_config.json` / `tasks.json` |
| `task_id` | string | | null | 只规划指定任务；与 `limit` 互斥 |
| `limit` | int | | null | 只取前 N 个任务（联调时快速验证用） |
| `mode` | string | | `"batch"` | `"batch"`（多机批量）或 `"single"`（优先级最高的单任务） |
| `**options` | | | | 透传给 `plan_single` / `plan_batch` |

**与 CLI 的一致性**：`mode="single"` 时按「业务优先级 → 调度优先级」选最高任务，
传**完整航点序列**，结果与 `python plan_single.py` 逐位一致；`mode="batch"` 时
默认 `validate_tasks=True`，与 `python plan_multi.py` 一致。
额外返回 `data_quality` 字段（多源融合的数据质量报告）。

### 3.5 `get_version()` → `dict`

| 字段 | 类型 | 说明 |
|------|------|------|
| `api_version` | string | 接口版本 |
| `algorithms` | object | 封装的算法组件说明 |
| `defaults.config` | object | 默认时空网格配置 |
| `defaults.grid_cells` | int | 默认网格总格数（3000000） |

---

## 4. 返回结构

### 4.1 单任务返回（`plan_single`）

| 字段 | 类型 | 说明 |
|------|------|------|
| `status` | string | `planned` / `failed` |
| `request_id` | string | 请求标识，形如 `single-{task_id}` |
| `task_id` | string | 任务标识 |
| `aircraft_id` | string | 飞行器标识 |
| `algorithm` | string | 实际使用的算法 |
| `elapsed_ms` | float | 端到端耗时（毫秒，含验证） |
| `route` | object | 航线详情，见 [§4.3](#43-航线对象) |
| `route_count` | int | 中间结果条数（A\* 原始 + 优化后） |
| `validation` | object \| null | 验证结果；`validate=false` 或规划失败时为 `null` |
| `warnings` | array[string] | 非致命告警（如飞行器与任务需求不匹配） |

> **`status="failed"` 不是异常**。规划不出路径是路径规划的常态结果，
> 接口正常返回 200，失败原因在 `route.message` 里。只有**调用参数有问题**
> 才抛异常 / 返回 4xx。

### 4.2 批量返回（`plan_batch`）

| 字段 | 类型 | 说明 |
|------|------|------|
| `status` | string | `planned`（至少 1 条成功）/ `failed`（全失败） |
| `request_id` | string | 请求标识 |
| `total_routes` | int | 航线总数 |
| `success_count` / `failed_count` | int | 成功 / 失败计数 |
| `total_planning_time_ms` | float | A\* 累计耗时（毫秒） |
| `elapsed_ms` | float | 端到端墙钟耗时（毫秒） |
| `routes` | array[object] | 航线列表，每项见 [§4.3](#43-航线对象) |
| `conflicts` | array[object] | 残余冲突列表，见下 |
| `conflict_count` | int | 残余冲突数 |
| `warnings` | array[string] | `validate_tasks=true` 时被剔除任务的说明 |

**`conflicts[]` 元素**：

| 字段 | 类型 | 说明 |
|------|------|------|
| `route_a` / `route_b` | string | 冲突的两条航线 ID |
| `time` | float | 冲突时刻（秒） |
| `position_a` / `position_b` | [float, float, float] | 双方位置 |
| `distance` | float | 间距（米） |

> `conflict_avoidance=false` 时 `conflicts` **仍会如实报告**检测到的冲突——
> 不会把「没检查」包装成「无冲突」。

### 4.3 航线对象

| 字段 | 类型 | 说明 |
|------|------|------|
| `route_id` | string | 航线 ID |
| `task_id` | string | 任务 ID |
| `aircraft_id` | string | 飞行器 ID |
| `status` | string | `planned` / `failed` / `partial` |
| `message` | string | 状态说明；失败时为原因，成功时含规划解释摘要 |
| `algorithm` | string | 算法名 |
| `planning_time_ms` | float | 该航线规划耗时（毫秒） |
| `metrics` | object | 指标，见下 |
| `trajectory` | array[`TrajectoryPoint`] | 轨迹点序列 |
| `explanation` | object \| null | 结构化规划解释，见下 |

**`metrics`**：

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| `total_distance_m` | float | 米 | 总航程 |
| `total_time_s` | float | 秒 | 总飞行时间 |
| `total_energy` | float | 相对值 | 估算能耗 |
| `max_risk` | float | 0-1 | 最大风险值 |
| `avg_risk` | float | 0-1 | 平均风险值 |
| `min_obstacle_distance_m` | float | 米 | 距最近障碍物距离；无障碍物时为 999.0 |

**`explanation`**（任务书 §六 解释模块）：

| 字段 | 类型 | 说明 |
|------|------|------|
| `summary` | string | 一句话总结 |
| `score` | float | 多目标综合得分（**越低越优**） |
| `objective_breakdown` | object | 各目标贡献度 `{distance, time, risk, energy}` |
| `avoidance_count` | int | 避障次数 |
| `performance_margin` | object | 性能裕度 `{speed_ratio, climb_ratio, descent_ratio, turn_rate_ratio}`，越接近 1 越贴近极限 |
| `notes` | array[string] | 补充说明 |

### 4.4 验证结果

| 字段 | 类型 | 说明 |
|------|------|------|
| `is_valid` | bool | 是否合法（存在 ERROR 级问题即为 `false`） |
| `summary_text` | string | 人类可读摘要，如 `✓ 合法：共 0 个问题（0 错误, 0 警告, 0 提示）` |
| `summary` | object | 结构化统计，见下 |
| `issue_count` | int | 问题总数 |
| `issues` | array[object] | 问题列表，见下 |
| `route_id` | string | 被校验的航线 ID（仅 `validate_route` 返回） |
| `elapsed_ms` | float | 校验耗时（仅 `validate_route` 返回） |

**`summary`**：`trajectory_points`（轨迹点数）、`total_distance_m`、`total_time_s`、
`n_errors`、`n_warnings`。

**`issues[]`**：

| 字段 | 类型 | 说明 |
|------|------|------|
| `severity` | string | `info` / `warning` / `error` |
| `code` | string | 问题代码，见下表 |
| `message` | string | 中文描述 |
| `trajectory_index` | int | 相关轨迹点索引（-1 = 整体） |
| `time` | float | 相关时刻（秒） |
| `position` | [float, float, float] | 相关位置 |

**问题代码**：

| `code` | 级别 | 含义 |
|--------|------|------|
| `EMPTY_TRAJECTORY` | error | 轨迹为空或点数不足 2 |
| `STATIC_OBSTACLE_COLLISION` | error | 轨迹点落在静态障碍物内 |
| `STATIC_OBSTACLE_PROXIMITY` | warning | 距静态障碍物小于安全裕度（5 m） |
| `DYNAMIC_OBSTACLE_COLLISION` | error | 进入动态障碍物安全半径 |
| `DYNAMIC_OBSTACLE_PROXIMITY` | warning | 距动态障碍物小于安全半径 + 裕度 |
| `SEGMENT_TOO_LONG` | error | 相邻轨迹点间距超过 1000 m（疑似跳变） |
| `ALTITUDE_TOO_LOW` / `ALTITUDE_TOO_HIGH` | error | 超出飞行器高度范围 |
| `SPEED_EXCEEDED` | error | 水平速度超限 |
| `CLIMB_RATE_EXCEEDED` / `DESCENT_RATE_EXCEEDED` | error | 爬升 / 下降率超限 |
| `ACCELERATION_EXCEEDED` | error | 加速度超限 |
| `TURN_RATE_EXCEEDED` | error | 转弯率超限 |
| `NON_MONOTONIC_TIME` | warning | 时间倒退 |
| `MULTI_UAV_COLLISION` | error | 多机间距 < 最小间隔的一半 |
| `MULTI_UAV_PROXIMITY` | warning | 多机间距 < 最小间隔 |
| `GRID_OCCUPIED` | error | 轨迹点落在时空网格占用格 |

---

## 5. 错误码

**错误信封**（HTTP 与 SDK 一致）：

```json
{
  "error": {
    "code": "INVALID_INPUT",
    "message": "config.dx 必须大于 0，实际 0",
    "detail": {"field": "dx", "got": 0}
  }
}
```

| `code` | HTTP | 含义 | 触发示例 |
|--------|------|------|----------|
| `INVALID_INPUT` | 400 | 入参不合法 | 缺 `start`/`goal`；`dx=0`；点缺 `z`；权重和不为 1 |
| `TASK_NOT_FOUND` | 404 | 找不到任务 / 作业 | `task_id` 不存在；作业 ID 不存在 |
| `AIRCRAFT_NOT_FOUND` | 404 | 任务引用的飞行器未定义 | `aircraft_id: "a2"` 不在 `aircraft` 里 |
| `PLANNING_FAILED` | 422 | 请求合法但无法完成规划 | 所有任务都不兼容，无可规划任务 |
| `TIMEOUT` | 504 | 超过时间预算 | — |
| `INTERNAL_ERROR` | 500 | 未预期的内部异常 | 服务内部错误 |

SDK 侧抛 `uav_planner.PlanningError`，带 `.code` / `.message` / `.detail` 属性
和 `.to_dict()` 方法：

```python
from uav_planner import plan_single, PlanningError, ErrorCode

try:
    plan_single()
except PlanningError as e:
    if e.code == ErrorCode.INVALID_INPUT:
        print(e.detail)   # {'start_given': False, 'goal_given': False}
```

---

## 6. HTTP 接口

服务启动：`python serve.py --host 0.0.0.0 --port 8000`（详见 `API_QUICKSTART.md`）

| 方法 | 路径 | 说明 |
|------|------|------|
| GET | `/api/v1/health` | 健康检查 |
| GET | `/api/v1/health/jobs` | 异步作业队列统计 |
| GET | `/api/v1/version` | 版本与默认参数 |
| POST | `/api/v1/plan/single` | 单任务规划（同步），对应 `plan_single` |
| POST | `/api/v1/plan/batch` | 批量规划（同步），对应 `plan_batch` |
| POST | `/api/v1/validate` | 航线校验，对应 `validate_route` |
| POST | `/api/v1/jobs` | 提交异步批量作业，返回 **202** + `job_id` |
| GET | `/api/v1/jobs/{job_id}` | 查询作业状态与结果 |
| GET | `/api/v1/examples/single` | 示例请求体（单任务） |
| GET | `/api/v1/examples/batch` | 示例请求体（批量） |
| GET | `/docs` | Swagger 交互文档（**联调首选**） |
| GET | `/openapi.json` | OpenAPI 3 规范，可导入 Postman / Apifox |

**请求体字段与 SDK 参数逐字一致**。例如 `POST /api/v1/plan/single` 的请求体
就是 `plan_single()` 的关键字参数：

```json
{
  "start": {"x": 500, "y": 500, "z": 100},
  "goal": {"x": 2500, "y": 2500, "z": 150},
  "optimize": true,
  "validate": true
}
```

**异步作业**：`POST /api/v1/jobs` 请求体与 `/plan/batch` 完全相同，响应：

```json
{"job_id": "5338933eafb54cf9", "status": "pending", "poll_url": "/api/v1/jobs/5338933eafb54cf9"}
```

轮询 `GET /api/v1/jobs/{job_id}`：

```json
{
  "job_id": "5338933eafb54cf9",
  "status": "succeeded",
  "created_at": "2026-10-12T15:30:00",
  "started_at": "2026-10-12T15:30:00",
  "finished_at": "2026-10-12T15:30:22",
  "elapsed_ms": 22631.5,
  "result": { "...与 /plan/batch 响应相同..." },
  "error": null
}
```

`status` 取值：`pending` / `running` / `succeeded` / `failed`。
失败时 `error` 是标准错误信封。

> ⚠️ 作业表存在**服务进程内存**中：服务重启后历史作业不可查；多 worker 部署时
> 轮询可能落到没有该作业的进程。联调阶段够用，生产需换持久化存储。

> ⚠️ 本服务**没有任何认证**，且 CORS 默认放开。仅限内网联调使用，
> 切勿暴露到公网。

---

## 7. 内部变量

本章记录算法内部状态与常量。**调用方不需要设置这些**，但排查问题时
（「为什么这条航线绕远了」「为什么判定不合法」）需要知道它们的存在与取值。

### 7.1 `SpaceTimeConfig` — 网格维度由范围与分辨率推导

`nx / ny / nz / nt` 是**派生属性**，不可直接设置：

```
nx = ceil((x_max - x_min) / dx)      ny = ceil((y_max - y_min) / dy)
nz = ceil((z_max - z_min) / dz)      nt = ceil((t_max - t_min) / dt)
```

默认配置 → `50 × 50 × 10 × 120 = 3,000,000` 格。

坐标转换（**越界静默钳制到边界格**，不报错）：

| 方法 | 行为 |
|------|------|
| `to_grid(x, y, z)` | 世界坐标 → 网格索引，`np.clip` 到 `[0, n-1]` |
| `to_world(ix, iy, iz)` | 网格索引 → **格中心**世界坐标：`x_min + (ix + 0.5) * dx` |
| `time_to_step(t)` | 时间 → 时间步，钳制到 `[0, nt-1]` |
| `step_to_time(it)` | 时间步 → **步中心**时刻：`t_min + (it + 0.5) * dt` |

> 由于返回格中心，`to_world` 的结果不是格边界。一条从 `(0,0,0)` 出发的航线，
> 首个轨迹点的实际坐标是 `(50, 50, 25)`（默认分辨率下）。

### 7.2 `SpaceTimeGrid.occupancy` — 四维占用网格

| 属性 | 值 |
|------|-----|
| 类型 | `numpy.ndarray`，`dtype=bool` |
| 形状 | `(nx, ny, nz, nt)` |
| 默认规模 | `50 × 50 × 10 × 120` = 300 万格 ≈ **2.9 MB** |
| 含义 | `True` = 该时空格被占用，规划器不得进入 |

标记接口：

| 方法 | 说明 |
|------|------|
| `set_static_obstacle(x, y, z, radius)` | 球形障碍，**所有时间步**全部占用 |
| `set_static_box(x_min, x_max, y_min, y_max, z_min, z_max)` | 长方体障碍，**所有时间步**全部占用 |
| `set_dynamic_obstacle(trajectory, radius)` | 动态障碍，**仅在轨迹对应的时间步**占用 |
| `is_free(ix, iy, iz, it)` | 是否空闲；**越界一律视为占用** |
| `occupancy_count()` | 已占用格数 |

### 7.3 `SpaceTimeAStar` — 搜索器内部状态

| 变量 | 类型 | 说明 |
|------|------|------|
| `neighbors` | list | 邻域模板。默认 **6 邻域**（面连接）`[(±1,0,0),(0,±1,0),(0,0,±1)]`；`neighbor_mode="26"` 时为 26 邻域 |
| `wait_enabled` | bool | 是否允许「原地等待」动作（时间推进一步，代价 ×1.05） |
| `cost_grid` | `CostGrid` \| None | **通行代价层**，见 §7.4 |
| `min_altitude` / `max_altitude` | float \| None | 飞行器高度包线 |
| `max_climb_rate` / `max_descent_rate` | float \| None | 爬升 / 下降率上限，逐段校验 |
| `occupancy_mask` | ndarray \| None | **多机冲突避免**用的禁飞时空掩码，与其他已规划航线取并集；`None` 时退化为单机规划 |
| `_iz_min` / `_iz_max` | int | 网格 Z 范围与飞行器高度包线的**交集**，实际搜索的 z 边界 |
| `max_iterations` | int | 迭代上限，超出即返回 `failed` 并注明「未找到可行路径」 |

**搜索代价**：每次移动消耗 `dt`（时间成本）+ `spatial_dist × 0.001`（距离惩罚，
用于打破平局、偏好短路径）。等待动作代价为 `dt × 1.05`。

**启发函数**：曼哈顿网格距离 × `dt`（可采纳，保证 A\* 最优性）。

**为什么 6 邻域而非 26**：分支因子从 26 降到 6，搜索空间小得多；代价是路径
呈阶梯状，两点「直飞」的曼哈顿距离等于 `|Δx| + |Δy| + |Δz|` 而非欧氏距离。
例如 (0,0)→(3500,3500) 直飞约 7000 m 而非 4949 m。**这是正常行为，不是绕远**。

### 7.4 `CostGrid` — 通行代价层

与占用网格**正交**：`occupancy` 决定「能不能走」，`cost_grid` 决定「更费力」。

| 属性 | 说明 |
|------|------|
| `lookup_array()` | 返回代价查询数组 |
| `min_cost` | 全网格最小代价，用于启发函数的可采纳性修正（返回 `max(1e-6, min_cost)`） |
| `default_cost` | 基础代价（默认 1.0） |

构建方式（`build_clearance_cost_grid`）：把每个静态障碍物包围盒向外膨胀
`clearance` 米，膨胀区内代价抬高到 `clearance_cost` 倍（默认 200 m / 3.0 倍）。
效果是规划器天然偏好远离建筑群的航路，与 risk 目标方向一致。
`skip_types=("terrain",)` 跳过地形（地形本身就是地面，不需要净空）。

### 7.5 `AircraftPerformance` — 五个子规格

| 分组 | 类 | 关键内容 |
|------|-----|----------|
| `physical` | `PhysicalSpec` | 自重、最大起飞重量、载荷上限、尺寸 |
| `performance` | `PerformanceSpec` | 最大速度、爬升 / 下降率、转弯率、加速度、最小转弯半径 |
| `endurance` | `EnduranceSpec` | 最大飞行时间、最大航程、电池容量、巡航 / 悬停功率 |
| `operational` | `OperationalSpec` | **最低 / 最高飞行高度**（硬约束）、工作温度、抗风 |
| `safety` | `SafetySpec` | 旋翼半径、安全裕度、地理围栏、返航高度 |

顶层保留了扁平字段（`max_speed` / `max_climb_rate` / …）作为向后兼容的
fallback；取值一律通过 `get_max_speed()` / `get_max_altitude()` 等 getter，
它们优先读子规格、为空时回退扁平字段。

### 7.6 `ParallelPlanner` — 共享网格

| 变量 | 说明 |
|------|------|
| `_shared_grid` | **所有任务共用一份** `SpaceTimeGrid`。关键优化：原本每个任务各建一份 3 MB 网格，10000 个任务要分配 30 GB；共享后只分配一次 |
| `_shared_cost_grid` | 共享的通行代价层（只读，可跨线程复用） |
| `_planner` | 共享的单进程 A\* 实例（线程模式与顺序回退用） |
| `_shared_grid` 填充 | 静态障碍写入全部时间步；动态障碍只写轨迹覆盖的时间步 |

**注意**：共享网格是**只读**的。多机冲突避免不通过修改共享网格实现，
而是给每个任务传入各自的 `occupancy_mask`，避免线程间互相污染。

### 7.7 `PathValidator` — 阈值常量

类属性，可通过继承覆盖：

| 常量 | 默认值 | 单位 | 含义 |
|------|--------|------|------|
| `OBSTACLE_SAFETY_MARGIN` | 5.0 | 米 | 距障碍物小于此值 → warning |
| `APPROACH_WARNING_MARGIN` | 10.0 | 米 | 接近警告距离 |
| `MULTI_UAV_MIN_SEPARATION` | 10.0 | 米 | 多机最小间隔；小于其**一半**升级为 error |
| `MULTI_UAV_TIME_WINDOW` | 5.0 | 秒 | 多机冲突检查的时间对齐窗口 |
| `MAX_SEGMENT_LENGTH` | 1000.0 | 米 | 单段最大允许距离，超过判为跳变 |

### 7.8 `PlannedRoute` — 指标字段

| 字段 | 推算方式 |
|------|----------|
| `total_distance` | 相邻轨迹点欧氏距离累加 |
| `total_time` | `trajectory[-1].t - trajectory[0].t` |
| `total_energy` | 有飞行器参数时按速度 / 高度变化估算；否则退化为 `total_distance × 1.0` |
| `min_obstacle_distance` | 所有轨迹点到最近障碍物（AABB 距离）的最小值；无障碍物时为 `999.0` |
| `max_risk` / `avg_risk` | 由 `min_obstacle_distance` 映射：`clip(1 - d/50, 0, 1)`，即 50 m 内线性下降到 1 |

---

## 8. 单位与约定

| 量 | 单位 | 备注 |
|----|------|------|
| 距离 | 米 (m) | |
| 速度 | 米/秒 (m/s) | |
| 加速度 | 米/秒² (m/s²) | |
| 时间 | 秒 (s) | |
| 角度 | 度 (°) | 航向角以 X 轴正方向为 0，逆时针为正 |
| 重量 | 千克 (kg) | |
| 温度 | 摄氏度 (℃) | |
| 功率 | 瓦 (W) | |
| 电量 | 瓦时 (Wh) | |
| 风险值 | 无量纲 0-1 | 越大越危险 |

**坐标系**：右手系，X 向东、Y 向北、Z 向上（Z=0 为地面）。

**时间原点**：由 `t_min` 决定（默认 0），所有时间戳为相对该原点的秒数。

---

## 附：相关文档

| 文档 | 内容 |
|------|------|
| [`API_QUICKSTART.md`](API_QUICKSTART.md) | 联调对接步骤、可直接执行的命令、常见问题 |
| [`../README.md`](../README.md) | 项目总览、算法说明、命令行用法 |
| [`deprecated/algorithm_design.md`](deprecated/algorithm_design.md) | 算法原理（**仍然有效**，仅 I/O 部分失效） |
| `/docs`（服务端点） | Swagger 交互文档 |

---

## 附：文档一致性

本文档中的字段表、错误码映射与内部常量**不是手抄的**，而是与实现逐项核对过。
核对程序覆盖以下条目（共 124 项，全部通过）：

| 核对项 | 对应章节 |
|--------|----------|
| `plan_single` / `plan_batch` / 航线对象 / 验证结果的返回字段（含「实际无文档外字段」的反向检查） | §4 |
| 六个错误码常量存在，且 HTTP 状态码映射与 `server/app.py::_ERROR_STATUS` 一致 | §5 |
| 网格维度 `50×50×10×120`、`occupancy` 形状 `(50,50,10,120)` / dtype `bool` / 实际占用 2.86 MB | §7.1、§7.2 |
| `PathValidator` 五个阈值常量的实际取值 | §7.7 |
| `SpaceTimeAStar.__init__` 的 12 个参数名、6 邻域定义 | §7.3 |
| `AircraftPerformance` 五个子规格属性 | §7.5 |
| 无障碍时 `min_obstacle_distance_m == 999.0` | §7.8 |
| 文档中提到的每个 HTTP 端点确实存在 | §6 |

**修改接口时请同步更新本文档**，并重新跑一遍核对，避免文档与实现脱节。
