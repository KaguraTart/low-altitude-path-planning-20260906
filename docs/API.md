# 低空无人机路径规划 API

本文是对接方唯一需要阅读的接口说明。`/api/v1` 保留原有局部米制 JSON 契约；新增的 `/api/v2` 使用 WGS84 经纬度 WKT、显式高度基准、版本化场景与 20 类规则。**下面 §0–§9 是 v1，§10 起是 v2。两个版本的参数和默认机型不同，不能混用。**

运行服务的 `/openapi.json` 由 FastAPI 请求/响应模型生成；`/docs` 是读取该规范的 Swagger UI。Markdown 不参与生成 Swagger。仓库 `docs/openapi.json` 是同一应用的规范快照，发布时与代码、本文一起进入同一镜像。

## 0. 项目简介

本项目提供低空无人机路径规划的算法接口，覆盖 **多维时空大规模并行规划** 与
**智能飞行线路规划** 两大能力。

核心能力是把一段低空空域（默认 5000m × 5000m × 500m、时间窗 1200 秒）建模成
**四维时空网格** `(x, y, z, t)`，在其中有约束地搜索无人机航线，并给出：

- **可行航线**：避开静态建筑 / 塔杆、绕开动态障碍物（其他飞行器、鸟群）的安全半径；
- **多机协同**：批量规划时保证任意两条航线满足最小安全间隔，冲突在规划闭环内消化；
- **多目标权衡**：在航程、时间、风险、能耗四个目标间按权重折中；
- **合法性验证**：对航线做 7 个维度的检查（连续性、障碍、性能、高度、时空占用等）；
- **规划解释**：说明这条航线为什么这样走、避障几次、性能裕度还剩多少。

---

## 目录

0. [项目简介](#0-项目简介)
1. [接口总览](#1-接口总览)
2. [输入数据结构](#2-输入数据结构)
3. [返回结构](#3-返回结构)
4. [错误码](#4-错误码)
5. [HTTP 接口](#5-http-接口)
6. [快速上手](#6-快速上手)
7. [常见问题](#7-常见问题)
8. [单位与约定](#8-单位与约定)
9. [内部变量](#9-内部变量)
10. [v2 接入与通用约定](#10-v2-接入与通用约定)
11. [v2 场景与输入字段](#11-v2-场景与输入字段)
12. [v2 全部规则](#12-v2-全部规则)
13. [v2 返回与错误](#13-v2-返回与错误)
14. [v2 完整调用示例](#14-v2-完整调用示例)
15. [v1 迁移到 v2](#15-v1-迁移到-v2)

---

## 1. 接口总览

算法以 **HTTP 服务** 的形式对外提供，请求与响应均为纯 JSON。

```
┌──────────────────────────────────────────────────────────┐
│  REST HTTP 服务（端口 8000）                              │
│     POST /api/v1/plan/single      POST /api/v1/plan/batch │
│     POST /api/v1/validate         POST /api/v1/jobs       │
│     GET  /docs  ← Swagger 交互文档                        │
└───────────────────────────┬──────────────────────────────┘
                            │
┌───────────────────────────▼──────────────────────────────┐
│  算法内核                                                  │
│     时空 A* · 多目标优化 · 并行规划 · 冲突避免 · 路径验证  │
└──────────────────────────────────────────────────────────┘
```

**两条核心设计约定**：

1. **请求与响应都是纯 JSON**，不含 `NaN` / `Infinity` 等非法字面量，
   任何语言的 JSON 库都能直接解析。
2. **字段名全程一致**：请求体字段、返回结构字段、错误码字段在各端点间保持同名，
   你只需记住一套名字。

> 打开 `http://<服务地址>:8000/docs` 可以直接在网页上构造请求、查看真实响应，
> 不必先写代码。


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

### 2.9 `Route` — 待校验航线（`POST /api/v1/validate` 的输入）

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

## 3. 返回结构

### 3.1 单任务返回

| 字段 | 类型 | 说明 |
|------|------|------|
| `status` | string | `planned` / `failed` |
| `request_id` | string | 请求标识，形如 `single-{task_id}` |
| `task_id` | string | 任务标识 |
| `aircraft_id` | string | 飞行器标识 |
| `algorithm` | string | 实际使用的算法 |
| `elapsed_ms` | float | 端到端耗时（毫秒，含验证） |
| `route` | object | 航线详情，见 [§3.3](#33-航线对象) |
| `route_count` | int | 中间结果条数（A\* 原始 + 优化后） |
| `validation` | object \| null | 验证结果；`validate=false` 或规划失败时为 `null` |
| `warnings` | array[string] | 非致命告警（如飞行器与任务需求不匹配） |

> **`status="failed"` 不是异常**。规划不出路径是路径规划的常态结果，
> 接口正常返回 200，失败原因在 `route.message` 里。只有**调用参数有问题**
> 才抛异常 / 返回 4xx。

### 3.2 批量返回

| 字段 | 类型 | 说明 |
|------|------|------|
| `status` | string | `planned`（至少 1 条成功）/ `failed`（全失败） |
| `request_id` | string | 请求标识 |
| `total_routes` | int | 航线总数 |
| `success_count` / `failed_count` | int | 成功 / 失败计数 |
| `total_planning_time_ms` | float | A\* 累计耗时（毫秒） |
| `elapsed_ms` | float | 端到端墙钟耗时（毫秒） |
| `routes` | array[object] | 航线列表，每项见 [§3.3](#33-航线对象) |
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

### 3.3 航线对象

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

**`explanation`**（结构化规划解释）：

| 字段 | 类型 | 说明 |
|------|------|------|
| `summary` | string | 一句话总结 |
| `score` | float | 多目标综合得分（**越低越优**） |
| `objective_breakdown` | object | 各目标贡献度 `{distance, time, risk, energy}` |
| `avoidance_count` | int | 避障次数 |
| `performance_margin` | object | 性能裕度 `{speed_ratio, climb_ratio, descent_ratio, turn_rate_ratio}`，越接近 1 越贴近极限 |
| `notes` | array[string] | 补充说明 |

### 3.4 验证结果

| 字段 | 类型 | 说明 |
|------|------|------|
| `is_valid` | bool | 是否合法（存在 ERROR 级问题即为 `false`） |
| `summary_text` | string | 人类可读摘要，如 `✓ 合法：共 0 个问题（0 错误, 0 警告, 0 提示）` |
| `summary` | object | 结构化统计，见下 |
| `issue_count` | int | 问题总数 |
| `issues` | array[object] | 问题列表，见下 |
| `route_id` | string | 被校验的航线 ID（仅 `POST /api/v1/validate` 返回） |
| `elapsed_ms` | float | 校验耗时（仅 `POST /api/v1/validate` 返回） |

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

## 4. 错误码

**错误信封**（所有错误响应统一为此结构）：

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

**客户端处理建议**：按 `error.code` 分支，不要解析 `message` 文本——文本可能调整，
错误码是稳定契约。

```python
import requests

resp = requests.post(f"{API}/api/v1/plan/single", json=payload, timeout=60)
if resp.status_code >= 400:
    err = resp.json()["error"]
    if err["code"] == "INVALID_INPUT":
        print("参数有问题：", err["detail"])     # 定位到具体字段
    else:
        print("调用失败：", err["code"], err["message"])
```

---

## 5. HTTP 接口

服务启动：`python serve.py --host 0.0.0.0 --port 8000`（参数见 `python serve.py --help`）

### 5.1 端点一览

| 方法 | 路径 | 说明 |
|------|------|------|
| GET | `/api/v1/health` | 健康检查 |
| GET | `/api/v1/health/jobs` | 异步作业队列统计 |
| GET | `/api/v1/version` | 版本与默认参数 |
| POST | `/api/v1/plan/single` | 单任务规划（同步） |
| POST | `/api/v1/plan/batch` | 多机批量规划（同步） |
| POST | `/api/v1/validate` | 校验一条已有航线的合法性 |
| POST | `/api/v1/jobs` | 提交异步批量作业，返回 **202** + `job_id` |
| GET | `/api/v1/jobs/{job_id}` | 查询作业状态与结果 |
| GET | `/api/v1/examples/single` | 示例请求体（单任务） |
| GET | `/api/v1/examples/batch` | 示例请求体（批量） |
| GET | `/docs` | Swagger 交互文档（**联调首选**） |
| GET | `/openapi.json` | OpenAPI 3 规范，可导入 Postman / Apifox |

### 5.2 单任务规划

`POST /api/v1/plan/single`

单条航线的完整规划：时空 A\* → 多目标优化 → 合法性验证。

**起终点二选一**（同时给出会返回 `INVALID_INPUT`）：

- `start` + `goal`：两点直飞；
- `waypoints`：完整航点序列。**有中间经停点时必须用这种方式传**——只给起终点会
  丢掉经停，规划出一条更短但不合要求的航线，且从返回结果上看不出来。

**请求体字段**：

| 字段 | 类型 | 必填 | 默认 | 说明 |
|------|------|------|------|------|
| `start` | `Point` \| `[x,y,z]` | 二选一 | — | 起点 |
| `goal` | `Point` \| `[x,y,z]` | 二选一 | — | 终点 |
| `waypoints` | `array[Waypoint]` | 二选一 | — | 完整航点序列，与上两者互斥 |
| `config` | `SpaceTimeConfig` | | 默认网格 | 时空网格配置 |
| `static_obstacles` | `array[StaticObstacle]` | | `[]` | 静态障碍物 |
| `dynamic_obstacles` | `array[DynamicObstacle]` | | `[]` | 动态障碍物 |
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

**返回**：见 [§3.1](#31-单任务返回)。

### 5.3 批量规划

`POST /api/v1/plan/batch`

多任务批量并行规划，含多机冲突避免。

**请求体字段**：

| 字段 | 类型 | 必填 | 默认 | 说明 |
|------|------|------|------|------|
| `tasks` | `array[Task]` | ✅ | — | 任务列表，不能为空 |
| `aircraft` | `array[Aircraft]` \| `{id: Aircraft}` | ✅ | — | 飞行器集合 |
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

- `false`（默认）：所有任务都尝试规划，不兼容的任务以 `status="failed"` 返回并
  在 `message` 里说明原因。**不丢数据**。
- `true`：预先剔除不兼容任务，被剔除的记入 `warnings`。

**返回**：见 [§3.2](#32-批量返回)。

> **性能提示**：实测 10000 个任务约需 **437 秒**。批量超过约 500 个任务时，
> 请改用 [§5.5 异步作业](#55-异步作业)，否则客户端会超时。

### 5.4 航线校验

`POST /api/v1/validate`

校验一条**已有**航线的合法性。航线可以来自外部系统，不限于本算法产出——
这是验证外部航线是否合规的主要手段。

**请求体字段**：

| 字段 | 类型 | 必填 | 默认 | 说明 |
|------|------|------|------|------|
| `route` | `Route` | ✅ | — | 待校验航线，至少含 `trajectory` |
| `aircraft` | `Aircraft` | | null | 提供则检查速度 / 爬升 / 高度约束 |
| `static_obstacles` / `dynamic_obstacles` | array | | `[]` | 障碍物 |
| `min_separation` | float | | `10.0` | 多机最小间隔（米） |

**返回**：见 [§3.4](#34-验证结果)。

### 5.5 异步作业

`POST /api/v1/jobs` 的请求体与 [§5.3 批量规划](#53-批量规划)完全相同，只是改为后台执行。提交后立即返回（HTTP **202**）：

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

### 5.6 其他端点

| 方法 | 路径 | 说明 |
|------|------|------|
| `GET` | `/api/v1/health` | 健康检查 |
| `GET` | `/api/v1/version` | 版本、算法组件与默认参数 |
| `GET` | `/api/v1/health/jobs` | 异步作业队列统计（pending / running / succeeded / failed） |
| `GET` | `/api/v1/examples/single` | 单任务示例请求体（可直接 POST 回来） |
| `GET` | `/api/v1/examples/batch` | 批量示例请求体 |
| `GET` | `/docs` | Swagger 交互文档 |
| `GET` | `/openapi.json` | OpenAPI 3 规范，可导入 Postman / Apifox |

**`GET /api/v1/version` 返回**：

| 字段 | 类型 | 说明 |
|------|------|------|
| `api_version` | string | 接口版本 |
| `algorithms` | object | 封装的算法组件说明 |
| `defaults.config` | object | 默认时空网格配置 |
| `defaults.grid_cells` | int | 默认网格总格数（3000000） |


---

## 6. 快速上手

> ## ⚠️ 禁止在联调期间进行压测！！！
>
> 压测会长时间占满 CPU、挤掉正常请求，导致联调双方互相阻塞。
> 请务必等联调结束后，单独安排时间窗口进行。

### 6.1 三步跑起来

#### 第 1 步：装依赖并起服务

```bash
conda activate uav_path_planning
cd <项目根目录>
python serve.py --host 0.0.0.0 --port 8000
```

启动后会打印可用的访问地址，例如：

```
  本机访问:
    交互文档  http://127.0.0.1:8000/docs

  局域网访问:
    http://192.168.124.23:8000/docs
```

把「局域网访问」那一行的地址发给需要调用的人，同一网段即可直接访问。

> `--host 0.0.0.0` 表示同网段可访问。若只在本机用，改成 `127.0.0.1`。

#### 第 2 步：打开交互文档

浏览器打开：

```
http://<服务所在机器的IP>:8000/docs
```

这是自动生成的 Swagger 页面，**可以直接在网页上构造请求并看到真实响应**——
不用写一行代码就能确认字段。

#### 第 3 步：确认版本一致

```bash
curl http://<服务所在机器的IP>:8000/api/v1/version
```

返回 `api_version` 应对得上本文档顶部的版本号。

---

### 6.2 调用示例

以下每条都是**可直接执行的完整命令**（把 `HOST` 换成实际地址）。

#### 7.2.1 健康检查

```bash
curl -s http://HOST:8000/api/v1/health
```

#### 7.2.2 单任务规划——最快看到完整响应结构

先取示例请求体，再原样打回去：

```bash
curl -s http://HOST:8000/api/v1/examples/single -o ex_single.json
curl -s -X POST http://HOST:8000/api/v1/plan/single \
    -H 'Content-Type: application/json' \
    -d @ex_single.json | python -m json.tool
```

也可以直接手写一个最小请求：

```bash
curl -s -X POST http://HOST:8000/api/v1/plan/single \
    -H 'Content-Type: application/json' \
    -d '{
      "start": {"x": 500, "y": 500, "z": 100},
      "goal":  {"x": 2500, "y": 2500, "z": 150}
    }' | python -m json.tool
```

#### 7.2.3 多机批量规划

```bash
curl -s http://HOST:8000/api/v1/examples/batch -o ex_batch.json
curl -s -X POST http://HOST:8000/api/v1/plan/batch \
    -H 'Content-Type: application/json' \
    -d @ex_batch.json | python -m json.tool
```

#### 7.2.4 校验外部航线

校验一条**不是本算法产出**的航线——用来确认外部系统规划的航线是否合规：

```bash
curl -s -X POST http://HOST:8000/api/v1/validate \
    -H 'Content-Type: application/json' \
    -d '{
      "route": {
        "route_id": "external-route-001",
        "trajectory": [
          {"x": 0,   "y": 0, "z": 50, "t": 0},
          {"x": 100, "y": 0, "z": 50, "t": 10},
          {"x": 200, "y": 0, "z": 50, "t": 20}
        ]
      },
      "static_obstacles": [
        {"id": "b1", "name": "楼", "type": "building",
         "x": 500, "y": 0, "z": 0, "width": 80, "depth": 80, "height": 200}
      ]
    }' | python -m json.tool
```

返回 `is_valid` 与 `issues[]`（问题列表，含代码、级别、位置、时刻）。

#### 7.2.5 Python 调用

**方式 A：Python `requests`**

```python
import requests

API = "http://HOST:8000"

# 先取示例请求体，直接拿来用
payload = requests.get(f"{API}/api/v1/examples/single", timeout=30).json()

resp = requests.post(f"{API}/api/v1/plan/single", json=payload, timeout=60)
resp.raise_for_status()
data = resp.json()

print(data["status"])                        # planned
print(data["route"]["metrics"])              # 里程 / 时间 / 风险
print(data["validation"]["summary_text"])    # ✓ 合法：共 0 个问题...
```

**方式 B：Postman / Apifox**


导入 `http://HOST:8000/openapi.json`，全部接口与字段会自动生成，无需手写。

---

### 6.3 大批量任务：用异步作业

**同步的 `/plan/batch` 只适合中小批量**。实测 10000 个任务约需 437 秒，
远超任何 HTTP 客户端的默认超时，同步调用一定会被掐断。

超过约 500 个任务时改用异步作业：

```bash
# 提交，立即拿到 job_id（HTTP 202）
curl -s -X POST http://HOST:8000/api/v1/jobs \
    -H 'Content-Type: application/json' \
    -d @ex_batch.json | python -m json.tool

# 轮询（把 <job_id> 换成上一步返回的值）
curl -s http://HOST:8000/api/v1/jobs/<job_id> | python -m json.tool
```

`status` 流转：`pending` → `running` → `succeeded` / `failed`。

Python 轮询示例：

```python
import time, requests

job = requests.post("http://HOST:8000/api/v1/jobs", json=payload, timeout=30).json()
job_id = job["job_id"]

while True:
    body = requests.get(f"http://HOST:8000/api/v1/jobs/{job_id}", timeout=30).json()
    if body["status"] in ("succeeded", "failed"):
        break
    time.sleep(2)

if body["status"] == "failed":
    print("失败:", body["error"])
else:
    print("成功", body["result"]["success_count"], "条")
```

> ⚠️ 作业存在**服务进程内存**中：服务重启后查不到历史作业。
> 联调够用；正式对接如果要求作业持久化，需要另行说明。

---

## 7. 常见问题

### 7.1 连不上怎么办

按顺序排查：

| 现象 | 原因 | 处理 |
|------|------|------|
| `Connection refused` | 服务没起，或端口不对 | 看服务端终端有没有 `Uvicorn running on ...` |
| 本机能访问、其他机器不能 | 服务绑了 `127.0.0.1` | 改用 `--host 0.0.0.0` 重启 |
| 超时无响应 | 防火墙拦截 | macOS：系统设置 → 网络 → 防火墙，放行 Python；或临时关闭 |
| 能连但 IP 不对 | 有多张网卡（有线/无线/虚拟网卡） | 启动日志会列出**所有**局域网地址，逐个试 |
| 两台机器在不同网段 | 不在同一局域网 | 需内网穿透（ngrok / frp），或部署到同一内网服务器 |

**自查清单**：

```bash
# 服务端：确认服务在监听
lsof -i :8000

# 服务端：确认本机局域网 IP
ipconfig getifaddr en0        # macOS 无线
ipconfig getifaddr en1        # macOS 有线

# 调用方：先确认网络可达
ping <服务所在机器的IP>
```

---

### 7.2 常见问题

#### Q：返回的 `status` 是 `failed`，是不是接口坏了？

不是。**规划不出路径是路径规划的常态结果**，接口正常返回（HTTP 200），
失败原因在 `route.message` 里，例如：

```json
{"status": "failed", "route": {"message": "终点高度 220.0m 超过飞行器最高高度 200.0m"}}
```

只有**参数本身有问题**才会返回 4xx 并带 `error` 信封。

#### Q：报 400，怎么快速定位？

响应里的 `error.detail` 会指出具体字段：

```json
{"error": {"code": "INVALID_INPUT",
           "message": "config.dx 必须大于 0，实际 0",
           "detail": {"field": "dx", "got": 0}}}
```

形状类错误（类型不对）会列出所有不合格字段：

```json
{"error": {"code": "INVALID_INPUT", "message": "请求体校验失败",
           "detail": {"errors": [{"field": "start.z", "reason": "Input should be a valid number"}]}}}
```

#### Q：`width` / `depth` 到底填全长还是半长？

**半长**。`width` 是 X 方向**半宽**、`depth` 是 Y 方向**半深**。

一个 160m × 160m 的建筑：

```json
{"id": "b1", "type": "building", "x": 2500, "y": 2500, "z": 0,
 "width": 80, "depth": 80, "height": 200}
```

填错的最典型症状是障碍物只有实际的一半大，航线「贴着建筑飞过去」但校验通过。

#### Q：多点航线怎么传？传了中间点却没经过？

必须用 `waypoints` 传**完整航点序列**，不要只传 `start` / `goal`：

```json
{
  "waypoints": [
    {"id": "s", "x": 500,  "y": 500,  "z": 100, "type": "start"},
    {"id": "w", "x": 3000, "y": 3000, "z": 120, "type": "via"},
    {"id": "e", "x": 1000, "y": 3000, "z": 150, "type": "end"}
  ]
}
```

只传起终点会**丢掉中间经停**，规划出一条更短但不合要求的航线，
而且从里程数字上很难看出来。

#### Q：为什么两点之间走的是折线，不是直线？

A\* 使用 **6 邻域**（上下左右前后）移动，路径天然呈阶梯状。
两点间沿网格的曼哈顿距离通常大于欧氏距离——这是算法特性，不是绕远。
例如从 (0,0) 到 (3500,3500)，里程约 7000 m 而非欧氏距离 4949 m。

#### Q：算出来的距离比我预期长/短？

按影响从大到小排查：

1. **是否漏传了中间航点**（见上一条）；
2. **通行代价层**（`cost_grid`，默认开启）会引导航线远离建筑群，
   可能比最短路更长——这是设计意图，与 risk 目标一致；关掉可对比；
3. **飞行器升限**：`operational.max_altitude_m` 会限制可用高度层，
   可能迫使绕行；
4. **障碍物尺寸填错**（半宽/半深），导致可用空间被高估或低估。

#### Q：结果里 `min_obstacle_distance_m` 是 999.0？

表示场景里**没有任何障碍物**（或全部被跳过），不是异常值。

### 7.3 联调自查清单

对接完成前，建议逐项确认：

- [ ] `/api/v1/version` 的 `api_version` 与文档一致
- [ ] `/api/v1/examples/single` 的请求体能成功打回并拿到 `status: planned`
- [ ] 我方构造的真实任务（含真实障碍物尺寸）能规划成功
- [ ] 中间经停点确实被经过（对比轨迹与经停坐标）
- [ ] 我方自产航线经 `/api/v1/validate` 校验的 `is_valid` 符合预期
- [ ] 报错场景（缺字段、类型错）能正确收到 `error.code`
- [ ] 大批量走 `/api/v1/jobs` 能正常轮询到结果
- [ ] 确认批量规模与超时设置匹配（>500 任务必须走异步）

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

## 9. 内部变量

本章记录算法内部状态与常量。**调用方不需要设置这些**，但排查问题时
（「为什么这条航线绕远了」「为什么判定不合法」）需要知道它们的存在与取值。

### 9.1 `SpaceTimeConfig` — 网格维度由范围与分辨率推导

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

### 9.2 `SpaceTimeGrid.occupancy` — 四维占用网格

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

### 9.3 `SpaceTimeAStar` — 搜索器内部状态

| 变量 | 类型 | 说明 |
|------|------|------|
| `neighbors` | list | 邻域模板。默认 **6 邻域**（面连接）`[(±1,0,0),(0,±1,0),(0,0,±1)]`；`neighbor_mode="26"` 时为 26 邻域 |
| `wait_enabled` | bool | 是否允许「原地等待」动作（时间推进一步，代价 ×1.05） |
| `cost_grid` | `CostGrid` \| None | **通行代价层**，见 §9.4 |
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

### 9.4 `CostGrid` — 通行代价层

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

### 9.5 `AircraftPerformance` — 五个子规格

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

### 9.6 `ParallelPlanner` — 共享网格

| 变量 | 说明 |
|------|------|
| `_shared_grid` | **所有任务共用一份** `SpaceTimeGrid`。关键优化：原本每个任务各建一份 3 MB 网格，10000 个任务要分配 30 GB；共享后只分配一次 |
| `_shared_cost_grid` | 共享的通行代价层（只读，可跨线程复用） |
| `_planner` | 共享的单进程 A\* 实例（线程模式与顺序回退用） |
| `_shared_grid` 填充 | 静态障碍写入全部时间步；动态障碍只写轨迹覆盖的时间步 |

**注意**：共享网格是**只读**的。多机冲突避免不通过修改共享网格实现，
而是给每个任务传入各自的 `occupancy_mask`，避免线程间互相污染。

### 9.7 `PathValidator` — 阈值常量

类属性，可通过继承覆盖：

| 常量 | 默认值 | 单位 | 含义 |
|------|--------|------|------|
| `OBSTACLE_SAFETY_MARGIN` | 5.0 | 米 | 距障碍物小于此值 → warning |
| `APPROACH_WARNING_MARGIN` | 10.0 | 米 | 接近警告距离 |
| `MULTI_UAV_MIN_SEPARATION` | 10.0 | 米 | 多机最小间隔；小于其**一半**升级为 error |
| `MULTI_UAV_TIME_WINDOW` | 5.0 | 秒 | 多机冲突检查的时间对齐窗口 |
| `MAX_SEGMENT_LENGTH` | 1000.0 | 米 | 单段最大允许距离，超过判为跳变 |

### 9.8 `PlannedRoute` — 指标字段

| 字段 | 推算方式 |
|------|----------|
| `total_distance` | 相邻轨迹点欧氏距离累加 |
| `total_time` | `trajectory[-1].t - trajectory[0].t` |
| `total_energy` | 有飞行器参数时按速度 / 高度变化估算；否则退化为 `total_distance × 1.0` |
| `min_obstacle_distance` | 所有轨迹点到最近障碍物（AABB 距离）的最小值；无障碍物时为 `999.0` |
| `max_risk` / `avg_risk` | 由 `min_obstacle_distance` 映射：`clip(1 - d/50, 0, 1)`，即 50 m 内线性下降到 1 |

---

## 10. v2 接入与通用约定

### 10.1 端点

| 方法 | 路径 | 请求 / 返回 |
|------|------|-------------|
| GET | `/api/v2/health` | 原生内核是否可用、API 版本、作业计数 |
| GET | `/api/v2/health/jobs` | `pending/running/succeeded/failed/total` 计数 |
| GET | `/api/v2/version` | API 版本、C++ 内核信息、默认机型完整参数 |
| GET | `/api/v2/capabilities` | 路径模式、20 类规则、高度基准、几何容差、网格格式与场景生命周期 |
| POST | `/api/v2/scenes` | `Scene` → 场景 ID、版本和编译摘要 |
| GET | `/api/v2/scenes/{scene_id}` | 已注册场景的编译摘要；不返回原始场景 JSON |
| POST | `/api/v2/coordinates/convert` | 批量 WGS84 ↔ 局部 AEQD WKT 转换 |
| POST | `/api/v2/plan/single` | 场景选择 + `task` → `RouteResult` |
| POST | `/api/v2/plan/batch` | 场景选择 + `tasks` → `BatchResult` |
| POST | `/api/v2/validate` | 场景、任务与带时间航线 → 违规项和规则报告 |
| POST | `/api/v2/jobs` | 与批量请求相同 → HTTP 202、`job_id`、`poll_url` |
| GET | `/api/v2/jobs/{job_id}` | 作业状态及完整批量结果 |
| GET | `/api/v2/examples/single` | 可直接提交的完整单任务请求 |
| GET | `/api/v2/examples/batch` | 可直接提交的完整双任务请求 |

所有 v2 对象拒绝未知字段，数值必须是有限 JSON number；字符串数字、布尔值充当数值、`NaN` 和 `Infinity` 均不接受。时间字段使用带时区的 ISO 8601，如 `2026-10-16T09:00:00+08:00`。WKT 只放几何，属性、时间和规则放在 JSON 字段；不支持把时间藏在 WKT M 坐标中。

### 10.2 坐标、高度与时间

外部几何使用 WGS84，轴序固定为 **经度、纬度、高度**。任务起终点和业务航点必须是 `POINT Z`，航路中心线必须是 `LINESTRING Z`。体积边界为 `POLYGON` 或 `MULTIPOLYGON` 的平面足迹，垂直范围由单独字段确定；足迹 WKT 中的 Z 不改变体积上下限。空几何、非有限坐标、自相交等无效拓扑和 `GEOMETRYCOLLECTION` 会被拒绝。

每个场景以 `coordinates.origin_lon/origin_lat` 建立局部 AEQD 投影：`x` 向东、`y` 向北，单位米；`z = MSL 高度 - origin_alt_msl`，单位米。`bounds`、`resolution`、规则距离和网格都使用这个局部坐标。场景半径由 `max_radius_m` 限制，最大 50000 m；越界报错，不继续计算。

| `alt_reference` / `altReference` | 输入 Z 的含义 | 转成 MSL 的依据 |
|------|------|------|
| `MSL` | 海拔米 | 直接使用 |
| `AGL` | 离地米 | `MSL = Z + 地形海拔`；需要场景 terrain 数据或声明的常量 `terrain_msl_m` |
| `ELLIPSOID` | WGS84 椭球高米 | `MSL = Z - geoid_separation_m`；需提供大地水准面差 |
| `QNE` / `QNH` | 相应气压高度米 | `MSL = Z + offset_m`；需有效位置、时刻内唯一的 `height_corrections` 记录 |

服务不会猜高度基准或用零代替缺少的证据。独立坐标转换端点只接受 `CoordinateSpec` 的常量地形；场景规划会将 terrain 空间场注册为转换回调，按位置和有效时间读取。数据的来源、版本和有效期由上游负责。QNE/QNH 转换必须有带时区的 `at`；缺少修正报 `HEIGHT_REFERENCE_UNAVAILABLE`，同时匹配多条修正报 `AMBIGUOUS_HEIGHT_CORRECTION`。

`Scene.time_origin` 是所有数值时间的零点。`vertex_times_s`、动态障碍时间、关键点 `time_s`、运动段起止时间、冲突时间均为**相对这个零点的秒数**，不是相对各任务起飞的秒数。`departure_time`、有效期和时间窗仍使用绝对 ISO 时间。例如场景零点为 09:00，任务在 09:05 起飞，则首个数值时间至少是 300；为避让延迟到 09:06 时至少是 360。时间戳与几何顶点逐一对应，必须严格递增；悬停用两个相同位置、不同时间表示。

### 10.3 三种路径模式

| `route_mode` | 行为 |
|------|------|
| `strict`（默认） | 飞行器及安全裕度形成的扫掠体必须完全位于当时有效的航路体或已明确授权的起终点接驳体内。必须有航路；找不到合法路线就失败，不自动改为自由飞行 |
| `prefer` | 允许航路外飞行；候选图代价中，航路外基础距离代价为航路内的 2 倍；其余安全、规则和性能约束照常执行 |
| `free` | 不要求航路包含；其余安全、规则和性能约束照常执行 |

中心线 Z 是航路参考路径，**不把整个航路压成一条固定高度线**。硬高度边界是 `minAltitude/maxAltitude` 及其 `altReference`。`widthMeters` 是航路**全宽**，编译时除以 2；规则 `WIDTH.content.width` 是**半宽**，另减去 `bufferWidth` 与飞机安全裕度。

严格模式下，航路外起终点需显式提交 `connectors`，任务填写 `start_connector_id` / `end_connector_id`；`Connector.airway_ids` 指定可以接入的航路。接驳仅授权实际起点到首次进入航路、最后离开航路到实际终点的连续部分，不能当作中段捷径。接驳体也有高度、时间和净空限制。中心线、端点和接驳未连通时直接失败。

### 10.4 场景选择与生命周期

单任务、批量和验证请求中 `scene_id` 与 `scene` **必须且只能给一个**。推荐先注册一次静态场景，再用返回的 `scene_id` 提交多轮任务；首次编译耗时在 `compilation_ms` 中单独返回。场景版本为规范化场景 JSON 的 SHA-256；相同输入复用编译结果，修改规则、数据版本等会得到新版本。服务不缓存任务答案。

场景和作业存在当前服务进程内存，重启后必须重新注册场景，历史作业不保证可查。当前最多保留 8 个场景，登记场景的合计估算存储预算为 1 GiB；编译前按最近使用顺序淘汰以满足数量和估算预算，新场景编译串行执行。保留场景原始 JSON，遇到场景不存在时重新注册。估算预算不等于进程 RSS 硬上限，容器资源限制仍独立生效。单进程部署可确保场景与作业查询落在同一份内存中。

## 11. v2 场景与输入字段

### 11.1 外层请求

| 对象 / 字段 | 类型、必填与默认 | 说明 |
|------|------|------|
| 单任务 `task` | `Task`，必填 | 一条业务任务 |
| 批量 `tasks` | `Task[]`，1–10000 项 | HTTP 与 SDK 均检查此范围；ID 不可重复；结果顺序与请求顺序一致 |
| `scene_id` / `scene` | string / `Scene`，二选一 | 引用已注册场景或内联注册 |
| 规划 `time_budget_s` | number，默认 10，`0 < 值 <= 300` | 一次规划的总预算；不保证每个输入都能成功 |
| 批量 `request_id` | string，可省略 | 省略则生成 UUID；原样写入批量结果 |
| 验证 `task`、`trajectory_wkt`、`vertex_times_s` | 必填；时间数组 1–100000 项 | 任务、WGS84 `POINT Z/LINESTRING Z` 与相对场景零点的时间；顶点与时间长度须匹配，HTTP 与 SDK 均检查时间项数 |
| 验证 `motion_segments` | `MotionSegment[] \| null`，模型允许省略，最多 100000 段 | **移动航线必须提交**；直接复制规划结果的同名字段。缺少时返回 `valid:false`、`MOTION_MODEL_REQUIRED`，不能仅凭 WKT 与顶点时间通过验证 |

### 11.2 `Scene`

| 字段 | 类型 | 必填 / 默认 | 含义 |
|------|------|------|------|
| `coordinates` | `CoordinateSpec` | 必填 | 投影原点与高度换算证据 |
| `bounds` | `Bounds3D` | 必填 | 局部米制 `x_min/x_max/y_min/y_max/z_min/z_max`，每轴 max 必须大于 min |
| `resolution` | `[number,number,number]` | `[25,25,10]` | 局部 x/y/z 搜索分辨率，均须 >0 |
| `horizon_s` | number | 3600 | 从 `time_origin` 起的规划时间范围，`0 < 值 <= 86400` 秒 |
| `time_origin` | timestamp | 必填 | 带时区的场景时间零点 |
| `airways` | `Airway[]` | `[]` | 航路体 |
| `connectors` | `Connector[]` | `[]` | 严格模式显式接驳体 |
| `obstacles` | `Volume[]` | `[]` | 静态障碍体 |
| `dynamic_obstacles` | `TimedTrajectory[]` | `[]` | 上游已知、不可由本次任务调度改变的轨迹 |
| `control_zones` | `ControlZone[]` | `[]` | 管制体 |
| `rules` | `Rule[]` | `[]` | 20 类规则的实例 |
| `datasets` | `Dataset[]` | `[]` | 地形、天气、信号、区域、风险等空间场 |
| `profiles` | `AircraftProfile[]` | `[]` | 自定义飞机性能配置 |
| `grid` | `GridSpec \| null` | null | 可选外部占用网格 |

各列表内部 ID 唯一；几何资源（航路、接驳、障碍、管制、数据集）之间也不能重名。规则引用不存在的资源、任务引用不存在的规则或机型时拒绝请求。

编译前检查以下单场景运行预算；超出返回 HTTP 400、`INVALID_INPUT`，detail 给出 resource/actual/limit。它们是部署约束，输入不必达到上限才有效；实际可用规模还受场景存储估算与搜索约束共同限制。

| 资源 | 上限 |
|------|------|
| `airways/obstacles/control_zones` | 200 / 1000 / 20 |
| `connectors/rules/profiles/datasets` | 1000 / 1000 / 32 / 100 |
| `geometry_vertices` | 50000，累计所有输入 WKT 顶点 |
| `field_cells` | 1200000，累计所有数据集格子 |
| `grid_entries` | 1200000，occupied、occupied_ranges、cells 的条目总数 |
| `grid_nodes` | 4000000，外部网格 shape 三轴乘积 |
| `estimated_scene_bytes` | 536870912（512 MiB），模型、几何、索引和缓存的保守存储估算 |

场景登记另受上面的 8 场景 / 合计 1 GiB 估算预算限制。固定内存估算与条目限制不能推导出每个上限组合都可同时达到；例如大量格子的估算存储可能先触发 512 MiB 限制。`GET /api/v2/capabilities` 用于查询当前镜像实际能力和限额。

每条路线的认证碰撞采样与导出运动顶点各有 100000 项预算；达到预算时返回明确的 sample budget exceeded 错误，不截断轨迹后当作成功结果。业务可缩短任务、调整场景/运动参数或拆分有明确衔接约定的任务，不能忽略缺失的后半段。

### 11.3 `CoordinateSpec` 与高度修正

| 字段 | 类型 / 默认 | 含义 |
|------|------|------|
| `origin_lon` / `origin_lat` | number，必填 | 经度 `[-180,180]`、纬度 `[-90,90]` |
| `origin_alt_msl` | number，0 | 局部 z=0 的 MSL 海拔 |
| `max_radius_m` | number，50000 | 大于 0、最大 50000 |
| `terrain_msl_m` / `terrain_source` | number / string，null | 声明的平地海拔与来源；非平地区域应提供 terrain 空间场 |
| `geoid_separation_m` / `geoid_source` | number / string，null | 椭球高减 MSL 的差值与来源 |
| `height_corrections` | `HeightCorrection[]`，`[]` | QNE/QNH 的显式修正 |

`HeightCorrection` 必填 `reference`（`QNE/QNH`）、`offset_m`、非空 `source/version`、`valid_from/valid_to`；可选 `geometry_wkt` 限定 WGS84 平面覆盖。有效期必须递增，修正采用 `valid_from <= at < valid_to`。

### 11.4 航路、障碍、接驳和管制区

| `Airway` 字段 | 类型 / 默认 | 含义 |
|------|------|------|
| `id` | string，必填 | 航路 ID |
| `centerLine` | WGS84 `LINESTRING Z`，必填 | 三维参考中心线 |
| `widthMeters` | number >0，必填 | 全宽，米 |
| `minAltitude` / `maxAltitude` | number，必填 | 硬高度上下界，max >= min |
| `altReference` | 高度枚举，必填 | 上下界和中心线的基准 |
| `startGrid` / `endGrid` | `GeographicBounds`，必填 | 端点范围元数据；规范键为 `minLon/maxLon/minLat/maxLat/minAlt/maxAlt` |
| `status` | `ACTIVE/INACTIVE`，`ACTIVE` | 非活动航路不授权严格飞行 |
| `direction` | `BOTH/FORWARD/REVERSE`，`BOTH` | 相对中心线顺序的通行方向 |
| `valid_from` / `valid_to` | timestamp，可省略 | 航路有效时段 |
| `climbConstraint` | object/null | `maxClimbRate/maxDescentRate`，均 >0，m/s |
| `turnConstraint` | object/null | `maxTurnAngle`（`0<值<=180` 度）、`minTurnRadius`（>=0 米） |

`Volume` 必填 `id/geometry_wkt/min_altitude/max_altitude/alt_reference`；可选 `buffer_m`（默认 0，>=0）、`valid_from/valid_to`。`Connector` 在此基础上增加 `airway_ids: string[]`（默认 `[]`）；显式列出允许连接的航路能避免误接其他航路。体积有效期必须递增。

`TimedTrajectory` 必填 `id/trajectory_wkt/vertex_times_s/alt_reference`；几何必须是 WGS84 `POINT Z` 或 `LINESTRING Z`，时间数组与顶点等长、非负且严格递增。可选 `horizontal_separation_m/vertical_separation_m` 均 >=0，默认各 5 m；它们与本任务飞机尺寸及安全裕度共同形成保守避让间隔。所有数值时间相对 Scene.time_origin。单点轨迹仅在其时刻存在；需要持续悬停时提供两个相同位置和不同时间的顶点。

`ControlZone` 使用业务字段：

| 字段 | 类型 / 默认 | 含义 |
|------|------|------|
| `id`、`geom` | string、WKT，必填 | 唯一 ID 和权威平面足迹 |
| `minAlt/maxAlt/altReference` | number/number/高度枚举，必填 | 硬高度上下界及基准 |
| `zoneType` | string，必填 | 管制形状类型 |
| `boundary2d/centerPoint` | WKT/null | 辅助元数据，不覆盖 `geom` |
| `shapeParams` | JSON object，`{}` | 辅助形状参数 |
| `areaM2/volumeM3` | number >=0/null | 元数据；兼容输入 `area/volume`，不改变权威几何 |
| `level` | string/int/null | 规则 `zoneLevels` 的匹配值 |
| `status` | `ACTIVE/INACTIVE`，`ACTIVE` | 是否启用 |
| `effective_start/effective_end` | timestamp/null | 生效时段 |

蘑菇形等复杂管制体使用 `geom` 的保守足迹覆盖完整高度，不能用辅助元数据缩小禁入区。

### 11.5 空间场与地形

`Dataset` 必填 `id/kind/version`；`kind` 为 `terrain/weather/signal/noise/area/grid/risk/corridor` 之一；可选 `valid_from/valid_to`、`geometry_wkt`、`cells`（默认 `[]`）。数据集和单元格的有效期共同约束适用范围。

`FieldCell` 的 `geometry_wkt`（WGS84 足迹）与 `bounds`（局部 AEQD 六个边界）必须二选一；`values` 必填。WKT 格可用 `min_altitude/max_altitude` 限定 MSL 海拔；使用 `bounds` 时其局部 z 范围生效。可选 `valid_from/valid_to` 为带时区时间。未给高度界限的足迹格覆盖全高度。

| 数据类型 | `values` 允许键 | 单位与解释 |
|------|------|------|
| `terrain` | `elevation_msl_m`、`terrain_msl_m`、`nodata` | MSL 米；前者为规范键。可用地形必须有有限海拔 |
| `weather` | `wind_speed_mps`、`windSpeed`、`precipitation`、`precipitation_mm_h`、`icing`、`nodata` | 风 m/s；降水和结冰可为布尔或非负数，0/false 表示无 |
| `signal` | `bands`、`signal_strength_dbm`、`nodata` | `bands` 为频段到 dBm 的对象，如 `{"2.4GHz":-60,"5.8GHz":-70}`；亦可给统一强度 |
| `noise` / `risk` | `noise_db` / `risk`、`nodata` | 噪声 dB；风险数值，供代价计算 |
| `area` | `adminCode/gridCode/code/nodata` | 行政区、网格或通用编码 |
| `grid` | `adminCode/gridCode/code/occupied/nodata` | 区域编码与占用属性 |
| `corridor` | `nodata` | 明确允许穿行的几何区域 |

未知 `values` 键报 `INVALID_INPUT`。规则引用天气、信号或风险场时，缺少匹配数据集、版本、单元格或必要数值会拒绝编译。规划经过未覆盖、过期或 `nodata=true` 的相关空间场时视为不可通行。terrain 一旦配置，也必须在飞行位置与时间全程有效；不能把缺地形当平地。重叠有效地形格取较高海拔，避免低估地面。

算法不拉取实时天气、不测量无线信号；平台向服务提供已有观测/预报和质量标记，服务据此规划与验证。实际天气采集、链路监测和飞控执行由平台完成。

### 11.6 外部占用网格 `GridSpec`

当前网格 `crs` 必须为 **`LOCAL_AEQD`**；不接受 WGS84 网格坐标。经纬度网格需先通过坐标转换并在调用方重新构建米制单元格。

| 字段 | 类型 / 默认 | 含义 |
|------|------|------|
| `crs` | string，`LOCAL_AEQD` | 必须显式保持本地 AEQD 语义 |
| `origin` | `[x,y,z]`，必填 | 第 `(0,0,0)` 格的最小角，局部米 |
| `resolution` | `[dx,dy,dz]`，必填 | 每轴格边长，均 >0 |
| `shape` | `[nx,ny,nz]`，必填 | 每轴格数，均为正整数 |
| `occupied` | `[[i,j,k],...]`，`[]` | 稀疏占用索引；从 0 开始，必须在 shape 内 |
| `occupied_ranges` | `[{"start":[i,j,k],"end":[i,j,k]},...]`，`[]` | 两端都包含的占用索引范围 |
| `cells` | `[{"bounds":{...},"code":"...","occupied":true},...]`，`[]` | 独立的显式局部边界格；`code` 可省略，occupied 默认 true |
| `codes` | `{"i,j,k":"业务编码"}`，`{}` | 规则与输出关键点使用的编码映射 |

稀疏索引、索引范围与显式边界格可以并存，作为占用体的并集。规则 `AREA.gridCodes` 引用的编码必须能解析到真实几何，不能只给没有范围的字符串。

### 11.7 `Task`、航点和当前状态

| `Task` 字段 | 类型 / 默认 | 含义 |
|------|------|------|
| `id` | 非空 string，必填 | 任务 ID |
| `start_wkt` / `goal_wkt` | `POINT Z`，必填 | WGS84 起终点 |
| `departure_time` | timestamp，必填 | 最早起飞时间，必须在场景时域内 |
| `alt_reference` | 高度枚举，必填 | 本任务起终点、航点与输出 WKT 的高度基准 |
| `route_mode` | `strict/prefer/free`，`strict` | 路径模式 |
| `waypoints` | `Waypoint[]`，`[]`，最多 256 项 | 按顺序经过的中间业务航点，不重复放起终点；另受 WAYPOINT 规则约束 |
| `earliest_arrival/latest_arrival` | timestamp/null | 终点到达窗口；最晚到达不能早于 departure |
| `profile_id` | string，`dji_mini_4_pro` | 引用场景配置或内置默认机型 |
| `aircraft` | `AircraftProfile`/null | 直接指定时优先于 profile_id |
| `state` | `AircraftState`/null | 上游当前飞机状态 |
| `payload_kg` | number >=0，0 | 实际载荷 |
| `priority` | int，0 | 越大先调度；并列再按最晚到达与 ID 排序 |
| `start_connector_id/end_connector_id` | string/null | 显式接驳资源 ID |
| `rule_ids` | string[]/null | null 为选择全部场景规则；`[]` 为不选择业务规则，基本障碍/航路/性能检查仍执行 |

`Waypoint` 必填 `wkt: POINT Z`；可选 `earliest/latest`、`hover_s`（>=0，默认 0）、`hover`（默认 false）。悬停应在业务航点标记；`hover_s` 给明确时长，`hover=true` 可配合 HOVER 规则的 `defaultHoverTime`。时间窗均须带时区。

`AircraftState` 字段均可省略：`at`（timestamp）、`battery_percent`（0–100）、`remaining_flight_time_s`、`remaining_range_m`（均 >=0）、`home_wkt`（`POINT Z`）、`alt_reference`。RTL 启用低电触发时，必须提供电量、剩余飞行时间、剩余航程与 home，不能只给一个低电标志。平台应使用与任务一致的状态高度基准。

RTL 是**根据提交时当前状态生成的返航方案**：触发后将目标改为 `home_wkt`，生成满足 `rtlAltitude` 的返航路径，并检查当前剩余航程与续航；服务不持续监听电池，也不向无人机下发返航指令。持续储备监测、实际执行与后续重规划由平台负责。

### 11.8 默认 DJI 工程规划配置

默认 ID 为 `dji_mini_4_pro`。这些是仓库提供的工程规划参数，不代表厂商认证飞控包线；`engineering_values` 显式列出工程取值字段。业务可提交自己的完整配置，`GET /api/v2/version` 返回运行镜像实际默认值。

| 字段 | 默认 | 单位 / 约束 |
|------|------|------|
| `id/name` | `dji_mini_4_pro` / `DJI Mini 4 Pro (engineering planning profile)` | 字符串 |
| `max_speed_mps/cruise_speed_mps` | 12 / 6 | m/s，均 >0，cruise <= max |
| `climb_speed_mps/descent_speed_mps` | 3 / 3 | m/s，>0 |
| `max_yaw_rate_deg_s` | 60 | 度/秒，>0 |
| `acceleration_mps2/deceleration_mps2` | 2 / 2 | m/s²，>0 |
| `min_turn_radius_m` | 0 | 米，>=0；零半径转向需停车并计入原地偏航时间 |
| `radius_m/height_m/safety_margin_m` | 0.3 / 0.2 / 5 | 米，半径和高度 >0，裕度 >=0 |
| `endurance_s/max_range_m` | 2040 / 18000 | 秒 / 米，>0 |
| `payload_kg` | 0 | kg，>=0 |
| `rtl_altitude_m` | 20 | 米，>=0 |
| `can_hover` | true | boolean |
| `supported_bands` | `["2.4GHz","5.8GHz"]` | 支持频段 |
| `low_battery_percent` | 20 | 0–100 |
| `engineering_values` | 速度、爬降率、偏航率、加减速、转弯半径、尺寸、安全裕度、返航高度与低电阈值字段名列表 | 完整列表以 version 响应为准 |

## 12. v2 全部规则

每条 `Rule` 必填 `id`、`rule_type`、非空 `content`；可选 `resource_ids`（默认 `[]`）、`effective_start/effective_end`。`content` 采用下表精确键名，未知类型、未知键、错误类型、反向区间和缺少必要字段都会报 `INVALID_INPUT`。时间与空间适用范围在整段上求交，不能只检查端点。无 `resource_ids` 时按规则类别选择场景中对应资源；给出引用时每个 ID 都必须存在。

| 类型 | `content` 全部允许键 | 执行语义和数据要求 |
|------|------|------|
| `HEIGHT_LAYER` | `minHeight,maxHeight,layerStep,minAltitude,maxAltitude,altDatum` | 至少给一个高度上下界；`altDatum` 为五类高度基准，默认 MSL；layerStep >0，用高度层锚定巡航高度，爬降过渡仍查上下界；AGL 必须有地形 |
| `WIDTH` | `width,bufferWidth` | width 必填且 >0，表示半宽；bufferWidth >=0。需要航路，实际可用半宽再减飞机半径与安全裕度 |
| `DIRECTION` | `direction,directions` | 至少给一项；direction=`ONE_WAY/BIDIRECTIONAL`。directions 是 `[0,360)` 角度数组，东为 0°、逆时针；单向未给角度时按引用航路方向检查 |
| `SPEED` | `minSpeed,maxSpeed,cruiseSpeed` | m/s；max/cruise >0，min >=0，min<=max。巡航和物理起停加减速阶段区分处理 |
| `TIME` | `dailyStart,dailyEnd,weekdays,earliestDeparture,latestArrival` | 搜索将日历开放期转为起飞窗口，冲突时可重试后续窗口；日内时间 `HH:mm`，Asia/Shanghai；dailyStart/dailyEnd 必须成对，支持跨午夜；weekdays 为 ISO 1–7 整数；earliestDeparture/latestArrival 也是日内 `HH:mm` |
| `AREA` | `scope,adminCodes,gridCodes,zoneIds` | scope 必填=`INCLUDE/EXCLUDE`；需可解析的区域、网格码或管制几何；INCLUDE 要求整段留在包含区，EXCLUDE 禁止穿入 |
| `CLIMB` | `maxClimbRate,maxDescentRate` | m/s；与机型能力取更严格限制，逐段检查 |
| `TURN` | `maxTurnAngle,minTurnRadius` | 角度 `0<值<=180`，半径米 >=0；约束完整转弯累计角及几何半径 |
| `SEPARATION` | `verticalSeparation,horizontalSeparation` | 米，>=0；与飞机尺寸和安全裕度取保守值，连续时空检查双方同时过近 |
| `NO_FLY` | `noFlyMode,heightCeiling,includeBuffer` | noFlyMode 必填=`PERMANENT/TEMPORARY`；临时规则须显式起止有效期；需要匹配管制几何；heightCeiling 为 MSL 米 |
| `BUILDING` | `avoidMode,safeDistance,bufferRadius` | avoidMode 必填=`BUFFER/CORRIDOR`；距离米 >=0；需障碍几何，CORRIDOR 还需明确 corridor 数据集，不能把建筑整体当穿行区 |
| `ZONE` | `zoneLevels,coverageThreshold` | 至少给一项；zoneLevels 为非空字符串数组；coverageThreshold 为 0–1；需匹配管制体，阈值不能取消已经触及的硬禁入体 |
| `WEATHER` | `maxWindSpeed,noPrecipitation,noIcing` | 至少给一项；风 m/s，后两项 boolean；需 versioned weather 格场，相应风/降水/结冰数值必须存在，全程有效且覆盖 |
| `SIGNAL` | `minSignalStrength,bands,band_policy` | 强度 dBm 必填；bands 为非空字符串数组；band_policy=`any`（默认）或 `all`；需 signal 格场，并匹配飞机支持频段 |
| `PRIORITY` | `priorityScene,costBonus` | 两者必填；场景=`LOW_RISK/FAST/QUIET`，bonus 为 0–1；LOW_RISK 需 risk 格场，QUIET 需 noise 格场，覆盖缺口不放行；影响候选路线代价 |
| `HOVER` | `maxHoverTime,defaultHoverTime` | 秒 >=0，default <= max；只允许明确标记的业务航点悬停，实际持续时间不能超限 |
| `ENDURANCE` | `maxRange,minEndurance` | 米 / 秒，均 >0；minEndurance 是机型容量最低要求；同时检查路径与当前剩余续航/航程 |
| `PAYLOAD` | `maxPayloadKg` | kg >=0，必填；检查机型载荷能力与实际任务载荷 |
| `WAYPOINT` | `maxWaypoints,minWaypointInterval` | maxWaypoints 为 >=2 的整数，计起点+业务中间点+终点，不计搜索采样点；间隔为米 >=0 |
| `RTL` | `rtlAltitude,rtlOnLowBattery,lowBatteryPercent` | 三项必填；米 >=0、boolean、0–100。低电开关启用时要求完整上游状态；达到阈值生成 home 返航方案，并检查当前剩余资源 |

示例规则（加入上面的严格航路场景即可运行）：

```json
[
  {"id":"height-1","rule_type":"HEIGHT_LAYER","content":{"minAltitude":60,"maxAltitude":120,"altDatum":"MSL"}},
  {"id":"speed-1","rule_type":"SPEED","content":{"maxSpeed":8,"cruiseSpeed":5}},
  {"id":"width-1","rule_type":"WIDTH","resource_ids":["demo-airway"],"content":{"width":40,"bufferWidth":2}},
  {"id":"separation-1","rule_type":"SEPARATION","content":{"horizontalSeparation":15,"verticalSeparation":12}}
]
```

全部规则均有执行路径；不能将“解析成功”视作“本次航线已满足”。调用方要同时检查路线 `status`、`validation.valid` 和 `rule_report`。规则未被选择或不在生效期会标记为不适用；缺少应有引用数据不会以“已应用”代替实际检查。

## 13. v2 返回与错误

### 13.1 场景、坐标与基础端点

| 对象 | 返回字段 |
|------|------|
| 场景注册/查询 | `scene_id`、`scene_version`（SHA-256）、`status="ready"`、`compilation_ms`、`coordinate_context`、`warnings`、`counts`（airways/obstacles/control_zones/rules/datasets 数量） |
| 坐标转换 | `geometries_wkt`（与输入顺序相同）、`direction`、`alt_reference`、`coordinate_context` |
| health | `status="ok"/"degraded"`、`api_version`、`native_available`、`jobs`；缺原生扩展时 degraded，不能据此承诺可规划 |
| version | `api_version`、`engine`（available/engine/version/error）、`default_profile` |
| capabilities | `api_version/route_modes/default_route_mode/rule_types/altitude_references/geometric_tolerance_m/max_scene_radius_m/grid_formats/scene_lifecycle`；另有 `scene_limits`（§11.2 单场景预算整数对象）、`max_registered_scenes`（默认 8）、`max_registered_scene_bytes`（默认 1073741824，登记合计估算预算） |

`coordinate_context` 包含输入 EPSG:4326、轴序、局部 AEQD CRS、原点、半径、地形/水准面来源与气压修正版本。公开几何容差为 0.1 m；这用于数值和离散近似的保守处理，不允许调用方削减配置的安全距离。

### 13.2 `RouteResult`

| 字段 | 类型 | 含义 |
|------|------|------|
| `task_id` | string | 对应任务 ID |
| `status` | `planned/failed/timeout` | 逐任务结果；HTTP 200 不代表每个任务成功 |
| `code` / `message` | string/null / string | 失败代码和解释；成功时 code 通常 null |
| `route_mode` | 路径枚举 | 实际任务模式 |
| `start_wkt/goal_wkt` | WKT | 本次生效的起终点；触发 RTL 时 goal 为 home |
| `departure_time` | timestamp | 请求的最早起飞时刻；实际出发看首个 vertex time |
| `time_origin` | timestamp/null | 本结果数字时间的场景零点；成功路线会填写 |
| `action` | `mission/return_to_home` | 普通任务或当前低电触发后的返航方案 |
| `trajectory_wkt` | WKT/null | 任务高度基准下的 WGS84 `LINESTRING Z`，静止点可为 `POINT Z`；失败无轨迹 |
| `vertex_times_s` | number[] | 每个 WKT 顶点的相对场景零点时间 |
| `motion_segments` | `MotionSegment[]` | 精确运动原语和时间律，局部 AEQD 米制位置 |
| `keypoints` | `Keypoint[]` | 业务与运动事件关键点 |
| `metrics` | object | `distance_m/flight_time_s/wait_time_s`；wait 是调度的起飞延迟 |
| `validation` | object | `valid` 与 `violations` |
| `rule_report` | `RuleReport[]` | 每条规则的应用情况、原因与数据版本 |

失败结果的数组通常为空，指标默认 0，不能当可飞行路线使用。`flight_time_s` 为实际轨迹首末时刻差；起飞前地面等待不视为已经起飞的空中占用。

`MotionSegment` 的 `kind` 为 `line/arc/climb/descent/hover/yaw`。`start/end/center` 均是**局部 AEQD 米制三元组**，不是经纬度；`start_time_s/end_time_s` 相对场景零点。`airway_ids` 列出该段相关的航路 ID，未匹配时为空数组。`length_m` 为原语长度；`radius_m/center` 用于圆弧；`start_speed_mps/end_speed_mps/acceleration_mps2` 与 `profile` 描述加速、匀速、减速时间律。`profile` 的规范键为 `peak_speed_mps/deceleration_mps2/accel_time_s/cruise_time_s/decel_time_s/sweep_angle_deg`，sweep 为带符号角度，原地 yaw 同样保留真实转角。

模型还保留 `distance_m/points/times_s/max_speed_mps/deceleration_mps2/peak_speed_mps/accel_time_s/cruise_time_s/decel_time_s/sweep_angle_deg` 可选描述字段；应以服务实际填充的原语和 profile 值重建运动。WKT 折线是原语的有限误差采样视图；折线顶点本身不能证明采样间加速度、圆弧峰值或偏航满足包线，执行平台应保留原语时间律。

`Keypoint` 含 `types:string[]`、`wkt`、`time_s`、可空 `grid_indices:[i,j,k]`、`grid_code` 与 `waypoint_index`。业务 `WAYPOINT` 事件的 waypoint_index 对应 Task.waypoints 的从 0 开始的索引，其他事件通常为 null。同一点可有多个事件；航点、悬停、爬升/下降与转弯的起止由事件类型标记。`AIRWAY_SWITCH` 表示当前航路覆盖结束后的确定性交接：重叠区继续保留原航路，再按稳定 ID 选择接续航路。切换关键点使用解析运动的准确位置和时刻，可以位于两个导出 WKT 顶点之间，不改变运动时间律。`grid_indices` 始终对应 Scene.bounds/resolution 的内部规划网格；`grid_code` 另按上游 GridSpec 的原点和分辨率或显式网格单元匹配。

### 13.3 批量、验证与规则报告

| `BatchResult` 字段 | 含义 |
|------|------|
| `planned_at` | 返回生成的 UTC 时间 |
| `request_id` | 调用方 ID 或生成 UUID |
| `scene_id/scene_version` | 实际场景及 SHA-256 |
| `routes` | 每个输入任务一个 RouteResult，保留输入顺序 |
| `success_count/failed_count` | planned 与其他结果计数 |
| `conflicts` | 连续冲突，元素含 `task_ids/start_time_s/end_time_s/minimum_distance_m`（距离可空） |
| `elapsed_ms` | 本次 SDK 规划墙钟耗时，毫秒；客户端完整 HTTP 耗时还包含传输与序列化 |
| `engine` | 当前为 `native-v2` |
| `coordinate_context` | 本批次的投影原点、轴序、半径与高度证据来源，供解释原语局部坐标 |

`validation` 和验证端点返回 `valid:boolean`、`violations:Violation[]`；验证端点另有 `rule_report`。`Violation` 含 `code/message`，可选 `rule_id/time_s` 与 `details`。`RuleReport` 含 `rule_id/rule_type/status/message/details/reasons/dataset_versions`；status 枚举为 `applied/not_applicable/inactive/failed/unsupported`。当前报告的原因及版本证据主要位于 `details.reasons` 和 `details.dataset_versions`，顶层字段为兼容保留；调用方应保存完整报告。

验证移动航线时，必须将规划返回的 `motion_segments` 与 `trajectory_wkt/vertex_times_s` 一起提交。服务重建解析运动，核对它与 WKT/时间视图一致，运行原生运动学证书，再以保守采样覆盖检查场景、规则、端点、业务航点和动态轨迹冲突。仅提交折线和逐点时间会得到 HTTP 200、`valid:false`、`MOTION_MODEL_REQUIRED`；原语不一致或运动学超限报告 `MOTION_MODEL` 等违规码，不能用任意折线插值替代真实时间律。

`POINT Z` 加一个时间表示零持续时间的静止位置，可省略 motion_segments；持续等待使用重复位置的 `LINESTRING Z`、对应严格递增的时间以及 hover 原语，仍检查飞机悬停能力、任务航点标记与 HOVER 规则。缺少原语、规则失败等属于验证结果，不是 HTTP 请求成功即可使用的飞行许可。

### 13.4 异步作业

提交返回 `{"job_id":"...","status":"pending","poll_url":"/api/v2/jobs/..."}`。查询含 `job_id/status/created_at/started_at/finished_at/elapsed_ms/result/error`。状态为 `pending/running/succeeded/failed`；`succeeded` 表示函数已返回，仍须检查 `result.failed_count` 与每条路线。`failed` 时 error 为 `code/message/detail`。作业元数据时间字符串使用服务记录的格式，飞行计算仍以场景带时区时间为准。

作业在进程内执行；服务重启或已完成作业被淘汰后，查询可返回 404。提交 202 的延迟不能用作完整规划性能。

v2 同时接纳的 `pending + running` 最多 32 个，执行线程为 1；达到容量或服务停止接纳时，POST /jobs 返回 HTTP 429、`JOB_QUEUE_FULL`，不生成新 job_id。调用方应延迟重试，不能无限加速提交。已完成历史按数量和结果估算存储预算淘汰，结果合计预算为 134217728 字节（128 MiB）；这是内存对象估算，非 JSON 字节数或进程 RSS。单个结果超过该保留预算时，作业转为 failed、error.code=`JOB_RESULT_TOO_LARGE`，查询本身仍为 HTTP 200。需要长期保存的结果由平台获取后持久化。

服务关闭时停止接纳，排队的 pending 作业可转为 failed、`JOB_CANCELLED`；正在执行的作业随应用生命周期等待完成。部署仍先在上游暂停提交并排空，避免依赖关机阶段完成任务。

### 13.5 错误与任务失败

v2 沿用 §4 的统一 `{"error":{"code","message","detail"}}`。模型校验失败返回 HTTP 400、`INVALID_INPUT`，detail.errors 带字段与原因；无场景/作业为 404、`TASK_NOT_FOUND`；未知机型为 404、`AIRCRAFT_NOT_FOUND`。下表是 v2 额外的参数错误，均返回 400：

v2 OpenAPI 为所有端点声明同一 `ErrorEnvelope`，覆盖 `400/404/429/500/503` 和 `4XX`；不使用 FastAPI 自动的 422 校验格式。HTTP 429 为后台作业容量/关闭拒绝，code 为 `JOB_QUEUE_FULL`；HTTP 503 为必须的原生内核不可用，code 为 `NATIVE_UNAVAILABLE`；未预期异常为 HTTP 500、`INTERNAL_ERROR`。

| code | 原因 |
|------|------|
| `HEIGHT_REFERENCE_UNAVAILABLE` | 缺地形、水准面差或有效气压高度修正 |
| `AMBIGUOUS_HEIGHT_CORRECTION` | 同位置同时间有多条气压高度修正 |
| `INVALID_ALTITUDE_REFERENCE` | 不支持的高度基准（通常先被模型拒绝为 INVALID_INPUT） |
| `INVALID_COORDINATE` | 无效几何、坐标范围或非有限坐标 |
| `COORDINATE_RANGE_EXCEEDED` | 超出声明的局部半径 |
| `COORDINATE_CONTEXT_MISMATCH` | SDK 使用的 CoordinateFrame 与请求原点配置不同 |

请求合法而路径不可行时，HTTP 200，RouteResult.status 为 failed/timeout，code 常见 `SEARCH_NOT_FOUND/TIME/CONSTRAINT_VIOLATION/SEPARATION/TIME_BUDGET_EXCEEDED`。其中 timeout 专指规划预算耗尽。缺 C++ 扩展时 health degraded，规划返回 503/NATIVE_UNAVAILABLE；不静默降级为 Python 算法。验证端点的 MOTION_MODEL_REQUIRED/MOTION_MODEL、作业内的 JOB_RESULT_TOO_LARGE/JOB_CANCELLED 属于结果违规/作业错误，检查响应体，不能只按 HTTP 状态判断成功。

## 14. v2 完整调用示例

### 14.1 完整单任务 JSON

将下列内容保存为 `single-v2.json`，再提交到 `/api/v2/plan/single`。这是带完整航路的严格模式示例，地形使用显式声明的平地，不依赖外部数据源。

```json
{
  "scene": {
    "coordinates": {
      "origin_lon": 104.77,
      "origin_lat": 29.33,
      "origin_alt_msl": 0,
      "terrain_msl_m": 0,
      "terrain_source": "example-flat-terrain-v1"
    },
    "bounds": {"x_min": -500, "x_max": 2000, "y_min": -500, "y_max": 1000, "z_min": 0, "z_max": 300},
    "resolution": [25, 25, 10],
    "horizon_s": 3600,
    "time_origin": "2026-10-16T09:00:00+08:00",
    "airways": [
      {
        "id": "demo-airway",
        "centerLine": "LINESTRING Z (104.771 29.331 80, 104.780 29.331 80)",
        "widthMeters": 100,
        "minAltitude": 50,
        "maxAltitude": 150,
        "altReference": "MSL",
        "startGrid": {"minLon":104.7709,"maxLon":104.7711,"minLat":29.3309,"maxLat":29.3311,"minAlt":50,"maxAlt":150},
        "endGrid": {"minLon":104.7799,"maxLon":104.7801,"minLat":29.3309,"maxLat":29.3311,"minAlt":50,"maxAlt":150},
        "status": "ACTIVE",
        "direction": "BOTH"
      }
    ],
    "rules": [
      {"id":"speed-1","rule_type":"SPEED","content":{"maxSpeed":8,"cruiseSpeed":5}},
      {"id":"separation-1","rule_type":"SEPARATION","content":{"horizontalSeparation":15,"verticalSeparation":12}}
    ]
  },
  "task": {
    "id": "demo-v2-001",
    "start_wkt": "POINT Z (104.771 29.331 80)",
    "goal_wkt": "POINT Z (104.780 29.331 80)",
    "departure_time": "2026-10-16T09:00:00+08:00",
    "alt_reference": "MSL",
    "route_mode": "strict",
    "latest_arrival": "2026-10-16T09:30:00+08:00"
  },
  "time_budget_s": 10
}
```

```bash
curl --fail-with-body -sS http://127.0.0.1:18000/api/v2/plan/single \
  -H 'Content-Type: application/json' --data-binary @single-v2.json

# 也可直接下载当前镜像提供的可运行示例。
curl --fail-with-body -sS http://127.0.0.1:18000/api/v2/examples/single -o single-v2.json
```

上面完整 JSON 的一次实际单任务返回如下。浮点末位可能因投影库版本有所变化；`profile` 保存实际梯形速度时间律，WKT 的 4 个顶点依次对应起点、加速结束、减速开始和终点。

```json
{
  "task_id": "demo-v2-001",
  "status": "planned",
  "code": null,
  "message": "",
  "route_mode": "strict",
  "trajectory_wkt": "LINESTRING Z (104.771 29.331 80, 104.771064348832 29.331000002154 80, 104.779935651168 29.331000002154 80, 104.78 29.331 80)",
  "start_wkt": "POINT Z (104.771 29.331 80)",
  "goal_wkt": "POINT Z (104.780 29.331 80)",
  "departure_time": "2026-10-16T09:00:00+08:00",
  "time_origin": "2026-10-16T09:00:00+08:00",
  "action": "mission",
  "vertex_times_s": [0.0, 2.5, 174.82834746683343, 177.32834746683343],
  "motion_segments": [
    {
      "airway_ids": ["demo-airway"],
      "kind": "line",
      "start_time_s": 0.0,
      "end_time_s": 177.32834746683343,
      "start": [97.12685972800742, 110.8416525552106, 80.0],
      "end": [971.2685960957951, 110.88275614663762, 80.0],
      "length_m": 874.1417373341671,
      "distance_m": 0.0,
      "points": [],
      "times_s": [],
      "max_speed_mps": null,
      "radius_m": null,
      "center": null,
      "start_speed_mps": 0.0,
      "end_speed_mps": 0.0,
      "acceleration_mps2": 2.0,
      "deceleration_mps2": null,
      "peak_speed_mps": null,
      "accel_time_s": null,
      "cruise_time_s": null,
      "decel_time_s": null,
      "sweep_angle_deg": null,
      "profile": {
        "peak_speed_mps": 5.0,
        "deceleration_mps2": 2.0,
        "accel_time_s": 2.5,
        "cruise_time_s": 172.32834746683343,
        "decel_time_s": 2.5,
        "sweep_angle_deg": 0.0
      }
    }
  ],
  "keypoints": [
    {"types":["START"],"wkt":"POINT Z (104.771 29.331 80)","time_s":0.0,"grid_indices":[23,24,8],"grid_code":null,"waypoint_index":null},
    {"types":["GOAL"],"wkt":"POINT Z (104.78 29.331 80)","time_s":177.32834746683343,"grid_indices":[58,24,8],"grid_code":null,"waypoint_index":null}
  ],
  "metrics": {"distance_m":874.1417373341671,"flight_time_s":177.32834746683343,"wait_time_s":0.0},
  "validation": {"valid":true,"violations":[]},
  "rule_report": [
    {"rule_id":"speed-1","rule_type":"SPEED","status":"applied","message":"","details":{"reasons":[],"dataset_versions":{}},"reasons":[],"dataset_versions":{}},
    {"rule_id":"separation-1","rule_type":"SEPARATION","status":"applied","message":"","details":{"reasons":[],"dataset_versions":{}},"reasons":[],"dataset_versions":{}}
  ]
}
```

### 14.2 Python SDK：注册、单任务、批量、验证

SDK 与 HTTP 共用类型模型和实际规划实现；安装包须包含 C++17 原生扩展。以下脚本读取上一节 JSON，也演示两个相同起飞位置/时刻的任务由调度协调。

```python
import copy
import json
from uav_planner import register_scene, plan_single_v2, plan_batch_v2, validate_route_v2

with open("single-v2.json", encoding="utf-8") as source:
    request = json.load(source)
registered = register_scene(request["scene"])
scene_id = registered["scene_id"]
task = request["task"]

route = plan_single_v2(scene_id=scene_id, task=task, time_budget_s=10)
assert route["status"] == "planned", route
assert route["validation"]["valid"], route["validation"]

second = copy.deepcopy(task)
second["id"] = "demo-v2-002"
batch = plan_batch_v2(scene_id=scene_id, tasks=[task, second],
                      request_id="sdk-v2-demo", time_budget_s=10)
assert batch["success_count"] == 2, batch
assert not batch["conflicts"], batch["conflicts"]
print([(r["task_id"], r["metrics"]["wait_time_s"]) for r in batch["routes"]])

checked = validate_route_v2(scene_id=scene_id, task=task,
    trajectory_wkt=route["trajectory_wkt"], vertex_times_s=route["vertex_times_s"],
    motion_segments=route["motion_segments"])
assert checked["valid"], checked
```

### 14.3 requests：场景复用、完整结果和作业

```python
import copy
import json
import time
import requests

API = "http://127.0.0.1:18000"

def post(path, payload):
    response = requests.post(API + path, json=payload, timeout=60)
    data = response.json()
    if not response.ok:
        raise RuntimeError(data["error"])
    return data

# 此 GET 返回完整 scene/task，不依赖本地示例文件。
response = requests.get(API + "/api/v2/examples/single", timeout=10)
response.raise_for_status()
example = response.json()
registered = post("/api/v2/scenes", example["scene"])
task = example["task"]
route = post("/api/v2/plan/single", {"scene_id":registered["scene_id"], "task":task})
assert route["status"] == "planned" and route["validation"]["valid"], route
checked = post("/api/v2/validate", {"scene_id":registered["scene_id"], "task":task,
    "trajectory_wkt":route["trajectory_wkt"], "vertex_times_s":route["vertex_times_s"],
    "motion_segments":route["motion_segments"]})
assert checked["valid"], checked

second = copy.deepcopy(task)
second["id"] = "http-demo-002"
request = {"scene_id":registered["scene_id"], "tasks":[task, second],
           "request_id":"http-v2-demo", "time_budget_s":10}
batch = post("/api/v2/plan/batch", request)
assert batch["success_count"] == 2 and not batch["conflicts"], batch

submitted = post("/api/v2/jobs", request)
while True:
    response = requests.get(API + submitted["poll_url"], timeout=60)
    response.raise_for_status()
    job = response.json()
    if job["status"] == "failed":
        raise RuntimeError(job["error"])
    if job["status"] == "succeeded":
        assert job["result"]["success_count"] == 2, job["result"]
        break
    time.sleep(0.2)
print(json.dumps(job["result"], ensure_ascii=False))
```

### 14.4 坐标转换

```json
{
  "geometries_wkt": ["POINT Z (104.771 29.331 80)"],
  "direction": "wgs84_to_local",
  "alt_reference": "MSL",
  "coordinates": {"origin_lon":104.77,"origin_lat":29.33,"origin_alt_msl":20},
  "at": "2026-10-16T09:00:00+08:00"
}
```

该例的本地 z 为 60 m；水平坐标由 AEQD 计算。反向转换把返回的本地 WKT 放回 `geometries_wkt`，将 direction 改为 `local_to_wgs84`，使用完全相同的 coordinates/alt_reference/at。批量支持多条 2D 足迹和 3D 几何，顺序不变。

### 14.5 显式接驳与空间场片段

下面是可并入 Scene 的局部片段；坐标应按实际场景调整，接驳体必须真实覆盖起终点和授权航路之间的扫掠体。

```json
{
  "connectors": [
    {"id":"launch-connector","geometry_wkt":"POLYGON ((104.770 29.330,104.772 29.330,104.772 29.332,104.770 29.332,104.770 29.330))",
     "min_altitude":50,"max_altitude":150,"alt_reference":"MSL","airway_ids":["demo-airway"]}
  ],
  "datasets": [
    {"id":"weather-v1","kind":"weather","version":"2026-10-16-09",
     "valid_from":"2026-10-16T09:00:00+08:00","valid_to":"2026-10-16T10:00:00+08:00",
     "cells":[{"bounds":{"x_min":-500,"x_max":2000,"y_min":-500,"y_max":1000,"z_min":0,"z_max":300},
               "values":{"wind_speed_mps":3,"precipitation":false,"icing":false}}]}
  ],
  "rules": [
    {"id":"weather-rule","rule_type":"WEATHER","resource_ids":["weather-v1"],
     "content":{"maxWindSpeed":8,"noPrecipitation":true,"noIcing":true}}
  ]
}
```

使用接驳时在任务中设置 `start_connector_id="launch-connector"`；无需接驳的任务省略该字段。该片段的 rules 是一个独立示例；与已有规则合并时追加数组，避免覆盖原有约束。

## 15. v1 迁移到 v2

| 项目 | v1 | v2 |
|------|------|------|
| URL | `/api/v1/...` | `/api/v2/...` |
| SDK | `plan_single/plan_batch/validate_route` | `plan_single_v2/plan_batch_v2/validate_route_v2/register_scene` |
| 空间点 | 局部 `{x,y,z}` 或数组 | WGS84 `POINT Z`，明确 alt_reference |
| 场景 | config、障碍等随请求传入 | 完整 Scene 预注册或内联；用 scene_id 复用编译 |
| 时间 | 局部秒 | 绝对任务时刻 + 明确 time_origin；输出数值秒相对场景零点 |
| 航路范围 | 不提供 v2 航路语义 | strict 默认；外部自由飞行需显式选 free/prefer |
| 障碍大小 | AABB width/depth 为半宽/半深 | WKT 足迹 + 高度上下界；Airway.widthMeters 为全宽，WIDTH.width 为半宽 |
| 默认机型 | 通用四旋翼包线 | DJI Mini 4 Pro 工程规划配置；单位及默认值见 §11.8 |
| 轨迹 | `trajectory:[{x,y,z,t,...}]` | `trajectory_wkt` + 等长 `vertex_times_s` + 精确 motion_segments |
| 单任务结果 | 顶层 status + route + validation | 直接 RouteResult，含规则报告与关键点 |
| 多机结果 | v1 routes/conflicts 格式 | BatchResult，逐任务状态、场景版本、连续冲突 |

迁移时先确定投影原点与真实高度来源，把 v1 局部几何经坐标转换成为 WGS84；不要把米坐标直接写入经纬度 WKT。再把障碍半宽换算成足迹、注册场景，显式选择路径模式和机型，核对相对 time_origin 的时间。先用单任务比对端点、距离、高度和运动约束，再用有交叉任务的批量请求检查逐条状态与 conflicts。现有调用方可继续使用 v1，不需为新增 v2 修改请求。

v2 的固定性能验收使用 `input/v2_benchmarks` 的 SHA-256 冻结输入、三套各 1000 任务、各 10 轮、8 CPU/8 GB，要求每轮所有任务成功并完整返回 JSON ≤10 秒。场景注册单独计时；HTTP 异步提交的 202 不计作完成。该条件是验收口径，不是对任意地图、任意约束或不同行为任务的通用时延承诺。实际发布结果以对应镜像的验收记录为准。

批量结果 `stage_timings_ms` 记录场景/任务解析、搜索与运动证书、预约调度、最终校验与 WKT、SDK 响应序列化的毫秒耗时。HTTP 验收另计完整传输与 JSON 编解码，不以这些阶段之和替代整批计时。`GET /api/v2/version` 的 `engine.git_sha` 标识镜像构建提交。
