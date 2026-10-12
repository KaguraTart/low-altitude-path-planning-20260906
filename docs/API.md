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
