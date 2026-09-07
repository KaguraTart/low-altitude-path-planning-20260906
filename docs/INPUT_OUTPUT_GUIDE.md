# 输入输出格式指南

> 本文档详细描述本项目所有输入/输出文件的字段、类型、单位、约束和示例。
>
> 对应任务书 §三「输出要求」和 §十一「代码交付要求」。

---

## 目录

1. [输入文件（`input/`）](#1-输入文件input)
   - [1.1 时空网格配置 `config.json`](#11-时空网格配置-configjson)
   - [1.2 静态障碍物 `static_obstacles.json`](#12-静态障碍物-static_obstaclesjson)
   - [1.3 动态障碍物 `dynamic_obstacles.json`](#13-动态障碍物-dynamic_obstaclesjson)
   - [1.4 飞行器配置 `aircraft_config.json`](#14-飞行器配置-aircraft_configjson)
   - [1.5 飞行任务 `tasks.json`](#15-飞行任务-tasksjson)
2. [输出文件（`output/`）](#2-输出文件output)
   - [2.1 规划结果 `planning_result_*.json`](#21-规划结果-planning_result_json)
   - [2.2 航点指令 `waypoints_*.json`](#22-航点指令-waypoints_json)
   - [2.3 单航线 `routes/*.json`](#23-单航线-routesjson)
   - [2.4 环境快照 `environment.json`](#24-环境快照-environmentjson)
   - [2.5 冲突报告 `conflicts.json`](#25-冲突报告-conflictsjson)
   - [2.6 Markdown 报告 `report_*.md`](#26-markdown-报告-report_md)
   - [2.7 压力测试结果 `stress/`](#27-压力测试结果-stress)

---

## 1. 输入文件（`input/`）

所有输入文件位于 `input/` 目录，由 `plan_single.py` / `plan_multi.py` / `run_stress_test.py` 加载。

### 1.1 时空网格配置 `config.json`

**任务书依据**：§三"网格作为搜索基础" / §五.1"建立三维 GridMap"

**作用**：定义规划空域的大小、分辨率和时间窗口。A\* 在这个网格上做四维时空搜索。

**完整字段**：

| 字段 | 类型 | 单位 | 必填 | 默认 | 说明 |
|------|------|------|------|------|------|
| `x_min` | float | 米 | 是 | 0 | X 轴最小值 |
| `x_max` | float | 米 | 是 | 5000 | X 轴最大值 |
| `y_min` | float | 米 | 是 | 0 | Y 轴最小值 |
| `y_max` | float | 米 | 是 | 5000 | Y 轴最大值 |
| `z_min` | float | 米 | 是 | 0 | Z 轴最小值（地面） |
| `z_max` | float | 米 | 是 | 500 | Z 轴最大值（空域顶） |
| `dx` | float | 米/格 | 是 | 100 | X 方向分辨率 |
| `dy` | float | 米/格 | 是 | 100 | Y 方向分辨率 |
| `dz` | float | 米/格 | 是 | 50 | Z 方向分辨率 |
| `t_min` | float | 秒 | 是 | 0 | 规划窗口起点 |
| `t_max` | float | 秒 | 是 | 1200 | 规划窗口终点 |
| `dt` | float | 秒/步 | 是 | 10 | 时间步长 |

**示例**（demo 输入）：
```json
{
  "x_min": 0,  "x_max": 5000,
  "y_min": 0,  "y_max": 5000,
  "z_min": 0,  "z_max": 500,
  "dx": 100,   "dy": 100,  "dz": 50,
  "t_min": 0,  "t_max": 1200,
  "dt": 10
}
```

**自动计算**：网格维度 `nx = ceil((x_max-x_min)/dx)`，依次类推。
当前 demo 配置 → 50×50×10×120 = **300 万个时空格**。

**约束**：
- `dx, dy, dz, dt > 0`（不允许 0）
- `x_min < x_max` 等
- 推荐 nx×ny×nz×nt ≤ 10M（否则单次规划内存 > 10MB bool 网格）

---

### 1.2 静态障碍物 `static_obstacles.json`

**任务书依据**：§五.1"模拟建筑物、禁飞区、限制区域等静态障碍" / §三"满足静态障碍约束"

**作用**：定义不能飞入的固定物体。A\* 在这些格内占用 = True。

**顶层结构**：`{"obstacles": [ {...}, ... ]}`

**单条障碍物字段**：

| 字段 | 类型 | 单位 | 必填 | 默认 | 说明 |
|------|------|------|------|------|------|
| `id` | string | - | 是 | - | 障碍物唯一 ID |
| `name` | string | - | 否 | "unnamed" | 显示名 |
| `type` | enum | - | 是 | - | `building` / `tower` / `terrain` / `other` |
| `x` | float | 米 | 是 | - | 中心 X 坐标 |
| `y` | float | 米 | 是 | - | 中心 Y 坐标 |
| `z` | float | 米 | 是 | - | 底部 Z 坐标 |
| `width` | float | 米 | 是 | 20 | X 方向**半宽** |
| `depth` | float | 米 | 是 | 20 | Y 方向**半深** |
| `height` | float | 米 | 是 | 50 | 高度（从 z 基准向上） |
| `source` | string | - | 否 | "file" | 数据源标识 |

**示例**（建筑物）：
```json
{
  "id": "building-001",
  "name": "中心大厦",
  "type": "building",
  "x": 2500, "y": 2500, "z": 0,
  "width": 80, "depth": 80, "height": 200,
  "source": "city_gis"
}
```

**几何含义**：
- 建筑物在 X 方向占据 [x-width, x+width]
- Y 方向占据 [y-depth, y+depth]
- Z 方向占据 [z, z+height]

**注意**：建筑物**不是点**，是 AABB 长方体包围盒，A\* 在其中所有格都标记为占用。

---

### 1.3 动态障碍物 `dynamic_obstacles.json`

**任务书依据**：§三"满足多机安全约束" / §五.1"支持动态占用状态更新"

**作用**：定义移动障碍物（其他飞行器、鸟群等），只在轨迹时刻占用空间。

**单条障碍物字段**：

| 字段 | 类型 | 单位 | 必填 | 默认 | 说明 |
|------|------|------|------|------|------|
| `id` | string | - | 是 | - | 障碍物唯一 ID |
| `name` | string | - | 否 | "unnamed" | 显示名 |
| `type` | enum | - | 是 | - | `uav` / `vehicle` / `bird_flock` / `other` |
| `safety_radius` | float | 米 | 是 | 10 | 安全避让半径 |
| `source` | string | - | 否 | "file" | 数据源 |
| `trajectory` | array | - | 是 | - | 轨迹点列表 |

**轨迹点字段**：

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| `x` | float | 米 | X 坐标 |
| `y` | float | 米 | Y 坐标 |
| `z` | float | 米 | Z 坐标 |
| `t` | float | 秒 | 时间戳 |

**示例**（飞行器入侵者）：
```json
{
  "id": "uav-intruder-001",
  "name": "闯入无人机A",
  "type": "uav",
  "safety_radius": 15,
  "source": "radar",
  "trajectory": [
    {"x": 500,  "y": 2500, "z": 80, "t": 0},
    {"x": 1000, "y": 2500, "z": 80, "t": 30},
    {"x": 1500, "y": 2500, "z": 80, "t": 60}
  ]
}
```

**位置插值**：任意时刻的位置由 `position_at(t)` 通过轨迹点线性插值得到。

---

### 1.4 飞行器配置 `aircraft_config.json`

**任务书依据**：§五.3 飞行器性能数据（最大速度、爬升下降、加速度、转弯角、最小转弯半径、续航）

**作用**：定义每架无人机的物理参数、运动学约束、续航能力和安全参数。

**顶层结构**：`{"aircraft": [ {...}, ... ]}`

**完整字段**（5 分组）：

#### 顶层字段

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `id` | string | 是 | 飞行器唯一 ID（被任务引用） |
| `model` | string | 否 | 机型名 |
| `category` | enum | 否 | `heavy_payload` / `medium_payload` / `delivery` / `inspection` / `general` |
| `manufacturer` | string | 否 | 厂商 |

#### `physical`（物理参数）

| 字段 | 类型 | 单位 | 默认 | 说明 |
|------|------|------|------|------|
| `weight_kg` | float | kg | 1.0 | 自重 |
| `max_takeoff_weight_kg` | float | kg | 1.5 | 最大起飞重量 |
| `payload_capacity_kg` | float | kg | 0.5 | 有效载荷上限 |
| `wingspan_m` | float | 米 | 0.5 | 翼展 |
| `length_m` / `width_m` / `height_m` | float | 米 | 0.5/0.5/0.2 | 长宽高 |
| `rotor_radius_m` | float | 米 | 0.2 | 旋翼半径 |
| `diagonal_m` | float | 米 | 0.7 | 对角线 |

#### `performance`（运动性能）

| 字段 | 类型 | 单位 | 默认 | 说明 |
|------|------|------|------|------|
| `max_speed_mps` | float | m/s | 15 | 最大水平速度 |
| `max_climb_rate_mps` | float | m/s | 5 | 最大爬升率 |
| `max_descent_rate_mps` | float | m/s | 3 | 最大下降率 |
| `max_turn_rate_dps` | float | 度/秒 | 45 | 最大转弯率 |
| `max_acceleration_mps2` | float | m/s² | 3 | 最大加速度 |
| `max_deceleration_mps2` | float | m/s² | 2.5 | 最大减速度 |
| `min_turn_radius_m` | float | 米 | 0 | 最小转弯半径（0 表示不约束） |

#### `endurance`（续航与电源）

| 字段 | 类型 | 单位 | 默认 | 说明 |
|------|------|------|------|------|
| `max_flight_time_s` | float | 秒 | 1800 | 最大飞行时间 |
| `max_range_m` | float | 米 | 15000 | 最大航程 |
| `battery_capacity_wh` | float | Wh | 200 | 电池容量 |
| `cruise_power_w` | float | W | 500 | 巡航功率 |
| `hover_power_w` | float | W | 350 | 悬停功率 |

#### `operational`（运行环境）

| 字段 | 类型 | 单位 | 默认 | 说明 |
|------|------|------|------|------|
| `min_altitude_m` | float | 米 | 10 | 最低飞行高度 |
| `max_altitude_m` | float | 米 | 300 | 最高飞行高度 |
| `min_operating_temp_c` | float | ℃ | -10 | 最低工作温度 |
| `max_operating_temp_c` | float | ℃ | 40 | 最高工作温度 |
| `max_wind_resistance_mps` | float | m/s | 12 | 抗风能力 |
| `ip_rating` | string | - | "IP43" | 防护等级 |

#### `safety`（安全参数）

| 字段 | 类型 | 单位 | 默认 | 说明 |
|------|------|------|------|------|
| `rotor_radius_m` | float | 米 | 0.2 | 旋翼半径 |
| `safety_margin_m` | float | 米 | 5 | 安全裕度 |
| `geofence_compliant` | bool | - | true | 地理围栏合规 |
| `return_to_home_altitude_m` | float | 米 | 50 | 返航高度 |

**完整示例**（参见 `input/aircraft_config.json`）：
```json
{
  "id": "uav-001",
  "model": "DJI-Matrice-300",
  "category": "heavy_payload",
  "manufacturer": "DJI",
  "physical": {"weight_kg": 6.3, "max_takeoff_weight_kg": 9.0, "payload_capacity_kg": 2.7, ...},
  "performance": {"max_speed_mps": 18.0, "max_climb_rate_mps": 6.0, ...},
  "endurance": {"max_flight_time_s": 1800.0, "battery_capacity_wh": 274.0, ...},
  "operational": {"min_altitude_m": 10.0, "max_altitude_m": 300.0, ...},
  "safety": {"rotor_radius_m": 0.43, "safety_margin_m": 5.0, ...}
}
```

**兼容性**：同时支持旧版扁平字段（`max_speed` / `max_climb_rate` 等）作为 fallback。

---

### 1.5 飞行任务 `tasks.json`

**任务书依据**：§五.2 任务数据（编号、起终点、时间窗、优先级、多任务并发）

**作用**：定义要规划的飞行任务。`plan_multi.py` 会校验任务的飞行器需求兼容性。

**顶层结构**：`{"tasks": [ {...}, ... ]}`

**完整字段**：

#### 顶层字段

| 字段 | 类型 | 单位 | 必填 | 说明 |
|------|------|------|------|------|
| `id` | string | - | 是 | 任务唯一 ID |
| `name` | string | - | 否 | 任务名 |
| `aircraft_id` | string | - | 是 | 分配的执行飞机（必须存在于 aircraft_config） |
| `type` | enum | - | 否 | `delivery` / `inspection` / `patrol` / `surveillance` |
| `priority` | int (1-10) | - | 否 | 调度优先级（10最高） |
| `departure_time` | float | 秒 | 否 | 期望起飞时间 |
| `objective_weights` | object | - | 否 | 多目标权重（见下） |
| `waypoints` | array | - | 是 | 航点序列（含起终点） |
| `requirements` | object | - | 否 | 任务需求约束（见下） |

#### `objective_weights`（多目标权重）

| 键 | 含义 | 默认 |
|----|------|------|
| `distance` | 总航程最短 | 0.3 |
| `time` | 总飞行时间最短 | 0.3 |
| `risk` | 距障碍物风险最低 | 0.2 |
| `energy` | 估算能耗最低 | 0.2 |

权重之和必须 = 1.0。

#### `waypoints`（航点序列）

每个航点字段：

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| `id` | string | - | 航点 ID |
| `x`, `y`, `z` | float | 米 | 航点位置 |
| `type` | enum | - | `start` / `via` / `end` / `task` |
| `earliest_arrival` | float | 秒 | 最早到达时间 |
| `latest_arrival` | float | 秒 | 最晚到达时间 |
| `hover_time` | float | 秒 | 悬停时间 |
| `task_description` | string | - | 任务描述 |

#### `requirements`（任务需求）

`plan_multi.py` 会校验飞机性能是否满足：

| 字段 | 类型 | 单位 | 默认 | 说明 |
|------|------|------|------|------|
| `payload_kg` | float | kg | 0 | 需要的载荷重量 |
| `required_endurance_s` | float | 秒 | 0 | 需要最小续航 |
| `required_range_m` | float | 米 | 0 | 需要最小航程 |
| `required_max_speed_mps` | float | m/s | 0 | 需要最小最大速度 |
| `min_ceiling_m` | float | 米 | 0 | 需要最低飞行高度 |
| `delivery_window` | [float, float] | 秒 | null | 投递时间窗 |
| `priority_level` | int (1-5) | - | 1 | 业务优先级 |
| `service_type` | string | - | "delivery" | 服务类型 |

**校验规则**（`check_aircraft_compatibility`）：
```
if payload_kg > aircraft.physical.payload_capacity_kg: 错误
if required_endurance_s > aircraft.endurance.max_flight_time_s: 错误
if required_max_speed_mps > aircraft.performance.max_speed_mps: 错误
if min_ceiling_m > aircraft.operational.max_altitude_m: 错误
```

**完整示例**：
```json
{
  "id": "task-001",
  "name": "医疗物资配送",
  "aircraft_id": "uav-001",
  "type": "delivery",
  "priority": 9,
  "departure_time": 0,
  "objective_weights": {"distance": 0.2, "time": 0.5, "risk": 0.2, "energy": 0.1},
  "waypoints": [
    {"id": "start", "x": 500, "y": 500, "z": 80, "type": "start"},
    {"id": "end", "x": 600, "y": 2200, "z": 170, "type": "end"}
  ],
  "requirements": {
    "payload_kg": 1.5,
    "required_endurance_s": 300.0,
    "min_ceiling_m": 100.0,
    "delivery_window": [0, 120],
    "priority_level": 5,
    "service_type": "delivery"
  }
}
```

---

## 2. 输出文件（`output/`）

### 2.1 规划结果 `planning_result_*.json`

**任务书依据**：§十 输出要求全部字段

**完整结构**：
```json
{
  "request_id": "single-task-001",
  "total_routes": 2,
  "success_count": 2,
  "failed_count": 0,
  "total_planning_time_ms": 56.5,
  "routes": [
    {
      "route_id": "astar-task-001",
      "task_id": "task-001",
      "aircraft_id": "uav-001",
      "status": "planned",
      "message": "规划完成",
      "algorithm": "SpaceTimeAStar",
      "planning_time_ms": 15.2,
      "metrics": {
        "total_distance_m": 1900.0,
        "total_time_s": 200.0,
        "total_energy": 1900.0,
        "max_risk": 0.0,
        "avg_risk": 0.0,
        "min_obstacle_distance_m": 65.0
      },
      "trajectory": [
        {"x": 550.0, "y": 550.0, "z": 75.0, "t": 5.0, "heading": 45.0},
        {"x": 650.0, "y": 650.0, "z": 75.0, "t": 15.0, "heading": 45.0}
      ],
      "explanation": {
        "summary": "...",
        "score": 0.1234,
        "objective_breakdown": {"distance": 0.1, "time": 0.2, "risk": 0.05, "energy": 0.08},
        "avoidance_count": 3,
        "performance_margin": {"speed": 0.85, "climb": 0.50},
        "notes": []
      }
    }
  ]
}
```

**字段说明**：

| 字段 | 类型 | 说明 |
|------|------|------|
| `request_id` | string | 请求 ID（含任务 ID 或批次） |
| `total_routes` / `success_count` / `failed_count` | int | 数量统计 |
| `total_planning_time_ms` | float | 总 A* 耗时（毫秒） |
| `routes[].route_id` | string | 航线唯一 ID |
| `routes[].status` | enum | `planned` / `failed` / `partial` |
| `routes[].algorithm` | string | 使用的算法名 |
| `routes[].message` | string | 状态说明（成功时为说明，失败时为原因） |
| `routes[].metrics.total_distance_m` | float | 总航程（米） |
| `routes[].metrics.total_time_s` | float | 总飞行时间（秒） |
| `routes[].metrics.total_energy` | float | 估算能耗（相对值） |
| `routes[].metrics.max_risk` / `avg_risk` | float (0-1) | 风险值 |
| `routes[].metrics.min_obstacle_distance_m` | float | 距最近障碍物距离 |
| `routes[].trajectory[]` | array | 轨迹点序列 |
| `routes[].trajectory[].x/y/z` | float | 位置（米） |
| `routes[].trajectory[].t` | float | 时间（秒） |
| `routes[].trajectory[].heading` | float | 航向角（度） |
| `routes[].explanation` | object | 规划解释（任务书 §六） |

---

### 2.2 航点指令 `waypoints_*.json`

**任务书依据**：§十"航点序列" + §五.6"航点生成"

**完整结构**：
```json
{
  "generated_at": "2026-09-07T17:00:00",
  "waypoint_count": 23,
  "waypoints": [
    {
      "seq": 0,
      "x": 550.0, "y": 550.0, "z": 75.0,
      "t": 5.0, "heading": 45.0,
      "action": "takeoff"
    },
    {"seq": 1, ..., "action": "fly"},
    ...
    {"seq": 22, ..., "action": "land"}
  ]
}
```

**字段**：

| 字段 | 说明 |
|------|------|
| `seq` | 序号（从 0 开始） |
| `x`, `y`, `z` | 位置 |
| `t` | 时间（秒） |
| `heading` | 航向（度） |
| `action` | `takeoff`（起点）/ `fly`（巡航）/ `land`（终点） |

**生成规则**（`route_optimizer.generate_waypoints`）：
- 按最小间距（默认 80m）采样
- 航向变化超过 15° 时插入新航点
- 起点 action = "takeoff"
- 终点 action = "land"

---

### 2.3 单航线 `routes/*.json`

**作用**：每条航线单独存一个文件，便于追踪

**文件名**：`{algorithm}-{task_id}.json` 或 `{algorithm}-{task_id}-seg{i}.json`（多航点分段）

**结构**：与 §2.1 中的单条 route 完全一致

---

### 2.4 环境快照 `environment.json`

**作用**：保存输入数据快照，供 `visualize.py` 复现障碍物（无需原始 `input/` 目录）

**结构**：
```json
{
  "config": {...},                    // 同 config.json
  "static_obstacles": {...},         // 同 static_obstacles.json
  "dynamic_obstacles": {...}         // 同 dynamic_obstacles.json
}
```

---

### 2.5 冲突报告 `conflicts.json`

**任务书依据**：§五.4 多机航线冲突避免

**作用**：仅当 `plan_multi.py --conflict-check` 时生成

**结构**：
```json
[
  {
    "route_a": "route-001",
    "route_b": "aircraft:uav-002",
    "time": 152.5,
    "position_a": [550.0, 1500.0, 80.0],
    "position_b": [560.0, 1505.0, 82.0],
    "distance": 11.2
  }
]
```

---

### 2.6 Markdown 报告 `report_*.md`

**任务书依据**：§六 解释模块

**结构**：包含
1. 规划概览（请求 ID / 总任务数 / 成功率 / 总耗时）
2. 环境配置（空域范围 + 分辨率）
3. 障碍物信息
4. 各航线详情（每条单独一节，含状态 + 算法 + 耗时 + 指标）

---

### 2.7 压力测试结果 `stress/`

**目录结构**：
```
output/stress/
├── stress_results_full.json       # FULL 模式 4 规模原始数据
├── stress_results_dispatch.json   # DISPATCH 模式 4 规模原始数据
├── stress_results.json            # 主结果（最后一次跑）
├── STRESS_TEST_RESULTS.md         # 自动生成的简版 Markdown
├── STRESS_TEST_REPORT.md          # 详细压力测试报告（11 节）
├── console.log                     # 完整控制台输出
└── visualizations/                 # visualize_stress.py 输出
    ├── wall_clock.png             # 墙钟时间 vs N
    ├── speedup.png                # 加速比柱状图
    ├── success_rate.png           # 成功率对比
    ├── throughput.png             # 吞吐对比
    ├── full_vs_dispatch.png       # FULL vs DISPATCH 综合 4 子图
    └── task_time_percentiles.png  # P50/P95/P99/max 分布
```

**`stress_results_*.json` 字段**：

```json
[
  {
    "scale": 10000,
    "success_count": 9143,
    "failed_count": 857,
    "success_rate": 91.43,
    "total_planning_time_ms": 451670.0,
    "avg_planning_time_per_task_ms": 45.17,
    "min_task_time_ms": 0.01,
    "max_task_time_ms": 5998.02,
    "p50_task_time_ms": 50.9,
    "p95_task_time_ms": 4024.0,
    "p99_task_time_ms": 5069.0,
    "avg_distance_m": 3739.9,
    "avg_time_s": 382.8,
    "avg_min_obstacle_dist_m": 139.8,
    "wall_clock_time_s": 451.67,
    "throughput_tasks_per_s": 22.1,
    "conflict_count_sample200": 47,
    "workers_used": 8
  }
]
```

---

## 附录：单位约定

| 量 | 单位 |
|----|------|
| 距离 | 米 (m) |
| 速度 | 米/秒 (m/s) |
| 加速度 | 米/秒² (m/s²) |
| 时间 | 秒 (s) |
| 角度 | 度 (°) |
| 温度 | 摄氏度 (℃) |
| 电池 | 瓦时 (Wh) |
| 功率 | 瓦 (W) |
| 重量 | 千克 (kg) |

---

## 复现命令

```bash
conda activate uav_path_planning

# 单机规划
python plan_single.py --output-dir ./output/single

# 多机批量
python plan_multi.py --output-dir ./output/multi --conflict-check

# 压力测试
python -u run_stress_test.py --scales 10 100 1000 10000 --compare \
    --output ./output/stress
```