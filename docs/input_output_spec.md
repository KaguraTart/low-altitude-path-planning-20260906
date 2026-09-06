# 输入输出数据格式规范

> 对应需求：3.25 & 3.26 — "需提供基础障碍物数据（静态的、动态的）；输入数据格式及样例；输出格式及样例"

## 1. 输入数据

### 1.1 时空网格配置 (`config.json`)

定义规划的空间范围、分辨率和时间范围。

```json
{
  "x_min": 0,
  "x_max": 5000,
  "y_min": 0,
  "y_max": 5000,
  "z_min": 0,
  "z_max": 500,
  "dx": 100,
  "dy": 100,
  "dz": 50,
  "t_min": 0,
  "t_max": 1200,
  "dt": 10
}
```

| 字段 | 类型 | 说明 | 单位 |
|------|------|------|------|
| x_min/x_max | float | X 轴空间范围 | 米 |
| y_min/y_max | float | Y 轴空间范围 | 米 |
| z_min/z_max | float | Z 轴空间范围（高度） | 米 |
| dx/dy/dz | float | 空间网格分辨率 | 米/格 |
| t_min/t_max | float | 时间范围 | 秒 |
| dt | float | 时间步长 | 秒/步 |

### 1.2 静态障碍物数据 (`static_obstacles.json`)

建筑物、地形、通信塔等固定障碍物。

```json
{
  "obstacles": [
    {
      "id": "building-001",
      "name": "中心大厦",
      "type": "building",
      "x": 2500,
      "y": 2500,
      "z": 0,
      "width": 80,
      "depth": 80,
      "height": 200,
      "source": "city_gis"
    }
  ]
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| id | string | 障碍物唯一标识 |
| name | string | 障碍物名称 |
| type | string | 类型：building / tower / terrain / other |
| x, y, z | float | 障碍物中心位置（z 为底部高度） |
| width | float | X 方向半宽 |
| depth | float | Y 方向半深 |
| height | float | 障碍物高度 |
| source | string | 数据来源标识 |

### 1.3 动态障碍物数据 (`dynamic_obstacles.json`)

其他飞行器、鸟群等移动障碍物，带时间戳轨迹。

```json
{
  "obstacles": [
    {
      "id": "uav-intruder-001",
      "name": "闯入无人机A",
      "type": "uav",
      "safety_radius": 15,
      "source": "radar",
      "trajectory": [
        {"x": 500, "y": 2500, "z": 80, "t": 0},
        {"x": 1000, "y": 2500, "z": 80, "t": 30},
        {"x": 1500, "y": 2500, "z": 80, "t": 60}
      ]
    }
  ]
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| id | string | 障碍物唯一标识 |
| name | string | 障碍物名称 |
| type | string | 类型：uav / vehicle / bird_flock / other |
| safety_radius | float | 安全避让半径 |
| trajectory | array | 轨迹点列表，每点含 x,y,z,t |
| source | string | 数据来源标识 |

### 1.4 飞行器性能配置 (`aircraft_config.json`)

```json
{
  "aircraft": [
    {
      "id": "uav-001",
      "model": "DJI-Matrice-300",
      "max_speed": 18.0,
      "max_climb_rate": 6.0,
      "max_descent_rate": 4.0,
      "max_turn_rate": 60.0,
      "max_flight_time": 1800.0,
      "max_range": 15000.0,
      "min_altitude": 10.0,
      "max_altitude": 300.0
    }
  ]
}
```

| 字段 | 类型 | 说明 | 单位 |
|------|------|------|------|
| id | string | 飞行器唯一标识 |
| model | string | 机型 |
| max_speed | float | 最大水平速度 | m/s |
| max_climb_rate | float | 最大爬升率 | m/s |
| max_descent_rate | float | 最大下降率 | m/s |
| max_turn_rate | float | 最大转弯率 | 度/秒 |
| max_flight_time | float | 最大续航时间 | 秒 |
| max_range | float | 最大航程 | 米 |
| min_altitude | float | 最低飞行高度 | 米 |
| max_altitude | float | 最高飞行高度 | 米 |

### 1.5 飞行任务 (`tasks.json`)

```json
{
  "tasks": [
    {
      "id": "task-001",
      "name": "医疗物资配送",
      "aircraft_id": "uav-001",
      "type": "delivery",
      "priority": 9,
      "departure_time": 0,
      "objective_weights": {
        "distance": 0.2,
        "time": 0.5,
        "risk": 0.2,
        "energy": 0.1
      },
      "waypoints": [
        {"id": "start", "x": 500, "y": 500, "z": 80, "type": "start"},
        {"id": "end", "x": 600, "y": 2200, "z": 170, "type": "end", "task_description": "配送至楼顶"}
      ]
    }
  ]
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| id | string | 任务唯一标识 |
| name | string | 任务名称 |
| aircraft_id | string | 执行飞行器 ID |
| type | string | 任务类型：delivery / inspection / patrol / surveillance |
| priority | int | 优先级 1-10（10 最高） |
| departure_time | float | 期望起飞时间 |
| objective_weights | object | 多目标优化权重（和为 1.0） |
| waypoints | array | 航点序列（含起点终点） |

**航点字段：**

| 字段 | 类型 | 说明 |
|------|------|------|
| id | string | 航点 ID |
| x, y, z | float | 航点坐标 |
| type | string | start / end / via / task |
| earliest_arrival | float | 最早到达时间（可选） |
| latest_arrival | float | 最晚到达时间（可选） |
| hover_time | float | 悬停时间 |
| task_description | string | 任务描述 |

---

## 2. 输出数据

### 2.1 规划结果 (`planning_result_*.json`)

```json
{
  "request_id": "demo-both",
  "total_routes": 10,
  "success_count": 10,
  "failed_count": 0,
  "total_planning_time_ms": 56.5,
  "routes": [
    {
      "route_id": "opt-task-001",
      "task_id": "task-001",
      "aircraft_id": "uav-001",
      "status": "planned",
      "message": "规划完成 | 综合得分: 0.1234 | ...",
      "algorithm": "RouteOptimizer(MultiObjective)",
      "planning_time_ms": 12.3,
      "metrics": {
        "total_distance_m": 1870.4,
        "total_time_s": 150.8,
        "total_energy": 1870.4,
        "max_risk": 0.0,
        "avg_risk": 0.0,
        "min_obstacle_distance_m": 65.0
      },
      "trajectory": [
        {"x": 550.0, "y": 550.0, "z": 75.0, "t": 5.0, "heading": 45.0},
        {"x": 650.0, "y": 650.0, "z": 75.0, "t": 15.0, "heading": 45.0}
      ]
    }
  ]
}
```

### 2.2 航点指令 (`waypoints_*.json`)

从连续轨迹提取的可执行航点序列。

```json
{
  "generated_at": "2026-09-06T20:00:00",
  "waypoint_count": 23,
  "waypoints": [
    {"seq": 0, "x": 550.0, "y": 550.0, "z": 75.0, "t": 5.0, "heading": 45.0, "action": "takeoff"},
    {"seq": 1, "x": 650.0, "y": 650.0, "z": 75.0, "t": 15.0, "heading": 45.0, "action": "fly"},
    {"seq": 22, "x": 650.0, "y": 2250.0, "z": 175.0, "t": 155.0, "heading": 90.0, "action": "land"}
  ]
}
```

### 2.3 规划报告 (`report_*.md`)

Markdown 格式的完整规划报告，包含环境配置、障碍物信息、每条航线的详细指标。

### 2.4 可视化输出

| 文件 | 格式 | 说明 |
|------|------|------|
| `routes_3d.png` | PNG | Matplotlib 3D 静态图（论文/报告用） |
| `routes_interactive.html` | HTML | Plotly 交互式 3D 可视化（演示用） |
| `risk_heatmap.png` | PNG | 风险热力图（2D 俯视图） |
