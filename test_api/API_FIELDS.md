# 接口字段与修改说明

文档日期：2026-10-30。

本说明解释 `test_api` 测试 JSON 的输入、输出和修改方法。服务地址为 `https://api.kaguratart.com`。Swagger 地址为 [在线接口文档](https://api.kaguratart.com/docs#/)。

按业务需求查找接口，见[需求与 API 对照表](REQUIREMENTS_API_MAPPING.md)。

- [复制并修改一份请求](#edit-request)
- [全部接口与请求结构](#endpoints)
- [任务、航点与当前状态](#task)
- [轨迹、运动段与验证结果](#route)
- [场景与空间对象](#scene)
- [坐标与高度换算](#coordinates)
- [无人机参数](#profiles)
- [网格输入](#grid)
- [20 类规则](#rules)

字段名区分大小写。JSON 数值直接写数字。布尔值写 `true` 或 `false`。表中的“默认值”表示省略字段后的取值。`null` 表示该字段没有值，`[]` 表示空数组。

## 通用约定

| 项目 | 填写方法 |
|---|---|
| 请求体 | 使用 JSON。发送 `Content-Type: application/json` |
| WKT 点 | 使用 `POINT Z (经度 纬度 高度)` |
| WKT 线路 | 使用 `LINESTRING Z (经度 纬度 高度, 经度 纬度 高度, ...)` |
| WKT 区域 | 使用 `POLYGON` 或 `MULTIPOLYGON`。另填区域高度范围和高度基准 |
| 经纬度 | 使用 WGS84。先写经度，再写纬度，单位为度 |
| 局部坐标 | X 向东，Y 向北。Z 等于 EG 椭球高减 `origin_alt_ellipsoid`，单位为米 |
| 高度 | 明确填写 `EG`、`MSL`、`AGL`、`ELLIPSOID`、`QNE` 或 `QNH` |
| 其他单位 | 距离用米；时间用秒；速度用 m/s；加速度用 m/s²；载荷用 kg |
| 任务时间 | 使用带时区的 ISO 8601，例如 `2026-10-18T09:00:00+08:00` |
| 相对时间 | 轨迹和运动段的秒数以 `scene.time_origin` 为零点 |
| 对象引用 | 使用同一场景中的对象 `id`，例如 `profile_id`、`rule_ids`、`airway_ids` |
| 场景选择 | 单机、批量和验证请求填写 `scene` 或 `scene_id` 中的一项 |

<a id="edit-request"></a>
## 复制并修改一份请求

### 选择输入文件

| 文件 | 修改内容 |
|---|---|
| [`data/examples/single_strict_request.json`](data/examples/single_strict_request.json) | 严格航路单机任务、完整场景、机型和时间 |
| [`data/examples/single_prefer_request.json`](data/examples/single_prefer_request.json) | 优先航路任务 |
| [`data/examples/single_free_request.json`](data/examples/single_free_request.json) | 自由规划任务 |
| [`data/examples/batch_request.json`](data/examples/batch_request.json) | 批量任务数组，同一请求也用于异步作业 |
| [`data/examples/coordinates_to_local_request.json`](data/examples/coordinates_to_local_request.json) | 坐标转换方向、原点、高度基准和几何 |
| [`data/examples/validate_request.json`](data/examples/validate_request.json) | 原始任务、完整轨迹、时间数组和运动段 |
| [`data/benchmarks/strict_scene.json`](data/benchmarks/strict_scene.json)、[`mixed_scene.json`](data/benchmarks/mixed_scene.json)、[`free_scene.json`](data/benchmarks/free_scene.json) | 三套场景。文件内容直接作为场景登记请求体 |
| [`data/benchmarks/strict_tasks.json`](data/benchmarks/strict_tasks.json)、[`mixed_tasks.json`](data/benchmarks/mixed_tasks.json)、[`free_tasks.json`](data/benchmarks/free_tasks.json) | 三套任务。将数组放入批量请求的 `tasks` 字段 |

`run_tests.py` 使用 `data/manifest.json` 校验随包输入。自定义请求使用副本。下面的命令读取原始示例，在内存中修改请求，并将实际输入和输出保存到 `results/`。

### 修改坐标、时间、机型和航点

在 `test_api` 目录执行以下命令。脚本使用本目录的 `client.py`，只依赖 Python 标准库。

本例将终点改到局部坐标 `[800, 100, 80]`。该点位于示例航路内。任务在中间航点悬停 5 秒。巡航速度改为 5 m/s。

```bash
python - <<'PY'
from datetime import datetime, timezone
import json
import os
from pathlib import Path
from client import HTTPClient

request = json.loads(Path("data/examples/single_strict_request.json").read_text(encoding="utf-8"))
output = Path("results") / ("custom-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ"))
client = HTTPClient(os.environ.get("API_BASE_URL", "https://api.kaguratart.com"), output)

converted = client.request("POST", "/api/v2/coordinates/convert", {
    "coordinates": request["scene"]["coordinates"],
    "direction": "local_to_wgs84",
    "alt_reference": "EG",
    "geometries_wkt": [
        "POINT Z (100 100 80)",
        "POINT Z (450 100 80)",
        "POINT Z (800 100 80)"
    ]
})
start, middle, goal = converted["geometries_wkt"]
request["task"]["rule_ids"] = [
    rule["id"] for rule in request["scene"]["rules"]
    if rule.get("rule_type", rule.get("type")) in {"SPEED", "HOVER"}
]
request["task"].update({
    "id": "integration-demo-001",
    "start_wkt": start,
    "goal_wkt": goal,
    "departure_time": request["scene"]["time_origin"],
    "priority": 10,
    "aircraft": {"max_speed_mps": 10, "cruise_speed_mps": 5},
    "waypoints": [{"wkt": middle, "hover": True, "hover_s": 5}]
})
result = client.request("POST", "/api/v2/plan/single", request)
print("状态：", result["status"])
print("终点：", result["goal_wkt"])
print("指标：", result["metrics"])
print("完整输入与输出：", output / "http")
PY
```

本例选择 SPEED/HOVER 业务规则，并继续执行机型、航路和默认爬降/转弯约束。提交后检查实际 `status` 与 `validation.valid`；航时以返回的 `metrics` 为准。

本例的局部坐标单位为米。更换业务地点时，先设置场景原点，再通过坐标转换接口生成经纬度 WKT。

### 按业务修改参数

| 目标 | 修改位置 | 同时调整 |
|---|---|---|
| 更换业务区域 | `scene.coordinates.origin_lon`、`origin_lat` 和 `scene.bounds` | 将航路、接驳区、障碍、管制区、数据集和任务几何移到同一业务区域 |
| 更改起终点 | `task.start_wkt`、`goal_wkt` | `strict` 起终点放在有效航路或对应接驳区内；航路外起降填写接驳 ID |
| 切换航路模式 | `task.route_mode` | `strict` 沿航路飞行；`prefer` 优先航路；`free` 使用合法三维空域 |
| 更改飞行高度 | 点 WKT 的 Z、`task.alt_reference` | 航路高度带、场景 Z 范围、高度规则和所需的高度转换数据 |
| 更改出发时间 | `task.departure_time` | 使时间落在 `scene.time_origin` 与 `horizon_s` 覆盖的时段。同步调整到达窗、航点窗、规则和数据有效期 |
| 设置到达时间窗 | `task.earliest_arrival`、`latest_arrival` | 时间使用相同时区约定，并为飞行和悬停保留时长 |
| 更改机型性能 | `task.aircraft`，或 `scene.profiles` 与 `task.profile_id` | 巡航速度、最大速度、升降率、加减速、净空和续航一起匹配飞机能力 |
| 设置返航 | `task.state` 和 `RTL` 规则的 `content` | 填当前电量、剩余时间、剩余航程、返航点和返航高度 |
| 增加批量任务 | `tasks` 数组 | 为每个任务填写唯一 `id`。同一批任务共用一个场景 |
| 修改已登记场景 | 修改完整场景 JSON，再次 `POST /api/v2/scenes` | 后续请求改用本次返回的 `scene_id` |
| 复核规划输出 | 验证请求中的任务、场景、轨迹、时间数组和运动段 | 从同一次规划响应复制完整 `trajectory_wkt`、`vertex_times_s`、`motion_segments` |

<a id="endpoints"></a>
## 接口与完整记录

接口地址为 `https://api.kaguratart.com`。下表中的路径接在这个地址后面。POST 请求使用 `Content-Type: application/json`。

| 方法 | 路径 | 输入 | 输出 | 成功状态 |
|---|---|---|---|---|
| GET | `/api/v2/health` | 无请求体 | [健康状态](#health) | 200 |
| GET | `/api/v2/health/jobs` | 无请求体 | [作业计数](#job-counts) | 200 |
| GET | `/api/v2/version` | 无请求体 | [版本与默认机型](#version) | 200 |
| GET | `/api/v2/capabilities` | 无请求体 | [接口能力](#capabilities) | 200 |
| POST | `/api/v2/scenes` | [Scene 场景](#scene) | [场景登记结果](#scene-response) | 200 |
| GET | `/api/v2/scenes/{scene_id}` | 路径中的 `scene_id` | [场景登记结果](#scene-response) | 200 |
| POST | `/api/v2/coordinates/convert` | [坐标转换请求](#coordinates) | [坐标转换结果](#coordinates) | 200 |
| POST | `/api/v2/plan/single` | [单任务请求](#single-request) | [路线结果](#route) | 200 |
| POST | `/api/v2/plan/batch` | [批量请求](#batch-request) | [批量结果](#batch-result) | 200 |
| POST | `/api/v2/validate` | [轨迹验证请求](#validate-request) | [验证结果](#validation)及规则报告 | 200 |
| POST | `/api/v2/jobs` | [批量请求](#batch-request) | [作业提交结果](#job-submitted) | 202 |
| GET | `/api/v2/jobs/{job_id}` | 路径中的 `job_id` | [作业结果](#job-result) | 200 |
| GET | `/api/v2/examples/single` | 无请求体 | [完整单任务请求](#single-request) | 200 |
| GET | `/api/v2/examples/batch` | 无请求体 | [完整批量请求](#batch-request) | 200 |
| POST | `/api/v2/networks/analyze` | scene 或 scene_id、可选 at | 图版本、节点/有效边、强弱连通分量、失效边 | 200 |
| POST | `/api/v2/plan/replan` | 原始场景/任务/完整路线、当前状态、剩余任务、最新场景、事件 | decision、完整新路线、复用/搜索指标及触发原因 | 200 |

`200` 表示接口已处理请求。读取 `RouteResult.status` 确认路线结果。读取 `BatchResult.success_count` 和 `failed_count` 确认批量结果。读取 `ValidationResponse.valid` 确认验证结果。

当前契约的完整记录见[最新测试索引](reports/README.md)。下列链接保留历史 HTTP 记录。GET 请求的输入文件保存方法、地址和请求头。POST 请求的输入文件保存实际 JSON 请求体。

| 接口 | 完整输入 | 完整输出 |
|---|---|---|
| GET health | [请求描述](reports/20261017T135804Z_public/http/001_get_api_v2_health_request.json) | [输出](reports/20261017T135804Z_public/http/001_get_api_v2_health_output.json) |
| GET health/jobs | [请求描述](reports/20261017T135804Z_public/http/002_get_api_v2_health_jobs_request.json) | [输出](reports/20261017T135804Z_public/http/002_get_api_v2_health_jobs_output.json) |
| GET version | [请求描述](reports/20261017T135804Z_public/http/003_get_api_v2_version_request.json) | [输出](reports/20261017T135804Z_public/http/003_get_api_v2_version_output.json) |
| GET capabilities | [请求描述](reports/20261017T135804Z_public/http/004_get_api_v2_capabilities_request.json) | [输出](reports/20261017T135804Z_public/http/004_get_api_v2_capabilities_output.json) |
| POST scenes | [输入](reports/20261017T135804Z_public/http/019_post_api_v2_scenes_input.json) | [输出](reports/20261017T135804Z_public/http/019_post_api_v2_scenes_output.json) |
| GET scenes/{scene_id} | [请求描述](reports/20261017T135804Z_public/http/020_get_api_v2_scenes_scene-5c4cc87b4e9bf08cd5a7_request.json) | [输出](reports/20261017T135804Z_public/http/020_get_api_v2_scenes_scene-5c4cc87b4e9bf08cd5a7_output.json) |
| POST coordinates/convert | [输入](reports/20261017T135804Z_public/http/023_post_api_v2_coordinates_convert_input.json) | [输出](reports/20261017T135804Z_public/http/023_post_api_v2_coordinates_convert_output.json) |
| POST plan/single | [输入](reports/20261017T135804Z_public/http/012_post_api_v2_plan_single_input.json) | [输出](reports/20261017T135804Z_public/http/012_post_api_v2_plan_single_output.json) |
| POST plan/batch | [输入](reports/20261017T135804Z_public/http/018_post_api_v2_plan_batch_input.json) | [输出](reports/20261017T135804Z_public/http/018_post_api_v2_plan_batch_output.json) |
| POST validate | [输入](reports/20261017T135804Z_public/http/025_post_api_v2_validate_input.json) | [输出](reports/20261017T135804Z_public/http/025_post_api_v2_validate_output.json) |
| POST jobs | [输入](reports/20261017T135804Z_public/http/026_post_api_v2_jobs_input.json) | [输出](reports/20261017T135804Z_public/http/026_post_api_v2_jobs_output.json) |
| GET jobs/{job_id} | [请求描述](reports/20261017T135804Z_public/http/027_get_api_v2_jobs_d877fe14f8bb42e5_request.json) | [输出](reports/20261017T135804Z_public/http/027_get_api_v2_jobs_d877fe14f8bb42e5_output.json) |
| GET examples/single | [请求描述](reports/20261017T135804Z_public/http/007_get_api_v2_examples_single_request.json) | [输出](reports/20261017T135804Z_public/http/007_get_api_v2_examples_single_output.json) |
| GET examples/batch | [请求描述](reports/20261017T135804Z_public/http/010_get_api_v2_examples_batch_request.json) | [输出](reports/20261017T135804Z_public/http/010_get_api_v2_examples_batch_output.json) |

### 选择场景

单任务、批量和验证请求都使用下面两个字段。每次选择一个字段。

| 字段 | 类型 | 必填与默认值 | 填写方法 |
|---|---|---|---|
| `scene` | Scene 对象 | 与 `scene_id` 二选一 | 填入完整场景。服务登记并编译该场景。 |
| `scene_id` | 字符串 | 与 `scene` 二选一 | 填入 `POST /api/v2/scenes` 返回的场景 ID。 |

使用内嵌场景时，直接修改 `scene` 内的场景字段。使用登记场景时，修改场景 JSON 后重新登记，再填写新返回的 `scene_id`。`scene_id` 与 `scene_version` 由服务生成。场景保存在当前服务进程中，服务重启后重新登记。

<a id="single-request"></a>
### POST /api/v2/plan/single：单任务请求

| 字段 | 类型 | 必填与默认值 | 填写方法 |
|---|---|---|---|
| `scene` | Scene 对象 | 与 `scene_id` 二选一 | 见[场景字段](#scene)。 |
| `scene_id` | 字符串 | 与 `scene` 二选一 | 使用登记接口返回值。 |
| `task` | Task 对象 | 必填 | 见[任务字段](#task)。 |
| `time_budget_s` | 正数或 `null` | 默认 `null` | 秒。`null` 表示不设置服务端计算时限。填写正数后，服务按该计算时限处理本次请求。 |

输入示例：[single_strict_request.json](data/examples/single_strict_request.json)。成功响应为[路线结果](#route)，HTTP 状态为 `200`。

`time_budget_s` 控制服务端计算时间。测试脚本的 `--timeout` 控制客户端等待 HTTP 响应的时间。这两个参数分别设置。

<a id="batch-request"></a>
### POST /api/v2/plan/batch：批量请求

| 字段 | 类型 | 必填与默认值 | 填写方法 |
|---|---|---|---|
| `scene` | Scene 对象 | 与 `scene_id` 二选一 | 全部任务共用此场景。 |
| `scene_id` | 字符串 | 与 `scene` 二选一 | 使用登记接口返回值。 |
| `tasks` | Task 数组 | 必填，至少一个任务 | 每个对象是一条任务。为各任务填写不同的 `id`。 |
| `request_id` | 字符串或 `null` | 默认 `null` | 填写业务批次编号。省略或填空字符串时，服务生成编号。 |
| `execution` | ExecutionOptions 或 null | 默认 null | 设置候选计算 workers 与 batch_size；统一预约全部任务。 |
| `time_budget_s` | 正数或 `null` | 默认 `null` | 秒。作用于整个批次。 |

输入示例：[batch_request.json](data/examples/batch_request.json)。成功响应为[批量结果](#batch-result)，HTTP 状态为 `200`。

服务按 `priority` 从大到小安排任务。同优先级时，先安排 `latest_arrival` 较早的任务，再按任务 `id` 排序。响应 `routes` 按输入任务顺序返回。

三套 1000 任务分别使用 [strict](data/benchmarks/strict_tasks.json)、[mixed](data/benchmarks/mixed_tasks.json) 和 [free](data/benchmarks/free_tasks.json)。这些文件的顶层是任务数组。把数组放入 `tasks`，再填入对应场景的 `scene_id`，即可组成完整批量请求。

<a id="task"></a>
### Task：任务字段

| 字段 | 类型 | 必填与默认值 | 含义与修改方法 |
|---|---|---|---|
| `id` | 非空字符串 | 必填 | 任务编号。结果中的 `task_id` 对应此值。 |
| `engine_id` | ENG-06/ENG-07 或 null | 自动推断 | strict/prefer 对应 ENG-06；free 对应 ENG-07。显式输入须一致。 |
| `network` | NetworkSelection 或 null | 默认 null | ENG-06 的图节点及权重选择，见本文件航路图说明。 |
| `start_grid/goal_grid` | 三个整数的数组或 null | 默认 null | 起终点的外部网格索引，省略 WKT 时使用格中心。 |
| `search` | SearchOptions 或 null | 默认 null | 网格搜索算法、邻域、层级及可选扩展预算。 |
| `objectives` | ObjectiveConfig 或 null | 默认 null | ENG-07/free 的多目标权重、归一化、能耗模型和自定义代价，见多目标参数。 |
| `start_wkt` | `POINT Z` 字符串或 null | 默认 null；端点选择必填一项 | 起点。使用 network 或 start_grid 时可省略。顺序为经度、纬度、高度。经纬度单位为度，高度单位为米。 |
| `goal_wkt` | `POINT Z` 字符串或 null | 默认 null；端点选择必填一项 | 任务终点。使用 network 或 goal_grid 时可省略。触发返航时，服务按 `state.home_wkt` 生成返航路线。 |
| `departure_time` | 带时区时间字符串 | 必填 | 允许开始任务的时刻，如 `2026-10-16T09:00:00+08:00`。服务可为避让安排较晚起飞。 |
| `alt_reference` | 枚举字符串 | 必填 | 任务 WKT 的高度基准：`EG`、`MSL`、`AGL`、`ELLIPSOID`、`QNE`、`QNH`。见[坐标与高度](#coordinates)。 |
| `route_mode` | 枚举字符串 | 默认 `strict` | `strict` 使用有效航路与已授权接驳；`prefer` 优先使用航路；`free` 使用合法自由空域。 |
| `waypoints` | Waypoint 数组 | 默认 `[]` | 按数组顺序经过业务航点。见[航点字段](#waypoint)。 |
| `earliest_arrival` | 带时区时间或 `null` | 默认 `null` | 最早到达终点的时刻。 |
| `latest_arrival` | 带时区时间或 `null` | 默认 `null` | 最晚到达终点的时刻。填写时，它应不早于 `departure_time` 和 `earliest_arrival`。 |
| `profile_id` | 字符串 | 默认 `dji_mini_4_pro` | 引用 `scene.profiles[].id`。默认 ID 可直接使用内置机型。 |
| `aircraft` | AircraftProfile 对象或 `null` | 默认 `null` | 填写当前任务的机型参数。填写后，服务优先使用此对象。省略字段采用机型模型默认值。见[机型字段](#profiles)。 |
| `state` | AircraftState 对象或 `null` | 默认 `null` | 当前电量、剩余能力和返航点。见[状态字段](#state)。 |
| `payload_kg` | 非负数 | 默认 `0` | 本任务实际外加载荷，单位 kg。机型的同名字段表示载荷能力。 |
| `priority` | 整数 | 默认 `0` | 批量调度优先级。数值越大，越先安排。 |
| `start_connector_id` | 字符串或 `null` | 默认 `null` | 引用 `scene.connectors[].id`，授权起点接入航路。 |
| `end_connector_id` | 字符串或 `null` | 默认 `null` | 引用 `scene.connectors[].id`，授权从航路离开并到达终点。 |
| `rule_ids` | 字符串数组或 `null` | 默认 `null` | `null` 选择场景全部规则；数组选择列出的规则 ID；`[]` 选择零条场景规则。航路几何、障碍和机型约束继续执行。 |

修改起终点时，替换 `POINT Z (...)` 内的三个数值。修改飞行高度时，同时检查场景高度范围、航路高度带和任务 `alt_reference`。使用 `strict` 时，把起终点放在航路内，或填写对应接驳区 ID。

`prefer` 的航路外基础通行代价为航路内的 2 倍。三种模式都执行管制区、选定规则和机型约束。

<a id="waypoint"></a>
### Waypoint：业务航点

| 字段 | 类型 | 必填与默认值 | 含义 |
|---|---|---|---|
| `wkt` | `POINT Z` 字符串 | 必填 | 航点位置。使用任务的 `alt_reference`。 |
| `earliest` | 带时区时间或 `null` | 默认 `null` | 最早经过航点的时刻。 |
| `latest` | 带时区时间或 `null` | 默认 `null` | 最晚经过航点的时刻。填写两个时刻时，`latest` 应不早于 `earliest`。 |
| `hover_s` | 非负数 | 默认 `0` | 在该航点悬停的秒数。 |
| `hover` | 布尔值 | 默认 `false` | `true` 且 `hover_s=0` 时，使用选定 HOVER 规则的最大 `defaultHoverTime`。直接指定时长时填写正数 `hover_s`。 |

为任务添加一个业务航点时，在 `task.waypoints` 中添加对象。返回结果中的 `waypoint_index=0` 对应数组第一个航点。

填写正数 `hover_s` 可直接指定停留时长。使用规则默认时长时，填写 `hover=true` 并选择对应 HOVER 规则。默认时长为 0 时，停留时间为 0。

<a id="state"></a>
### AircraftState：任务状态

| 字段 | 类型 | 必填与默认值 | 含义与修改方法 |
|---|---|---|---|
| `at` | 带时区时间或 `null` | 默认 `null` | 状态记录时刻。服务转换返航点高度时使用此时刻；省略时使用任务 `departure_time`。 |
| `battery_percent` | 数值或 `null` | 默认 `null` | 电量百分比，取值 `0` 到 `100`。 |
| `remaining_flight_time_s` | 非负数或 `null` | 默认 `null` | 上游给出的剩余可飞时间，单位秒。 |
| `remaining_range_m` | 非负数或 `null` | 默认 `null` | 上游给出的剩余可飞航程，单位米。 |
| `home_wkt` | `POINT Z` 字符串或 `null` | 默认 `null` | 返航点，经度、纬度、高度。 |
| `alt_reference` | 高度基准或 `null` | 默认 `null` | `home_wkt` 的高度基准。省略时使用任务 `alt_reference`。 |

启用低电量返航规则时，填写电量、剩余时间、剩余航程和返航点。服务使用规则中的 `lowBatteryPercent` 判断是否触发。触发后，返回 `action="return_to_home"`，终点改为返航点，原任务途经点清空。返回 `goal_wkt` 使用任务的高度基准表达换算后的返航点。

<a id="route"></a>
### RouteResult：单条路线结果

每条结果显式返回 `engine_id`、`alt_reference`、`coordinate_context`。包括失败结果。WKT 采用任务请求的高度基准；运动段采用局部 EG 米制坐标。

| 字段 | 类型 | 含义 |
|---|---|---|
| `task_id` | 字符串 | 输入任务 `id`。 |
| `network_route` | NetworkRoute 或 null | 图版本、节点/边序列、边权字段、原始及有效代价。 |
| `search_report` | SearchReport 或 null | 显式算法、邻域、层级及实际扩展数。 |
| `traversed_grids` | TraversedGrid 数组 | 实际全部途经网格、上游编码及进入/离开时刻。 |
| `grid_context` | JSON 对象 | 完整途经网格的坐标框架、层级、分辨率及边界规则。 |
| `objectives` | RouteObjectives 或 null | 原始/归一化目标值、单位、权重、总分、公式和数据版本。 |
| `optimization` | OptimizationReport 或 null | 候选生成状态、迭代、候选与最终方案数量。 |
| `alternatives` | RouteResult 数组 | 其他非支配完整方案，默认不参与执行。 |
| `status` | 枚举字符串 | `planned`：已生成路线；`failed`：本任务规划失败；`timeout`：达到请求指定的计算时限。 |
| `code` | 字符串或 `null` | 路线失败原因码。成功通常为 `null`。见[结果与错误状态](#errors)。 |
| `message` | 字符串 | 路线结果说明。成功通常为空字符串。 |
| `route_mode` | 字符串 | 本任务使用的 `strict`、`prefer` 或 `free`。 |
| `trajectory_wkt` | WKT 字符串或 `null` | 飞行轨迹。移动轨迹使用 `LINESTRING Z`；单点轨迹使用 `POINT Z`。失败路线为 `null`。 |
| `start_wkt` | `POINT Z` 字符串 | 本次路线的起点。 |
| `goal_wkt` | `POINT Z` 字符串 | 本次路线的终点。返航路线使用转换后的返航点。 |
| `departure_time` | 带时区时间字符串 | 输入任务的允许起飞时刻。实际开始时刻由首个 `vertex_times_s` 给出。 |
| `time_origin` | 带时区时间或 `null` | 数值时间的零点。规划成功时对应 `scene.time_origin`。 |
| `action` | 枚举字符串 | `mission`：执行任务；`return_to_home`：执行返航。 |
| `vertex_times_s` | 数值数组 | WKT 顶点逐一对应的时间。单位秒，相对 `time_origin`。 |
| `initial_velocity_xyz_mps` | 三个数值 | 局部东/北/上初始速度，m/s；复制到独立验证请求。 |
| `motion_segments` | MotionSegment 数组 | 实际运动规律。包含加速、巡航、减速、转弯和等待。见[运动段](#motion-segment)。 |
| `keypoints` | Keypoint 数组 | 起终点、转弯、升降、悬停、业务航点和航路切换事件。见[关键点](#keypoint)。 |
| `metrics` | RouteMetrics 对象 | 航程、飞行时间和调度延迟。见[指标](#metrics)。 |
| `validation` | RouteValidation 对象 | 此路线的验证状态与违规项。见[验证结果](#validation)。 |
| `rule_report` | RuleReport 数组 | 场景各输入规则的执行情况。见[规则执行报告](#rule-report)。 |

数值时间都从场景 `time_origin` 开始计秒。例如零点为 09:00，首个顶点时间为 `300`，则实际开始时刻为 09:05。WKT 顶点数与 `vertex_times_s` 数量相同。多顶点轨迹的时间严格递增。悬停段可使用相同位置和不同时间表达。

失败路线保留任务编号、起终点、模式和原因。其轨迹字段通常为 `null`，数组通常为空。批量响应把这些路线计入 `failed_count`。

<a id="motion-segment"></a>
### MotionSegment：运动段

运动段的位置数组使用场景的局部米制坐标，顺序为 `[east_m, north_m, ellipsoid_minus_origin_m]`。轨迹 WKT 使用 WGS84。使用[坐标转换接口](#coordinates)在两种表达之间换算。

| 字段 | 类型与默认值 | 含义 |
|---|---|---|
| `airway_ids` | 字符串数组，默认 `[]` | 该运动段使用的航路 ID。 |
| `kind` | 必填枚举 | `line` 直线；`arc` 圆弧；`climb` 爬升；`descent` 下降；`hover` 悬停；`yaw` 原地转向。 |
| `start_time_s` | 必填非负数 | 段开始时刻，相对场景零点的秒数。 |
| `end_time_s` | 必填非负数 | 段结束时刻，相对场景零点的秒数。 |
| `start` | 必填三元素数值数组 | 局部坐标起点，单位米。 |
| `end` | 必填三元素数值数组 | 局部坐标终点，单位米。 |
| `length_m` | 非负数，默认 `0` | 实际运动段长度，单位米。圆弧段使用弧长。 |
| `distance_m` | 非负数，默认 `0` | 辅助字段。当前规划输出为 `0`。读取 `length_m` 获取段长。 |
| `points` | 三元素数值数组的数组，默认 `[]` | 辅助点列表，单位米。当前规划输出为空数组。 |
| `times_s` | 数值数组，默认 `[]` | 辅助点时间，单位秒。当前规划输出为空数组。 |
| `max_speed_mps` | 非负数或 `null` | 辅助速度字段，单位 m/s。当前规划输出为 `null`。 |
| `radius_m` | 非负数或 `null` | 圆弧半径，单位米。圆弧段填写数值。 |
| `center` | 三元素数值数组或 `null` | 圆弧圆心的局部坐标，单位米。 |
| `start_speed_mps` | 非负数或 `null` | 段起点速度，单位 m/s。 |
| `end_speed_mps` | 非负数或 `null` | 段终点速度，单位 m/s。 |
| `acceleration_mps2` | 数值或 `null` | 加速阶段使用的加速度，单位 m/s²。 |
| `deceleration_mps2` | 非负数或 `null` | 减速阶段的减速度大小，单位 m/s²。当前服务将有效数值写在 `profile` 中。 |
| `peak_speed_mps` | 非负数或 `null` | 峰值速度，单位 m/s。当前服务将有效数值写在 `profile` 中。 |
| `accel_time_s` | 非负数或 `null` | 加速时长，单位秒。当前服务将有效数值写在 `profile` 中。 |
| `cruise_time_s` | 非负数或 `null` | 匀速时长，单位秒。当前服务将有效数值写在 `profile` 中。 |
| `decel_time_s` | 非负数或 `null` | 减速时长，单位秒。当前服务将有效数值写在 `profile` 中。 |
| `sweep_angle_deg` | 数值或 `null` | 圆弧转过的有符号角度，单位度。当前服务将有效数值写在 `profile` 中。 |
| `profile` | JSON 对象，默认 `{}` | 连续运动参数，见下表。 |

当前规划结果在 `profile` 中给出下列字段。验证接口读取这些字段；同名字段同时出现在 `profile` 和顶层时，使用 `profile` 中的值。

| `profile` 内字段 | 单位 | 含义 |
|---|---|---|
| `peak_speed_mps` | m/s | 该段达到的峰值速度。 |
| `deceleration_mps2` | m/s² | 减速大小。 |
| `accel_time_s` | s | 加速阶段时长。 |
| `cruise_time_s` | s | 匀速阶段时长。 |
| `decel_time_s` | s | 减速阶段时长。 |
| `sweep_angle_deg` | 度 | 圆弧有符号转角。非圆弧段通常为 `0`。 |

调用验证接口时，把规划响应中的 `motion_segments` 原样复制。同时复制对应 `trajectory_wkt` 和 `vertex_times_s`。这样三个字段描述同一次运动。

<a id="keypoint"></a>
### Keypoint：关键点

| 字段 | 类型 | 含义 |
|---|---|---|
| `types` | 字符串数组 | 此点的事件类型。同一点可同时发生多种事件。 |
| `wkt` | `POINT Z` 字符串 | 关键点的 WGS84 位置。高度使用任务基准。 |
| `time_s` | 非负数 | 事件时刻，相对场景零点的秒数。 |
| `grid_indices` | 三整数数组或 `null` | 场景内部网格索引 `[i,j,k]`。索引从零开始，使用场景 `bounds` 与 `resolution` 计算。 |
| `grid_code` | 字符串或 `null` | 上游网格编码。场景提供对应编码时返回。 |
| `waypoint_index` | 非负整数或 `null` | 业务航点在 `task.waypoints` 中的索引。第一个航点为 `0`。 |

当前生成的类型包括 `START`、`GOAL`、`WAYPOINT`、`TURN_START`、`TURN_END`、`CLIMB_START`、`CLIMB_END`、`DESCENT_START`、`DESCENT_END`、`HOVER_START`、`HOVER_END` 和 `AIRWAY_SWITCH`。

<a id="metrics"></a>
### RouteMetrics：路线指标

| 字段 | 单位 | 含义 |
|---|---|---|
| `distance_m` | m | 运动段累计航程。 |
| `flight_time_s` | s | 末顶点时间减首顶点时间，包含飞行中的悬停。 |
| `wait_time_s` | s | 调度附加的起飞延迟。 |

`wait_time_s` 与各悬停段的时长分别读取。统计空中悬停时，读取 `kind="hover"` 段的 `end_time_s - start_time_s`。

<a id="validate-request"></a>
### POST /api/v2/validate：轨迹验证请求

| 字段 | 类型 | 必填与默认值 | 填写方法 |
|---|---|---|---|
| `scene` | Scene 对象 | 与 `scene_id` 二选一 | 使用规划时的同一场景。 |
| `scene_id` | 字符串 | 与 `scene` 二选一 | 使用规划时的同一登记场景。 |
| `task` | Task 对象 | 必填 | 使用规划时的完整任务。 |
| `trajectory_wkt` | WKT 字符串 | 必填 | 复制规划响应的轨迹。 |
| `vertex_times_s` | 数值数组 | 必填，至少一个值 | 复制规划响应的顶点时间。多顶点轨迹的时间严格递增。 |
| `motion_segments` | MotionSegment 数组或 `null` | 默认 `null`；移动轨迹需要此字段 | 复制规划响应的完整运动段。 |
| `network_route` | NetworkRoute 或 null | 默认 null | 图路线复制节点/边序列；省略时从轨迹证明图走法。 |
| `initial_velocity_xyz_mps` | 三个数值 | 默认 [0,0,0] | 复制路线中的局部东/北/上起始速度，单位 m/s。 |

输入示例：[validate_request.json](data/examples/validate_request.json)。响应包括 `valid`、`violations` 和 `rule_report`。HTTP 状态为 `200`，`valid` 表示本次验证结果。

<a id="validation"></a>
### 验证结果与违规项

`RouteResult.validation` 包含 `valid` 和 `violations`。验证接口在这两个字段之外，还返回 `rule_report`。

| 字段 | 类型 | 含义 |
|---|---|---|
| `valid` | 布尔值 | `true` 表示验证通过；`false` 表示存在违规项。 |
| `violations` | Violation 数组 | 全部返回的违规项。验证通过时为 `[]`。 |
| `rule_report` | RuleReport 数组 | 验证接口顶层字段。每条输入规则的执行结果。 |

`Violation` 的字段如下。

| 字段 | 类型与默认值 | 含义 |
|---|---|---|
| `code` | 必填字符串 | 违规类别，如 `TIME`、`ENDPOINT`、`MOTION_MODEL`、`SEPARATION`。 |
| `message` | 必填字符串 | 具体原因。 |
| `rule_id` | 字符串或 `null`，默认 `null` | 产生该项违规的规则 ID。 |
| `time_s` | 数值或 `null`，默认 `null` | 违规时刻，相对场景零点的秒数。 |
| `details` | JSON 对象，默认 `{}` | 位置、障碍 ID、运动段或其他上下文。 |

<a id="rule-report"></a>
### RuleReport：规则执行报告

| 字段 | 类型与默认值 | 含义 |
|---|---|---|
| `rule_id` | 必填字符串 | `scene.rules[].id`。 |
| `rule_type` | 必填枚举 | 20 类规则之一，见[规则字段](#rules)。 |
| `status` | 必填枚举 | 规则执行状态。 |
| `message` | 字符串，默认 `""` | 执行状态说明。 |
| `details` | JSON 对象，默认 `{}` | 当前服务在此对象内返回 `reasons` 和 `dataset_versions`。 |
| `reasons` | 字符串数组，默认 `[]` | 模型中的同名辅助字段。当前有效原因列表位于 `details.reasons`。 |
| `dataset_versions` | 字符串字典，默认 `{}` | 模型中的同名辅助字段。当前有效版本表位于 `details.dataset_versions`。 |

`details.reasons` 是原因数组。`details.dataset_versions` 以数据集 ID 为键，以版本字符串为值。

未绑定有效 CLIMB/TURN 时，默认约束也产生执行报告。当前规则引擎生成 `applied`、`not_applicable` 和 `failed`。`applied` 表示已选择且有效期与路线相交；`not_applicable` 表示未选择或时间不适用；`failed` 表示规则产生违规项。响应模型另接受 `inactive` 和 `unsupported` 这两个状态值。

<a id="batch-result"></a>
### BatchResult：批量结果

| 字段 | 类型 | 含义 |
|---|---|---|
| `planned_at` | 带时区时间字符串 | 本批结果生成时刻。当前服务使用 UTC 表达。 |
| `request_id` | 字符串 | 输入业务批次编号，或服务生成的编号。 |
| `scene_id` | 字符串 | 本批使用的登记场景 ID。 |
| `scene_version` | 字符串 | 场景规范化 JSON 的 SHA-256。 |
| `routes` | RouteResult 数组 | 每个任务的结果，顺序与输入 `tasks` 一致。 |
| `success_count` | 非负整数 | `status="planned"` 的路线数。 |
| `failed_count` | 非负整数 | 未成功生成的路线数，包括超时任务。 |
| `conflicts` | Conflict 数组 | 最终连续时空冲突列表。无冲突时为 `[]`。 |
| `elapsed_ms` | 非负数 | 服务本次规划处理耗时，单位毫秒。 |
| `engine` | 字符串 | 本批使用的规划引擎标识，当前为 `native-v2`。 |
| `coordinate_context` | JSON 对象 | 本场景的坐标上下文，见[坐标上下文](#coordinate-context)。 |
| `stage_timings_ms` | 数值字典 | 处理阶段耗时，单位毫秒，见下表。 |

| `stage_timings_ms` 内字段 | 含义 |
|---|---|
| `scene_and_task_parsing` | 场景处理与任务解析。 |
| `search_motion_and_certificate` | 搜索、运动生成与候选验证。 |
| `reservation_scheduling` | 多机时间预约与调度。 |
| `final_validation_and_wkt` | 最终验证和 WKT 输出构造。 |
| `sdk_response_serialization` | 结果模型转为 JSON 兼容对象。 |

`elapsed_ms` 在 SDK 响应转换前取值。测试报告的 HTTP 耗时还包括响应转换、网络传输和客户端接收。

`Conflict` 的字段如下。

| 字段 | 类型 | 含义 |
|---|---|---|
| `task_ids` | 字符串数组 | 发生冲突的任务 ID。 |
| `start_time_s` | 非负数 | 冲突开始时刻，相对场景零点的秒数。 |
| `end_time_s` | 非负数 | 冲突结束时刻，相对场景零点的秒数。 |
| `minimum_distance_m` | 非负数或 `null` | 冲突期间返回的最小距离，单位米。 |

### POST /api/v2/jobs：提交异步任务

请求字段与[批量请求](#batch-request)相同。输入示例使用 [batch_request.json](data/examples/batch_request.json)。HTTP `202` 表示服务已接收作业。

<a id="job-submitted"></a>
#### 作业提交结果

| 字段 | 类型 | 含义 |
|---|---|---|
| `job_id` | 字符串 | 服务生成的作业 ID。 |
| `status` | 字符串 | 提交响应为 `pending`。 |
| `poll_url` | 字符串 | 查询作业的相对路径，如 `/api/v2/jobs/d877fe14f8bb42e5`。 |

取得响应后，用 GET 访问服务地址加 `poll_url`。

<a id="job-result"></a>
### GET /api/v2/jobs/{job_id}：查询异步任务

输入为提交响应中的 `job_id`。GET 请求没有 JSON 请求体。正常查询返回 HTTP `200`；未知 ID 返回 `404`。

| 字段 | 类型 | 含义 |
|---|---|---|
| `job_id` | 字符串 | 当前作业 ID。 |
| `status` | 枚举字符串 | `pending` 排队；`running` 执行中；`succeeded` 函数执行结束；`failed` 执行异常或关闭时取消。 |
| `created_at` | 字符串 | 作业建立时刻。当前使用服务进程本地时间字符串。 |
| `started_at` | 字符串或 `null` | 开始执行时刻。排队时为 `null`。 |
| `finished_at` | 字符串或 `null` | 执行结束时刻。完成前为 `null`。 |
| `elapsed_ms` | 数值或 `null` | 作业实际执行耗时，单位毫秒。排队时为 `null`。 |
| `result` | BatchResult 对象或 `null` | 作业成功执行后返回完整批量结果。 |
| `error` | JobError 对象或 `null` | 作业失败原因，字段见下表。 |

当前作业时间字符串采用 `2026-10-17T13:59:04` 这种格式，不含时区。`elapsed_ms` 从开始执行计算，不含排队时间。

作业 `status="succeeded"` 后，继续读取 `result.success_count`、`result.failed_count` 和每条路线的 `status`。作业执行完成与每条路线成功分别表达。

作业和完整结果保存在当前服务进程中。服务重启后作业记录清空。

| `JobError` 字段 | 类型 | 含义 |
|---|---|---|
| `code` | 字符串 | 稳定错误码。 |
| `message` | 字符串 | 原因说明。 |
| `detail` | JSON 对象 | 错误上下文，例如异常类型。 |

### POST /api/v2/scenes：登记场景

请求体直接使用 [Scene 对象](#scene)。此接口的请求体顶层为 `coordinates`、`bounds` 等场景字段。单任务请求中的 `scene` 包装层不放在这里。

成功返回 HTTP `200`，字段见[场景登记结果](#scene-response)。重复登记相同场景时，服务返回相同 ID 和版本。修改场景后重新登记，获得新 ID 和版本。

### GET /api/v2/scenes/{scene_id}：查询场景

在路径中填写登记接口返回的 `scene_id`。没有 JSON 请求体。成功返回 HTTP `200`，字段与[场景登记结果](#scene-response)相同。未知 ID 返回 HTTP `404` 和 `TASK_NOT_FOUND`。

此接口查询登记信息、坐标上下文和对象计数。修改场景使用重新登记流程。

### POST /api/v2/coordinates/convert：转换坐标

请求和响应字段见[坐标转换](#coordinates)。方向为 `wgs84_to_local` 或 `local_to_wgs84`。输入示例：[coordinates_to_local_request.json](data/examples/coordinates_to_local_request.json)。成功返回 HTTP `200`。

正向完整记录：[输入](reports/20261017T135804Z_public/http/023_post_api_v2_coordinates_convert_input.json)、[输出](reports/20261017T135804Z_public/http/023_post_api_v2_coordinates_convert_output.json)。反向完整记录：[输入](reports/20261017T135804Z_public/http/024_post_api_v2_coordinates_convert_input.json)、[输出](reports/20261017T135804Z_public/http/024_post_api_v2_coordinates_convert_output.json)。

### GET /api/v2/examples/single：取得单任务示例

没有请求体。HTTP `200` 响应是[单任务请求](#single-request)，包含 `scene`、`task` 和 `time_budget_s`。把整个响应提交到 `/api/v2/plan/single` 即可运行示例。

### GET /api/v2/examples/batch：取得批量示例

没有请求体。HTTP `200` 响应是[批量请求](#batch-request)，包含 `scene`、`tasks`、`request_id` 和 `time_budget_s`。把整个响应提交到 `/api/v2/plan/batch`，或提交到 `/api/v2/jobs`。

<a id="health"></a>
### GET /api/v2/health：健康状态

没有请求体。HTTP `200` 响应字段如下。

| 字段 | 类型 | 含义 |
|---|---|---|
| `status` | 字符串 | `ok` 表示本地规划引擎可用；`degraded` 表示引擎不可用。 |
| `api_version` | 字符串 | 当前接口版本，`2.0.0`。 |
| `native_available` | 布尔值 | 本地规划引擎是否已加载。 |
| `jobs` | 作业计数对象 | 各状态的作业数量，字段见[作业计数](#job-counts)。 |

<a id="job-counts"></a>
### GET /api/v2/health/jobs：作业计数

没有请求体。HTTP `200` 响应中各字段都是非负整数。

| 字段 | 含义 |
|---|---|
| `pending` | 排队作业数量。 |
| `running` | 执行中作业数量。 |
| `succeeded` | 已成功执行的作业数量。 |
| `failed` | 执行失败或已取消的作业数量。 |
| `total` | 当前服务进程保存的全部作业数量，等于前四项之和。 |

<a id="version"></a>
### GET /api/v2/version：版本与默认机型

没有请求体。HTTP `200` 响应字段如下。

| 字段 | 类型 | 含义 |
|---|---|---|
| `api_version` | 字符串 | 服务接口版本。 |
| `engine` | JSON 对象 | 当前引擎与构建版本，字段见下表。 |
| `default_profile` | AircraftProfile 对象 | 内置默认机型的全部参数，见[机型字段](#profiles)。 |

| `engine` 内字段 | 类型 | 含义 |
|---|---|---|
| `available` | 布尔值 | 本地引擎是否可用。 |
| `engine` | 字符串 | 引擎类型，当前为 `cpp17-pybind11`。 |
| `version` | 字符串或 `null` | 本地引擎版本。 |
| `git_sha` | 字符串 | 服务构建提交 SHA。部署时写入。未设置时返回 `unknown`。 |
| `error` | 字符串或 `null` | 引擎加载错误。正常加载时为 `null`。 |

<a id="capabilities"></a>
### GET /api/v2/capabilities：接口能力

本次契约 `contract_version="2026-10-30"`。`engine_rules` 返回 ENG-06/ENG-07 规则分配；`canonical_alt_reference="EG"`。对接前核对运行服务的这些字段。

没有请求体。HTTP `200` 响应字段如下。

| 字段 | 类型 | 当前含义 |
|---|---|---|
| `api_version` | 字符串 | 当前接口版本。 |
| `route_modes` | 字符串数组 | `strict`、`prefer`、`free`。 |
| `default_route_mode` | 字符串 | 默认 `strict`。 |
| `rule_types` | 字符串数组 | 支持的 20 类规则。见[规则字段](#rules)。 |
| `altitude_references` | 字符串数组 | `EG`、`MSL`、`AGL`、`ELLIPSOID`、`QNE`、`QNH`。 |
| `geometric_tolerance_m` | 数值 | 轨迹几何近似精度，单位米，当前为 `0.1`。 |
| `max_scene_radius_m` | 数值或 `null` | 当前为 `null`，服务不设置固定场景半径上限。场景覆盖半径由 `coordinates.max_radius_m` 指定。 |
| `grid_formats` | 字符串数组 | `regular_sparse_indices`、`regular_index_ranges`、`explicit_bounds_cells`。见[网格字段](#grid)。 |
| `scene_limits` | 对象 | 对象数量和存储限额信息。当前各值为 `null`，表示不设固定配额。 |
| `max_registered_scenes` | 整数或 `null` | 当前为 `null`，不设登记场景数配额。 |
| `max_registered_scene_bytes` | 整数或 `null` | 当前为 `null`，不设估算场景字节数配额。 |
| `scene_lifecycle` | 字符串 | 场景生命周期说明：进程内保存、重启后重新登记、任务答案不缓存。 |

`scene_limits` 包含 `airways`、`obstacles`、`control_zones`、`connectors`、`rules`、`profiles`、`datasets`、`geometry_vertices`、`field_cells`、`grid_entries`、`grid_nodes` 和 `estimated_scene_bytes`。这些字段当前全部返回 `null`。

<a id="errors"></a>
### 结果状态、HTTP 状态和错误字段

规划失败与请求错误分别返回。路线已进入规划流程时，HTTP `200` 响应中的 `status`、`code` 和 `message` 说明任务结果。请求字段或引用错误时，接口返回错误 HTTP 状态和错误对象。

| HTTP 状态 | 错误码 | 含义 |
|---|---|---|
| 400 | `INVALID_INPUT` | 请求字段、类型、场景选择或引用约束有误。 |
| 400 | 具体输入错误码 | 坐标转换和规则处理还可返回具体错误码，如 `HEIGHT_REFERENCE_UNAVAILABLE`。 |
| 404 | `TASK_NOT_FOUND` | 场景或作业 ID 不存在。 |
| 404 | `AIRCRAFT_NOT_FOUND` | `profile_id` 对应的机型不存在。 |
| 429 | `JOB_QUEUE_FULL` | 服务正在关闭，停止接收新作业。 |
| 500 | `INTERNAL_ERROR` | 服务执行异常。 |
| 503 | `NATIVE_UNAVAILABLE` | 本地规划引擎不可用。 |
| 503 | `RESOURCE_EXHAUSTED` | 运行时内存分配失败。 |

错误响应结构如下。

```json
{
  "error": {
    "code": "INVALID_INPUT",
    "message": "请求体校验失败",
    "detail": {
      "errors": [
        {
          "field": "body.unknown_field",
          "reason": "Extra inputs are not permitted"
        }
      ]
    }
  }
}
```

| 字段 | 类型 | 含义 |
|---|---|---|
| `error` | 对象 | 错误信封。 |
| `error.code` | 字符串 | 用于程序判断的错误码。 |
| `error.message` | 字符串 | 原因说明。 |
| `error.detail` | JSON 对象 | 字段位置、非法引用或异常类型等上下文。 |
| `error.detail.errors` | 数组 | 请求模型错误列表，模型校验失败时返回。 |
| `error.detail.errors[].field` | 字符串 | 错误字段路径。 |
| `error.detail.errors[].reason` | 字符串 | 字段错误原因。 |

完整错误示例：[400 未知字段](reports/20261017T135804Z_public/http/028_post_api_v2_plan_single_output.json)、[404 未知场景](reports/20261017T135804Z_public/http/029_get_api_v2_scenes_test-api-missing-scene_output.json)、[404 未知作业](reports/20261017T135804Z_public/http/030_get_api_v2_jobs_test-api-missing-job_output.json)。

当前任务结果使用的原因码包括：

| `RouteResult.code` | 含义 |
|---|---|
| `SEARCH_NOT_FOUND` | 当前搜索未找到满足要求的路线或安全起飞时段。 |
| `TIME` | 到达窗口或允许起飞窗口不能满足。 |
| `TIME_BUDGET_EXCEEDED` | 达到本次请求指定的计算时限。 |
| `CONSTRAINT_VIOLATION` | 最终独立验证发现约束违规。 |
| `SEPARATION` | 最终连续时空间隔验证发现冲突。 |
| `VALIDATION_INCOMPLETE` | 连续间隔验证没有完成。 |

<a id="scene"></a>
## 场景 `Scene`

场景定义坐标、规划范围、航路、空间对象和环境数据。将场景对象直接发送到 `POST /api/v2/scenes`。单机、批量和验证请求也可在 `scene` 字段中包含完整场景。

示例文件：[单机完整请求](data/examples/single_strict_request.json)、[混合场景](data/benchmarks/mixed_scene.json)。单机文件中的 `scene` 是场景对象。混合场景文件本身就是场景对象。

| 字段 | 类型、单位 | 默认值 | 填写方法与作用 |
|---|---|---|---|
| `scene_type` | 枚举 | `LOW_ALTITUDE` | 当前只接受低空场景；地面和水上暂无。 |
| `network_version` | 字符串或 null | null | 上游路网版本，用于追溯；提供 airway_graph 时须与图版本一致。 |
| `airway_graph` | AirwayGraph 或 null | null | 节点、方向边、边权和关联航路；结构见本文件航路图说明。 |
| `coordinates` | 对象 | 必填 | 配置 WGS84 与局部米制坐标。见[坐标配置](#coordinates)。 |
| `bounds` | 对象，m | 必填 | 设置局部规划范围。六个字段为 `x_min`、`x_max`、`y_min`、`y_max`、`z_min`、`z_max`。x 向东，y 向北，z 为 EG 椭球高减 `origin_alt_ellipsoid`。每轴最大值大于最小值。 |
| `resolution` | 三个正数，m | `[25,25,10]` | 按 `[dx,dy,dz]` 设置内部搜索网格分辨率。修改此值会改变搜索网格和结果中的内部网格索引。 |
| `horizon_s` | 正数，s | `3600` | 设置从 `time_origin` 开始的任务规划时域。按任务时间调整。 |
| `time_origin` | 带时区时间 | 必填 | 设置场景时间原点。例如 `2026-10-16T09:00:00+08:00`。轨迹的秒数相对此时刻计量。 |
| `airways` | 对象数组 | `[]` | 配置航路。见[航路](#airways)。 |
| `connectors` | 对象数组 | `[]` | 配置起降接驳区域。见[接驳区域](#connectors)。 |
| `obstacles` | 对象数组 | `[]` | 配置静态障碍和建筑。见[障碍物](#obstacles)。 |
| `dynamic_obstacles` | 对象数组 | `[]` | 配置已知动态轨迹。见[动态轨迹](#dynamic-obstacles)。 |
| `control_zones` | 对象数组 | `[]` | 配置管制区。见[管制区](#control-zones)。 |
| `rules` | 对象数组 | `[]` | 配置执行规则。见[20 类规则](#rules)。 |
| `datasets` | 对象数组 | `[]` | 配置地形、天气、信号等空间字段。见[数据集](#datasets)。 |
| `profiles` | 对象数组 | `[]` | 登记任务引用的机型参数。见[机型](#profiles)。 |
| `grid` | 对象或 `null` | `null` | 导入外部占用网格和编码。见[网格](#grid)。 |

为空间资源设置稳定的 `id`。航路、接驳区、障碍物、管制区和数据集共用资源引用表。任务通过机型 ID、接驳区 ID 和规则 ID 引用配置。规则通过 `resource_ids` 引用空间资源。时间字段填写带时区的 ISO 8601 字符串。仅规则外层 `effective_start/effective_end` 兼容无时区的北京时间和空字符串。可选有效期的开始或结束为 `null` 时，该方向保持开放。

### 修改场景并用于任务

1. 复制 [strict 场景](data/benchmarks/strict_scene.json)，或提取 [单机请求](data/examples/single_strict_request.json)中的 `scene`。
2. 修改所需坐标、航路、区域或环境数据。
3. 将完整场景发送到 `POST /api/v2/scenes`。
4. 从响应中读取 `scene_id` 和 `scene_version`。
5. 在规划请求中填写返回的 `scene_id`。请求同时填写 `task` 或 `tasks`。
6. 在 `GET /api/v2/scenes/{scene_id}` 中查询登记结果。

例如，登记场景返回 `scene_id` 后，用以下 Python 代码组成批量请求：

```python
import json
from pathlib import Path
from urllib.request import Request, urlopen

scene = json.loads(Path("data/benchmarks/strict_scene.json").read_text(encoding="utf-8"))
tasks = json.loads(Path("data/benchmarks/strict_tasks.json").read_text(encoding="utf-8"))
registration_request = Request(
    "https://api.kaguratart.com/api/v2/scenes",
    data=json.dumps(scene).encode("utf-8"),
    headers={
        "Content-Type": "application/json",
        "Accept": "application/json",
        "User-Agent": "UAV-API-Test/1.0",
    },
    method="POST",
)
with urlopen(registration_request) as response:
    registration = json.load(response)
request = {"scene_id": registration["scene_id"], "tasks": tasks, "request_id": "strict-check-01"}
print(request["scene_id"])
```

改变场景内容后，重新登记场景，并将新的 `scene_id` 写入请求。相同的规范化场景内容对应相同的版本摘要。

修改坐标原点时，同步处理局部米制数据：重新换算 `bounds`、字段单元的 `bounds`、外部 `grid` 的原点和范围。WGS84 几何保留其实际地理位置。若要移动整个业务场景，同步修改航路中心线、端点网格、接驳区、障碍物、管制区、字段范围和任务点。

修改 `origin_alt_ellipsoid` 时，重新计算全部局部 z 值。修改高度基准时，同步修改相应高度值及其转换数据。修改 `time_origin` 时，同步调整任务时间、对象有效期、字段有效期和动态轨迹秒数。

<a id="coordinates"></a>
## 坐标和高度

WGS84 几何按 `经度 纬度 高度` 排列。经纬度使用度，高度使用米。局部几何按 `东向米 北向米 局部高度米` 排列。

水平转换使用以场景原点建立的 AEQD 投影。局部高度按以下公式计算：

```text
local_z_m = altitude_ellipsoid_m - origin_alt_ellipsoid
```

### `coordinates` 配置字段

场景和坐标转换请求使用同一结构。

| 字段 | 类型、单位 | 默认值 | 填写方法与作用 |
|---|---|---|---|
| `origin_lon` | 数值，度 | 必填 | 填写局部原点的 WGS84 经度。 |
| `origin_lat` | 数值，度 | 必填 | 填写局部原点的 WGS84 纬度。 |
| `origin_alt_ellipsoid` | 数值，m EG | `0` | 设置局部 z=0 对应的 EG 椭球高。 |
| `max_radius_m` | 正数，m | `50000` | 声明本次局部坐标覆盖半径。按场景实际范围填写。 |
| `terrain_ellipsoid_m` | 数值或 `null`，m EG | `null` | 填写场景平坦地形椭球高，用于 AGL 转换和地面检查。场景也可通过 `terrain` 数据集提供分区地形。 |
| `geoid_separation_m` | 数值或 `null`，m | `null` | 填写大地水准面差 `N=椭球高−MSL高`，用于 MSL 与 EG 转换；使用 MSL 时还必须填写非空 geoid_source。 |
| `terrain_source` | 字符串或 `null` | `null` | 记录常量地形的来源。响应的坐标上下文会返回此值。 |
| `geoid_source` | 字符串或 `null` | `null` | 记录大地水准面差的来源。响应的坐标上下文会返回此值。 |
| `height_corrections` | 对象数组 | `[]` | 填写 QNE、QNH 高度改正。见[高度改正](#altitude-corrections)。 |

### `alt_reference` 与换算

任务、障碍物、动态轨迹和转换请求使用 `alt_reference`。航路和管制区使用 `altReference`。

| 值 | 高度含义 | 换算为 EG | 配套数据 |
|---|---|---|---|
| `EG` / `ELLIPSOID` | WGS84 椭球高 | `EG = 输入高度` | 直接使用；ELLIPSOID 是兼容名。 |
| `MSL` | 相对平均海平面的高度 | `EG = 输入高度 + geoid_separation_m` | 必须填写差值和非空 `geoid_source`。 |
| `AGL` | 相对当地地面的高度 | `EG = 输入高度 + 地形椭球高` | 常量 `terrain_ellipsoid_m` 或分区 terrain 数据。 |
| `QNE` / `QNH` | 气压高度 | `EG = 输入高度 + offset_m` | 对应位置和时刻唯一有效的改正；target_reference=EG。 |

AGL 航路和区域沿运动段使用对应地形。分区地形通过 `datasets[].kind="terrain"` 提供。当地形单元重叠时，服务使用较高的地形椭球高进行地面检查。

<a id="altitude-corrections"></a>
### `height_corrections[]` 高度改正

| 字段 | 类型、单位 | 默认值 | 填写方法与作用 |
|---|---|---|---|
| `target_reference` | `EG` | `EG` | offset_m 的目标基准固定为 EG。 |
| `reference` | `QNE` 或 `QNH` | 必填 | 指定改正的高度基准。 |
| `offset_m` | 数值，m | 必填 | 填写 `EG−输入高度`。正值表示 EG 高于该基准的输入值。 |
| `source` | 非空字符串 | 必填 | 记录改正数据来源。 |
| `version` | 非空字符串 | 必填 | 记录改正数据版本。 |
| `valid_from` | 带时区时间 | 必填 | 设置生效开始时刻。 |
| `valid_to` | 带时区时间 | 必填 | 设置生效结束时刻，并晚于 `valid_from`。点坐标转换采用 `[valid_from,valid_to)`。 |
| `geometry_wkt` | WGS84 WKT 或 `null` | `null` | 用面几何指定适用区域。`null` 表示覆盖当前局部坐标区域。 |

为每个转换位置和时刻提供一条适用改正。QNE、QNH 坐标转换同时填写 `at`。

<a id="coordinate-context"></a>
### 响应中的 `coordinate_context`

场景登记、场景查询、规划结果和坐标转换响应返回坐标上下文。

| 字段 | 单位或取值 | 含义 |
|---|---|---|
| `input_horizontal_crs` | `EPSG:4326` | 外部水平坐标系。 |
| `axis_order` | `longitude,latitude,altitude` | 外部 WKT 轴顺序。 |
| `local_horizontal_crs` | 投影定义字符串 | 完整 AEQD 投影，包含原点和米制单位。 |
| `local_axis_order` | `east_m,north_m,ellipsoid_minus_origin_m` | 局部坐标轴顺序。 |
| `origin_lon` | 度 | 实际使用的原点经度。 |
| `origin_lat` | 度 | 实际使用的原点纬度。 |
| `origin_alt_ellipsoid` | m EG | 实际使用的局部高度原点。 |
| `canonical_alt_reference` | `EG` | 内部统一垂直基准。 |
| `max_radius_m` | m | 请求声明的局部坐标半径。 |
| `terrain_source` | 字符串或 `null` | 请求声明的常量地形来源。 |
| `geoid_source` | 字符串或 `null` | 请求声明的椭球高改正来源。 |
| `height_correction_versions` | 字符串数组 | 请求中的高度改正版本，保持配置顺序。 |

### `POST /api/v2/coordinates/convert`

完整输入见 [coordinates_to_local_request.json](data/examples/coordinates_to_local_request.json)。

| 输入字段 | 类型、单位 | 默认值 | 填写方法 |
|---|---|---|---|
| `geometries_wkt` | 非空 WKT 字符串数组 | 必填 | 一次填写多个点、线或面。各项保持原有顶点及二维或三维形式。 |
| `direction` | 字符串 | 必填 | `wgs84_to_local` 转为米制；`local_to_wgs84` 转为经纬度。 |
| `alt_reference` | 高度基准字符串 | 必填 | 指定正向输入 Z 或反向输出 Z 的高度基准。 |
| `coordinates` | 坐标配置对象 | 必填 | 填写[坐标配置](#coordinates)。正反向使用相同配置。 |
| `at` | 带时区时间或 `null` | `null` | 设置高度改正使用的时刻。QNE、QNH 使用该值选择改正。 |

| 输出字段 | 类型 | 含义 |
|---|---|---|
| `geometries_wkt` | WKT 字符串数组 | 与输入顺序一一对应的转换结果。 |
| `direction` | 字符串 | 返回本次转换方向。 |
| `alt_reference` | 字符串 | 返回本次外部高度基准。 |
| `coordinate_context` | 对象 | 返回实际使用的坐标配置和投影。见[坐标上下文](#coordinate-context)。 |

执行反向转换时，将正向响应中的 `geometries_wkt` 放入反向请求，保留 `coordinates`、`alt_reference` 和 `at`，将 `direction` 改为 `local_to_wgs84`。局部输入的 Z 始终为 `EG−origin_alt_ellipsoid`。

本包正向示例将 `POINT Z (104.771029580333 29.330902187336 80)` 转为约 `POINT Z (100 100 80)`。该示例的 `origin_alt_ellipsoid` 为 `0`，外部高度基准为 `EG`。

<a id="airways"></a>
## 航路 `airways[]`

中心线用 `LINESTRING Z` 表示。Z 给出参考高度剖面。`minAltitude` 和 `maxAltitude` 给出实际可用高度带。

| 字段 | 类型、单位 | 默认值 | 填写方法与作用 |
|---|---|---|---|
| `id` | 非空字符串 | 必填 | 设置航路 ID，供接驳区和规则引用。 |
| `allowed_profile_ids` | string[] | `[]` | 机型准入白名单；空数组不限制。strict/prefer 中，不准入航路体为禁入区，重叠获准航路不能抵消；free 不使用航路准入。 |
| `centerLine` | WGS84 `LINESTRING Z` | 必填 | 按飞行方向填写中心线顶点。首点落在 `startGrid` 内，末点落在 `endGrid` 内。全部参考高度落在航路高度带内。 |
| `widthMeters` | 正数，m | 必填 | 设置水平走廊**全宽**。中心线每侧宽度为该值的一半。规则 `WIDTH.content.width` 也使用全宽，见[规则](#rules)。 |
| `minAltitude` | 数值，m | 必填 | 设置航路最低高度，采用 `altReference`。 |
| `maxAltitude` | 数值，m | 必填 | 设置航路最高高度，并大于最低高度。 |
| `altReference` | 高度基准 | 必填 | 指定中心线 Z、端点网格高度和航路高度带的基准。 |
| `startGrid` | 地理范围对象 | 必填 | 描述中心线首点所在的三维网格。字段见下表。 |
| `endGrid` | 地理范围对象 | 必填 | 描述中心线末点所在的三维网格。字段见下表。 |
| `status` | `ACTIVE` 或 `INACTIVE` | `ACTIVE` | 选择航路是否启用。 |
| `direction` | `BOTH`、`FORWARD`、`REVERSE` | `BOTH` | `FORWARD` 沿中心线顶点顺序通行；`REVERSE` 反向通行；`BOTH` 双向通行。 |
| `valid_from` | 带时区时间或 `null` | `null` | 设置航路生效开始时刻。 |
| `valid_to` | 带时区时间或 `null` | `null` | 设置航路生效结束时刻。 |
| `climbConstraint` | 对象或 `null` | `null` | 用 `maxClimbRate`、`maxDescentRate` 设置航路最大爬升、下降率，单位 m/s，均为正数。 |
| `turnConstraint` | 对象或 `null` | `null` | 用 `maxTurnAngle` 设置最大转向角，取值大于 0 且不大于 180，单位度；用 `minTurnRadius` 设置非负的最小转弯半径，单位 m。 |

`startGrid` 和 `endGrid` 使用以下六个字段：

| 字段 | 单位 | 含义 |
|---|---|---|
| `minLon`、`maxLon` | 度 | 经度范围。 |
| `minLat`、`maxLat` | 度 | 纬度范围。 |
| `minAlt`、`maxAlt` | m | 高度范围，使用航路的 `altReference`。 |

输入也接受 `minLongitude`/`maxLongitude`、`minLatitude`/`maxLatitude`、`minAltitude`/`maxAltitude`，以及同义的 `x_min`/`x_max`、`y_min`/`y_max`、`z_min`/`z_max`。交付数据使用上表字段。

移动航路时，同步更新 `centerLine`、`startGrid`、`endGrid` 和接驳区域。调整高度带时，同步检查中心线 Z、端点网格和任务高度。服务按飞机包络及安全净空收缩航路可用空间。

<a id="obstacles"></a>
## 障碍物 `obstacles[]`

障碍物由水平面几何、高度带和有效期组成。建筑也放在此数组中。服务在规划和轨迹验证中检查完整运动段。

| 字段 | 类型、单位 | 默认值 | 填写方法与作用 |
|---|---|---|---|
| `id` | 非空字符串 | 必填 | 设置障碍物 ID。`BUILDING` 规则可引用该值。 |
| `geometry_wkt` | WGS84 `POLYGON` 或 `MULTIPOLYGON` | 必填 | 填写水平轮廓。高度使用下面的独立字段。 |
| `min_altitude` | 数值，m | 必填 | 填写最低高度。 |
| `max_altitude` | 数值，m | 必填 | 填写最高高度。 |
| `alt_reference` | 高度基准 | 必填 | 指定高度带的基准。 |
| `buffer_m` | 非负数，m | `0` | 在碰撞检查中增加水平缓冲距离。 |
| `valid_from` | 带时区时间或 `null` | `null` | 设置生效开始时刻；省略时从场景开始前持续生效。 |
| `valid_to` | 带时区时间或 `null` | `null` | 设置生效结束时刻；省略时持续生效。 |

调整障碍物位置时修改 `geometry_wkt`。调整建筑高度时修改高度带。调整安全距离时修改 `buffer_m` 或相应 `BUILDING` 规则，并同步查看[机型净空](#profiles)。

<a id="connectors"></a>
## 接驳区域 `connectors[]`

接驳区域使用[障碍物表](#obstacles)中的全部字段，并增加以下字段。其几何表示起降接驳可用空间。

| 字段 | 类型 | 默认值 | 填写方法与作用 |
|---|---|---|---|
| `airway_ids` | 航路 ID 字符串数组 | `[]` | 指定该接驳区连接的航路。填写 `airways[].id`。 |

在任务中用 `start_connector_id` 指定起点接驳区，用 `end_connector_id` 指定终点接驳区。将相应起终点放入接驳区，并使接驳区与关联航路连通。服务只在航路进入段和离开段使用接驳区。

<a id="control-zones"></a>
## 管制区 `control_zones[]`

管制区由 `geom`、高度带、状态和有效期定义。服务使用其原始几何检查运动段。

| 字段 | 类型、单位 | 默认值 | 填写方法与作用 |
|---|---|---|---|
| `id` | 非空字符串 | 必填 | 设置管制区 ID。规则用此值引用区域。 |
| `geom` | WGS84 `POLYGON` 或 `MULTIPOLYGON` | 必填 | 填写实际使用的水平占用轮廓。 |
| `boundary2d` | WGS84 WKT 或 `null` | `null` | 保存业务侧二维边界资料。碰撞检查使用 `geom`。 |
| `centerPoint` | WGS84 WKT 或 `null` | `null` | 保存业务侧中心点资料。 |
| `minAlt` | 数值，m | 必填 | 设置管制区最低高度。 |
| `maxAlt` | 数值，m | 必填 | 设置管制区最高高度。 |
| `altReference` | 高度基准 | 必填 | 指定高度带的基准。 |
| `zoneType` | 字符串 | 必填 | 保存业务区域类型。对 `MUSHROOM`，用 `geom` 表示最大截面，并在整个高度带使用该截面。 |
| `shapeParams` | JSON 对象 | `{}` | 保存业务形状参数。规划几何由 `geom` 提供。 |
| `areaM2` | 非负数或 `null`，m² | `null` | 保存面积资料；也接受输入名 `area`。服务按局部米制 `geom` 计算实际占用面积。 |
| `volumeM3` | 非负数或 `null`，m³ | `null` | 保存体积资料；也接受输入名 `volume`。实际占用体积由面积和高度带确定。 |
| `level` | 字符串、整数或 `null` | `null` | 填写管制级别。`ZONE.content.zoneLevels` 按该值选择对象。 |
| `status` | `ACTIVE` 或 `INACTIVE` | `ACTIVE` | 选择是否启用该管制区。 |
| `effective_start` | 带时区时间或 `null` | `null` | 设置生效开始时刻。 |
| `effective_end` | 带时区时间或 `null` | `null` | 设置生效结束时刻。 |

更新业务形状参数时，同时生成并更新 `geom`。修改 `geom` 后，将 `areaM2`、`volumeM3` 设为 `null`，或填写重新计算的面积、体积。变更区域 ID 后，同步修改规则的 `resource_ids` 和 `AREA.content.zoneIds`。

<a id="dynamic-obstacles"></a>
## 动态轨迹 `dynamic_obstacles[]`

| 字段 | 类型、单位 | 默认值 | 填写方法与作用 |
|---|---|---|---|
| `id` | 非空字符串 | 必填 | 设置动态目标 ID。 |
| `trajectory_wkt` | WGS84 `POINT Z` 或 `LINESTRING Z` | 必填 | 按时间顺序填写动态目标位置。 |
| `vertex_times_s` | 非负数数组，s | 必填 | 为每个 WKT 顶点填写相对 `time_origin` 的秒数，按严格递增顺序排列。点轨迹填写一个时间值。 |
| `alt_reference` | 高度基准 | 必填 | 指定轨迹 Z 的高度基准。 |
| `horizontal_separation_m` | 非负数，m | `5` | 设置与该目标的水平间隔。 |
| `vertical_separation_m` | 非负数，m | `5` | 设置与该目标的垂直间隔。 |

修改轨迹位置时同步修改对应时间。修改场景时间原点时重新计算 `vertex_times_s`。

<a id="datasets"></a>
## 地形和环境数据集 `datasets[]`

每个数据集使用稳定 ID、类型和版本。规则通过 `resource_ids` 选择对应数据集。混合场景 [mixed_scene.json](data/benchmarks/mixed_scene.json) 包含天气、信号、噪声和允许区域数据。

| 字段 | 类型 | 默认值 | 填写方法与作用 |
|---|---|---|---|
| `id` | 非空字符串 | 必填 | 设置数据集 ID，供规则引用。 |
| `kind` | 类型字符串 | 必填 | 使用 `terrain`、`weather`、`signal`、`noise`、`area`、`grid`、`risk`、`cost` 或 `corridor`。值字段见下表。 |
| `version` | 非空字符串 | 必填 | 记录数据版本，执行报告返回引用的数据版本。 |
| `valid_from` | 带时区时间或 `null` | `null` | 设置数据集生效开始时刻。 |
| `valid_to` | 带时区时间或 `null` | `null` | 设置数据集生效结束时刻。 |
| `cells` | 字段单元数组 | `[]` | 给出分区范围、数值和单元有效期。 |
| `geometry_wkt` | WGS84 WKT 或 `null` | `null` | 指定数据集整体轮廓。省略时，服务合并单元轮廓。区域和批准通道可直接使用整体面几何。 |

### 字段单元 `cells[]`

| 字段 | 类型、单位 | 默认值 | 填写方法与作用 |
|---|---|---|---|
| `geometry_wkt` | WGS84 WKT 或 `null` | `null` | 用经纬度定义单元水平面；与 `bounds` 二选一。 |
| `bounds` | 局部米制范围或 `null` | `null` | 使用 `x_min`、`x_max`、`y_min`、`y_max`、`z_min`、`z_max`；z 为局部高度。与 `geometry_wkt` 二选一。 |
| `min_altitude` | 数值或 `null`，m EG | `null` | 使用 `geometry_wkt` 时，设置单元最低椭球高。省略时覆盖全部较低高度。 |
| `max_altitude` | 数值或 `null`，m EG | `null` | 使用 `geometry_wkt` 时，设置单元最高椭球高。省略时覆盖全部较高高度。 |
| `values` | JSON 对象 | 必填 | 按数据集 `kind` 填写下表字段。 |
| `valid_from` | 带时区时间或 `null` | `null` | 设置单元生效开始时刻。 |
| `valid_to` | 带时区时间或 `null` | `null` | 设置单元生效结束时刻。 |

单元使用数据集有效期与自身有效期的交集。`bounds` 已包含完整高度带；使用该形式时直接修改 `bounds.z_min` 和 `bounds.z_max`。

### `values` 数值字段

| `kind` | `values` 字段与单位 | 使用方法 |
|---|---|---|
| `terrain` | `elevation_ellipsoid_m`，m EG；也接受 `terrain_ellipsoid_m` | 为单元填写地形椭球高。AGL 高度换算和地面检查读取该值。 |
| `weather` | `wind_speed_mps`，m/s；也接受 `windSpeed` | 天气规则将此值与 `maxWindSpeed` 比较。 |
| `weather` | `precipitation`，布尔值或非负数；`precipitation_mm_h`，mm/h | `false` 或 `0` 表示无降水。若填写数值强度，优先使用 `precipitation_mm_h` 明确单位。两者都填写时读取 `precipitation`。 |
| `weather` | `icing`，布尔值或非负数 | `false` 或 `0` 表示无结冰。 |
| `weather` | `visibility_m`，非负米值 | 与 WEATHER.minVisibility 比较。 |
| `signal` | `bands`，频段到 dBm 的对象 | 例如 `{"2.4GHz":-60,"5.8GHz":-65}`。信号规则结合机型频段执行 `any` 或 `all` 检查。 |
| `signal` | `signal_strength_dbm`，dBm | 用一个数值表示各请求频段的统一强度。也可填写频段到 dBm 的对象。填写 `bands` 时优先读取 `bands`。 |
| `noise` | `noise_db`，dB | 安静偏好读取此值。数值越低，噪声越小。 |
| `risk` | `risk`，数值 | 低风险偏好和多目标风险积分读取此值。使用非负统一标度，数值越大风险越高。 |
| `cost` | `cost`，非负数值 | 由 objectives.cost_layers 引用，按距离或时间积分；积分后单位由 unit 声明。 |
| `area` | `adminCode`、`gridCode`、`code` | 记录单元业务编码。`AREA` 规则按行政区编码或网格编码选择轮廓。 |
| `grid` | `adminCode`、`gridCode`、`code`、`occupied` | 保存字段网格的编码和占用属性。外部占用网格输入使用场景 `grid`，见[网格](#grid)。 |
| `corridor` | 使用整体 `geometry_wkt` 或单元轮廓 | 给 `BUILDING` 的 `CORRIDOR` 模式提供批准通道。 |
| 全部类型 | `nodata`，布尔值，默认不设置 | `true` 标记缺测单元。对应字段规则将该单元作为缺测区域处理。 |

更新字段值后，同步更新 `version` 和有效期，再重新登记场景。修改 `id` 时，同步修改规则的 `resource_ids`。

例如，用以下完整数据集提供一块平坦地形：

```json
{
  "id": "terrain-local-01",
  "kind": "terrain",
  "version": "2026-10-18",
  "cells": [
    {
      "bounds": {
        "x_min": 0,
        "x_max": 1500,
        "y_min": 0,
        "y_max": 500,
        "z_min": 0,
        "z_max": 300
      },
      "values": {"elevation_ellipsoid_m": 0}
    }
  ]
}
```

<a id="scene-response"></a>
## 场景登记和查询响应

响应还返回 `scene_type="LOW_ALTITUDE"` 和上游 `network_version`（未填写则 null）。`scene_version` 是完整规范化场景的内容哈希，与上游路网版本分别保存。

`POST /api/v2/scenes` 和 `GET /api/v2/scenes/{scene_id}` 返回相同字段结构。

| 字段 | 类型、单位 | 含义 |
|---|---|---|
| `scene_id` | 字符串 | 当前场景标识。将它写入规划、验证和异步请求。 |
| `scene_version` | 字符串 | 规范化场景内容的完整 SHA-256 摘要。 |
| `status` | `ready` | 场景已编译，可以提交任务。 |
| `compilation_ms` | 数值，ms | 本场景初次编译耗时。重复登记同一场景会返回已保存的编译信息。 |
| `coordinate_context` | 对象 | 实际使用的[坐标上下文](#coordinate-context)。 |
| `warnings` | 数组 | 几何处理提示。蘑菇形区域等处理方式写入此处。 |
| `counts` | 对象 | 返回 `airways`、`obstacles`、`control_zones`、`rules`、`datasets` 的输入数量。 |

<a id="profiles"></a>
## 飞机配置 `AircraftProfile`

将配置放入 `Scene.profiles`，再用 `Task.profile_id` 引用。也可直接填写 `Task.aircraft`。直接配置优先于 `profile_id`。每项省略字段使用下表缺省值。

| 字段 | 类型 / 单位 | 缺省值 | 含义与修改方法 |
|---|---|---|---|
| `id` | string | `dji_mini_4_pro` | 机型配置 ID。自定义机型使用新 ID。 |
| `name` | string | `DJI Mini 4 Pro (engineering planning profile)` | 显示名称。 |
| `max_speed_mps` | number > 0 / m/s | 12 | 最大三维移动速度。规则和航路可进一步限制速度。 |
| `cruise_speed_mps` | number > 0 / m/s | 6 | 计划巡航速度。其值不大于 `max_speed_mps`。 |
| `climb_speed_mps` | number > 0 / m/s | 3 | 最大爬升速度。 |
| `descent_speed_mps` | number > 0 / m/s | 3 | 最大下降速度。填写速度大小。 |
| `max_yaw_rate_deg_s` | number > 0 / 度/s | 60 | 偏航速度上限。停车转向时，用此值计算转向时间。 |
| `acceleration_mps2` | number > 0 / m/s² | 2 | 加速上限。降低此值会增加加速所需时间与距离。 |
| `deceleration_mps2` | number > 0 / m/s² | 2 | 减速上限。填写正值。 |
| `min_turn_radius_m` | number >= 0 / m | 0 | 最小转弯半径。正值要求圆弧转弯。0 配合可悬停机型支持停稳转向。 |
| `radius_m` | number > 0 / m | 0.3 | 飞机水平碰撞包络半径。 |
| `height_m` | number > 0 / m | 0.2 | 飞机垂直碰撞包络的完整高度。 |
| `safety_margin_m` | number >= 0 / m | 5 | 额外安全净空。水平净空为 `radius_m+safety_margin_m`，垂直半高为 `height_m/2+safety_margin_m`。 |
| `endurance_s` | number > 0 / s | 2040 | 机型标称可飞时间。规划持续时间包含等待与悬停。 |
| `max_range_m` | number > 0 / m | 18000 | 机型最大航程。 |
| `payload_kg` | number >= 0 / kg | 0 | 机型的外加载荷能力。任务的同名字段表示实际载荷。 |
| `rtl_altitude_m` | number >= 0 / m | 20 | 配置中的返航高度值。当前返航路径使用 `RTL.content.rtlAltitude`。 |
| `can_hover` | boolean | true | 是否支持静止飞行、等待和悬停。 |
| `supported_bands` | string[] | `["2.4GHz","5.8GHz"]` | 信号规则使用的飞机支持频段。 |
| `low_battery_percent` | number，0–100 / % | 20 | 配置中的低电阈值。当前低电返航使用 `RTL.content.lowBatteryPercent`。 |
| `engineering_values` | string[] | 见下文 | 标记采用工程缺省值的字段名。此列表记录参数来源类别，不参与路径计算。 |

`engineering_values` 缺省为以下字段名：`max_speed_mps`、`cruise_speed_mps`、`climb_speed_mps`、`descent_speed_mps`、`max_yaw_rate_deg_s`、`acceleration_mps2`、`deceleration_mps2`、`min_turn_radius_m`、`radius_m`、`height_m`、`safety_margin_m`、`rtl_altitude_m`、`low_battery_percent`。

修改机型性能时填写实测或已确认的参数。内置配置 ID 为 `dji_mini_4_pro`。`GET /api/v2/version` 返回该配置的完整当前值。

混合数据集在 `Scene.profiles` 中定义 `synthetic-0` 至 `synthetic-3` 四种机型。它们使用 5–8 m/s 巡航速度、0.25–2 kg 载荷能力，以及 1800 s 续航。任务通过 `profile_id` 选择其中一种。

<a id="grid"></a>
## 外部占用网格 `GridSpec`

在 `Scene.grid` 中填写网格。水平与高度单位均为米。坐标使用场景的局部 AEQD，z 相对 `origin_alt_ellipsoid`。数组顺序固定为 `[x,y,z]`，索引固定为 `[i,j,k]`。

`origin`、`resolution`、`shape` 为必填元数据。三种占用输入可同时使用：稀疏 `occupied`、连续索引块 `occupied_ranges`、显式边界 `cells`。服务合并三者的占用几何。

| 字段 | 类型 / 单位 | 必填 / 默认 | 含义与修改方法 |
|---|---|---|---|
| `crs` | string | `LOCAL_AEQD` | 使用场景局部 AEQD。当前取值为 `LOCAL_AEQD`。 |
| `origin` | `[number,number,number]` / m | 必填 | 第 `[0,0,0]` 格的最小角坐标。 |
| `resolution` | `[number,number,number]` / m | 必填，每项 > 0 | 每格沿 x、y、z 的边长 `[dx,dy,dz]`。 |
| `shape` | `[int,int,int]` | 必填，每项 > 0 | 三轴格数 `[nx,ny,nz]`。索引从 0 开始。 |
| `occupied` | `[GridIndex,...]` | `[]` | 每项为一个占用格索引 `[i,j,k]`。每轴索引小于对应格数。 |
| `occupied_ranges` | `GridRange[]` | `[]` | 每项将一个连续三维索引块设为占用。首尾索引都包含。 |
| `cells` | `GridCell[]` | `[]` | 单独填写每格的局部三维边界。此边界独立于规则网格的分辨率。 |
| `codes` | object，string 到 string | `{}` | 从规则网格索引键映射到上游编码。写为 `{"i,j,k":"业务编码"}`。例如 `{"1,2,3":"grid-A"}`。 |

`GridIndex` 包含三个整数，不另带类型或编码。网格覆盖的最大角为 `origin + shape*resolution`。第 `[i,j,k]` 格的最小角为 `origin + [i,j,k]*resolution`，最大角再加 `resolution`。

### 连续索引块 `GridRange`

| 字段 | 类型 | 必填 / 默认 | 含义与修改方法 |
|---|---|---|---|
| `start` | `[int,int,int]` | 必填 | 最小索引，每项 >= 0。 |
| `end` | `[int,int,int]` | 必填 | 最大索引，每项不小于 start 对应项，且小于 shape 对应项。 |

例如 `start=[1,2,3]`、`end=[2,4,3]` 代表 `2×3×1` 个占用格。改变 end 可调整整块占用空间。

### 显式单元 `GridCell`

| 字段 | 类型 | 必填 / 默认 | 含义与修改方法 |
|---|---|---|---|
| `bounds` | `Bounds3D` | 必填 | 格子的局部米制三维边界。 |
| `code` | string 或 null | null | 上游业务编码。关键点落入该格时可返回此编码。AREA 规则可按此编码引用区域。 |
| `occupied` | boolean | true | true 时增加占用障碍。false 时，该条单元只提供位置或编码。 |

`Bounds3D` 包含 `x_min/x_max/y_min/y_max/z_min/z_max` 六个必填数值。每轴最大值大于最小值。它使用局部米制坐标，区别于航路端点中的经纬度网格范围。

### 关键点与上游编码

服务先按外部 `grid.origin/resolution` 查找 `codes`。索引键使用逗号格式。未找到时，服务查找包含关键点的 `cells`，取第一个带 `code` 的单元。

结果 `keypoints[].grid_code` 保存上游编码。`keypoints[].grid_indices` 保存内部搜索网格索引。内部索引使用 `Scene.bounds` 最小角与 `Scene.resolution`，与外部格网索引分开计算。

### 网格修改示例

以下为 `Scene.grid` 的完整示例。将其并入场景后，按场景重新登记流程提交。

```json
{
  "crs": "LOCAL_AEQD",
  "origin": [0, 0, 0],
  "resolution": [25, 25, 10],
  "shape": [20, 20, 20],
  "occupied": [[1, 2, 3]],
  "occupied_ranges": [{"start": [4, 4, 2], "end": [5, 6, 3]}],
  "cells": [
    {
      "bounds": {"x_min": 200, "x_max": 230, "y_min": 100, "y_max": 140, "z_min": 20, "z_max": 40},
      "code": "building-cell-1",
      "occupied": true
    }
  ],
  "codes": {"1,2,3": "upstream-grid-001"}
}
```

现有混合基准场景的 `grid=null`。它用 WKT 障碍、管制区和数据格表达空间约束。上面的网格字段可加入自定义场景。

<a id="rules"></a>
## 规则字段 `Rule`

在 `Scene.rules` 中添加规则。用 `Task.rule_ids` 选择任务执行的规则。`rule_ids` 为 null 或省略时，任务选择全部场景规则。`rule_ids=[]` 选择空规则集。航路、障碍和机型检查仍由各自的场景字段执行。

规则参数放在 `content` 对象中；也接受 Word 样例的 `type` / `constraints`，分别对应 `rule_type` / `content`。必须补充稳定的 `id`。响应使用规范字段名。

| 字段 | 类型 | 必填 / 默认 | 含义与修改方法 |
|---|---|---|---|
| `id` | 非空 string | 必填 | 本条规则的唯一 ID。修改后同步修改任务的 `rule_ids`。 |
| `rule_type` | string 枚举 | 必填 | 选择下列 20 种规则之一。 |
| `content` | object | 必填，非空 | 填写本规则的参数。每种类型的参数见下表。 |
| `resource_ids` | string[] | `[]` | 指定规则引用的资源 ID。天气、信号和偏好规则引用对应数据集。建筑规则引用障碍体。禁飞和管制规则引用管制区。高度、速度等规则可引用几何资源，限定作用位置。空数组按规则类型选择场景资源，或执行全局约束。 |
| `scope` | 对象或 null | null | 本条规则的空间作用域，见下表。 |
| `effective_start` | timestamp 或 null | null | 规则开始生效的绝对时刻；无时区按 Asia/Shanghai，空字符串转 null。省略时，规则没有起始时刻条件。 |
| `effective_end` | 带时区 timestamp 或 null | null | 规则结束生效的绝对时刻；无时区按 Asia/Shanghai，空字符串转 null。省略时，规则没有结束时刻条件。填写起止时刻时，结束时刻晚于开始时刻。 |

| scope 字段 | 输入要求 |
|---|---|
| `grid[].x/y/z` | 三个非负整数；与 geometry 二选一。通过 scene.grid 解析，越界或缺少网格则拒绝。 |
| `grid[].geometry` | POINT Z 或 POLYGON Z；经纬度和 EG 高度。 |
| `geom[].type` | CONTROL_ZONE / NO_FLY_ZONE / ADMIN_DIVISION / AIRSPACE / LANDING_SITE。 |
| `geom[].geom` | CONTROL_ZONE/NO_FLY_ZONE/AIRSPACE 使用 POLYGON Z，LANDING_SITE 使用 POINT Z；行政区不使用此字段。 |
| `geom[].center_point` | 管制区和起降场使用 POINT Z；行政区使用 POINT；禁飞区和空域不使用此字段。 |
| `geom[].boundary_2d` | 二维 POLYGON/MULTIPOLYGON。除 LANDING_SITE 外，每个要素必须提供 geom 或 boundary_2d，不能只给中心点；同时有 geom 与 boundary_2d 时，水平足迹须在 0.1 m 容差内一致。 |

BUILDING/WEATHER/SIGNAL/WIDTH 使用 scope.grid；NO_FLY/ZONE/AREA 使用 scope.geom；PRIORITY 可使用两种范围。其余规则不带 scope。缺省 scope 时按场景及资源引用执行。scope 不能代替天气、信号等数据集的实际数值。SEPARATION 不接受 scope，仍执行全局多机间隔检查；区域邻近提醒归属 ENG-11，不在本次 ENG-06/07 交付范围。

未选择有效 CLIMB/TURN 规则时，默认约束为爬升 5 m/s、下降 3 m/s、转角 90°、最小半径 10 m；机型与航路约束继续取更严格值。下面各规则的字段默认值只描述一条显式规则的参数。

数值参数使用 JSON number。布尔参数使用 `true` 或 `false`。下表中的“未设置”表示这条规则不增加该项限制。飞机、航路和任务的相关参数继续参与计算。

在 [`data/benchmarks/mixed_scene.json`](data/benchmarks/mixed_scene.json) 中，`rule-规则类型` 对应每种规则的完整实例。相关任务见 [`data/benchmarks/mixed_tasks.json`](data/benchmarks/mixed_tasks.json)。

<a id="rule-height-layer"></a>
### `HEIGHT_LAYER`：飞行高度与高度层

至少填写一个高度上界或下界。

| `content` 字段 | 类型 / 单位 | 默认 | 含义与修改方法 |
|---|---|---|---|
| `minHeight` | number / m | 未设置 | 飞行高度下界。与 `maxHeight` 配对使用。 |
| `maxHeight` | number / m | 未设置 | 飞行高度上界。设为 140 表示最高 140 m。 |
| `minAltitude` | number / m | 未设置 | 另一组高度下界字段。与 `maxAltitude` 配对使用。高度检查优先读取 `minAltitude`。 |
| `maxAltitude` | number / m | 未设置 | 另一组高度上界字段。高度检查优先读取 `maxAltitude`。 |
| `layerStep` | number > 0 / m | 未设置 | 水平巡航的高度层间距。下界为 60、间距为 20 时，高度层为 60、80、100 m 等。爬升和下降段连接各层。 |
| `altDatum` | `EG/MSL/AGL/ELLIPSOID/QNE/QNH` | `EG` | 上述高度的基准。设置 `AGL` 时，同时填写场景地形。其他换算依据见[坐标字段](#coordinates)。 |

修改高度时使用一组上下界。`layerStep` 以 `minHeight` 为层起点；省略 `minHeight` 时使用 `minAltitude`，两者均省略时使用 0。

混合集使用 `minHeight=60`、`maxHeight=140`、`altDatum=EG`。

<a id="rule-width"></a>
### `WIDTH`：航路宽度

| `content` 字段 | 类型 / 单位 | 默认 | 含义与修改方法 |
|---|---|---|---|
| `width` | number > 0 / m | 必填 | 航路全宽。设为 20 表示以中心线为基准，左右各 10 m。 |
| `bufferWidth` | number >= 0 / m | 0 | 走廊外侧保护带宽度；不增加可飞行宽度。 |

用 `resource_ids` 选择航路。空数组选择场景中的航路。中心可用半宽为 `min(width, widthMeters)/2 - radius_m - safety_margin_m`。两个 width 字段都使用全宽；`bufferWidth` 用于走廊外侧保护。

修改 width 改变可用全宽；修改 bufferWidth 改变外侧障碍保护距离。障碍在有效高度和时间内进入已使用航路段中心线 `min(width,widthMeters)/2 + bufferWidth` 的保护范围时，该段不可用；外部占用网格及选定规避规则的 scope 几何同样参与检查。只检查 WIDTH 与规避规则实际生效的时间和空间交集。

<a id="rule-direction"></a>
### `DIRECTION`：飞行方向

至少填写 `direction` 或 `directions`。

| `content` 字段 | 类型 / 单位 | 默认 | 含义与修改方法 |
|---|---|---|---|
| `direction` | `ONE_WAY/BIDIRECTIONAL` | 未设置 | `ONE_WAY` 使用航路中心线顺序。航路声明 `REVERSE` 时使用反向。`BIDIRECTIONAL` 允许双向。 |
| `directions` | 非空 number[] / 度 | 未设置 | 允许的水平航向。北为 0°，东为 90°，南为 180°，西为 270°。每个值为 `[0,360)`。填写后按这些航向检查。 |

混合集使用 `direction=ONE_WAY`。修改中心线顺序或航路 `direction` 可改变沿航路的通行方向。填写 `directions=[0,90]` 可指定东向与北向航段。

<a id="rule-speed"></a>
### `SPEED`：飞行速度

| `content` 字段 | 类型 / 单位 | 默认 | 含义与修改方法 |
|---|---|---|---|
| `minSpeed` | number >= 0 / m/s | 0 | 移动巡航段的最低速度。起停加减速段按运动模型处理。 |
| `maxSpeed` | number > 0 / m/s | 未设置 | 最高速度。与机型最高速度取较小值。 |
| `cruiseSpeed` | number > 0 / m/s | 未设置 | 计划巡航速度。与机型巡航速度及最终最高速度取较小值。 |

混合集使用 `maxSpeed=4`、`cruiseSpeed=4`。提高巡航速度会改变航时和多机占用时序。最低速度不大于最高速度。

<a id="rule-time"></a>
### `TIME`：每日时间与工作日

时间使用 `Asia/Shanghai`。时刻字符串使用 `HH:mm`。这些字段是每日钟表时刻，区别于规则外层和任务中的带日期 timestamp。

| `content` 字段 | 类型 | 默认 | 含义与修改方法 |
|---|---|---|---|
| `dailyStart` | string，`HH:mm` | 未设置 | 每日开放开始时刻。与 `dailyEnd` 一起填写。 |
| `dailyEnd` | string，`HH:mm` | 未设置 | 每日开放结束时刻。结束时刻早于或等于开始时刻时，窗口跨到次日。 |
| `weekdays` | 非空 int[] | 每周全部 7 天 | 星期编号为 0–6。0 为周日，1 为周一，6 为周六。跨午夜窗口使用其开始日期的星期。 |
| `earliestDeparture` | string，`HH:mm` | 未设置 | 任务起飞当天的最早起飞时刻。 |
| `latestArrival` | string，`HH:mm` | 未设置 | 任务到达当天的最晚到达时刻。 |

混合集使用周五 `[5]`、开放 `09:00` 至 `10:00`、最晚到达 `09:59`。修改每日开放时间时，同时修改开始与结束字段。

<a id="rule-area"></a>
### `AREA`：允许区与排除区

| `content` 字段 | 类型 | 默认 | 含义与修改方法 |
|---|---|---|---|
| `scope` | `BBOX/WKT/INCLUDE/EXCLUDE` | 必填 | BBOX/WKT 定义允许区域；兼容 INCLUDE 包含引用区域、EXCLUDE 排除引用区域。 |
| `minLon/minLat/maxLon/maxLat` | number，度 | BBOX 必填 | 西、南、东、北边界；最大值须大于最小值。 |
| `areaWkt` | POLYGON/MULTIPOLYGON 字符串 | WKT 必填 | 允许区域的 WGS84 几何。 |
| `adminCodes` | 非空 string[] | 未设置 | 匹配 area/grid 数据格中的 `adminCode`、`gridCode` 或 `code`。 |
| `gridCodes` | 非空 string[] | 未设置 | 匹配数据格编码、`grid.cells[].code` 或 `grid.codes`。 |
| `zoneIds` | 非空 string[] | 未设置 | 增加按 ID 引用的几何区域。与外层 `resource_ids` 合并使用。 |

用外层 `resource_ids`、`zoneIds` 或区域编码定位实际区域。服务对包含区扣除机体净空，对排除区增加机体净空。

混合集使用 `scope=INCLUDE`，引用 `allowed-route-bands`。修改引用 ID 可选择另一组允许区。修改 `scope=EXCLUDE` 可将引用区域设为排除区。

<a id="rule-climb"></a>
### `CLIMB`：爬升与下降

| `content` 字段 | 类型 / 单位 | 默认 | 含义与修改方法 |
|---|---|---|---|
| `maxClimbRate` | number >= 0 / m/s | 未设置 | 最大爬升速度。与机型、航路的爬升速度取较小值。 |
| `maxClimbAngle` | number，0<值≤90 / 度 | 未设置 | 爬升轨迹相对水平面的最大角度。降低数值需要更长的水平距离。 |
| `maxDescentAngle` | number，0<值≤90 / 度 | 未设置 | 下降轨迹相对水平面的最大角度，填写正值大小。 |
| `maxDescentRate` | number >= 0 / m/s | 未设置 | 最大下降速度。填写正值的速度大小。0 表示本规则不允许下降。 |

混合集将两项都设为 1 m/s。降低数值会增加升降时间。

<a id="rule-turn"></a>
### `TURN`：转向角与转弯半径

| `content` 字段 | 类型 / 单位 | 默认 | 含义与修改方法 |
|---|---|---|---|
| `maxTurnAngle` | number，0–180 / 度 | 180 | 一次连续同向转弯的最大累计角。0 表示只允许保持方向。 |
| `minTurnRadius` | number >= 0 / m | 0 | 最小转弯半径。与机型半径取较大值。正值生成圆弧转弯。 |

混合集使用 `maxTurnAngle=100`、`minTurnRadius=0`。增大最小半径需要更宽的转弯空间。零半径机型可停车转向，并按偏航速度计时。

<a id="rule-separation"></a>
### `SEPARATION`：多机间隔

当前执行全局间隔规则。携带外层 scope 会被拒绝，区域邻近提醒尚未交付。

| `content` 字段 | 类型 / 单位 | 默认 | 含义与修改方法 |
|---|---|---|---|
| `horizontalSeparation` | number >= 0 / m | 0 | 同时飞行的水平间隔要求。 |
| `verticalSeparation` | number >= 0 / m | 0 | 同时飞行的垂直间隔要求。 |

服务将规则要求与机体尺寸、安全裕度、动态障碍间隔合并。水平和垂直距离同时小于对应要求时产生冲突。混合集将两项都设为 12 m。增大间隔会改变延迟起飞和避让安排。

<a id="rule-no-fly"></a>
### `NO_FLY`：禁飞区

| `content` 字段 | 类型 / 单位 | 默认 | 含义与修改方法 |
|---|---|---|---|
| `noFlyMode` | `STANDARD/PERMANENT/TEMPORARY` | 必填 | STANDARD 按有效期执行硬禁飞；也接受永久或临时禁飞。临时禁飞同时填写外层 `effective_start/effective_end`。 |
| `heightCeiling` | number >= 0 / EG m | 未设置 | 本条禁飞规则检查的最高海拔。规则同时使用所引用管制区的原始高度范围。 |
| `includeBuffer` | boolean | false | 保留的附加缓冲开关。当前管制区模型的附加缓冲为 0，因此 true 与 false 使用相同几何。实际飞机净空由机型的 `radius_m/height_m/safety_margin_m` 提供。 |

用 `resource_ids` 选择管制区。混合集引用 `no-fly-0`，使用永久禁飞，并将 `includeBuffer` 设为 true。管制区自身的禁入几何和有效期由场景执行。扩大禁飞区时，修改管制区 `geom`。

<a id="rule-building"></a>
### `BUILDING`：建筑避让与批准通道

| `content` 字段 | 类型 / 单位 | 默认 | 含义与修改方法 |
|---|---|---|---|
| `avoidMode` | `STRICT/BUFFER/CORRIDOR` | 必填 | `STRICT` 执行硬规避；`STRICT` 执行硬规避；`BUFFER` 绕开建筑及缓冲。`CORRIDOR` 使用显式批准通道。 |
| `safeDistance` | number >= 0 / m | 0 | 在飞机净空和障碍自身缓冲之外，增加的水平距离。 |
| `bufferRadius` | number >= 0 / m | 0 | `BUFFER` 模式的缓冲半径。该模式使用 `max(safeDistance,bufferRadius)`。 |

用 `resource_ids` 选择障碍体。`CORRIDOR` 模式同时引用 kind 为 `corridor` 的数据集。通道几何容纳完整飞机净空后，服务才将该部分作为穿行通道。

混合集使用 `avoidMode=BUFFER`、`safeDistance=8`。增大距离会扩大建筑避让范围。

<a id="rule-zone"></a>
### `ZONE`：管制级别

至少填写一项参数。

| `content` 字段 | 类型 | 默认 | 含义与修改方法 |
|---|---|---|---|
| `zoneLevels` | 非空 string[] | 全部级别 | 按管制区的 `level` 选择级别。服务将管制区 level 转成字符串后匹配。 |
| `coverageThreshold` | number，0–1 | 未设置 | 记录覆盖阈值。当前碰撞检查使用真实管制体；此值不改变碰撞判定。 |

用 `resource_ids` 进一步选择管制区。混合集选择 `zone-0`，级别为 `["restricted"]`，阈值为 0。修改 `zoneLevels` 可改变本条规则选择的管制级别。

<a id="rule-weather"></a>
### `WEATHER`：天气条件

至少填写一项参数。用 `resource_ids` 引用 kind 为 `weather` 的版本化数据集。数据格提供风、降水和结冰值。

| `content` 字段 | 类型 / 单位 | 默认 | 含义与修改方法 |
|---|---|---|---|
| `maxWindSpeed` | number >= 0 / m/s | 未设置 | 路线允许的最大风速。读取 `wind_speed_mps`，兼容 `windSpeed`。 |
| `minVisibility` | number >=0 / m | 未设置 | 最小能见度。天气单元 `values.visibility_m` 必须提供同单位的有效观测值。 |
| `noPrecipitation` | boolean | false | true 时只通过降水值为 0 或 false 的格子。读取 `precipitation`，兼容 `precipitation_mm_h`。 |
| `noIcing` | boolean | false | true 时只通过 `icing` 为 0 或 false 的格子。 |

混合集引用 `weather-v1`，最大风速为 8 m/s，降水与结冰开关都为 true。修改阈值时同步使用对应时段和位置的天气数据。

<a id="rule-signal"></a>
### `SIGNAL`：信号覆盖

用 `resource_ids` 引用 kind 为 `signal` 的数据集。频段信号放在 `values.bands` 中，例如 `{"2.4GHz":-60,"5.8GHz":-70}`。统一强度可放在 `values.signal_strength_dbm` 中。

| `content` 字段 | 类型 / 单位 | 默认 | 含义与修改方法 |
|---|---|---|---|
| `minSignalStrength` | number / dBm | 必填 | 最低信号强度。-70 dBm 比 -85 dBm 强。提高到 -70 会要求更强信号。 |
| `bands` | 非空 string[] | 机型 `supported_bands` | 检查的频段名称。名称与信号数据和飞机支持频段对应。匹配忽略大小写、空格及 Hz 后缀。 |
| `band_policy` | `any/all` | `any` | `any` 要求至少一个支持频段达标。`all` 要求列出的全部频段均由机型支持且达标。 |

混合集引用 `signal-v1`，检查 2.4 GHz 和 5.8 GHz，最低 -85 dBm，策略为 `all`。

<a id="rule-priority"></a>
### `PRIORITY`：路线代价偏好

两项参数都必填。本规则影响路径候选代价，任务排序使用 `Task.priority`。

| `content` 字段 | 类型 | 默认 | 含义与修改方法 |
|---|---|---|---|
| `priorityScene` | `PLANNING/LOW_RISK/FAST/QUIET` | 必填 | `LOW_RISK` 优先低风险区；引用 kind 为 `risk` 的数据集。`QUIET` 优先低噪声区；引用 kind 为 `noise` 的数据集。`FAST`、`PLANNING` 将适用范围内的基础代价乘以 `1-costBonus`，不放宽硬约束。 |
| `costBonus` | number，0–1 | 必填 | 候选代价的优惠系数。0 表示无优惠，数值越大，偏好作用越强。 |

风险使用 `values.risk`。安静偏好使用 `values.noise_db`。混合集引用 `noise-v1`，使用 `QUIET` 和 `costBonus=0.8`。

<a id="rule-hover"></a>
### `HOVER`：业务航点悬停

| `content` 字段 | 类型 / 单位 | 默认 | 含义与修改方法 |
|---|---|---|---|
| `maxHoverTime` | number >= 0 / s | 未设置 | 单次连续悬停的最长时间。 |
| `defaultHoverTime` | number >= 0 / s | 0 | 业务航点声明悬停、且 `hover_s=0` 时使用的时长。 |

在任务中添加业务航点。填写正的 `hover_s`，或将 `hover` 设为 true。填写 `defaultHoverTime` 时，其值不大于 `maxHoverTime`。机型 `can_hover` 为 true 时执行悬停。

混合集使用默认 1.5 s、最长 1.6 s。修改规则时同步修改业务航点的悬停时长。

<a id="rule-endurance"></a>
### `ENDURANCE`：航程与续航

| `content` 字段 | 类型 / 单位 | 默认 | 含义与修改方法 |
|---|---|---|---|
| `maxRange` | number > 0 / m | 未设置 | 本任务路线长度上限。 |
| `minEndurance` | number > 0 / s | 未设置 | 要求机型的 `endurance_s` 至少达到此值。此项检查机型容量。 |

服务同时检查实际路线的长度和持续时间，以及上游剩余航程和剩余可飞时间。悬停计入路线持续时间。混合集使用 `maxRange=600`、`minEndurance=1200`。

<a id="rule-payload"></a>
### `PAYLOAD`：载荷能力与实际载荷

| `content` 字段 | 类型 / 单位 | 默认 | 含义与修改方法 |
|---|---|---|---|
| `maxPayloadKg` | number >= 0 / kg | 必填 | 机型的载荷能力至少达到此值。任务实际载荷同时不超过此值和机型能力。 |

混合集使用 `maxPayloadKg=0.1`。提高任务载荷时，同步修改 `Task.payload_kg`、本规则参数和[机型载荷能力](#profiles)。

<a id="rule-waypoint"></a>
### `WAYPOINT`：业务航点

| `content` 字段 | 类型 / 单位 | 默认 | 含义与修改方法 |
|---|---|---|---|
| `maxWaypoints` | int >= 0 | 未设置 | 仅计 `task.waypoints` 途经点数量；起点、终点和输出采样点均不计入。 |
| `minWaypointInterval` | number >= 0 / m | 0 | 相邻业务航点之间的三维直线距离下限。 |

混合集使用 `maxWaypoints=4`、`minWaypointInterval=10`。增加业务航点时，将数量设置为途经点数量，不包含起点和终点。

<a id="rule-rtl"></a>
### `RTL`：低电返航

三项参数都必填。`rtlOnLowBattery=true` 时，在任务 `state` 中填写 `battery_percent`、`remaining_flight_time_s`、`remaining_range_m` 和 `home_wkt`。

| `content` 字段 | 类型 / 单位 | 默认 | 含义与修改方法 |
|---|---|---|---|
| `rtlAltitude` | number >= 0 / EG m | 必填 | 返航水平飞行阶段的最低 EG 椭球高。服务在起点和返航点之间安排升降与水平飞行。 |
| `rtlOnLowBattery` | boolean | 必填 | true 时启用按当前电量触发返航。 |
| `lowBatteryPercent` | number，0–100 / % | 必填 | 电量小于或等于此值时触发返航。 |

触发后，服务以 `home_wkt` 为终点，并生成返航路线。结果 `action=return_to_home`。服务检查返航路线与当前剩余时间、航程。平台使用该结果执行飞行。

混合集使用最低返航 EG 椭球高 100 m、电量阈值 20%。提高阈值可更早触发返航。

## 搜索与完整途经网格

Task.search 接受 algorithm=ASTAR/DIJKSTRA、neighbors=6/18/26、可选 grid_level 和 max_expanded_nodes。层级须与 scene.grid.level 一致。扩展预算默认 null，表示不设固定上限；多个航段共用预算，耗尽返回 SEARCH_BUDGET_EXCEEDED。

显式 search 表示按指定网格算法搜索。未填写时沿用组合规划。search_report 给出实际扩展数。traversed_grids 是完整运动穿越网格，不只是关键点：每项有 grid_indices、可空 grid_code 和相对场景零点的 entry_time_s/exit_time_s。grid_context 给出这份列表采用的原点、分辨率、shape 和层级。原有 keypoints 的索引仍对应内部规划网格。

## 显式航路图输入

完整样例在 [network_request.json](data/features/network_request.json)。修改 `scene.airway_graph.nodes[].point_wkt` 设置节点位置，固定用 EG 椭球高。每条边引用 from_node、to_node 和 airway_id；修改 weights 中的非负数值设置边权，task.network.weight_field 选择其中一个字段。direction 指定方向，status=CLOSED 关闭边。图版本与 scene.network_version 一致。

示例任务省略 start_wkt/goal_wkt，由 A、D 两个节点生成准确端点。修改 task.network 的节点 ID 可更换起终点；如果另外填写 WKT，须与对应节点一致。via_node_ids 设置必须经过的业务节点。network_route 给出最终节点、边序列和代价。

[network_analyze_request.json](data/features/network_analyze_request.json) 调用 POST /api/v2/networks/analyze。弱连通分量忽略方向，强连通分量要求相互可达；活动边列表已过滤关闭状态、时间和关联航路状态。

[search_request.json](data/features/search_request.json) 使用 start_grid/goal_grid 索引格中心。索引对应 scene.grid，均从 0 开始；起终点不能超出 shape。请求填写 search.grid_level=12，与 scene.grid.level 一致。

批量和异步请求可设置 `execution={"workers":2,"batch_size":100}`。`workers` 是候选计算最大线程数，`batch_size` 是每组候选提交数；两者为正整数，`batch_size=null` 表示一组。全部任务统一避让。响应 `execution_report.actual_workers` 是实际执行线程数，`batches` 是提交组数，`candidate_count` 是进入统一预约的候选数。最终结果读取 `success_count/failed_count/conflicts`。

### 2026-10-30 新计算用例

| 完整输入 | 用途 | 主要修改位置 |
|---|---|---|
| `data/features/network_request.json` | 按航路图权重选路 | 修改 `scene.airway_graph.nodes/edges` 与关联航路；`task.network` 选择节点、版本及权重字段。示例选 A→B→D，原始 `cost` 合计 2 |
| `data/features/network_analyze_request.json` | 查询连通性 | 修改同一航路图和 `at`；响应区分弱连通、强连通和失效边，E 是孤立节点 |
| `data/features/search_request.json` | 网格起终点与指定搜索 | 修改 `grid` 的层级、原点、分辨率、shape 和编码，再修改 `start_grid/goal_grid`；示例使用第 12 层、6 邻域 DIJKSTRA；天气场能见度 5000 m，规则最低 3000 m |

网格端点没有 WKT 时使用对应单元格中心。若同时提交 WKT，该点必须位于指定格内。图端点没有 WKT 时使用图节点位置。图规划使用 `task.network`；网格搜索使用 `task.search`，两者不同时指定。

### 多目标参数

ENG-07/free 任务可增加 `objectives`。`weights.distance/time/energy/risk` 为非负权重，默认 1/0/0/0。`normalization.distance_m/time_s/energy_wh/risk_s` 是正归一化尺度，默认均为 1。总分为每项目标的原始值除以尺度再乘权重后求和，分数越小越优。

能耗目标需提供 `energy_model.mass_kg/hover_power_w/cruise_power_w/climb_efficiency`，输出单位 Wh。风险目标需提供带版本的 risk 数据集，`values.risk` 为非负风险值，输出为风险×秒。自定义代价使用 cost 数据集的 `values.cost` 和 `cost_layers`：每项填写 `id/dataset_id/weight/normalization/basis/unit`；basis 为 distance 或 time，分别积分代价×米或代价×秒。图层 ID 不使用 distance/time/energy/risk。

`max_iterations/max_solutions` 可填写正整数，也可保持 null。前者限制候选标签扩展次数，后者限制生成的完整候选路径数。响应 `optimization` 说明候选生成是否结束、是否遇到调用方预算，以及实际候选和方案数。`objectives.values` 给出每项 raw、normalized、normalization、unit、weight；`weighted_score` 是总分。`alternatives` 保存其他非支配完整方案，比较范围为已生成并通过验证的候选。批量默认执行主方案。

| 完整输入 | 用途 | 修改方法 |
|---|---|---|
| `data/features/objectives_request.json` | 多目标与两个自定义代价图层 | 修改 weights 和 normalization 调整主方案；修改 energy_model 使用实际机型测量数据；修改 risk/north/south 数据集及版本更新风险和业务代价。示例同时返回短直线与低风险绕行方案 |

## 增量重规划字段与修改方法

完整输入：[replan_request.json](data/features/replan_request.json)。本例在原路径前方新增 block 障碍，复用安全前段并重算后续路线。

| 请求字段 | 必填 / 默认 | 填写方法 |
|---|---|---|
| `previous_scene` | 必填 Scene | 前次规划使用的原始场景 |
| `previous_task` | 必填 Task | 前次规划的原始任务 |
| `previous_route` | 必填 RouteResult | 前次完整规划响应，保留 WKT、时间、运动段及坐标上下文；静止任务可为 POINT |
| `scene` / `scene_id` | 二选一 | 最新场景，或其已登记 ID |
| `remaining_task` | 必填 Task | 当前起点、目标及尚未完成的业务航点；departure_time 等于 current_state.at |
| `current_state` | 必填 CurrentState | position_wkt、velocity_xyz_mps、at |
| `event` | 必填 ReplanEvent | 变化事件与变化量或资源引用 |
| `trigger` | 默认 {} | 数值阈值、频率和事件历史 |
| `time_budget_s` | 默认 null | 本次证明、复用、搜索和最终验证共用的计算时限 |

`current_state.position_wkt` 使用 remaining_task.alt_reference。`velocity_xyz_mps=[vx,vy,vz]` 是当前场景局部东、北、上速度，单位 m/s。`at` 是带时区时刻，位于最新场景时域内；remaining_task 起点与当前位置一致。前后场景使用相同投影、高度转换上下文和 time_origin，前次路线按其原始场景独立证明。

| event 字段 | 填写方法 |
|---|---|
| `id` | 非空业务事件 ID |
| `type` | obstacle、dynamic_obstacle、rule、dataset、aircraft_state、mission、scene、weather、control_zone、airspace |
| `at` | 事件发生时刻，不晚于 current_state.at |
| `old_value/new_value` | 同时提供的有限数值，用于变化阈值 |
| `changed_resource_ids` | 已变化对象 ID；在前次或最新场景中存在，可表示新增、删除或更新 |

事件提供数值 old/new 或变化资源 ID 至少一组。天气使用 weather/dataset，管制区使用 control_zone/airspace，数据和几何的实际更新放入最新 Scene。

`trigger.threshold` 为非负绝对变化阈值，与事件数值同单位；启用后提供 old/new。`min_interval_s` 为非负秒数，`max_frequency_hz` 为正频率；同时提供时执行较长间隔。`last_event_time/last_replan_time` 由平台保存并回传，均不晚于当前状态时刻。事件不晚于 last_event_time 时为 stale_event；变化量小于阈值时为 below_threshold；未到下次允许时刻时为 frequency_limited。忽略事件的响应不返回执行路线。

| 响应字段 | 含义 |
|---|---|
| `decision` | reused：复用已验后续路线；replanned：已重算；ignored：未触发；failed：本次未生成可执行路线 |
| `reason` | 本次复用、重算、忽略或失败原因 |
| `route` | 完整 RouteResult 或 null；包含实际初始速度和独立验证结果 |
| `current_state` | 本次输入的当前位置、速度和时刻 |
| `reused_primitives/reused_distance_m/reused_time_s` | 实际复用的原语数、米数和秒数 |
| `new_search_calls` | 本次实际搜索调用数；未受影响后续路线可为 0 |
| `recomputed_from_s` | 重算接续时刻，相对场景 time_origin |
| `next_eligible_time` | 频率限制后的下次允许时刻，其他情况为 null |
| `violations` | 本次失败的约束及原因 |

服务复核原路线并裁剪已完成部分。后续路线有效时直接复用；受影响时，复用安全前段，按当前速度减速至可接续状态，再搜索后续路线。减速、原地偏航和等待均计入运动时间、预算和占用。最终检查完整剩余任务的航点、时间窗、续航、返航及动态冲突。多目标任务比较接续后的完整候选，并返回主方案与非支配备选。

使用新场景和 remaining_task（作为 task），复制 route 的 trajectory_wkt、vertex_times_s、motion_segments、initial_velocity_xyz_mps 到 `/api/v2/validate`。普通规划的初始速度默认 [0,0,0]；飞行中重规划使用响应的实际值。

### 能力查询新增字段

| GET /api/v2/capabilities 字段 | 含义 |
|---|---|
| search_algorithms / grid_neighbors | 当前可用搜索算法和邻域 |
| airway_graph_supported / network_analysis_supported | 航路图规划与连通分析支持状态 |
| traversed_grids_supported | 完整途经网格支持状态 |
| batch_execution_supported | 可配置并行提交支持状态 |
| objective_optimization_supported / objective_dimensions | 多目标支持状态及基础目标维度 |
| cost_layer_bases | 自定义代价可按 distance 或 time 积分 |
| incremental_replanning_supported / replanning_engine_ids | 增量重规划支持状态及适用引擎 |
| initial_velocity_validation_supported | 初始速度独立验证支持状态 |
