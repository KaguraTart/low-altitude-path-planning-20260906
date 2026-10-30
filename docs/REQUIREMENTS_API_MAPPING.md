# 路径规划业务需求与 API 对照

文档日期：2026-10-30。

本表按交付清单“路径规划”的 **901–916 共 16 项业务功能**核对接口。后端按业务功能选择接口、设置参数和读取结果。内部引擎编号及规则分配另行记录，不作为本表的功能分类。

服务地址：`https://api.kaguratart.com`。调用时在表中路径前加服务地址。字段见 [API 说明](API.md) 和 [测试 JSON 字段说明](../test_api/API_FIELDS.md)，在线调用见 [Swagger](https://api.kaguratart.com/docs#/)。

状态：**已实现**表示已有对应计算能力；**部分实现**列出可用部分及待接入部分；**暂无**表示本服务没有对应业务接口。平台负责结果持久化、业务日志和展示。

## 业务需求逐项对应

| 编号 | 业务功能 | 需求说明 | 状态 | 对应接口 | 怎么用、看什么结果 |
|---|---|---|---|---|---|
| 901 | 航路连通性路径规划 | 提供基于航路划设连通性的路径规划 | 已实现 | `POST /api/v2/plan/single`；`POST /api/v2/plan/batch`；`POST /api/v2/networks/analyze` | `task.route_mode="strict"`（默认），`scene.airways` 提供航路。显式路网使用 `scene.airway_graph` 和 `task.network` 的起终节点、途经节点及权重字段。读取轨迹和 `network_route` 节点、边及代价。分析接口返回有效边及强弱连通分量；实际可飞性由规划接口判断。 |
| 902 | 三维网格无航路规划 | 提供基于立体三维网格连通性的无航路路径规划 | 已实现 | `POST /api/v2/plan/single`；`POST /api/v2/plan/batch` | 设置 `task.route_mode="free"`。提供 `scene.bounds/resolution`，起终点使用 WKT，或通过 `scene.grid` 和 `task.start_grid/goal_grid` 指定外部网格。`task.search` 可选算法、邻域及层级。读取 `trajectory_wkt`、`traversed_grids`、`grid_context` 和 `search_report`。 |
| 903 | 多场景路径支持 | 支持多种使用场景下的路径规划 | 已实现低空业务场景 | `POST /api/v2/scenes`；`GET /api/v2/scenes/{scene_id}`；`POST /api/v2/plan/single`；`POST /api/v2/plan/batch` | 为不同区域、地图和任务环境登记场景，用对应 `scene_id` 规划。`strict/prefer/free` 选择航路或自由规划。多无人机使用 `batch`，同批共用一个场景并统一避让。多场景与多无人机分别配置。 |
| 904 | 路径规划算法引擎 | 使用算法计算最优路径 | 已实现路径搜索与目标比较 | `POST /api/v2/plan/single`；`POST /api/v2/plan/batch`；`POST /api/v2/jobs` | 提交场景和起终点、机型、时间、规则等参数。自由规划用 `task.search` 选择 `ASTAR/DIJKSTRA`；航路图用 `task.network.weight_field` 选择边权。读取有效路线和代价。多目标候选比较见 912。 |
| 905 | 路径规划结果存储 | 存储路径规划的结果数据 | 暂无；平台负责持久存储 | 暂无 | 平台保存请求、原始场景、版本、完整响应和验证结果。异步作业结果仅保存在当前进程，重启后清空。 |
| 906 | 路径规划日志记录 | 记录路径规划的过程和结果 | 暂无业务日志接口；平台负责 | 暂无 | 平台按业务任务 ID 记录请求、调用时间、结果及错误。响应中的 `rule_report/search_report` 和批量 `stage_timings_ms` 可写入日志。服务运行日志由部署侧采集。 |
| 907 | 路径规划结果查询 | 查询历史路径规划记录 | 部分实现；可查询当前进程异步作业 | `POST /api/v2/jobs`；`GET /api/v2/jobs/{job_id}` | 先提交异步批量请求，保存 `job_id`，再查询状态和结果。持久历史列表、分页及重启后的记录查询接口暂无，由平台查询自己的存储。 |
| 908 | 路径规划结果可视化 | 可视化展示路径规划的结果 | 暂无；平台负责展示 | 暂无 | 平台读取规划结果的 `trajectory_wkt`、`vertex_times_s`、`keypoints` 和 `traversed_grids`，绘制地图、时间轴及网格。`POST /api/v2/coordinates/convert` 可提供绘图所需的坐标换算。 |
| 909 | 路径规划性能优化 | 优化路径规划的性能和准确性 | 已实现计算配置和耗时输出 | `POST /api/v2/scenes`；`POST /api/v2/plan/batch`；`POST /api/v2/jobs`；`POST /api/v2/plan/replan` | 用 `scene_id` 复用预编译场景，批量设置 `execution.workers/batch_size`，重规划复用已验路径。读取 `elapsed_ms/stage_timings_ms/execution_report/search_report` 及复用量。性能优化随计算执行。 |
| 910 | 路径规划 API 接口 | 提供路径规划的 API 接口供外部调用 | 已实现 | 完整清单见下表 | 使用规划、场景、验证、异步、换算、连通分析、重规划及状态接口。在 `/docs` 查看示例，通过 `/openapi.json` 获取机器可读规范。 |
| 911 | 路径规划系统监控 | 监控路径规划系统的运行状态 | 部分实现；基础状态已实现 | `GET /api/v2/health`；`GET /api/v2/health/jobs`；`GET /api/v2/version`；`GET /api/v2/capabilities` | 读取健康状态、原生核状态、作业计数、构建版本及支持能力。CPU、内存、长期指标和告警由平台或部署监控系统采集。 |
| 912 | 多目标路径规划 | 支持多目标的路径规划 | 已实现 | `POST /api/v2/plan/single`；`POST /api/v2/plan/batch` | 设置 `task.route_mode="free"` 和 `task.objectives`，填写距离、时间、能耗、风险权重及归一化尺度，可加入业务代价图层。读取 `objectives/alternatives/optimization`，比较范围为实际验证通过的候选。多个途经目的地使用 `task.waypoints`，多个飞行任务使用 `tasks`。 |
| 913 | 动态路径重新规划 | 支持动态情况下的路径重新规划 | 已实现 | `POST /api/v2/plan/replan` | 提交 `previous_scene/previous_task/previous_route`、`current_state`、`remaining_task`、最新 `scene` 或 `scene_id`、`event` 和可选 `trigger`。读取 `decision/route`、复用量、`new_search_calls` 和触发原因。平台收到环境或任务变化后调用。 |
| 914 | 路径规划安全验证 | 验证路径规划结果的安全性 | 已实现 | `POST /api/v2/validate`；规划结果同时包含自动验证 | 使用原场景和任务，提交轨迹 WKT、顶点时间、完整运动段及实际初始速度；航路图结果同时传 `network_route`。读取 `valid/violations/rule_report`。多机批量结果还需检查 `conflicts`。 |
| 915 | 路径规划结果评估 | 评估路径规划结果的有效性 | 部分实现；有效性校验和指标已实现 | `POST /api/v2/validate`；`POST /api/v2/plan/single`；`POST /api/v2/plan/batch` | 验证接口核对轨迹和约束。`metrics` 提供航程、航时等指标，`validation/rule_report` 提供有效性依据；多目标结果的 `objectives` 用于候选比较。独立业务综合评分、评级及历史方案排名接口暂无。 |
| 916 | 路径规划策略配置 | 配置路径规划的策略和规则 | 已实现计算参数；平台保存策略档案 | `POST /api/v2/scenes`；`POST /api/v2/plan/single`；`POST /api/v2/plan/batch` | 在 `scene.rules` 配置规则，用 `task.rule_ids` 选择规则；通过 `route_mode/search/objectives` 设置规划方式，用 `profile_id/aircraft/state` 设置飞机和状态。配置参与搜索、定时和验证。平台保存策略名称、版本、绑定和增删改查。 |

## 完整接口清单

| 方法 | 路径 | 业务用途 |
|---|---|---|
| GET | `/api/v2/health` | 服务健康 |
| GET | `/api/v2/health/jobs` | 当前进程作业计数 |
| GET | `/api/v2/version` | 软件与构建版本 |
| GET | `/api/v2/capabilities` | 当前版本支持能力 |
| GET | `/api/v2/examples/single` | 单任务示例 |
| GET | `/api/v2/examples/batch` | 批量示例 |
| POST | `/api/v2/scenes` | 登记和预编译场景 |
| GET | `/api/v2/scenes/{scene_id}` | 查询场景版本和编译摘要 |
| POST | `/api/v2/plan/single` | 单任务规划 |
| POST | `/api/v2/plan/batch` | 多任务规划和批内冲突协调 |
| POST | `/api/v2/validate` | 独立轨迹验证 |
| POST | `/api/v2/coordinates/convert` | WGS84 与局部米制坐标换算 |
| POST | `/api/v2/jobs` | 提交异步批量规划 |
| GET | `/api/v2/jobs/{job_id}` | 查询当前进程异步作业及结果 |
| POST | `/api/v2/networks/analyze` | 分析航路图连通性 |
| POST | `/api/v2/plan/replan` | 事件触发增量重规划 |

## 调用步骤与完整测试 JSON

1. 选择下表的业务示例，复制完整 JSON，按 [字段说明](../test_api/API_FIELDS.md) 修改地图、起终点、任务时间和约束。WKT 顺序为经度、纬度、高度；高度为 WGS84 椭球高 `EG`，单位为米。
2. 小场景直接放在请求的 `scene` 中。重复使用同一地图时，先把完整 Scene 提交到 `/api/v2/scenes`，规划时改用 `scene_id`。二者选一个。
3. 单任务检查 `status="planned"` 和 `validation.valid=true`。批量逐条读取路线状态，检查成功数量和 `conflicts`。
4. 平台保存完整请求和响应。复核时使用原场景、原任务以及完整轨迹、运动段和时间。

| 业务 | 可直接提交的输入 | 方法与路径 |
|---|---|---|
| 沿航路规划 | [single_strict_request.json](../test_api/data/examples/single_strict_request.json) | `POST /api/v2/plan/single` |
| 优先航路、允许合法绕行 | [single_prefer_request.json](../test_api/data/examples/single_prefer_request.json) | `POST /api/v2/plan/single` |
| 三维无航路规划 | [single_free_request.json](../test_api/data/examples/single_free_request.json) | `POST /api/v2/plan/single` |
| 航路图加权选路 | [network_request.json](../test_api/data/features/network_request.json) | `POST /api/v2/plan/single` |
| 航路图连通分析 | [network_analyze_request.json](../test_api/data/features/network_analyze_request.json) | `POST /api/v2/networks/analyze` |
| 外部网格与搜索配置 | [search_request.json](../test_api/data/features/search_request.json) | `POST /api/v2/plan/single` |
| 多机协同 | [batch_request.json](../test_api/data/examples/batch_request.json) | `POST /api/v2/plan/batch`；异步使用 `POST /api/v2/jobs` |
| 多目标候选比较 | [objectives_request.json](../test_api/data/features/objectives_request.json) | `POST /api/v2/plan/single` |
| 环境变化后增量重规划 | [replan_request.json](../test_api/data/features/replan_request.json) | `POST /api/v2/plan/replan` |
| 安全与有效性验证 | [validate_request.json](../test_api/data/examples/validate_request.json) | `POST /api/v2/validate` |
| 经纬度与米制换算 | [coordinates_to_local_request.json](../test_api/data/examples/coordinates_to_local_request.json) | `POST /api/v2/coordinates/convert` |

在仓库根目录执行以下命令，测试三维无航路规划：

```bash
curl --fail-with-body https://api.kaguratart.com/api/v2/plan/single \
  -H 'Content-Type: application/json' \
  --data-binary @test_api/data/examples/single_free_request.json
```

`strict` 的巡航段位于有效航路内，起降接驳使用显式接驳区域；`prefer` 优先航路，默认航路外代价为航路内的 2 倍；`free` 使用三维网格自由搜索。三种模式都执行适用的禁入空间、飞机性能和规则约束。

## 联调与交付核对

在 `test_api` 目录执行 `python run_tests.py --examples-only`，检查全部 16 个 v2 接口和专用业务示例。执行 `python run_tests.py`，再运行三套各 1000 条任务。

验证记录见 [测试报告索引](../test_api/reports/README.md)，包含构建版本、完整输入输出、接口结果和三套任务的独立复核。本次只调整业务文档分类，接口和测试 JSON 保持现有契约。

交付核对按 901–916 逐项进行。平台按表中分工接入持久存储、业务日志、历史查询、可视化、资源告警和综合评级。内部引擎编号通过 `engine_id` 等字段保留，按字段说明填写。
