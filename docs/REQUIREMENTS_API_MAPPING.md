# 路径规划需求与 API 对照

本表按需求清单的 901–916 项对应当前 v2 接口。表中使用完整公网地址，后端可直接据此接入。

Swagger：[https://api.kaguratart.com/docs#/](https://api.kaguratart.com/docs#/)。
完整字段见 [API 文档](API.md)与[测试 JSON 字段说明](../test_api/API_FIELDS.md)。

## 1. 需求对应表

“已有”表示当前接口直接提供该能力。“部分对应”列出可使用的能力，其余部分写为“暂无”。“暂无”表示没有对应的业务接口。

| 编号 | 需求 | 对应 API（方法与完整地址） | 怎么用、读取什么结果 | 对应情况 |
|---|---|---|---|---|
| 901 | 航路连通性路径规划 | `POST https://api.kaguratart.com/api/v2/plan/single`<br>`POST https://api.kaguratart.com/api/v2/plan/batch` | 每个任务填写 `route_mode="strict"`，省略时也使用该模式。场景提供 `airways`；航路外起降配置接驳区及任务接驳 ID。返回航路内的轨迹、时间和航路使用记录。 | 已有 |
| 902 | 三维网格无航路规划 | `POST https://api.kaguratart.com/api/v2/plan/single`<br>`POST https://api.kaguratart.com/api/v2/plan/batch` | 每个任务填写 `route_mode="free"`。场景提供规划范围、分辨率和障碍；外部占用网格放在 `scene.grid`，采用 `GridSpec`。可省略航路。 | 已有 |
| 903 | 多场景路径支持 | `POST https://api.kaguratart.com/api/v2/scenes`<br>`POST https://api.kaguratart.com/api/v2/plan/single`<br>`POST https://api.kaguratart.com/api/v2/plan/batch` | 为不同地图、应用和规则登记场景。规划时选择对应 `scene_id`，也可直接填写完整 `scene`。多无人机规划使用批量接口的 `tasks`。 | 已有；多场景与多机批量的用法见第 2 节 |
| 904 | 路径规划算法引擎 | `POST https://api.kaguratart.com/api/v2/plan/single`<br>`POST https://api.kaguratart.com/api/v2/plan/batch` | 提交场景与任务，执行路径搜索、运动定时和约束校验。读取 `trajectory_wkt`、`vertex_times_s`、`motion_segments`、`metrics`。异步计算也可提交到 `/api/v2/jobs`。 | 已有，随规划接口执行 |
| 905 | 路径规划结果存储 | 暂无 | 平台保存完整请求、场景原始 JSON、完整响应和版本。异步作业会在服务进程内保留结果，业务持久化由平台实现。 | 持久化存储接口暂无 |
| 906 | 路径规划日志记录 | 暂无 | 平台记录请求 ID、任务 ID、调用时间、响应状态和错误信息。服务有运行日志，当前没有业务日志写入、查询或导出接口。 | 暂无 |
| 907 | 路径规划结果查询 | `GET https://api.kaguratart.com/api/v2/jobs/{job_id}` | 用提交作业时返回的 `job_id` 查询当前进程中的作业状态和完整结果。历史业务记录从平台自己的存储中查询。 | 部分对应；当前作业查询已有，持久历史查询暂无 |
| 908 | 路径规划结果可视化 | 暂无 | 前端读取 `trajectory_wkt` 绘制线路，结合 `vertex_times_s` 播放轨迹，使用 `keypoints` 标记转弯、升降、悬停和航路切换。 | 可视化接口与页面暂无 |
| 909 | 路径规划性能优化 | 暂无 | 场景登记支持预编译复用。批量结果中的 `elapsed_ms`、`stage_timings_ms` 可用于记录耗时。没有单独的“执行性能优化”接口。 | 独立性能优化接口暂无 |
| 910 | 路径规划 API 接口 | `POST https://api.kaguratart.com/api/v2/plan/single`<br>`POST https://api.kaguratart.com/api/v2/plan/batch`<br>`GET https://api.kaguratart.com/openapi.json` | 单机调用 `single`，多任务调用 `batch`。OpenAPI 列出全部 14 个 v2 接口及请求响应模型。 | 已有 |
| 911 | 路径规划系统监控 | `GET https://api.kaguratart.com/api/v2/health`<br>`GET https://api.kaguratart.com/api/v2/health/jobs`<br>`GET https://api.kaguratart.com/api/v2/version` | 查询健康状态、原生引擎是否可用、作业计数和构建版本。CPU、内存监控及告警由平台运维系统提供。 | 部分对应；基础运行状态已有，资源监控与告警接口暂无 |
| 912 | 多目标路径规划 | `POST https://api.kaguratart.com/api/v2/plan/single`<br>`POST https://api.kaguratart.com/api/v2/plan/batch` | 一条路线依次访问多个目标时，填写 `task.waypoints`，按数组顺序访问。多个独立起终点任务放入 `tasks`，由批量接口协调规划。 | 部分对应；顺序多航点和多任务已有，目标自动排序、综合多目标优化接口暂无 |
| 913 | 动态路径重新规划 | `POST https://api.kaguratart.com/api/v2/plan/single`<br>`POST https://api.kaguratart.com/api/v2/plan/batch` | 平台触发重新计算。更新当前位置、出发时间、剩余任务和最新场景后，再次调用规划接口。多架飞机一起调整时使用批量接口。 | 可通过重新调用实现；独立重规划、自动触发接口暂无 |
| 914 | 路径规划安全验证 | `POST https://api.kaguratart.com/api/v2/validate` | 提交原场景、原任务及规划结果的 `trajectory_wkt`、`vertex_times_s`、`motion_segments`。读取 `valid`、`violations`、`rule_report`。 | 已有，具体用法见第 3 节 |
| 915 | 路径规划结果评估 | `POST https://api.kaguratart.com/api/v2/validate`<br>已有规划响应的 `metrics`、`validation`、`rule_report` | 用验证结果判断是否满足约束；读取航程、任务航时、调度延迟和规则执行情况。批量结果另提供成功数、失败数及冲突列表。 | 部分对应；有效性检查与指标已有，独立评分、评级或方案排名接口暂无 |
| 916 | 路径规划策略配置 | `POST https://api.kaguratart.com/api/v2/scenes`<br>`POST https://api.kaguratart.com/api/v2/plan/single`<br>`POST https://api.kaguratart.com/api/v2/plan/batch` | 在场景中填写 `rules`、航路、区域和机型。任务通过 `rule_ids`、`route_mode`、`profile_id` 或 `aircraft`、`priority` 选择策略。修改场景后重新登记，或直接提交新场景。 | 已有，通过场景和任务参数配置；独立策略管理接口暂无 |

将 URL 中的 `{job_id}` 替换为作业提交响应中的实际 ID。路线结果通过 `status` 表示 `planned`、`failed` 或 `timeout`。批量结果通过 `success_count`、`failed_count` 和 `conflicts` 表示本批结果。

## 2. 路径模式、场景与多无人机

### 2.1 三种模式使用同一规划接口

修改字段为单任务请求中的 `task.route_mode`，或批量请求中每个 `tasks[i].route_mode`。

| 参数值 | 用途 | 配置方法 |
|---|---|---|
| `strict` | 按有效航路连通性规划 | 默认模式。提供航路，航路外起降使用显式接驳区。 |
| `free` | 三维自由空域规划 | 提供场景范围、分辨率和障碍。已有外部网格时填入 `scene.grid`。 |
| `prefer` | 优先航路，允许合法绕行 | 航路外基础距离代价乘以 2。这个数值是路径代价系数，没有“两倍距离限制”的含义。 |

外部网格的原点、分辨率、维度、占用格和编码按 [GridSpec 字段说明](../test_api/API_FIELDS.md#grid)填写。`datasets` 中的可选网格编码数据集可按业务需要提供。

### 2.2 多场景与多无人机分别设置

- **多场景**：不同地图、应用区域或约束配置使用不同场景。通过 `POST /api/v2/scenes` 登记，再用对应的 `scene_id` 规划。
- **多无人机或多航线**：一次提交 `tasks` 数组到 `POST /api/v2/plan/batch`。引擎协调本批任务的时序并复核冲突。

一个批量请求共用一个场景。不同场景分别提交请求。每个请求选择 `scene_id` 或完整 `scene` 中的一项。

任务默认使用 `dji_mini_4_pro` 工程规划参数。规划会执行机型约束及冲突检查。业务航路、管制区、环境数据和规则由平台在场景中提供。`scene.rules` 省略时为空数组；需要执行的业务规则填写到此数组中。

### 2.3 四个直接调用示例

在仓库根目录执行以下命令。每次调用都将完整响应保存到指定文件。

```bash
# 航路连通性规划：示例任务使用 strict。
curl --fail-with-body --silent --show-error \
  https://api.kaguratart.com/api/v2/plan/single \
  -H 'Content-Type: application/json' -H 'User-Agent: UAV-API-Test/1.0' \
  --data-binary @test_api/data/examples/single_strict_request.json \
  -o /tmp/uav-requirement-strict.json

# 三维网格无航路规划：示例任务使用 free。
curl --fail-with-body --silent --show-error \
  https://api.kaguratart.com/api/v2/plan/single \
  -H 'Content-Type: application/json' -H 'User-Agent: UAV-API-Test/1.0' \
  --data-binary @test_api/data/examples/single_free_request.json \
  -o /tmp/uav-requirement-free.json

# 多无人机规划：示例包含两个任务。
curl --fail-with-body --silent --show-error \
  https://api.kaguratart.com/api/v2/plan/batch \
  -H 'Content-Type: application/json' -H 'User-Agent: UAV-API-Test/1.0' \
  --data-binary @test_api/data/examples/batch_request.json \
  -o /tmp/uav-requirement-batch.json

# 独立安全验证：示例包含原任务、轨迹、时间和完整运动段。
curl --fail-with-body --silent --show-error \
  https://api.kaguratart.com/api/v2/validate \
  -H 'Content-Type: application/json' -H 'User-Agent: UAV-API-Test/1.0' \
  --data-binary @test_api/data/examples/validate_request.json \
  -o /tmp/uav-requirement-validation.json
```

## 3. 安全验证与结果评估

这两项都有可对应的现有能力。安全验证有独立接口；结果评估使用验证接口和规划响应中的指标。

### 3.1 验证一条规划路线

调用 `POST https://api.kaguratart.com/api/v2/validate`，按以下方式组成请求：

| 输入字段 | 从哪里取得 |
|---|---|
| `scene_id` 或 `scene` | 该路线规划时使用的场景。 |
| `task` | 该路线规划时提交的完整任务。 |
| `trajectory_wkt` | 规划响应中的同名字段。 |
| `vertex_times_s` | 规划响应中的同名字段。 |
| `motion_segments` | 规划响应中的完整同名数组。移动路线使用这些运动段进行验证。 |

验证会检查任务起终点、时间窗、航点、航路、障碍与管制区、选定规则、机型运动约束，以及场景中已提供的动态轨迹。

| 返回字段 | 怎么用 |
|---|---|
| `valid` | `true` 表示本次验证通过。 |
| `violations` | 查看违规类型、原因、规则 ID、时间和上下文。通过时为空数组。 |
| `rule_report` | 查看各条规则的执行状态、原因和数据版本。 |

`/validate` 每次验证一条路线。多机批量规划会协调并复核本批路线，返回 `conflicts`。需要检查与其他在飞无人机的间隔时，在场景 `dynamic_obstacles` 中提供它们的轨迹。

### 3.2 读取有效性与规划指标

| 评估内容 | 对应结果字段 |
|---|---|
| 是否生成路线 | 单条结果 `status`。 |
| 是否满足约束 | 单条结果 `validation.valid`、`validation.violations`；也可独立调用 `/validate`。 |
| 规则执行情况 | `rule_report`。 |
| 航程 | `metrics.distance_m`，单位 m。 |
| 任务航时 | `metrics.flight_time_s`，单位 s，包含飞行中的悬停。 |
| 调度延迟 | `metrics.wait_time_s`，单位 s，表示调度附加的起飞延迟。 |
| 批量成功与失败 | `success_count`、`failed_count`。 |
| 本批冲突 | `conflicts`。 |
| 计算耗时 | 批量结果 `elapsed_ms`、`stage_timings_ms`，单位 ms。 |

独立的综合评分、等级评定和方案排名接口：**暂无**。平台可读取上述指标实现业务评价。

## 4. 动态重新规划怎么调用

重新规划由平台触发，继续使用单机或批量规划接口。

1. 将 `task.start_wkt` 更新为当前起点，将 `departure_time` 更新为本次任务开始时刻。
2. 保留或修改 `goal_wkt`，并移除已经完成的业务航点。
3. 按业务需要更新电量、剩余航时、剩余航程和返航点。
4. 更新障碍、管制区及其他无人机的后续轨迹。其他无人机轨迹放入 `scene.dynamic_obstacles`。
5. 使 `time_origin` 与 `horizon_s` 覆盖新任务时间。动态轨迹秒数相对该 `time_origin` 计算。
6. 场景有变化时重新登记并取得新 `scene_id`，或直接提交新 `scene`。
7. 调用 `/api/v2/plan/single`。多架无人机需要共同调整时，将它们放入同一次 `/api/v2/plan/batch` 请求。
8. 平台读取新轨迹并接续业务执行。

平台负责提供其他在飞无人机的轨迹。服务按本次场景和任务计算，先前请求的路线不会自动加入本次空域预约。

## 5. 异步查询与平台业务功能

较长的批量计算可使用异步作业：

1. 将与 `/plan/batch` 相同的 JSON 提交到 `POST https://api.kaguratart.com/api/v2/jobs`。
2. 保存响应中的 `job_id`，用返回的 `poll_url` 查询。
3. 请求 `GET https://api.kaguratart.com/api/v2/jobs/{job_id}`，直至状态为 `succeeded` 或 `failed`。
4. 成功执行后读取 `result`，继续检查其中的路线状态、成功数、失败数和冲突列表。

作业记录和完整结果保存在服务进程内存中，服务重启后清空。业务需要持久存储、历史检索、日志查询或可视化时，由平台保存并展示完整规划数据。对应的独立业务 API 均为**暂无**。

## 6. 相关资料

- [完整接口文档](API.md)：全部接口、模型、规则和调用示例。
- [测试 JSON 字段说明](../test_api/API_FIELDS.md)：逐项字段、单位、默认值和修改方法。
- [独立接口测试包](../test_api/README.md)：执行脚本、完整请求响应及实测报告。
- [全部接口的输入输出记录](../test_api/API_FIELDS.md#endpoints)：每个接口的完整 JSON 链接。
