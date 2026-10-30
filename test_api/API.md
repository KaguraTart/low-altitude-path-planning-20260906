# 低空无人机路径规划 API

文档日期：2026-10-30。

服务提供 `/api/v2` 接口。请求和响应使用 JSON。外部几何使用 WGS84 经纬度 WKT。接口支持航路、管制区、机型性能和 20 类规则。

完整字段以运行服务的 `/openapi.json` 为准。打开 `/docs` 可查看 Swagger 并直接调用接口。

按业务需求查找接口，见[路径规划需求与 API 对照](REQUIREMENTS_API_MAPPING.md)。该表逐项列出 ENG-06/ENG-07 的 41 项需求、平台责任、接口输入输出和验证证据，未提供的业务接口标为“暂无”。

## 目录

1. [接入与通用约定](#1-接入与通用约定)
2. [场景与输入字段](#2-场景与输入字段)
3. [全部规则](#3-全部规则)
4. [返回与错误](#4-返回与错误)
5. [完整调用示例](#5-完整调用示例)
6. [数据归档与性能记录](#6-数据归档与性能记录)

本地安装与启动：

```bash
uv pip install -e '.[server,test]' -c requirements-runtime.lock
python serve.py --host 127.0.0.1 --port 8000
curl --fail-with-body -sS http://127.0.0.1:8000/api/v2/health
```

正式 Docker 服务地址为 `http://127.0.0.1:18000`。以下示例使用该地址。

## 1. 接入与通用约定

### 1.1 端点

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
| POST | `/api/v2/plan/replan` | 前次方案 + 当前状态 + 剩余任务 + 场景/事件 → ReplanResponse |
| POST | `/api/v2/networks/analyze` | 场景选择 + 可选 at → 航路图结构连通性 |
| POST | `/api/v2/plan/batch` | 场景选择 + `tasks` → `BatchResult` |
| POST | `/api/v2/validate` | 场景、任务与带时间航线 → 违规项和规则报告 |
| POST | `/api/v2/jobs` | 与批量请求相同 → HTTP 202、`job_id`、`poll_url` |
| GET | `/api/v2/jobs/{job_id}` | 作业状态及完整批量结果 |
| GET | `/api/v2/examples/single` | 可直接提交的完整单任务请求 |
| GET | `/api/v2/examples/batch` | 可直接提交的完整双任务请求 |

所有 v2 对象拒绝未知字段，数值必须是有限 JSON number；字符串数字、布尔值充当数值、`NaN` 和 `Infinity` 均不接受。时间字段使用带时区的 ISO 8601，如 `2026-10-16T09:00:00+08:00`。WKT 只放几何，属性、时间和规则放在 JSON 字段；不支持把时间藏在 WKT M 坐标中。

### 1.2 坐标、高度与时间

外部几何使用 WGS84，轴序固定为 **经度、纬度、高度**。任务起终点和业务航点必须是 `POINT Z`，航路中心线必须是 `LINESTRING Z`。体积边界为 `POLYGON` 或 `MULTIPOLYGON` 的平面足迹，垂直范围由单独字段确定；足迹 WKT 中的 Z 不改变体积上下限。空几何、非有限坐标、自相交等无效拓扑和 `GEOMETRYCOLLECTION` 会被拒绝。

每个场景以 `coordinates.origin_lon/origin_lat` 建立局部 AEQD 投影。`x` 向东，`y` 向北，单位为米。`z = EG 椭球高 - origin_alt_ellipsoid`，单位为米。`bounds`、`resolution`、规则距离和网格使用此坐标系。`max_radius_m` 指定场景覆盖半径，默认 50000 m。可按实际范围设置更大的正数。

| `alt_reference` / `altReference` | 输入 Z 的含义 | 转成 EG 的依据 |
|------|------|------|
| `EG` / `ELLIPSOID` | WGS84 椭球高米 | 直接使用；ELLIPSOID 为兼容别名 |
| `MSL` | 海拔米 | `EG = Z + geoid_separation_m`；必须提供差值及非空 `geoid_source` |
| `AGL` | 离地米 | `EG = Z + 地形椭球高`；需要 terrain 数据或常量 `terrain_ellipsoid_m` |
| `QNE` / `QNH` | 相应气压高度米 | `EG = Z + offset_m`；需在该位置和时刻唯一有效的高度改正，`target_reference="EG"` |

服务不会猜高度基准或用零代替缺少的证据。独立坐标转换端点只接受 `CoordinateSpec` 的常量地形；场景规划会将 terrain 空间场注册为转换回调，按位置和有效时间读取。数据的来源、版本和有效期由上游负责。QNE/QNH 转换必须有带时区的 `at`；缺少修正报 `HEIGHT_REFERENCE_UNAVAILABLE`，同时匹配多条修正报 `AMBIGUOUS_HEIGHT_CORRECTION`。

`Scene.time_origin` 是所有数值时间的零点。`vertex_times_s`、动态障碍时间、关键点 `time_s`、运动段起止时间、冲突时间均为**相对这个零点的秒数**，不是相对各任务起飞的秒数。`departure_time`、有效期和时间窗仍使用绝对 ISO 时间。例如场景零点为 09:00，任务在 09:05 起飞，则首个数值时间至少是 300；为避让延迟到 09:06 时至少是 360。时间戳与几何顶点逐一对应，必须严格递增；悬停用两个相同位置、不同时间表示。

### 1.3 三种路径模式

| `route_mode` | 行为 |
|------|------|
| `strict`（默认） | 飞行器及安全裕度形成的扫掠体必须完全位于当时有效的航路体或已明确授权的起终点接驳体内。必须有航路；找不到合法路线就失败，不自动改为自由飞行 |
| `prefer` | 允许航路外飞行；候选图代价中，航路外基础距离代价为航路内的 2 倍；其余安全、规则和性能约束照常执行 |
| `free` | 不要求航路包含；其余安全、规则和性能约束照常执行 |

中心线 Z 是参考高度。`minAltitude/maxAltitude` 和 `altReference` 确定可用高度带。`widthMeters` 是航路全宽，编译时除以 2。规则 `WIDTH.content.width` 同样是全宽。可用中心半宽为 `min(widthMeters,width)/2` 再扣除飞机水平净空；`bufferWidth` 是走廊外侧保护距离，不增加允许飞行范围。障碍在有效高度和时间内进入已用航路段中心线 `min(width,widthMeters)/2 + bufferWidth` 的保护范围时，该段不可用；占用网格和选定规避规则的 scope 几何同样参与硬检查；检查只在 WIDTH 及规避规则实际生效的时间和空间交集内执行。

严格模式下，航路外起终点需显式提交 `connectors`，任务填写 `start_connector_id` / `end_connector_id`；`Connector.airway_ids` 指定可以接入的航路。接驳仅授权实际起点到首次进入航路、最后离开航路到实际终点的连续部分，不能当作中段捷径。接驳体也有高度、时间和净空限制。中心线、端点和接驳未连通时直接失败。

### 1.4 场景选择与生命周期

单任务、批量和验证请求选择 `scene_id` 或 `scene`。多轮规划时，先注册场景，再使用返回的 `scene_id`。`compilation_ms` 返回首次编译耗时。场景版本为规范化场景 JSON 的 SHA-256。相同输入复用编译结果；修改输入后生成新版本。服务不缓存任务答案。

场景和作业保存在服务进程内存中。重启后重新注册场景。登记表不设固定数量或估算存储上限，也不自动淘汰场景。新场景按顺序编译。保留场景原始 JSON，以便重新注册。服务使用单进程。

## 2. 场景与输入字段

### 2.1 外层请求

| 对象 / 字段 | 类型、必填与默认 | 说明 |
|------|------|------|
| 单任务 `task` | `Task`，必填 | 一条业务任务 |
| 批量 `tasks` | 非空 `Task[]` | ID 不可重复；结果顺序与请求顺序一致 |
| `scene_id` / `scene` | string / `Scene`，二选一 | 引用已注册场景或内联注册 |
| 规划 `time_budget_s` | number \| null，默认 null | 省略或 null 时持续计算；传正有限秒数时，在该时限内计算 |
| 批量 `request_id` | string，可省略 | 省略则生成 UUID；原样写入批量结果 |
| 批量 `execution` | ExecutionOptions/null，默认 null | 候选计算的 workers 和 batch_size；所有任务统一预约 |
| 验证 `task`、`trajectory_wkt`、`vertex_times_s` | 必填；时间数组非空 | 任务、WGS84 `POINT Z/LINESTRING Z` 与相对场景零点的时间；顶点数与时间项数相同 |
| 验证 `motion_segments` | `MotionSegment[] \| null`，模型允许省略 | 移动航线需提交此字段。复制规划结果的同名字段；缺少时返回 `valid:false`、`MOTION_MODEL_REQUIRED` |
| 验证 `network_route` | NetworkRoute/null，默认 null | 图路线可复制原响应的节点/边序列；省略时由验证器从轨迹证明有效图走法 |
| 验证 `initial_velocity_xyz_mps` | 三个数值，默认 `[0,0,0]` | 局部东/北/上初始速度，m/s；复制重规划路线的同名字段，与解析运动首速度核对 |

### 2.2 `Scene`

| 字段 | 类型 | 必填 / 默认 | 含义 |
|------|------|------|------|
| `scene_type` | `LOW_ALTITUDE` | `LOW_ALTITUDE` | 当前仅提供低空场景；地面、水上拒绝输入 |
| `network_version` | string/null | null | 平台路网版本，用于追溯；场景另有内容哈希版本 |
| `airway_graph` | AirwayGraph/null | null | 显式节点/边/权重；与 network_version 一致，结构见航路图小节 |
| `coordinates` | `CoordinateSpec` | 必填 | 投影原点与高度换算证据 |
| `bounds` | `Bounds3D` | 必填 | 局部米制 `x_min/x_max/y_min/y_max/z_min/z_max`，每轴 max 必须大于 min |
| `resolution` | `[number,number,number]` | `[25,25,10]` | 局部 x/y/z 搜索分辨率，均须 >0 |
| `horizon_s` | 正数 | 3600 | 从 `time_origin` 起的飞行时域，单位秒；与计算时限分开设置 |
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

场景对象、WKT 顶点、数据格、网格节点及估算存储不设固定上限。批量任务、业务航点、验证数据、碰撞采样和运动顶点也不设固定条目上限。服务处理完整输入，并生成完整结果。

默认不设计算时限或搜索次数。计算在得到结果、搜索结束或发生运行错误时停止。调用方可用 `time_budget_s` 指定停止时间。飞行仍遵守场景时域、机型性能和选定规则。

服务捕获到内存分配失败时，返回 HTTP 503、`RESOURCE_EXHAUSTED`。异步作业以 `failed` 和同名错误码记录失败。操作系统直接因 OOM 终止进程时，连接可能断开；Docker 记录 OOM 并按重启策略重启服务。

`GET /api/v2/capabilities` 返回当前能力。资源上限字段为 null 时，表示未设置固定上限。正式 Docker 配额保持 2 CPU / 2 GiB。

### 2.3 `CoordinateSpec` 与高度修正

| 字段 | 类型 / 默认 | 含义 |
|------|------|------|
| `origin_lon` / `origin_lat` | number，必填 | 经度 `[-180,180]`、纬度 `[-90,90]` |
| `origin_alt_ellipsoid` | number，0 | 局部 z=0 的 EG 椭球高 |
| `max_radius_m` | number，50000 | 场景覆盖半径，单位米；可设置任意正有限值 |
| `terrain_ellipsoid_m` / `terrain_source` | number / string，null | 声明的平地椭球高与来源；非平地区域应提供 terrain 空间场 |
| `geoid_separation_m` / `geoid_source` | number / string，null | 椭球高减 MSL 的差值与来源 |
| `height_corrections` | `HeightCorrection[]`，`[]` | QNE/QNH 的显式修正 |

`HeightCorrection.target_reference` 固定为 EG，`offset_m=EG−输入高度`；必填 `reference`（`QNE/QNH`）、`offset_m`、非空 `source/version`、`valid_from/valid_to`；可选 `geometry_wkt` 限定 WGS84 平面覆盖。有效期必须递增，修正采用 `valid_from <= at < valid_to`。

### 2.4 航路、障碍、接驳和管制区

| `Airway` 字段 | 类型 / 默认 | 含义 |
|------|------|------|
| `id` | string，必填 | 航路 ID |
| `centerLine` | WGS84 `LINESTRING Z`，必填 | 三维参考中心线 |
| `widthMeters` | number >0，必填 | 全宽，米 |
| `minAltitude` / `maxAltitude` | number，必填 | 硬高度上下界，max >= min |
| `altReference` | 高度枚举，必填 | 上下界和中心线的基准 |
| `startGrid` / `endGrid` | `GeographicBounds`，必填 | 端点范围元数据；规范键为 `minLon/maxLon/minLat/maxLat/minAlt/maxAlt` |
| `allowed_profile_ids` | string[]，`[]` | 航路机型准入白名单；空数组不限制。strict/prefer 中，不准入航路体为禁入区，重叠获准航路不能抵消该禁入；free 不使用航路准入 |
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

### 2.5 空间场与地形

`Dataset` 必填 `id/kind/version`；`kind` 为 `terrain/weather/signal/noise/area/grid/risk/cost/corridor` 之一；可选 `valid_from/valid_to`、`geometry_wkt`、`cells`（默认 `[]`）。数据集和单元格的有效期共同约束适用范围。

`FieldCell` 的 `geometry_wkt`（WGS84 足迹）与 `bounds`（局部 AEQD 六个边界）必须二选一；`values` 必填。WKT 格可用 `min_altitude/max_altitude` 限定 EG 椭球高；使用 `bounds` 时其局部 z 范围生效。可选 `valid_from/valid_to` 为带时区时间。未给高度界限的足迹格覆盖全高度。

| 数据类型 | `values` 允许键 | 单位与解释 |
|------|------|------|
| `terrain` | `elevation_ellipsoid_m`、`terrain_ellipsoid_m`、`nodata` | EG 米；前者为规范键。可用地形必须有有限椭球高 |
| `weather` | `wind_speed_mps`、`windSpeed`、`precipitation`、`precipitation_mm_h`、`icing`、`visibility_m`、`nodata` | 风 m/s；能见度 `visibility_m` 为非负米值；降水和结冰可为布尔或非负数，0/false 表示无 |
| `signal` | `bands`、`signal_strength_dbm`、`nodata` | `bands` 为频段到 dBm 的对象，如 `{"2.4GHz":-60,"5.8GHz":-70}`；亦可给统一强度 |
| `noise` / `risk` | `noise_db` / `risk`、`nodata` | 噪声 dB；风险数值，供代价计算 |
| `area` | `adminCode/gridCode/code/nodata` | 行政区、网格或通用编码 |
| `grid` | `adminCode/gridCode/code/occupied/nodata` | 区域编码与占用属性 |
| `cost` | `cost`、`nodata` | 非负自定义代价值，单位由任务 cost_layers 声明 |
| `corridor` | `nodata` | 明确允许穿行的几何区域 |

未知 `values` 键报 `INVALID_INPUT`。规则引用天气、信号或风险场时，缺少匹配数据集、版本、单元格或必要数值会拒绝编译。规划经过未覆盖、过期或 `nodata=true` 的相关空间场时视为不可通行。terrain 一旦配置，也必须在飞行位置与时间全程有效；不能把缺地形当平地。重叠有效地形格取较高椭球高，避免低估地面。

算法不拉取实时天气、不测量无线信号；平台向服务提供已有观测/预报和质量标记，服务据此规划与验证。实际天气采集、链路监测和飞控执行由平台完成。

### 2.6 外部占用网格 `GridSpec`

当前网格 `crs` 必须为 **`LOCAL_AEQD`**；不接受 WGS84 网格坐标。经纬度网格需先通过坐标转换并在调用方重新构建米制单元格。

| 字段 | 类型 / 默认 | 含义 |
|------|------|------|
| `crs` | string，`LOCAL_AEQD` | 必须显式保持本地 AEQD 语义 |
| `level` | 非负整数 / null | 上游网格层级标识；与任务指定层级一致 |
| `origin` | `[x,y,z]`，必填 | 第 `(0,0,0)` 格的最小角，局部米 |
| `resolution` | `[dx,dy,dz]`，必填 | 每轴格边长，均 >0 |
| `shape` | `[nx,ny,nz]`，必填 | 每轴格数，均为正整数 |
| `occupied` | `[[i,j,k],...]`，`[]` | 稀疏占用索引；从 0 开始，必须在 shape 内 |
| `occupied_ranges` | `[{"start":[i,j,k],"end":[i,j,k]},...]`，`[]` | 两端都包含的占用索引范围 |
| `cells` | `[{"bounds":{...},"code":"...","occupied":true},...]`，`[]` | 独立的显式局部边界格；`code` 可省略，occupied 默认 true |
| `codes` | `{"i,j,k":"业务编码"}`，`{}` | 规则与输出关键点使用的编码映射 |

稀疏索引、索引范围与显式边界格可以并存，作为占用体的并集。规则 `AREA.gridCodes` 引用的编码必须能解析到真实几何，不能只给没有范围的字符串。

### 2.7 `Task`、航点和当前状态

| `Task` 字段 | 类型 / 默认 | 含义 |
|------|------|------|
| `id` | 非空 string，必填 | 任务 ID |
| `start_wkt` / `goal_wkt` | `POINT Z` / null | WGS84 起终点；使用图节点或网格端点时可省略，由服务换算 |
| `network` | NetworkSelection / null | ENG-06 的图节点、途经节点和边权选择 |
| `start_grid` / `goal_grid` | `[i,j,k]` / null | 外部 scene.grid 的端点格，省略 WKT 时使用格中心；同时提供时检查坐标属于该格 |
| `departure_time` | timestamp，必填 | 最早起飞时间，必须在场景时域内 |
| `alt_reference` | 高度枚举，必填 | 本任务起终点、航点与输出 WKT 的高度基准 |
| `route_mode` | `strict/prefer/free`，`strict` | 路径模式 |
| `search` | `SearchOptions` / null | null 使用原有组合搜索；显式对象使用指定网格搜索 |
| `objectives` | ObjectiveConfig / null | ENG-07/free 的多目标计算；null 使用普通规划 |
| `engine_id` | `ENG-06/ENG-07`，可省略 | strict/prefer 自动归属 ENG-06；free 自动归属 ENG-07。显式值须与模式一致 |
| `waypoints` | `Waypoint[]`，`[]` | 按顺序经过的中间业务航点，不重复放起终点；执行 WAYPOINT 规则 |
| `earliest_arrival/latest_arrival` | timestamp/null | 终点到达窗口；最晚到达不能早于 departure |
| `profile_id` | string，`dji_mini_4_pro` | 引用场景配置或内置默认机型 |
| `aircraft` | `AircraftProfile`/null | 直接指定时优先于 profile_id |
| `state` | `AircraftState`/null | 上游当前飞机状态 |
| `payload_kg` | number >=0，0 | 实际载荷 |
| `priority` | int，0 | 越大先调度；并列再按最晚到达与 ID 排序 |
| `start_connector_id/end_connector_id` | string/null | 显式接驳资源 ID |
| `rule_ids` | string[]/null | null 为选择全部场景规则；`[]` 为不选择业务规则，基本障碍/航路/性能检查仍执行 |

`Waypoint` 必填 `wkt: POINT Z`；可选 `earliest/latest`、`hover_s`（>=0，默认 0）、`hover`（默认 false）。悬停应在业务航点标记；`hover_s` 给明确时长，`hover=true` 可配合 HOVER 规则的 `defaultHoverTime`。时间窗均须带时区。

`AircraftState` 字段均可省略：`at`（timestamp）、`battery_percent`（0–100）、`remaining_flight_time_s`、`remaining_range_m`（均 >=0）、`home_wkt`（`POINT Z`）、`alt_reference`。RTL 启用低电触发时，必须提供电量、剩余飞行时间、剩余航程与 home，不能只给一个低电标志。`state.alt_reference` 显式指定时独立解释 home 的高度，可与任务基准不同；省略或为 null 时沿用 `Task.alt_reference`，服务将 home 换算到任务输出基准。

RTL 是**根据提交时当前状态生成的返航方案**：触发后将目标改为 `home_wkt`，生成满足 `rtlAltitude` 的返航路径，并检查当前剩余航程与续航；服务不持续监听电池，也不向无人机下发返航指令。持续储备监测、实际执行与后续重规划由平台负责。

### 显式航路图与连通性

`Scene.airway_graph` 提供 `version`、`nodes` 和 `edges`。`version` 是非空路网版本；同时提供 Scene.network_version 时必须一致。图节点为 `{ "id":"A", "point_wkt":"POINT Z (...)" }`，高度固定为 EG 椭球高。

每条边包含 `id`、`from_node`、`to_node`、`airway_id` 和 `weights`。`weights` 是非负有限数值的对象，如 `{"distance_m":500,"cost":2}`。可选字段：

| 字段 | 默认 | 含义 |
|---|---|---|
| `geometry_wkt` | null | EG 的 LINESTRING Z；省略时连接两端节点；几何端点与节点偏差不超过 0.1 m |
| `direction` | BOTH | FORWARD 从 from_node 到 to_node；REVERSE 反向；BOTH 双向 |
| `status` | ACTIVE | CLOSED 边不参与规划 |
| `valid_from/valid_to` | null | 带时区的边有效时间，实际飞行与预约延迟后继续检查 |

任务选择示例：

```json
{"start_node_id":"A","goal_node_id":"D","weight_field":"cost","expected_version":"handoff-20261030","via_node_ids":[]}
```

把该对象放在 `task.network`。`weight_field` 必须存在于各边，`expected_version` 可省略；提供时与图版本一致。`via_node_ids` 是按顺序经过的显式业务节点，执行航点数量及间距规则。自动经过的中间图节点不计入业务航点数量。图节点引用与外部网格端点使用各自的输入方式；显式坐标须与所选端点一致。

服务按方向、边权、准入和规则搜索，再生成满足飞机约束的实际运动。节点表示拓扑连接，允许在有效航路内生成圆弧转弯。没有共同节点的几何交叉不形成图连接。结果 `network_route` 返回路网版本、实际节点/边序列、所选边权字段、原始权重之和 `total_cost`，以及应用优先规则后的 `effective_cost`。优先规则调整非负代价，禁飞、净空、准入等硬约束继续执行。断连返回 `NETWORK_DISCONNECTED`。

`POST /api/v2/networks/analyze` 接受 `scene` 或 `scene_id`，可选 `at`；默认使用 scene.time_origin。返回节点 ID、活动边 ID、弱连通分量、强连通分量和失效边 ID。它描述图的结构连通性；飞行器净空和运动约束由规划及验证接口执行。

完整输入见 [`network_request.json`](data/features/network_request.json) 和 [`network_analyze_request.json`](data/features/network_analyze_request.json)。网格端点与搜索配置见 [`search_request.json`](data/features/search_request.json)。

### 搜索配置与完整途经网格

`Task.search` 示例：

```json
{"algorithm":"ASTAR","neighbors":26,"grid_level":12,"max_expanded_nodes":null}
```

| 字段 | 取值 / 默认 | 修改方法 |
|---|---|---|
| `algorithm` | `ASTAR` / `DIJKSTRA`；默认 ASTAR | 选择启发式搜索或 Dijkstra 搜索 |
| `neighbors` | 整数 6、18、26；默认 26 | 6 仅连接面相邻节点；18 增加棱方向；26 再增加角方向。对角移动继续检查穿角碰撞 |
| `grid_level` | 非负整数 / null | 指定时须等于 `scene.grid.level` |
| `max_expanded_nodes` | 正整数 / null | 可选任务级扩展预算，多个途经航段共用。null 不设上限；耗尽返回 `SEARCH_BUDGET_EXCEEDED` |

搜索邻域定义搜索节点的连接。实际飞行曲线仍执行转弯半径、角度、爬降和净空要求。`search_report` 返回所选算法、邻域、层级、实际扩展数及调用方预算。

`traversed_grids` 按实际运动顺序返回全部经过的格子，每项包含 `grid_indices`、`grid_code`、`entry_time_s`、`exit_time_s`。时间以场景零点为基准。直线和圆弧按解析几何与网格面求交；等待保留占用时段。同一格连续运动会合并时段，离开后再次进入保留新记录。

`grid_context` 说明遍历采用的原点、分辨率、shape、层级及来源。有 `scene.grid` 时使用外部网格，否则使用场景规划网格。遍历使用格子上界不包含的约定，同时穿过面、棱或角时不增加无停留时间的相邻格。原有 `keypoints.grid_indices` 继续使用内部规划网格；关键点编码先查规则网格映射，再查显式单元格。

### 2.8 默认 DJI 工程规划配置

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
| `rtl_altitude_m` | 20 | 米，>=0；配置记录值。返航高度使用 `RTL.content.rtlAltitude` |
| `can_hover` | true | boolean |
| `supported_bands` | `["2.4GHz","5.8GHz"]` | 支持频段 |
| `low_battery_percent` | 20 | 0–100；配置记录值。返航阈值使用 `RTL.content.lowBatteryPercent` |
| `engineering_values` | 速度、爬降率、偏航率、加减速、转弯半径、尺寸、安全裕度、返航高度与低电阈值字段名列表 | 完整列表以 version 响应为准 |

### 多目标配置 `Task.objectives`

用于 ENG-07、`route_mode=free`。配置省略时使用普通规划。服务生成候选路线，完成运动定时和独立验证后，计算目标值并选择归一化加权总分最小的主方案。

| 字段 | 默认 / 单位 | 修改方法 |
|---|---|---|
| `weights.distance/time/energy/risk` | 1 / 0 / 0 / 0 | 非负权重；与自定义图层权重合计至少一项大于 0 |
| `normalization.distance_m` | 1 m | 距离归一化尺度，大于 0 |
| `normalization.time_s` | 1 s | 航时归一化尺度，大于 0 |
| `normalization.energy_wh` | 1 Wh | 能耗归一化尺度，大于 0 |
| `normalization.risk_s` | 1 risk·s | 风险暴露归一化尺度，大于 0 |
| `energy_model` | null | 能耗权重大于 0 时提供机体质量和功率模型 |
| `cost_layers` | [] | 追加自定义代价图层 |
| `max_solutions` / `max_iterations` | null | 调用方可指定正整数计算预算；max_solutions 限制生成的完整候选路径数，max_iterations 限制标签扩展次数；默认不设置 |

`energy_model` 包含 `mass_kg`、`hover_power_w`、`cruise_power_w` 和 `climb_efficiency`。质量、功率使用调用方给定值；效率大于 0 且不超过 1。能耗按实际运动时间、悬停和正爬升势能计算，结果单位 Wh，公式随响应返回。

风险目标使用 `kind=risk`、`values.risk` 的带版本数据集，按真实运动的时间积分得到风险暴露。自定义图层使用 `kind=cost`、`values.cost`；每项填写 `id/dataset_id/weight/normalization/basis/unit`。`basis=distance` 积分代价×米，`basis=time` 积分代价×秒；`unit` 声明积分后的业务单位，归一化尺度与该单位一致。

每个目标先用原始值除以归一化尺度，再乘权重。`objectives.weighted_score` 是各项之和。目标场应覆盖整条候选运动的时间、高度和位置。缺失或 nodata 的代价数据不按零值处理。

响应主方案及 `alternatives` 组成已验证候选中的非支配集合：一个方案各项目标均不优于另一个方案，且至少一项更差时，被剔除。`optimization.scope=validated_candidates` 表明比较范围。`status` 区分候选生成结束、迭代预算、方案预算、时间预算和搜索扩展预算。所有返回方案保留完整 WKT、时间、运动段、网格和验证结果；批量中只执行主方案，备选也检查与其他主方案的冲突。

### 增量重规划 `POST /api/v2/plan/replan`

用于 ENG-07/free。平台保留前次完整请求和响应，再提交当前状态、最新完整场景、剩余任务及变化事件。事件说明变化来源；实际约束取自最新场景。平台负责采集事件和执行新方案。

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

完整输入见 [`replan_request.json`](data/features/replan_request.json)。把响应 route 的 trajectory_wkt、vertex_times_s、motion_segments 和 initial_velocity_xyz_mps，连同最新 scene 和 remaining_task（作为 task）提交到 `/api/v2/validate`，即可独立复核。

### 批量执行配置 `execution`

同步批量请求和异步作业请求可填写 `execution`。单机请求不使用此字段。

| 字段 | 类型 / 默认 | 用法 |
|---|---|---|
| `workers` | 正整数，`1` | 候选计算的最大并行工作线程数；布尔值和字符串不接受 |
| `batch_size` | 正整数/null，`null` | 每组提交的候选任务数；null 将所有任务作为一组 |

所有候选组使用同一截止时刻。候选计算完成后，全部任务统一预约并执行连续冲突检查。`batch_size` 不拆分安全协调范围。省略 `execution` 时顺序计算。

`BatchResult.execution_report` 返回 `requested_workers`、实际参与候选计算的线程数 `actual_workers`、`batch_size`、提交组数 `batches` 和成功候选数 `candidate_count`。候选成功后仍需通过预约和最终验证，最终成功数读取 `success_count`。

## 3. 全部规则

每条 `Rule` 必填 `id`、`rule_type`、非空 `content`；兼容样例名 `type`、`constraints`，分别对应前两项业务字段。可选 `scope`、`resource_ids`（默认 `[]`）、`effective_start/effective_end`。规则有效期可输入无时区时间（按 Asia/Shanghai 解释）；空字符串转 null。其他对象时间仍必须带时区。`content` 采用下表精确键名，未知类型、未知键、错误类型、反向区间和缺少必要字段都会报 `INVALID_INPUT`。时间与空间适用范围在整段上求交，不能只检查端点。无 `resource_ids` 时按规则类别选择场景中对应资源；给出引用时每个 ID 都必须存在。

平台管理规则 CRUD、规则绑定和机型准入关系。引擎消费本次 `scene.rules`、`task.rule_ids`、`airways[].allowed_profile_ids`，执行约束；没有对应的管理端点。未选择有效 CLIMB/TURN 规则时，使用爬升 5 m/s、下降 3 m/s、转角 90°、半径 10 m 的默认约束，机型和航路限制继续取更严格值。

`scope` 是规则外层作用域，与 `content.scope` 的 AREA 类型字段不同。`scope.grid[]` 每项填 `{x,y,z}` 或 `{geometry:"POINT Z / POLYGON Z ..."}`。索引必须能通过 `scene.grid` 的原点、分辨率和维度解析；不能将北斗编码整数直接当局部索引。几何形式的 Z 使用 EG。`scope.geom[]` 每项填 `type` 和适用的 `geom/center_point/boundary_2d`，类型为 CONTROL_ZONE、NO_FLY_ZONE、ADMIN_DIVISION、AIRSPACE、LANDING_SITE。行政区使用二维边界（支持 MULTIPOLYGON）和可选 POINT 中心；其余体几何使用 POLYGON Z，起降场使用 POINT Z。非起降场要素必须提供区域 geom 或 boundary_2d，单独 center_point 不能代表区域；同时给出 geom 与 boundary_2d 时，水平足迹须在 0.1 m 容差内一致。天气、信号作用域只限定检查范围，仍须提供版本化数据值。

ENG-06 消费 HEIGHT_LAYER、WIDTH、DIRECTION、PRIORITY、SPEED、TIME、AREA、CLIMB、TURN、WAYPOINT；ENG-07 消费 HEIGHT_LAYER、BUILDING、NO_FLY、ZONE、WEATHER、SIGNAL、CLIMB、TURN、HOVER、ENDURANCE、PAYLOAD、RTL、PRIORITY、WAYPOINT。此为业务归属，通用安全校验仍执行输入中选定的适用规则。SEPARATION 保留全局多机间隔检查，携带 scope 会被拒绝；区域邻近提醒归属 ENG-11，不在本次 ENG-06/07 交付范围。

| 类型 | `content` 全部允许键 | 执行语义和数据要求 |
|------|------|------|
| `HEIGHT_LAYER` | `minHeight,maxHeight,layerStep,minAltitude,maxAltitude,altDatum` | 至少给一个高度上下界；`altDatum` 支持 EG/MSL/AGL/ELLIPSOID/QNE/QNH，默认 EG；layerStep >0，用高度层锚定巡航高度，爬降过渡仍查上下界；AGL 必须有地形 |
| `WIDTH` | `width,bufferWidth` | width 必填且 >0，表示全宽；bufferWidth >=0 是外侧保护带。需要航路，可用半宽取规则与航路全宽较小值的一半，再减飞机净空 |
| `DIRECTION` | `direction,directions` | 至少给一项；direction=`ONE_WAY/BIDIRECTIONAL`。directions 是 `[0,360)` 角度数组，北为 0°，顺时针增加；单向未给角度时按引用航路方向检查 |
| `SPEED` | `minSpeed,maxSpeed,cruiseSpeed` | m/s；max/cruise >0，min >=0，min<=max。巡航和物理起停加减速阶段区分处理 |
| `TIME` | `dailyStart,dailyEnd,weekdays,earliestDeparture,latestArrival` | 搜索将日历开放期转为起飞窗口，冲突时可重试后续窗口；日内时间 `HH:mm`，Asia/Shanghai；dailyStart/dailyEnd 必须成对，支持跨午夜；weekdays 为 0–6 整数，0 周日、1 周一至 6 周六；earliestDeparture/latestArrival 也是日内 `HH:mm` |
| `AREA` | `scope,minLon,minLat,maxLon,maxLat,areaWkt,adminCodes,gridCodes,zoneIds` | scope=BBOX 时填四个经纬度边界，WKT 时填 areaWkt；两者均为允许区域。兼容 INCLUDE/EXCLUDE 与资源/编码引用；所有模式检查完整包络 |
| `CLIMB` | `maxClimbRate,maxDescentRate,maxClimbAngle,maxDescentAngle` | 速率 m/s；角度为相对水平面的度数，0<角度≤90；与机型能力共同检查，按实际区域和有效时间执行 |
| `TURN` | `maxTurnAngle,minTurnRadius` | 角度 `0<=值<=180`，半径米 >=0；约束完整转弯累计角及几何半径 |
| `SEPARATION` | `verticalSeparation,horizontalSeparation` | 米，>=0；全局连续时空间隔，与飞机尺寸和安全裕度取保守值；不接受 scope |
| `NO_FLY` | `noFlyMode,heightCeiling,includeBuffer` | noFlyMode 必填=`STANDARD/PERMANENT/TEMPORARY`；临时规则须显式起止有效期；需要匹配管制几何；heightCeiling 为 EG 米；includeBuffer 是保留开关，当前管制区附加缓冲为 0，飞机净空单独参与检查 |
| `BUILDING` | `avoidMode,safeDistance,bufferRadius` | avoidMode 必填=`STRICT/BUFFER/CORRIDOR`；距离米 >=0；需障碍几何，CORRIDOR 还需明确 corridor 数据集，不能把建筑整体当穿行区 |
| `ZONE` | `zoneLevels,coverageThreshold` | 至少给一项；zoneLevels 为非空字符串数组；coverageThreshold 为 0–1，记录覆盖阈值；碰撞结果由真实管制体决定 |
| `WEATHER` | `maxWindSpeed,minVisibility,noPrecipitation,noIcing` | 至少给一项；风 m/s、能见度 m、后两项 boolean；天气格场提供相应数值，按位置和时间覆盖路线 |
| `SIGNAL` | `minSignalStrength,bands,band_policy` | 强度 dBm 必填；bands 为非空字符串数组；band_policy=`any`（默认）或 `all`；需 signal 格场，并匹配飞机支持频段 |
| `PRIORITY` | `priorityScene,costBonus` | 两者必填；场景=`PLANNING/LOW_RISK/FAST/QUIET`，bonus 为 0–1；LOW_RISK 需 risk 格场，QUIET 需 noise 格场，覆盖缺口不放行；影响候选路线代价 |
| `HOVER` | `maxHoverTime,defaultHoverTime` | 秒 >=0，default <= max；只允许明确标记的业务航点悬停，实际持续时间不能超限 |
| `ENDURANCE` | `maxRange,minEndurance` | 米 / 秒，均 >0；minEndurance 是机型容量最低要求；同时检查路径与当前剩余续航/航程 |
| `PAYLOAD` | `maxPayloadKg` | kg >=0，必填；检查机型载荷能力与实际任务载荷 |
| `WAYPOINT` | `maxWaypoints,minWaypointInterval` | maxWaypoints 为 >=0 的整数，只计 task.waypoints 中途经点，不计起终点和搜索采样点；间隔为米 >=0 |
| `RTL` | `rtlAltitude,rtlOnLowBattery,lowBatteryPercent` | 三项必填；rtlAltitude 为 **EG 椭球高米**且 >=0，其余为 boolean、0–100。低电开关启用时要求完整上游状态；达到阈值生成 home 返航方案，并检查当前剩余资源 |

规则结构示例（引用 ID 和坐标基准须与实际场景一致）：

```json
[
  {"id":"height-1","rule_type":"HEIGHT_LAYER","content":{"minAltitude":60,"maxAltitude":120,"altDatum":"EG"}},
  {"id":"speed-1","rule_type":"SPEED","content":{"maxSpeed":8,"cruiseSpeed":5}},
  {"id":"width-1","rule_type":"WIDTH","resource_ids":["demo-airway"],"content":{"width":40,"bufferWidth":2}},
  {"id":"separation-1","rule_type":"SEPARATION","content":{"horizontalSeparation":15,"verticalSeparation":12}}
]
```

调用方检查路线 `status`、`validation.valid` 和 `rule_report`。报告记录每条规则的执行状态。未选择或未生效的规则标为不适用。默认 CLIMB/TURN 也应在报告中保留执行记录。缺少必要引用数据时返回错误。

## 4. 返回与错误

### 4.1 场景、坐标与基础端点

| 对象 | 返回字段 |
|------|------|
| 场景注册/查询 | `scene_id`、`scene_version`（SHA-256）、`scene_type`、`network_version`、`status="ready"`、`compilation_ms`、`coordinate_context`、`warnings`、`counts`（airways/obstacles/control_zones/rules/datasets 数量） |
| 坐标转换 | `geometries_wkt`（与输入顺序相同）、`direction`、`alt_reference`、`coordinate_context` |
| health | `status="ok"/"degraded"`、`api_version`、`native_available`、`jobs`；缺原生扩展时 degraded，不能据此承诺可规划 |
| version | `api_version`、`engine`（available/engine/version/error/git_sha）、`default_profile`；`engine.git_sha` 为运行镜像的构建 revision，本地未设置时为 `unknown` |
| capabilities | `api_version/contract_version/route_modes/default_route_mode/rule_types/engine_rules/canonical_alt_reference/altitude_references/geometric_tolerance_m/grid_formats/scene_lifecycle`；`scene_limits` 中各项以及 `max_registered_scenes`、`max_registered_scene_bytes`、`max_scene_radius_m` 为 null，表示未设置固定资源上限 |

`capabilities` 同时返回 search_algorithms、grid_neighbors、航路图/完整网格/并行/多目标/增量支持状态，以及 objective_dimensions、cost_layer_bases 和 replanning_engine_ids。

`capabilities.contract_version="2026-10-30"` 标记本次契约；`engine_rules` 给出两引擎规则分配，`canonical_alt_reference="EG"` 声明内部高度基准。

`coordinate_context` 包含 EPSG:4326、轴序、局部 AEQD CRS、原点、半径、高度数据来源与气压修正版本。几何容差为 0.1 m，用于数值和离散近似处理。安全距离使用场景与机型配置值。

### 4.2 `RouteResult`

| 字段 | 类型 | 含义 |
|------|------|------|
| `task_id` | string | 对应任务 ID |
| `search_report` | SearchReport / null | 显式搜索配置与实际扩展数 |
| `network_route` | NetworkRoute / null | 图规划的版本、节点、边权及实际边序列 |
| `objectives` | RouteObjectives / null | 各目标的原始值、归一化值、单位、权重、总分和数据版本 |
| `optimization` | OptimizationReport / null | 候选搜索状态、迭代及方案数量 |
| `alternatives` | RouteResult[] | 非支配的其他完整方案；主方案参与批量执行，备选方案仅供选择 |
| `traversed_grids` | TraversedGrid[] | 全部途经网格及进入、离开时刻 |
| `grid_context` | object | 完整途经网格的坐标和层级说明 |
| `status` | `planned/failed/timeout` | 逐任务结果；HTTP 200 不代表每个任务成功 |
| `code` / `message` | string/null / string | 失败代码和解释；成功时 code 通常 null |
| `route_mode` | 路径枚举 | 实际任务模式 |
| `engine_id` | `ENG-06/ENG-07` | 本任务实际归属 |
| `alt_reference` | 高度枚举 | 输出 WKT 使用的任务高度基准，失败结果也返回 |
| `coordinate_context` | object | canonical_alt_reference=EG、origin_alt_ellipsoid、投影与高度证据；用于解释局部运动段 |
| `start_wkt/goal_wkt` | WKT | 本次生效的起终点；触发 RTL 时 goal 为 home |
| `departure_time` | timestamp | 请求的最早起飞时刻；实际出发看首个 vertex time |
| `time_origin` | timestamp/null | 本结果数字时间的场景零点；成功路线会填写 |
| `action` | `mission/return_to_home` | 普通任务或当前低电触发后的返航方案 |
| `trajectory_wkt` | WKT/null | 任务高度基准下的 WGS84 `LINESTRING Z`，静止点可为 `POINT Z`；失败无轨迹 |
| `vertex_times_s` | number[] | 每个 WKT 顶点的相对场景零点时间 |
| `motion_segments` | `MotionSegment[]` | 精确运动原语和时间律，局部 AEQD 米制位置 |
| `initial_velocity_xyz_mps` | `[vx,vy,vz]` | 局部东/北/上起始速度，m/s；普通规划为零，重规划保留当前实际速度 |
| `keypoints` | `Keypoint[]` | 业务与运动事件关键点 |
| `metrics` | object | `distance_m/flight_time_s/wait_time_s`；wait 是调度的起飞延迟 |
| `validation` | object | `valid` 与 `violations` |
| `rule_report` | `RuleReport[]` | 每条规则的应用情况、原因与数据版本 |

失败结果的数组通常为空，指标默认 0，不能当可飞行路线使用。`flight_time_s` 为实际轨迹首末时刻差；起飞前地面等待不视为已经起飞的空中占用。

`MotionSegment` 的 `kind` 为 `line/arc/climb/descent/hover/yaw`。`start/end/center` 均是**局部 AEQD 米制三元组**，不是经纬度；`start_time_s/end_time_s` 相对场景零点。`airway_ids` 列出该段相关的航路 ID，未匹配时为空数组。`length_m` 为原语长度；`radius_m/center` 用于圆弧；`start_speed_mps/end_speed_mps/acceleration_mps2` 与 `profile` 描述加速、匀速、减速时间律。`profile` 的规范键为 `peak_speed_mps/deceleration_mps2/accel_time_s/cruise_time_s/decel_time_s/sweep_angle_deg`，sweep 为带符号角度，原地 yaw 同样保留真实转角。

模型还保留 `distance_m/points/times_s/max_speed_mps/deceleration_mps2/peak_speed_mps/accel_time_s/cruise_time_s/decel_time_s/sweep_angle_deg` 可选描述字段；应以服务实际填充的原语和 profile 值重建运动。WKT 折线是原语的有限误差采样视图；折线顶点本身不能证明采样间加速度、圆弧峰值或偏航满足包线，执行平台应保留原语时间律。

`Keypoint` 含 `types:string[]`、`wkt`、`time_s`、可空 `grid_indices:[i,j,k]`、`grid_code` 与 `waypoint_index`。业务 `WAYPOINT` 事件的 waypoint_index 对应 Task.waypoints 的从 0 开始的索引，其他事件通常为 null。同一点可有多个事件；航点、悬停、爬升/下降与转弯的起止由事件类型标记。`AIRWAY_SWITCH` 表示当前航路覆盖结束后的确定性交接：重叠区继续保留原航路，再按稳定 ID 选择接续航路。切换关键点使用解析运动的准确位置和时刻，可以位于两个导出 WKT 顶点之间，不改变运动时间律。`grid_indices` 始终对应 Scene.bounds/resolution 的内部规划网格；`grid_code` 另按上游 GridSpec 的原点和分辨率或显式网格单元匹配。

### 4.3 批量、验证与规则报告

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
| `stage_timings_ms` | 服务阶段耗时对象，单位毫秒：`scene_and_task_parsing/search_motion_and_certificate/reservation_scheduling/final_validation_and_wkt/sdk_response_serialization`；用于诊断，完整 HTTP 耗时以客户端测量为准 |

`validation` 和验证端点返回 `valid:boolean`、`violations:Violation[]`；验证端点另有 `rule_report`。`Violation` 含 `code/message`，可选 `rule_id/time_s` 与 `details`。`RuleReport` 含 `rule_id/rule_type/status/message/details/reasons/dataset_versions`；status 枚举为 `applied/not_applicable/inactive/failed/unsupported`。当前报告的原因及版本证据主要位于 `details.reasons` 和 `details.dataset_versions`，顶层字段为兼容保留；调用方应保存完整报告。

验证移动航线时，必须将规划返回的 `motion_segments` 与 `trajectory_wkt/vertex_times_s` 一起提交。服务重建解析运动，核对它与 WKT/时间视图一致，运行原生运动学证书，再以保守采样覆盖检查场景、规则、端点、业务航点和动态轨迹冲突。仅提交折线和逐点时间会得到 HTTP 200、`valid:false`、`MOTION_MODEL_REQUIRED`；原语不一致或运动学超限报告 `MOTION_MODEL` 等违规码，不能用任意折线插值替代真实时间律。

`POINT Z` 加一个时间表示零持续时间的静止位置，可省略 motion_segments；持续等待使用重复位置的 `LINESTRING Z`、对应严格递增的时间以及 hover 原语，仍检查飞机悬停能力、任务航点标记与 HOVER 规则。缺少原语、规则失败等属于验证结果，不是 HTTP 请求成功即可使用的飞行许可。

### 4.4 异步作业

提交返回 `{"job_id":"...","status":"pending","poll_url":"/api/v2/jobs/..."}`。查询含 `job_id/status/created_at/started_at/finished_at/elapsed_ms/result/error`。状态为 `pending/running/succeeded/failed`；`succeeded` 表示函数已返回，仍须检查 `result.failed_count` 与每条路线。`failed` 时 error 为 `code/message/detail`。作业元数据时间字符串使用服务记录的格式，飞行计算仍以场景带时区时间为准。

作业在进程内执行，服务重启后清空。v2 使用 1 个执行线程，其他作业排队等待。默认不限制队列数量、历史数量或结果存储字节数，也不自动淘汰历史作业。需要长期保存时，下载并保存完整结果。完整规划耗时从提交请求计到取回结果。

服务关闭时停止接纳新作业。排队作业可转为 failed、`JOB_CANCELLED`。正在执行的作业等待完成。部署前暂停提交，并排空作业。

### 4.5 错误与任务失败

错误使用统一结构：

```json
{
  "error": {
    "code": "INVALID_INPUT",
    "message": "Invalid request",
    "detail": {"field": "task"}
  }
}
```

模型校验失败返回 HTTP 400、`INVALID_INPUT`。`detail.errors` 给出字段与原因。场景或作业不存在时返回 HTTP 404、`TASK_NOT_FOUND`。机型不存在时返回 HTTP 404、`AIRCRAFT_NOT_FOUND`。下表的坐标与高度错误返回 HTTP 400。

v2 OpenAPI 为所有端点声明同一 `ErrorEnvelope`，覆盖 `400/404/429/500/503` 和 `4XX`。服务停止接纳作业时，返回 HTTP 429、`JOB_QUEUE_FULL`。内存分配失败返回 HTTP 503、`RESOURCE_EXHAUSTED`。原生内核不可用返回 HTTP 503、`NATIVE_UNAVAILABLE`。未预期异常返回 HTTP 500、`INTERNAL_ERROR`。

| code | 原因 |
|------|------|
| `HEIGHT_REFERENCE_UNAVAILABLE` | 缺地形、水准面差或有效气压高度修正 |
| `AMBIGUOUS_HEIGHT_CORRECTION` | 同位置同时间有多条气压高度修正 |
| `INVALID_ALTITUDE_REFERENCE` | 不支持的高度基准（通常先被模型拒绝为 INVALID_INPUT） |
| `INVALID_COORDINATE` | 无效几何、坐标范围或非有限坐标 |
| `COORDINATE_RANGE_EXCEEDED` | 超出声明的局部半径 |
| `COORDINATE_CONTEXT_MISMATCH` | SDK 使用的 CoordinateFrame 与请求原点配置不同 |

请求合法而路径不可行时，HTTP 200，RouteResult.status 为 failed。常见 code 为 `SEARCH_NOT_FOUND/TIME/CONSTRAINT_VIOLATION/SEPARATION`。调用方设置的 `time_budget_s` 到期时，返回 status=timeout、code=`TIME_BUDGET_EXCEEDED`。缺 C++ 扩展时，health 为 degraded，规划返回 HTTP 503、`NATIVE_UNAVAILABLE`。验证违规和作业错误写入响应体；检查 `validation`、`status` 和 `error`。

## 5. 完整调用示例

### 5.1 完整单任务 JSON

将下列内容保存为 `single-v2.json`，再提交到 `/api/v2/plan/single`。这是带完整航路的严格模式示例，地形使用显式声明的平地，不依赖外部数据源。

```json
{
  "scene": {
    "coordinates": {
      "origin_lon": 104.77,
      "origin_lat": 29.33,
      "origin_alt_ellipsoid": 0,
      "terrain_ellipsoid_m": 0,
      "terrain_source": "example-flat-terrain-20261016"
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
        "altReference": "EG",
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
    "alt_reference": "EG",
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
  "initial_velocity_xyz_mps": [
    0.0,
    0.0,
    0.0
  ],
  "objectives": null,
  "optimization": null,
  "alternatives": [],
  "network_route": null,
  "search_report": null,
  "traversed_grids": [
    {
      "grid_indices": [
        23,
        24,
        8
      ],
      "grid_code": null,
      "entry_time_s": 0.0,
      "exit_time_s": 1.6950340041274239
    },
    {
      "grid_indices": [
        24,
        24,
        8
      ],
      "grid_code": null,
      "entry_time_s": 1.6950340041274239,
      "exit_time_s": 6.824628060567422
    },
    {
      "grid_indices": [
        25,
        24,
        8
      ],
      "grid_code": null,
      "entry_time_s": 6.824628060567422,
      "exit_time_s": 11.824628066087712
    },
    {
      "grid_indices": [
        26,
        24,
        8
      ],
      "grid_code": null,
      "entry_time_s": 11.824628066087712,
      "exit_time_s": 16.824628071608004
    },
    {
      "grid_indices": [
        27,
        24,
        8
      ],
      "grid_code": null,
      "entry_time_s": 16.824628071608004,
      "exit_time_s": 21.82462807714578
    },
    {
      "grid_indices": [
        28,
        24,
        8
      ],
      "grid_code": null,
      "entry_time_s": 21.82462807714578,
      "exit_time_s": 26.824628082666063
    },
    {
      "grid_indices": [
        29,
        24,
        8
      ],
      "grid_code": null,
      "entry_time_s": 26.824628082666063,
      "exit_time_s": 31.824628088203838
    },
    {
      "grid_indices": [
        30,
        24,
        8
      ],
      "grid_code": null,
      "entry_time_s": 31.824628088203838,
      "exit_time_s": 36.82462809372413
    },
    {
      "grid_indices": [
        31,
        24,
        8
      ],
      "grid_code": null,
      "entry_time_s": 36.82462809372413,
      "exit_time_s": 41.8246280992619
    },
    {
      "grid_indices": [
        32,
        24,
        8
      ],
      "grid_code": null,
      "entry_time_s": 41.8246280992619,
      "exit_time_s": 46.82462810478219
    },
    {
      "grid_indices": [
        33,
        24,
        8
      ],
      "grid_code": null,
      "entry_time_s": 46.82462810478219,
      "exit_time_s": 51.824628110302484
    },
    {
      "grid_indices": [
        34,
        24,
        8
      ],
      "grid_code": null,
      "entry_time_s": 51.824628110302484,
      "exit_time_s": 56.824628115840255
    },
    {
      "grid_indices": [
        35,
        24,
        8
      ],
      "grid_code": null,
      "entry_time_s": 56.824628115840255,
      "exit_time_s": 61.82462812136055
    },
    {
      "grid_indices": [
        36,
        24,
        8
      ],
      "grid_code": null,
      "entry_time_s": 61.82462812136055,
      "exit_time_s": 66.82462812689832
    },
    {
      "grid_indices": [
        37,
        24,
        8
      ],
      "grid_code": null,
      "entry_time_s": 66.82462812689832,
      "exit_time_s": 71.82462813241861
    },
    {
      "grid_indices": [
        38,
        24,
        8
      ],
      "grid_code": null,
      "entry_time_s": 71.82462813241861,
      "exit_time_s": 76.8246281379389
    },
    {
      "grid_indices": [
        39,
        24,
        8
      ],
      "grid_code": null,
      "entry_time_s": 76.8246281379389,
      "exit_time_s": 81.82462814347667
    },
    {
      "grid_indices": [
        40,
        24,
        8
      ],
      "grid_code": null,
      "entry_time_s": 81.82462814347667,
      "exit_time_s": 86.82462814899696
    },
    {
      "grid_indices": [
        41,
        24,
        8
      ],
      "grid_code": null,
      "entry_time_s": 86.82462814899696,
      "exit_time_s": 91.82462815453474
    },
    {
      "grid_indices": [
        42,
        24,
        8
      ],
      "grid_code": null,
      "entry_time_s": 91.82462815453474,
      "exit_time_s": 96.82462816005503
    },
    {
      "grid_indices": [
        43,
        24,
        8
      ],
      "grid_code": null,
      "entry_time_s": 96.82462816005503,
      "exit_time_s": 101.8246281655928
    },
    {
      "grid_indices": [
        44,
        24,
        8
      ],
      "grid_code": null,
      "entry_time_s": 101.8246281655928,
      "exit_time_s": 106.82462817111309
    },
    {
      "grid_indices": [
        45,
        24,
        8
      ],
      "grid_code": null,
      "entry_time_s": 106.82462817111309,
      "exit_time_s": 111.82462817663338
    },
    {
      "grid_indices": [
        46,
        24,
        8
      ],
      "grid_code": null,
      "entry_time_s": 111.82462817663338,
      "exit_time_s": 116.82462818217115
    },
    {
      "grid_indices": [
        47,
        24,
        8
      ],
      "grid_code": null,
      "entry_time_s": 116.82462818217115,
      "exit_time_s": 121.82462818769145
    },
    {
      "grid_indices": [
        48,
        24,
        8
      ],
      "grid_code": null,
      "entry_time_s": 121.82462818769145,
      "exit_time_s": 126.8246281932292
    },
    {
      "grid_indices": [
        49,
        24,
        8
      ],
      "grid_code": null,
      "entry_time_s": 126.8246281932292,
      "exit_time_s": 131.8246281987495
    },
    {
      "grid_indices": [
        50,
        24,
        8
      ],
      "grid_code": null,
      "entry_time_s": 131.8246281987495,
      "exit_time_s": 136.82462820428728
    },
    {
      "grid_indices": [
        51,
        24,
        8
      ],
      "grid_code": null,
      "entry_time_s": 136.82462820428728,
      "exit_time_s": 141.82462820980757
    },
    {
      "grid_indices": [
        52,
        24,
        8
      ],
      "grid_code": null,
      "entry_time_s": 141.82462820980757,
      "exit_time_s": 146.82462821532786
    },
    {
      "grid_indices": [
        53,
        24,
        8
      ],
      "grid_code": null,
      "entry_time_s": 146.82462821532786,
      "exit_time_s": 151.82462822086563
    },
    {
      "grid_indices": [
        54,
        24,
        8
      ],
      "grid_code": null,
      "entry_time_s": 151.82462822086563,
      "exit_time_s": 156.82462822638593
    },
    {
      "grid_indices": [
        55,
        24,
        8
      ],
      "grid_code": null,
      "entry_time_s": 156.82462822638593,
      "exit_time_s": 161.82462823192367
    },
    {
      "grid_indices": [
        56,
        24,
        8
      ],
      "grid_code": null,
      "entry_time_s": 161.82462823192367,
      "exit_time_s": 166.82462823744396
    },
    {
      "grid_indices": [
        57,
        24,
        8
      ],
      "grid_code": null,
      "entry_time_s": 166.82462823744396,
      "exit_time_s": 171.82462824296425
    },
    {
      "grid_indices": [
        58,
        24,
        8
      ],
      "grid_code": null,
      "entry_time_s": 171.82462824296425,
      "exit_time_s": 177.32834746683343
    }
  ],
  "grid_context": {
    "crs": "LOCAL_AEQD",
    "origin": [
      -500.0,
      -500.0,
      0.0
    ],
    "resolution": [
      25.0,
      25.0,
      10.0
    ],
    "shape": [
      100,
      60,
      30
    ],
    "level": null,
    "source": "scene.bounds",
    "keypoint_indices_source": "scene.bounds/resolution",
    "boundary_convention": "half-open; simultaneous face/corner crossings have no dwell"
  },
  "engine_id": "ENG-06",
  "task_id": "demo-v2-001",
  "alt_reference": "EG",
  "coordinate_context": {
    "input_horizontal_crs": "EPSG:4326",
    "axis_order": "longitude,latitude,altitude",
    "local_horizontal_crs": "+proj=aeqd +lat_0=29.33 +lon_0=104.77 +datum=WGS84 +units=m +no_defs +type=crs",
    "local_axis_order": "east_m,north_m,ellipsoid_minus_origin_m",
    "canonical_alt_reference": "EG",
    "origin_lon": 104.77,
    "origin_lat": 29.33,
    "origin_alt_ellipsoid": 0.0,
    "max_radius_m": 50000.0,
    "terrain_source": "example-flat-terrain-20261016",
    "geoid_source": null,
    "height_correction_versions": []
  },
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
  "vertex_times_s": [
    0.0,
    2.5,
    174.82834746683343,
    177.32834746683343
  ],
  "motion_segments": [
    {
      "airway_ids": [
        "demo-airway"
      ],
      "kind": "line",
      "start_time_s": 0.0,
      "end_time_s": 177.32834746683343,
      "start": [
        97.12685972800742,
        110.8416525552106,
        80.0
      ],
      "end": [
        971.2685960957951,
        110.88275614663762,
        80.0
      ],
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
    {
      "types": [
        "START"
      ],
      "wkt": "POINT Z (104.771 29.331 80)",
      "time_s": 0.0,
      "grid_indices": [
        23,
        24,
        8
      ],
      "grid_code": null,
      "waypoint_index": null
    },
    {
      "types": [
        "GOAL"
      ],
      "wkt": "POINT Z (104.78 29.331 80)",
      "time_s": 177.32834746683343,
      "grid_indices": [
        58,
        24,
        8
      ],
      "grid_code": null,
      "waypoint_index": null
    }
  ],
  "metrics": {
    "distance_m": 874.1417373341671,
    "flight_time_s": 177.32834746683343,
    "wait_time_s": 0.0
  },
  "validation": {
    "valid": true,
    "violations": []
  },
  "rule_report": [
    {
      "rule_id": "speed-1",
      "rule_type": "SPEED",
      "status": "applied",
      "message": "",
      "details": {
        "reasons": [],
        "dataset_versions": {}
      },
      "reasons": [],
      "dataset_versions": {}
    },
    {
      "rule_id": "separation-1",
      "rule_type": "SEPARATION",
      "status": "applied",
      "message": "",
      "details": {
        "reasons": [],
        "dataset_versions": {}
      },
      "reasons": [],
      "dataset_versions": {}
    },
    {
      "rule_id": "default:CLIMB",
      "rule_type": "CLIMB",
      "status": "applied",
      "message": "",
      "details": {
        "reasons": [],
        "dataset_versions": {}
      },
      "reasons": [],
      "dataset_versions": {}
    },
    {
      "rule_id": "default:TURN",
      "rule_type": "TURN",
      "status": "applied",
      "message": "",
      "details": {
        "reasons": [],
        "dataset_versions": {}
      },
      "reasons": [],
      "dataset_versions": {}
    }
  ]
}
```

### 5.2 Python SDK：注册、单任务、批量、验证

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

### 5.3 requests：场景复用、完整结果和作业

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

### 5.4 坐标转换

```json
{
  "geometries_wkt": ["POINT Z (104.771 29.331 80)"],
  "direction": "wgs84_to_local",
  "alt_reference": "EG",
  "coordinates": {"origin_lon":104.77,"origin_lat":29.33,"origin_alt_ellipsoid":20},
  "at": "2026-10-16T09:00:00+08:00"
}
```

该例的本地 z 为 60 m；水平坐标由 AEQD 计算。反向转换把返回的本地 WKT 放回 `geometries_wkt`，将 direction 改为 `local_to_wgs84`，使用完全相同的 coordinates/alt_reference/at。批量支持多条 2D 足迹和 3D 几何，顺序不变。

### 5.5 显式接驳与空间场片段

下面是可并入 Scene 的局部片段；坐标应按实际场景调整，接驳体必须真实覆盖起终点和授权航路之间的扫掠体。

```json
{
  "connectors": [
    {"id":"launch-connector","geometry_wkt":"POLYGON ((104.770 29.330,104.772 29.330,104.772 29.332,104.770 29.332,104.770 29.330))",
     "min_altitude":50,"max_altitude":150,"alt_reference":"EG","airway_ids":["demo-airway"]}
  ],
  "datasets": [
    {"id":"weather-20261016","kind":"weather","version":"2026-10-16-09",
     "valid_from":"2026-10-16T09:00:00+08:00","valid_to":"2026-10-16T10:00:00+08:00",
     "cells":[{"bounds":{"x_min":-500,"x_max":2000,"y_min":-500,"y_max":1000,"z_min":0,"z_max":300},
               "values":{"wind_speed_mps":3,"precipitation":false,"icing":false}}]}
  ],
  "rules": [
    {"id":"weather-rule","rule_type":"WEATHER","resource_ids":["weather-20261016"],
     "content":{"maxWindSpeed":8,"noPrecipitation":true,"noIcing":true}}
  ]
}
```

使用接驳时在任务中设置 `start_connector_id="launch-connector"`；无需接驳的任务省略该字段。该片段的 rules 是一个独立示例；与已有规则合并时追加数组，避免覆盖原有约束。

## 6. 数据归档与性能记录

对接方使用 [`../test_api/`](README.md) 执行公网接口测试。逐项字段说明与修改步骤见 [`接口字段与修改说明`](API_FIELDS.md)。复制整个目录后运行 `python run_tests.py`。默认地址为 `https://api.kaguratart.com`。测试覆盖全部 16 个接口和三套各 1000 个任务，保存完整请求、响应与报告。

测试目录与分类运行命令见 [`../tests/README.md`](https://github.com/KaguraTart/low-altitude-path-planning-20260906/blob/main/tests/README.md)。完整测试请求与响应见 [`../tests/data/README.md`](https://github.com/KaguraTart/low-altitude-path-planning-20260906/blob/main/tests/data/README.md)。归档只包含 v2，保存接口示例、冻结场景和三套各 1000 条航线结果。每组记录输入、输出、运行环境和 SHA-256。重跑时先注册场景，再替换请求中的 `scene_id`。

历史性能验收使用 `input/v2_benchmarks` 的冻结输入。strict/mixed/free 三套各有 1000 个任务，各运行 10 轮，资源为 8 CPU / 8 GiB。每轮全部任务成功，完整 JSON 在 10 秒内返回。场景注册单独计时。正式服务使用 2 CPU / 2 GiB，按实际场景和负载测量耗时。历史证据见 [`../internal/reports/v2-20261016/PERFORMANCE.md`](https://github.com/KaguraTart/low-altitude-path-planning-20260906/blob/main/internal/reports/v2-20261016/PERFORMANCE.md)。

批量结果 `stage_timings_ms` 记录各阶段毫秒耗时。完整 HTTP 耗时另外包含传输和 JSON 编解码。`GET /api/v2/version` 的 `engine.git_sha` 标识运行镜像的构建提交。
