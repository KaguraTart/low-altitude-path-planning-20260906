# 低空无人机路径规划 API

服务提供 `/api/v2` 接口。请求和响应使用 JSON。外部几何使用 WGS84 经纬度 WKT。接口支持航路、管制区、机型性能和 20 类规则。

完整字段以运行服务的 `/openapi.json` 为准。打开 `/docs` 可查看 Swagger 并直接调用接口。

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
| POST | `/api/v2/plan/batch` | 场景选择 + `tasks` → `BatchResult` |
| POST | `/api/v2/validate` | 场景、任务与带时间航线 → 违规项和规则报告 |
| POST | `/api/v2/jobs` | 与批量请求相同 → HTTP 202、`job_id`、`poll_url` |
| GET | `/api/v2/jobs/{job_id}` | 作业状态及完整批量结果 |
| GET | `/api/v2/examples/single` | 可直接提交的完整单任务请求 |
| GET | `/api/v2/examples/batch` | 可直接提交的完整双任务请求 |

所有 v2 对象拒绝未知字段，数值必须是有限 JSON number；字符串数字、布尔值充当数值、`NaN` 和 `Infinity` 均不接受。时间字段使用带时区的 ISO 8601，如 `2026-10-16T09:00:00+08:00`。WKT 只放几何，属性、时间和规则放在 JSON 字段；不支持把时间藏在 WKT M 坐标中。

### 1.2 坐标、高度与时间

外部几何使用 WGS84，轴序固定为 **经度、纬度、高度**。任务起终点和业务航点必须是 `POINT Z`，航路中心线必须是 `LINESTRING Z`。体积边界为 `POLYGON` 或 `MULTIPOLYGON` 的平面足迹，垂直范围由单独字段确定；足迹 WKT 中的 Z 不改变体积上下限。空几何、非有限坐标、自相交等无效拓扑和 `GEOMETRYCOLLECTION` 会被拒绝。

每个场景以 `coordinates.origin_lon/origin_lat` 建立局部 AEQD 投影。`x` 向东，`y` 向北，单位为米。`z = MSL 高度 - origin_alt_msl`，单位为米。`bounds`、`resolution`、规则距离和网格使用此坐标系。`max_radius_m` 指定场景覆盖半径，默认 50000 m。可按实际范围设置更大的正数。

| `alt_reference` / `altReference` | 输入 Z 的含义 | 转成 MSL 的依据 |
|------|------|------|
| `MSL` | 海拔米 | 直接使用 |
| `AGL` | 离地米 | `MSL = Z + 地形海拔`；需要场景 terrain 数据或声明的常量 `terrain_msl_m` |
| `ELLIPSOID` | WGS84 椭球高米 | `MSL = Z - geoid_separation_m`；需提供大地水准面差 |
| `QNE` / `QNH` | 相应气压高度米 | `MSL = Z + offset_m`；需有效位置、时刻内唯一的 `height_corrections` 记录 |

服务不会猜高度基准或用零代替缺少的证据。独立坐标转换端点只接受 `CoordinateSpec` 的常量地形；场景规划会将 terrain 空间场注册为转换回调，按位置和有效时间读取。数据的来源、版本和有效期由上游负责。QNE/QNH 转换必须有带时区的 `at`；缺少修正报 `HEIGHT_REFERENCE_UNAVAILABLE`，同时匹配多条修正报 `AMBIGUOUS_HEIGHT_CORRECTION`。

`Scene.time_origin` 是所有数值时间的零点。`vertex_times_s`、动态障碍时间、关键点 `time_s`、运动段起止时间、冲突时间均为**相对这个零点的秒数**，不是相对各任务起飞的秒数。`departure_time`、有效期和时间窗仍使用绝对 ISO 时间。例如场景零点为 09:00，任务在 09:05 起飞，则首个数值时间至少是 300；为避让延迟到 09:06 时至少是 360。时间戳与几何顶点逐一对应，必须严格递增；悬停用两个相同位置、不同时间表示。

### 1.3 三种路径模式

| `route_mode` | 行为 |
|------|------|
| `strict`（默认） | 飞行器及安全裕度形成的扫掠体必须完全位于当时有效的航路体或已明确授权的起终点接驳体内。必须有航路；找不到合法路线就失败，不自动改为自由飞行 |
| `prefer` | 允许航路外飞行；候选图代价中，航路外基础距离代价为航路内的 2 倍；其余安全、规则和性能约束照常执行 |
| `free` | 不要求航路包含；其余安全、规则和性能约束照常执行 |

中心线 Z 是参考高度。`minAltitude/maxAltitude` 和 `altReference` 确定可用高度带。`widthMeters` 是航路全宽，编译时除以 2。规则 `WIDTH.content.width` 是半宽。可用宽度再扣除 `bufferWidth` 和飞机安全裕度。

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
| 验证 `task`、`trajectory_wkt`、`vertex_times_s` | 必填；时间数组非空 | 任务、WGS84 `POINT Z/LINESTRING Z` 与相对场景零点的时间；顶点数与时间项数相同 |
| 验证 `motion_segments` | `MotionSegment[] \| null`，模型允许省略 | 移动航线需提交此字段。复制规划结果的同名字段；缺少时返回 `valid:false`、`MOTION_MODEL_REQUIRED` |

### 2.2 `Scene`

| 字段 | 类型 | 必填 / 默认 | 含义 |
|------|------|------|------|
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
| `origin_alt_msl` | number，0 | 局部 z=0 的 MSL 海拔 |
| `max_radius_m` | number，50000 | 场景覆盖半径，单位米；可设置任意正有限值 |
| `terrain_msl_m` / `terrain_source` | number / string，null | 声明的平地海拔与来源；非平地区域应提供 terrain 空间场 |
| `geoid_separation_m` / `geoid_source` | number / string，null | 椭球高减 MSL 的差值与来源 |
| `height_corrections` | `HeightCorrection[]`，`[]` | QNE/QNH 的显式修正 |

`HeightCorrection` 必填 `reference`（`QNE/QNH`）、`offset_m`、非空 `source/version`、`valid_from/valid_to`；可选 `geometry_wkt` 限定 WGS84 平面覆盖。有效期必须递增，修正采用 `valid_from <= at < valid_to`。

### 2.4 航路、障碍、接驳和管制区

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

### 2.5 空间场与地形

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

### 2.6 外部占用网格 `GridSpec`

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

### 2.7 `Task`、航点和当前状态

| `Task` 字段 | 类型 / 默认 | 含义 |
|------|------|------|
| `id` | 非空 string，必填 | 任务 ID |
| `start_wkt` / `goal_wkt` | `POINT Z`，必填 | WGS84 起终点 |
| `departure_time` | timestamp，必填 | 最早起飞时间，必须在场景时域内 |
| `alt_reference` | 高度枚举，必填 | 本任务起终点、航点与输出 WKT 的高度基准 |
| `route_mode` | `strict/prefer/free`，`strict` | 路径模式 |
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

## 3. 全部规则

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
| `TURN` | `maxTurnAngle,minTurnRadius` | 角度 `0<=值<=180`，半径米 >=0；约束完整转弯累计角及几何半径 |
| `SEPARATION` | `verticalSeparation,horizontalSeparation` | 米，>=0；与飞机尺寸和安全裕度取保守值，连续时空检查双方同时过近 |
| `NO_FLY` | `noFlyMode,heightCeiling,includeBuffer` | noFlyMode 必填=`PERMANENT/TEMPORARY`；临时规则须显式起止有效期；需要匹配管制几何；heightCeiling 为 MSL 米；includeBuffer 是保留开关，当前管制区附加缓冲为 0，飞机净空单独参与检查 |
| `BUILDING` | `avoidMode,safeDistance,bufferRadius` | avoidMode 必填=`BUFFER/CORRIDOR`；距离米 >=0；需障碍几何，CORRIDOR 还需明确 corridor 数据集，不能把建筑整体当穿行区 |
| `ZONE` | `zoneLevels,coverageThreshold` | 至少给一项；zoneLevels 为非空字符串数组；coverageThreshold 为 0–1，记录覆盖阈值；碰撞结果由真实管制体决定 |
| `WEATHER` | `maxWindSpeed,noPrecipitation,noIcing` | 至少给一项；风 m/s，后两项 boolean；需 versioned weather 格场，相应风/降水/结冰数值必须存在，全程有效且覆盖 |
| `SIGNAL` | `minSignalStrength,bands,band_policy` | 强度 dBm 必填；bands 为非空字符串数组；band_policy=`any`（默认）或 `all`；需 signal 格场，并匹配飞机支持频段 |
| `PRIORITY` | `priorityScene,costBonus` | 两者必填；场景=`LOW_RISK/FAST/QUIET`，bonus 为 0–1；LOW_RISK 需 risk 格场，QUIET 需 noise 格场，覆盖缺口不放行；影响候选路线代价 |
| `HOVER` | `maxHoverTime,defaultHoverTime` | 秒 >=0，default <= max；只允许明确标记的业务航点悬停，实际持续时间不能超限 |
| `ENDURANCE` | `maxRange,minEndurance` | 米 / 秒，均 >0；minEndurance 是机型容量最低要求；同时检查路径与当前剩余续航/航程 |
| `PAYLOAD` | `maxPayloadKg` | kg >=0，必填；检查机型载荷能力与实际任务载荷 |
| `WAYPOINT` | `maxWaypoints,minWaypointInterval` | maxWaypoints 为 >=2 的整数，计起点+业务中间点+终点，不计搜索采样点；间隔为米 >=0 |
| `RTL` | `rtlAltitude,rtlOnLowBattery,lowBatteryPercent` | 三项必填；rtlAltitude 为 **MSL 海拔米**且 >=0，其余为 boolean、0–100。低电开关启用时要求完整上游状态；达到阈值生成 home 返航方案，并检查当前剩余资源 |

示例规则（加入上面的严格航路场景即可运行）：

```json
[
  {"id":"height-1","rule_type":"HEIGHT_LAYER","content":{"minAltitude":60,"maxAltitude":120,"altDatum":"MSL"}},
  {"id":"speed-1","rule_type":"SPEED","content":{"maxSpeed":8,"cruiseSpeed":5}},
  {"id":"width-1","rule_type":"WIDTH","resource_ids":["demo-airway"],"content":{"width":40,"bufferWidth":2}},
  {"id":"separation-1","rule_type":"SEPARATION","content":{"horizontalSeparation":15,"verticalSeparation":12}}
]
```

调用方检查路线 `status`、`validation.valid` 和 `rule_report`。报告记录每条规则的执行状态。未选择或未生效的规则标为不适用。缺少必要引用数据时返回错误。

## 4. 返回与错误

### 4.1 场景、坐标与基础端点

| 对象 | 返回字段 |
|------|------|
| 场景注册/查询 | `scene_id`、`scene_version`（SHA-256）、`status="ready"`、`compilation_ms`、`coordinate_context`、`warnings`、`counts`（airways/obstacles/control_zones/rules/datasets 数量） |
| 坐标转换 | `geometries_wkt`（与输入顺序相同）、`direction`、`alt_reference`、`coordinate_context` |
| health | `status="ok"/"degraded"`、`api_version`、`native_available`、`jobs`；缺原生扩展时 degraded，不能据此承诺可规划 |
| version | `api_version`、`engine`（available/engine/version/error/git_sha）、`default_profile`；`engine.git_sha` 为运行镜像的构建 revision，本地未设置时为 `unknown` |
| capabilities | `api_version/route_modes/default_route_mode/rule_types/altitude_references/geometric_tolerance_m/grid_formats/scene_lifecycle`；`scene_limits`、`max_registered_scenes`、`max_registered_scene_bytes`、`max_scene_radius_m` 为 null，表示未设置固定资源上限 |

`coordinate_context` 包含 EPSG:4326、轴序、局部 AEQD CRS、原点、半径、高度数据来源与气压修正版本。几何容差为 0.1 m，用于数值和离散近似处理。安全距离使用场景与机型配置值。

### 4.2 `RouteResult`

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
      "origin_alt_msl": 0,
      "terrain_msl_m": 0,
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
  "alt_reference": "MSL",
  "coordinates": {"origin_lon":104.77,"origin_lat":29.33,"origin_alt_msl":20},
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
     "min_altitude":50,"max_altitude":150,"alt_reference":"MSL","airway_ids":["demo-airway"]}
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

对接方使用 [`../test_api/`](../test_api/README.md) 执行公网接口测试。逐项字段说明与修改步骤见 [`接口字段与修改说明`](../test_api/API_FIELDS.md)。复制整个目录后运行 `python run_tests.py`。默认地址为 `https://api.kaguratart.com`。测试覆盖全部 14 个接口和三套各 1000 个任务，保存完整请求、响应与报告。

测试目录与分类运行命令见 [`../tests/README.md`](../tests/README.md)。完整测试请求与响应见 [`../tests/data/README.md`](../tests/data/README.md)。归档只包含 v2，保存接口示例、冻结场景和三套各 1000 条航线结果。每组记录输入、输出、运行环境和 SHA-256。重跑时先注册场景，再替换请求中的 `scene_id`。

历史性能验收使用 `input/v2_benchmarks` 的冻结输入。strict/mixed/free 三套各有 1000 个任务，各运行 10 轮，资源为 8 CPU / 8 GiB。每轮全部任务成功，完整 JSON 在 10 秒内返回。场景注册单独计时。正式服务使用 2 CPU / 2 GiB，按实际场景和负载测量耗时。历史证据见 [`../internal/reports/v2-20261016/PERFORMANCE.md`](../internal/reports/v2-20261016/PERFORMANCE.md)。

批量结果 `stage_timings_ms` 记录各阶段毫秒耗时。完整 HTTP 耗时另外包含传输和 JSON 编解码。`GET /api/v2/version` 的 `engine.git_sha` 标识运行镜像的构建提交。
