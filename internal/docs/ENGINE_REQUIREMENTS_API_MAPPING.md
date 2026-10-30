# 内部引擎分配与参数追踪

文档日期：2026-10-30。

本文用于内部引擎分配、规则字段和 41 项原始编号追踪。对外交付按 [901–916 业务功能对照](../../docs/REQUIREMENTS_API_MAPPING.md) 核对。

基线：2026-10-28《现有规则到算法引擎的分配与约束 JSON 样例》与《路径规划与导航服务系统算法引擎清单.xlsx》。本表保留清单中 ENG-06、ENG-07 的全部 **41 个原始编号**。编号不重新排序或替换。原表行号指需求映射工作表的行号。

平台负责规则、条件、绑定和设备类型准入的增删改查、对象存在性、引用检查、审计及持久化。引擎消费平台下发的有效配置，执行路径计算和约束校验。新增管理功能不计为算法接口交付。服务提供 **16 个 v2 接口**。

状态口径：**已实现**为当前可调用能力；**部分实现**为列明的子能力；**平台负责**为平台业务职责，引擎仅消费配置；**暂无**为未实现能力。测试入口对应实际计算行为。当前容器与公网的完整请求、响应和结果见 [测试记录索引](../../test_api/reports/README.md)。历史报告保留采集日期和版本。

## 引擎分配

| 引擎 | 计算职责与规则归属 | 本服务交付边界 |
|---|---|---|
| ENG-01 北斗网格剖分与编码 | 无直接业务规则；为 AREA.gridCodes 提供编码 | 外部。接受 GridSpec 和已实现编码；北斗编码/剖分接口暂无 |
| ENG-02 三维空域几何建模 | 无直接业务规则；提供高度与空间基础 | 部分。WKT、体积和 EG/AGL/MSL 换算已实现；独立建模管理暂无 |
| ENG-03 空间分析量算 | 无直接业务规则 | 部分。规划内部几何计算已实现；通用空间量算 API 暂无 |
| ENG-04 网格查询统计 | 无直接业务规则；支撑行政区与网格范围 | 外部。消费上游网格；网格查询统计 API 暂无 |
| ENG-05 起降场选址 | BUILDING、NO_FLY、ZONE | 外部。接收已实现起降点及接驳区；智能选址 API 暂无 |
| ENG-06 航路航线网络规划 | HEIGHT_LAYER、WIDTH、DIRECTION、PRIORITY、SPEED、TIME、AREA、CLIMB、TURN、WAYPOINT | 本次主范围。strict/prefer、航路准入、约束过滤与规划；分工见 41 项表 |
| ENG-07 低空三维路径规划 | HEIGHT_LAYER、BUILDING、NO_FLY、ZONE、WEATHER、SIGNAL、CLIMB、TURN、HOVER、ENDURANCE、PAYLOAD、RTL、PRIORITY、WAYPOINT | 本次主范围。free 三维搜索和校验；支持多目标、增量重规划及完整网格轨迹；地面/水上场景由对应引擎负责 |
| ENG-08 地面交通路径规划 | DIRECTION、SPEED、AREA、TIME、HEIGHT_LAYER、PAYLOAD | 外部。车辆/道路场景及独立接口暂无 |
| ENG-09 水上交通路径规划 | DIRECTION、SPEED、AREA、TIME、WEATHER | 外部。船舶/航道场景及独立接口暂无 |
| ENG-10 安全验证与评估 | NO_FLY、ZONE、HEIGHT_LAYER、SEPARATION、ENDURANCE、PAYLOAD | 部分。/validate 与 metrics 已实现；综合评分、评级和方案排名暂无 |
| ENG-11 多机冲突消解 | SEPARATION、PRIORITY | 部分。batch 内时序协调和 conflicts 已实现；跨请求常驻调度暂无 |
| ENG-12 碰撞预警 | SEPARATION、PRIORITY | 部分。规划与验证检查已实现动态轨迹；运行时预警推送暂无 |
| ENG-13 态势异常检测 | SPEED、HEIGHT_LAYER、TIME、SEPARATION | 外部。持续态势监测、异常事件服务暂无 |
| ENG-14 航路偏离检测 | WIDTH、DIRECTION、ZONE | 部分。提交轨迹可验证航路约束；实时偏离检测服务暂无 |
| ENG-15 路径异常诊断 | WEATHER、SIGNAL | 部分。规则报告给出本次违规原因；持续诊断和告警服务暂无 |

规则归属表示业务计算职责。同一规则可以被多个引擎消费。ENG-06/07 共用模型与安全检查，不因主归属不同而跳过输入中选定的适用安全规则。

## 调用与证据索引

所有路径均相对服务根地址，例如 `https://api.kaguratart.com`。运行服务的 [Swagger](https://api.kaguratart.com/docs#/) 和 [OpenAPI](https://api.kaguratart.com/openapi.json) 用于核对已部署版本；本表描述本次源码契约。

| 简写 | 真实接口 | 输入 | 输出与使用方式 |
|---|---|---|---|
| P | POST /api/v2/plan/single；POST /api/v2/plan/batch | scene 或 scene_id 二选一；task 或 tasks；可选 time_budget_s | RouteResult 或 BatchResult；先检查 status、validation.valid，再读取轨迹与指标 |
| S | POST /api/v2/scenes；GET /api/v2/scenes/{scene_id} | 完整 Scene；或登记后取得的 scene_id | scene_id、scene_version、编译摘要；GET 不返回原始 Scene |
| V | POST /api/v2/validate | 原 scene/task、trajectory_wkt、vertex_times_s、完整 motion_segments | valid、violations、rule_report |
| J | POST /api/v2/jobs；GET /api/v2/jobs/{job_id} | 批量规划请求；或 job_id | 当前进程作业状态与完整结果；重启清空 |
| C | POST /api/v2/coordinates/convert | coordinates、direction、alt_reference、geometries_wkt、必要时 at | 几何数组、alt_reference、coordinate_context |
| R | POST /api/v2/plan/replan | 前次 scene/task/route、当前状态、remaining_task、最新场景、事件和触发参数 | 复用或重算的完整路线、实际初始速度、复用/搜索指标及忽略原因 |

下表 T 编号引用仓库测试；独立测试包不包含源码测试时，应在完整仓库执行。平台管理行为必须由平台另行验收。

| 证据 | 仓库入口 | 证明范围 |
|---|---|---|
| T1 | tests/planning/test_engine_contracts.py | ENG-06/07 选择、LOW_ALTITUDE 场景、RuleBO/scope/默认约束/准入等本次契约回归 |
| T2 | tests/planning/test_rules.py | 20 类规则边界、全段检查、WIDTH/方向/航点语义、优先代价与硬约束；test_width_buffer_checks_outer_protection_without_reducing_flight_width 检查外保护带 |
| T3 | tests/planning/test_time_windows.py；test_temporal_safety.py | 日历、跨午夜、有效期与完整路线时序 |
| T4 | tests/planning/test_coordinates.py；test_ellipsoid_altitudes.py | 非零椭球高原点、h=H+N、AGL 地形、输出基准与高度证据缺失 |
| T5 | tests/planning/test_engine.py；test_airway_events.py | strict/free 搜索、不可达、完整运动段、航路切换与独立验证 |
| T6 | tests/planning/test_dynamic_safety.py；test_motion_budgets.py | 动态轨迹冲突与时域/运动预算 |
| T7 | tests/api/test_public_api.py；test_jobs.py；test_capabilities.py | HTTP 模型、真实端点、作业与能力声明 |
| T8 | input/engine_contract_20261029/；test_api/data/ | 固定基准及当前完整联调输入，实际报告见 test_api/reports/README.md |
| T9 | tests/planning/test_search_grid_contract.py；test_network.py；tests/api/test_network_api.py | 搜索选择、完整网格、显式航路图、加权备选及独立证明 |
| T10 | tests/planning/test_execution.py；tests/api/test_execution_api.py | 实际线程并行、跨组避让、串行一致性及异步传播 |
| T11 | tests/planning/test_objectives.py；tests/api/test_objectives_api.py | 目标改变路线、能耗/场积分、非支配方案、数值与预算 |
| T12 | tests/planning/test_replanning.py | 零搜索复用、障碍事件重算、速度连续、返航、目标联动及独立验证 |

## 41 项需求逐项交付

P/S/V/J/C/R 的方法与路径见上表。管理项的“P/S”表示消费配置的入口，不表示提供增删改查接口。

| 原编号（行） | 需求 / 引擎 | 分工与交付状态 | API | 引擎输入及约束 | 输出 / 平台工作 / 缺口 | 验证证据 |
|---|---|---|---|---|---|---|
| 687（13） | 航路航线优先条件信息 / ENG-06 | 平台负责；约束消费已实现 | P/S/V | PRIORITY：priorityScene=PLANNING/LOW_RISK/QUIET/FAST、costBonus∈[0,1]；scope/资源范围、effective_start/end | rule_report、validation；规则仅调整非负候选代价，不能覆盖硬约束；ENG-06 选择图边权并执行 PRIORITY 空间代价，ENG-07 的目标归一化与自定义图层见 912；平台保存结构化条件及版本 | T1/T2/T3；管理功能由平台测试 |
| 688（14） | 航路航线优先条件添加 / ENG-06 | 平台负责；约束消费已实现 | P/S/V | PRIORITY：priorityScene=PLANNING/LOW_RISK/QUIET/FAST、costBonus∈[0,1]；scope/资源范围、effective_start/end | rule_report、validation；规则仅调整非负候选代价，不能覆盖硬约束；ENG-06 选择图边权并执行 PRIORITY 空间代价，ENG-07 的目标归一化与自定义图层见 912；平台检查必填、重复及关联对象，再下发新规则 | T1/T2/T3；管理功能由平台测试 |
| 689（15） | 航路航线优先条件修改 / ENG-06 | 平台负责；约束消费已实现 | P/S/V | PRIORITY：priorityScene=PLANNING/LOW_RISK/QUIET/FAST、costBonus∈[0,1]；scope/资源范围、effective_start/end | rule_report、validation；规则仅调整非负候选代价，不能覆盖硬约束；ENG-06 选择图边权并执行 PRIORITY 空间代价，ENG-07 的目标归一化与自定义图层见 912；平台按既有 ID 更新，重新校验后下发全量场景 | T1/T2/T3；管理功能由平台测试 |
| 690（16） | 航路航线优先条件删除 / ENG-06 | 平台负责；约束消费已实现 | P/S/V | PRIORITY：priorityScene=PLANNING/LOW_RISK/QUIET/FAST、costBonus∈[0,1]；scope/资源范围、effective_start/end | rule_report、validation；规则仅调整非负候选代价，不能覆盖硬约束；ENG-06 选择图边权并执行 PRIORITY 空间代价，ENG-07 的目标归一化与自定义图层见 912；平台检查引用并删除/解绑，后续场景移除规则，历史归档保留 | T1/T2/T3；管理功能由平台测试 |
| 691（17） | 航路航线优先条件查询 / ENG-06 | 平台负责；约束消费已实现 | P/S/V | PRIORITY：priorityScene=PLANNING/LOW_RISK/QUIET/FAST、costBonus∈[0,1]；scope/资源范围、effective_start/end | rule_report、validation；规则仅调整非负候选代价，不能覆盖硬约束；ENG-06 选择图边权并执行 PRIORITY 空间代价，ENG-07 的目标归一化与自定义图层见 912；平台按 ID/区域/时间/状态分页查询；引擎无条件检索接口 | T1/T2/T3；管理功能由平台测试 |
| 692（18） | 航路航线规避条件信息 / ENG-06 | 平台负责；约束消费已实现 | P/S/V | BUILDING/NO_FLY/ZONE；scene.obstacles/control_zones 或 scope；几何、高度带、有效期 | rule_report、validation；命中活动时空区域按硬约束拒绝；过期规则不参与；平台另验业务图层存在性；平台保存结构化条件及版本 | T1/T2/T3；管理功能由平台测试 |
| 693（19） | 航路航线规避条件添加 / ENG-06 | 平台负责；约束消费已实现 | P/S/V | BUILDING/NO_FLY/ZONE；scene.obstacles/control_zones 或 scope；几何、高度带、有效期 | rule_report、validation；命中活动时空区域按硬约束拒绝；过期规则不参与；平台另验业务图层存在性；平台检查必填、重复及关联对象，再下发新规则 | T1/T2/T3；管理功能由平台测试 |
| 694（20） | 航路航线规避条件修改 / ENG-06 | 平台负责；约束消费已实现 | P/S/V | BUILDING/NO_FLY/ZONE；scene.obstacles/control_zones 或 scope；几何、高度带、有效期 | rule_report、validation；命中活动时空区域按硬约束拒绝；过期规则不参与；平台另验业务图层存在性；平台按既有 ID 更新，重新校验后下发全量场景 | T1/T2/T3；管理功能由平台测试 |
| 695（21） | 航路航线规避条件删除 / ENG-06 | 平台负责；约束消费已实现 | P/S/V | BUILDING/NO_FLY/ZONE；scene.obstacles/control_zones 或 scope；几何、高度带、有效期 | rule_report、validation；命中活动时空区域按硬约束拒绝；过期规则不参与；平台另验业务图层存在性；平台检查引用并删除/解绑，后续场景移除规则，历史归档保留 | T1/T2/T3；管理功能由平台测试 |
| 696（22） | 航路航线规避条件查询 / ENG-06 | 平台负责；约束消费已实现 | P/S/V | BUILDING/NO_FLY/ZONE；scene.obstacles/control_zones 或 scope；几何、高度带、有效期 | rule_report、validation；命中活动时空区域按硬约束拒绝；过期规则不参与；平台另验业务图层存在性；平台按 ID/区域/时间/状态分页查询；引擎无条件检索接口 | T1/T2/T3；管理功能由平台测试 |
| 697（23） | 飞行活动规避条件信息 / ENG-07 | 平台负责；约束消费已实现 | P/S/V | BUILDING/NO_FLY/ZONE；scene.obstacles/control_zones 或 scope；几何、高度带、有效期 | rule_report、validation；命中活动时空区域按硬约束拒绝；过期规则不参与；平台另验业务图层存在性；平台保存结构化条件及版本 | T1/T2/T3；管理功能由平台测试 |
| 698（24） | 飞行活动规避条件添加 / ENG-07 | 平台负责；约束消费已实现 | P/S/V | BUILDING/NO_FLY/ZONE；scene.obstacles/control_zones 或 scope；几何、高度带、有效期 | rule_report、validation；命中活动时空区域按硬约束拒绝；过期规则不参与；平台另验业务图层存在性；平台检查必填、重复及关联对象，再下发新规则 | T1/T2/T3；管理功能由平台测试 |
| 699（25） | 飞行活动规避条件删除 / ENG-07 | 平台负责；约束消费已实现 | P/S/V | BUILDING/NO_FLY/ZONE；scene.obstacles/control_zones 或 scope；几何、高度带、有效期 | rule_report、validation；命中活动时空区域按硬约束拒绝；过期规则不参与；平台另验业务图层存在性；平台检查引用并删除/解绑，后续场景移除规则，历史归档保留 | T1/T2/T3；管理功能由平台测试 |
| 700（26） | 飞行活动规避条件修改 / ENG-07 | 平台负责；约束消费已实现 | P/S/V | BUILDING/NO_FLY/ZONE；scene.obstacles/control_zones 或 scope；几何、高度带、有效期 | rule_report、validation；命中活动时空区域按硬约束拒绝；过期规则不参与；平台另验业务图层存在性；平台按既有 ID 更新，重新校验后下发全量场景 | T1/T2/T3；管理功能由平台测试 |
| 701（27） | 飞行活动规避条件查询 / ENG-07 | 平台负责；约束消费已实现 | P/S/V | BUILDING/NO_FLY/ZONE；scene.obstacles/control_zones 或 scope；几何、高度带、有效期 | rule_report、validation；命中活动时空区域按硬约束拒绝；过期规则不参与；平台另验业务图层存在性；平台按 ID/区域/时间/状态分页查询；引擎无条件检索接口 | T1/T2/T3；管理功能由平台测试 |
| 702（28） | 飞行活动优先条件信息 / ENG-07 | 平台负责；约束消费已实现 | P/S/V | PRIORITY：priorityScene=PLANNING/LOW_RISK/QUIET/FAST、costBonus∈[0,1]；scope/资源范围、effective_start/end | rule_report、validation；规则仅调整非负候选代价，不能覆盖硬约束；ENG-06 选择图边权并执行 PRIORITY 空间代价，ENG-07 的目标归一化与自定义图层见 912；平台保存结构化条件及版本 | T1/T2/T3；管理功能由平台测试 |
| 703（29） | 飞行活动优先条件添加 / ENG-07 | 平台负责；约束消费已实现 | P/S/V | PRIORITY：priorityScene=PLANNING/LOW_RISK/QUIET/FAST、costBonus∈[0,1]；scope/资源范围、effective_start/end | rule_report、validation；规则仅调整非负候选代价，不能覆盖硬约束；ENG-06 选择图边权并执行 PRIORITY 空间代价，ENG-07 的目标归一化与自定义图层见 912；平台检查必填、重复及关联对象，再下发新规则 | T1/T2/T3；管理功能由平台测试 |
| 704（30） | 飞行活动优先条件删除 / ENG-07 | 平台负责；约束消费已实现 | P/S/V | PRIORITY：priorityScene=PLANNING/LOW_RISK/QUIET/FAST、costBonus∈[0,1]；scope/资源范围、effective_start/end | rule_report、validation；规则仅调整非负候选代价，不能覆盖硬约束；ENG-06 选择图边权并执行 PRIORITY 空间代价，ENG-07 的目标归一化与自定义图层见 912；平台检查引用并删除/解绑，后续场景移除规则，历史归档保留 | T1/T2/T3；管理功能由平台测试 |
| 705（31） | 飞行活动优先条件修改 / ENG-07 | 平台负责；约束消费已实现 | P/S/V | PRIORITY：priorityScene=PLANNING/LOW_RISK/QUIET/FAST、costBonus∈[0,1]；scope/资源范围、effective_start/end | rule_report、validation；规则仅调整非负候选代价，不能覆盖硬约束；ENG-06 选择图边权并执行 PRIORITY 空间代价，ENG-07 的目标归一化与自定义图层见 912；平台按既有 ID 更新，重新校验后下发全量场景 | T1/T2/T3；管理功能由平台测试 |
| 706（32） | 飞行活动优先条件查询 / ENG-07 | 平台负责；约束消费已实现 | P/S/V | PRIORITY：priorityScene=PLANNING/LOW_RISK/QUIET/FAST、costBonus∈[0,1]；scope/资源范围、effective_start/end | rule_report、validation；规则仅调整非负候选代价，不能覆盖硬约束；ENG-06 选择图边权并执行 PRIORITY 空间代价，ENG-07 的目标归一化与自定义图层见 912；平台按 ID/区域/时间/状态分页查询；引擎无条件检索接口 | T1/T2/T3；管理功能由平台测试 |
| 778（49） | 航路航线下发 / ENG-06 | 部分实现；规划已实现，设备下发由平台 | P/V | ENG-06，strict/prefer；航路/接驳、有效规则、机型与时间；先执行硬约束 | trajectory_wkt、vertex_times_s、motion_segments、keypoints；平台进行业务审批、设备协议转换和实际下发；设备控制 API 暂无 | T2/T4/T5/T7 |
| 855（72） | 规则库新增 / ENG-06 | 平台负责；规则消费已实现 | P/S/V | scene.rules：id、rule_type/content（兼容 type/constraints）、scope、resource_ids、effective_start/end；类型阈值严格校验 | rule_report/violations；平台新增时校验规则 ID/名称、类型、阈值单位、对象和重复；规则库 CRUD、名称/等级与历史管理 API 暂无 | T1/T2/T3；CRUD/审计由平台测试 |
| 856（73） | 规则库查询 / ENG-06 | 平台负责；规则消费已实现 | P/S/V | scene.rules：id、rule_type/content（兼容 type/constraints）、scope、resource_ids、effective_start/end；类型阈值严格校验 | rule_report/violations；平台按 ID/类型/对象/时段分页查询版本和状态；规则库 CRUD、名称/等级与历史管理 API 暂无 | T1/T2/T3；CRUD/审计由平台测试 |
| 857（74） | 规则库编辑 / ENG-06 | 平台负责；规则消费已实现 | P/S/V | scene.rules：id、rule_type/content（兼容 type/constraints）、scope、resource_ids、effective_start/end；类型阈值严格校验 | rule_report/violations；平台按已实现 ID 更新后重验全部字段及关联；规则库 CRUD、名称/等级与历史管理 API 暂无 | T1/T2/T3；CRUD/审计由平台测试 |
| 858（75） | 规则库删除 / ENG-06 | 平台负责；规则消费已实现 | P/S/V | scene.rules：id、rule_type/content（兼容 type/constraints）、scope、resource_ids、effective_start/end；类型阈值严格校验 | rule_report/violations；平台删除前检查引用/执行任务并保存审计；规则库 CRUD、名称/等级与历史管理 API 暂无 | T1/T2/T3；CRUD/审计由平台测试 |
| 891（76） | 航线规则绑定信息新增 / ENG-06 | 平台负责；绑定结果消费已实现 | P/S/V | scene.rules + task.rule_ids；null 选择全部、[] 不选业务规则；每个 ID 必须存在；生效时段按规则字段下发 | rule_report 区分应用/不适用/失败；平台校验航线/规则存在、去重、作用范围并建立绑定；绑定记录 CRUD API 暂无 | T1/T3；绑定 CRUD 由平台测试 |
| 892（77） | 航线规则绑定信息编辑 / ENG-06 | 平台负责；绑定结果消费已实现 | P/S/V | scene.rules + task.rule_ids；null 选择全部、[] 不选业务规则；每个 ID 必须存在；生效时段按规则字段下发 | rule_report 区分应用/不适用/失败；平台按绑定 ID 修改关系与时段并重验；绑定记录 CRUD API 暂无 | T1/T3；绑定 CRUD 由平台测试 |
| 893（78） | 航线规则绑定信息查询 / ENG-06 | 平台负责；绑定结果消费已实现 | P/S/V | scene.rules + task.rule_ids；null 选择全部、[] 不选业务规则；每个 ID 必须存在；生效时段按规则字段下发 | rule_report 区分应用/不适用/失败；平台查询绑定、规则版本与有效状态；绑定记录 CRUD API 暂无 | T1/T3；绑定 CRUD 由平台测试 |
| 894（79） | 航线规则绑定信息删除 / ENG-06 | 平台负责；绑定结果消费已实现 | P/S/V | scene.rules + task.rule_ids；null 选择全部、[] 不选业务规则；每个 ID 必须存在；生效时段按规则字段下发 | rule_report 区分应用/不适用/失败；平台检查执行任务及下游引用后解绑；绑定记录 CRUD API 暂无 | T1/T3；绑定 CRUD 由平台测试 |
| 897（80） | 航线绑定设备类型信息新增 / ENG-06 | 平台负责；机型准入硬校验已实现 | P/S/V | airways[].allowed_profile_ids；task.profile_id 或 aircraft.id；白名单非空时实际机型必须获准 | strict/prefer 中不准入航路体禁入，重叠获准航路不抵消；free 不使用航路准入；平台建立设备类型与航路关系并检查两端对象及重复；设备档案与绑定 CRUD API 暂无 | T1/T5；管理功能由平台测试 |
| 898（81） | 航线绑定设备类型信息查询 / ENG-06 | 平台负责；机型准入硬校验已实现 | P/S/V | airways[].allowed_profile_ids；task.profile_id 或 aircraft.id；白名单非空时实际机型必须获准 | strict/prefer 中不准入航路体禁入，重叠获准航路不抵消；free 不使用航路准入；平台查询准入绑定及状态；设备档案与绑定 CRUD API 暂无 | T1/T5；管理功能由平台测试 |
| 899（82） | 航线绑定设备类型信息编辑 / ENG-06 | 平台负责；机型准入硬校验已实现 | P/S/V | airways[].allowed_profile_ids；task.profile_id 或 aircraft.id；白名单非空时实际机型必须获准 | strict/prefer 中不准入航路体禁入，重叠获准航路不抵消；free 不使用航路准入；平台按绑定 ID 更新并重验；设备档案与绑定 CRUD API 暂无 | T1/T5；管理功能由平台测试 |
| 900（83） | 航线绑定设备类型信息删除 / ENG-06 | 平台负责；机型准入硬校验已实现 | P/S/V | airways[].allowed_profile_ids；task.profile_id 或 aircraft.id；白名单非空时实际机型必须获准 | strict/prefer 中不准入航路体禁入，重叠获准航路不抵消；free 不使用航路准入；平台检查执行任务引用后删除绑定；设备档案与绑定 CRUD API 暂无 | T1/T5；管理功能由平台测试 |
| 901（84） | 航路连通性路径规划 / ENG-06 | 已实现 | POST /api/v2/plan/single；POST /api/v2/plan/batch；POST /api/v2/networks/analyze；POST /api/v2/validate | engine_id=ENG-06；strict/prefer；scene.airway_graph 节点、方向边、关联航路及版本；task.network 指定起终节点、途经节点、权重字段及预期版本 | network_route 返回节点、边、原始权重和、优先规则调整后的代价；分析接口返回有效边、弱连通及强连通分量；运动执行航路、机型、规则及连续安全检查 | T1/T2/T5；tests/planning/test_network.py；tests/api/test_network_api.py |
| 902（85） | 三维网格无航路规划 / ENG-07 | 已实现 | POST /api/v2/plan/single；POST /api/v2/plan/batch；POST /api/v2/validate | engine_id=ENG-07；route_mode=free；bounds/resolution；GridSpec.level；起终点 WKT 或 start_grid/goal_grid；search 指定 ASTAR/DIJKSTRA、6/18/26 邻域及可选扩展预算 | traversed_grids 按真实运动穿越顺序返回全部网格及进入/离开时刻，保留上游编码；grid_context 描述网格；search_report 返回实际配置及扩展数 | T1/T2/T5；tests/planning/test_search_grid_contract.py；tests/planning/test_network.py |
| 903（86） | 多场景路径支持 / ENG-07 | 部分实现；低空已实现 | POST /api/v2/scenes；POST /api/v2/plan/single；POST /api/v2/plan/batch | scene_type=LOW_ALTITUDE；不同地图可登记不同 scene；一次 batch 共用一个 scene | 低空单机/多机规划使用不同 scene_id 选择地图；地面车辆和水上船舶由 ENG-08/09 负责，其接口暂无 | T1/T7 |
| 904（87） | 路径规划算法引擎 / ENG-07 | 已实现 | POST /api/v2/plan/single；POST /api/v2/plan/batch；POST /api/v2/jobs；POST /api/v2/validate | 起终点或网格端点、bounds/resolution、机型、天气、规则；search 指定算法/邻域/可选扩展预算；CLIMB 爬降速率和角度；objectives 自定义非负代价 | 实际三维路径、运动定时、完整网格和规则报告；search_report 记录搜索配置及扩展数，预算耗尽返回明确状态 | T1/T2/T4/T5；tests/planning/test_search_grid_contract.py；test_objectives.py |
| 909（88） | 路径规划性能优化 / ENG-07 | 已实现 | POST /api/v2/scenes；POST /api/v2/plan/batch；POST /api/v2/jobs；POST /api/v2/plan/replan | scene_id 复用预编译；execution.workers/batch_size；可选 time_budget_s、search.max_expanded_nodes；trigger.threshold/min_interval_s/max_frequency_hz | 实际 execution_report、stage_timings_ms、search_report；增量输出复用量与 new_search_calls；所有提交组统一预约，默认不设置计算预算 | T9/T10/T12；实际耗时见本次容器报告 |
| 912（89） | 多目标路径规划 / ENG-07 | 已实现 | POST /api/v2/plan/single；POST /api/v2/plan/batch；POST /api/v2/validate | ENG-07/free；objectives 配置距离/时间/能耗/风险权重及归一化，显式能耗模型；带版本 risk/cost 数据集，自定义 cost_layers；可选迭代/候选预算 | 主方案按已验证候选的加权归一化总分选择；alternatives 返回其他非支配完整路线；objectives 给出原始值、单位和数据版本；optimization 给出实际迭代及预算状态，比较范围为 validated_candidates | tests/planning/test_objectives.py；tests/api/test_objectives_api.py；test_api/data/features/objectives_request.json |
| 913（90） | 动态路径重新规划 / ENG-07 | 已实现 | POST /api/v2/plan/replan；POST /api/v2/validate | previous_scene/task/route；current_state 的位置、东/北/上速度及时间；remaining_task；最新 scene/scene_id；event 与 trigger；EG 坐标上下文一致 | decision 区分 reused/replanned/ignored/failed；复用已验后续或前段，真实减速/偏航/等待后接续；完整路线含实际初始速度，检查剩余任务、返航、多目标及动态冲突 | T12；test_api/data/features/replan_request.json |
| 916（93） | 路径规划策略配置 / ENG-07 | 已实现计算配置；策略档案由平台负责 | POST /api/v2/scenes；POST /api/v2/plan/single；POST /api/v2/plan/batch；POST /api/v2/validate | objectives 权重与归一化；SPEED 巡航速度；HEIGHT_LAYER 高度带；CLIMB 爬降率和角度；TURN 半径/转角；WEATHER 风速/能见度；profile/state；各数值单位见 API.md | 配置参与实际搜索、运动定时和验证，输出目标值与规则报告；平台保存策略 ID、版本及增删改查 | T1/T2/T3/T4；test_search_grid_contract.py；test_objectives.py |

## 规则与高度接入检查

1. 所有 WKT 使用经度、纬度、高度顺序，单位为度、度、米。内部统一 EG 椭球高，局部 z=EG−origin_alt_ellipsoid。MSL 转换使用 h=H+N，必须提供 geoid_separation_m 和 geoid_source；AGL 必须提供地形椭球高。QNE/QNH 的 offset_m 定义为 EG−输入高度。
2. RuleBO 使用 id/rule_type/content；Word 样例 type/constraints 可作为输入别名，仍必须补 id。scope.grid 索引需要 scene.grid；几何形式使用 POINT Z/POLYGON Z。scope.geom 类型和几何必须匹配；除起降场外不能只给中心点，geom 与 boundary_2d 同时输入时水平足迹须在 0.1 m 内一致。SEPARATION 只提供全局间隔检查并拒绝 scope，区域邻近提醒归属 ENG-11，当前暂无。scope 只定位作用域，不能代替天气、信号观测值。
3. DIRECTION 为北 0°、东 90°、南 180°、西 270°。TIME.weekdays 为 0 周日、1 周一至 6 周六。规则生效时间无时区时按 Asia/Shanghai，空字符串转 null；其他对象仍要求带时区。
4. WIDTH.width 与 airway.widthMeters 均为全宽。bufferWidth 表示外侧保护带，不增加可飞行宽度；障碍在有效高度和时间内进入已用航路段中心线 min(width,widthMeters)/2+bufferWidth 的保护范围时，该段不可用，外部占用网格和选定规避规则的 scope 几何同样参与硬检查，按规则实际生效的时间和空间交集裁剪。WAYPOINT.maxWaypoints 只计 task.waypoints 的途经点，不计起终点与采样点。
5. 未绑定有效 CLIMB/TURN 时默认爬升 5 m/s、下降 3 m/s、最大转角 90°、最小半径 10 m；同时满足机型和航路限制。NO_FLY 支持 STANDARD，BUILDING 支持 STRICT，PRIORITY 支持 PLANNING。
6. 查询每条结果的 engine_id、alt_reference、coordinate_context。成功必须同时检查 status、validation.valid 和规则报告。HTTP 200、作业 succeeded、或仅能解析输入均不等于规划可用。

## 不在这 41 项中的平台功能

历史版本曾将 901–916 连续编号都视为算法需求。本次按源表准确归属。905 结果持久存储、906 业务日志、908 可视化由平台实现；907 可查询当前进程作业，持久历史查询暂无；910 使用现有 16 接口；911 提供 health/health/jobs/version 基础状态，资源监控和告警由平台实现；914 使用 V；915 读取 validation、rule_report、metrics，综合评分与排名暂无。

平台保存完整输入、原始场景、版本、响应和验证结果。场景与作业只驻留当前进程，重启后清空。后续请求不会自动继承先前飞行轨迹；其他在飞任务必须通过 dynamic_obstacles 下发。

## 本次验收方法

在仓库运行 `python -m pytest tests/planning/test_engine_contracts.py tests/planning/test_rules.py tests/planning/test_coordinates.py tests/planning/test_ellipsoid_altitudes.py tests/planning/test_time_windows.py tests/api/ -q`，并保存实际结果。使用新契约输入联调，归档完整请求和响应；修改高度口径后不能将旧 JSON 直接作为通过证据。本服务的计算项按实际输入、输出和验证结果验收；平台管理和其他引擎按各自职责验收。

## 本次接入顺序

1. 航路任务：登记带 airway_graph 和关联航路的 Scene，调用 plan/single 或 plan/batch，task.network 选择起终节点、途经节点和边权字段。networks/analyze 查询结构连通性。
2. 三维自由任务：route_mode=free，输入 WKT 或外部网格端点；search 选择算法和邻域。输出 traversed_grids 给出完整网格序列。
3. 多机任务：plan/batch 提交 tasks，按需设置 execution；同一批全部任务统一避让。
4. 多目标任务：填写 objectives 权重、归一化、能耗模型及风险/业务代价数据，读取主方案、各项目标和 alternatives。
5. 飞行中变化：平台保留原输入和完整结果，携带当前状态、剩余任务、最新场景和事件调用 plan/replan。新路线连同 initial_velocity_xyz_mps 提交 validate 独立复核。

41 项中的规则/条件/绑定/机型准入管理共 32 项由平台负责，778 的设备下发由平台负责。903 的低空部分由本服务实现，地面和水上部分由 ENG-08/09 负责。本轮补齐其余计算与策略参数；新增管理和设备控制接口由平台交付。
