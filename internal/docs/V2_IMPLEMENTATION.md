# 实施与验收记录

服务仅提供 `/api/v2`。对外字段、规则与示例见 [`../../docs/API.md`](../../docs/API.md)。运行版本通过 `/api/v2/version` 查询。

## 1. 当前实现

- JSON 请求与响应，WGS84 经纬度 WKT。
- 显式 MSL/AGL/ELLIPSOID/QNE/QNH 高度基准与转换。
- 默认 strict 航路模式，支持 prefer 和 free。
- 航路、管制区、机型性能和 20 类规则执行与验证。
- 场景注册、网格适配、单任务与批量规划、异步作业。
- C++17 搜索、连续时间预约、碰撞检查和运动学验证。
- 轨迹 WKT、顶点时间、运动原语、关键点与规则报告。
- 完整测试输入与输出归档、重放和校验和。

## 2. 2026-10-17 调整

当前服务和 SDK 仅保留地理坐标规划接口。对外文档、Swagger 和发布检查使用同一接口集合。

场景对象、几何顶点、数据格、网格节点、任务、航点、验证数据和采样不设固定数量上限。场景登记、队列和历史结果不设固定存储预算，也不自动淘汰。

`time_budget_s` 默认 null，省略时持续计算。调用方可设置任意正有限秒数。`horizon_s` 默认 3600 秒，表示飞行时域。`max_radius_m` 默认 50000 m，表示场景覆盖半径。后二者可设置更大的正数。

内存分配失败返回 HTTP 503、`RESOURCE_EXHAUSTED`。异步作业记录 failed 和同名错误码。系统 OOM 终止进程时，连接可能断开，Docker 记录事件并重启服务。正式资源保持 2 CPU / 2 GiB。

## 3. 验证方法

```bash
python -m pytest tests/ -q
python tools/benchmark_v2.py --validate-only
python tools/smoke_v2.py --base-url http://127.0.0.1:18000
```

记录当次提交、测试输出、镜像 ID、资源配额和完整烟测结果。接口和资源调整后的结果以本次验证为准。发布步骤见 [`DEPLOYMENT.md`](DEPLOYMENT.md)。

测试按 `api/`、`planning/`、`native/`、`integration/` 分类。只保留 v2 接口及其规划实现的测试。目录与命令见 [`../../tests/README.md`](../../tests/README.md)。完整 JSON 保存在 `tests/data/recordings/`。

## 4. 历史基准

固定数据种子为 20261016。场景为 5 km × 5 km，局部高度 0–300 m，分辨率 25 m × 25 m × 10 m，飞行时域 3600 秒。strict/mixed/free 三套各有 1000 个任务及参考调度。mixed 覆盖全部 20 类规则。

2026-10-16 的性能验收使用 8 CPU / 8 GiB，三套各运行 10 轮。30/30 轮通过。每轮返回 1000 条有效路线，连续冲突为 0。strict/mixed/free 最大完整 HTTP/JSON 耗时为 1.454/4.541/1.191 秒。cgroup memory.peak 为 281.72 MiB，进程 VmHWM 为 327.51 MiB，OOM 为 0。

原始报告见 [`../reports/v2-20261016/PERFORMANCE.md`](../reports/v2-20261016/PERFORMANCE.md)。历史输入、报告和采集结果保持原样。完整请求与响应见 [`../../tests/data/README.md`](../../tests/data/README.md)。
