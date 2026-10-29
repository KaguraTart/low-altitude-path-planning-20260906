# 交付说明

文档日期：2026-10-29。

服务仅提供 `/api/v2`。软件版本为 2.0.0。对接方阅读 [`../../docs/API.md`](../../docs/API.md)，机器可读字段以同一镜像的 `/openapi.json` 为准。

## 1. 交付内容

| 内容 | 路径 / 入口 | 说明 |
|------|------|------|
| 对外接口说明 | `docs/API.md` | 全部字段、20 类规则、错误与完整调用示例 |
| 接口规范 | `docs/openapi.json`、`/openapi.json` | 从实际类型模型生成；Swagger `/docs` 读取规范 |
| Python SDK | `uav_planner/` | 场景、规划、坐标转换与验证 |
| 原生核 | `native/`、`CMakeLists.txt`、`uav_planner.v2._native` | C++17 / pybind11 搜索、调度、碰撞与运动学验证 |
| HTTP 服务 | `server/`、`serve.py` | `/api/v2`、异步作业与统一错误信封 |
| 构建环境 | `pyproject.toml`、`requirements-runtime.lock`、Dockerfile、compose.yaml | Python 3.11 多阶段镜像；默认 2 CPU / 2 GiB |
| 回归测试 | `tests/`、`tests/README.md` | 按接口、规划、原生核和集成流程分类 |
| 完整测试数据 | `tests/data/` | 输入、完整响应、环境、索引与 SHA-256 |
| 固定基准 | `input/engine_contract_20261029/`、`tools/benchmark_v2.py` | 三套输入、参考调度和独立验证 |
| 运维与架构 | `internal/docs/` | 部署、回滚、实现与验收记录 |

`output/` 保存本次运行结果。需要交付的完整 v2 记录归档到 `tests/data/recordings/`。每套记录附采集版本和校验和。

## 2. 能力与输入

服务提供版本化场景、WKT/WGS84、显式高度换算、三种路径模式、20 类规则、运动建模、多机调度与轨迹验证。默认 `strict` 模式使用有效航路和显式起终点接驳。

平台提供航路、障碍、管制区、网格、地形、天气、信号、风险数据，以及飞机当前状态。算法根据提交状态生成任务或返航方案。平台负责状态监测、飞控执行和后续重规划。

场景、网格、任务、采样与输出不设固定资源上限。默认不设计算时限或搜索次数。调用方可用 `time_budget_s` 设置计算时限。飞行时域、场景覆盖范围、机型性能和选定规则照常执行。

内存分配失败返回 HTTP 503、`RESOURCE_EXHAUSTED`。异步作业记录 failed 和同名错误码。系统 OOM 终止进程时，连接可能断开，Docker 记录事件并重启服务。

## 3. 交付检查

```bash
uv pip install -e '.[server,test]' -c requirements-runtime.lock
python -m pytest tests/ -q
python tools/benchmark_v2.py --validate-only
python tools/smoke_v2.py --base-url http://127.0.0.1:18000
```

记录当次测试数量与结果。SDK 检查 `register_scene/plan_single_v2/plan_batch_v2/validate_route_v2/convert_coordinates`。HTTP 检查 health、示例、场景、规划、异步完整结果、坐标转换和错误信封。

分类运行命令见 [`../../tests/README.md`](../../tests/README.md)。归档校验见 `tests/integration/test_recordings.py`。

发布检查还需核对原生扩展、镜像 revision、文档、规范快照与运行时 OpenAPI。步骤见 [`DEPLOYMENT.md`](DEPLOYMENT.md)。

## 4. 性能与数据

基准种子为 20261016。冻结输入和参考调度位于 `input/engine_contract_20261029/`，manifest 记录 SHA-256。独立验证检查端点、时间窗、规则、解析运动学证书和连续机间间隔。

历史验收使用 8 CPU / 8 GiB，strict/mixed/free 三套各 1000 个任务，各运行 10 轮。30/30 轮通过，每轮返回 1000 条有效路线，冲突为 0。最大完整 HTTP/JSON 耗时为 1.454/4.541/1.191 秒。记录见 [`../reports/v2-20261016/PERFORMANCE.md`](../reports/v2-20261016/PERFORMANCE.md)。

正式服务使用 2 CPU / 2 GiB。按实际场景和负载记录耗时。新运行完整保存输入与输出，并附镜像版本、资源、场景版本和校验和。

## 5. 发布与回滚

使用实际 Git SHA 构建原生 wheel 和不可变镜像。镜像同时包含代码、`docs/API.md` 与 `docs/openapi.json`。原生扩展可导入，`/api/v2/version` 的 `engine.git_sha` 与构建提交一致。

发布前暂停新提交，并排空 pending/running 作业。保存完整结果与原始场景。切换后检查 health、version、Swagger、完整规划和异步结果，再重新注册场景并恢复提交。

场景和作业保存在单个进程内存中，重启后清空。默认不自动淘汰。正式端口为 `127.0.0.1:18000 → 8000`，资源为 2 CPU / 2 GiB。候选端口为 18001。

合入 main 后，清理已整合分支和旧镜像。回滚时从已验收的提交重新构建，在候选端口验证后切换。具体命令见 [`DEPLOYMENT.md`](DEPLOYMENT.md)。
