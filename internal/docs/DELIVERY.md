# v1 / v2 交付说明

本文件用于内部交付管理。对接方只需阅读 [`../../docs/API.md`](../../docs/API.md)，字段与规范以同一镜像提供的 `/openapi.json` 为准。v1 保留；软件包和服务版本为 2.0.0。

## 1. 交付内容

| 内容 | 路径 / 入口 | 说明 |
|------|------|------|
| 唯一对外说明 | `docs/API.md` | v1 原契约、v2 全部字段、20 类规则、示例、迁移和错误 |
| 机器可读契约 | `docs/openapi.json`、运行端点 `/openapi.json` | 从 `server.app:app` 类型模型生成；Swagger `/docs` 读取规范，Markdown 不参与生成 |
| Python SDK | `uav_planner/`、`src/` | v1 与 v2 的实际规划、规则、坐标、序列化和验证实现 |
| 原生内核 | `native/`、`CMakeLists.txt`、构建后的 `uav_planner.v2._native` | C++17 / pybind11；v2 不以 Python 回退掩盖缺扩展 |
| REST 服务 | `server/`、`serve.py` | `/api/v1` 与 `/api/v2`、类型模型、异步作业、统一错误信封 |
| 可复现环境 | `pyproject.toml`、依赖文件、Dockerfile、compose.yaml | Python 3.11 多阶段构建；同一镜像包含代码、API 文档与规范 |
| 功能回归 | `tests/` | 原 v1 用例、v1 安全回归及 v2 坐标/规则/内核/接口/基准契约；用当次 pytest 输出记录用例数，不沿用旧计数 |
| 示例数据 | `input/`、`GET /api/v1/examples/*`、`GET /api/v2/examples/*` | 无真实外部数据也能完成基本接入 |
| 固定性能验收 | `input/v2_benchmarks/`、`tools/benchmark_v2.py` | 输入 SHA-256 manifest、独立参考调度、完整 SDK/HTTP 规划与独立验证 |
| 运维与架构 | `internal/docs/DEPLOYMENT.md`、`internal/docs/V2_ARCHITECTURE.md` | 内部审查与运行手册，不增加第二份对外接口说明 |

`tools/` 中普通规划和可视化脚本仍是开发工具，不构成 HTTP/SDK 调用依赖；验收脚本及冻结输入要随验收包保留。`output/` 为当次结果证据，按交付版本归档，不混入默认示例。课题原始材料、历史归档和个人环境文件不作为调用依赖。

## 2. 对外能力与平台责任

v2 提供版本化场景、WGS84/WKT 与显式高度换算、三种路径模式、20 类规则、运动学建模、连续冲突调度、路径校验及解释。严格模式默认启用；越出航路仅允许显式起终点接驳，失败不自动变更模式。

平台提供真实航路、障碍、管制、区域/网格编码、地形、天气/信号场、数据有效期与来源，以及飞机当前电量、剩余航程/续航和 home。算法不采集这些外部数据。RTL 根据当前提交状态生成返航方案，平台负责持续监测、飞控执行和后续重规划。内置 DJI 配置属于明确标注的工程规划默认值。

高度换算需提供对应数据。缺少数据或引用时返回错误。路线经过未覆盖、过期或 nodata 区域时，规划按相关规则处理。

场景、网格、任务、采样和输出不设固定资源上限。默认不设计算时限或搜索次数。调用方可用 `time_budget_s` 设置时限。内存分配失败返回 HTTP 503、`RESOURCE_EXHAUSTED`。系统 OOM 终止进程时，连接可能断开，Docker 记录事件并重启服务。飞行器性能和 20 类规则照常执行。

## 3. 验证与验收口径

```bash
uv pip install -e '.[server,test]' -c requirements-runtime.lock
python -m pytest tests/ -q
python tools/benchmark_v2.py --validate-only
```

SDK 冒烟应调用 v1 `plan_single` 和 v2 `register_scene/plan_single_v2/plan_batch_v2/validate_route_v2`；HTTP 冒烟应验证两版本 health、examples、同步规划、异步完整结果、v2 坐标转换及错误信封。具体可执行步骤见部署指南。

验收场景在优化前冻结，使用种子 20261016 与 `input/v2_benchmarks/manifest.json` 中的 SHA-256；不改任务、减少约束或缓存任务答案来制造达标。历史性能验收条件为 strict/mixed/free 三套各 1000 个已知可行任务，在场景预注册后，**8 CPU / 8 GiB、每套 10 轮、每轮 1000 条全部成功并独立验证通过、完整 HTTP 与 JSON 返回 ≤10 秒**。正式默认配额现为 **2 CPU / 2 GiB**，上述性能结果不能用于承诺该配额下 1000 任务仍在 10 秒内完成。脚本另支持 10/100 规模诊断，不计入历史 30 轮性能门槛。

独立验证包括端点/到达窗口、连续几何与规则、从公开运动原语重建的原生运动学证书、连续多机间隔和返回条数。计时包含请求构建、实际规划和完整 JSON；场景编译单独记录；验证发生在计时结束后。异步 202、排队开始、仅原生核耗时、失败任务省略和只看成功任务耗时都不满足验收口径。HTTP 客户端 RSS 不是服务端内存证据，需另存 `docker stats`/容器资源限制。

| 历史证据 | 已归档状态 |
|------|------|
| 宿主机 SDK 诊断：每套 1000 条 | 初轮 strict 1.514 s / mixed 4.623 s / free 1.211 s，全部成功；仅诊断 |
| 8 CPU / 8 GiB 候选镜像、三套十轮完整 HTTP | 30/30 轮通过，每轮 1000/1000；最大 strict/mixed/free 1.454/4.541/1.191 s |
| 镜像内 v1/v2 冒烟与 OpenAPI 一致性 | 候选与正式服务均通过，源码及规范哈希一致 |
| 当次正式服务替换 | 18000 端口 API 2.0.0 通过当次烟测；不表示历史镜像仍在本地，回滚改为按 Git SHA 重建 |

完整证据见 [性能报告](../reports/v2-20261016/PERFORMANCE.md)。历史被测/已发布提交为 c14b18677a00abf20b8500f827417dd35aaa0d27，历史镜像 ID 为 sha256:60479a4fd6be59b40defd8e7b146a98522d649b04d134e901d0e8aacd42ada42；原始报告归档提交为 cd4780e，未改变算法代码。552 项完整回归及被测提交当次 GitHub CI 通过，30 轮原始记录与资源证据已归档；当前 main、最新 CI、部署与清理状态通过实时检查核对，不能沿用历史结论。

## 4. 打包与版本一致性

运行镜像必须由当前 Git SHA 构建原生 wheel，再将 wheel 安装到同版本 Python 3.11 运行层；不要只复制 Python 源文件并漏掉扩展。同一镜像包含 `docs/API.md` 和 `docs/openapi.json`。规范以锁定依赖的 Python 3.11 镜像为准；其他 FastAPI/Pydantic 版本的 schema 拆分可能不同。生成规范只读取模型，不执行规划 handler：

```bash
python - <<'PY'
import json
from pathlib import Path
from server.app import app
Path('docs/openapi.json').write_text(
    json.dumps(app.openapi(), ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
PY
```

镜像内应能导入 `uav_planner.v2._native`，且 `/api/v2/version`、包版本、构建标签与交付记录对应。v1 `/version` 保留其兼容语义，不以 v2 替换既有 v1 请求/返回字段。

## 5. 发布与回滚

场景与作业表保存在服务进程内存中，默认不自动淘汰。重启后清空。替换容器前，停止上游新提交，并排空 v1 和 v2 的 pending/running 作业。保存原始场景 JSON 和完整作业结果。重启后重新注册场景。服务保持单进程。

正式部署默认配额为 **2 CPU / 2 GiB**，端口保持 `127.0.0.1:18000 → 8000`；候选隔离在 18001。8 CPU / 8 GiB 仅用于复现历史性能验收。旧 v1 镜像 `low-altitude-path-planning:9816f8b` 及 ID `sha256:c03196d8cc7d1952ad29013e9dd70e12323127601615ba5253af7bc7f80e4e89` 仅作历史记录，不要求继续保留，也不声称仍在本地。回滚从选定 Git SHA 及该版本的构建配置重新构建并验收，再按 2 CPU / 2 GiB 切换。

必要变更全部合入 main 后，清理已整合的旧本地/远端分支及旧镜像；删除分支不能代替必要变更合并。实际 main SHA、运行镜像 ID、容器资源和剩余分支/镜像使用部署指南第 8 节的命令核对，不要求文档永久写死最新部署 SHA。完整构建、验收、切换与重建回滚命令见 [`DEPLOYMENT.md`](DEPLOYMENT.md)。

接口沿用现有无内置鉴权的部署方式；访问控制由已有可信网络、隧道或反向代理负责。CPU/内存配额用于资源隔离，不能代替访问控制。
