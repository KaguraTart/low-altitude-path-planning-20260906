# 低空无人机路径规划

文档日期：2026-10-29。

本项目提供航路规划、管制区避让和多机协同规划服务。本次交付聚焦 ENG-06 航路航线网络规划与 ENG-07 低空三维路径规划。

当前接口为 `/api/v2`。软件版本为 2.0.0。请求和响应使用 JSON，外部几何使用 WGS84 经纬度 WKT。内部计算使用米和秒，垂直基准统一为 EG 椭球高。契约版本为 `2026-10-29`，接口路径仍为 `/api/v2`。

## 文档入口

| 用途 | 文档 |
|------|------|
| 对接接口、字段、规则与示例 | [`docs/API.md`](docs/API.md) |
| 需求与 API 对照、调用方法 | [`docs/REQUIREMENTS_API_MAPPING.md`](docs/REQUIREMENTS_API_MAPPING.md) |
| Swagger 交互文档 | 运行服务的 `/docs` |
| 机器可读接口规范 | [`docs/openapi.json`](docs/openapi.json)；运行服务的 `/openapi.json` |
| Docker 部署、验证与回滚 | [`internal/docs/DEPLOYMENT.md`](internal/docs/DEPLOYMENT.md) |
| 架构与实现 | [`internal/docs/V2_ARCHITECTURE.md`](internal/docs/V2_ARCHITECTURE.md) |
| 交付检查 | [`internal/docs/DELIVERY.md`](internal/docs/DELIVERY.md) |
| 测试目录与运行命令 | [`tests/README.md`](tests/README.md) |
| 完整测试输入与输出 | [`tests/data/README.md`](tests/data/README.md) |
| 可独立交付的公网接口测试包 | [`test_api/README.md`](test_api/README.md) |
| 接口字段与测试 JSON 修改方法 | [`test_api/API_FIELDS.md`](test_api/API_FIELDS.md) |

## 本次交付范围

平台负责规则 CRUD、绑定管理和设备类型准入管理；引擎消费 `scene.rules`、`task.rule_ids` 和 `airways[].allowed_profile_ids` 并执行约束。`strict/prefer` 自动归属 ENG-06，`free` 自动归属 ENG-07；也可显式填写一致的 `task.engine_id`。场景仅支持 `scene_type=LOW_ALTITUDE`。strict/prefer 将不准入航路体作为禁入区；free 不使用航路准入。

[需求对照表](docs/REQUIREMENTS_API_MAPPING.md)覆盖源清单全部 41 项 ENG-06/07 需求，并逐项列出输入、输出、验证入口及缺口。地面/水上交通、Pareto 多目标优化、增量重规划、规则管理、持久历史与展示不作为已交付能力。顺序途经点与重新提交规划分别只满足相应子能力。

新契约使用 `origin_alt_ellipsoid`、`terrain_ellipsoid_m`。MSL 转换需提供大地水准面差及来源；EG/ELLIPSOID 可直接使用。每条规划结果返回 `engine_id`、`alt_reference` 和 `coordinate_context`。WIDTH 为全宽及外侧保护带硬检查；航向北 0°顺时针；星期 0 周日、1–6 周一至周六。详见 [API](docs/API.md)。

## 规划能力

- 注册场景，复用已编译的几何、规则和空间索引。
- 使用 `strict`、`prefer` 或 `free` 模式规划。默认 `strict` 沿有效航路和显式起终点接驳飞行。
- 执行航路、管制区、飞行器性能和 20 类规则。
- 读取上游提供的地形、天气、信号、风险、区域和飞机状态。
- 接受 WKT 几何和局部米制占用网格。
- 双向转换 WGS84 与局部 AEQD 坐标，并处理声明的高度基准。
- 调度多机连续时间间隔，检查交叉、对向、悬停和动态轨迹冲突。
- 返回轨迹 WKT、顶点时间、解析运动段、关键点、规则报告与验证结果。
- 使用 C++17 / pybind11 原生核执行搜索、调度和运动学验证。

场景、网格、任务、采样与输出不设固定资源上限。`time_budget_s` 省略或为 null 时持续计算。调用方可传正有限秒数设置计算时限。`horizon_s` 指定飞行时域，`max_radius_m` 指定场景覆盖半径。

内存分配失败返回 `RESOURCE_EXHAUSTED`。系统 OOM 终止进程时，连接可能断开；Docker 记录事件并重启服务。正式 Docker 配额保持 2 CPU / 2 GiB。

## 本地启动

构建需要 Python 3.11、C++17 编译器和 CMake。

```bash
uv pip install -e '.[server,test]' -c requirements-runtime.lock
python serve.py --host 127.0.0.1 --port 8000
```

检查服务并提交完整示例：

```bash
curl --fail-with-body -sS http://127.0.0.1:8000/api/v2/health
curl --fail-with-body -sS http://127.0.0.1:8000/api/v2/examples/single -o /tmp/uav-single.json
curl --fail-with-body -sS http://127.0.0.1:8000/api/v2/plan/single \
  -H 'Content-Type: application/json' --data-binary @/tmp/uav-single.json
```

打开 `http://127.0.0.1:8000/docs` 查看接口。正式 Docker 服务使用 `http://127.0.0.1:18000`。

## Python SDK

SDK 与 HTTP 使用相同模型和规划实现。

```python
import json
from uav_planner import register_scene, plan_single_v2, validate_route_v2

with open('/tmp/uav-single.json', encoding='utf-8') as source:
    request = json.load(source)
scene = register_scene(request['scene'])
route = plan_single_v2(scene_id=scene['scene_id'], task=request['task'])
assert route['status'] == 'planned', route
checked = validate_route_v2(
    scene_id=scene['scene_id'], task=request['task'],
    trajectory_wkt=route['trajectory_wkt'],
    vertex_times_s=route['vertex_times_s'],
    motion_segments=route['motion_segments'],
)
assert checked['valid'], checked
```

批量规划使用 `plan_batch_v2`。坐标转换使用 `convert_coordinates`。完整示例见接口文档。

## Docker 部署

镜像使用提交 SHA 作为标签。Compose 默认分配 2 CPU / 2 GiB，映射 `127.0.0.1:18000 → 8000`。

```bash
UAV_BUILD_SHA=$(git rev-parse HEAD)
docker build --build-arg GIT_SHA="$UAV_BUILD_SHA" \
  -t "low-altitude-path-planning:$UAV_BUILD_SHA" .
bash tools/check_docker_release.sh "low-altitude-path-planning:$UAV_BUILD_SHA" \
  "$UAV_BUILD_SHA" output/ci-docker
```

发布前暂停新提交，并等待已有作业完成。保存原始场景和完整作业结果。按部署指南切换镜像，并重新注册场景。场景与作业保存在进程内存中，重启后清空。

## 测试与数据

向对接方提供整个 [`test_api/`](test_api/README.md) 目录。对方使用 Python 3.11 或更新版本运行 `python run_tests.py`，即可测试 `https://api.kaguratart.com` 的全部接口和三套各 1000 个任务。目录内附完整输入和实测报告。[2026-10-29 本地测试](test_api/reports/20261029_local/REPORT.md)通过 22 项检查，覆盖 14 个接口和三套各 1000 个任务。

测试按接口、规划、原生核和集成流程分类。目录说明与分类运行命令见 [`tests/README.md`](tests/README.md)。

```bash
python -m pytest tests/ -q
python tools/benchmark_v2.py --validate-only
python tools/smoke_v2.py --base-url http://127.0.0.1:18000
```

完整输入和响应保存在 [`tests/data/`](tests/data/README.md)。归档包含接口示例和 strict/mixed/free 三套各 1000 条航线结果。文件附运行环境、对应关系和 SHA-256。

对正式配额做完整功能重跑：

```bash
python tools/benchmark_v2.py --functional-only --base-url http://127.0.0.1:18000 \
  --scales 1000 --repeats 1 --output output/v2-functional
```

2026-10-16 的历史性能验收使用 8 CPU / 8 GiB。三套各运行 10 轮，30/30 轮通过。每轮均有 1000 条有效路线，冲突为 0。strict/mixed/free 最大完整 HTTP/JSON 耗时为 1.454/4.541/1.191 秒。完整证据见 [`internal/reports/v2-20261016/PERFORMANCE.md`](internal/reports/v2-20261016/PERFORMANCE.md)。正式配额的耗时按实际场景与负载测量。

v2 报告和完整测试归档按采集版本保存。当前接口以 `docs/API.md` 和运行服务的 OpenAPI 为准。
