# 低空横向无人机路径调度

> 全空间无人体系建设项目 — 路径规划与导航算法
>
> **3.25 多维时空大规模并行规划算法** & **3.26 智能飞行线路规划算法**
>
> 当前版本：**2.0.0**（v1 保留，新增 WKT / WGS84 v2） · 最后更新：2026-10-16
>
> v2 Docker HTTP 验收 **30/30 轮通过**，正式服务已更新为同一验收镜像。下文原有 demo、压测和审计评分为 v1 历史记录；v2 证据见 [`性能报告`](internal/reports/v2-20261016/PERFORMANCE.md)。

---

## 0. 从哪看起

| 你是 | 看这里 |
|------|--------|
| **对接方**（要调用算法） | [`docs/API.md`](docs/API.md) → 起服务 → 打开 `/docs` |
| **要查字段**（参数 / 返回 / 错误码） | [`docs/API.md`](docs/API.md) |
| **要部署**（Docker / 公网 / 边缘鉴权） | [`internal/docs/DEPLOYMENT.md`](internal/docs/DEPLOYMENT.md) |
| **要交付**（交什么、不交什么） | [`internal/docs/DELIVERY.md`](internal/docs/DELIVERY.md) |
| **要跑命令行**（规划 / 压测 / 可视化） | 本文档 §3.3、§3.4；脚本在 [`tools/`](tools/) |
| **要了解算法** | 本文档 §5；[`internal/docs/archive/algorithm_design.md`](internal/docs/archive/algorithm_design.md) |
| **要了解 v2 实现与验收边界** | [`internal/docs/V2_ARCHITECTURE.md`](internal/docs/V2_ARCHITECTURE.md)；[`input/v2_benchmarks/`](input/v2_benchmarks/) |

---

## v2 当前实现

v2 在 `/api/v2` 提供强类型 JSON + WKT 接口，外部坐标为 WGS84 经度/纬度和显式高度基准，内部用局部 AEQD 米制坐标计算。v1 原有 URL、SDK 和输入结构继续保留。所有对外字段、20 类规则、完整 v1/v2 示例与迁移方法集中在 [`docs/API.md`](docs/API.md)；Swagger `/docs` 从同一服务的类型模型生成 OpenAPI，Markdown 不参与生成。

- 默认 `route_mode="strict"`：扫掠体须在有效航路或显式起终点接驳体内，无路就失败；`prefer` 允许航路外飞行并在候选图中使用 2 倍航路外基础代价，`free` 不要求航路包含。
- 场景可预注册，按输入 SHA-256 版本复用不可变编译结果；任务答案不缓存。场景和异步作业存在进程内存，重启后重新注册场景，服务保持单进程。
- 全部 20 类规则接入搜索筛选和最终验证；天气、信号、地形、区域、风险等数据及当前飞机状态由平台提供，缺少引用或相关场数据不可用会拒绝或阻断路径。
- C++17 / pybind11 原生内核承担图/网格搜索、连续时间预约、碰撞检测和独立运动学验证。输出含 WKT、逐顶点时间、解析运动原语与关键点；没有原生扩展时明确报服务故障。
- 内置 DJI Mini 4 Pro 为工程规划配置；平台负责真实包线、环境数据、飞机状态、飞控执行和持续重规划。

```bash
# 安装时构建 C++17 扩展；需 C++17 编译器与 CMake。
uv pip install -e '.[server,test]'
python serve.py --host 127.0.0.1 --port 8000
curl --fail-with-body -sS http://127.0.0.1:8000/api/v2/health
curl --fail-with-body -sS http://127.0.0.1:8000/api/v2/examples/single -o /tmp/uav-single-v2.json
curl --fail-with-body -sS http://127.0.0.1:8000/api/v2/plan/single \
  -H 'Content-Type: application/json' --data-binary @/tmp/uav-single-v2.json
```

部署使用仓库根目录多阶段 Dockerfile 构建 Python 3.11 原生包，Compose 为 `127.0.0.1:18000 → 8000`、8 CPU / 8 GB，通过 `UAV_IMAGE_TAG` 选择不可变 Git SHA 镜像。候选端口 18001、双版本作业排空、烟测与回滚命令见 [`internal/docs/DEPLOYMENT.md`](internal/docs/DEPLOYMENT.md)。

固定验收数据已冻结在 [`input/v2_benchmarks`](input/v2_benchmarks)，manifest 记录 SHA-256。`tools/benchmark_v2.py` 对 strict/mixed/free 三套各 1000 个已知可行任务各跑 10 轮；每轮要求全部成功、独立验证通过、完整 HTTP 与 JSON 返回 ≤10 秒。场景预加载和编译另计，不能只测异步提交。

```bash
python tools/benchmark_v2.py --validate-only
python tools/benchmark_v2.py --base-url http://127.0.0.1:18001 \
  --repeats 10 --scales 10 100 1000 --output output/v2_benchmarks/http
```

2026-10-16 Docker HTTP 验收已完成：8 CPU / 8 GiB、三套各 1000 任务各 10 轮，30/30 轮全部成功且独立验证、运动学证书与连续冲突检查通过。strict/mixed/free 最大完整 HTTP/JSON 耗时分别为 **1.454 / 4.541 / 1.191 秒**，均低于 10 秒门槛。完整回归 **552 项通过**。原始逐轮报告、输入哈希、阶段耗时和服务内存证据见 [`internal/reports/v2-20261016/PERFORMANCE.md`](internal/reports/v2-20261016/PERFORMANCE.md)。

正式服务 `127.0.0.1:18000` 已部署并通过 v1/v2、坐标往返及异步完整结果烟测；被测/已发布 revision 为 `c14b18677a00abf20b8500f827417dd35aaa0d27`，镜像 ID 为 `sha256:60479a4fd6be59b40defd8e7b146a98522d649b04d134e901d0e8aacd42ada42`。后续报告与发布证据提交晚于该 revision，算法代码保持一致，详见报告中的版本说明。

以下 §1–§9 保留原 v1 项目说明、工具和历史性能记录；其中“不采集气象”不影响 v2 使用上游提供的天气空间场执行规则。

---

## 1. 项目简介

本项目实现低空无人机（UAV）在复杂城市环境中的**多维时空路径规划**与**智能航线优化**，覆盖：

- **3.25 多维时空大规模并行规划**：四维时空网格 `(x, y, z, t)` 建模、时空 A*、多源障碍物融合、多线程并行引擎
- **3.26 智能飞行线路规划**：多目标加权优化（航程/时间/风险/能耗）、三次样条平滑、性能约束校验、航点指令生成

支持静态障碍（建筑物、塔、地形）+ 动态障碍（其他飞行器、鸟群、移动车辆）联合避障，并提供 3D 可视化与风险热力图。

---

## 2. 项目状态与审计结论

### 2.1 完成度

| 子项目 | 模块 | 状态 |
|--------|------|------|
| 3.25 多维时空大规模并行规划 | 多维时空约束建模 | ✅ 完成 |
| 3.25 | 静态与动态避障 | ✅ 完成 |
| 3.25 | 多源规划数据融合 | ✅ 完成 |
| 3.25 | 大规模并行规划引擎 | ✅ 完成 |
| 3.25 | 秒级性能优化与规模验证 | ✅ 完成（详见 `output/stress/STRESS_TEST_RESULTS.md`） |
| 3.26 智能飞行线路规划 | 地理管制数据融合（气象不做） | ✅ 完成 |
| 3.26 | 飞行器性能与任务约束建模 | ✅ 完成 |
| 3.26 | 多目标航线优化 | ✅ 完成 |
| 3.26 | 航点生成与结果解释 | ✅ 完成 |
| 3.26 | 规划服务封装与综合评测 | ✅ 完成 |

> **气象相关功能按需求文档 `demand.md` 中「气象相关的做不了」明确不做。**

### 2.2 综合评分

| 维度 | 评分 | 说明 |
|------|------|------|
| 功能完整性 | 9 / 10 | 全部需求模块实现并接入主链路；§五.4 多机冲突避免已进入规划主循环（见 §7.1 P0）。剩余：代价层只读静态障碍、`ConflictResolver.suggest_delay` 之外的协调策略 |
| 代码可读性 | 8 / 10 | 结构清晰，命名规范；存在少量死代码（已识别待清理） |
| 算法正确性 | 8 / 10 | Demo 合规任务 100% 成功且通过 7 维合法性验证；**多机航线残余冲突 0、安全率 100%**；成功率与安全率是两个维度，见 §8.2 |
| 性能 | 8 / 10 | **P2 共享网格优化 1.81× 提速**；仍可 JIT 进一步加速 |
| 工程化 | 8 / 10 | 217 个单元测试 + GitHub Actions CI；日志/监控未接入 |
| 可视化 | 8 / 10 | Matplotlib 3D + Plotly HTML + 风险热力图齐全 |
| **综合** | **8.3 / 10** | **可交付**；大规模生产前需要关键瓶颈优化（尾延迟） |

### 2.3 一句话总结

> **核心算法在 demo 场景下合规任务 100% 成功且全部通过合法性验证；万级规模随机任务压测保持 ≥ 85% 成功率，且规划成功的航线残余冲突为 0（安全率 100%）。**
> **成功率与安全率是正交指标：前者回答"能不能算出路径"，后者回答"算出的路径能不能无碰撞飞行"，§8.2 两个数字同时给出。**
> **主要短板是极端规模下的尾延迟（P95 数秒）与吞吐优化（空间/时间复杂度均有优化空间）。**

---

## 3. 快速开始

### 3.1 环境要求

- Python 3.11（已在 3.11.16 / 3.14.5 测试）
- 依赖：`numpy`, `scipy`, `matplotlib`, `plotly`, `pydantic`

### 3.2 环境配置（conda 推荐）

本项目使用**专属 conda 环境** `uav_path_planning`（Python 3.11），避免与其他项目依赖冲突。

#### 一键脚本

```bash
./tools/setup_env.sh           # 创建/激活环境并安装依赖
./tools/setup_env.sh plan      # 同上 + 跑 demo 规划 + 自动可视化
./tools/setup_env.sh stress    # 同上 + 跑压力测试 + 自动可视化
./tools/setup_env.sh all       # 完整流程
```

#### 手动配置

```bash
# 方式 A：用 environment.yml 复现（推荐）
conda env create -f environment.yml
conda activate uav_path_planning

# 方式 B：从零创建
conda create -n uav_path_planning python=3.11 -y
conda activate uav_path_planning
pip install -r requirements.txt

# 验证
python -c "import numpy, scipy, matplotlib, plotly, pydantic; print('OK', plotly.__version__)"
```

### 3.3 运行（**规划与可视化分离**）

> 📘 **接口说明文档**：[`docs/API.md`](docs/API.md)（接口列表、参数、返回结构、错误码、调用示例、FAQ）
> ｜ 命令行输出文件字段：[`internal/docs/archive/INPUT_OUTPUT_GUIDE.md`](internal/docs/archive/INPUT_OUTPUT_GUIDE.md)

本项目把**规划代码**和**可视化代码**完全分离：

| 步骤 | 脚本 | 作用 | 输出 |
|------|------|------|------|
| ① 单机规划 | `tools/plan_single.py` | 选最高优先级任务 + 飞行器需求校验 + A\* + 优化 + 写 JSON/MD | `output/single/*.json` `output/single/*.md` |
| ② 多机批量 | `tools/plan_multi.py` | 多线程并行 + 需求校验 + **多机冲突避免** + 写 JSON/MD | `output/multi/*.json` |
| ③ 可视化（demo） | `tools/visualize.py` | 读 JSON + 渲染 3D/热力图/对比柱状图 | `output/{single\|multi}/visualizations/*.png\|html` |
| ④ 压力测试 | `tools/run_stress_test.py` | 随机任务批量规划 + 统计 | `output/stress/*.json` |
| ⑤ 压测可视化 | `tools/visualize_stress.py` | 读 JSON + 渲染对比图 | `output/stress/visualizations/` |

```bash
conda activate uav_path_planning

# 第 1 步：单机规划（默认按业务优先级选最高任务，含 3.26 多目标优化）
python tools/plan_single.py --output-dir ./output/single

# 指定任务
python tools/plan_single.py --task task-001 --output-dir ./output/single
python tools/plan_single.py --no-optimize  # 跳过 3.26 多目标优化

# 第 2 步：可视化
python tools/visualize.py --input ./output/single

# 多机批量规划（含需求校验 + 多机冲突避免，残余冲突时退出码 2）
python tools/plan_multi.py --output-dir ./output/multi --conflict-check
python tools/plan_multi.py --no-avoid-conflicts   # 只检测不避让（对照用，仍会报冲突）
python tools/plan_multi.py --dispatch       # 调度模式（更快）
```

支持的模式：

```bash
python tools/plan_single.py --no-optimize    # 仅 A*（跳过 3.26）
python tools/plan_single.py --task task-002  # 指定任务
python tools/plan_multi.py  --dispatch       # DISPATCH 模式
python tools/plan_multi.py  --workers 8      # 自定义线程数
```

> **本机多 Python 环境说明**：系统自带 Python 3.14 当前未安装 `plotly`；其他 conda 环境（`DLNN`/`LLM`/`Python13`/`UAV_platform`/`air`/`simlingo`/`yolo`）当前也未安装本项目所需依赖。**始终使用 `uav_path_planning` 环境**最稳。

### 3.4 压力测试

压力测试同样分离：

```bash
conda activate uav_path_planning

# 第 1 步：跑测试（输出 JSON + Markdown）
python -u run_stress_test.py --scales 10 100 1000 10000 --compare \
    --output ./output/stress

# 第 2 步：生成对比图表
python tools/visualize_stress.py --input ./output/stress
```

支持的标志：

| 标志 | 含义 |
|------|------|
| `--dispatch` | 调度模式（共享网格 + 跳过 risk/heading + 预微调） |
| `--compare` | 同时跑 FULL 和 DISPATCH，自动输出加速比 |
| `--scales` | 自定义规模列表（默认 10 100 1000 10000） |
| `--workers` | 并行线程数（默认 8） |
| `--max-iterations` | A\* 单任务最大迭代次数（默认 200000） |

### 3.5 输出目录约定

```
output/
├── single/                          # plan_single.py 输出
│   ├── planning_result_*.json
│   ├── environment.json
│   ├── routes/*.json
│   ├── waypoints_*.json
│   ├── report_*.md
│   └── visualizations/               # visualize.py 输出
│       ├── routes_3d.png
│       ├── routes_interactive.html
│       ├── risk_heatmap.png
│       └── route_metrics_bar.png
│
├── multi/                           # plan_multi.py 输出
│   ├── planning_result_*.json
│   ├── routes/*.json
│   ├── conflicts.json               # 残余冲突 + 协调建议（--conflict-check 或有冲突时）
│   ├── environment.json
│   ├── report_*.md
│   └── visualizations/
│
└── stress/                          # run_stress_test.py 输出
    ├── stress_results_full.json
    ├── stress_results_dispatch.json
    ├── stress_results.json
    ├── STRESS_TEST_RESULTS.md
    ├── STRESS_TEST_REPORT.md         # 详细报告
    ├── console.log
    └── visualizations/               # visualize_stress.py 输出
        ├── wall_clock.png
        ├── speedup.png
        ├── success_rate.png
        ├── throughput.png
        ├── full_vs_dispatch.png
        └── task_time_percentiles.png
```

---

### 3.6 作为接口调用（供其他系统集成）

除命令行脚本外，算法还以**两层接口**对外提供，供其他系统直接调用：

| 层 | 入口 | 适用场景 |
|----|------|----------|
| Python SDK | `import uav_planner` | 对方是 Python 项目 |
| REST HTTP | `python serve.py` | 对方是 Java / Go / 前端 / 其他语言 |

**方式 A：Python SDK**

```python
from uav_planner import plan_single

result = plan_single(
    start={"x": 500, "y": 500, "z": 100},
    goal={"x": 2500, "y": 2500, "z": 150},
)
print(result["status"])                       # planned
print(result["route"]["metrics"])             # 里程 / 时间 / 风险
print(result["validation"]["summary_text"])   # ✓ 合法：共 0 个问题...
```

**方式 B：REST 服务（局域网联调）**

```bash
python serve.py --host 0.0.0.0 --port 8000
```

启动后会打印可直接发给对接方的地址，对方浏览器打开 `http://<IP>:8000/docs`
即可在 Swagger 页面上直接试调用，无需写代码。

| 端点 | 说明 |
|------|------|
| `POST /api/v1/plan/single` | 单任务规划 |
| `POST /api/v1/plan/batch` | 多机批量规划（同步，建议 ≤500 任务） |
| `POST /api/v1/validate` | 校验外部给出的航线合法性 |
| `POST /api/v1/jobs` + `GET /api/v1/jobs/{id}` | 大批量异步作业 |
| `GET /docs` | Swagger 交互文档 |

> 完整接口说明（参数 / 返回结构 / 错误码 / 调用示例 / 常见问题）见
> [`docs/API.md`](docs/API.md)。

---

## 4. 项目结构

```
.
│  ══════════ 交付物：算法接口层 ══════════
├── uav_planner/                  # ★ Python SDK（对外接口）
│   ├── api.py                    #   plan_single / plan_batch / plan_from_dir / validate_route
│   ├── pipeline.py               #   规划编排（A* → 优化；并行 → 冲突避免）
│   ├── adapters.py               #   dict <-> dataclass 转换 + 入参校验
│   ├── serializer.py             #   dataclass -> 纯 JSON（含 numpy 标量降级）
│   └── errors.py                 #   PlanningError + 错误码
├── server/                       # ★ REST 服务（FastAPI）
│   ├── app.py                    #   路由 + 统一错误信封
│   ├── schemas.py                #   Pydantic 请求/响应模型
│   └── jobs.py                   #   异步作业注册表
├── serve.py                      # ★ REST 服务启动脚本（打印局域网地址）
├── src/                          # ★ 算法内核（展开见下）
├── docs/                         # ★ 对外文档（随交付提供）
│   └── API.md                    #   接口说明：接口/参数/返回/错误码/示例/FAQ
├── input/                        # ★ 示例数据（模拟文件）
│   ├── config.json               #   时空网格配置 (50×50×10×120)
│   ├── static_obstacles.json     #   12 个静态障碍物
│   ├── dynamic_obstacles.json    #   4 个动态障碍物
│   ├── aircraft_config.json      #   5 架飞行器（含扩展动力学字段）
│   └── tasks.json                #   5 个飞行任务（含需求约束）
├── tests/                        # ★ 测试（272 个用例，质量证据）
│   ├── conftest.py               #   共享 fixtures（含真实样例数据）
│   ├── test_sdk.py               #   Python SDK 契约（JSON 纯净性/错误码/多航点）
│   ├── test_api.py               #   REST 端点（含异步作业与错误信封）
│   └── ...                       #   其余 13 个测试文件见 §7.2 P9
├── pyproject.toml                # ★ 打包配置（pip install -e ".[server]"）
├── requirements.txt              # ★ 依赖清单
├── environment.yml               # ★ conda 环境复现配置
│
│  ══════════ 非交付物：开发工具与课题材料 ══════════
├── tools/                        # 命令行工具（不属于接口交付范围）
│   ├── plan_single.py            #   单机规划 CLI
│   ├── plan_multi.py             #   多机批量规划 CLI
│   ├── run_stress_test.py        #   压力测试（10/100/1000/10000）
│   ├── visualize.py              #   Demo 可视化
│   ├── visualize_stress.py       #   压测可视化
│   └── setup_env.sh              #   一键 conda 环境 + 运行脚本
├── internal/                     # 课题材料
│   ├── demand.md
│   └── 全空间无人体系建设项目一期_排期与基础.docx
├── output/                       # 规划产物（已 gitignore）
├── pytest.ini                    # pytest 配置
├── .github/workflows/ci.yml      # CI（单测 + 端到端 + 服务冒烟 + 可视化冒烟）
└── README.md                     # 本文档

```

**依赖方向**（整理后已单向，接口层可独立交付）：

```
tools/*.py  ──┐
              ├──→  uav_planner/  ──→  src/
纯接口调用方  ──┘
```

> 命令行工具**调用** SDK，而不是 SDK 反向依赖命令行脚本。
> 因此交付「仅接口层」时不会出现 import 失败。
> `tools/` 不属于交付范围——对方只需调用接口，不需要这些开发脚本。

### 4.1 `src/` 算法内核

```
├── src/
│   ├── models/                   # 数据模型
│   │   ├── space.py              # 时空网格 + 禁飞区 + 空域边界
│   │   ├── obstacles.py          # 静态/动态障碍物
│   │   ├── aircraft.py           # 飞行器（物理/性能/续航/作业/安全）+ 任务需求
│   │   └── task.py               # 轨迹点 + 规划结果 + PlanningExplanation
│   ├── grid/                     # ★ 网格（任务书 §十一 要求）
│   │   ├── __init__.py           #   导出 SpaceTimeGrid / CostGrid
│   │   └── cost.py               #   通行代价（CBD / 居民区 / 风场）
│   ├── planners/                 # 规划算法
│   │   ├── spacetime_astar.py    # 多维时空 A*（3.25 核心）
│   │   ├── parallel_planner.py   # 大规模并行 + 时空占用表 + 冲突解决
│   │   └── route_optimizer.py    # 多目标航线优化（3.26）
│   ├── validator/                # ★ 验证模块（任务书 §六）
│   │   ├── __init__.py
│   │   └── path_validator.py     #   连续性/障碍/性能/高度/多机冲突
│   ├── data_fusion/
│   │   └── obstacle_fusion.py    # 多源障碍物数据融合
│   ├── io/
│   │   ├── input_loader.py       # JSON 输入加载（支持扩展字段）
│   │   └── output_writer.py      # JSON / Markdown 输出
│   └── visualization/
│       └── viewer.py             # 3D 可视化 + 风险热力图
│
├── docs/                         # ★ 对外文档（随交付提供）
│   └── API.md                    #   接口说明：接口/参数/返回/错误码/示例/FAQ
│
├── internal/                     # 课题材料与内部资料（不交付）
│   ├── demand.md
│   ├── *.docx                    # 需求与排期材料
│   └── docs/
│       ├── DELIVERY.md           # 交付清单（交什么 / 不交什么）
│       ├── DEPLOYMENT.md         # 部署运维（Docker / 边缘鉴权）
│       └── archive/              # 旧文档与算法设计说明
│
└── output/                       # 规划输出
    ├── single/                   # plan_single.py 输出
    │   ├── planning_result_*.json
    │   ├── environment.json
    │   ├── routes/*.json
    │   ├── waypoints_*.json
    │   ├── validation_*.json     # ★ 合法性验证结果
    │   ├── report_*.md
    │   └── visualizations/
    ├── multi/                    # plan_multi.py 输出
    │   ├── planning_result_*.json
    │   ├── environment.json
    │   ├── routes/*.json
    │   ├── conflicts.json        # ★ 冲突详情 + 协调建议
    │   ├── validation.json       # ★ 合法性验证结果
    │   ├── report_*.md
    │   └── visualizations/
    └── stress/                   # run_stress_test.py 输出
        ├── stress_results_full.json
        ├── stress_results_dispatch.json
        ├── stress_results.json
        ├── STRESS_TEST_RESULTS.md
        ├── STRESS_TEST_REPORT.md  # 详细压力测试报告
        └── visualizations/
```

---

## 5. 核心算法

### 5.1 多维时空 A*（3.25）

在 `(x, y, z, t)` 四维网格中搜索无碰撞路径：

- **状态**：`(ix, iy, iz, it)` 网格坐标
- **动作**：6 邻域空间移动 + 等待动作（每次推进 1 个时间步）
- **启发**：曼哈顿网格距离 × `dt`（可采纳）
- **代价**：时间步长 + 微小距离惩罚（打破平局）
- **避障**：静态障碍（全时段占用）+ 动态障碍（时段占用）

关键文件：[src/planners/spacetime_astar.py](src/planners/spacetime_astar.py)

#### 飞行器包线约束（2026-10 新增）

A* 现在直接接受飞行器性能参数，越界路径在搜索阶段就被排除，而不是"先规划出
非法航路、再由验证模块事后报错"：

| 参数 | 作用 | 实现 |
|---|---|---|
| `min_altitude` / `max_altitude` | 把搜索的 `iz` 范围限制在飞行器工作包线内 | `_resolve_altitude_band()` |
| `max_climb_rate` / `max_descent_rate` | 垂直位移按速率折算所需时间步 | `_get_neighbors()` |

两点需要注意：

- **阶梯式垂直移动**：栅格 `dz=50m` 而 `dt=10s`，单时间步垂直位移隐含 5 m/s，
  高于多数机型的升降率。因此一次 `dz` 位移会按
  `ceil(dz / (rate × dt))` **跨多个时间步**完成（阶梯），并沿途检查碰撞。
- **起终点预检**：起终点超出升限时直接失败并给出可读原因（如
  `终点高度 220.0m 超过飞行器最高高度 200.0m`），而不是耗尽 200k 次迭代后
  报"未找到可行路径"。

#### 通行代价层（2026-10 接入）

`CostGrid` 与占用网格**正交**：occupancy 决定"能不能走"，cost 决定"走起来多费力"。

- 默认用 `build_clearance_cost_grid` 按障碍物几何自动生成：每个静态障碍物包围盒
  外扩 `clearance`（默认 200m），该缓冲带通行代价 ×3（`--clearance` / `--clearance-cost` 可调）
- 效果实测：航线最近障碍距离从贴障提升到 ~65m 以上（与 3.26 的 risk 目标同向）
- **启发函数按 `cost_grid.min_cost` 缩放**，保证任意代价倍率下 A* 仍可采纳、仍得最优解
  （`CostGrid` 增量维护 `min_cost`，无全量扫描开销）
- 关闭：`--no-cost-grid` / `ParallelPlanner(use_cost_grid=False)`

### 5.2 大规模并行规划（3.25）

`ParallelPlanner` 类提供：

- 基于 `ThreadPoolExecutor` 的多线程并行
- 按任务优先级排序（高优先级先规划）
- **共享只读 `SpaceTimeGrid`**（P2 优化）+ 共享只读 `CostGrid`，消除每任务重建
- 多航点任务分段规划后拼接
- **多机冲突避免（§五.4）**：`plan_batch()` 默认 `conflict_avoidance=True`，
  流程 = `detect_conflicts_occupancy()` 检测 → 每对选优先级低者 →
  `to_dense_mask()` 生成禁飞掩码 → `SpaceTimeAStar` 带掩码重规划 → 串行更新占用表，
  最多 3 轮；关闭时（`--no-avoid-conflicts`）仍**如实报告**检出的冲突
- **时空占用表冲突检测**：`detect_conflicts_occupancy()` 按航线增量登记，
  避免 O(n²·m²) 重复比对；冲突条目带真实 `route_id` + `aircraft_id`
- **冲突协调建议**：`resolve_conflicts()` 对每条冲突给出"延迟起飞"或"高度调整 +1 层"
  （按对方航线真实通过时刻反推，而非固定步长）
- 已通过 10 / 100 / 1000 / 10000 规模压测，**各规模残余冲突均为 0**

### 5.3 多目标航线优化（3.26）

四目标加权优化：

| 目标 | 含义 | 默认权重区间 |
|------|------|---------------|
| distance | 总航程最短 | 0.2-0.4 |
| time | 总飞行时间最短 | 0.2-0.6 |
| risk | 距障碍物风险最低 | 0.1-0.5 |
| energy | 估算能耗最低 | 0.1-0.2 |

**优化流程**：
1. 时空 A* 生成初始无碰撞路径（受飞行器包线约束）
2. 三次样条插值平滑
3. 性能约束修正（速度/爬升率/转弯率）
4. 碰撞检测：若平滑后穿模则回退到原始 A* 路径
5. **生成结构化规划解释**并挂到 `route.explanation`，随 JSON 输出（`explanation` 字段）
6. 航点提取（按最小间距 + 航向变化）

**规划解释**（任务书 §六）包含：综合得分、四个目标的加权贡献度（之和 = 综合得分）、
性能裕度（速度/爬升/下降/转弯的极限占用比例）、规避过的障碍物数量、以及需要复核的提示
（如"样条回退"、"转弯率接近极限"、"飞行时间已达续航 80%"）。

### 5.4 路径合法性验证（3.26）

`PathValidator` 覆盖 7 个维度：**连续性 / 静态障碍 / 动态障碍 / 高度 / 飞行性能 / 时空占用 / 多机间隔**。

现已接入两个入口脚本：

- `tools/plan_single.py`：规划后自动校验（`--no-validate` 关闭），产出 `validation_<task>.json`
- `tools/plan_multi.py`：`--validate` 触发，含多机两两间隔检查，产出 `validation.json`
- 存在 ERROR 级问题时**退出码 2**，便于 CI 拦截

#### ⚠️ 接入验证模块后暴露并修复的 3 个真实缺陷

验证模块此前从未运行，因此以下缺陷一直静默存在——规划器会为不满足约束的任务
输出 `status="planned"` 并报告"成功"：

| # | 缺陷 | 现象 | 修复 |
|---|---|---|---|
| 1 | **A* 不知道飞行器高度包线** | `task-003` 目标 220m、机型升限 200m，仍规划出 225m 的"成功"航路；`plan_multi` 首次接验证时报 **47 个 ERROR** | `SpaceTimeAStar` 接入 `min/max_altitude` + 起终点预检 |
| 2 | **A* 不检查爬升/下降率** | 栅格隐含 5 m/s 垂直速度，超过 `uav-005` 的 3 m/s 限制 | 接入 `max_climb_rate`/`max_descent_rate`，并实现**阶梯式垂直移动**避免"完全无法改变高度" |
| 3 | **需求校验不核对航点实际高度** | `task-003`/`task-004` 的航点高度超出对应机型升限，直到 A* 失败才暴露，浪费规划开销 | `FlightTask.check_aircraft_compatibility()` 增加航点高度逐条核对 |

> 另修复两个既有缺陷：障碍物 JSON 的**裸数组格式**在所有 loader 中都会抛
> `AttributeError`（文档声称支持）；`ConflictResolver` 的高度调整分支因
> `suggest_delay` 总被截断到 `max_delay_s` 而**永远不可达**。

> 净效果：demo 场景由"5/5 规划成功"（其中 2 条实际违规）变为"**2 个任务在需求
> 校验阶段被识别为不可行、3 个合规任务 100% 规划成功且零 ERROR**"。压测成功率数字
> 因此相应变化，见 §4——**旧的成功率包含了违反飞行器约束的航路，不可比**。

---

## 6. 输入输出格式

### 6.1 输入（`input/` 目录）

| 文件 | 内容 |
|------|------|
| `config.json` | 时空网格配置 |
| `static_obstacles.json` | 静态障碍物（建筑物/塔/地形） |
| `dynamic_obstacles.json` | 动态障碍物（带轨迹点） |
| `aircraft_config.json` | 飞行器动力学参数（含物理/性能/续航/安全） |
| `tasks.json` | 飞行任务（航点 + 需求约束 + 优化权重） |

#### 飞行器动力学字段（`aircraft_config.json`）

```json
{
  "id": "uav-001",
  "model": "DJI-Matrice-300",
  "category": "heavy_payload",
  "physical": {
    "weight_kg": 6.3, "max_takeoff_weight_kg": 9.0, "payload_capacity_kg": 2.7,
    "wingspan_m": 0.81, "length_m": 0.81, "width_m": 0.81, "height_m": 0.43,
    "rotor_radius_m": 0.43, "diagonal_m": 1.05
  },
  "performance": {
    "max_speed_mps": 18.0, "max_climb_rate_mps": 6.0, "max_descent_rate_mps": 4.0,
    "max_turn_rate_dps": 60.0, "max_acceleration_mps2": 3.0, "max_deceleration_mps2": 2.5
  },
  "endurance": {
    "max_flight_time_s": 1800.0, "max_range_m": 15000.0,
    "battery_capacity_wh": 274.0, "cruise_power_w": 850.0, "hover_power_w": 600.0
  },
  "operational": {
    "min_altitude_m": 10.0, "max_altitude_m": 300.0,
    "min_operating_temp_c": -20.0, "max_operating_temp_c": 50.0,
    "max_wind_resistance_mps": 12.0, "ip_rating": "IP45"
  },
  "safety": {
    "rotor_radius_m": 0.43, "safety_margin_m": 5.0,
    "geofence_compliant": true, "return_to_home_altitude_m": 50.0
  }
}
```

**字段分类**：
- `physical`：自重、最大起飞重量、载荷容量、几何尺寸
- `performance`：最大速度、爬升/下降率、转弯率、加速度
- `endurance`：续航时间、最大航程、电池容量、巡航/悬停功率
- `operational`：高度限制、温度、抗风、IP 防护等级
- `safety`：旋翼半径、安全裕度、地理围栏合规、返航高度

#### 任务需求字段（`tasks.json`）

```json
{
  "id": "task-001",
  "requirements": {
      "payload_kg": 1.5,
      "required_endurance_s": 300.0,
      "required_range_m": 5000.0,
      "required_max_speed_mps": 10.0,
      "min_ceiling_m": 100.0,
      "delivery_window": [0, 120],
      "priority_level": 5,
      "service_type": "delivery"
    }
}
```

**`tools/plan_multi.py` 启动时自动校验**：每个任务的飞行器性能是否满足 `requirements`（载荷 ≤ 容量、续航 ≤ 上限、速度 ≤ 上限、高度 ≤ 上限）。

**详细字段定义**：
- 📘 **接口说明文档**：[`docs/API.md`](docs/API.md)（接口列表、参数、返回结构、错误码、调用示例、FAQ）
- 📦 命令行写盘文件的字段：[`internal/docs/archive/INPUT_OUTPUT_GUIDE.md`](internal/docs/archive/INPUT_OUTPUT_GUIDE.md)（已废弃，仅作历史参考）

### 6.2 输出（`output/` 目录）

| 文件 | 内容 |
|------|------|
| `planning_result_*.json` | 完整规划结果（轨迹 + 指标 + 算法信息 + **结构化规划解释**） |
| `waypoints_*.json` | 航点指令（带 `takeoff`/`fly`/`land` 动作） |
| `validation_*.json` / `validation.json` | **合法性验证结果**（7 维问题清单 + 严重度 + 数据质量） |
| `report_*.md` | Markdown 格式规划报告（含验证章节与数据质量章节） |
| `environment.json` | 环境快照（供 `tools/visualize.py` 复现障碍物） |
| `conflicts.json` | **残余冲突** + **协调建议**（`delay` / `altitude_shift`）；传 `--conflict-check` 或存在残余冲突时生成 |
| `visualizations/*.png\|html` | 3D 可视化、热力图、对比柱状图 |
| `stress/STRESS_TEST_RESULTS.md` | 压力测试 Markdown 报告（自动生成） |
| `stress/STRESS_TEST_REPORT.md` | 压力测试详细报告（结论+建议） |

---

## 7. 当前问题与改进建议（按优先级）

### 7.1 🔴 高优先级（影响正确性或大规模性能）

#### P0. 多机冲突避免只做"事后检测"，规划时不避让 ✅ **已修复**

**位置**：[src/planners/occupancy.py](src/planners/occupancy.py) + [src/planners/parallel_planner.py](src/planners/parallel_planner.py) + [src/planners/spacetime_astar.py](src/planners/spacetime_astar.py)

**修复前**（任务书 §五.4 未真正实现）：

- A\* 从不消费 `SpatioTemporalOccupancyTable`，规划阶段完全不知道别的飞机在哪；
- `detect_conflicts` 只做事后检查，检出冲突仍返回 `planned`、`tools/plan_multi.py` 仍 `exit 0`；
- 实测对向两机在同一点同一时刻**距离 0.0 m**（要求 30 m）；随机任务安全率
  N=100 → 82.8%、N=400 → **53.2%**（46.8% 航线涉碰撞），而这些航线在报告里
  全都记作"成功"。

**修复内容**：

1. `SpatioTemporalOccupancyTable` 抽到 [src/planners/occupancy.py](src/planners/occupancy.py)（避免 A\* ↔ planner 循环导入），
   新增 `to_dense_mask()` / `clearance_cells()`，把占用表变成与 `SpaceTimeGrid` 同构的
   4D bool 数组；
2. `SpaceTimeAStar` 消费 `occupancy_mask`：`_get_neighbors` 里对阶梯中间格、碰撞格、
   等待动作三处做禁飞检查；起点被占提前失败并给出"建议延迟起飞"原因；
3. `ParallelPlanner.plan_batch()` 默认 `conflict_avoidance=True`：
   检测 → 每对选优先级低者 → 带掩码重规划 → 串行更新占用表 → 最多 3 轮；
4. 关闭避让时**仍然如实检测并报告** `result.conflicts`（"未检查" ≠ "无冲突"）；
5. `tools/plan_multi.py` 残余冲突 → `sys.exit(2)`；`--conflict-check` 报告 `result.conflicts` 全量
   （旧的 `--conflict-sample` 前缀切片 `planned[:200]` 改为默认全量）；
6. `PathValidator.validate_multi_routes` 在有 `config` 时改走线性复杂度的占用表路径，
   检测口径与规划侧一致（同一 `time_window=5s`、同一 `min_separation`）。

**实测**（`min_separation=30m`，`ParallelPlanner` 默认 `max_iterations=2,000,000`、
`workers=8`；**每一行的"开启/关闭"两列使用完全相同的任务集**，是严格的 A/B 对照。
残余冲突 / 安全率）：

| 规模 | 开启避让 | 关闭避让 |
|------|----------|----------|
| 对向穿越（4~6 条车道） | **0 对 / 100%** | 0.0 m 重叠 / 0% |
| N=40 | 0 对 / 100% | 1 对 / 94.4% |
| N=100 | 0 对 / 100% | 13 对 / 84.2% |
| N=200 | 0 对 / 100% | 85 对 / 68.5% |
| N=400 | 0 对 / 100% | 264 对 / 50.1% |
| N=1000 | 0 对 / 100% | 1,431 对 / 28.7% |

墙钟开销可忽略（同配置 A/B：N=400 → 158.1s vs 163.0s，N=1000 → 488.8s vs 459.3s，
约 +6%）。§8.2 的正式压测另用 `--max-iterations 200000` 与 `seed + N`，两处数字
**不可直接互相比较**，但结论一致：安全率 100%，墙钟开销个位数百分比。

**顺带修掉的真 bug**（曾导致避让空转）：`find_conflicts` 的时间步区间没按
`step_to_time(it) = t_min + (it+0.5)*dt`（**步中心**）反解，把窗外一整步纳入，
报出 `distance=0` 的假冲突；避让循环反复重规划却"路径未变"（那格本来就没被占用）。
现改为先按闭区间反解出**超集**，再用 `abs(pa.t - step_to_time(it)) <= time_window`
显式裁剪，与 `PathValidator` 朴素路径的口径一致。回归用例见
`tests/test_conflict_avoidance_e2e.py::TestTimeWindowSemantics`。

**已知剩余缺口**：`min_separation` 通过 `clearance_cells()` 折算成膨胀格数，
`cell_size > min_separation` 时折算为 0（当前 dx=dy=100、dz=50、间隔 30m → 0），
真正起作用的是"同格同刻禁飞"；若未来把间隔要求提到 > 50m，需要同时验证膨胀层数。

#### P1. A* 起点/终点超出时间窗口被静默钳制

**位置**：[src/models/space.py:74-76](src/models/space.py#L74)

`time_to_step` 使用 `np.clip` 把时间裁剪到 `[0, nt-1]`，当下游传入 `start_time > t_max` 时不会报错，而是被当作 `nt-1`。建议在 `to_grid`/`time_to_step` 增加越界警告或显式抛错，避免静默失败。

#### P2. 每个任务重复分配 3 MB 时空网格 ✅ **已修复**

**位置**：[src/planners/spacetime_astar.py](src/planners/spacetime_astar.py) + [src/planners/parallel_planner.py](src/planners/parallel_planner.py)

`ParallelPlanner.__init__` 现在**预先构建一份只读 `SpaceTimeGrid`**，所有子规划器共享同一个引用。10000 任务从原来的 30 GB 分配降至**单次 3 MB**。

实测收益（与原始 13 分钟对比）：**N=10000 墙钟从 789s 降至 437s**（2026-10-01 基线，详见 [`output/stress/STRESS_TEST_REPORT.md`](output/stress/STRESS_TEST_REPORT.md) §6）。

> **注**：加入包线约束与代价层后，N=10000 为 1,560s。共享网格本身的收益（1.81×）不变，
> 增量来自约束收紧，见 §8.2「约束收紧的代价」分解。代价层同样共享（N=10000 不再
> 重复分配 12 MB float32 数组）。

#### P3. 失败任务没有重试 ✅ **已修复（但实测收益有限）**

`ParallelPlanner._plan_one` 在 `dispatch=True` 时，会在 A* 之前**预先检查起点/终点是否落在障碍物内**；如果在，则调用 `_snap_to_free` 自动微调到最近空闲格（搜索半径 2 格）。

> **注意 1**：最初实现为"失败后重试"，但实测发现对 max_iter 超时类失败重试是浪费（同样的 A* 还是会超时），改成"预校验"后更稳健。
>
> **注意 2**：2026-10-08 压测显示，FULL 与 DISPATCH 成功率**完全相同**（各规模逐条一致），
> 说明起点预校验在随机任务集上**未实际救回任何任务**——随机任务中"起点恰在障碍物边界"
> 的占比本身就不高。DISPATCH 的实际收益仅来自跳过 risk/heading 计算（约 3%）。

#### P4. `route_optimizer.py` 中存在奇怪的多重赋值 ✅ **已修复**

**位置**：[src/planners/route_optimizer.py:229](src/planners/route_optimizer.py#L229)

```python
t = task_departure = trajectory[0].t + (s_new[i] * total_time)
```

`task_departure` 变量没有其他用途（推断是历史调试残留），命名误导且无意义。建议简化为：

```python
t = trajectory[0].t + s_new[i] * total_time
```

#### P5. 路径平滑后时间分配不一致

**位置**：[src/planners/route_optimizer.py:222-225](src/planners/route_optimizer.py#L222)

```python
avg_speed = self.performance.max_speed * 0.7
total_dist = dists[-1]
total_time = total_dist / avg_speed
```

使用**原始 A* 路径的总弧长**和**70% 巡航速度**估算总时间，但实际平滑后路径长度会变化（样条可能更短也可能更长）。建议先用样条重算弧长，再分配时间，避免 `total_time` 与实际速度不符。

### 7.2 🟡 中优先级（工程化 & 可维护性）

#### P6. 死代码与误导性命名 ✅ **已修复**

- ~~`[src/planners/parallel_planner.py:24-76](src/planners/parallel_planner.py#L24)`~~ `_plan_single_task` 函数已删除（从未被调用，且其内部 `obstacle_set = ObstacleSet()` 为空会导致完全忽略障碍物，潜在 bug）
- ~~`_plan_multiprocess`~~ 已删除（实际只是回退到 `_plan_multithread`，误导命名）
- `use_processes=False` 参数已删除
- ~~[src/visualization/viewer.py:187](src/visualization/viewer.py#L187)`~~ `generate_plotly_html` 的 `animate=True` 未使用参数已删除
- 内部缩进 [src/visualization/viewer.py:204](src/visualization/viewer.py#L204)`from plotly.subplots import make_subplots` 未使用的 import 已删除

#### P7. 可视化标题中英混用回退

**位置**：[src/visualization/viewer.py:141](src/visualization/viewer.py#L141)

```python
ax.set_title(title if title.isascii() else "Low-Altitude UAV Path Planning 3D Visualization", ...)
```

当 `title` 包含 CJK 字符时直接被英文替换，丢失原始语义。建议改为设置中文字体（`matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Arial Unicode MS']`）或保留原文 + 加英文副标题。

#### P8. 风险热力图计算开销巨大

**位置**：[src/visualization/viewer.py:340-358](src/visualization/viewer.py#L340)

`plot_risk_heatmap` 中双重 `for i in range(nx): for j in range(ny):` 嵌套循环（默认 100×100 = 10000 次），且每次内层循环遍历所有静态障碍物。当障碍物数量上升时该函数会变成 O(nx·ny·n_obs)。建议向量化：用 `np.meshgrid` 一次性算出所有网格点到每个障碍物的距离，再 `np.min`/`np.max` 归并。

#### P9. ~~测试目录为空~~ ✅ 已解决

**位置**：[tests/](tests/)

原状态：`tests/` 目录存在但无任何测试文件。现已补充 **272 个用例**（13 个测试文件 + `conftest.py`），覆盖：

| 测试文件 | 用例数 | 覆盖内容 |
|---|---:|---|
| `test_space.py` | 16 | 时空网格坐标换算、占用网格读写 |
| `test_obstacles.py` | 16 | 静态/动态障碍、AABB 距离、轨迹插值 |
| `test_aircraft.py` | 17 | 性能校验、能耗估算、任务兼容性 |
| `test_astar.py` | 7 | 4D 时空 A*：可达性、避障、限速、迭代截断 |
| `test_validator.py` | 9 | PathValidator 连续性/障碍/性能/高度检查 |
| `test_cost_grid.py` | 25 | CostGrid 读写、区域代价、净空代价、**A\* 代价层不破坏最优性**、时不变 3D 视图 |
| `test_aircraft_envelope.py` | 19 | **高度包线 + 升降率约束**、阶梯式垂直移动、航点兼容性 |
| `test_occupancy_conflicts.py` | 26 | 时空占用表、冲突检测、冲突协调建议、代价层共享 |
| `test_conflict_avoidance_e2e.py` | 16 | **端到端多机冲突避免**：对向穿越残余冲突为 0、关掉避让仍如实报冲突、时间窗语义回归、独立交叉验证、**进程级退出码 0/2** |
| `test_data_fusion.py` | 21 | 多源融合：去重、优先级、容错、质量报告、裸数组格式 |
| `test_explanation_output.py` | 24 | PlanningExplanation 填充、避障计数、报告/验证结果序列化 |
| `test_pipeline_wiring.py` | 21 | **主链路接线回归**：入口确实调用了各模块，规划结果零 ERROR |
| `test_sdk.py` | 35 | **Python SDK 契约**：返回值为纯 JSON（无 numpy 标量 / NaN）、错误码稳定、多航点真正生效、外部航线校验 |
| `test_api.py` | 20 | **REST 端点**：全部接口、统一错误信封与状态码映射、异步作业提交轮询、OpenAPI 可生成 |
| **合计** | **272** | |

运行：`./tools/setup_env.sh test` 或 `python -m pytest tests/ -v`
CI：[.github/workflows/ci.yml](.github/workflows/ci.yml)（push/PR 自动跑单测 + 双入口端到端 + 可视化冒烟）

### 7.3 🟢 低优先级（建议打磨）

#### P10. 文档同步问题

`generate_waypoints` 默认 `min_spacing=50.0`，但 [internal/docs/archive/algorithm_design.md](internal/docs/archive/algorithm_design.md) 和 [README 历史版本](README.md) 都写 80m。建议要么把代码默认改为 80m，要么更新文档。

#### P11. 日志与监控缺失

当前 `print(...)` 直接打到 stdout，没有日志框架（`logging`）和结构化字段。生产化建议：

```python
import logging
logger = logging.getLogger(__name__)
logger.info("task_planned", extra={"task_id": ..., "time_ms": ...})
```

#### P12. 飞行器间冲突检测是 O(n²·m²) ✅ **已修复**

**位置**：[src/planners/occupancy.py:125](src/planners/occupancy.py#L125)（主路径）+ [src/planners/parallel_planner.py:411](src/planners/parallel_planner.py#L411)（保留的朴素实现）

主路径 `find_conflicts` 原先对**每个轨迹点遍历全表**（O(|occupancy|)），万级航线下退化成 O(n²·m²)。现改为：按"中心距 < `min_separation` 的格必落在 `r = floor(min_separation / 最小格长) + 1` 邻域内"只查局部，复杂度降到 **O(|轨迹点| × 时间窗步数 × (2r+1)³)**，与全表扫描结果一致（由 `test_occupancy_conflicts.py` 对照断言守护）。

> 仍保留 O(n²) 的 `detect_conflicts` 两两比对，用作小规模参考实现与交叉验证；
> `tools/plan_multi.py` / `tools/run_stress_test.py` 均已不再调用它。

#### P13. README 缺 3D 网格规模计算说明

config.json `dt=10` 与 `SpaceTimeConfig` 默认 `dt=5` 不同，加载后网格时间维是 120 而非 240。建议在 README 明确说明当前默认网格是 `50×50×10×120 = 3M cells`。

#### P14. 通行代价层只读静态障碍，不含动态障碍

**位置**：[src/grid/cost.py](src/grid/cost.py) `build_clearance_cost_grid()`

代价层只遍历 `obstacle_set.static_obstacles` 做净空惩罚；动态障碍**不进代价层**。

> **不是正确性问题**：A\* 的碰撞判定走 `SpaceTimeGrid`，动态障碍按时间戳参与
> `is_blocked()`，飞机不会撞上它们。缺的只是"绕开动态障碍更划算"的**路径质量**偏好
> ——当前 A\* 会在代价相同的前提下径直穿过动态障碍的时间窗（只保证不相撞，不保证好看）。
>
> **没有顺手修的原因**：代价层刻意做成**时不变 3D 数组**（`lookup_array()` 直取 +
> 缓存友好），这是 §8.2「代价层自身开销已优化」里 154.6s → 127.7s 那次优化的基础。改成 4D 时变代价
> 会直接摧毁该优化。可行的折中是"把动态障碍的扫掠包络（全时段最大范围）并入 3D
> 静态代价"，会略微保守但保持 3D 结构。

---

## 8. 性能指标

> **测试环境**：macOS (M-series), Python 3.11.16, 8 工作线程, A\* 单任务最大 50,000 次迭代
> **测试时间**：2026-10-01
> **测试命令**：`python tools/run_stress_test.py --scales 10 100 1000 10000 --workers 8 --max-iterations 50000 --compare`

### 8.1 Demo 场景（5 任务）

| 指标 | 数值 |
|------|------|
| 网格规模 | 50×50×10×120 (3M cells) |
| 5 任务规划 | ~91 ms |
| 单任务 A\*（中位数） | ~91 ms |
| 规划成功率 | 合规任务 100%（3/3）；另 2 个任务因航点高度超出机型升限在需求校验阶段被识别 |

### 8.2 规模压测（FULL 模式 / DISPATCH 模式）

> **测试时间**：2026-10-08（飞行器包线约束 + 通行代价层接入后重测）
> **命令**：`python tools/run_stress_test.py --scales 10 100 1000 10000 --seed 20260906 --workers 8 --compare`

| 规模 N | 成功 | 成功率 | FULL 墙钟 | DISPATCH 墙钟 | FULL 吞吐 | DISPATCH 吞吐 |
|--------|------|--------|------------|-----------------|------------|---------------|
| 10     | 10/10   | 100.0% | 0.4 s   | 0.2 s   | 28.1/s   | 43.2/s   |
| 100    | 89/100  | 89.0%  | 19.1 s  | 18.0 s  | 5.2/s    | 5.5/s    |
| 1,000  | 920/1000 | 92.0% | 129.0 s | 122.5 s | 7.8/s    | 8.2/s    |
| 10,000 | 9,095/10,000 | 90.95% | 1,560.0 s (26.0 min) | 1,512.8 s (25.2 min) | 6.4/s | 6.6/s |

**累计 11,110 条任务 × 2 模式**，整体成功率 **91.04%**，两模式结果完全一致。

> ⚠️ **与 2026-10-01 旧数据的可比性**：旧数据为 N=10000 → 437.31s / 91.43%。
> 新数据慢 **3.6×**。这不是性能退化，而是**约束收紧的必然代价**——见下方分解。
> 旧的成功率里包含了违反飞行器高度包线与升降率限制的航路，因此两个数字**不可直接比较**。

**耗时增加分解**（N=1000 实测对照）：

| 配置 | 墙钟 | 平均最近障碍距离 | 说明 |
|------|------|----------------|------|
| 无代价层（仅包线约束） | 101.6 s | 138.1 m | 基线 |
| 开启代价层（默认） | 127.7 s | 196.8 m | **+26% 耗时，换 +59m 障碍净空** |

| 约束 | 代价来源 |
|------|----------|
| 高度包线 `min/max_altitude` | 搜索的 `iz` 范围收窄，启发式引导在垂直方向变弱 |
| 升降率 `climb/descent_rate` | **阶梯式垂直移动**使一次 `dz` 位移跨多个时间步，路径步数与迭代数增加 |
| 通行代价层 | 内循环每扩展多 ~7 次代价查表（已用 3D 扁平视图 + 数组直取优化，见下方「代价层自身开销已优化」） |

> 代价层自身开销已优化：165.0 s → 154.6 s（3D 缓存友好视图）→ **127.7 s**
> （`lookup_array()` 直取数组，省掉热循环里的 Python 方法调用），合计 **1.29×**。

**失败原因**（均为搜索受限或起点被占，非非法航路）：

- A\* 达到迭代上限仍未找到可行路径（受升降率阶梯与高度包线影响，搜索量增大）
- 起点落在建筑物包围盒内（栅格量化后被判为占用）
- 多航点中间航段失败

### 8.3 单任务耗时分布（DISPATCH 模式）

| 规模 N | 最小 | P50 | P95 | P99 | 最大 |
|--------|------|------|------|------|------|
| 10     | 1.18 ms | 83.7 ms | 172 ms | 200 ms | 208 ms |
| 100    | 0.01 ms | 94.4 ms | 11,451 ms | 17,219 ms | 17,275 ms |
| 1,000  | 0.01 ms | 93.3 ms | 4,364 ms | 20,484 ms | 22,097 ms |
| 10,000 | 0.01 ms | 91.0 ms | 7,221 ms | 21,403 ms | 24,708 ms |

（FULL 模式 P50 为 82-135 ms，与 DISPATCH 同量级）

**结论**：**P50 中位数约 91 ms**（一半任务在此之内完成），但 **P95/P99 达到 4-22 秒级**。
尾延迟比中位数高 2-3 个数量级，**墙钟时间几乎完全被尾延迟拖慢**——这是当前最主要的性能瓶颈，
也是 §7.1 P3 建议 numba JIT 的目标函数。

### 8.4 P2 优化效果（共享时空网格）

**关键性能优化**：将 [SpaceTimeGrid](src/models/space.py) 从"每任务重建 3MB"改为"全任务共用 1 份"：

| 规模 N | 优化前墙钟 | 优化后 (FULL) | 加速比 |
|--------|------------|---------------|--------|
| 1,000  | 67.8 s     | 36.10 s       | **1.88×** |
| 10,000 | 789.6 s    | 437.31 s      | **1.81×** |

**N=10000 总耗时从 13.2 分钟降至 7.3 分钟**。内存分配从 30 GB 降至 3 MB（减少 99.99%）。

> 该对比为 2026-10-01 基线（尚未加入包线约束与代价层），保留用于量化共享网格本身的价值。

### 8.5 DISPATCH vs FULL 对比

| 规模 N | FULL (s) | DISPATCH (s) | 加速比 | FULL 成功率 | DISPATCH 成功率 |
|--------|----------|---------------|---------|--------------|------------------|
| 10     | 0.4     | 0.2          | 1.75×  | 100.0%      | 100.0%          |
| 100    | 19.1    | 18.0         | 1.06×  | 89.0%       | 89.0%           |
| 1,000  | 129.0   | 122.5        | 1.05×  | 92.0%       | 92.0%           |
| 10,000 | 1,560.0 | 1,512.8      | 1.03×  | 90.95%      | 90.95%          |

> **DISPATCH 模式**：跳过 risk/heading/metrics 等附加计算，并在规划前对起点/终点**预校验**（自动微调到最近空闲格）。
> 实测提速 1.03-1.06×（N≥100），主要价值是**跳过 risk 计算可降低 I/O 与报告生成开销**。
> 两模式成功率完全相同——随机任务中"起点恰在障碍物边界"的占比本身就不高，
> 因此起点预校验基本没有救回任务。

### 8.6 可视化产物

详细对比图已生成在 [`output/stress/visualizations/`](output/stress/visualizations/)：

| 文件 | 内容 |
|------|------|
| `wall_clock.png` | 墙钟时间 vs 规模 N（log-log） |
| `speedup.png` | DISPATCH 加速比柱状图 |
| `success_rate.png` | FULL vs DISPATCH 成功率对比 |
| `throughput.png` | 吞吐 vs 规模 N |
| `full_vs_dispatch.png` | 4 子图综合对比（墙钟/加速比/成功率/吞吐） |
| `task_time_percentiles.png` | 单任务耗时 P50/P95/P99/max 分布 |

> **运行命令**：`python tools/run_stress_test.py --compare --output ./output/stress`
> **可视化**：`python tools/visualize_stress.py --input ./output/stress`

---

## 9. 项目交付清单

- [x] 完整源码（4 个规划模块 + 4 个数据模型 + grid/validator + 可视化 + I/O）
- [x] Demo 数据集（12 静态 + 4 动态障碍物 + 5 飞行器 + 5 任务，含扩展动力学字段）
- [x] 单机规划脚本 `tools/plan_single.py`
- [x] 多机批量脚本 `tools/plan_multi.py`
- [x] 压力测试脚本 `tools/run_stress_test.py`
- [x] 可视化脚本 `tools/visualize.py` / `tools/visualize_stress.py`
- [x] 算法设计文档 `internal/docs/archive/algorithm_design.md`（算法原理仍有效，仅 I/O 部分被接口文档取代）
- [x] **接口说明文档** [`docs/API.md`](docs/API.md)（接口列表 / 参数 / 返回结构 / 错误码 / 调用示例 / 常见问题）
- [x] **Python SDK** `uav_planner/`（`plan_single` / `plan_batch` / `plan_from_dir` / `validate_route`）
- [x] **REST 服务** `server/` + `serve.py`（Swagger 文档、统一错误信封、异步作业）
- [x] 命令行 I/O 字段说明 `internal/docs/archive/INPUT_OUTPUT_GUIDE.md`（已废弃）
- [x] README（本文档）
- [x] conda 环境 `environment.yml` + `setup_env.sh`
- [x] 单元测试（见 §7.2 P9）
- [x] CI 配置（[.github/workflows/ci.yml](.github/workflows/ci.yml)）

---

## 10. 开发人员

**未来出行低空项目组**

排期：2026/09/01 — 2026/09/20

---

## 11. 任务书对齐

实现完全覆盖 3.25/3.26 任务书（V1.0）所有要求，**且各模块均已接入 `tools/plan_single.py` /
`tools/plan_multi.py` / `tools/run_stress_test.py` 主链路**（由 `tests/test_pipeline_wiring.py` 回归守护）：

| 任务书要求 | 本项目实现 | 接入方式 | 位置 |
|------------|------------|----------|------|
| §三 网格作为搜索基础 | `SpaceTimeGrid` 四维占用网格 | A* 共享网格参数 | `src/models/space.py` |
| §三 静态障碍约束 | `StaticObstacle` AABB + `is_static_blocked` | 多源融合加载 → 栅格化 | `src/models/obstacles.py` |
| §三 多机安全约束 | `SpatioTemporalOccupancyTable` + `ConflictResolver` | **`plan_batch()` 默认开启避让；`plan_multi.py --conflict-check` 复核；残余冲突 exit 2** | `src/planners/occupancy.py` |
| §三 飞行性能约束 | `AircraftPerformance.is_speed_valid/altitude_valid` | **A* 直接消费速度/爬升/降率/高度** | `src/models/aircraft.py` |
| §三 输出完整航点 | `RouteOptimizer.generate_waypoints` | `tools/plan_single.py` 写 `waypoints_*.json` | `src/planners/route_optimizer.py` |
| §三 失败返回原因 | `PlannedRoute.message` 字段 | 所有 Planner | `src/models/task.py` |
| §三 合法性验证 | `PathValidator` 7 维校验 | **两个入口默认执行，ERROR 时退出码 2** | `src/validator/path_validator.py` |
| §三.1 Planner 职责 | `SpaceTimeAStar` + `RouteOptimizer` | `tools/plan_single.py` | `src/planners/` |
| §三.2 Scheduler 职责 | `ParallelPlanner` | `tools/plan_multi.py` | `src/planners/parallel_planner.py` |
| §五.1 时空网格 + 通行代价 | `CostGrid` + `build_clearance_cost_grid` | **A* 代价层（默认开启）** | `src/grid/cost.py` |
| §五.2 任务时间窗 | `Waypoint.earliest_arrival/latest_arrival` | 任务模型 + 兼容性校验 | `src/models/aircraft.py` |
| §五.3 飞行器性能（速度/爬升/转弯/加速度/续航） | `PerformanceSpec` + `PhysicalSpec` + `EnduranceSpec` | A* 限速/升降率 + 优化器约束修正 | `src/models/aircraft.py` |
| §五.4 多机冲突避免 | `to_dense_mask()` 占用掩码 + `SpaceTimeAStar` 禁飞检查 + 时间/高度调整 | **`plan_batch(conflict_avoidance=True)` 检测→带掩码重规划→串行更新占用表（默认开启，残余冲突 exit 2）** | `src/planners/occupancy.py` + `src/planners/spacetime_astar.py` |
| §五.5 多目标优化 | `RouteOptimizer` + `objective_weights` | `tools/plan_single.py`（默认开启） | `src/planners/route_optimizer.py` |
| §五.6 航点生成 + 解释 | `generate_waypoints` + `PlanningExplanation` | **解释已填充并写入 JSON `explanation` 字段** | `src/models/task.py` |
| §六 解释模块 | `PlanningExplanation.to_dict()` | `_explain_result()` 生成 + 报告渲染 | `src/models/task.py` |
| §六 验证模块 | `PathValidator.validate_route/multi_routes` | **写 `validation*.json`** | `src/validator/` |
| §十 输出要求 | `PlannedRoute.to_dict()` 含全部字段 | 含 `explanation` | `src/models/task.py` |
| §十一 交付目录 | `src/{models,grid,planners,data_fusion,io,visualization,validator}` | 完整 | — |

---

## 12. 许可证

MIT License
