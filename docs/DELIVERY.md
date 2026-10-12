# 交付清单

> 本文档回答一个问题：**联调结束后，哪些文件是要交给对方的，哪些不是。**
>
> 交付形式（压缩包 / tag / 独立仓库）尚未确定，待对方明确要求后补充。
> 无论用哪种形式，**下面的清单就是依据**。

**版本**：v1.0.0 　|　**更新日期**：2026-10-12

---

## 1. 一句话说明

本次交付的是**算法接口层**：对方拿到后可以部署服务或 `import` 调用，
完成无人机路径规划与航线校验。**不含**我们内部使用的命令行工具、压测脚本、
可视化脚本和课题管理材料。

---

## 2. 交付物（必须交给对方）

### 2.1 代码

| 路径 | 内容 | 为什么必须交 |
|------|------|--------------|
| `uav_planner/` | Python SDK：`plan_single` / `plan_batch` / `plan_from_dir` / `validate_route` / `get_version` | 对方的主调用入口 |
| `uav_planner/pipeline.py` | 规划编排（A\* → 多目标优化；并行 → 冲突避免） | SDK 内部依赖 |
| `server/` | REST 服务（FastAPI）：路由、请求模型、异步作业表 | 对方用非 Python 语言时的入口 |
| `serve.py` | 服务启动脚本，启动时打印局域网地址 | 一行命令起服务 |
| `src/` | 算法内核：时空 A\*、多目标优化、并行规划、冲突避免、路径验证、数据融合 | SDK 与服务的实际实现，**缺了跑不起来** |

### 2.2 文档

| 路径 | 内容 |
|------|------|
| `docs/API_INTERFACE.md` | **接口契约主文档**：函数签名、参数表、返回结构、错误码、内部变量 |
| `docs/API_QUICKSTART.md` | **联调对接说明**：起服务、排查、可直接执行的调用示例、FAQ |
| `README.md` | 项目总览、算法简介、命令行用法 |
| `docs/deprecated/algorithm_design.md` | 算法原理（**内容仍然有效**，仅 I/O 章节被接口文档取代） |

> 对接方只需要读前两份就能完成接入；`README.md` 与算法设计文档供对方
> 了解算法背景，或做验收时参考。

### 2.3 示例数据

| 路径 | 内容 |
|------|------|
| `input/config.json` | 时空网格配置（5000×5000×500 m，50×50×10×120 格） |
| `input/static_obstacles.json` | 12 个静态障碍物（建筑、塔杆） |
| `input/dynamic_obstacles.json` | 4 个动态障碍物（轨迹 + 安全半径） |
| `input/aircraft_config.json` | 5 架飞行器性能参数 |
| `input/tasks.json` | 5 个飞行任务 |

> 让对方拿到就能跑通，不必等我们提供真实数据。

### 2.4 环境与打包配置

| 路径 | 内容 |
|------|------|
| `pyproject.toml` | 打包配置，支持 `pip install -e ".[server]"` |
| `requirements.txt` | pip 依赖清单 |
| `environment.yml` | conda 环境复现配置 |

---

## 3. 不交付物（留在我们仓库即可）

| 路径 | 内容 | 为什么不交 |
|------|------|------------|
| `tools/plan_single.py`<br>`tools/plan_multi.py` | 单机 / 多机**命令行**工具 | 对方走接口调用，不需要命令行脚本；且这些脚本**调用** SDK，属于我们自己的操作工具 |
| `tools/run_stress_test.py` | 压力测试脚本 | 内部性能验证用，与接口使用无关 |
| `tools/visualize.py`<br>`tools/visualize_stress.py` | 可视化脚本 | 内部演示用；对方如有可视化需求可自行基于接口返回的 `trajectory` 实现 |
| `tools/setup_env.sh` | 一键环境脚本 | 依赖我们的 conda 环境名 `uav_path_planning`，对方直接用 `pip install` 即可 |
| `internal/demand.md`<br>`internal/*.docx` | 课题需求与排期材料 | 课题管理材料，不属于技术交付 |
| `output/` | 规划产物（JSON、PNG、HTML） | 生成结果，可由对方自己跑出来 |
| `tests/` | **272 个单元测试** | 见下方说明 |
| `.github/workflows/ci.yml`<br>`pytest.ini` | CI 与测试配置 | 跟随 `tests/` 一起 |
| `docs/deprecated/INPUT_OUTPUT_GUIDE.md`<br>`docs/deprecated/input_output_spec.md` | 旧的 I/O 格式文档 | 已被 `API_INTERFACE.md` 取代 |

### 关于 `tests/` 的说明

按当前确定的**「仅接口层」**交付范围，测试**不纳入交付包**。

但如果对方需要做**验收**或**二次开发**，建议把 `tests/` 一并提供——
272 个用例是最直接的质量证据，而且对方改动算法后可以靠它发现回归。
这一点建议在交付前与对方确认。

---

## 4. 交付前验证

交付前按顺序跑一遍，确认交付物本身是自洽的：

```bash
conda activate uav_path_planning

# 1. 单元测试全绿
python -m pytest tests/ -q

# 2. SDK 可用（不依赖命令行工具）
python -c "from uav_planner import plan_single; \
r = plan_single({'x':500,'y':500,'z':100}, {'x':2500,'y':2500,'z':150}); \
print(r['status'], r['route']['metrics'])"

# 3. 服务可起、可调
python serve.py --host 127.0.0.1 --port 8000 &
curl -s http://127.0.0.1:8000/api/v1/health
curl -s http://127.0.0.1:8000/api/v1/examples/single -o /tmp/ex.json
curl -s -X POST http://127.0.0.1:8000/api/v1/plan/single \
     -H 'Content-Type: application/json' -d @/tmp/ex.json | head -c 300
kill %1
```

**第 2 步是最关键的**：它验证了「接口层脱离命令行工具也能独立工作」——
这正是本次结构调整要保证的事情。

进一步可做一次**纯净环境验证**（确认交付物不依赖仓库里的其他东西）：

```bash
pip wheel . --no-deps -w /tmp/wheel          # 构建
pip install --target /tmp/isolated /tmp/wheel/*.whl
cd /tmp && PYTHONPATH=/tmp/isolated python -c "from uav_planner import plan_single; \
print(plan_single({'x':500,'y':500,'z':100},{'x':2500,'y':2500,'z':150})['status'])"
```

---

## 5. 交付边界与已知限制

**必须向对方说明的三点**（详见 `docs/API_INTERFACE.md` §6）：

1. **REST 服务无任何认证**，CORS 默认放开 —— 仅限内网联调，勿暴露公网；
   生产部署前需补鉴权。
2. **异步作业表存放在服务进程内存** —— 服务重启后历史作业不可查；
   若对方要求作业持久化，需另行改造（换 Redis / 数据库）。
3. **作业队列固定单进程**（`serve.py --workers 1`）—— 规划本身是 CPU 密集
   且内部已多线程并行，多进程会互相抢核；且内存态作业表在多进程下会失效。

**性能特征**（对方接入前需要知道）：

- 单任务规划：通常 **10–100 ms**；
- 万级批量：实测 **约 437 秒**，必须走异步作业接口 `/api/v1/jobs`，
  同步接口 `/api/v1/plan/batch` 建议不超过 500 个任务。

---

## 6. 目录速查

```
交付 ──┬── uav_planner/     Python SDK
       ├── server/          REST 服务
       ├── serve.py         服务启动脚本
       ├── src/             算法内核
       ├── docs/API_*.md    接口文档
       ├── docs/deprecated/algorithm_design.md
       ├── input/           示例数据
       ├── pyproject.toml
       ├── requirements.txt
       └── environment.yml

不交付 ┬── tools/           命令行工具、压测、可视化
       ├── internal/        课题材料
       ├── output/          生成产物
       ├── tests/           测试（验收时可另议）
       └── docs/deprecated/INPUT_OUTPUT_GUIDE.md、input_output_spec.md
```
