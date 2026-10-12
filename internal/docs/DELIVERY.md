# 交付清单

> 本文档回答一个问题：**交付时哪些文件给对方，哪些不给。**
>
> 本文件本身**不随交付提供**（它与 `DEPLOYMENT.md` 同属内部资料）。

**版本**：v1.0.0 　|　**更新日期**：2026-10-12

---

## 1. 一句话说明

交付**算法本体 + 调用接口 + 接口文档 + 测试用例**。对方拿到后可以部署服务、
`import` 调用、并用自己的用例验证行为。

不含我们内部的开发工具、课题材料和运维资料。

---

## 2. 交付物

### 2.1 接口文档（唯一一份）

| 路径 | 内容 |
|------|------|
| `docs/API.md` | **接口说明文档**：接口列表、输入数据结构、函数说明、返回结构、错误码、调用示例、常见问题、内部变量 |

> 这是**唯一对外的说明文档**，对方读完即可完成接入。接口契约与调用示例合在
> 一起，不要求对方在两个文件间跳转。

### 2.2 核心算法与调用代码

| 路径 | 内容 | 为什么必须给 |
|------|------|--------------|
| `src/` | 算法内核：时空 A\*、多目标优化、并行规划、冲突避免、路径验证、数据融合、可视化 | 接口层的实际实现，**缺了跑不起来** |
| `uav_planner/` | Python SDK：`plan_single` / `plan_batch` / `plan_from_dir` / `validate_route` / `get_version` | Python 调用入口 |
| `server/` | REST 服务（FastAPI）：路由、请求模型、异步作业表 | 非 Python 语言的调用入口 |
| `serve.py` | 服务启动脚本 | 一行命令起服务 |

**依赖方向**（单向，接口层可独立交付）：

```
server/ ──┐
serve.py ─┼──→ uav_planner/ ──→ src/
纯调用方 ─┘
```

### 2.3 测试用例

| 路径 | 内容 |
|------|------|
| `tests/` | **272 个用例**：时空 A\*、代价层、冲突避免、数据融合、SDK 契约、REST 端点、主链路接线 |

对方的用途：

- **验收**：直接跑一遍就是最直接的质量证据；
- **二次开发**：改动算法后靠它发现回归。

```bash
pip install -r requirements.txt
python -m pytest tests/ -q
```

### 2.4 示例数据

| 路径 | 内容 |
|------|------|
| `input/config.json` | 时空网格配置（5000×5000×500 m，50×50×10×120 格） |
| `input/static_obstacles.json` | 12 个静态障碍物（建筑、塔杆） |
| `input/dynamic_obstacles.json` | 4 个动态障碍物（轨迹 + 安全半径） |
| `input/aircraft_config.json` | 5 架飞行器性能参数 |
| `input/tasks.json` | 5 个飞行任务 |

> 对方拿到就能跑通，不必等我们提供真实数据。

### 2.5 环境与打包配置

| 路径 | 内容 |
|------|------|
| `pyproject.toml` | 打包配置，支持 `pip install -e ".[server]"` |
| `requirements.txt` | pip 依赖清单 |
| `environment.yml` | conda 环境复现配置 |
| `pytest.ini` | 测试配置（跑 `tests/` 需要） |
| `README.md` | 项目总览、算法简介、命令行用法 |

---

## 3. 不交付物

| 路径 | 内容 | 为什么不交 |
|------|------|------------|
| `tools/` | 命令行工具（`plan_single.py` / `plan_multi.py`）、压测、两个可视化脚本、环境脚本 | 我们自己的操作工具。它们**调用** SDK，不是接口的一部分 |
| `internal/docs/DELIVERY.md` | 本文档 | 交付管理材料 |
| `internal/docs/DEPLOYMENT.md` | 部署运维（Docker、Cloudflare、边缘鉴权） | 我们自己的运维资料 |
| `internal/docs/archive/` | 旧版 I/O 文档与算法设计说明 | 已被 `docs/API.md` 取代 |
| `internal/demand.md`、`internal/*.docx` | 课题需求与排期材料 | 课题管理材料 |
| `output/` | 规划产物（JSON、PNG、HTML） | 生成结果，对方自己跑就有 |
| `.github/` | CI 配置 | 我们自己的流水线 |

---

## 4. 交付前验证

```bash
# 1. 测试全绿（对方拿到后也会跑这一条）
python -m pytest tests/ -q

# 2. SDK 可用
python -c "from uav_planner import plan_single; \
print(plan_single({'x':500,'y':500,'z':100},{'x':2500,'y':2500,'z':150})['status'])"

# 3. 服务可起、可调
python serve.py --host 127.0.0.1 --port 8000 &
curl -s http://127.0.0.1:8000/api/v1/health
curl -s -X POST http://127.0.0.1:8000/api/v1/plan/single \
  -H 'Content-Type: application/json' \
  -d '{"start":{"x":500,"y":500,"z":100},"goal":{"x":2500,"y":2500,"z":150}}'
kill %1
```

**第 2 步是关键**：它验证接口层**脱离 `tools/` 也能独立工作**。

再做一次**纯净环境验证**（确认交付物不依赖仓库里的其他东西）：

```bash
pip wheel . --no-deps -w /tmp/wheel
pip install --target /tmp/isolated /tmp/wheel/*.whl
cd /tmp && PYTHONPATH=/tmp/isolated python -c "from uav_planner import plan_single; \
print(plan_single({'x':500,'y':500,'z':100},{'x':2500,'y':2500,'z':150})['status'])"
```

---

## 5. 必须向对方说明的事

### 安全

**服务不做认证与限流** —— 能访问到端口就能提交规划任务。部署时须控制暴露范围：

| 范围 | 做法 |
|------|------|
| 仅本机 | 绑 `127.0.0.1` |
| 可信内网 | 绑 `0.0.0.0`，确保在受控网段 |
| **公网** | **必须在前面加边缘鉴权**（Cloudflare Access / 反向代理 Basic Auth / VPN） |

> ⚠️ **资源限制不等于鉴权**。给容器限 CPU 能防止它拖垮宿主机，但拦不住谁在调用——
> 访问者照样能把分配的那点 CPU 跑满。

### 已知限制

1. **异步作业表存在服务进程内存** —— 重启后历史作业不可查；
   若要求持久化需另行改造（换 Redis / 数据库）。
2. **服务固定单进程**（`serve.py --workers 1`）—— 规划本身是 CPU 密集且内部已
   多线程并行；且内存态作业表在多进程下会失效。
3. **CORS 默认放开**（`allow_origins=["*"]`）—— 便于联调期前端直连；
   生产建议收紧为具体来源。

### 性能特征

| 场景 | 耗时 |
|------|------|
| 单任务规划 | 通常 **10–100 ms** |
| 万级批量（10000 任务） | 实测 **约 437 秒** → 必须走异步接口 `POST /api/v1/jobs` |

同步批量接口 `POST /api/v1/plan/batch` 建议不超过 500 个任务。

---

## 6. 目录速查

```
交付 ──┬── docs/API.md          接口说明（唯一对外文档）
       ├── src/                 算法内核
       ├── uav_planner/         Python SDK
       ├── server/              REST 服务
       ├── serve.py             服务启动脚本
       ├── tests/               272 个用例
       ├── input/               示例数据
       └── pyproject.toml / requirements.txt / environment.yml / pytest.ini / README.md

不交付 ┬── tools/               命令行工具、压测、可视化
       ├── internal/            课题材料 + 内部文档 + 旧文档归档
       ├── output/              生成产物
       └── .github/             CI 配置
```
