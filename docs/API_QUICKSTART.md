# 联调对接说明（Quickstart）

> 给对接方看的操作手册：**怎么把服务跑起来、怎么验证字段对不对**。
>
> 字段的完整契约见 [`API_INTERFACE.md`](API_INTERFACE.md)。
> 本文只讲操作，不重复字段表。

---

## 0. 背景与作用

### 这套接口能做什么

这是一个**低空无人机路径规划算法**的服务接口，输入一片空域里的障碍物和飞行任务，
输出可执行的无人机航线。具体能力：

| 能力 | 说明 |
|------|------|
| **单机路径规划** | 给定起点终点（或完整航点序列），算出一条避开建筑与动态障碍物的可行航线 |
| **多机批量规划** | 一次提交多个任务，并行规划并保证任意两机航线之间满足最小安全间隔 |
| **多目标优化** | 在航程最短、时间最短、风险最低、能耗最低之间按权重折中 |
| **航线合法性验证** | 校验一条航线是否满足连续性、障碍避让、飞行器性能、高度限制等约束 |
| **规划解释** | 说明航线为什么这样走、避障几次、性能裕度还剩多少 |

### 为什么现在要做接口

算法原本只有命令行脚本（`plan_single.py` / `plan_multi.py`）这一种用法，
其他系统要接入只能起子进程再去翻输出文件，很难集成。现在补上了
**Python SDK + REST 服务**两层接口：

- 对方是 Python 项目 → 直接 `import uav_planner` 调用；
- 对方是其他语言 / 前端 → 走 HTTP，自带 Swagger 页面，**不写代码就能试**。

### 本文档的作用

把**从零到跑通**的每一步写清楚，包括：

- 怎么起服务、怎么把地址给对方；
- 连不上时按什么顺序排查（网络 → 防火墙 → 绑定地址 → 网段）；
- 每种调用方式的可直接执行示例（curl / Python / Postman）；
- 联调中最容易踩的坑（半宽半深填错、漏传中间航点、大批量超时）。

**读完本文你就能独立完成对接**；需要查具体字段时再翻
[`API_INTERFACE.md`](API_INTERFACE.md)。

### 对接前的两个约定

1. **服务无任何认证**（当前版本），仅限内网联调，请勿把地址转发到公网；
2. **大批量必须走异步作业**——实测 10000 个任务约需 437 秒，同步请求必被超时掐断，
   超过约 500 个任务请改用 `POST /api/v1/jobs`（见 [§4](#4-大批量任务用异步作业)）。

---

## 1. 三步跑起来

### 第 1 步：装依赖并起服务（算法方操作）

```bash
conda activate uav_path_planning
cd 自贡-低空横向无人机路径调度-邓典涛-20260906
python serve.py --host 0.0.0.0 --port 8000
```

启动后会打印所有可用的访问地址，例如：

```
  本机访问:
    交互文档  http://127.0.0.1:8000/docs

  局域网访问（把下面地址发给对接方）:
    http://192.168.124.23:8000/docs
```

**把「局域网访问」那一行的地址发给对接方**，对方在同一网段即可直接访问。

> `--host 0.0.0.0` 表示同网段可访问。若只想自己用，改成 `127.0.0.1`。

### 第 2 步：对方先开交互文档（对接方操作）

浏览器打开：

```
http://<算法方的IP>:8000/docs
```

这是自动生成的 Swagger 页面，**可以直接在网页上构造请求并看到真实响应**——
联调阶段不用写一行代码就能确认字段。

### 第 3 步：确认版本一致

```bash
curl http://<算法方的IP>:8000/api/v1/version
```

返回 `api_version` 应对得上本文档顶部的版本号。

---

## 2. 连不上怎么办

按顺序排查：

| 现象 | 原因 | 处理 |
|------|------|------|
| `Connection refused` | 服务没起，或端口不对 | 看算法方终端有没有 `Uvicorn running on ...` |
| 本机能访问、对方不能 | 服务绑了 `127.0.0.1` | 改用 `--host 0.0.0.0` 重启 |
| 超时无响应 | 防火墙拦截 | macOS：系统设置 → 网络 → 防火墙，放行 Python；或临时关闭 |
| 能连但 IP 不对 | 有多张网卡（有线/无线/虚拟网卡） | 启动日志会列出**所有**局域网地址，逐个试 |
| 对方在不同网段 | 不在同一局域网 | 需内网穿透（ngrok / frp），或部署到同一内网服务器 |

**自查清单**：

```bash
# 算法方：确认服务在监听
lsof -i :8000

# 算法方：确认自己的局域网 IP
ipconfig getifaddr en0        # macOS 无线
ipconfig getifaddr en1        # macOS 有线

# 对接方：先确认网络可达
ping <算法方的IP>
```

---

## 3. 调用示例

以下每条都是**可直接执行的完整命令**（把 `HOST` 换成实际地址）。

### 3.1 健康检查

```bash
curl -s http://HOST:8000/api/v1/health
```

### 3.2 单任务规划——最快看到完整响应结构

先取示例请求体，再原样打回去：

```bash
curl -s http://HOST:8000/api/v1/examples/single -o ex_single.json
curl -s -X POST http://HOST:8000/api/v1/plan/single \
    -H 'Content-Type: application/json' \
    -d @ex_single.json | python -m json.tool
```

也可以直接手写一个最小请求：

```bash
curl -s -X POST http://HOST:8000/api/v1/plan/single \
    -H 'Content-Type: application/json' \
    -d '{
      "start": {"x": 500, "y": 500, "z": 100},
      "goal":  {"x": 2500, "y": 2500, "z": 150}
    }' | python -m json.tool
```

### 3.3 多机批量规划

```bash
curl -s http://HOST:8000/api/v1/examples/batch -o ex_batch.json
curl -s -X POST http://HOST:8000/api/v1/plan/batch \
    -H 'Content-Type: application/json' \
    -d @ex_batch.json | python -m json.tool
```

### 3.4 校验对方给出的航线

对方把自己的航线发过来校验，这是联调最常用的一步：

```bash
curl -s -X POST http://HOST:8000/api/v1/validate \
    -H 'Content-Type: application/json' \
    -d '{
      "route": {
        "route_id": "对方的航线-001",
        "trajectory": [
          {"x": 0,   "y": 0, "z": 50, "t": 0},
          {"x": 100, "y": 0, "z": 50, "t": 10},
          {"x": 200, "y": 0, "z": 50, "t": 20}
        ]
      },
      "static_obstacles": [
        {"id": "b1", "name": "楼", "type": "building",
         "x": 500, "y": 0, "z": 0, "width": 80, "depth": 80, "height": 200}
      ]
    }' | python -m json.tool
```

返回 `is_valid` 与 `issues[]`（问题列表，含代码、级别、位置、时刻）。

### 3.5 Python 调用

**方式 A：用 SDK 直接 import**（对方是 Python 项目时最省事）

先安装。三种途径按需选一：

```bash
# ① 从源码安装（在仓库根目录执行，推荐用于联调期——改代码即时生效）
pip install -e ".[server]"

# ② 从构建好的 wheel 安装（对方拿到 .whl 文件时）
pip install uav_path_planning-1.0.0-py3-none-any.whl

# ③ 不安装，临时加路径（不想动环境时）
export PYTHONPATH=/path/to/自贡-低空横向无人机路径调度-邓典涛-20260906:$PYTHONPATH
```

可选 extras：`[server]`（REST 服务）、`[viz]`（可视化）、`[test]`（测试）、`[all]`（全部）。

然后：

```python
from uav_planner import plan_single

result = plan_single(
    start={"x": 500, "y": 500, "z": 100},
    goal={"x": 2500, "y": 2500, "z": 150},
)
print(result["status"])                        # planned
print(result["route"]["metrics"])              # 里程/时间/风险
print(result["validation"]["summary_text"])    # ✓ 合法：共 0 个问题...
```

**方式 B：走 HTTP**

```python
import requests

resp = requests.post(
    "http://HOST:8000/api/v1/plan/single",
    json={
        "start": {"x": 500, "y": 500, "z": 100},
        "goal": {"x": 2500, "y": 2500, "z": 150},
    },
    timeout=60,
)
resp.raise_for_status()
data = resp.json()
print(data["status"], data["route"]["metrics"])
```

**方式 C：Postman / Apifox**

导入 `http://HOST:8000/openapi.json`，全部接口与字段会自动生成，无需手写。

---

## 4. 大批量任务：用异步作业

**同步的 `/plan/batch` 只适合中小批量**。实测 10000 个任务约需 437 秒，
远超任何 HTTP 客户端的默认超时，同步调用一定会被掐断。

超过约 500 个任务时改用异步作业：

```bash
# 提交，立即拿到 job_id（HTTP 202）
curl -s -X POST http://HOST:8000/api/v1/jobs \
    -H 'Content-Type: application/json' \
    -d @ex_batch.json | python -m json.tool

# 轮询（把 <job_id> 换成上一步返回的值）
curl -s http://HOST:8000/api/v1/jobs/<job_id> | python -m json.tool
```

`status` 流转：`pending` → `running` → `succeeded` / `failed`。

Python 轮询示例：

```python
import time, requests

job = requests.post("http://HOST:8000/api/v1/jobs", json=payload, timeout=30).json()
job_id = job["job_id"]

while True:
    body = requests.get(f"http://HOST:8000/api/v1/jobs/{job_id}", timeout=30).json()
    if body["status"] in ("succeeded", "failed"):
        break
    time.sleep(2)

if body["status"] == "failed":
    print("失败:", body["error"])
else:
    print("成功", body["result"]["success_count"], "条")
```

> ⚠️ 作业存在**服务进程内存**中：服务重启后查不到历史作业。
> 联调够用；正式对接如果要求作业持久化，需要另行说明。

---

## 5. 常见问题

### Q：返回的 `status` 是 `failed`，是不是接口坏了？

不是。**规划不出路径是路径规划的常态结果**，接口正常返回（HTTP 200），
失败原因在 `route.message` 里，例如：

```json
{"status": "failed", "route": {"message": "终点高度 220.0m 超过飞行器最高高度 200.0m"}}
```

只有**参数本身有问题**才会返回 4xx 并带 `error` 信封。

### Q：报 400，怎么快速定位？

响应里的 `error.detail` 会指出具体字段：

```json
{"error": {"code": "INVALID_INPUT",
           "message": "config.dx 必须大于 0，实际 0",
           "detail": {"field": "dx", "got": 0}}}
```

形状类错误（类型不对）会列出所有不合格字段：

```json
{"error": {"code": "INVALID_INPUT", "message": "请求体校验失败",
           "detail": {"errors": [{"field": "start.z", "reason": "Input should be a valid number"}]}}}
```

### Q：`width` / `depth` 到底填全长还是半长？

**半长**。`width` 是 X 方向**半宽**、`depth` 是 Y 方向**半深**。

一个 160m × 160m 的建筑：

```json
{"id": "b1", "type": "building", "x": 2500, "y": 2500, "z": 0,
 "width": 80, "depth": 80, "height": 200}
```

填错的最典型症状是障碍物只有实际的一半大，航线「贴着建筑飞过去」但校验通过。

### Q：多点航线怎么传？传了中间点却没经过？

必须用 `waypoints` 传**完整航点序列**，不要只传 `start` / `goal`：

```json
{
  "waypoints": [
    {"id": "s", "x": 500,  "y": 500,  "z": 100, "type": "start"},
    {"id": "w", "x": 3000, "y": 3000, "z": 120, "type": "via"},
    {"id": "e", "x": 1000, "y": 3000, "z": 150, "type": "end"}
  ]
}
```

只传起终点会**丢掉中间经停**，规划出一条更短但不合要求的航线，
而且从里程数字上很难看出来。

### Q：为什么两点之间走的是折线，不是直线？

A\* 使用 **6 邻域**（上下左右前后）移动，路径天然呈阶梯状。
两点间沿网格的曼哈顿距离通常大于欧氏距离——这是算法特性，不是绕远。
例如从 (0,0) 到 (3500,3500)，里程约 7000 m 而非欧氏距离 4949 m。

### Q：算出来的距离比我预期长/短？

按影响从大到小排查：

1. **是否漏传了中间航点**（见上一条）；
2. **通行代价层**（`cost_grid`，默认开启）会引导航线远离建筑群，
   可能比最短路更长——这是设计意图，与 risk 目标一致；关掉可对比；
3. **飞行器升限**：`operational.max_altitude_m` 会限制可用高度层，
   可能迫使绕行；
4. **障碍物尺寸填错**（半宽/半深），导致可用空间被高估或低估。

### Q：结果里 `min_obstacle_distance_m` 是 999.0？

表示场景里**没有任何障碍物**（或全部被跳过），不是异常值。

### Q：接口有没有认证？

**没有**。当前版本无任何认证，CORS 也默认放开，仅供内网联调。
对接方请勿把地址转发到公网；生产部署前需要补鉴权。

---

## 6. 联调自查清单

对接完成前，建议逐项确认：

- [ ] `/api/v1/version` 的 `api_version` 与文档一致
- [ ] `/api/v1/examples/single` 的请求体能成功打回并拿到 `status: planned`
- [ ] 我方构造的真实任务（含真实障碍物尺寸）能规划成功
- [ ] 中间经停点确实被经过（对比轨迹与经停坐标）
- [ ] 我方自产航线经 `/api/v1/validate` 校验的 `is_valid` 符合预期
- [ ] 报错场景（缺字段、类型错）能正确收到 `error.code`
- [ ] 大批量走 `/api/v1/jobs` 能正常轮询到结果
- [ ] 确认批量规模与超时设置匹配（>500 任务必须走异步）
