# v1 / v2 部署与回滚

对外字段与示例统一见 [`../../docs/API.md`](../../docs/API.md)。本文件是内部运行手册，命令中的 SHA 必须来自本次实际构建；不把宿主机 SDK 诊断当作 Docker 发布验收。

## 1. 运行配置与基线

| 项目 | v2 目标 | 原正式服务 / 回滚基线 |
|------|------|------|
| Python / 包 | 3.11 / 2.0.0，含 C++17 扩展 | 原 v1 镜像 |
| 镜像标签 | `low-altitude-path-planning:<Git SHA>`，`UAV_IMAGE_TAG` 选择 | `low-altitude-path-planning:9816f8b` |
| 镜像 ID | 构建后记录 | `sha256:c03196d8cc7d1952ad29013e9dd70e12323127601615ba5253af7bc7f80e4e89` |
| 正式容器 | `low-altitude-path-planning-api` | 同名 |
| 正式映射 | `127.0.0.1:18000:8000` | 相同 |
| 资源 | 8 CPU / 8 GB（Compose `mem_limit: 8g`） | 2 CPU / 2 GiB |
| 候选容器 | `low-altitude-path-planning-candidate`，`127.0.0.1:18001:8000` | 不占正式端口 |

服务保持单进程。规划核内部并行，不通过增加 ASGI worker 共享进程内场景和作业表；多 worker 会导致 scene/job 查询落到没有记录的进程。容器内监听 `0.0.0.0:8000`，宿主机仅绑定回环，继续接入现有反向代理/隧道。服务不内置认证，公网访问沿用边缘鉴权与限流；限流范围须覆盖 `/api/v1/*` 和 `/api/v2/*`。

场景在编译前执行 `uav_planner/v2/budgets.py` 预算检查：单场景最多 200 航路、1000 障碍、20 管制区、1000 接驳、1000 规则、32 配置、100 数据集和 50000 WKT 顶点；格场/外部网格条目各最多 1200000，外部网格节点乘积最多 4000000，单场景估算存储最多 512 MiB。登记表最多 8 场景、合计估算存储 1 GiB，以 LRU 淘汰；新场景编译串行执行。估算量不是实际 RSS 硬上限，不能代替容器 8 GB 资源限制；验收需监测服务内存。当前镜像的限额通过 `/api/v2/capabilities` 核对。

时域 `horizon_s` 默认 3600、最大 86400 秒；单任务最多 256 个中间业务航点；SDK/HTTP 批量均为 1–10000 个任务。验证时间数组和解析原语分别最多 100000 项；每路线的认证碰撞采样与导出顶点也分别限制 100000 项，超额返回明确预算错误，不输出截断的成功路线。

v2 后台队列最多 32 个 pending+running、1 个执行线程；满载或关闭时拒绝新提交，返回 HTTP 429/JOB_QUEUE_FULL。已完成结果合计保留预算 128 MiB（Python 对象估算），按历史数量/字节预算淘汰；单结果超限则作业 failed/JOB_RESULT_TOO_LARGE。v1 共享相同保留与关闭机制，默认队列容量 200、执行线程 2。平台在取回完整结果后保存，不能依赖内存作业表长期留存。原生内核不可用为 HTTP 503/NATIVE_UNAVAILABLE，其他模型校验错误使用 HTTP 400 的统一错误信封；v2 OpenAPI 不声明自动 422 校验响应。

## 2. 本地安装与构建

开发安装会用 CMake 和 C++17 编译器构建 pybind11 扩展：

```bash
uv pip install -e '.[server,test]'
python -m pytest tests/ -q
python -c 'from uav_planner.v2 import native; print(native.engine_info()); assert native.available()'
python tools/benchmark_v2.py --validate-only
```

Dockerfile 使用构建层制作原生 wheel，在 Python 3.11 运行层安装包与 REST 依赖；文档和 OpenAPI 随同代码放入该镜像。任何模型变化后先重新生成快照：

```bash
python - <<'PY'
import json
from pathlib import Path
from server.app import app
Path('docs/openapi.json').write_text(
    json.dumps(app.openapi(), ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
PY
```

生成规范只导入 ASGI 应用并读类型模型，不执行规划 handler。Swagger `/docs` 是 UI，读取 `/openapi.json`；只改 Markdown 不会改变 Swagger 契约。

## 3. 分支、PR 与不可变候选镜像

先完成代码、文档、回归和固定输入校验，再推送当前 `codex/` 分支创建审查记录。PR 描述必须写实际功能与验证结果；容器性能尚未验证就明确标记待验证。

```bash
git status --short
git diff --check
git push -u origin HEAD
# 先将本次准确的变更与测试结果写入 /tmp/uav-v2-pr.md。
gh pr create --title '新增 WKT/WGS84 v2 约束规划接口并保留 v1' --body-file /tmp/uav-v2-pr.md
```

待需要进入镜像的改动均已提交后构建，记录完整 Git SHA 与镜像 ID：

```bash
UAV_BUILD_SHA=$(git rev-parse HEAD)
docker build --label org.opencontainers.image.revision="$UAV_BUILD_SHA" \
  -t "low-altitude-path-planning:$UAV_BUILD_SHA" .
docker image inspect "low-altitude-path-planning:$UAV_BUILD_SHA" \
  --format '{{.Id}} {{json .Config.Labels}}'
docker image inspect low-altitude-path-planning:9816f8b --format '{{.Id}}'
docker run -d --name low-altitude-path-planning-candidate \
  --cpus=8 --memory=8g -p 127.0.0.1:18001:8000 \
  "low-altitude-path-planning:$UAV_BUILD_SHA"
```

如使用镜像仓库，把同一构建按实际仓库名称加 `<完整 SHA>` 标签后 push；正式机拉取并核对 digest，不用浮动 latest 代替本次验收镜像。当前本机 Compose 可直接使用上述本地 SHA 标签。

## 4. 候选烟测：端口 18001

正式服务在 18000 继续运行。先检查候选原生核、版本、资源与文档：

```bash
docker inspect low-altitude-path-planning-candidate \
  --format 'image={{.Image}} cpus={{.HostConfig.NanoCpus}} memory={{.HostConfig.Memory}}'
docker exec low-altitude-path-planning-candidate python -c \
  'import uav_planner; from uav_planner.v2 import native; print(uav_planner.__version__, native.engine_info()); assert native.available()'
curl --fail-with-body -sS http://127.0.0.1:18001/api/v1/health
curl --fail-with-body -sS http://127.0.0.1:18001/api/v2/health
curl --fail-with-body -sS http://127.0.0.1:18001/api/v2/version
curl --fail-with-body -sS http://127.0.0.1:18001/api/v2/capabilities
curl --fail-with-body -sS http://127.0.0.1:18001/openapi.json -o /tmp/uav-candidate-openapi.json
docker exec low-altitude-path-planning-candidate test -s /app/docs/API.md
docker exec low-altitude-path-planning-candidate test -s /app/docs/openapi.json
```

下面脚本用标准库进行双版本真实规划与异步完整结果烟测；不是只检验端口或 202。v2 还检查场景复用、坐标转换、验证端点和错误信封。

```bash
python - <<'PY'
import json
import time
import urllib.error
import urllib.request

base = 'http://127.0.0.1:18001'
def call(path, data=None):
    body = None if data is None else json.dumps(data).encode()
    request = urllib.request.Request(base + path, body, {'Content-Type':'application/json'})
    with urllib.request.urlopen(request, timeout=120) as response:
        return json.load(response)

v1 = call('/api/v1/examples/single')
planned_v1 = call('/api/v1/plan/single', v1)
assert planned_v1['status'] == 'planned', planned_v1
assert planned_v1['validation']['is_valid'], planned_v1

example = call('/api/v2/examples/single')
registered = call('/api/v2/scenes', example['scene'])
scene_id = registered['scene_id']
assert call('/api/v2/scenes/' + scene_id)['scene_version'] == registered['scene_version']
task = example['task']
route = call('/api/v2/plan/single', {'scene_id':scene_id, 'task':task})
assert route['status'] == 'planned' and route['validation']['valid'], route
checked = call('/api/v2/validate', {'scene_id':scene_id, 'task':task,
    'trajectory_wkt':route['trajectory_wkt'], 'vertex_times_s':route['vertex_times_s'],
    'motion_segments':route['motion_segments']})
assert checked['valid'], checked
missing_motion = call('/api/v2/validate', {'scene_id':scene_id, 'task':task,
    'trajectory_wkt':route['trajectory_wkt'], 'vertex_times_s':route['vertex_times_s']})
assert not missing_motion['valid'], missing_motion
assert any(v['code'] == 'MOTION_MODEL_REQUIRED' for v in missing_motion['violations'])
converted = call('/api/v2/coordinates/convert', {'coordinates':example['scene']['coordinates'],
    'direction':'wgs84_to_local', 'alt_reference':'MSL', 'geometries_wkt':[task['start_wkt']]})
assert len(converted['geometries_wkt']) == 1, converted

for version in ('v1', 'v2'):
    payload = call(f'/api/{version}/examples/batch')
    submitted = call(f'/api/{version}/jobs', payload)
    deadline = time.monotonic() + 120
    while True:
        job = call(submitted['poll_url'])
        if job['status'] == 'failed':
            raise AssertionError(job)
        if job['status'] == 'succeeded':
            assert job['result']['success_count'] > 0, job
            if version == 'v2':
                assert job['result']['failed_count'] == 0, job
            break
        if time.monotonic() >= deadline:
            raise TimeoutError(job)
        time.sleep(0.2)

try:
    call('/api/v2/plan/single', {'unexpected':True})
    raise AssertionError('Expected HTTP 400')
except urllib.error.HTTPError as error:
    assert error.code == 400
    assert json.load(error)['error']['code'] == 'INVALID_INPUT'
print('v1/v2 candidate smoke passed')
PY
```

验证移动航线必须同时提交 route.motion_segments；模型允许字段省略，但缺失会返回 valid:false、MOTION_MODEL_REQUIRED。POINT Z 单个时刻的零持续时间静止状态可省略；持续等待用重复坐标 LINESTRING Z、递增时间和 hover 原语。发布必须核对镜像内规范快照和运行 `/openapi.json` 相同，不只检查 Markdown 文件存在。

## 5. 固定性能验收

场景预注册后，三套 strict/mixed/free，各 1000 个任务各 10 轮，在候选容器 8 CPU / 8 GB 上要求每轮全部成功、独立验证通过且完整 HTTP/JSON ≤10 秒。冻结输入、SHA 校验、参考可行性与 ALL20 反事实检查由 runner 执行；不能换掉失败输入、删规则或复用任务答案。

```bash
python tools/benchmark_v2.py --validate-only
python tools/benchmark_v2.py --base-url http://127.0.0.1:18001 \
  --repeats 10 --scales 10 100 1000 --output output/v2_benchmarks/docker-http
docker stats --no-stream low-altitude-path-planning-candidate
docker logs low-altitude-path-planning-candidate
```

计时包括请求创建、真实规划与完整返回 JSON；场景注册另计，独立验算在计时外。runner 报告中的 HTTP RSS 属于本地验算进程，服务内存需从容器监测另存。保留每轮结果、input/result SHA、场景版本、成功/失败数、运动学证书数、冲突数、min/median/p95/max、输出字节数、镜像 ID 与资源限制。异步提交的耗时不满足这一门槛。

| 项目 | 当前结果 |
|------|------|
| 宿主机 SDK 首轮 1000 条诊断 | strict 1.514 s，mixed 4.623 s，free 1.211 s；全部成功 |
| 候选镜像双版本烟测 | **待验证** |
| 8 CPU / 8 GB 三套十轮 HTTP | **待验证** |
| 正式替换后双版本复测 | **待验证** |

以上诊断不能证明 Docker 门槛已经达标。发布前填入本次实际证据。

## 6. 排空与正式切换

先在已有平台/代理处暂停新的同步规划和异步提交，再等待两个版本的 pending/running 均为 0；同时确认平台没有尚未完成的同步调用。服务没有内置维护开关，不存在可调用的“自动排空”端点。

```bash
curl --fail-with-body -sS http://127.0.0.1:18000/api/v1/health/jobs
curl --fail-with-body -sS http://127.0.0.1:18000/api/v2/health/jobs
```

若当前旧 v1 镜像没有 `/api/v2/health/jobs`，404 表示没有 v2 作业表；后续 v2→v2 发布必须排空两套表。不要仅查看 v1 健康后就中断 v2 作业。

应用关闭流程停止两版本接纳、将尚未执行的 pending 作业标为 JOB_CANCELLED，并等待 running 作业结束。它不替代切换前的上游暂停与排空；只有看到两套 pending/running 均为 0，才进入正常发布切换。

候选所有门槛通过后，用**验收过的同一 SHA 镜像**切换 Compose，保持正式回环端口：

```bash
UAV_RELEASE_SHA=$(git rev-parse HEAD)
UAV_IMAGE_TAG="$UAV_RELEASE_SHA" docker compose config
UAV_IMAGE_TAG="$UAV_RELEASE_SHA" docker compose up -d --no-build --force-recreate app
curl --fail-with-body -sS http://127.0.0.1:18000/api/v1/health
curl --fail-with-body -sS http://127.0.0.1:18000/api/v2/health
curl --fail-with-body -sS http://127.0.0.1:18000/api/v2/version
docker inspect low-altitude-path-planning-api --format '{{.Image}}'
```

如果当前 HEAD 已变化，将 `UAV_RELEASE_SHA` 设为第 3 节记录并验收通过的原 SHA；不能重新取一个未经测试的镜像。生产重复第 4 节烟测（base 改为 18000），重新注册平台保存的原始场景 JSON，然后恢复流量。重启会丢失内存中的场景与历史作业，旧 scene_id 不可作为恢复数据源。

确认正式运行后再删除候选容器；保留上一版镜像与发布证据：

```bash
docker stop low-altitude-path-planning-candidate
docker rm low-altitude-path-planning-candidate
```

## 7. 回滚到原 v1 镜像

先暂停新提交并尽可能排空，保存当前故障日志和镜像 ID。确认回滚镜像 ID 与基线完全一致，再使用资源覆盖文件恢复旧 2 CPU / 2 GiB 配置：

```bash
docker image inspect low-altitude-path-planning:9816f8b --format '{{.Id}}'
docker logs low-altitude-path-planning-api > /tmp/uav-v2-failed-container.log 2>&1
cat > /tmp/uav-rollback.yaml <<'YAML'
services:
  app:
    image: low-altitude-path-planning:9816f8b
    cpus: 2
    mem_limit: 2g
YAML
UAV_IMAGE_TAG=9816f8b docker compose -f compose.yaml -f /tmp/uav-rollback.yaml \
  up -d --no-build --force-recreate app
curl --fail-with-body -sS http://127.0.0.1:18000/api/v1/health
curl --fail-with-body -sS http://127.0.0.1:18000/api/v1/version
docker inspect low-altitude-path-planning-api \
  --format 'image={{.Image}} cpus={{.HostConfig.NanoCpus}} memory={{.HostConfig.Memory}}'
```

原镜像仅提供 v1，回滚后平台要恢复 v1 调用；不能把 v2 请求发送到旧镜像。v2 场景与作业内存无法跨版本恢复。验证 v1 示例真实规划成功后恢复流量。下次重新发布 v2 时去掉回滚覆盖文件，并明确指定新的 SHA。

## 8. 排错

| 现象 | 证据与处理 |
|------|------|
| v1 health 正常而 v2 degraded | 查看 v2 version 的 engine.error；检查 wheel 中扩展和 Python ABI，不以 v1 health 代替 v2 就绪 |
| scene/job 不存在 | 检查是否重启、被淘汰或误用多进程；重提原始 Scene，历史作业从平台保存的结果恢复 |
| 高度转换失败 | 补正确地形、水准面或位置/时间范围内唯一气压修正；不改为猜测基准 |
| strict 失败 | 检查航路高度、全宽、有效期、方向、真实连接和显式接驳；不自动改 free |
| 规则缺引用或 nodata | 补版本化空间场、有效时段和覆盖范围；不删除规则来通过验收 |
| 容器 HTTP 比 SDK 慢 | 对比完整 JSON 大小、CPU throttling、序列化和并发作业；按同一冻结输入定位 |
| 容器 OOM / 抢占 CPU | 检查实际 8 CPU/8 GB 限制、作业积压和服务 RSS；不能只看客户端 RSS |
| 代理超时 | 使用异步完整结果轮询或按业务调整代理超时；性能验收仍计完整规划返回 |
