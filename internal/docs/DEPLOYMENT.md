# 部署与回滚

接口字段与示例见 [`../../docs/API.md`](../../docs/API.md)。服务仅提供 `/api/v2`。本文件说明构建、验证、发布和回滚步骤。

## 1. 运行配置

| 项目 | 配置 |
|------|------|
| Python / 软件包 | 3.11 / 2.0.0，含 C++17 原生扩展 |
| 镜像 | `low-altitude-path-planning:<Git SHA>` |
| 正式容器 | `low-altitude-path-planning-api` |
| 正式端口 | `127.0.0.1:18000 → 8000` |
| 正式资源 | 2 CPU / 2 GiB |
| 候选容器 | `low-altitude-path-planning-candidate` |
| 候选端口 | `127.0.0.1:18001 → 8000` |

服务使用单进程。场景和作业保存在进程内存中，重启后清空。后台作业使用 1 个执行线程，其他作业排队等待。服务默认不限制场景、队列、历史数量或结果存储，也不自动淘汰。

场景、WKT 顶点、格子、网格节点、任务、航点、验证数据和采样不设固定数量上限。`time_budget_s` 省略或为 null 时持续计算。调用方可传正有限秒数设置计算时限。`horizon_s` 是飞行时域，默认 3600 秒。`max_radius_m` 是场景覆盖半径，默认 50000 m。后二者可设置更大的正数。

内存分配失败返回 HTTP 503、`RESOURCE_EXHAUSTED`。异步作业记录 failed 和同名错误码。系统 OOM 直接终止进程时，连接可能断开。Docker 记录 OOM 并按重启策略重启服务。正式配额保持 2 CPU / 2 GiB。

容器监听 `0.0.0.0:8000`，宿主机绑定回环地址。公网访问使用现有反向代理或隧道。访问控制由平台提供。

## 2. 安装与接口规范

构建需要 C++17 编译器和 CMake。

```bash
uv pip install -e '.[server,test]' -c requirements-runtime.lock
python -m pytest tests/ -q
python -c 'from uav_planner.v2 import native; print(native.engine_info()); assert native.available()'
python tools/benchmark_v2.py --validate-only
```

Dockerfile 先构建原生 wheel，再安装到 Python 3.11 运行层。镜像包含 `docs/API.md` 和 `docs/openapi.json`。模型变更后，在锁定依赖的 Python 3.11 环境重新生成规范：

```bash
python - <<'PY'
import json
from pathlib import Path
from server.app import app
Path('docs/openapi.json').write_text(
    json.dumps(app.openapi(), ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
PY
```

Swagger `/docs` 读取 `/openapi.json`。更新 Markdown 后，也要核对模型、规范快照和运行服务。

## 3. 构建与 CI

先完成代码、文档和回归。将实际变更及测试结果写入 PR。

```bash
git status --short
git diff --check
git push -u origin HEAD
# 将 PR 正文写入 /tmp/uav-release-pr.md。
gh pr create --title '更新航路规划服务' --body-file /tmp/uav-release-pr.md
```

GitHub CI 检查测试、真实服务调用和 Docker 发布契约。Docker 检查从同一提交构建镜像，核对原生核、提交 SHA、规划、场景、坐标、异步结果、错误和 Swagger。镜像内文档与规范需和提交一致。运行时规范需和快照一致。日志、镜像元数据和烟测结果保存在 `docker-release-contract` artifact。

构建不可变镜像，并执行同一发布检查：

```bash
UAV_BUILD_SHA=$(git rev-parse HEAD)
docker build --build-arg GIT_SHA="$UAV_BUILD_SHA" \
  -t "low-altitude-path-planning:$UAV_BUILD_SHA" .
docker image inspect "low-altitude-path-planning:$UAV_BUILD_SHA" \
  --format '{{.Id}} {{json .Config.Labels}}'
bash tools/check_docker_release.sh "low-altitude-path-planning:$UAV_BUILD_SHA" \
  "$UAV_BUILD_SHA" output/ci-docker
```

发布检查使用 2 CPU / 2 GiB。CI 不自动替换本机正式服务。PR 合入 main 后，核对 main 的 CI，并从 main 的实际提交构建发布镜像。

## 4. 候选验证

在独立端口启动已构建的镜像：

```bash
docker run -d --name low-altitude-path-planning-candidate \
  --cpus=2 --memory=2g --memory-swap=2g -p 127.0.0.1:18001:8000 \
  "low-altitude-path-planning:$UAV_BUILD_SHA"
curl --fail-with-body -sS http://127.0.0.1:18001/api/v2/health
python tools/smoke_v2.py --base-url http://127.0.0.1:18001 \
  --expected-git-sha "$UAV_BUILD_SHA" --openapi-snapshot docs/openapi.json
```

烟测执行真实规划、坐标往返、场景复用、批量和异步作业，并核对验证结果与错误信封。移动轨迹验证需提交规划返回的 `motion_segments`、`trajectory_wkt` 和 `vertex_times_s`。

检查候选容器的状态和资源：

```bash
docker stats --no-stream low-altitude-path-planning-candidate
docker inspect low-altitude-path-planning-candidate \
  --format 'status={{.State.Status}} oom={{.State.OOMKilled}} restarts={{.RestartCount}} cpus={{.HostConfig.NanoCpus}} memory={{.HostConfig.Memory}}'
docker logs low-altitude-path-planning-candidate
```

## 5. 性能与完整数据

在正式配额下重跑完整功能：

```bash
python tools/benchmark_v2.py --functional-only --base-url http://127.0.0.1:18001 \
  --scales 1000 --repeats 1 --output output/v2-functional
```

保存完整请求、响应、输入校验和、场景版本、成功数量、冲突、阶段耗时与输出大小。服务内存使用容器监测值。客户端 RSS 另行记录。

历史 10 秒性能验收使用专用 8 CPU / 8 GiB 容器。strict/mixed/free 三套各运行 10 轮，每轮 1000 条路线。复现时使用相同资源和冻结输入：

```bash
python tools/benchmark_v2.py --validate-only
python tools/benchmark_v2.py --base-url http://127.0.0.1:18001 \
  --repeats 10 --scales 1000 --output output/v2-performance
```

2026-10-16 的 30 轮记录见 [`../reports/v2-20261016/PERFORMANCE.md`](../reports/v2-20261016/PERFORMANCE.md)。这些记录按当时版本保存。当前版本的测试和实际耗时以本次输出为准。

完整测试输入与输出见 [`../../tests/data/README.md`](../../tests/data/README.md)。新采集使用独立目录，保存完整 JSON 和校验和。

## 6. 正式切换

暂停上游新提交。等待同步请求完成，并确认 pending/running 作业均为 0。保存完整作业结果和原始场景 JSON。

```bash
curl --fail-with-body -sS http://127.0.0.1:18000/api/v2/health/jobs
```

使用验收通过的 main 镜像切换服务。将 `UAV_RELEASE_SHA` 设为该镜像的完整提交 SHA：

```bash
: "${UAV_RELEASE_SHA:?Set the tested main Git SHA}"
UAV_IMAGE_TAG="$UAV_RELEASE_SHA" docker compose config
UAV_IMAGE_TAG="$UAV_RELEASE_SHA" docker compose up -d --no-build --force-recreate app
python tools/smoke_v2.py --base-url http://127.0.0.1:18000 \
  --expected-git-sha "$UAV_RELEASE_SHA" --openapi-snapshot docs/openapi.json
docker inspect low-altitude-path-planning-api \
  --format 'image={{.Image}} cpus={{.HostConfig.NanoCpus}} memory={{.HostConfig.Memory}}'
```

核对资源为 `NanoCpus=2000000000`、`Memory=2147483648`。重新注册场景，并恢复上游提交。确认正式服务正常后删除候选容器：

```bash
docker stop low-altitude-path-planning-candidate
docker rm low-altitude-path-planning-candidate
```

保存发布 SHA、镜像 ID、构建配置和验证结果。旧镜像可按精确 ID 清理。回滚按提交重新构建。

## 7. 回滚

暂停新提交，并等待现有作业完成。保存日志和镜像 ID。选择已验收的接口版本，在候选端口重新构建并验证。

```bash
docker logs low-altitude-path-planning-api > /tmp/uav-failed-container.log 2>&1
: "${UAV_ROLLBACK_SHA:?Set the verified rollback Git SHA}"
git cat-file -e "${UAV_ROLLBACK_SHA}^{commit}"
UAV_ROLLBACK_DIR=$(mktemp -d /tmp/uav-rollback.XXXXXX)
git worktree add --detach "$UAV_ROLLBACK_DIR" "$UAV_ROLLBACK_SHA"
docker build --build-arg GIT_SHA="$UAV_ROLLBACK_SHA" \
  -t "low-altitude-path-planning:$UAV_ROLLBACK_SHA" "$UAV_ROLLBACK_DIR"
git worktree remove "$UAV_ROLLBACK_DIR"
```

按第 4 节验证候选，再按第 6 节切换。使用所选版本对应的规范快照检查接口。重启后重新注册已保存的场景。

## 8. 当前状态核对

```bash
git rev-parse main
git branch -avv
git ls-remote --heads origin
curl --fail-with-body -sS http://127.0.0.1:18000/api/v2/version
docker inspect low-altitude-path-planning-api \
  --format 'image={{.Image}} oom={{.State.OOMKilled}} restarts={{.RestartCount}} cpus={{.HostConfig.NanoCpus}} memory={{.HostConfig.Memory}}'
docker image ls --no-trunc low-altitude-path-planning
```

合并必要变更后，清理已整合的开发分支和旧镜像。保留历史输入与原始报告。版本和资源使用上述命令实时核对。

## 9. 排错

| 现象 | 检查与处理 |
|------|------|
| health 为 degraded | 查看 version 的 engine.error；检查原生扩展、Python ABI 和安装结果 |
| scene/job 不存在 | 检查服务是否重启；重新注册原始场景，从已保存文件读取结果 |
| 高度转换失败 | 补充地形、水准面或对应位置与时间的气压修正 |
| strict 失败 | 检查航路高度、宽度、有效期、方向、连接和接驳 |
| 规则引用或场数据缺失 | 补充引用、版本、有效期和覆盖范围 |
| HTTP 耗时高 | 检查 JSON 大小、CPU throttling、序列化和作业积压 |
| RESOURCE_EXHAUSTED / OOM | 检查容器 OOMKilled、RestartCount、实际内存和日志 |
| 代理超时 | 使用异步轮询，或调整代理超时 |
