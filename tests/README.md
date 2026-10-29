# v2 测试

文档日期：2026-10-29。

本目录只测试当前 v2 接口及其规划实现。完整请求和响应保存在 [`data/`](data/README.md)。

向对接方提供独立的 [`../test_api/`](../test_api/README.md) 目录。该目录包含公网接口脚本、完整输入、测试报告和完整 HTTP 记录。使用 Python 3.11 或更新版本即可运行。

## 目录

| 目录 | 检查内容 |
|---|---|
| [`api/`](api/) | HTTP 接口、异步作业、能力查询、请求模型与错误响应 |
| [`planning/`](planning/) | 场景、坐标、网格、航路、规则、时序与动态避让 |
| [`native/`](native/) | 原生搜索、连续碰撞检查与运动学证书 |
| [`integration/`](integration/) | 完整调用流程、冻结基准、请求响应归档与数据校验 |
| [`data/`](data/README.md) | v2 接口示例和三套各 1000 条完整路线结果 |

[`conftest.py`](conftest.py) 设置源码导入路径。各目录中的 `test_*.py` 保存测试代码。

## 运行

在仓库根目录运行。先按 [`README`](../README.md#本地启动) 安装依赖和原生扩展。

```bash
# 全部测试
python -m pytest tests/ -q

# 按目录运行
python -m pytest tests/api/ -q
python -m pytest tests/planning/ -q
python -m pytest tests/native/ -q
python -m pytest tests/integration/ -q

# 只检查公开接口
python -m pytest tests/api/test_public_api.py -q
```

接口测试使用进程内测试客户端。它们检查路由、模型、响应和规划结果。对运行中的 Docker 服务执行真实 HTTP 检查：

```bash
python tools/smoke_v2.py --base-url http://127.0.0.1:18000
```

重跑三套各 1000 个任务，并保存完整请求与响应：

```bash
python tools/benchmark_v2.py --functional-only --base-url http://127.0.0.1:18000 \
  --scales 1000 --repeats 1 --output output/v2-functional
```

每次运行保存自己的输出。已归档数据的目录、校验和与重放步骤见 [`data/README.md`](data/README.md)。
