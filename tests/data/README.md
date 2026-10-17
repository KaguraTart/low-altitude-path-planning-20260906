# 完整测试输入与输出

`recordings/` 保存 v2 服务调用的完整 JSON，用于对接、回归和重跑。每次采集使用独立目录。目录时间使用 UTC，以 `Z` 结尾。采集过程保留完整请求和响应。

每个归档包含：

| 路径 | 内容 |
|---|---|
| `manifest.json` | 采集时间、用途、全部文件的相对路径、字节数和 SHA-256，以及请求与响应的对应关系 |
| `environment.json` | 实际服务版本、代码 revision、镜像 ID 和采集时资源限额 |
| `source_inputs/` | 冻结场景、任务和参考数据；源文件校验和保持不变 |
| `inputs/*_input.json` | 示例接口的完整请求；GET 调用保存 method、endpoint 和空 body 描述 |
| `outputs/*_output.json` | HTTP 实际返回的原始响应正文，保留全部字段和航点 |
| `benchmarks/payloads/` | strict、mixed、free 三套各 1000 任务的场景注册、规划请求和完整响应 |
| `benchmarks/*-report.json` | 每套的输入输出索引、原始/规范化摘要、实际耗时、独立几何和运动验证、连续冲突结果 |

归档只包含 v2。它保存三种模式的单机规划、批量规划、异步作业、坐标双向转换和验证示例，以及三套各 **1000 条完整航线结果**。批量请求引用同套归档中的场景。重跑时先注册场景，再把请求中的 `scene_id` 换成新返回值。

## 已保存的数据

[`recordings/20261017T112414Z_http_functional/`](recordings/20261017T112414Z_http_functional/manifest.json) 保存 15 次接口交换、三套各 1000 条完整结果及 57 份数据文件，约 17.4 MB。采集版本为 `52c12be`，容器资源为 2 CPU / 2 GiB。三套均有 1000 条成功结果，独立验证通过，连续冲突为 0。记录按采集时的版本保存。

数据用途标记为 `functional_replay_capture`。路线安全性和完整性均独立验证。耗时按实际调用记录。历史 8 CPU / 8 GiB 性能报告见 [`internal/reports/v2-20261016/`](../../internal/reports/v2-20261016/PERFORMANCE.md)。

## 校验与重跑

校验已保存数据，不调用服务：

```bash
python tools/archive_test_data.py --verify tests/data/recordings/20261017T112414Z_http_functional
python -m pytest tests/integration/test_recordings.py -q
```

从运行服务重新采集完整示例和三套结果，生成新目录：

```bash
python tools/archive_test_data.py --base-url http://127.0.0.1:18000 \
  --container low-altitude-path-planning-api
```

用归档的冻结输入重跑 1000 任务，完整保存每次请求与响应：

```bash
python tools/benchmark_v2.py --functional-only --base-url http://127.0.0.1:18000 \
  --input-directory tests/data/recordings/20261017T112414Z_http_functional/source_inputs/v2_benchmarks \
  --scales 1000 --repeats 1 --output output/replay/<新的目录名>
```

`--functional-only` 按功能判定结果。它要求所有任务成功，并完成独立规则、运动和连续冲突验证。默认基准命令执行 10 秒性能门槛。HTTP 原始响应摘要与规范化 JSON 摘要分开保存。归档写盘发生在计时结束后。

CI 保存当前 v2 请求和完整结果，作为工作流 artifact 提供下载。新采集保存在独立目录中，并附版本、资源配置和校验和。本目录属于测试交付资料，不进入服务镜像。
