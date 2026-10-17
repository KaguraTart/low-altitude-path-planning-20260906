# 完整测试输入与输出

`recordings/` 保存实际服务调用的完整 JSON，供对接检查、回归和重跑使用。每次采集建立一个独立目录，时间使用 UTC，目录名带 `Z`。原始请求、响应和既有文件均不会被静默覆盖。

每个归档包含：

| 路径 | 内容 |
|---|---|
| `manifest.json` | 采集时间、用途、全部文件的相对路径、字节数和 SHA-256，以及请求与响应的对应关系 |
| `environment.json` | 实际服务版本、代码 revision、镜像 ID 和采集时资源限额 |
| `source_inputs/` | 完整复制的原始 `input/*.json` 和冻结场景、任务、参考数据；源文件校验和保持不变 |
| `inputs/*_input.json` | 示例接口的完整请求；GET 调用保存 method、endpoint 和空 body 描述 |
| `outputs/*_output.json` | HTTP 实际返回的原始响应正文，保留全部字段和航点 |
| `benchmarks/payloads/` | strict、mixed、free 三套各 1000 任务的场景注册、规划请求和完整响应 |
| `benchmarks/*-report.json` | 每套的输入输出索引、原始/规范化摘要、实际耗时、独立几何和运动验证、连续冲突结果 |
| `original_outputs/` | 此前已经保存的少量历史完整响应，原样复制；未保存对应输入的文件明确标记 `input_available=false` |
| `cli_recordings/` | 单机、多机及小规模压力测试的原始输入、公开输出与未舍入内部模型状态；有意限制搜索预算的失败样例也完整保留 |

当前采集只调用 v2，包含三种模式的单机、批量、异步、坐标双向转换和验证示例，以及三套各 **1000 条完整航线结果**。既有归档还保留历史接口记录。批量请求引用的 `scene_id` 依赖同套归档中的场景注册输入和响应；重跑时先注册场景，再把请求中的 ID 换成新返回值。

已保存的首套归档为 [`recordings/20261016T160346Z_http_functional/`](recordings/20261016T160346Z_http_functional/manifest.json)，共 127 份数据文件，约 17.8 MB，包含 21 次示例接口交换和三套各 1000 条结果。目录时间 `2026-10-16 16:03:46 UTC` 对应北京时间 `2026-10-17 00:03:46`。具体文件清单及独立验证记录见该目录的 manifest 与报告。

当前 v2 归档为 [`recordings/20261017T112414Z_http_functional/`](recordings/20261017T112414Z_http_functional/manifest.json)。它保存 15 次示例接口交换、三套各 1000 条完整结果及 57 份数据文件，约 17.4 MB。采集版本为 `52c12be`，容器资源为 2 CPU / 2 GiB。三套均有 1000 条成功结果，独立验证通过，连续冲突为 0。

历史录制中的旧接口和 CLI 数据保持原字节，不用于当前服务调用。历史 8 CPU / 8 GiB 的 30 轮性能报告继续保存在 `internal/reports/v2-20261016/`。当时成功响应正文未留存，本目录记录新的实际采集结果。当前采集使用正式 2 CPU / 2 GiB 服务，标记为 `functional_replay_capture`；路线安全性与完整性均独立验证，耗时按实记录。固定性能验收条件见原报告。

校验已保存数据，不调用服务：

```bash
python tools/archive_test_data.py --verify tests/data/recordings/<归档目录>
python -m pytest tests/test_recorded_test_data.py -q
```

从运行服务重新采集完整示例和三套结果，生成新目录：

```bash
python tools/archive_test_data.py --base-url http://127.0.0.1:18000 \
  --container low-altitude-path-planning-api
```

用归档的冻结输入重跑 1000 任务，完整保存每次请求与响应：

```bash
python tools/benchmark_v2.py --functional-only --base-url http://127.0.0.1:18000 \
  --input-directory tests/data/recordings/<归档目录>/source_inputs/v2_benchmarks \
  --scales 1000 --repeats 1 --output output/replay/<新的目录名>
```

`--functional-only` 只切换性能判定口径，仍要求所有任务成功并完成独立规则、运动和连续冲突验证；默认基准命令继续执行原来的 10 秒门槛。HTTP 原始响应摘要与规范化 JSON 摘要分开保存。归档写盘发生在计时结束后。

CI 保存当前 v2 请求和完整结果，作为工作流 artifact 提供下载。新采集保存在独立目录中，并附版本、资源配置和校验和。本目录属于测试交付资料，不进入服务镜像。
