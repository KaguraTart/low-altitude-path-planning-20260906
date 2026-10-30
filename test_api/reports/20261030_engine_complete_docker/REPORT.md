# 接口测试报告

文档日期：2026-10-30。

测试地址：http://127.0.0.1:18002

服务版本：`2.0.0`。构建提交：`b5c8b8b68fab15c333c955c8fb72925edd95a8d2`。

开始时间：2026-10-30 14:06:40 +0800。
结束时间：2026-10-30 14:07:15 +0800。
时间使用北京时间。
原始开始时间：`2026-10-30T06:06:40.726Z`。
原始结束时间：`2026-10-30T06:07:15.291Z`。

总体结果：**通过**。
检查通过 28 项，失败 0 项。
接口覆盖 16 / 16 项。

## 接口覆盖

接口覆盖统计成功 HTTP 调用。检查记录列出业务验证结果。

| 方法 | 接口 | 结果 |
|---|---|---|
| GET | `/api/v2/health` | 通过 |
| GET | `/api/v2/health/jobs` | 通过 |
| GET | `/api/v2/version` | 通过 |
| GET | `/api/v2/capabilities` | 通过 |
| GET | `/api/v2/examples/single` | 通过 |
| GET | `/api/v2/examples/batch` | 通过 |
| POST | `/api/v2/scenes` | 通过 |
| GET | `/api/v2/scenes/{scene_id}` | 通过 |
| POST | `/api/v2/coordinates/convert` | 通过 |
| POST | `/api/v2/plan/single` | 通过 |
| POST | `/api/v2/plan/batch` | 通过 |
| POST | `/api/v2/validate` | 通过 |
| POST | `/api/v2/jobs` | 通过 |
| GET | `/api/v2/jobs/{job_id}` | 通过 |
| POST | `/api/v2/networks/analyze` | 通过 |
| POST | `/api/v2/plan/replan` | 通过 |

## 批量规划

| 数据集 | 任务数 | 成功数 | 失败数 | 冲突数 | HTTP 耗时 / ms | 完整响应 |
|---|---:|---:|---:|---:|---:|---|
| strict | 1000 | 1000 | 0 | 0 | 7690.07 | `http/055_post_api_v2_plan_batch_output.json` |
| mixed | 1000 | 1000 | 0 | 0 | 14490.955 | `http/058_post_api_v2_plan_batch_output.json` |
| free | 1000 | 1000 | 0 | 0 | 7342.054 | `http/061_post_api_v2_plan_batch_output.json` |

## 检查记录

| 检查 | 结果 | 记录 |
|---|---|---|
| 服务健康 | 通过 | 检查通过。 |
| 作业状态统计 | 通过 | 检查通过。 |
| 服务版本 | 通过 | 检查通过。 |
| 接口能力 | 通过 | 检查通过。 |
| OpenAPI 文档 | 通过 | 检查通过。 |
| Swagger 页面 | 通过 | 检查通过。 |
| 在线单机示例及验证 | 通过 | 检查通过。 |
| 在线批量示例 | 通过 | 检查通过。 |
| strict 单机规划及验证 | 通过 | 检查通过。 |
| prefer 单机规划及验证 | 通过 | 检查通过。 |
| free 单机规划及验证 | 通过 | 检查通过。 |
| 完整内嵌批量请求 | 通过 | 检查通过。 |
| 场景登记、查询及引用规划 | 通过 | 检查通过。 |
| 坐标双向转换 | 通过 | 检查通过。 |
| 归档运动段独立验证 | 通过 | 检查通过。 |
| 异步提交及结果轮询 | 通过 | 检查通过。 |
| 未知字段返回 400 | 通过 | 检查通过。 |
| 未知场景返回 404 | 通过 | 检查通过。 |
| 未知作业返回 404 | 通过 | 检查通过。 |
| 航路图权重规划及验证 | 通过 | 检查通过。 |
| 航路图连通性分析 | 通过 | 检查通过。 |
| 网格端点、搜索配置及完整网格序列 | 通过 | 检查通过。 |
| 批量并行与统一避让 | 通过 | 检查通过。 |
| 多目标、能耗及自定义代价方案比较 | 通过 | 检查通过。 |
| 增量重规划、零搜索复用及初始速度验证 | 通过 | 检查通过。 |
| strict 1000 任务规划及验证 | 通过 | 检查通过。 |
| mixed 1000 任务规划及验证 | 通过 | 检查通过。 |
| free 1000 任务规划及验证 | 通过 | 检查通过。 |

## 文件

- `report.json` 保存测试结果、耗时、接口覆盖和交换记录。
- `http/*_request.json` 保存 HTTP 方法、地址、请求头和输入文件路径。
- `http/*_input.json` 保存实际发送的完整 JSON 输入。
- `http/*_output.json`、`http/*_output.html` 保存实际收到的完整响应。
- `REPORT.md` 保存本报告。

源码回归见 [SOURCE_TESTS.md](SOURCE_TESTS.md)。全部 3000 条导出路线的独立复核见 [INDEPENDENT_VALIDATION.md](INDEPENDENT_VALIDATION.md)。

正式服务切换后的本地接口检查见 [production_smoke/REPORT.md](production_smoke/REPORT.md)。公网检查见 [公网报告](../20261030_engine_complete_public/REPORT.md)。
