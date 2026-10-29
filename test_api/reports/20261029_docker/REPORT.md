# 接口测试报告

文档日期：2026-10-30。

运行环境：Docker，2 CPU、2 GiB 内存。

测试地址：http://127.0.0.1:18002

服务版本：`2.0.0`。构建提交：`7541aacb18fa1a022455211e7d043b9f8ea0b750`。

开始时间：2026-10-29 23:59:36 +0800。
结束时间：2026-10-30 00:00:07 +0800。
时间使用北京时间。
原始开始时间：`2026-10-29T15:59:36.162Z`。
原始结束时间：`2026-10-29T16:00:07.505Z`。

总体结果：**通过**。
检查通过 22 项，失败 0 项。
接口覆盖 14 / 14 项。

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

## 批量规划

| 数据集 | 任务数 | 成功数 | 失败数 | 冲突数 | HTTP 耗时 / ms | 完整响应 |
|---|---:|---:|---:|---:|---:|---|
| strict | 1000 | 1000 | 0 | 0 | 7522.074 | `http/033_post_api_v2_plan_batch_output.json` |
| mixed | 1000 | 1000 | 0 | 0 | 14251.081 | `http/036_post_api_v2_plan_batch_output.json` |
| free | 1000 | 1000 | 0 | 0 | 7042.145 | `http/039_post_api_v2_plan_batch_output.json` |

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
| strict 1000 任务规划及验证 | 通过 | 检查通过。 |
| mixed 1000 任务规划及验证 | 通过 | 检查通过。 |
| free 1000 任务规划及验证 | 通过 | 检查通过。 |

## 文件

- `report.json` 保存测试结果、耗时、接口覆盖和交换记录。
- `http/*_request.json` 保存 HTTP 方法、地址、请求头和输入文件路径。
- `http/*_input.json` 保存实际发送的完整 JSON 输入。
- `http/*_output.json`、`http/*_output.html` 保存实际收到的完整响应。
- `REPORT.md` 保存本报告。

## 正式部署复核

2026-10-30 将修正 OpenAPI 快照后的镜像 `b118f86df64dec85071cbd301ab9a5042fbdf17a` 部署到本地正式服务。坐标换算、批量规划、异步作业和运行规范一致性均通过。记录见 [deployment-check.json](deployment-check.json)。
