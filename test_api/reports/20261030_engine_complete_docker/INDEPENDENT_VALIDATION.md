# 全部导出路线独立复核

文档日期：2026-10-30。

验证镜像提交：`b5c8b8b68fab15c333c955c8fb72925edd95a8d2`。验证在同一 2 CPU、2 GiB 候选容器内执行，使用安装好的 SDK 和现有 `tools/benchmark_v2.py --functional-only --scales 1000 --repeats 1`。

校验器从公开 JSON 重建解析运动学证书，逐条检查端点、时间窗和规则，再检查全部路线的连续机间冲突。完整输入与输出保存在 independent_validation/payloads/；三个报告记录校验和、运行环境及验证数。

| 数据集 | 有效路线 | 独立运动学证书 | 连续冲突 | 批量 HTTP 秒数 |
|---|---:|---:|---:|---:|
| strict | 1000 | 1000 | 0 | 7.750 |
| mixed | 1000 | 1000 | 0 | 14.506 |
| free | 1000 | 1000 | 0 | 7.462 |

批量 HTTP 耗时包括请求创建/序列化、规划和完整响应读取/JSON 解析；归档写入与后续独立复核单独执行。容器 cgroup 记录峰值内存 439.36 MiB，OOM 为 0。原始资源记录见 environment.json 和 cgroup.json。
