# 接口测试记录

文档日期：2026-10-30。

最新完整容器测试：[20261030_engine_complete_docker/REPORT.md](20261030_engine_complete_docker/REPORT.md)。配置为 2 CPU、2 GiB，检查全部 16 个接口、5 份新增功能输入和三套各 1000 个任务。源码回归及环境记录随该目录归档。

最新公网接口测试：[20261030_engine_complete_public/REPORT.md](20261030_engine_complete_public/REPORT.md)。核对运行版本、16 个接口及图规划、网格、多目标、并行、增量重规划和独立速度验证。公网采用接口示例集；完整 3000 任务记录见上面的容器报告。

## 历史记录

- [20261030_public_verified](20261030_public_verified/REPORT.md) 与 [mixed 复测](20261030_public_mixed_retry/REPORT.md)：旧契约的公网完整任务记录；mixed 使用默认 120 秒客户端等待时间后完成。
- [20261029_docker](20261029_docker/REPORT.md)：旧契约 2 CPU、2 GiB 容器的 22 项检查及三套各 1000 个任务。
- [20261029_local](20261029_local/REPORT.md)：旧契约本地 Python 测试。
- [20261017 公网](20261017T135804Z_public/REPORT.md)：早期接口版本与原始数据。

每组目录保存 REPORT.md、report.json 和 http/。http/ 保存实际请求、完整输入和完整响应。report.json 记录输入输出的 SHA-256 和运行版本。历史内容按采集版本保留。
