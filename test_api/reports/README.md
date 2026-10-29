# 接口测试记录

文档日期：2026-10-29。

最新本地记录：[20261029_local/REPORT.md](20261029_local/REPORT.md)。本地 Python 服务通过 22 项检查，覆盖全部 14 个 v2 接口。strict、mixed、free 各完成 1000 个任务，失败数和冲突数均为 0。该次测试未设置容器资源限额。

历史公网记录：[20261017T135804Z_public/REPORT.md](20261017T135804Z_public/REPORT.md)。保留原测试日期和接口版本。

每组目录保存 `REPORT.md`、`report.json` 和 `http/`。`http/` 保存实际请求、完整输入和完整响应。`report.json` 记录输入与响应的 SHA-256。
