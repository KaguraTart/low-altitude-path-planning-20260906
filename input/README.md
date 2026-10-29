# 输入数据

当前冻结基准：`engine_contract_20261029/`。坐标为 WGS84，空域高度为 EG。种子为 20261016；每套有 1000 个任务。

`v2_benchmarks/` 保存 2026-10-16 的原始基准，用于核对历史报告。它使用旧高度合同。当前服务测试使用新目录。

对外接口请求位于 `test_api/data/`。运行 `python tools/update_contract_fixtures.py` 可从当前模型和冻结输入重建测试包请求。
