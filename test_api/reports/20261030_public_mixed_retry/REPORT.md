# 公网 mixed 批量复测

文档日期：2026-10-30。

测试地址：https://api.kaguratart.com。通过本机代理通道访问。客户端等待时间为 120 秒。

测试范围：mixed 1000 个任务及完整轨迹验证。其他接口见同日公网完整记录。

结果：passed。

批量结果：

```json
[
  {
    "suite": "mixed",
    "task_count": 1000,
    "success_count": 1000,
    "failed_count": 0,
    "conflict_count": 0,
    "elapsed_ms": 47159.686,
    "response_path": "http/002_post_api_v2_plan_batch_output.json",
    "status": "passed"
  }
]
```

完整请求与响应保存在 http/，完整测试元数据见 report.json。
