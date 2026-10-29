"""Write a readable report from recorded HTTP test results."""

from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any


def _text(value: Any) -> str:
    return str(value).replace("|", "\\|").replace("\r", " ").replace("\n", " ")


def _status(value: Any) -> str:
    return {
        "passed": "通过",
        "failed": "失败",
        "skipped": "跳过",
        "not_run": "未运行",
        "succeeded": "通过",
    }.get(str(value), _text(value))


def _beijing_time(value: Any) -> str:
    if not value:
        return "未记录"
    stamp = datetime.fromisoformat(str(value).replace("Z", "+00:00"))
    if stamp.tzinfo is None:
        raise ValueError("报告时间必须包含时区。")
    local_zone = timezone(timedelta(hours=8), "Asia/Shanghai")
    return stamp.astimezone(local_zone).strftime("%Y-%m-%d %H:%M:%S %z")


def _version(version: Any) -> tuple[str, str]:
    if isinstance(version, dict):
        engine = version.get("engine", {})
        if not isinstance(engine, dict):
            engine = {}
        return (
            _text(version.get("api_version", version.get("version", "未记录"))),
            _text(engine.get("git_sha", version.get("git_sha", "未记录"))),
        )
    return _text(version or "未记录"), "未记录"


def write_report(report: dict[str, Any], output_dir: Path) -> Path:
    """Save REPORT.md and return its path."""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    version, revision = _version(report.get("service_version"))
    summary = report.get("summary", {})
    lines = [
        "# 接口测试报告",
        "",
        f"文档日期：{_beijing_time(report.get('finished_at') or report.get('started_at')).split()[0]}。",
        "",
        f"测试地址：{_text(report.get('base_url', '未记录'))}",
        "",
        f"服务版本：`{version}`。构建提交：`{revision}`。",
        "",
        f"开始时间：{_beijing_time(report.get('started_at'))}。",
        f"结束时间：{_beijing_time(report.get('finished_at'))}。",
        "时间使用北京时间。",
        f"原始开始时间：`{_text(report.get('started_at', '未记录'))}`。",
        f"原始结束时间：`{_text(report.get('finished_at', '未记录'))}`。",
        "",
        f"总体结果：**{_status(report.get('status', 'not_run'))}**。",
        f"检查通过 {summary.get('passed', 0)} 项，失败 {summary.get('failed', 0)} 项。",
        f"接口覆盖 {summary.get('covered_operations', 0)} / "
        f"{summary.get('operation_count', 0)} 项。",
        "",
        "## 接口覆盖",
        "",
        "接口覆盖统计成功 HTTP 调用。检查记录列出业务验证结果。",
        "",
        "| 方法 | 接口 | 结果 |",
        "|---|---|---|",
    ]
    for item in report.get("coverage", []):
        lines.append(
            f"| {_text(item.get('method', ''))} | `{_text(item.get('path', ''))}` "
            f"| {_status(item.get('status', 'not_run'))} |"
        )
    lines += ["", "## 批量规划", ""]
    benchmarks = report.get("benchmarks", [])
    if benchmarks:
        lines += [
            "| 数据集 | 任务数 | 成功数 | 失败数 | 冲突数 | HTTP 耗时 / ms | 完整响应 |",
            "|---|---:|---:|---:|---:|---:|---|",
        ]
        for item in benchmarks:
            response_path = _text(item.get("response_path", "未记录"))
            lines.append(
                f"| {_text(item.get('suite', ''))} | {item.get('task_count', 0)} "
                f"| {item.get('success_count', 0)} | {item.get('failed_count', 0)} "
                f"| {item.get('conflict_count', 0)} | {item.get('elapsed_ms', 0)} "
                f"| `{response_path}` |"
            )
    else:
        lines.append("本次运行未提交三套 1000 任务数据。")
    lines += [
        "",
        "## 检查记录",
        "",
        "| 检查 | 结果 | 记录 |",
        "|---|---|---|",
    ]
    for item in report.get("checks", []):
        lines.append(
            f"| {_text(item.get('name', ''))} "
            f"| {_status(item.get('status', 'not_run'))} "
            f"| {_text(item.get('message', ''))} |"
        )
    lines += [
        "",
        "## 文件",
        "",
        "- `report.json` 保存测试结果、耗时、接口覆盖和交换记录。",
        "- `http/*_request.json` 保存 HTTP 方法、地址、请求头和输入文件路径。",
        "- `http/*_input.json` 保存实际发送的完整 JSON 输入。",
        "- `http/*_output.json`、`http/*_output.html` 保存实际收到的完整响应。",
        "- `REPORT.md` 保存本报告。",
        "",
    ]
    path = output_dir / "REPORT.md"
    path.write_text("\n".join(lines), encoding="utf-8")
    return path
