"""Reports describe recorded outcomes and preserve useful failure details."""

from copy import deepcopy

import pytest

from test_api.reporting import write_report


@pytest.fixture
def report():
    return {
        "base_url": "https://api.kaguratart.com",
        "started_at": "2026-10-17T12:00:00Z",
        "finished_at": "2026-10-17T12:00:15+00:00",
        "status": "passed",
        "service_version": {"api_version": "2.0.0", "engine": {"git_sha": "a" * 40}},
        "summary": {"passed": 3, "failed": 0, "covered_operations": 14, "operation_count": 14},
        "coverage": [{"method": "GET", "path": "/api/v2/health", "status": "passed"}],
        "checks": [{"name": "健康检查", "status": "passed", "message": "HTTP 200"}],
        "benchmarks": [{"suite": "strict", "task_count": 1000, "success_count": 1000,
                        "failed_count": 0, "conflict_count": 0, "elapsed_ms": 1234.5,
                        "response_path": "http/strict_batch_output.json"}],
    }


def test_report_preserves_version_results_and_beijing_time(report, tmp_path):
    path = write_report(report, tmp_path / "delivery")
    text = path.read_text(encoding="utf-8")
    assert path.name == "REPORT.md"
    assert "2026-10-17 20:00:00 +0800" in text
    assert "2026-10-17 20:00:15 +0800" in text
    assert "`" + "a" * 40 + "`" in text
    assert "接口覆盖 14 / 14 项" in text
    assert "| strict | 1000 | 1000 | 0 | 0 | 1234.5 |" in text
    assert "http/strict_batch_output.json" in text


def test_failed_report_keeps_observed_error_without_success_claim(report, tmp_path):
    failed = deepcopy(report)
    failed.update(status="failed", benchmarks=[])
    failed["summary"].update(passed=1, failed=1)
    failed["checks"] = [{"name": "提交任务", "status": "failed",
                         "message": "HTTP 502 | upstream\nconnection reset"}]
    text = write_report(failed, tmp_path).read_text(encoding="utf-8")
    assert "总体结果：**失败**" in text
    assert "检查通过 1 项，失败 1 项" in text
    assert r"HTTP 502 \| upstream connection reset" in text
    assert "本次运行未提交三套 1000 任务数据" in text


def test_report_rejects_timestamp_without_timezone(report, tmp_path):
    report["started_at"] = "2026-10-17T12:00:00"
    with pytest.raises(ValueError, match="时区"):
        write_report(report, tmp_path)
