#!/usr/bin/env python3
"""Package current API inputs, documentation and selected test reports."""
from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--report", type=Path, action="append", default=[], metavar="DIRECTORY",
        help="Include a test run containing REPORT.md, report.json and http/; repeat for each run.",
    )
    args = parser.parse_args()
    kit = ROOT / "test_api"
    files = sorted(path for path in kit.rglob("*") if path.is_file()
                   and not any(part in {"__pycache__", "results", "reports"} or part.startswith(".")
                               for part in path.relative_to(kit).parts))
    names = set()
    for report in args.report:
        if not report.is_dir() or not (report / "REPORT.md").is_file() or not (report / "report.json").is_file() or not (report / "http").is_dir():
            parser.error(f"Invalid report directory: {report}")
        if report.name in names:
            parser.error(f"Duplicate report directory name: {report.name}")
        names.add(report.name)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with ZipFile(args.output, "w", compression=ZIP_DEFLATED, compresslevel=9) as archive:
        for path in files:
            archive.write(path, "test_api/" + path.relative_to(kit).as_posix())
        index = ["# 接口测试记录", "", "以各报告中的测试日期和构建提交识别运行版本。", ""]
        for report in args.report:
            index.append(f"- [{report.name}]({report.name}/REPORT.md)：完整请求、响应及测试结果。")
            for path in sorted(report.rglob("*")):
                if path.is_file() and not any(part.startswith(".") or part == "__pycache__"
                                              for part in path.relative_to(report).parts):
                    archive.write(path, f"test_api/reports/{report.name}/{path.relative_to(report).as_posix()}")
                    files.append(path)
        if not args.report:
            index.append("本包未附测试报告。运行测试脚本后，报告保存在指定的输出目录。")
        archive.writestr("test_api/reports/README.md", "\n".join(index) + "\n")
        files.append(kit / "reports" / "README.md")
    digest = hashlib.sha256(args.output.read_bytes()).hexdigest()
    args.output.with_suffix(args.output.suffix + ".sha256").write_text(f"{digest}  {args.output.name}\n")
    print(f"{len(files)} files; {args.output.stat().st_size} bytes; SHA-256 {digest}")


if __name__ == "__main__":
    main()
