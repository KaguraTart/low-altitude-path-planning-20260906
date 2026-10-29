#!/usr/bin/env python3
"""Package current API inputs, documentation and complete recorded responses."""
from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    kit = ROOT / "test_api"
    files = sorted(path for path in kit.rglob("*") if path.is_file()
                   and "__pycache__" not in path.parts and "results" not in path.relative_to(kit).parts)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with ZipFile(args.output, "w", compression=ZIP_DEFLATED, compresslevel=9) as archive:
        for path in files:
            archive.write(path, "test_api/" + path.relative_to(kit).as_posix())
    digest = hashlib.sha256(args.output.read_bytes()).hexdigest()
    args.output.with_suffix(args.output.suffix + ".sha256").write_text(f"{digest}  {args.output.name}\n")
    print(f"{len(files)} files; {args.output.stat().st_size} bytes; SHA-256 {digest}")


if __name__ == "__main__":
    main()
