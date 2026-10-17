"""Send HTTP requests and save the complete request and response."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import re
import time
from typing import Any
from urllib.error import HTTPError
from urllib.parse import urlsplit
from urllib.request import Request, urlopen


USER_AGENT = "UAV-API-Test/1.0"


def normalize_base_url(value: str) -> str:
    """Accept an HTTP service origin or its Swagger URL."""
    parsed = urlsplit(value)
    if parsed.scheme not in {"http", "https"} or not parsed.netloc:
        raise ValueError("base URL must start with http:// or https://")
    if parsed.username is not None or parsed.password is not None:
        raise ValueError("Use a base URL without credentials")
    if parsed.path.rstrip("/") not in {"", "/docs"}:
        raise ValueError("Use the service origin or its /docs URL")
    return f"{parsed.scheme}://{parsed.netloc}"


def save_bytes(path: Path, content: bytes) -> dict[str, Any]:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(content)
    return {"path": path.name, "bytes": len(content), "sha256": hashlib.sha256(content).hexdigest()}


class HTTPClient:
    """Archive each exchange before checking its HTTP status."""

    def __init__(self, base_url: str, output_dir: Path, timeout: float = 120) -> None:
        self.base_url = normalize_base_url(base_url)
        self.output_dir = output_dir
        self.timeout = timeout
        self.exchanges: list[dict[str, Any]] = []

    def request(self, method: str, path: str, body: Any = None, *,
                expected_status: int = 200, as_text: bool = False) -> Any:
        if not path.startswith("/") or path.startswith("//"):
            raise ValueError("Request path must start with one slash")
        data = None if body is None else json.dumps(body, ensure_ascii=False, allow_nan=False).encode("utf-8")
        headers = {"User-Agent": USER_AGENT, "Accept": "application/json"}
        if data is not None:
            headers["Content-Type"] = "application/json"
        number = len(self.exchanges) + 1
        slug = re.sub(r"[^A-Za-z0-9_-]", "_", path.strip("/"))
        stem = f"{number:03d}_{method.lower()}_{slug}"
        descriptor = {"method": method, "url": self.base_url + path, "path": path, "headers": headers,
                      "body_path": f"{stem}_input.json" if data is not None else None}
        descriptor_path = self.output_dir / "http" / f"{stem}_request.json"
        save_bytes(descriptor_path, (json.dumps(descriptor, ensure_ascii=False, indent=2) + "\n").encode("utf-8"))
        exchange: dict[str, Any] = {"method": method, "path": path,
                                    "request_path": str(descriptor_path.relative_to(self.output_dir)),
                                    "expected_status": expected_status}
        if data is not None:
            input_path = descriptor_path.with_name(f"{stem}_input.json")
            exchange["input"] = save_bytes(input_path, data)
            exchange["input_path"] = str(input_path.relative_to(self.output_dir))
        self.exchanges.append(exchange)
        started = time.perf_counter()
        try:
            req = Request(self.base_url + path, data=data, headers=headers, method=method)
            try:
                with urlopen(req, timeout=self.timeout) as response:
                    raw = response.read()
                    status = response.status
                    response_headers = response.headers
            except HTTPError as exc:
                raw = exc.read()
                status = exc.code
                response_headers = exc.headers
            exchange["elapsed_ms"] = round((time.perf_counter() - started) * 1000, 3)
            exchange["http_status"] = status
            exchange["headers"] = {key: response_headers[key] for key in
                                   ("Content-Type", "Content-Length", "Date", "Server")
                                   if response_headers is not None and key in response_headers}
            output_path = descriptor_path.with_name(f"{stem}_output.{ 'html' if as_text else 'json' }")
            exchange["output"] = save_bytes(output_path, raw)
            exchange["response_path"] = str(output_path.relative_to(self.output_dir))
            if status != expected_status:
                raise AssertionError(f"{method} {path}: expected HTTP {expected_status}, received {status}")
            result = raw.decode("utf-8") if as_text else json.loads(raw)
            exchange["status"] = "passed"
            return result
        except Exception as exc:
            exchange["elapsed_ms"] = round((time.perf_counter() - started) * 1000, 3)
            exchange["status"] = "failed"
            exchange["error"] = {"type": type(exc).__name__, "message": str(exc)}
            raise
