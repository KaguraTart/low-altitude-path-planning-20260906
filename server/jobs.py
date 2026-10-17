"""进程内异步作业。

通过 POST /api/v2/jobs 提交，通过 GET /api/v2/jobs/{job_id} 查询。
完整结果保留在内存中，服务重启后清空。
"""

from __future__ import annotations

import threading
import uuid
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from typing import Any, Callable, Dict, Optional


class JobRegistry:
    """线程安全的进程内作业表"""

    def __init__(self, max_workers: int = 2):
        """
        Args:
            max_workers: 同时执行的作业数。规划本身是 CPU 密集且内部已有
                         多线程并行，这里开太大会互相抢核，反而更慢。
        """
        self._max_workers = max(1, int(max_workers))
        self._executor = self._new_executor()
        self._jobs: Dict[str, Dict[str, Any]] = {}
        self._lock = threading.Lock()
        self._accepting = True

    def _new_executor(self) -> ThreadPoolExecutor:
        return ThreadPoolExecutor(max_workers=self._max_workers, thread_name_prefix="uav-job")

    def start(self) -> None:
        """Allow a fresh application lifecycle after the previous one drained."""
        with self._lock:
            if not self._accepting:
                self._executor = self._new_executor()
                self._accepting = True

    # ------------------------------------------------------------ 提交

    def submit(self, fn: Callable[..., Any], *args, **kwargs) -> str:
        """
        提交一个后台作业，立即返回 ``job_id``

        ``fn`` 抛出的异常会被捕获并记入作业的 ``error`` 字段，
        不会冒泡到线程池（否则只会打印到 stderr，轮询方看不到原因）。
        """
        job_id = uuid.uuid4().hex[:16]
        record: Dict[str, Any] = {
            "job_id": job_id,
            "status": "pending",
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "started_at": None,
            "finished_at": None,
            "elapsed_ms": None,
            "result": None,
            "error": None,
        }
        with self._lock:
            from uav_planner import ErrorCode, PlanningError

            if not self._accepting:
                raise PlanningError(ErrorCode.JOB_QUEUE_FULL, "Service is shutting down",
                                    {"accepting": False})
            self._jobs[job_id] = record
            try:
                self._executor.submit(self._run, job_id, fn, args, kwargs)
            except Exception:
                self._jobs.pop(job_id, None)
                raise
        return job_id

    def _run(self, job_id: str, fn: Callable, args: tuple, kwargs: dict):
        started = datetime.now()
        with self._lock:
            rec = self._jobs.get(job_id)
            if rec is None or rec["status"] != "pending":
                return
            rec["status"] = "running"
            rec["started_at"] = started.isoformat(timespec="seconds")

        try:
            result = fn(*args, **kwargs)
            error = None
        except Exception as e:  # noqa: BLE001 - 作业边界需要兜住一切异常
            from uav_planner import ErrorCode, PlanningError
            from uav_planner.v2.native import NativeUnavailableError

            result = None
            error = (
                e.to_dict()["error"] if isinstance(e, PlanningError)
                else {"code": (ErrorCode.RESOURCE_EXHAUSTED if isinstance(e, MemoryError)
                               else ErrorCode.NATIVE_UNAVAILABLE if isinstance(e, NativeUnavailableError)
                               else ErrorCode.INTERNAL_ERROR),
                      "message": "Memory allocation failed" if isinstance(e, MemoryError) else str(e),
                      "detail": {"type": type(e).__name__}}
            )

        finished = datetime.now()
        with self._lock:
            rec = self._jobs.get(job_id)
            if rec is None:
                return
            rec["finished_at"] = finished.isoformat(timespec="seconds")
            rec["elapsed_ms"] = round((finished - started).total_seconds() * 1000, 2)
            if error is not None:
                rec["status"] = "failed"
                rec["error"] = error
            else:
                rec["status"] = "succeeded"
                rec["result"] = result

    # ------------------------------------------------------------ 查询

    def get(self, job_id: str) -> Optional[Dict[str, Any]]:
        """取作业快照；不存在返回 ``None``"""
        with self._lock:
            rec = self._jobs.get(job_id)
            return dict(rec) if rec is not None else None

    def stats(self) -> Dict[str, int]:
        """各状态作业计数（供 /health 观测积压情况）"""
        with self._lock:
            out = {"pending": 0, "running": 0, "succeeded": 0, "failed": 0}
            for rec in self._jobs.values():
                out[rec["status"]] = out.get(rec["status"], 0) + 1
            out["total"] = len(self._jobs)
            return out

    # ------------------------------------------------------------ 内部

    def shutdown(self, wait: bool = False) -> None:
        with self._lock:
            self._accepting = False
            executor = self._executor
            for rec in self._jobs.values():
                if rec["status"] == "pending":
                    rec["status"] = "failed"
                    rec["finished_at"] = datetime.now().isoformat(timespec="seconds")
                    rec["error"] = {"code": "JOB_CANCELLED", "message": "Service shutdown cancelled the queued job", "detail": {}}
        executor.shutdown(wait=wait, cancel_futures=True)
