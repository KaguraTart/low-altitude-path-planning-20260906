"""
异步作业注册表

**为什么需要它**：本项目压测实测 10000 个任务的批量规划耗时约 437 秒
（见 ``output/stress/STRESS_TEST_REPORT.md``），远超任何合理的 HTTP
请求超时。同步端点只适合中小批量联调，大批量必须走异步：

    POST /api/v1/jobs          → 立即返回 job_id
    GET  /api/v1/jobs/{id}     → 轮询取状态与结果

**进程内内存态**：作业表存在本进程的字典里，不做持久化。这意味着
服务重启后历史作业会丢失、多 worker 部署时轮询可能落到没有该作业的
进程上。联调阶段够用；上生产需要换成 Redis/数据库 + 独立 worker，
届时替换本模块实现即可，``app.py`` 的调用方式不用改。
"""

from __future__ import annotations

import threading
import uuid
from sys import getsizeof
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from typing import Any, Callable, Dict, Optional


class JobRegistry:
    """线程安全的进程内作业表"""

    def __init__(self, max_workers: int = 2, max_jobs: int = 200,
                 max_completed_bytes: int = 128 * 1024 * 1024):
        """
        Args:
            max_workers: 同时执行的作业数。规划本身是 CPU 密集且内部已有
                         多线程并行，这里开太大会互相抢核，反而更慢。
            max_jobs: 未完成作业的接收上限和历史记录保留上限。
            max_completed_bytes: 已完成结果的内存估算上限；超出时淘汰旧结果。
        """
        self._max_workers = max(1, int(max_workers))
        self._executor = self._new_executor()
        self._jobs: Dict[str, Dict[str, Any]] = {}
        self._order: list = []
        self._lock = threading.Lock()
        self._max_jobs = max(1, int(max_jobs))
        self._max_completed_bytes = max(1, int(max_completed_bytes))
        self._result_sizes: Dict[str, int] = {}
        self._completed_bytes = 0
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

            active = sum(rec["status"] in ("pending", "running") for rec in self._jobs.values())
            if not self._accepting or active >= self._max_jobs:
                raise PlanningError(ErrorCode.JOB_QUEUE_FULL, "Background job queue is full or shutting down",
                                    {"active_jobs": active, "limit": self._max_jobs,
                                     "accepting": self._accepting})
            self._jobs[job_id] = record
            self._order.append(job_id)
            self._evict_locked()
            self._executor.submit(self._run, job_id, fn, args, kwargs)
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
                else {"code": ErrorCode.NATIVE_UNAVAILABLE if isinstance(e, NativeUnavailableError) else ErrorCode.INTERNAL_ERROR,
                      "message": str(e), "detail": {}}
            )

        result_bytes = _retained_size(result) if error is None else 0
        if result_bytes > self._max_completed_bytes:
            from uav_planner import ErrorCode
            result = None
            error = {"code": ErrorCode.JOB_RESULT_TOO_LARGE,
                     "message": "Background result exceeds the retained memory budget",
                     "detail": {"estimated_bytes": result_bytes, "limit_bytes": self._max_completed_bytes}}
            result_bytes = 0
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
                self._result_sizes[job_id] = result_bytes
                self._completed_bytes += result_bytes
            self._evict_locked()

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

    def _evict_locked(self):
        """淘汰最旧的已完成作业（调用方须持有锁）"""
        while len(self._order) > self._max_jobs or self._completed_bytes > self._max_completed_bytes:
            for idx, jid in enumerate(self._order):
                rec = self._jobs.get(jid)
                if rec is not None and rec["status"] in ("succeeded", "failed"):
                    self._jobs.pop(jid, None)
                    self._order.pop(idx)
                    self._completed_bytes -= self._result_sizes.pop(jid, 0)
                    break
            else:
                # 全是未完成的作业，暂不淘汰，等它们结束再说
                return

    def shutdown(self, wait: bool = False) -> None:
        with self._lock:
            self._accepting = False
            executor = self._executor
            for rec in self._jobs.values():
                if rec["status"] == "pending":
                    rec["status"] = "failed"
                    rec["finished_at"] = datetime.now().isoformat(timespec="seconds")
                    rec["error"] = {"code": "JOB_CANCELLED", "message": "Service shutdown cancelled the queued job", "detail": {}}
            self._evict_locked()
        executor.shutdown(wait=wait, cancel_futures=True)


def _retained_size(value: Any) -> int:
    """Estimate a JSON-shaped result without allocating a serialized copy."""
    seen: set[int] = set()
    pending = [value]
    total = 0
    while pending:
        item = pending.pop()
        identity = id(item)
        if identity in seen:
            continue
        seen.add(identity)
        total += getsizeof(item)
        if isinstance(item, dict):
            pending.extend(item.keys())
            pending.extend(item.values())
        elif isinstance(item, (list, tuple, set, frozenset)):
            pending.extend(item)
    return total
