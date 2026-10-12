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
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from typing import Any, Callable, Dict, Optional


class JobRegistry:
    """线程安全的进程内作业表"""

    def __init__(self, max_workers: int = 2, max_jobs: int = 200):
        """
        Args:
            max_workers: 同时执行的作业数。规划本身是 CPU 密集且内部已有
                         多线程并行，这里开太大会互相抢核，反而更慢。
            max_jobs: 保留的作业上限，超出后淘汰最旧的**已完成**作业，
                      防止长跑服务把内存吃满。
        """
        self._executor = ThreadPoolExecutor(
            max_workers=max(1, int(max_workers)),
            thread_name_prefix="uav-job",
        )
        self._jobs: Dict[str, Dict[str, Any]] = {}
        self._order: list = []
        self._lock = threading.Lock()
        self._max_jobs = max(1, int(max_jobs))

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
            self._jobs[job_id] = record
            self._order.append(job_id)
            self._evict_locked()

        self._executor.submit(self._run, job_id, fn, args, kwargs)
        return job_id

    def _run(self, job_id: str, fn: Callable, args: tuple, kwargs: dict):
        started = datetime.now()
        with self._lock:
            rec = self._jobs.get(job_id)
            if rec is None:            # 已被淘汰
                return
            rec["status"] = "running"
            rec["started_at"] = started.isoformat(timespec="seconds")

        try:
            result = fn(*args, **kwargs)
            error = None
        except Exception as e:  # noqa: BLE001 - 作业边界需要兜住一切异常
            from uav_planner import PlanningError

            result = None
            error = (
                e.to_dict()["error"] if isinstance(e, PlanningError)
                else {"code": "INTERNAL_ERROR", "message": str(e), "detail": {}}
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

    def _evict_locked(self):
        """淘汰最旧的已完成作业（调用方须持有锁）"""
        while len(self._order) > self._max_jobs:
            for idx, jid in enumerate(self._order):
                rec = self._jobs.get(jid)
                if rec is not None and rec["status"] in ("succeeded", "failed"):
                    self._jobs.pop(jid, None)
                    self._order.pop(idx)
                    break
            else:
                # 全是未完成的作业，暂不淘汰，等它们结束再说
                return

    def shutdown(self, wait: bool = False):
        self._executor.shutdown(wait=wait)
