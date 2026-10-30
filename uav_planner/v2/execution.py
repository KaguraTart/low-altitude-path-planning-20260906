"""Bounded candidate preparation; every outcome feeds global reservations."""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from copy import copy
from threading import Lock, get_ident, local
from typing import Callable, TypeVar

from .models import ExecutionOptions, ExecutionReport

Outcome = TypeVar("Outcome")


def clone_worker_scene(compiled):
    """Share immutable geometry/grid arrays, isolate mutable memoization maps.

    One clone lives for a worker's whole request, rather than allocating a scene
    or a full grid for each task. Cached values are read-only after insertion.
    Back references must follow the clone, including the terrain callback.
    """
    worker = copy(compiled)
    for name, value in vars(compiled).copy().items():
        if name.endswith("_cache") and isinstance(value, dict):
            setattr(worker, name, value.copy())
    worker.rules = copy(compiled.rules)
    worker.rules.scene = worker
    if hasattr(compiled, "network"):
        worker.network = copy(compiled.network)
        worker.network.compiled = worker
    worker.frame = copy(compiled.frame)
    worker.frame.terrain_provider = worker._terrain_at_geographic
    return worker


def run_candidates(ordered, compiled, options: ExecutionOptions,
                   prepare: Callable[..., Outcome]) -> tuple[list[Outcome], ExecutionReport]:
    """Submit groups in stable order and retain their results in that order.

    A group is only a submission boundary. Reservation never occurs here. None
    means one group containing every task, with no implicit task count cap.
    """
    count = len(ordered)
    group_size = options.batch_size or count
    workers = min(options.workers, count, group_size)
    report = ExecutionReport(requested_workers=options.workers, actual_workers=workers,
                             batch_size=options.batch_size,
                             batches=(count + group_size - 1) // group_size,
                             candidate_count=0)
    if workers == 1:
        # Preserve the original serial path and its warmed scene caches.
        return [prepare(index, task, compiled) for index, task in ordered], report
    state = local()
    used_threads = set()
    used_lock = Lock()

    def work(item):
        if not hasattr(state, "scene"):
            state.scene = clone_worker_scene(compiled)
            with used_lock:
                used_threads.add(get_ident())
        return prepare(*item, state.scene)

    outcomes = []
    with ThreadPoolExecutor(max_workers=workers, thread_name_prefix="candidate") as pool:
        for offset in range(0, count, group_size):
            futures = [pool.submit(work, item) for item in ordered[offset:offset + group_size]]
            outcomes.extend(future.result() for future in futures)
    report.actual_workers = len(used_threads)
    return outcomes, report
