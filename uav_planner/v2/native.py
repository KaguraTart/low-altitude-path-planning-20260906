"""Validated array boundary for the required C++17 planning engine.

Build with ``uv pip install -e .``. An unavailable extension is an explicit
operational error; calls never silently run a slower Python implementation.
Times are continuous seconds and positions are metric local coordinates.
"""
from __future__ import annotations

import importlib
from dataclasses import asdict, is_dataclass
from typing import Any, Iterable, Sequence

import numpy as np

try:
    _engine = importlib.import_module("uav_planner.v2._native")
    _import_error: BaseException | None = None
except (ImportError, OSError) as exc:
    _engine = None
    _import_error = exc


class NativeUnavailableError(RuntimeError):
    """The compiled engine is missing or incompatible with this interpreter."""


def available() -> bool:
    return _engine is not None


def require_native() -> Any:
    if _engine is None:
        raise NativeUnavailableError(
            "V2 requires its compiled C++17 engine. Install the package with "
            "`uv pip install -e .` using a C++17 compiler and CMake. "
            f"Import error: {_import_error}"
        ) from _import_error
    return _engine


def engine_info() -> dict[str, Any]:
    return {
        "available": available(),
        "engine": "cpp17-pybind11",
        "version": getattr(_engine, "ENGINE_VERSION", None),
        "error": str(_import_error) if _import_error else None,
    }


def _matrix(value: Any, width: int, dtype: Any = np.float64) -> np.ndarray:
    if dtype == np.int64:
        original = np.asarray(value)
        if original.size and (
            not np.all(np.isfinite(original))
            or not np.all(original == np.floor(original))
        ):
            raise ValueError("node and voxel indices must be finite integers")
    array = np.asarray(value, dtype=dtype)
    if array.size == 0:
        return np.empty((0, width), dtype=dtype)
    return np.ascontiguousarray(array)


def _trajectories(paths: Sequence[Any], times: Sequence[Any]) -> tuple[list[np.ndarray], list[np.ndarray]]:
    return (
        [_matrix(path, 3) for path in paths],
        [np.ascontiguousarray(time, dtype=np.float64) for time in times],
    )


def graph_search(
    nodes: Any, edges: Any, edge_costs: Any, start: int, goal: int, *,
    blocked_nodes: Iterable[int] = (), max_iterations: int = 200_000,
    deadline_ms: float = 0, directed: bool = False,
) -> dict[str, Any]:
    return require_native().graph_search(
        _matrix(nodes, 3), _matrix(edges, 2, np.int64),
        np.ascontiguousarray(edge_costs, dtype=np.float64), start, goal,
        list(blocked_nodes), max_iterations, deadline_ms, directed,
    )


def grid_search(
    start: Any, goal: Any, bounds: Any, resolution: Any, blocked: Any = (), *,
    allowed: Any | None = None, max_iterations: int = 200_000,
    deadline_ms: float = 0,
) -> dict[str, Any]:
    """26-neighbor A* with all diagonal subcells checked for corner cutting.

    Grid coordinate (i,j,k) is at bounds[0] + resolution * (i,j,k).
    Exact requested endpoints are preserved in the returned polyline.
    """
    spacing = np.asarray(resolution, dtype=float)
    if spacing.ndim == 0:
        spacing = np.repeat(spacing, 3)
    return require_native().grid_search(
        start, goal, _matrix(bounds, 3), spacing.tolist(),
        _matrix(blocked, 3, np.int64),
        None if allowed is None else _matrix(allowed, 3, np.int64),
        max_iterations, deadline_ms,
    )


def continuous_conflicts(
    paths: Sequence[Any], times: Sequence[Any], horizontal_separation: float,
    vertical_separation: float, *, max_conflicts: int = 10_000,
    deadline_ms: float = 0,
) -> dict[str, Any]:
    """Find continuous cylinder conflicts using a sparse 4-axis segment BVH.

    A conflict requires BOTH insufficient horizontal and vertical separation.
    Single-point trajectories exist only at their timestamp; hover requires
    two equal positions at strictly increasing timestamps. Separation equality
    is conservatively considered a conflict with a numerical 1e-8 margin.
    """
    ps, ts = _trajectories(paths, times)
    return require_native().continuous_conflicts(
        ps, ts, horizontal_separation, vertical_separation,
        max_conflicts, deadline_ms,
    )


def schedule_paths(
    paths: Sequence[Any], times: Sequence[Any], departure_windows: Any, *,
    horizontal_separation: float, vertical_separation: float,
    priorities: Sequence[int] | None = None, deadline_ms: float = 0,
    fixed_count: int = 0,
) -> dict[str, Any]:
    """Prioritized continuous reservations with exact forbidden delay jumps.

    Windows contain absolute earliest/latest departure timestamps. Returned
    delays are shifts relative to each input trajectory's first timestamp.
    None means the trajectory was not reserved; inspect its explicit status.
    Higher numeric priorities reserve first, with stable input-order ties.
    The first ``fixed_count`` inputs are mandatory observed obstacles, registered
    at zero delay before all new tasks even if they conflict with one another.
    Their output status is ``fixed`` and they count toward ``accepted``.
    """
    ps, ts = _trajectories(paths, times)
    return require_native().schedule_paths(
        ps, ts, _matrix(departure_windows, 2), horizontal_separation,
        vertical_separation, None if priorities is None else list(priorities),
        deadline_ms, fixed_count,
    )


def solve_sipp(
    nodes: Any, edges: Any, durations: Any, start: int, goal: int, *,
    reservations_paths: Sequence[Any] = (),
    reservations_times: Sequence[Any] = (),
    horizontal_separation: float = 0, vertical_separation: float = 0,
    start_time: float = 0, latest_arrival: float = float("inf"),
    max_iterations: int = 200_000, deadline_ms: float = 0,
    directed: bool = False, max_turn_angle: float = float(np.pi),
    wait_allowed: Sequence[bool] | None = None,
    edge_paths: Sequence[Any] | None = None,
    edge_times: Sequence[Any] | None = None,
) -> dict[str, Any]:
    """Safe interval search on a sparse graph with continuous edge reservations.

    Durations must be positive for traversable edges. Waiting stays inside a
    node's safe interval, exact edge swaps/crossings are checked, and returned
    repeated nodes explicitly encode waits. Starting before the first safe
    interval delays launch; the vehicle is not reserved before that launch.
    ``max_turn_angle`` is radians and adds incoming-heading search state.
    ``wait_allowed`` contains one boolean per node and defaults to all true.
    False forbids departure after arrival, including waits for another node's
    safe interval. Distinct arrivals are retained at such nodes, but delays are
    not propagated back across no-wait chains: contract continuous motion into
    shaped edges between permitted stops when that behavior is required.
    ``edge_paths`` and ``edge_times`` optionally give each edge's full timed
    polyline, with relative times from zero to its duration and endpoints equal
    to its source/destination nodes. Reverse traversal reverses its time law.
    Returned ``edge_ids`` identify traversed input edges, excluding waits;
    ``path`` and ``times`` contain graph nodes and explicit repeated-node waits.
    """
    ps, ts = _trajectories(reservations_paths, reservations_times)
    if (edge_paths is None) != (edge_times is None):
        raise ValueError("edge_paths and edge_times must be supplied together")
    shaped_paths, shaped_times = (
        (None, None) if edge_paths is None else _trajectories(edge_paths, edge_times)
    )
    wait_mask = None if wait_allowed is None else np.asarray(wait_allowed)
    if wait_mask is not None and (wait_mask.ndim != 1 or wait_mask.dtype != np.bool_):
        raise ValueError("wait_allowed must contain one boolean per node")
    return require_native().solve_sipp(
        _matrix(nodes, 3), _matrix(edges, 2, np.int64),
        np.ascontiguousarray(durations, dtype=np.float64), start, goal,
        ps, ts, horizontal_separation, vertical_separation, start_time,
        latest_arrival, max_iterations, deadline_ms, directed, max_turn_angle,
        wait_mask, shaped_paths, shaped_times,
    )


def validate_motion(
    primitives: Sequence[Any], limits: dict[str, float], *,
    starts: Sequence[float] | None = None,
) -> dict[str, Any]:
    """Independently validate exact line/helix trapezoidal motion in C++.

    Checks geometry, integrated phase distances, phase speed changes, temporal
    and vector velocity continuity, peak horizontal/climb/descent speed, yaw,
    positive turn radius, and combined tangential/centripetal acceleration.
    ``yaw`` primitives must carry their actual signed angle in ``sweep``.
    """
    records = []
    for primitive in primitives:
        record = asdict(primitive) if is_dataclass(primitive) else dict(primitive)
        for field in ("start", "end", "center"):
            if record.get(field) is not None:
                record[field] = np.asarray(record[field], dtype=float).tolist()
        records.append(record)
    return require_native().validate_motion(
        records, dict(limits), [] if starts is None else list(starts),
    )


def validate_trajectory(path: Any, times: Any, limits: dict[str, float]) -> dict[str, Any]:
    """Check sampled velocities and exact three-point horizontal circumradii.

    This validates the supplied samples. Use ``validate_motion`` to certify
    unsampled analytic peak speeds and accelerations between those samples.
    """
    return require_native().validate_trajectory(
        _matrix(path, 3), np.ascontiguousarray(times, dtype=np.float64), dict(limits),
    )
