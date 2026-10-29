"""Native safety regressions exercise continuous motion between samples."""
from __future__ import annotations

import concurrent.futures
import math
import time

import numpy as np
import pytest

from uav_planner.v2 import native


pytestmark = pytest.mark.skipif(not native.available(), reason="build the required C++ extension first")


def conflicts(paths, times, h=0.1, v=0.1):
    return native.continuous_conflicts(paths, times, h, v)["conflicts"]


def test_native_engine_required_and_reported():
    assert native.engine_info()["engine"] == "cpp17-pybind11"
    assert native.require_native().ENGINE_VERSION


def test_edge_exchange_detected_between_sparse_endpoints():
    paths = [[[0, 0, 0], [10, 0, 0]], [[10, 0, 0], [0, 0, 0]]]
    records = conflicts(paths, [[0, 10], [0, 10]])
    assert records and records[0]["start"] < 5 < records[0]["end"]
    assert records[0]["min_horizontal"] == pytest.approx(0)


def test_crossing_midpoints_and_different_time_scales():
    paths = [[[-10, 0, 0], [10, 0, 0]], [[0, -5, 0], [0, 5, 0]]]
    assert conflicts(paths, [[0, 20], [5, 15]])
    assert not conflicts(paths, [[0, 20], [30, 40]])


def test_both_horizontal_and_vertical_separation_required():
    paths = [[[0, 0, 0], [10, 0, 0]], [[0, 0, 20], [10, 0, 20]]]
    assert not conflicts(paths, [[0, 10], [0, 10]], h=2, v=10)
    assert conflicts(paths, [[0, 10], [0, 10]], h=2, v=21)
    paths[1] = [[0, 20, 0], [10, 20, 0]]
    assert not conflicts(paths, [[0, 10], [0, 10]], h=10, v=2)


def test_horizontal_and_vertical_minima_at_different_times_do_not_conflict():
    # Horizontal separation is small near t=0; vertical separation near t=10.
    paths = [[[0, 0, 0], [0, 0, 0]], [[0, 0, 10], [10, 0, 0]]]
    assert not conflicts(paths, [[0, 10], [0, 10]], h=1, v=1)


def test_hover_and_single_instant_are_valid():
    assert conflicts([[[0, 0, 0], [0, 0, 0]], [[-1, 0, 0], [1, 0, 0]]], [[0, 10], [4, 6]])
    assert conflicts([[[0, 0, 0]], [[-1, 0, 0], [1, 0, 0]]], [[5], [4, 6]])
    assert not conflicts([[[0, 0, 0]], [[-1, 0, 0], [1, 0, 0]]], [[3], [4, 6]])


@pytest.mark.parametrize("times", [[[0, 0]], [[1, 0]], [[0, math.nan]]])
def test_nonincreasing_or_nonfinite_times_are_rejected(times):
    with pytest.raises(ValueError):
        conflicts([[[0, 0, 0], [1, 0, 0]]], times)


def test_graph_search_uses_alternative_route_and_admissible_cost_scale():
    nodes = [[0, 0, 0], [10, 0, 0], [0, 100, 0], [20, 0, 0]]
    edges = [[0, 1], [1, 3], [0, 2], [2, 3]]
    result = native.graph_search(nodes, edges, [10, 10, 1, 1], 0, 3)
    assert result["path"] == [0, 2, 3]
    assert result["cost"] == 2
    result = native.graph_search(nodes, edges, [10, 10, 1, 1], 0, 3, blocked_nodes=[2])
    assert result["path"] == [0, 1, 3]


def test_grid_exact_endpoints_and_no_cut_corner():
    result = native.grid_search([0.1, 0.1, 0], [2, 2, 0], [[0, 0, 0], [2, 2, 0]], 1)
    assert result["status"] == "ok"
    assert result["path"][0] == [0.1, 0.1, 0]
    assert result["path"][-1] == [2, 2, 0]
    result = native.grid_search([0, 0, 0], [1, 1, 0], [[0, 0, 0], [1, 1, 0]], 1, [[1, 0, 0], [0, 1, 0]])
    assert result["status"] == "no_path"
    assert not result["path"]


def test_grid_allowed_sparse_voxels():
    result = native.grid_search([0, 0, 0], [2, 0, 0], [[0, 0, 0], [2, 2, 2]], 1,
                                allowed=[[0, 0, 0], [1, 0, 0], [2, 0, 0]])
    assert result["path"] == [[0, 0, 0], [1, 0, 0], [2, 0, 0]]


def test_schedule_edge_exchange_produces_continuous_safe_departures():
    paths = [[[0, 0, 0], [10, 0, 0]], [[10, 0, 0], [0, 0, 0]]]
    times = [[0, 10], [0, 10]]
    result = native.schedule_paths(paths, times, [[0, 0], [0, 20]], horizontal_separation=1, vertical_separation=1)
    assert result["status"] == "ok"
    assert result["delays"][1] > 10
    shifted = [(np.array(t) + d).tolist() for t, d in zip(times, result["delays"])]
    assert not conflicts(paths, shifted, h=1, v=1)


def test_schedule_priority_windows_and_absolute_timestamps():
    paths = [[[0, 0, 0], [10, 0, 0]]] * 2
    result = native.schedule_paths(paths, [[100, 110], [100, 110]], [[100, 100], [100, 100]],
                                   horizontal_separation=1, vertical_separation=1, priorities=[0, 10])
    assert result["statuses"] == ["window_exhausted", "scheduled"]
    assert result["delays"] == [None, 0]
    assert result["status"] == "partial"


def test_sipp_wait_is_explicit_and_edge_is_checked():
    nodes = [[0, 0, 0], [10, 0, 0]]
    reserved = [[[5, -5, 0], [5, 5, 0]]]
    result = native.solve_sipp(nodes, [[0, 1]], [10], 0, 1,
                               reservations_paths=reserved, reservations_times=[[0, 10]],
                               horizontal_separation=1, vertical_separation=1, latest_arrival=30)
    assert result["status"] == "ok"
    assert result["path"] == [0, 0, 1]
    assert result["times"][1] > 0
    actual = [nodes[n] for n in result["path"]]
    assert not conflicts([actual, *reserved], [result["times"], [0, 10]], h=1, v=1)


def test_sipp_safe_interval_blocked_goal_and_alternative_route():
    nodes = [[0, 0, 0], [10, 0, 0], [0, 10, 0], [10, 10, 0]]
    result = native.solve_sipp(nodes, [[0, 1], [1, 3], [0, 2], [2, 3]], [10] * 4, 0, 3,
                               reservations_paths=[[[10, 0, 0], [10, 0, 0]]],
                               reservations_times=[[0, 100]], horizontal_separation=1,
                               vertical_separation=1, latest_arrival=30)
    assert result["status"] == "ok"
    assert result["path"] == [0, 2, 3]


def test_sipp_prevents_edge_exchange_and_respects_latest_arrival():
    result = native.solve_sipp([[0, 0, 0], [10, 0, 0]], [[0, 1]], [10], 0, 1,
                               reservations_paths=[[[10, 0, 0], [0, 0, 0]]],
                               reservations_times=[[0, 10]], horizontal_separation=1,
                               vertical_separation=1, latest_arrival=15)
    assert result["status"] == "no_path"


def test_sipp_heading_limit_checks_transition_not_only_nodes():
    nodes = [[0, 0, 0], [1, 0, 0], [1, 1, 0]]
    result = native.solve_sipp(nodes, [[0, 1], [1, 2]], [1, 1], 0, 2, max_turn_angle=math.pi / 4)
    assert result["status"] == "no_path"


def test_sipp_wait_mask_forbids_internal_edge_delay_and_allows_physical_stop():
    nodes = [[-10, 0, 0], [0, 0, 0], [10, 0, 0]]
    options = dict(reservations_paths=[[[5, -5, 0], [5, 5, 0]]],
                   reservations_times=[[10, 20]], horizontal_separation=1,
                   vertical_separation=1, latest_arrival=40, directed=True)
    blocked = native.solve_sipp(nodes, [[0, 1], [1, 2]], [10, 10], 0, 2,
                                wait_allowed=[False, False, False], **options)
    assert blocked["status"] == "no_path"
    allowed = native.solve_sipp(nodes, [[0, 1], [1, 2]], [10, 10], 0, 2,
                                wait_allowed=[False, True, False], **options)
    assert allowed["path"] == [0, 1, 1, 2]
    assert allowed["times"][1] == 10
    assert allowed["times"][2] > 10
    assert allowed["times"][3] - allowed["times"][2] == pytest.approx(10)
    assert allowed["edge_ids"] == [0, 1]
    assert not conflicts([[nodes[i] for i in allowed["path"]], *options["reservations_paths"]],
                         [allowed["times"], *options["reservations_times"]], h=1, v=1)


def test_sipp_no_wait_node_retains_later_alternative_arrival():
    nodes = [[-10, 0, 0], [0, 0, 0], [-10, 10, 0], [10, 0, 0]]
    result = native.solve_sipp(
        nodes, [[0, 1], [0, 2], [2, 1], [1, 3]], [10, 10, 10, 10], 0, 3,
        wait_allowed=[False] * 4, directed=True, latest_arrival=40,
        reservations_paths=[[[5, -5, 0], [5, 5, 0]]], reservations_times=[[10, 20]],
        horizontal_separation=1, vertical_separation=1,
    )
    assert result["path"] == [0, 2, 1, 3]
    assert result["times"] == [0, 10, 20, 30]


def test_sipp_no_wait_node_cannot_jump_to_destination_safe_interval():
    options = dict(reservations_paths=[[[10, 0, 0], [10, 0, 0]]],
                   reservations_times=[[0, 25]], horizontal_separation=1,
                   vertical_separation=1, latest_arrival=40, directed=True)
    nodes = [[-10, 0, 0], [0, 0, 0], [10, 0, 0]]
    assert native.solve_sipp(nodes, [[0, 1], [1, 2]], [10, 10], 0, 2,
                             wait_allowed=[False] * 3, **options)["status"] == "no_path"
    allowed = native.solve_sipp(nodes, [[0, 1], [1, 2]], [10, 10], 0, 2,
                                wait_allowed=[False, True, False], **options)
    assert allowed["path"] == [0, 1, 1, 2]
    assert allowed["times"][-1] > 25


def test_sipp_no_wait_start_still_delays_launch_until_safe():
    result = native.solve_sipp(
        [[0, 0, 0], [10, 0, 0]], [[0, 1]], [10], 0, 1,
        reservations_paths=[[[0, 0, 0], [0, 0, 0]]], reservations_times=[[0, 5]],
        horizontal_separation=1, vertical_separation=1, latest_arrival=30,
        wait_allowed=[False, False],
    )
    assert result["path"] == [0, 1]
    assert result["times"][0] > 5
    assert result["times"][1] - result["times"][0] == pytest.approx(10)


@pytest.mark.parametrize("mask", [[], [True], [True, False, True], [1, 0],
                                  [True, None], "false", [[True, False]]])
def test_sipp_wait_mask_requires_exact_boolean_per_node(mask):
    with pytest.raises(ValueError, match="wait_allowed"):
        native.solve_sipp([[0, 0, 0], [10, 0, 0]], [[0, 1]], [10], 0, 1,
                          wait_allowed=mask)


@pytest.mark.parametrize("mask", [[1, 0], [True], [[True, False]]])
def test_raw_native_sipp_wait_mask_is_validated(mask):
    with pytest.raises(ValueError, match="wait_allowed"):
        native.require_native().solve_sipp(
            np.array([[0, 0, 0], [10, 0, 0]], dtype=float),
            np.array([[0, 1]], dtype=np.int64), np.array([10.]), 0, 1,
            wait_allowed=mask,
        )


@pytest.mark.parametrize("shape,timing,obstacle,occupied_times", [
    ([[0, 0, 0], [0, 10, 0], [10, 0, 0]], [0, 2, 10],
     [[0, 10, 0], [0, 10, 0]], [1, 3]),
    ([[0, 0, 0], [1, 0, 0], [10, 0, 0]], [0, 8, 10],
     [[1, -1, 0], [1, 1, 0]], [7, 9]),
])
def test_sipp_shaped_edge_checks_geometry_and_internal_time_law(shape, timing, obstacle, occupied_times):
    nodes = [shape[0], shape[-1]]
    options = dict(reservations_paths=[obstacle], reservations_times=[occupied_times],
                   horizontal_separation=.1, vertical_separation=.1, latest_arrival=30)
    straight = native.solve_sipp(nodes, [[0, 1]], [10], 0, 1, **options)
    assert straight["times"] == [0, 10]
    result = native.solve_sipp(nodes, [[0, 1]], [10], 0, 1,
                               edge_paths=[shape], edge_times=[timing], **options)
    assert result["path"] == [0, 0, 1]
    assert result["times"][1] > 0
    assert result["times"][-1] - result["times"][1] == pytest.approx(10)
    assert result["edge_ids"] == [0]
    actual = [shape[0], *shape]
    actual_times = [0, *[result["times"][1] + t for t in timing]]
    assert not conflicts([actual, obstacle], [actual_times, occupied_times], h=.1, v=.1)
    forbidden = native.solve_sipp(nodes, [[0, 1]], [10], 0, 1,
                                  edge_paths=[shape], edge_times=[timing],
                                  wait_allowed=np.array([False, False]), **options)
    assert forbidden["status"] == "no_path"


def test_sipp_undirected_shaped_edge_reverses_nonuniform_time_law():
    shape = [[0, 0, 0], [1, 0, 0], [10, 0, 0]]
    obstacle = [[1, -1, 0], [1, 1, 0]]
    result = native.solve_sipp(
        [shape[0], shape[-1]], [[0, 1]], [10], 1, 0,
        edge_paths=[shape], edge_times=[[0, 8, 10]],
        reservations_paths=[obstacle], reservations_times=[[1, 3]],
        horizontal_separation=.1, vertical_separation=.1, latest_arrival=30,
    )
    assert result["path"] == [1, 1, 0]
    assert result["times"][1] > 0
    actual = [shape[-1], *shape[::-1]]
    actual_times = [0, *[result["times"][1] + t for t in [0, 2, 10]]]
    assert not conflicts([actual, obstacle], [actual_times, [1, 3]], h=.1, v=.1)


def test_sipp_parallel_shaped_edges_report_chosen_input_edge():
    result = native.solve_sipp(
        [[0, 0, 0], [10, 0, 0]], [[0, 1], [0, 1]], [10, 10], 0, 1,
        edge_paths=[[[0, 0, 0], [0, 10, 0], [10, 0, 0]], [[0, 0, 0], [10, 0, 0]]],
        edge_times=[[0, 2, 10], [0, 10]], directed=True,
        reservations_paths=[[[0, 10, 0], [0, 10, 0]]], reservations_times=[[1, 3]],
        horizontal_separation=.1, vertical_separation=.1,
    )
    assert result["path"] == [0, 1]
    assert result["times"] == [0, 10]
    assert result["edge_ids"] == [1]


def test_sipp_shaped_heading_limit_uses_endpoint_tangents():
    result = native.solve_sipp(
        [[0, 0, 0], [10, 0, 0], [20, 0, 0]], [[0, 1], [1, 2]], [10, 10], 0, 2,
        edge_paths=[[[0, 0, 0], [10, -1, 0], [10, 0, 0]], [[10, 0, 0], [20, 0, 0]]],
        edge_times=[[0, 9, 10], [0, 10]], directed=True, max_turn_angle=math.pi / 4,
    )
    assert result["status"] == "no_path"


@pytest.mark.parametrize("paths,times", [
    ([[[0, 0, 0], [10, 0, 0]]], None),
    (None, [[0, 10]]),
    ([], []),
    ([[[0, 0, 0], [10, 0, 0]]], [[1, 10]]),
    ([[[0, 0, 0], [10, 0, 0]]], [[0, 9]]),
    ([[[1, 0, 0], [10, 0, 0]]], [[0, 10]]),
    ([[[0, 0, 0], [9, 0, 0]]], [[0, 10]]),
    ([[[0, 0, 0], [10, 0, 0]]], [[10, 0]]),
    ([[[0, 0, 0]]], [[0]]),
])
def test_sipp_shaped_edge_inputs_are_validated(paths, times):
    with pytest.raises(ValueError):
        native.solve_sipp([[0, 0, 0], [10, 0, 0]], [[0, 1]], [10], 0, 1,
                          edge_paths=paths, edge_times=times)


def test_explicit_iteration_limit_and_deadline_never_claim_success():
    assert native.graph_search([[0, 0, 0], [1, 0, 0], [2, 0, 0]],
                               [[0, 1], [1, 2]], [1, 1], 0, 2, max_iterations=1)["status"] == "iteration_limit"
    result = native.grid_search([0, 0, 0], [1000, 1000, 1000], [[0, 0, 0], [1000, 1000, 1000]], 1, deadline_ms=0.00001)
    assert result["status"] == "deadline_exceeded"
    assert not result["path"]


@pytest.mark.parametrize("budget", [0, None])
def test_zero_or_none_work_budgets_leave_all_searches_unrestricted(budget):
    nodes = [[0, 0, 0], [1, 0, 0], [2, 0, 0]]
    edges, costs = [[0, 1], [1, 2]], [1, 1]
    assert native.graph_search(nodes, edges, costs, 0, 2, max_iterations=budget,
                               deadline_ms=budget)["status"] == "ok"
    assert native.grid_search(nodes[0], nodes[-1], [nodes[0], nodes[-1]], 1,
                              max_iterations=budget, deadline_ms=budget)["status"] == "ok"
    assert native.solve_sipp(nodes, edges, costs, 0, 2, max_iterations=budget,
                             deadline_ms=budget)["status"] == "ok"


def test_default_graph_search_finishes_past_the_old_expansion_limit():
    count = 200_002
    nodes = np.zeros((count, 3))
    nodes[:, 0] = np.arange(count)
    edges = np.column_stack((np.arange(count - 1), np.arange(1, count)))
    costs = np.ones(count - 1)
    result = native.graph_search(nodes, edges, costs, 0, count - 1)
    assert result["status"] == "ok"
    assert len(result["path"]) == count
    assert result["expanded"] > 200_000
    capped = native.graph_search(nodes, edges, costs, 0, count - 1, max_iterations=200_000)
    assert capped["status"] == "iteration_limit" and not capped["path"]


@pytest.mark.parametrize("budget", [0, None])
def test_continuous_conflicts_preserves_every_record_above_old_limit(budget):
    count = 143
    result = native.continuous_conflicts([[[0, 0, 0]]] * count, [[0]] * count,
                                         1, 1, max_conflicts=budget)
    assert result["status"] == "ok"
    assert len(result["conflicts"]) == count * (count - 1) // 2 > 10_000
    assert len({(c["first"], c["second"]) for c in result["conflicts"]}) == len(result["conflicts"])


def test_explicit_conflict_record_limit_remains_available():
    result = native.continuous_conflicts([[[0, 0, 0]]] * 10, [[0]] * 10,
                                         1, 1, max_conflicts=7)
    assert result["status"] == "conflict_limit" and len(result["conflicts"]) == 7


def test_schedule_can_advance_through_more_than_one_hundred_thousand_reservations():
    count = 100_010
    points, times = np.zeros((count + 1, 3)), np.arange(count + 1, dtype=float)
    result = native.schedule_paths([points, [[0, 0, 0]]], [times, [0]],
        [[0, 0], [0, count + 2]], horizontal_separation=1, vertical_separation=1, fixed_count=1)
    assert result["status"] == "ok" and result["statuses"] == ["fixed", "scheduled"]
    assert count < result["delays"][1] < count + 2


def test_sipp_can_advance_through_more_than_one_hundred_thousand_reservations():
    count = 100_010
    points, times = np.zeros((count + 1, 3)), np.arange(count + 1, dtype=float)
    points[:, 0] = 5
    result = native.solve_sipp([[0, 0, 0], [10, 0, 0]], [[0, 1]], [10], 0, 1,
        reservations_paths=[points], reservations_times=[times], horizontal_separation=.01,
        vertical_separation=1, latest_arrival=count + 20, directed=True)
    assert result["status"] == "ok"
    assert count < result["times"][-1] <= count + 20


def test_deadline_and_grid_extents_use_actual_machine_range():
    assert native.graph_search([[0, 0, 0], [1, 0, 0]], [[0, 1]], [1], 0, 1,
                               deadline_ms=1e308)["status"] == "ok"
    result = native.grid_search([0, 0, 0], [1, 0, 0], [[0, 0, 0], [3e9, 0, 0]], 1)
    assert result["status"] == "ok"
    with pytest.raises(OverflowError, match="int64"):
        native.grid_search([0, 0, 0], [1, 0, 0], [[0, 0, 0], [1e20, 0, 0]], 1)


def test_independent_native_calls_can_run_on_threads():
    def search(_):
        return native.grid_search([0, 0, 0], [30, 30, 0], [[0, 0, 0], [30, 30, 0]], 1)["status"]
    with concurrent.futures.ThreadPoolExecutor(4) as executor:
        assert list(executor.map(search, range(8))) == ["ok"] * 8


def test_sparse_batch_one_thousand_routes_under_ten_seconds():
    paths = [[[0, i * 20, 50], [1000, i * 20, 50]] for i in range(1000)]
    times = [[0, 100] for _ in paths]
    begin = time.perf_counter()
    result = native.schedule_paths(paths, times, [[0, 100]] * len(paths), horizontal_separation=5, vertical_separation=5)
    assert result["accepted"] == 1000
    assert time.perf_counter() - begin < 10
    checked = native.continuous_conflicts(paths, times, 5, 5)
    assert checked["conflicts"] == []
    assert checked["checked_pairs"] < 1000


def test_analytic_motion_validates_independent_phase_time_law():
    primitive = {"kind": "line", "start": [0, 0, 0], "end": [100, 0, 0],
                 "length": 100, "duration": 15, "v0": 0, "v1": 0,
                 "peak": 10, "accel": 2, "decel": 2, "ta": 5, "tc": 5, "td": 5}
    limits = {"max_speed_mps": 10, "acceleration_mps2": 2, "deceleration_mps2": 2}
    assert native.validate_motion([primitive], limits)["valid"]
    tampered = dict(primitive, peak=11)
    result = native.validate_motion([tampered], limits)
    codes = {v["code"] for v in result["violations"]}
    assert {"HORIZONTAL_SPEED", "ACCEL_END_SPEED", "INTEGRATED_DISTANCE"} <= codes


def test_analytic_arc_checks_radius_yaw_and_centripetal_acceleration():
    arc = {"kind": "arc", "start": [10, 0, 0], "end": [0, 10, 0], "center": [0, 0, 0],
           "radius": 10, "sweep": math.pi / 2, "length": 5 * math.pi,
           "duration": 2.5 * math.pi, "v0": 2, "v1": 2, "peak": 2,
           "ta": 0, "tc": 2.5 * math.pi, "td": 0}
    assert native.validate_motion([arc], {"max_speed_mps": 3, "min_turn_radius_m": 10,
                                         "acceleration_mps2": 1, "deceleration_mps2": 1,
                                         "max_yaw_rate_deg_s": 20})["valid"]
    result = native.validate_motion([arc], {"min_turn_radius_m": 11, "max_yaw_rate_deg_s": 5,
                                          "acceleration_mps2": .1, "deceleration_mps2": .1})
    assert {"TURN_RADIUS", "YAW_RATE", "ACCELERATION", "DECELERATION"} <= {v["code"] for v in result["violations"]}


def test_analytic_turn_angle_uses_arc_sweep_even_without_polyline_corners():
    arc = {"kind": "arc", "start": [10, 0, 0], "end": [0, 10, 0], "center": [0, 0, 0],
           "radius": 10, "sweep": math.pi / 2, "length": 5 * math.pi,
           "duration": 2.5 * math.pi, "v0": 2, "v1": 2, "peak": 2,
           "tc": 2.5 * math.pi}
    result = native.validate_motion([arc], {"max_turn_angle_deg": 45})
    assert not result["valid"]
    assert any(v["code"] == "TURN_ANGLE" and v["actual"] == pytest.approx(90)
               for v in result["violations"])


def test_analytic_turn_angle_cannot_be_bypassed_by_splitting_tangent_arcs():
    middle = 10 / math.sqrt(2)
    first = {"kind": "arc", "start": [10, 0, 0], "end": [middle, middle, 0],
             "center": [0, 0, 0], "radius": 10, "sweep": math.pi / 4,
             "length": 2.5 * math.pi, "duration": 1.25 * math.pi,
             "v0": 2, "v1": 2, "peak": 2, "tc": 1.25 * math.pi}
    second = dict(first, start=first["end"], end=[0, 10, 0])
    assert native.validate_motion([first], {"max_turn_angle_deg": 45})["valid"]
    result = native.validate_motion([first, second], {"max_turn_angle_deg": 45})
    assert not result["valid"]
    assert any(v["primitive"] == 1 and v["code"] == "TURN_ANGLE" for v in result["violations"])


def test_analytic_positive_subnanosecond_phases_keep_their_acceleration():
    primitive = {"kind": "line", "start": [0, 0, 0], "end": [1, 0, 0],
                 "length": 1, "duration": 1.000000001, "v0": 0, "v1": 0,
                 "peak": 1, "accel": 1e9, "decel": 1e9,
                 "ta": 1e-9, "tc": .999999999, "td": 1e-9}
    result = native.validate_motion([primitive], {"acceleration_mps2": 2, "deceleration_mps2": 2})
    assert not result["valid"]
    assert {"ACCELERATION", "DECELERATION"} <= {v["code"] for v in result["violations"]}
    assert result["maxima"]["acceleration_mps2"] == pytest.approx(1e9)


def test_analytic_yaw_angle_uses_actual_sweep_instead_of_modulo_heading():
    yaw = {"kind": "yaw", "start": [0, 0, 0], "end": [0, 0, 0],
           "duration": 10, "sweep": 2 * math.pi}
    result = native.validate_motion([yaw], {"max_yaw_rate_deg_s": 60, "max_turn_angle_deg": 180})
    assert not result["valid"]
    assert any(v["code"] == "TURN_ANGLE" for v in result["violations"])


def test_analytic_arc_combines_tangential_and_normal_acceleration():
    length = 4 * math.pi / 2
    ta = 4 / 3
    tc = (length - 2 * ta) / 4
    arc = {"kind": "arc", "start": [4, 0, 0], "end": [0, 4, 0], "center": [0, 0, 0],
           "radius": 4, "sweep": math.pi / 2, "length": length,
           "duration": ta + tc, "v0": 0, "v1": 4, "peak": 4,
           "accel": 3, "ta": ta, "tc": tc, "td": 0}
    result = native.validate_motion([arc], {"acceleration_mps2": 4.5})
    assert any(v["code"] == "ACCELERATION" and v["actual"] == pytest.approx(5) for v in result["violations"])


def test_analytic_motion_checks_geometry_time_and_vector_velocity_continuity():
    line = {"kind": "line", "start": [0, 0, 0], "end": [10, 0, 0], "length": 10,
            "duration": 5, "v0": 2, "v1": 2, "peak": 2, "tc": 5}
    turn = dict(line, start=[10, 0, 0], end=[10, 10, 0])
    result = native.validate_motion([line, turn], {}, starts=[0, 6])
    assert {"TIME_DISCONTINUITY", "VELOCITY_DISCONTINUITY"} <= {v["code"] for v in result["violations"]}


def test_sparse_samples_use_exact_circumradius():
    angles = np.deg2rad([0, 60, 170])
    path = np.c_[10 * np.cos(angles), 10 * np.sin(angles), np.zeros(3)]
    result = native.validate_trajectory(path, [0, 100, 200], {"min_turn_radius_m": 10})
    assert result["valid"]
    assert result["maxima"]["curvature_per_m"] == pytest.approx(.1)
    assert not native.validate_trajectory(path, [0, 100, 200], {"min_turn_radius_m": 10.1})["valid"]


def test_analytic_yaw_requires_and_checks_real_sweep():
    yaw = {"kind": "yaw", "start": [0, 0, 0], "end": [0, 0, 0], "duration": 1, "sweep": math.pi / 2}
    assert native.validate_motion([yaw], {"max_yaw_rate_deg_s": 90})["valid"]
    assert not native.validate_motion([yaw], {"max_yaw_rate_deg_s": 45})["valid"]
    assert not native.validate_motion([dict(yaw, sweep=0)], {})["valid"]


def test_dense_collision_samples_do_not_rebuild_all_prior_reservations():
    x = np.linspace(0, 1000, 100)
    paths = [np.c_[x, np.full(100, i * 20), np.full(100, 50)] for i in range(1000)]
    begin = time.perf_counter()
    result = native.schedule_paths(paths, [np.linspace(0, 100, 100)] * 1000, [[0, 100]] * 1000,
                                   horizontal_separation=5, vertical_separation=5)
    assert result["accepted"] == 1000
    assert time.perf_counter() - begin < 10


def test_positive_radius_cannot_be_bypassed_by_stopping_at_corner():
    a = {"kind": "line", "start": [0, 0, 0], "end": [1, 0, 0], "length": 1,
         "duration": 2, "v0": 0, "v1": 0, "peak": 1, "accel": 1, "decel": 1, "ta": 1, "td": 1}
    b = dict(a, start=[1, 0, 0], end=[1, 1, 0])
    result = native.validate_motion([a, b], {"min_turn_radius_m": 5})
    assert "TURN_RADIUS_BYPASS" in {v["code"] for v in result["violations"]}


def test_yaw_sweep_must_match_actual_adjacent_headings():
    a = {"kind": "line", "start": [0, 0, 0], "end": [1, 0, 0], "length": 1,
         "duration": 2, "v0": 0, "v1": 0, "peak": 1, "accel": 1, "decel": 1, "ta": 1, "td": 1}
    b = dict(a, start=[1, 0, 0], end=[1, 1, 0])
    yaw = {"kind": "yaw", "start": [1, 0, 0], "end": [1, 0, 0], "duration": 1, "sweep": math.pi / 2}
    assert native.validate_motion([a, yaw, b], {"max_yaw_rate_deg_s": 90})["valid"]
    wrong = native.validate_motion([a, dict(yaw, sweep=-math.pi / 2), b], {"max_yaw_rate_deg_s": 90})
    assert "YAW_HEADING_MISMATCH" in {v["code"] for v in wrong["violations"]}
    missing = native.validate_motion([a, b], {"max_yaw_rate_deg_s": 90})
    assert "MISSING_YAW_TRANSITION" in {v["code"] for v in missing["violations"]}


def test_all_fixed_obstacles_remain_reserved_when_they_mutually_conflict():
    # The first obstacle crosses the stationary second obstacle early. New
    # traffic arrives much later, when only that second obstacle can block it.
    paths = [[[-10, 0, 0], [10, 0, 0]],
             [[0, 0, 0], [0, 0, 0]],
             [[0, -10, 0], [0, 10, 0]]]
    times = [[0, 10], [0, 100], [40, 60]]
    result = native.schedule_paths(
        paths, times, [[0, 0], [0, 0], [40, 40]],
        horizontal_separation=1, vertical_separation=1,
        priorities=[-100, -100, 100], fixed_count=2,
    )
    assert result["statuses"] == ["fixed", "fixed", "window_exhausted"]
    assert result["delays"] == [0, 0, None]
    assert result["accepted"] == 2
    assert result["status"] == "partial"


def test_fixed_obstacles_cannot_move_and_precede_deadline_checks():
    paths = [[[0, 0, 0], [1, 0, 0]]] * 3
    result = native.schedule_paths(
        paths, [[0, 10]] * 3, [[50, 60]] * 3,
        horizontal_separation=1, vertical_separation=1,
        deadline_ms=0.00001, fixed_count=2,
    )
    assert result["statuses"] == ["fixed", "fixed", "deadline_exceeded"]
    assert result["delays"] == [0, 0, None]
    assert result["accepted"] == 2


def test_fixed_count_is_validated_before_reservations_are_built():
    with pytest.raises(ValueError, match="fixed_count"):
        native.schedule_paths([[[0, 0, 0]]], [[0]], [[0, 0]],
                              horizontal_separation=1, vertical_separation=1,
                              fixed_count=2)


def test_analytic_validator_rejects_degenerate_arcs_and_unknown_stationary_kind():
    arc = {"kind": "arc", "start": [0, 0, 0], "end": [0, 0, 0],
           "length": 0, "duration": 1, "radius": 5, "sweep": 1}
    assert not native.validate_motion([arc], {"min_turn_radius_m": 5})["valid"]
    assert not native.validate_motion([dict(arc, kind="unknown")], {})["valid"]


def test_positive_horizontal_radius_allows_stopped_pitch_transition_but_not_vertical_detour():
    horizontal = {"kind": "line", "start": [0, 0, 0], "end": [1, 0, 0], "length": 1,
                  "duration": 2, "v0": 0, "v1": 0, "peak": 1, "accel": 1, "decel": 1, "ta": 1, "td": 1}
    climb = dict(horizontal, start=[1, 0, 0], end=[1, 0, 1], kind="climb")
    same_heading = dict(horizontal, start=[1, 0, 1], end=[2, 0, 1])
    assert native.validate_motion([horizontal, climb, same_heading], {"min_turn_radius_m": 10})["valid"]
    changed_heading = dict(same_heading, end=[1, 1, 1])
    result = native.validate_motion([horizontal, climb, changed_heading], {"min_turn_radius_m": 10})
    assert "TURN_RADIUS_BYPASS" in {issue["code"] for issue in result["violations"]}
