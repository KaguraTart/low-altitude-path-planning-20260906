"""Execution controls prepare independently but reserve/validate globally."""
from copy import deepcopy
from threading import Barrier, Lock
import time

import pytest
from pydantic import ValidationError

from server.schemas import BatchRequest
from server.v2 import example_request
from uav_planner import PlanningError
from uav_planner.v2 import engine
from uav_planner.v2.execution import clone_worker_scene
from uav_planner.v2.models import ExecutionOptions
from tests.planning.test_network import graph_request
from tests.planning.test_search_grid_contract import request as grid_request


def batch(count=4):
    request = example_request()
    return dict(scene=request['scene'], tasks=[dict(request['task'], id=f'task-{i}') for i in range(count)])


@pytest.mark.parametrize('field', ['workers', 'batch_size'])
@pytest.mark.parametrize('value', [True, False, 0, -1, '2', 2.0])
def test_execution_requires_strict_positive_integers(field, value):
    with pytest.raises(ValidationError):
        ExecutionOptions.model_validate({field: value})
    with pytest.raises(PlanningError):
        engine.plan_batch_v2(**batch(1), execution={field: value})
    with pytest.raises(ValidationError):
        BatchRequest.model_validate(dict(batch(1), execution={field: value}))


def test_execution_defaults_and_unknown_fields():
    assert ExecutionOptions().model_dump() == {'workers': 1, 'batch_size': None}
    for data in ({'workers': None}, {'unexpected': 3}):
        with pytest.raises(ValidationError):
            ExecutionOptions.model_validate(data)


@pytest.mark.parametrize('execution', [dict(workers=1), dict(workers=3), dict(workers=3,batch_size=2), dict(workers=8,batch_size=1)])
def test_all_coordinates_times_status_and_reports_equal_serial(execution):
    request = batch(5)
    # Input order is deliberately distinct from priority/window/id order.
    request['tasks'][1]['priority'] = 9
    request['tasks'].reverse()
    serial = engine.plan_batch_v2(**request)
    parallel = engine.plan_batch_v2(**request, execution=execution)
    assert parallel['routes'] == serial['routes']
    assert parallel['conflicts'] == serial['conflicts'] == []
    assert parallel['success_count'] == serial['success_count'] == 5
    report = parallel['execution_report']
    assert report['requested_workers'] == execution['workers']
    size = execution.get('batch_size') or 5
    assert 1 <= report['actual_workers'] <= min(execution['workers'], size, 5)
    assert report['batches'] == (5 + size - 1) // size
    assert report['batch_size'] == execution.get('batch_size')
    assert report['candidate_count'] == 5
    # Every route shares the corridor, including routes in different groups.
    assert len({r['vertex_times_s'][0] for r in parallel['routes']}) == 5


def test_real_candidate_helper_overlaps_and_reuses_worker_cache(monkeypatch):
    original = engine._prepare_candidate
    rendezvous = Barrier(3, timeout=10)
    lock = Lock()
    state = dict(active=0, peak=0, calls=0)
    scenes = {}

    def measured(index, task, model, compiled, *rest):
        from threading import get_ident
        with lock:
            state['active'] += 1
            state['peak'] = max(state['peak'], state['active'])
            call = state['calls']
            state['calls'] += 1
            scenes.setdefault(get_ident(), set()).add(id(compiled))
        try:
            if call < 3:
                rendezvous.wait()
            return original(index, task, model, compiled, *rest)
        finally:
            with lock:
                state['active'] -= 1

    monkeypatch.setattr(engine, '_prepare_candidate', measured)
    result = engine.plan_batch_v2(**batch(7), execution={'workers': 3, 'batch_size': 4})
    assert state['peak'] == 3
    assert state['calls'] == 7
    assert len(scenes) == 3 and all(len(ids) == 1 for ids in scenes.values())
    assert result['success_count'] == 7
    assert result['execution_report']['actual_workers'] == len(scenes)


def test_worker_scene_back_references_and_cache_isolation():
    request = graph_request()
    scene_id = engine.register_scene(request['scene'])['scene_id']
    compiled = engine.registry.get(scene_id)[1]
    a, b = clone_worker_scene(compiled), clone_worker_scene(compiled)
    for worker in (a, b):
        assert worker.rules.scene is worker
        assert worker.network.compiled is worker
        assert worker.frame.terrain_provider.__self__ is worker
        assert worker._obstacle_tree is compiled._obstacle_tree
        assert worker.network.lines is compiled.network.lines
        for name, value in vars(compiled).items():
            if name.endswith('_cache') and isinstance(value, dict):
                assert getattr(worker, name) is not value
    a._rule_geometry_cache['sentinel'] = 'a'
    assert 'sentinel' not in b._rule_geometry_cache
    assert 'sentinel' not in compiled._rule_geometry_cache


def test_mixed_network_free_and_business_failure_are_complete_and_deterministic():
    request = graph_request()
    free = example_request()['task']
    free.update(id='free', route_mode='free')
    disconnected = deepcopy(request['task'])
    disconnected.update(id='disconnected')
    disconnected['network']['goal_node_id'] = 'isolate'
    tasks = [dict(request['task'], id='graph'), free, disconnected]
    serial = engine.plan_batch_v2(scene=request['scene'], tasks=tasks)
    parallel = engine.plan_batch_v2(scene=request['scene'], tasks=tasks, execution={'workers': 3, 'batch_size': 2})
    assert parallel['routes'] == serial['routes']
    assert [r['task_id'] for r in parallel['routes']] == ['graph', 'free', 'disconnected']
    assert parallel['routes'][0]['network_route']['edge_ids'] == ['ab', 'bd']
    assert parallel['routes'][-1]['code'] == 'NETWORK_DISCONNECTED'


def test_search_budget_is_per_task_and_aggregate_across_legs():
    request = grid_request()
    request['task']['search'] = {'neighbors': 6, 'max_expanded_nodes': 120}
    request['task']['waypoints'] = [{'wkt': request['task']['goal_wkt']}]
    # Return to the start for a second leg; the second leg exhausts the remainder.
    request['task']['goal_wkt'] = request['task']['start_wkt']
    tasks = [dict(request['task'], id=f'grid-{i}') for i in range(3)]
    serial = engine.plan_batch_v2(scene=request['scene'], tasks=tasks)
    parallel = engine.plan_batch_v2(scene=request['scene'], tasks=tasks, execution={'workers': 3})
    assert parallel['routes'] == serial['routes']
    for route in parallel['routes']:
        assert route['code'] == 'SEARCH_BUDGET_EXCEEDED'
        assert route['search_report']['expanded_nodes'] == 120


def test_workers_receive_one_absolute_deadline_and_timeout_outputs(monkeypatch):
    original = engine._prepare_candidate
    deadlines = []
    lock = Lock()
    def expired(index, task, model, compiled, tracks, deadline, search_state):
        with lock:
            deadlines.append(deadline)
        # Only the initial group waits, so later groups must inherit expiry.
        if index < 2:
            time.sleep(.03)
        return original(index, task, model, compiled, tracks, deadline, search_state)
    request = batch(5)
    scene_id = engine.register_scene(request.pop('scene'))['scene_id']
    monkeypatch.setattr(engine, '_prepare_candidate', expired)
    result = engine.plan_batch_v2(**request, scene_id=scene_id, time_budget_s=.01,
                                  execution={'workers': 2, 'batch_size': 2})
    assert len(set(deadlines)) == 1 and len(deadlines) == 5
    assert result['success_count'] == 0
    assert [r['task_id'] for r in result['routes']] == [t['id'] for t in request['tasks']]
    assert all(r['code'] == 'TIME_BUDGET_EXCEEDED' for r in result['routes'])


def test_invalid_request_still_raises_in_parallel():
    request = batch(2)
    request['tasks'][1]['departure_time'] = '2025-01-01T00:00:00Z'
    with pytest.raises(PlanningError, match='horizon'):
        engine.plan_batch_v2(**request, execution={'workers': 2})


def test_successful_grid_search_keeps_coordinates_times_and_expansion_diagnostics():
    request = grid_request()
    request['task']['search'] = {'neighbors': 6}
    tasks = [dict(request['task'], id=f'success-grid-{i}') for i in range(3)]
    serial = engine.plan_batch_v2(scene=request['scene'], tasks=tasks)
    parallel = engine.plan_batch_v2(scene=request['scene'], tasks=tasks,
                                    execution={'workers': 3, 'batch_size': 2})
    assert serial['success_count'] == parallel['success_count'] == 3
    assert serial['routes'] == parallel['routes']
    assert all(route['search_report']['expanded_nodes'] > 1 for route in parallel['routes'])


def test_default_serial_reuses_compiled_scene_without_cloning(monkeypatch):
    from uav_planner.v2 import execution
    def unexpected_clone(*args):
        raise AssertionError('serial execution must retain warmed scene caches')
    monkeypatch.setattr(execution, 'clone_worker_scene', unexpected_clone)
    result = engine.plan_batch_v2(**batch(2))
    assert result['success_count'] == 2
    assert result['execution_report']['actual_workers'] == 1
