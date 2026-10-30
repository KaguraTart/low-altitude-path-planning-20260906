"""The public API kit contains complete, linked, and verified v2 input data."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pytest
from shapely import from_wkt

from server.schemas import BatchRequest, SingleRequest, ValidateRequest
from uav_planner.v2.models import CoordinateRequest, RULE_TYPES, Scene, Task


DATA = Path(__file__).resolve().parents[2] / "test_api" / "data"


def load(relative_path):
    return json.loads((DATA / relative_path).read_text(encoding="utf-8"))


def check_references(scene, tasks):
    """Check task, connector, airway, and rule references without a service."""
    profiles = {"dji_mini_4_pro", *(item.id for item in scene.profiles)}
    connectors = {item.id for item in scene.connectors}
    airways = {item.id for item in scene.airways}
    rules = {item.id for item in scene.rules}
    resources = {
        item.id
        for collection in (
            scene.airways, scene.connectors, scene.obstacles,
            scene.control_zones, scene.datasets,
        )
        for item in collection
    }
    for connector in scene.connectors:
        assert set(connector.airway_ids) <= airways
    for rule in scene.rules:
        assert set(rule.resource_ids) <= resources
    for task in tasks:
        assert task.aircraft is not None or task.profile_id in profiles
        for connector_id in (task.start_connector_id, task.end_connector_id):
            assert connector_id is None or connector_id in connectors
        assert task.rule_ids is None or set(task.rule_ids) <= rules


def test_manifest_lists_every_input_with_complete_integrity_metadata():
    manifest = load("manifest.json")
    assert manifest["schema_version"] == 1
    files = manifest["files"]
    paths = [entry["path"] for entry in files]
    assert len(paths) == len(set(paths))
    assert set(paths) == {
        path.relative_to(DATA).as_posix()
        for path in DATA.rglob("*.json")
        if path != DATA / "manifest.json"
    }
    for entry in files:
        relative = Path(entry["path"])
        assert not relative.is_absolute() and ".." not in relative.parts
        content = (DATA / relative).read_bytes()
        assert len(content) == entry["bytes"]
        assert hashlib.sha256(content).hexdigest() == entry["sha256"]
        json.loads(content)
    assert {item["path"] for item in manifest["examples"]} | {
        item[key] for item in manifest["benchmarks"] for key in ("scene", "tasks")
    } | {item["path"] for item in manifest.get("features", [])} == set(paths)


def test_six_examples_match_the_current_public_request_models():
    examples = load("manifest.json")["examples"]
    assert {item["name"] for item in examples} == {
        "single_strict", "single_prefer", "single_free", "batch",
        "coordinates_to_local", "validate",
    }
    for item in examples:
        assert item["description"]
        payload = load(item["path"])
        assert "scene_id" not in payload
        if item["name"].startswith("single_"):
            request = SingleRequest.model_validate(payload)
            assert request.task.route_mode == item["name"].removeprefix("single_")
            assert request.scene is not None
            check_references(request.scene, [request.task])
        elif item["name"] == "batch":
            request = BatchRequest.model_validate(payload)
            assert request.scene is not None and len(request.tasks) == 2
            check_references(request.scene, request.tasks)
        elif item["name"] == "coordinates_to_local":
            request = CoordinateRequest.model_validate(payload)
            assert request.direction == "wgs84_to_local"
            assert request.geometries_wkt and request.at is not None
        else:
            request = ValidateRequest.model_validate(payload)
            assert request.scene is not None and request.motion_segments
            assert len(from_wkt(request.trajectory_wkt).coords) == len(request.vertex_times_s)
            assert all(a < b for a, b in zip(request.vertex_times_s, request.vertex_times_s[1:]))
            check_references(request.scene, [request.task])


@pytest.mark.parametrize("suite", ["strict", "mixed", "free"])
def test_each_benchmark_has_one_complete_scene_and_1000_linked_tasks(suite):
    records = load("manifest.json")["benchmarks"]
    assert {record["suite"] for record in records} == {"strict", "mixed", "free"}
    record = next(record for record in records if record["suite"] == suite)
    scene = Scene.model_validate(load(record["scene"]))
    raw_tasks = load(record["tasks"])
    assert isinstance(raw_tasks, list)
    tasks = [Task.model_validate(task) for task in raw_tasks]
    assert len(tasks) == record["task_count"] == 1000
    assert len({task.id for task in tasks}) == 1000
    assert len(scene.profiles) == 4
    check_references(scene, tasks)
    assert {task.route_mode for task in tasks} == (
        {"strict", "prefer", "free"} if suite == "mixed" else {suite}
    )
    assert all(task.start_wkt.startswith("POINT Z") and task.goal_wkt.startswith("POINT Z")
               for task in tasks)
    assert all(task.departure_time.tzinfo is not None for task in tasks)
    if suite == "mixed":
        assert {rule.rule_type for rule in scene.rules} == set(RULE_TYPES)
        assert scene.datasets and scene.obstacles and scene.control_zones


def test_network_and_grid_handoff_inputs_match_request_contracts():
    from server.schemas import NetworkAnalyzeRequest
    from uav_planner.v2.engine import plan_single_v2
    from server.v2 import analyze_network

    source = load('features/network_request.json')
    request = SingleRequest.model_validate(source)
    check_references(request.scene, [request.task])
    route = plan_single_v2(**source)
    assert route['status'] == 'planned'
    assert route['network_route']['node_ids'] == ['A', 'B', 'D']
    assert route['network_route']['total_cost'] == 2
    analysis = load('features/network_analyze_request.json')
    NetworkAnalyzeRequest.model_validate(analysis)
    result = analyze_network(NetworkAnalyzeRequest.model_validate(analysis))
    assert ['E'] in result['weak_components']

    source = load('features/search_request.json')
    request = SingleRequest.model_validate(source)
    check_references(request.scene, [request.task])
    route = plan_single_v2(**source)
    assert route['status'] == 'planned'
    assert route['search_report']['algorithm'] == 'DIJKSTRA'
    assert route['search_report']['neighbors'] == 6
    assert route['grid_context']['level'] == 12
    assert route['traversed_grids'][0]['grid_code'] == 'start-cell'
    assert route['traversed_grids'][-1]['grid_code'] == 'goal-cell'


def test_public_sdk_supports_graph_trace_and_connectivity():
    from uav_planner import analyze_network_v2, plan_single_v2, validate_route_v2

    source = load('features/network_request.json')
    route = plan_single_v2(**source)
    validation = validate_route_v2(**source, **{
        key: route[key] for key in
        ('trajectory_wkt', 'vertex_times_s', 'motion_segments', 'network_route')
    })
    assert validation['valid']
    assert ['E'] in analyze_network_v2(**load('features/network_analyze_request.json'))['weak_components']


def test_objective_handoff_input_has_real_competing_valid_routes():
    from uav_planner import plan_single_v2, validate_route_v2

    source = load('features/objectives_request.json')
    SingleRequest.model_validate(source)
    primary = plan_single_v2(**source)
    assert primary['status'] == 'planned'
    routes = [primary, *primary['alternatives']]
    assert len(routes) >= 2
    assert primary['optimization']['scope'] == 'validated_candidates'
    assert primary['optimization']['status'] == 'generator_exhausted'
    primary_values = primary['objectives']['values']
    assert set(primary_values) == {'distance', 'time', 'energy', 'risk', 'north_cost', 'south_cost'}
    assert any(route['objectives']['values']['distance']['raw'] < primary_values['distance']['raw']
               and route['objectives']['values']['risk']['raw'] > primary_values['risk']['raw']
               for route in routes[1:])
    for route in routes:
        assert route['objectives']['weighted_score'] >= primary['objectives']['weighted_score'] - 1e-8
        assert validate_route_v2(**source, **{key: route[key] for key in
            ('trajectory_wkt', 'vertex_times_s', 'motion_segments')})['valid']


def test_replan_handoff_input_retains_prefix_and_declares_velocity():
    from uav_planner import replan_v2, validate_route_v2
    from uav_planner.v2.replanning import ReplanRequest

    source = load('features/replan_request.json')
    ReplanRequest.model_validate(source)
    result = replan_v2(**source)
    assert result['decision'] == 'replanned'
    assert result['reused_distance_m'] > 0 and result['new_search_calls'] > 0
    route = result['route']
    payload = {'scene':source['scene'], 'task':source['remaining_task'],
               **{key: route[key] for key in ('trajectory_wkt','vertex_times_s','motion_segments','initial_velocity_xyz_mps')}}
    assert validate_route_v2(**payload)['valid']
    payload['initial_velocity_xyz_mps'] = [0,0,0]
    assert not validate_route_v2(**payload)['valid']
