"""ENG-10 HTTP contracts for evaluation and plan comparison."""
from fastapi.testclient import TestClient

from server.app import app
from server.v2 import example_request


def planned(client, request):
    response = client.post('/api/v2/plan/single', json=request)
    assert response.status_code == 200, response.text
    route = response.json()
    assert route['status'] == 'planned'
    return route


def evaluate_payload(request, route):
    return {'scene': request['scene'], 'task': request['task'],
            'trajectory_wkt': route['trajectory_wkt'],
            'vertex_times_s': route['vertex_times_s'],
            'motion_segments': route['motion_segments']}


def test_evaluate_returns_safety_and_metrics():
    request = example_request()
    with TestClient(app) as client:
        route = planned(client, request)
        response = client.post('/api/v2/evaluate', json=evaluate_payload(request, route))
        assert response.status_code == 200, response.text
        body = response.json()
        assert body['engine_id'] == 'ENG-10'
        assert body['valid'] is True
        assert body['metrics']['distance_m']['available'] is True
        assert body['metrics']['energy_wh']['available'] is False
        assert body['metrics']['energy_wh']['value'] is None
        assert body['score']['available'] is True


def test_evaluate_rejects_missing_scene_source():
    request = example_request()
    with TestClient(app) as client:
        route = planned(client, request)
        payload = evaluate_payload(request, route)
        payload.pop('scene')
        response = client.post('/api/v2/evaluate', json=payload)
        assert response.status_code == 400, response.text
        assert response.json()['error']['code'] == 'INVALID_INPUT'


def test_compare_ranks_safe_plan_and_excludes_blocked_plan():
    request = example_request()
    with TestClient(app) as client:
        route = planned(client, request)
        plans = [
            {'plan_id': 'plan-a', 'task': request['task'],
             'trajectory_wkt': route['trajectory_wkt'],
             'vertex_times_s': route['vertex_times_s'],
             'motion_segments': route['motion_segments']},
            {'plan_id': 'plan-b', 'task': request['task'],
             'trajectory_wkt': route['trajectory_wkt'],
             'vertex_times_s': route['vertex_times_s'],
             'motion_segments': route['motion_segments']},
        ]
        response = client.post('/api/v2/evaluate/compare', json={
            'request_id': 'http-compare', 'scene': request['scene'],
            'objectives': {'weights': {'distance': 1, 'time': 0, 'energy': 0, 'risk': 0}},
            'plans': plans})
        assert response.status_code == 200, response.text
        body = response.json()
        assert body['scene_mode'] == 'shared'
        assert body['ranking'] == ['plan-a', 'plan-b']
        assert body['winner'] == 'plan-a'
        assert all(plan['rankable'] for plan in body['plans'])


def test_compare_requires_two_plans():
    request = example_request()
    with TestClient(app) as client:
        route = planned(client, request)
        response = client.post('/api/v2/evaluate/compare', json={
            'request_id': 'too-few', 'scene': request['scene'],
            'objectives': {'weights': {'distance': 1}},
            'plans': [{'plan_id': 'only', 'task': request['task'],
                       'trajectory_wkt': route['trajectory_wkt'],
                       'vertex_times_s': route['vertex_times_s'],
                       'motion_segments': route['motion_segments']}]})
        assert response.status_code == 400, response.text
        assert response.json()['error']['code'] == 'INVALID_INPUT'


def test_capabilities_report_eng10():
    with TestClient(app) as client:
        body = client.get('/api/v2/capabilities').json()
        assert body['engine_rules']['ENG-10'] == [
            'NO_FLY', 'ZONE', 'HEIGHT_LAYER', 'SEPARATION', 'ENDURANCE', 'PAYLOAD']
        assert body['evaluation_supported'] is True
        assert body['plan_comparison_supported'] is True
