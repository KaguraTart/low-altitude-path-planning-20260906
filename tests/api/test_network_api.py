"""Typed network analysis HTTP contracts."""
from fastapi.testclient import TestClient

from server.app import app
from server.v2 import example_request


def scene_with_graph():
    request = example_request(); scene = request['scene']
    scene['airway_graph'] = {'version': 'api-v1', 'nodes': [
        {'id': 'a', 'point_wkt': request['task']['start_wkt']},
        {'id': 'b', 'point_wkt': request['task']['goal_wkt']},
        {'id': 'isolate', 'point_wkt': request['task']['start_wkt']}],
        'edges': [{'id': 'ab', 'from_node': 'a', 'to_node': 'b', 'airway_id': 'demo-airway',
                   'direction': 'FORWARD', 'weights': {'distance': 1}}]}
    return request


def test_network_analysis_inline_registered_and_time():
    request = scene_with_graph()
    with TestClient(app) as client:
        first = client.post('/api/v2/networks/analyze', json={'scene': request['scene']})
        assert first.status_code == 200, first.text
        assert first.json() == {'version': 'api-v1', 'nodes': ['a','b','isolate'], 'active_edges': ['ab'],
                                'weak_components': [['a','b'],['isolate']], 'strong_components': [['a'],['b'],['isolate']], 'inactive_edge_ids': []}
        registered = client.post('/api/v2/scenes', json=request['scene']).json()
        assert client.post('/api/v2/networks/analyze', json={'scene_id':registered['scene_id']}).json() == first.json()
        request['scene']['airway_graph']['edges'][0]['valid_to'] = '2026-10-16T09:00:01+08:00'
        expired = client.post('/api/v2/networks/analyze',json={'scene':request['scene'],'at':'2026-10-16T09:00:02+08:00'})
        assert expired.status_code == 200
        assert expired.json()['inactive_edge_ids'] == ['ab']
        assert expired.json()['weak_components'] == [['a'],['b'],['isolate']]


def test_node_only_http_plan_and_validate():
    request = scene_with_graph(); request['task'].pop('start_wkt'); request['task'].pop('goal_wkt')
    request['task']['network'] = {'start_node_id':'a','goal_node_id':'b','weight_field':'distance','expected_version':'api-v1'}
    with TestClient(app) as client:
        planned = client.post('/api/v2/plan/single',json=request)
        assert planned.status_code == 200, planned.text
        route = planned.json(); assert route['status'] == 'planned', route
        assert route['network_route']['edge_ids'] == ['ab']
        check = client.post('/api/v2/validate',json={**request,'trajectory_wkt':route['trajectory_wkt'], 'vertex_times_s':route['vertex_times_s'], 'motion_segments':route['motion_segments']})
        assert check.status_code == 200, check.text
        assert check.json()['valid'], check.json()


def test_network_analysis_requires_graph_and_valid_time():
    with TestClient(app) as client:
        assert client.post('/api/v2/networks/analyze',json={'scene':example_request()['scene']}).status_code == 400
        assert client.post('/api/v2/networks/analyze',json={'scene':scene_with_graph()['scene'],'at':'2026-10-16T09:00:00'}).status_code == 400
