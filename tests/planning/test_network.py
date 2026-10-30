"""Explicit topology and grid endpoint contract regressions."""
from copy import deepcopy
from datetime import datetime, timedelta

import pytest
from shapely.geometry import Point

from server.v2 import example_request
from uav_planner import PlanningError
from uav_planner.v2.coordinates import CoordinateFrame
from uav_planner.v2.engine import plan_single_v2, plan_batch_v2, register_scene, registry, validate_route_v2


def graph_request():
    request = example_request(); scene = request['scene']; task = request['task']
    frame = CoordinateFrame(scene['coordinates'])
    scene['airways'][0]['widthMeters'] = 500
    nodes = {'a': (100,100,80), 'b': (500,180,80), 'c': (500,50,80), 'd': (1200,100,80), 'isolate': (700,300,80)}
    scene['airway_graph'] = {'version': 'v1', 'nodes': [{'id': k, 'point_wkt': frame.to_wgs84(Point(p))} for k,p in nodes.items()], 'edges': [
        {'id': a+b, 'from_node': a, 'to_node': b, 'airway_id': 'demo-airway', 'direction': 'FORWARD', 'weights': {'distance': cost, 'risk': risk}}
        for a,b,cost,risk in [('a','b',1,5),('b','d',1,5),('a','c',3,1),('c','d',3,1)]]}
    task.pop('start_wkt'); task.pop('goal_wkt')
    task['network'] = {'start_node_id': 'a', 'goal_node_id': 'd', 'weight_field': 'distance'}
    return request


def test_weighted_diamond_and_independent_validation():
    request = graph_request(); result = plan_single_v2(**request)
    assert result['status'] == 'planned', result
    assert result['network_route']['edge_ids'] == ['ab','bd']
    assert result['network_route']['total_cost'] == 2
    assert result['start_wkt'] and result['goal_wkt']
    check = validate_route_v2(**request, trajectory_wkt=result['trajectory_wkt'], vertex_times_s=result['vertex_times_s'], motion_segments=result['motion_segments'])
    assert check['valid'], check
    request['task']['network']['weight_field'] = 'risk'
    other = plan_single_v2(**request)
    assert other['status'] == 'planned', other
    assert other['network_route']['edge_ids'] == ['ac','cd']


@pytest.mark.parametrize('mutation', ['closed','reverse','inactive'])
def test_unavailable_edges_and_no_direct_shortcut(mutation):
    request = graph_request(); edges = request['scene']['airway_graph']['edges']
    if mutation == 'closed':
        for edge in edges: edge['status'] = 'CLOSED'
    elif mutation == 'reverse':
        for edge in edges: edge['direction'] = 'REVERSE'
    else:
        request['scene']['airways'][0]['status'] = 'INACTIVE'
    result = plan_single_v2(**request)
    assert result['status'] == 'failed'
    assert result['code'] == 'NETWORK_DISCONNECTED'


def test_zero_cost_parallel_edges():
    request = graph_request(); edges = request['scene']['airway_graph']['edges']
    cheap = dict(edges[0], id='parallel', weights={'distance': 0, 'risk': 0}); edges.append(cheap)
    edges[1]['weights']['distance'] = 0
    result = plan_single_v2(**request)
    assert result['status'] == 'planned', result
    assert result['network_route']['edge_ids'] == ['parallel','bd']
    assert result['network_route']['total_cost'] == 0


@pytest.mark.parametrize('mutation', ['negative','duplicate','node','airway','version','geometry','missing_weight'])
def test_invalid_graph_registration_or_selection(mutation):
    request = graph_request(); graph = request['scene']['airway_graph']
    if mutation == 'negative': graph['edges'][0]['weights']['distance'] = -1
    elif mutation == 'duplicate': graph['nodes'].append(graph['nodes'][0])
    elif mutation == 'node': graph['edges'][0]['from_node'] = 'missing'
    elif mutation == 'airway': graph['edges'][0]['airway_id'] = 'missing'
    elif mutation == 'version': request['scene']['network_version'] = 'v2'
    elif mutation == 'geometry': graph['edges'][0]['geometry_wkt'] = request['scene']['airways'][0]['centerLine']
    else: request['task']['network']['weight_field'] = 'missing'
    with pytest.raises(PlanningError): plan_single_v2(**request)


@pytest.mark.parametrize('mutation', ['unknown','version','coordinates','waypoint','free','mixed'])
def test_invalid_network_task(mutation):
    request = graph_request(); task = request['task']
    if mutation == 'unknown': task['network']['start_node_id'] = 'missing'
    elif mutation == 'version': task['network']['expected_version'] = 'v2'
    elif mutation == 'coordinates': task['start_wkt'] = request['scene']['airway_graph']['nodes'][1]['point_wkt']
    elif mutation == 'waypoint': task['waypoints'] = [{'wkt': request['scene']['airway_graph']['nodes'][1]['point_wkt']}]
    elif mutation == 'free': task['route_mode'] = 'free'
    else: task['start_grid'] = [0,0,0]
    with pytest.raises(PlanningError): plan_single_v2(**request)


def test_components_and_isolate():
    request = graph_request(); info = register_scene(request['scene']); compiled = registry.get(info['scene_id'])[1]
    analysis = compiled.network.analyze(0)
    assert analysis['weak_components'] == [['a','b','c','d'],['isolate']]
    assert analysis['strong_components'] == [[n] for n in ['a','b','c','d','isolate']]
    assert info['network_version'] == 'v1'
    assert info['counts']['network_nodes'] == 5


def test_via_nodes_profile_admission_and_expired_edges():
    request = graph_request(); request['task']['network']['via_node_ids'] = ['c']
    result = plan_single_v2(**request)
    assert result['network_route']['node_ids'] == ['a','c','d']
    request['scene']['airways'][0]['allowed_profile_ids'] = ['other']
    assert plan_single_v2(**request)['status'] == 'failed'
    request = graph_request()
    for edge in request['scene']['airway_graph']['edges']:
        edge['valid_to'] = '2026-10-16T09:00:01+08:00'
    result = plan_single_v2(**request)
    assert result['status'] == 'failed'
    assert any(v['code'] == 'NETWORK_ROUTE' for v in result['validation']['violations'])


def test_direct_geometric_path_is_not_graph_connected_validation():
    request = graph_request(); ordinary = deepcopy(request)
    ordinary['task'].pop('network'); nodes = ordinary['scene']['airway_graph']['nodes']
    ordinary['task']['start_wkt'] = nodes[0]['point_wkt']; ordinary['task']['goal_wkt'] = nodes[3]['point_wkt']
    route = plan_single_v2(**ordinary); assert route['status'] == 'planned', route
    check = validate_route_v2(**request, trajectory_wkt=route['trajectory_wkt'], vertex_times_s=route['vertex_times_s'], motion_segments=route['motion_segments'])
    assert not check['valid']
    assert any(v['code'] == 'NETWORK_ROUTE' for v in check['violations'])


def grid_request():
    request = example_request(); request['task']['route_mode'] = 'free'; request['scene']['airways'] = []
    request['scene']['grid'] = {'level': 4, 'origin': [0,0,0], 'resolution': [100,100,20], 'shape': [15,5,15]}
    request['task'].pop('start_wkt'); request['task'].pop('goal_wkt')
    request['task'].update(start_grid=[1,1,4], goal_grid=[10,1,4], search={'grid_level': 4})
    return request


def test_grid_only_endpoints_and_validation():
    request = grid_request(); route = plan_single_v2(**request)
    assert route['status'] == 'planned', route
    frame = CoordinateFrame(request['scene']['coordinates'])
    assert list(frame.to_local(route['start_wkt'],'EG').coords[0]) == pytest.approx([150,150,90], abs=1e-6)
    assert list(frame.to_local(route['goal_wkt'],'EG').coords[0]) == pytest.approx([1050,150,90], abs=1e-6)
    assert validate_route_v2(**request, trajectory_wkt=route['trajectory_wkt'], vertex_times_s=route['vertex_times_s'], motion_segments=route['motion_segments'])['valid']


@pytest.mark.parametrize('mutation', ['bounds','negative','fraction','mismatch','level','no_grid'])
def test_invalid_grid_endpoint(mutation):
    request = grid_request()
    if mutation == 'bounds': request['task']['start_grid'] = [15,0,0]
    elif mutation == 'negative': request['task']['start_grid'] = [-1,0,0]
    elif mutation == 'fraction': request['task']['start_grid'] = [1.5,0,0]
    elif mutation == 'mismatch': request['task']['start_wkt'] = example_request()['task']['goal_wkt']
    elif mutation == 'level': request['task']['search'] = {'grid_level': 5}
    else: request['scene'].pop('grid')
    with pytest.raises(PlanningError): plan_single_v2(**request)


def test_positive_radius_diamond_and_delayed_edge_expiry():
    request = graph_request()
    request['task']['aircraft'] = {'min_turn_radius_m': 20}
    route = plan_single_v2(**request)
    assert route['status'] == 'planned', route
    assert any(s['kind'] == 'arc' and s['radius_m'] >= 20 for s in route['motion_segments'])
    assert validate_route_v2(**request, trajectory_wkt=route['trajectory_wkt'], vertex_times_s=route['vertex_times_s'], motion_segments=route['motion_segments'])['valid']
    expiry = datetime.fromisoformat(request['scene']['time_origin']) + timedelta(seconds=route['vertex_times_s'][-1]+1)
    for edge in request['scene']['airway_graph']['edges']:
        edge['valid_to'] = expiry.isoformat()
    result = plan_batch_v2(scene=request['scene'], tasks=[dict(request['task'],id='a'),dict(request['task'],id='b')])
    assert result['routes'][0]['status'] == 'planned', result
    assert result['routes'][1]['status'] == 'failed', result
    assert any(v['code'] == 'NETWORK_ROUTE' for v in result['routes'][1]['validation']['violations'])


def test_crossing_graph_edges_do_not_create_connections():
    request = graph_request()
    graph = request['scene']['airway_graph']
    graph['edges'] = [dict(graph['edges'][0], id='ad', to_node='d'), dict(graph['edges'][2], id='bc', from_node='b')]
    request['task']['network']['goal_node_id'] = 'c'
    assert plan_single_v2(**request)['code'] == 'NETWORK_DISCONNECTED'


def test_independent_validation_accepts_legal_nonminimum_walk_and_exact_trace():
    request = graph_request(); alternative = deepcopy(request)
    alternative['task']['network']['weight_field'] = 'risk'
    route = plan_single_v2(**alternative); assert route['status'] == 'planned', route
    args = dict(trajectory_wkt=route['trajectory_wkt'],vertex_times_s=route['vertex_times_s'],motion_segments=route['motion_segments'])
    # ac,cd is a valid distance walk even though ab,bd is cheaper.
    check = validate_route_v2(**request,**args)
    assert check['valid'], check
    trace = dict(route['network_route'],weight_field='distance',total_cost=6,effective_cost=6)
    assert validate_route_v2(**request,**args,network_route=trace)['valid']
    trace['edge_ids'] = ['ab','bd']
    assert not validate_route_v2(**request,**args,network_route=trace)['valid']


def test_requested_network_via_nodes_obey_waypoint_rules():
    request = graph_request()
    request['scene']['rules'] = [{'id':'waypoint-limit','rule_type':'WAYPOINT','content':{'maxWaypoints':0}}]
    # Automatic topology vertices consume no mission waypoint allowance.
    assert plan_single_v2(**request)['status'] == 'planned'
    request['task']['network']['via_node_ids'] = ['b']
    result = plan_single_v2(**request)
    assert result['status'] == 'failed', result
    assert any(v['code'] == 'WAYPOINT' for v in result['validation']['violations'])


def test_scoped_priority_changes_graph_objective_without_overriding_obstacles():
    from shapely.geometry import Polygon
    request = graph_request(); frame = CoordinateFrame(request['scene']['coordinates'])
    scope = frame.to_wgs84(Polygon([(90,0,80),(1210,0,80),(1210,120,80),(90,120,80)]))
    request['scene']['rules'] = [{'id':'prefer-lower','rule_type':'PRIORITY','content':{'priorityScene':'PLANNING','costBonus':.9},'scope':{'geom':[{'type':'AIRSPACE','geom':scope}]}}]
    route = plan_single_v2(**request)
    assert route['status'] == 'planned', route
    assert route['network_route']['edge_ids'] == ['ac','cd']
    assert route['network_route']['total_cost'] == 6
    assert route['network_route']['effective_cost'] == pytest.approx(.6)
    request['scene']['obstacles'] = [{'id':'lower-block','geometry_wkt':frame.to_wgs84(Polygon([(450,35),(550,35),(550,65),(450,65)])), 'min_altitude':50,'max_altitude':150,'alt_reference':'EG'}]
    blocked = plan_single_v2(**request)
    assert blocked['status'] == 'planned', blocked
    assert blocked['network_route']['edge_ids'] == ['ab','bd']


@pytest.mark.parametrize('include_trace', [False, True])
def test_validation_cannot_mask_wrong_edge_airway_with_other_selected_airway(include_trace):
    from shapely.geometry import LineString
    request = graph_request(); route = plan_single_v2(**request)
    assert route['status'] == 'planned', route
    frame = CoordinateFrame(request['scene']['coordinates'])
    line = frame.to_wgs84(LineString([(100,300,80),(1200,300,80)]))
    far = deepcopy(request['scene']['airways'][0])
    far.update(id='far-narrow',centerLine=line,widthMeters=30)
    for label in ('startGrid','endGrid'):
        far[label].update(minLon=104,maxLon=105,minLat=29,maxLat=30)
    request['scene']['airways'].append(far)
    request['scene']['airway_graph']['edges'][0]['airway_id'] = 'far-narrow'
    # bd's broad corridor geometrically covers ab, but cannot authorize it.
    replacement = plan_single_v2(**request)
    assert replacement['status'] == 'planned', replacement
    assert replacement['network_route']['edge_ids'] == ['ac','cd']
    extra = {'network_route':route['network_route']} if include_trace else {}
    check = validate_route_v2(**request,trajectory_wkt=route['trajectory_wkt'],vertex_times_s=route['vertex_times_s'],motion_segments=route['motion_segments'],**extra)
    assert not check['valid'], check
    assert any(v['code'] == 'NETWORK_ROUTE' for v in check['violations'])


def test_independent_validation_accepts_larger_legal_common_turn_radius():
    request = graph_request(); wider = deepcopy(request)
    wider['task']['aircraft'] = {'min_turn_radius_m':30}
    route = plan_single_v2(**wider); assert route['status'] == 'planned', route
    for extra in ({}, {'network_route':route['network_route']}):
        check = validate_route_v2(**request,trajectory_wkt=route['trajectory_wkt'],vertex_times_s=route['vertex_times_s'],motion_segments=route['motion_segments'],**extra)
        assert check['valid'], check


@pytest.mark.parametrize('geometry_case', ['remote_nodes','detour_edges','equivalent_states'])
def test_trace_free_invalid_layered_walk_prunes_before_complete_walk_enumeration(monkeypatch,geometry_case):
    import numpy as np
    from shapely.geometry import LineString
    from uav_planner.v2 import engine
    from uav_planner.v2.models import Task
    from uav_planner.v2.network import normalize_task
    request = graph_request(); frame = CoordinateFrame(request['scene']['coordinates'])
    coordinates = {'a':(100,100,80),'d':(1200,100,80)}
    edges = []; previous = ['a']; layers = 18
    for layer in range(layers):
        current = []
        for branch,y in [('a',80),('b',120)]:
            node = f'n{layer}{branch}'
            coordinates[node] = (150+(layer+1)*900/(layers+1),y if geometry_case == 'remote_nodes' else 100,80)
            current.append(node)
            for parent in previous:
                edge = {'id':f'{parent}-{node}','from_node':parent,'to_node':node,'airway_id':'demo-airway','direction':'FORWARD','weights':{'distance':1}}
                if geometry_case == 'detour_edges':
                    a,b = coordinates[parent],coordinates[node]
                    middle = ((a[0]+b[0])/2,120,80)
                    edge['geometry_wkt'] = frame.to_wgs84(LineString([a,middle,b]))
                edges.append(edge)
        previous = current
    for parent in previous:
        edges.append({'id':f'{parent}-d','from_node':parent,'to_node':'d','airway_id':'demo-airway','direction':'FORWARD','weights':{'distance':1}})
    if geometry_case == 'equivalent_states':
        coordinates['isolate'] = (700,300,80)
        request['task']['network']['via_node_ids'] = ['isolate']
    request['scene']['airway_graph'] = {'version':'layered','nodes':[{'id':node,'point_wkt':frame.to_wgs84(Point(point))} for node,point in coordinates.items()],'edges':edges}
    compiled = registry.get(register_scene(request['scene'])['scene_id'])[1]
    task = normalize_task(compiled,Task.model_validate(request['task']))
    calls = 0; original = engine._task_motion
    def counted(*args,**kwargs):
        nonlocal calls
        calls += 1
        return original(*args,**kwargs)
    monkeypatch.setattr(engine,'_task_motion',counted)
    admissibility_calls = 0; original_allowed = compiled.network.edge_geometry_allowed
    def counted_allowed(*args,**kwargs):
        nonlocal admissibility_calls
        admissibility_calls += 1
        # Abort promptly if equivalent states ever regress to enumerating
        # complete histories; this is a work bound, not a timing threshold.
        assert admissibility_calls <= 4*len(edges)
        return original_allowed(*args,**kwargs)
    monkeypatch.setattr(compiled.network,'edge_geometry_allowed',counted_allowed)
    walk_edge_reads = 0
    class CountedEdges(dict):
        def get(self,key,default=None):
            nonlocal walk_edge_reads
            walk_edge_reads += 1
            assert walk_edge_reads <= 4*len(edges)
            return super().get(key,default)
    monkeypatch.setattr(compiled.network,'edges',CountedEdges(compiled.network.edges))
    errors = compiled.network.validate(task,np.array([[100,100,80],[1200,100,80]]),np.array([0.,200.]))
    assert errors and errors[0]['code'] == 'NETWORK_ROUTE'
    # 2**18 complete topology walks exist. Only the optimizer fast-path is
    # regenerated; incompatible node/edge prefixes never reach certification.
    assert calls <= 1


def test_duplicate_consecutive_via_nodes_preserve_request_count_and_validate():
    request = graph_request(); request['task']['network']['via_node_ids'] = ['b','b']
    route = plan_single_v2(**request)
    assert route['status'] == 'planned', route
    assert route['network_route']['node_ids'] == ['a','b','d']
    assert validate_route_v2(**request,trajectory_wkt=route['trajectory_wkt'],vertex_times_s=route['vertex_times_s'],motion_segments=route['motion_segments'])['valid']
    request['scene']['rules'] = [{'id':'one-via','rule_type':'WAYPOINT','content':{'maxWaypoints':1}}]
    limited = plan_single_v2(**request)
    assert limited['status'] == 'failed'
    assert any(v['code'] == 'WAYPOINT' for v in limited['validation']['violations'])


@pytest.mark.parametrize('search', [{},{'max_expanded_nodes':1},{'algorithm':'DIJKSTRA'}])
def test_grid_search_strategy_cannot_silently_override_network_selection(search):
    request = graph_request(); request['task']['search'] = search
    with pytest.raises(PlanningError,match='GRID strategy'):
        plan_single_v2(**request)


def test_trace_free_validation_preserves_nonflat_rounded_graph_junction():
    request = graph_request(); frame = CoordinateFrame(request['scene']['coordinates'])
    request['scene']['airway_graph']['nodes'][1]['point_wkt'] = frame.to_wgs84(Point(500,180,110))
    request['task']['aircraft'] = {'min_turn_radius_m':20}
    route = plan_single_v2(**request)
    assert route['status'] == 'planned', route
    assert route['network_route']['node_ids'] == ['a','b','d']
    assert any(s['kind'] == 'arc' for s in route['motion_segments'])
    # Force inference: this flown upper branch is not the risk minimum.
    request['task']['network']['weight_field'] = 'risk'
    check = validate_route_v2(**request,trajectory_wkt=route['trajectory_wkt'],vertex_times_s=route['vertex_times_s'],motion_segments=route['motion_segments'])
    assert check['valid'], check


def test_memoized_fallback_preserves_broad_valid_prefix_after_narrow_turn_failure():
    from shapely.geometry import LineString
    request = graph_request(); frame = CoordinateFrame(request['scene']['coordinates'])
    positions = {'a':(100,100,80),'b':(500,100,80),'c':(500,300,80),'d':(500,400,80)}
    airways = []
    for name,start,end,width in [('broad','a','b',200),('narrow','a','b',20),('out','b','d',20)]:
        airway = deepcopy(request['scene']['airways'][0])
        airway.update(id=name,centerLine=frame.to_wgs84(LineString([positions[start],positions[end]])),widthMeters=width)
        for label in ('startGrid','endGrid'):
            airway[label].update(minLon=104,maxLon=105,minLat=29,maxLat=30)
        airways.append(airway)
    request['scene']['airways'] = airways
    request['scene']['airway_graph'] = {'version':'parallel-corridors','nodes':[{'id':node,'point_wkt':frame.to_wgs84(Point(point))} for node,point in positions.items()], 'edges':[
        {'id':edge_id,'from_node':start,'to_node':end,'airway_id':airway,'direction':'FORWARD','weights':{'distance':weight}}
        for edge_id,start,end,airway,weight in [('good','a','b','broad',1),('bad','a','b','narrow',3),('bc','b','c','out',1),('cd','c','d','out',1)]]}
    request['task']['aircraft'] = {'min_turn_radius_m':60}
    route = plan_single_v2(**request)
    assert route['status'] == 'planned', route
    assert route['network_route']['edge_ids'] == ['good','bc','cd']
    request['scene']['airway_graph']['edges'][0]['weights']['distance'] = 3
    request['scene']['airway_graph']['edges'][1]['weights']['distance'] = 1
    args = dict(trajectory_wkt=route['trajectory_wkt'],vertex_times_s=route['vertex_times_s'],motion_segments=route['motion_segments'])
    trace = dict(route['network_route'],total_cost=5,effective_cost=5)
    explicit = validate_route_v2(**request,**args,network_route=trace)
    assert explicit['valid'], explicit
    inferred = validate_route_v2(**request,**args)
    assert inferred['valid'], inferred


def test_memoized_fallback_rejects_infeasible_internal_edge_corners_before_merge():
    from shapely.geometry import LineString
    request = graph_request(); frame = CoordinateFrame(request['scene']['coordinates'])
    positions = {'a':(100,100,80),'b':(500,100,80),'c':(700,100,80),'d':(1200,100,80)}
    request['scene']['airway_graph'] = {'version':'internal-corners','nodes':[{'id':node,'point_wkt':frame.to_wgs84(Point(point))} for node,point in positions.items()], 'edges':[
        {'id':edge_id,'from_node':start,'to_node':end,'airway_id':'demo-airway','direction':'FORWARD','weights':{'distance':weight}}
        for edge_id,start,end,weight in [('good','a','b',1),('bad','a','b',3),('bc','b','c',1),('cd','c','d',1)]]}
    request['scene']['airway_graph']['edges'][1]['geometry_wkt'] = frame.to_wgs84(LineString([positions['a'],(300,100,80),(300,100.1,80),(300.1,100.1,80),positions['b']]))
    request['task']['aircraft'] = {'min_turn_radius_m':60}
    route = plan_single_v2(**request); assert route['status'] == 'planned', route
    request['scene']['airway_graph']['edges'][0]['weights']['distance'] = 3
    request['scene']['airway_graph']['edges'][1]['weights']['distance'] = 1
    args = dict(trajectory_wkt=route['trajectory_wkt'],vertex_times_s=route['vertex_times_s'],motion_segments=route['motion_segments'])
    trace = dict(route['network_route'],total_cost=5,effective_cost=5)
    assert validate_route_v2(**request,**args,network_route=trace)['valid']
    inferred = validate_route_v2(**request,**args)
    assert inferred['valid'], inferred


def test_nonincident_selected_airway_cannot_authorize_an_earlier_junction():
    from shapely.geometry import LineString
    request = graph_request(); frame = CoordinateFrame(request['scene']['coordinates'])
    positions = {'a':(100,100,80),'b':(500,100,80),'c':(500,300,80),'d':(500,400,80)}
    airways = []
    for name,vertices,width in [('in',['a','b'],200),('out',['b','d'],20),('late',['a','b','c','d'],200)]:
        airway = deepcopy(request['scene']['airways'][0])
        airway.update(id=name,centerLine=frame.to_wgs84(LineString([positions[n] for n in vertices])),widthMeters=width)
        for label in ('startGrid','endGrid'):
            airway[label].update(minLon=104,maxLon=105,minLat=29,maxLat=30)
        airways.append(airway)
    request['scene']['airways'] = airways
    request['scene']['airway_graph'] = {'version':'incident-only','nodes':[{'id':node,'point_wkt':frame.to_wgs84(Point(point))} for node,point in positions.items()], 'edges':[
        {'id':edge_id,'from_node':start,'to_node':end,'airway_id':airway,'direction':'FORWARD','weights':{'distance':1}}
        for edge_id,start,end,airway in [('ab','a','b','in'),('bc','b','c','out'),('cd','c','d','late')]]}
    request['task']['aircraft'] = {'min_turn_radius_m':60}
    route = plan_single_v2(**request); assert route['status'] == 'planned', route
    request['scene']['airways'][0]['widthMeters'] = 20
    # late's broad corridor covers the b turn geometrically but is a later edge.
    planned = plan_single_v2(**request)
    assert planned['status'] == 'failed', planned
    assert any(v['code'] == 'NETWORK_ROUTE' for v in planned['validation']['violations'])
    args = dict(trajectory_wkt=route['trajectory_wkt'],vertex_times_s=route['vertex_times_s'],motion_segments=route['motion_segments'])
    for extra in ({},{'network_route':route['network_route']}):
        checked = validate_route_v2(**request,**args,**extra)
        assert not checked['valid'], checked
        assert any(v['code'] == 'NETWORK_ROUTE' for v in checked['violations'])
