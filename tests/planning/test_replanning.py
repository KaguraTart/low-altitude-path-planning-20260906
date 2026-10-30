"""Incremental reuse, physical handoffs and independently checked public contract."""
from copy import deepcopy
from datetime import timedelta
import json

import numpy as np
import pytest
from fastapi.encoders import jsonable_encoder
from fastapi.testclient import TestClient
from shapely.geometry import Point, box, LineString

from tests.planning.test_rules import base_scene, task, geo, ORIGIN
from uav_planner import plan_single_v2, replan_v2, validate_route_v2
from uav_planner.errors import PlanningError
from uav_planner.v2 import engine as e
from uav_planner.v2.replanning import tangent
from server.app import app


def request():
    scene=base_scene()
    scene['rules']=[dict(id='turn',rule_type='TURN',content={'minTurnRadius':0,'maxTurnAngle':180})]
    t=task(); old=plan_single_v2(scene=scene,task=t)
    assert old['status']=='planned'
    sid=e.register_scene(scene)['scene_id'];model,compiled,_,_=e.registry.get(sid)
    motion=e._reconstruct_motion(old['motion_segments'],e._timed_local(compiled,old['trajectory_wkt'],np.array(old['vertex_times_s']),'EG',model),np.array(old['vertex_times_s']))
    p=motion.primitives[0]; now=3.
    state=dict(position_wkt=geo(Point(*p.position(p.distance_at(now)))),
               velocity_xyz_mps=(tangent(p)*6).tolist(),at=ORIGIN+timedelta(seconds=now))
    remaining={**t,'start_wkt':state['position_wkt'],'departure_time':state['at']}
    return dict(current_state=state,previous_route=old,previous_scene=scene,previous_task=t,
                remaining_task=remaining,scene=deepcopy(scene),
                event=dict(id='e',type='aircraft_state',at=state['at'],old_value=90,new_value=85))


def obstacle(req,x0=65,x1=75):
    req['scene']['obstacles']=[dict(id='block',geometry_wkt=geo(box(x0,-8,x1,8)),min_altitude=80,max_altitude=120,alt_reference='EG')]


def check(req,route,**updates):
    return validate_route_v2(scene=req['scene'],task=req['remaining_task'],
        trajectory_wkt=route['trajectory_wkt'],vertex_times_s=route['vertex_times_s'],
        motion_segments=route['motion_segments'],initial_velocity_xyz_mps=route['initial_velocity_xyz_mps'],**updates)


def test_unaffected_suffix_reused_without_search(monkeypatch):
    req=request()
    def fail(*a,**kw):raise AssertionError('reuse must not search')
    monkeypatch.setattr(e,'_prepare_candidate',fail)
    result=replan_v2(**req)
    assert result['decision']=='reused'
    assert result['new_search_calls']==0
    assert result['reused_distance_m']==pytest.approx(91,abs=1e-5)
    assert result['reused_time_s']==pytest.approx(result['route']['vertex_times_s'][-1]-3)
    assert result['reused_primitives']>0
    assert check(req,result['route'])['valid']


def test_partial_recompute_preserves_verified_prefix_and_physical_handoff():
    req=request();obstacle(req)
    result=replan_v2(**req)
    assert result['decision']=='replanned',result
    assert 0<result['reused_distance_m']<91
    assert result['new_search_calls']==1
    route=result['route']; assert check(req,route)['valid']
    assert route['initial_velocity_xyz_mps']==req['current_state']['velocity_xyz_mps']
    segments=route['motion_segments']
    assert any(p['kind']=='yaw' for p in segments)
    assert any(p['profile']['decel_time_s']>0 and p['end_speed_mps']==0 for p in segments)
    assert all(a['end_time_s']==pytest.approx(b['start_time_s']) for a,b in zip(segments,segments[1:]))
    wrong=deepcopy(route);wrong['initial_velocity_xyz_mps']=[0,6,0]
    assert not check(req,wrong)['valid']
    # Omitting the declaration uses rest and must reject this moving start.
    assert not validate_route_v2(scene=req['scene'],task=req['remaining_task'],trajectory_wkt=route['trajectory_wkt'],vertex_times_s=route['vertex_times_s'],motion_segments=route['motion_segments'])['valid']


@pytest.mark.parametrize('trigger,reason',[
    ({'last_event_time':ORIGIN+timedelta(seconds=3)},'stale_event'),
    ({'threshold':6},'below_threshold'),
    ({'min_interval_s':10,'last_replan_time':ORIGIN},'frequency_limited'),
    ({'max_frequency_hz':.1,'last_replan_time':ORIGIN},'frequency_limited'),
])
def test_ignored_event_never_returns_unsafe_old_route(trigger,reason):
    req=request();obstacle(req);req['trigger']=trigger
    result=replan_v2(**req)
    assert result['decision']=='ignored' and result['reason']==reason
    assert result['route'] is None and result['new_search_calls']==0
    if reason=='frequency_limited':
        assert result['next_eligible_time']=='2026-10-16T08:00:10+08:00'


@pytest.mark.parametrize('mutation,match',[
    (lambda r:r['previous_route']['motion_segments'][0].update(length_m=500),'independent validation'),
    (lambda r:r['previous_route'].update(motion_segments=[]),'MOTION_MODEL_REQUIRED'),
    (lambda r:r['previous_route']['coordinate_context'].update(origin_lon=100),'coordinate/time context'),
    (lambda r:r['previous_scene'].update(horizon_s=10),'horizon'),
    (lambda r:r['scene']['coordinates'].update(origin_lon=100),'same canonical'),
    (lambda r:r['event'].update(changed_resource_ids=['missing']),'unknown resource'),
    (lambda r:r['remaining_task'].update(rule_ids=['missing']),'missing rules'),
    (lambda r:r['current_state'].update(velocity_xyz_mps=[1000,0,0]),'velocity exceeds'),
    (lambda r:r.update(trigger={'threshold':2},event={'id':'e','type':'rule','at':ORIGIN,'changed_resource_ids':['turn']}),'requires numeric'),
])
def test_forged_context_unknown_references_and_state_rejected(mutation,match):
    req=request();mutation(req)
    with pytest.raises(PlanningError,match=match):replan_v2(**req)


def test_blocked_braking_fails_without_executable_route():
    req=request();obstacle(req,x0=12,x1=25)
    result=replan_v2(**req)
    assert result['decision']=='failed' and result['reason']=='unsafe_braking'
    assert result['route'] is None and result['new_search_calls']==0


def test_authoritative_mismatched_velocity_restarts_continuously():
    req=request();req['current_state']['velocity_xyz_mps']=[0,2,0]
    result=replan_v2(**req)
    assert result['decision']=='replanned' and result['reason']=='current_state_mismatch',result
    assert result['reused_primitives']==0
    assert check(req,result['route'])['valid']


def test_dynamic_collision_cannot_be_returned_as_valid_reuse():
    req=request()
    req['scene']['dynamic_obstacles']=[dict(id='traffic',trajectory_wkt=geo(LineString([(60,0,100),(60,0,100)])),
        vertex_times_s=[0,100],alt_reference='EG',horizontal_separation_m=3,vertical_separation_m=3)]
    result=replan_v2(**req)
    assert result['decision'] in ('replanned','failed'),result
    if result['route'] is not None:assert check(req,result['route'])['valid']


@pytest.mark.parametrize('change',[
    {'state':{'remaining_flight_time_s':1}},
    {'state':{'remaining_range_m':10}},
    {'latest_arrival':ORIGIN+timedelta(seconds=4)},
    {'waypoints':[{'wkt':geo(Point(0,100,100)),'latest':ORIGIN+timedelta(seconds=4)}]},
])
def test_entire_remaining_mission_budgets_windows_and_waypoints(change):
    req=request();req['remaining_task'].update(change)
    result=replan_v2(**req)
    assert result['decision']=='failed' and result['route'] is None,result


def test_removed_resource_id_is_known_in_old_scene():
    req=request();req['scene']['rules']=[]
    req['event'].update(type='rule',changed_resource_ids=['turn'])
    assert replan_v2(**req)['decision']=='reused'


def test_shared_absolute_budget_includes_previous_proof():
    req=request();req['time_budget_s']=1e-12
    with pytest.raises(PlanningError,match='time budget'):replan_v2(**req)


def test_typed_http_sdk_and_validator_contract():
    req=request();obstacle(req)
    client=TestClient(app)
    response=client.post('/api/v2/plan/replan',json=jsonable_encoder(req))
    assert response.status_code==200,response.text
    result=response.json();assert result['decision']=='replanned'
    route=result['route']
    validation=client.post('/api/v2/validate',json=jsonable_encoder(dict(scene=req['scene'],task=req['remaining_task'],
        trajectory_wkt=route['trajectory_wkt'],vertex_times_s=route['vertex_times_s'],motion_segments=route['motion_segments'],initial_velocity_xyz_mps=route['initial_velocity_xyz_mps'])))
    assert validation.status_code==200 and validation.json()['valid'],validation.text
    for path in ('unexpected', 'event', 'current_state'):
        bad=jsonable_encoder(req)
        if path=='unexpected':bad[path]=True
        else:bad[path]['unexpected']=True
        assert client.post('/api/v2/plan/replan',json=bad).status_code==400
    schema=client.get('/openapi.json').json()
    assert schema['paths']['/api/v2/plan/replan']['post']['responses']['200']['content']['application/json']['schema']['$ref'].endswith('ReplanResponse')
    with open('/tmp/replan_handoff_request.json','w') as f:json.dump(jsonable_encoder(req),f,indent=2)
    with open('/tmp/replan_handoff_response.json','w') as f:json.dump(result,f,indent=2)


def test_changed_time_rule_cannot_be_filtered_out_of_braking_proof():
    req=request()
    req['scene']['rules'].append(dict(id='closed',rule_type='TIME',content={
        'earliestDeparture':'08:01'}))
    result=replan_v2(**req)
    assert result['decision']=='failed' and result['reason']=='unsafe_braking'
    assert any(v['code']=='TIME' for v in result['violations'])
    assert result['route'] is None


def test_partial_hover_proof_keeps_selected_hover_rule():
    from uav_planner.v2.motion import Primitive
    from uav_planner.v2.replanning import _motion,_validate
    req=request();req['scene']['rules'].append(dict(id='hold',rule_type='HOVER',content={'maxHoverTime':10,'defaultHoverTime':0}))
    sid=e.register_scene(req['scene'])['scene_id'];model,compiled,_,_=e.registry.get(sid)
    t=e._parse(e.Task,req['remaining_task']);point=e._local(compiled,t.start_wkt,t)
    hover=_motion([Primitive('hover',point,point,0,0,duration=1)],[3])
    violations=_validate(hover,t,model,compiled,sid,[0,0,0],partial=True)
    assert any(v['code']=='HOVER' for v in violations)


@pytest.mark.parametrize('remaining_time',[200,1])
def test_triggered_low_battery_return_preserves_obligations(remaining_time):
    req=request()
    req['scene']['rules'].append(dict(id='return',rule_type='RTL',content={
        'rtlAltitude':80,'rtlOnLowBattery':True,'lowBatteryPercent':20}))
    req['remaining_task']['state']=dict(at=req['current_state']['at'],battery_percent=15,
        remaining_flight_time_s=remaining_time,remaining_range_m=1000,home_wkt=geo(Point(0,30,100)),alt_reference='EG')
    result=replan_v2(**req)
    if remaining_time==1:
        assert result['decision']=='failed' and result['route'] is None,result
    else:
        assert result['decision']=='replanned',result
        assert result['route']['action']=='return_to_home'
        assert result['route']['goal_wkt']==req['remaining_task']['state']['home_wkt']
        assert check(req,result['route'])['valid']


@pytest.mark.parametrize('at',[.5,10,18])
def test_exact_clipping_inside_acceleration_cruise_and_deceleration(at):
    from uav_planner.v2.replanning import _speed
    req=request();old=req['previous_route']
    sid=e.register_scene(req['previous_scene'])['scene_id'];model,compiled,_,_=e.registry.get(sid)
    p=e._reconstruct_motion(old['motion_segments'],np.zeros((0,3)),np.zeros(0)).primitives[0]
    point=p.position(p.distance_at(at)); velocity=tangent(p)*_speed(p,at)
    req['current_state']=dict(position_wkt=geo(Point(*point)),velocity_xyz_mps=velocity.tolist(),at=ORIGIN+timedelta(seconds=at))
    req['remaining_task'].update(start_wkt=req['current_state']['position_wkt'],departure_time=req['current_state']['at'])
    req['event']['at']=req['current_state']['at']
    result=replan_v2(**req)
    assert result['decision']=='reused',result
    assert check(req,result['route'])['valid']


def test_exact_clipping_inside_curved_analytic_motion():
    from uav_planner.v2.motion import build_motion
    from uav_planner.v2.replanning import _speed
    req=request();req['previous_scene']['rules'][0]['content']['minTurnRadius']=10
    req['scene']=deepcopy(req['previous_scene'])
    req['previous_task']['goal_wkt']=geo(Point(50,50,100))
    sid=e.register_scene(req['previous_scene'])['scene_id'];model,compiled,_,_=e.registry.get(sid)
    t=e._parse(e.Task,req['previous_task']);profile=e._profile(model,t)
    motion=build_motion([[0,0,100],[50,0,100],[50,50,100]],e._limits(compiled,t,profile))
    req['previous_route']=e._result(compiled,model,t,profile,motion)
    index=next(i for i,p in enumerate(motion.primitives) if p.kind=='arc')
    p=motion.primitives[index];local=p.duration*.4;at=motion.starts[index]+local
    point=p.position(p.distance_at(local))
    from copy import copy
    clipped=copy(p);clipped.start=point
    velocity=tangent(clipped)*_speed(p,local)
    req['current_state']=dict(position_wkt=geo(Point(*point)),velocity_xyz_mps=velocity.tolist(),at=ORIGIN+timedelta(seconds=at))
    req['remaining_task'].update(start_wkt=req['current_state']['position_wkt'],goal_wkt=t.goal_wkt,departure_time=req['current_state']['at'])
    req['event']['at']=req['current_state']['at']
    result=replan_v2(**req)
    assert result['decision']=='reused',result
    assert check(req,result['route'])['valid']


def test_objective_replan_searches_and_scores_whole_safe_alternatives():
    from tests.planning.test_objectives import request as objective_request
    data=objective_request();previous=deepcopy(data['task']);previous.pop('objectives')
    old=plan_single_v2(scene=data['scene'],task=previous)
    assert old['status']=='planned'
    state=dict(position_wkt=geo(Point(9,0,20)),velocity_xyz_mps=[6,0,0],at=ORIGIN+timedelta(seconds=3))
    remaining=deepcopy(data['task']);remaining.update(start_wkt=state['position_wkt'],departure_time=state['at'])
    req=dict(previous_task=previous,previous_scene=data['scene'],previous_route=old,current_state=state,
             remaining_task=remaining,scene=deepcopy(data['scene']),
             event=dict(id='weights',type='mission',at=state['at'],old_value=.001,new_value=1))
    result=replan_v2(**req)
    assert result['decision']=='replanned' and result['reason']=='objective_reoptimization',result
    assert result['new_search_calls']>0
    assert result['reused_primitives']==0
    route=result['route']
    assert route['objectives']['values']['risk']['raw']==0
    assert route['optimization']['generated_candidates']>1
    assert route['alternatives']
    for r in [route,*route['alternatives']]:
        assert check(req,r)['valid']
        assert r['objectives']['weighted_score']==pytest.approx(sum(v['weight']*v['normalized'] for v in r['objectives']['values'].values()))
        assert r['initial_velocity_xyz_mps']==state['velocity_xyz_mps']
    req['remaining_task']['objectives']['weights']['risk']=.001
    cheap=replan_v2(**req)
    assert cheap['decision']=='replanned',cheap
    assert cheap['route']['trajectory_wkt']!=route['trajectory_wkt']
    assert cheap['route']['metrics']['distance_m']<route['metrics']['distance_m']
    assert check(req,cheap['route'])['valid']


def test_current_time_within_scene_can_restart_after_previous_route_ended():
    req=request();at=30
    state=dict(position_wkt=geo(Point(100,0,100)),velocity_xyz_mps=[0,0,0],at=ORIGIN+timedelta(seconds=at))
    req['current_state']=state
    req['remaining_task'].update(start_wkt=state['position_wkt'],goal_wkt=geo(Point(120,0,100)),departure_time=state['at'])
    req['event']['at']=state['at']
    result=replan_v2(**req)
    assert result['decision']=='replanned' and result['reused_primitives']==0,result
    assert check(req,result['route'])['valid']


def test_stationary_remaining_mission_preserves_singleton_sdk_and_http(monkeypatch):
    req=request();req['current_state']['velocity_xyz_mps']=[0,0,0]
    req['remaining_task']['goal_wkt']=req['current_state']['position_wkt']
    checks=[];original=e.validate_route_v2
    def counted(**kwargs):
        checks.append(kwargs)
        return original(**kwargs)
    monkeypatch.setattr(e,'validate_route_v2',counted)
    sdk=replan_v2(**req)
    assert sdk['decision']=='replanned',sdk
    assert sdk['route']['trajectory_wkt'].startswith('POINT Z')
    assert sdk['route']['vertex_times_s']==[3]
    assert sdk['route']['motion_segments']==[]
    assert sdk['route']['metrics']['distance_m']==0
    assert check(req,sdk['route'])['valid']
    assert any(len(c['vertex_times_s'])==1 for c in checks)
    response=TestClient(app).post('/api/v2/plan/replan',json=jsonable_encoder(req))
    assert response.status_code==200,response.text
    assert response.json()['route']==sdk['route']
    # The singleton must still undergo scene checks rather than vacuous success.
    obstacle(req,x0=0,x1=20)
    failed=replan_v2(**req)
    assert failed['decision']=='failed' and failed['route'] is None,failed


def test_replan_assembles_all_valid_objective_tails_before_dominance(monkeypatch):
    import uav_planner.v2.replanning as replanning
    import uav_planner.v2.objectives as objectives
    from tests.planning.test_objectives import request as objective_request
    data=objective_request();previous=deepcopy(data['task']);previous.pop('objectives')
    old=plan_single_v2(scene=data['scene'],task=previous)
    state=dict(position_wkt=geo(Point(9,0,20)),velocity_xyz_mps=[6,0,0],at=ORIGIN+timedelta(seconds=3))
    remaining=deepcopy(data['task']);remaining.update(start_wkt=state['position_wkt'],departure_time=state['at'])
    req=dict(previous_task=previous,previous_scene=data['scene'],previous_route=old,current_state=state,
        remaining_task=remaining,scene=deepcopy(data['scene']),event=dict(id='weights',type='mission',at=state['at'],old_value=0,new_value=1))
    captured={};original_prepare=objectives.prepare;original_assemble=replanning._assemble
    def prepare(*args,**kwargs):
        assert kwargs['retain_all_candidates'] is True
        candidate,failure=original_prepare(*args,**kwargs)
        assert failure is None
        tails=candidate['objective_candidates']
        captured['tails']=len(tails)
        captured['tail_frontier']=len(objectives.frontier(tails))
        return candidate,failure
    def assemble(*args,**kwargs):
        captured['assembled']=captured.get('assembled',0)+1
        return original_assemble(*args,**kwargs)
    monkeypatch.setattr(objectives,'prepare',prepare)
    monkeypatch.setattr(replanning,'_assemble',assemble)
    result=replan_v2(**req)
    assert result['decision']=='replanned',result
    assert captured['assembled']==captured['tails']
    assert captured['tails']>captured['tail_frontier']
    assert result['route']['optimization']['validated_candidates']==captured['assembled']
    for route in [result['route'],*result['route']['alternatives']]:
        assert check(req,route)['valid']


def test_explicit_dijkstra_six_search_reports_actual_recompute_and_zero_reuse():
    req=request()
    req['scene'].update(bounds=dict(x_min=-20,x_max=140,y_min=-40,y_max=40,z_min=80,z_max=120),resolution=[10,10,10])
    req['remaining_task']['search']=dict(algorithm='DIJKSTRA',neighbors=6)
    reused=replan_v2(**req)
    assert reused['decision']=='reused',reused
    report=reused['route']['search_report']
    assert report==dict(algorithm='DIJKSTRA',neighbors=6,grid_level=None,expanded_nodes=0,max_expanded_nodes=None)
    assert reused['new_search_calls']==0
    req['current_state']['velocity_xyz_mps']=[0,0,0]
    obstacle(req)
    result=replan_v2(**req)
    assert result['decision']=='replanned',result
    report=result['route']['search_report']
    assert report['algorithm']=='DIJKSTRA' and report['neighbors']==6
    assert report['expanded_nodes']>0
    assert report['max_expanded_nodes'] is None
    assert result['new_search_calls']>0
    assert check(req,result['route'])['valid']


def test_previous_normal_stationary_point_can_restart_moving_sdk_and_http():
    req=request()
    previous=deepcopy(req['previous_task']);previous['goal_wkt']=previous['start_wkt']
    old=plan_single_v2(scene=req['previous_scene'],task=previous)
    assert old['status']=='planned' and old['trajectory_wkt'].startswith('POINT Z'),old
    assert old['motion_segments']==[] and len(old['vertex_times_s'])==1
    req.update(previous_task=previous,previous_route=old)
    req['current_state'].update(position_wkt=previous['start_wkt'],velocity_xyz_mps=[0,0,0])
    req['remaining_task']['start_wkt']=req['current_state']['position_wkt']
    result=replan_v2(**req)
    assert result['decision']=='replanned' and result['reused_primitives']==0,result
    assert result['route']['motion_segments'] and result['route']['metrics']['distance_m']>0
    assert check(req,result['route'])['valid']
    response=TestClient(app).post('/api/v2/plan/replan',json=jsonable_encoder(req))
    assert response.status_code==200,response.text
    assert response.json()['decision']=='replanned'
    assert check(req,response.json()['route'])['valid']


@pytest.mark.parametrize('settings',[{'max_frequency_hz':1e-320},{'min_interval_s':1e308}])
def test_unrepresentable_trigger_interval_is_typed_sdk_and_http_input_error(settings):
    req=request();req['trigger']={**settings,'last_replan_time':ORIGIN}
    with pytest.raises(PlanningError,match='not representable') as failure:
        replan_v2(**req)
    assert failure.value.code=='INVALID_INPUT'
    assert failure.value.detail['last_replan_time']==ORIGIN.isoformat()
    for key,value in settings.items():
        assert failure.value.detail[key]==value
    response=TestClient(app).post('/api/v2/plan/replan',json=jsonable_encoder(req))
    assert response.status_code==400,response.text
    assert response.json()['error']['code']=='INVALID_INPUT'
    assert 'not representable' in response.json()['error']['message']


def test_large_representable_trigger_interval_has_no_artificial_cap():
    req=request();req['trigger']={'min_interval_s':1e10,'last_replan_time':ORIGIN}
    result=replan_v2(**req)
    assert result['decision']=='ignored' and result['reason']=='frequency_limited'
    assert result['next_eligible_time']==(ORIGIN+timedelta(seconds=1e10)).isoformat()
