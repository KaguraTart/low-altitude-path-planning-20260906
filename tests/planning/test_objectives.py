"""Real ENG-07 weighted candidate exploration and continuous field integration."""
from copy import deepcopy
import math

import numpy as np
import pytest
from pydantic import ValidationError
from shapely.geometry import Point, box

from tests.planning.test_rules import base_scene, task, geo
from uav_planner.errors import PlanningError
from uav_planner.v2.engine import plan_single_v2, plan_batch_v2, validate_route_v2
from uav_planner.v2.models import ObjectiveConfig, Task
from uav_planner.v2.scene import CompiledScene
from uav_planner.v2.motion import build_motion, Primitive, Motion
from uav_planner.v2.objectives import evaluate, field_integral, dominates
from tests.planning.test_search_grid_contract import LIMITS


def request():
    scene = base_scene()
    scene['bounds'] = dict(x_min=-20,x_max=100,y_min=-40,y_max=40,z_min=0,z_max=40)
    scene['resolution'] = [20,20,20]
    scene['rules'] = [dict(id='turn',rule_type='TURN',content={'minTurnRadius':0,'maxTurnAngle':180})]
    scene['datasets'] = [dict(id='risk',kind='risk',version='risk-v1',cells=[
        dict(bounds=scene['bounds'],values={'risk':0}),
        dict(bounds=dict(x_min=39,x_max=41,y_min=-4,y_max=4,z_min=0,z_max=40),values={'risk':100})])]
    t = task(start_wkt=geo(Point(0,0,20)),goal_wkt=geo(Point(80,0,20)),
             objectives=dict(weights=dict(distance=1,risk=1),normalization=dict(distance_m=80,risk_s=10)))
    return dict(scene=scene,task=t)


def independently_valid(req, route):
    return validate_route_v2(scene=req['scene'],task=req['task'],trajectory_wkt=route['trajectory_wkt'],
                             vertex_times_s=route['vertex_times_s'],motion_segments=route['motion_segments'])


def test_real_distance_risk_tradeoff_weights_select_different_routes():
    req = request()
    route = plan_single_v2(**req)
    assert route['status'] == 'planned',route
    assert route['objectives']['values']['risk']['raw'] == 0
    assert len(route['alternatives']) >= 1
    solutions = [route,*route['alternatives']]
    assert len({r['trajectory_wkt'] for r in solutions}) == len(solutions)
    assert route['optimization']['generated_candidates'] > 1
    assert route['optimization']['status'] == 'generator_exhausted'
    assert route['optimization']['scope'] == 'validated_candidates'
    assert route['optimization']['primary_executed_only']
    for r in solutions:
        assert independently_valid(req,r)['valid']
        assert r['traversed_grids'] and r['motion_segments'] and r['validation']['valid']
        obj = r['objectives']
        assert obj['weighted_score'] == pytest.approx(sum(v['weight']*v['normalized'] for v in obj['values'].values()))
        assert all(v['normalized'] == pytest.approx(v['raw']/v['normalization']) for v in obj['values'].values())
        vector = tuple(v['normalized'] for v in obj['values'].values())
        assert not any(dominates(tuple(v['normalized'] for v in o['objectives']['values'].values()),vector) for o in solutions)
    req['task']['objectives']['weights']['risk'] = .001
    direct = plan_single_v2(**req)
    assert direct['trajectory_wkt'] != route['trajectory_wkt']
    assert direct['metrics']['distance_m'] == pytest.approx(80,abs=1e-5)
    assert direct['objectives']['weighted_score'] <= min(a['objectives']['weighted_score'] for a in direct['alternatives'])


@pytest.mark.parametrize('config',[
    {'weights':{'distance':-1}}, {'weights':{'risk':math.nan}}, {'weights':{'distance':True}},
    {'weights':{'distance':0}}, {'normalization':{'time_s':0}}, {'max_solutions':True},
    {'max_iterations':0}, {'weights':{'energy':1}},
    {'energy_model':{'mass_kg':1,'cruise_power_w':1,'hover_power_w':1,'climb_efficiency':1.1}},
])
def test_strict_config_rejects_invalid_values(config):
    with pytest.raises(ValidationError):ObjectiveConfig.model_validate(config)


def test_free_only_and_no_unknown_layer_references():
    req=request()
    req['task']['route_mode']='strict'
    with pytest.raises(ValidationError):Task.model_validate(req['task'])
    req=request();req['scene']['datasets']=[]
    with pytest.raises(PlanningError,match='risk'):plan_single_v2(**req)
    req=request();req['task']['objectives']['cost_layers']=[dict(id='c',dataset_id='absent',weight=1,normalization=1,basis='time',unit='cost*s')]
    with pytest.raises(PlanningError,match='cost dataset'):plan_single_v2(**req)


def test_exact_energy_power_hover_and_positive_climb_wh():
    compiled=CompiledScene(request()['scene'])
    limits=dict(LIMITS)
    motion=build_motion([[0,0,20],[20,0,30],[40,0,20]],limits)
    hold=Primitive('hover',motion.primitives[-1].end,motion.primitives[-1].end,0,0,duration=5)
    motion.primitives.append(hold);motion.starts.append(float(motion.times[-1]))
    motion.times=np.append(motion.times,motion.times[-1]+5)
    config=ObjectiveConfig.model_validate(dict(weights=dict(distance=0,energy=2),normalization=dict(energy_wh=3),
        energy_model=dict(mass_kg=2,cruise_power_w=120,hover_power_w=80,climb_efficiency=.5)))
    result=evaluate(compiled,config,motion)
    expected=(120*(motion.times[-1]-5)+80*5+2*9.80665*10/.5)/3600
    energy=result['values']['energy']
    assert energy['unit']=='Wh'
    assert energy['raw']==pytest.approx(expected)
    assert energy['normalized']==pytest.approx(expected/3)
    assert result['weighted_score']==pytest.approx(2*expected/3)
    assert 'no descent regeneration' in result['energy_formula']


def test_narrow_cell_and_time_transitions_integrated_analytically():
    req=request();req['scene']['datasets'][0]['cells'][1]['bounds'].update(x_min=39.999,x_max=40.001)
    compiled=CompiledScene(req['scene']);motion=build_motion([[0,0,20],[80,0,20]],LIMITS)
    result=evaluate(compiled,ObjectiveConfig.model_validate(req['task']['objectives']),motion)
    assert result['values']['risk']['raw']==pytest.approx(100*.002/6,rel=1e-6)
    ds=compiled.datasets[0]
    ds.cells=ds.cells[:1]
    ds.cells[0].values={'risk':2};ds.cells[0].valid_to_s=5
    with pytest.raises(ValueError,match='coverage'):field_integral(motion,[ds],'risk','time')


def test_analytic_arc_crossings_cannot_skip_a_thin_cost_strip():
    req=request();req['scene']['datasets']=[dict(id='c',kind='cost',version='v1',cells=[
        dict(bounds=dict(x_min=-30,x_max=110,y_min=-50,y_max=110,z_min=0,z_max=40),values={'cost':0}),
        dict(bounds=dict(x_min=9.999,x_max=10.001,y_min=-50,y_max=110,z_min=0,z_max=40),values={'cost':7})])]
    compiled=CompiledScene(req['scene'])
    motion=build_motion([[0,0,20],[20,0,20],[20,20,20]],dict(LIMITS,min_turn_radius_m=10))
    value=field_integral(motion,compiled.datasets,'cost','distance')
    assert value>0
    assert value==pytest.approx(7*.002,rel=.01)


@pytest.mark.parametrize('kind',['risk','cost'])
@pytest.mark.parametrize('value',[-1,True,'2',float('inf')])
def test_nonnegative_dataset_values_required(kind,value):
    req=request();req['scene']['datasets']=[dict(id='bad',kind=kind,version='v1',cells=[dict(bounds=req['scene']['bounds'],values={kind:value})])]
    with pytest.raises((PlanningError,ValidationError)):CompiledScene(req['scene'])


def test_cost_layers_are_real_route_costs_with_complete_output():
    req=request();req['task']['objectives']['weights']={'distance':1,'risk':0}
    for id,lo,hi in [('north',0,40),('south',-40,0)]:
        req['scene']['datasets'].append(dict(id=id,kind='cost',version='v3',cells=[
            dict(bounds=req['scene']['bounds'],values={'cost':0}),
            dict(bounds=dict(x_min=20,x_max=60,y_min=lo,y_max=hi,z_min=0,z_max=40),values={'cost':10})]))
    req['task']['objectives']['cost_layers']=[dict(id='north_cost',dataset_id='north',weight=1,normalization=1,basis='distance',unit='cost*m'),
                                            dict(id='south_cost',dataset_id='south',weight=.001,normalization=1,basis='time',unit='cost*s')]
    north=plan_single_v2(**req)
    req['task']['objectives']['cost_layers'][0]['weight']=.001
    req['task']['objectives']['cost_layers'][1]['weight']=100
    south=plan_single_v2(**req)
    assert north['status']==south['status']=='planned'
    assert north['trajectory_wkt'] != south['trajectory_wkt']
    assert north['objectives']['values']['north_cost']['raw'] < south['objectives']['values']['north_cost']['raw']
    assert north['objectives']['values']['south_cost']['raw'] > south['objectives']['values']['south_cost']['raw']
    assert north['objectives']['dataset_versions']['north']=='v3'
    assert independently_valid(req,south)['valid']


@pytest.mark.parametrize('unknown',['nodata','height','time','space'])
def test_missing_coverage_never_becomes_zero(unknown):
    req=request();ds=req['scene']['datasets'][0];ds['cells']=ds['cells'][:1]
    if unknown=='nodata':ds['cells'][0]['values']={'nodata':True}
    if unknown=='height':ds['cells'][0]['bounds']['z_max']=10
    if unknown=='time':ds['valid_from']='2026-10-17T00:00:00Z'
    if unknown=='space':ds['cells'][0]['bounds']['x_min']=10
    route=plan_single_v2(**req)
    assert route['status']=='failed'


@pytest.mark.parametrize('budget,status',[('max_iterations','iteration_limit'),('max_solutions','solution_limit')])
def test_optional_optimization_budgets_are_explicit(budget,status):
    req=request();req['task']['objectives'][budget]=1
    route=plan_single_v2(**req)
    assert route['optimization']['status']==status
    assert route['optimization']['generated_candidates']==1
    assert route['status']=='planned'


def test_grid_options_budget_and_default_no_extra_exploration(monkeypatch):
    req=request();req['task']['search']={'neighbors':6,'max_expanded_nodes':1}
    route=plan_single_v2(**req)
    assert route['code']=='SEARCH_BUDGET_EXCEEDED'
    assert route['search_report']['expanded_nodes']==1
    assert route['optimization']['status']=='search_budget_exceeded'
    req=request();req['task'].pop('objectives')
    from uav_planner.v2 import objectives
    monkeypatch.setattr(objectives,'prepare',lambda *a,**kw:pytest.fail('default invokes objective search'))
    route=plan_single_v2(**req)
    assert route['status']=='planned' and route['objectives'] is None and route['alternatives']==[]


def test_hard_obstacles_and_batch_alternatives_all_independently_safe():
    req=request()
    req['scene']['bounds'].update(y_min=-60,y_max=60)
    req['scene']['obstacles']=[dict(id='hard',geometry_wkt=geo(box(35,-5,45,5)),min_altitude=0,max_altitude=40,alt_reference='EG')]
    route=plan_single_v2(**req)
    assert route['status']=='planned'
    assert all(independently_valid(req,r)['valid'] for r in [route,*route['alternatives']])
    req=request();a=deepcopy(req['task']);a['id']='a';b=deepcopy(a);b['id']='b'
    result=plan_batch_v2(scene=req['scene'],tasks=[a,b],execution={'workers':2})
    assert result['success_count']==2 and not result['conflicts']
    from uav_planner.v2 import native
    from uav_planner.v2.engine import _reconstruct_motion
    from uav_planner.v2.coordinates import CoordinateFrame
    frame=CoordinateFrame(req['scene']['coordinates'])
    def samples(r):
        points=frame.to_local(r['trajectory_wkt'],'EG')
        motion=_reconstruct_motion(r['motion_segments'],np.asarray(points.coords),np.asarray(r['vertex_times_s']))
        return motion.collision_samples()
    for i,r in enumerate(result['routes']):
        other=result['routes'][1-i]
        for alt in [r,*r['alternatives']]:
            pa,ta=samples(alt);pb,tb=samples(other)
            assert not native.continuous_conflicts([pa,pb],[ta,tb],10.7,10.3)['conflicts']


def test_actual_time_energy_tradeoff_changes_selected_geometry():
    req=request();req['scene']['bounds'].update(y_min=-60,y_max=60,z_max=60)
    req['scene']['datasets'][0]['cells'][1]['bounds'].update(y_min=-20,y_max=20,z_max=28)
    req['task']['aircraft']={'climb_speed_mps':12,'descent_speed_mps':12}
    req['task']['objectives']={'weights':{'distance':0,'time':1,'risk':10000,'energy':0},
        'energy_model':{'mass_kg':100,'hover_power_w':1,'cruise_power_w':1,'climb_efficiency':.5}}
    fast=plan_single_v2(**req)
    req['task']['objectives']['weights'].update(time=0,energy=1)
    efficient=plan_single_v2(**req)
    assert fast['status']==efficient['status']=='planned'
    assert fast['trajectory_wkt'] != efficient['trajectory_wkt']
    assert fast['objectives']['values']['risk']['raw']==efficient['objectives']['values']['risk']['raw']==0
    assert fast['metrics']['flight_time_s'] < efficient['metrics']['flight_time_s']
    assert fast['objectives']['values']['energy']['raw'] > efficient['objectives']['values']['energy']['raw']*100
    assert independently_valid(req,fast)['valid'] and independently_valid(req,efficient)['valid']


def test_waypoint_hover_is_preserved_and_accounted_in_energy():
    req=request();req['task']['waypoints']=[{'wkt':geo(Point(40,0,20)),'hover_s':5}]
    req['task']['objectives']={'weights':{'distance':0,'energy':1},'energy_model':
        {'mass_kg':1,'hover_power_w':100,'cruise_power_w':50,'climb_efficiency':1}}
    route=plan_single_v2(**req)
    assert route['status']=='planned',route
    assert independently_valid(req,route)['valid']
    hovering=sum(s['end_time_s']-s['start_time_s'] for s in route['motion_segments'] if s['kind'] in {'hover','yaw'})
    assert hovering==pytest.approx(5)
    expected=(50*(route['metrics']['flight_time_s']-hovering)+100*hovering)/3600
    assert route['objectives']['values']['energy']['raw']==pytest.approx(expected)


def test_request_deadline_is_shared_by_exploration_and_validation(monkeypatch):
    from uav_planner.v2 import objectives,engine
    clock=[100.]
    original=objectives.evaluate
    def consuming(*args,**kwargs):
        clock[0]+=1
        return original(*args,**kwargs)
    monkeypatch.setattr(engine.time,'perf_counter',lambda:clock[0])
    monkeypatch.setattr(objectives,'evaluate',consuming)
    route=plan_single_v2(**request(),time_budget_s=3)
    assert route['status']=='timeout' and route['code']=='TIME_BUDGET_EXCEEDED'
    assert route['optimization']['status']=='deadline_exceeded'
    assert not route['alternatives']


@pytest.mark.parametrize('unknown',['nodata','height','time','space'])
def test_custom_cost_missing_coverage_fails_closed(unknown):
    req=request();ds=req['scene']['datasets'][0];ds.update(kind='cost',id='cost');ds['cells']=ds['cells'][:1]
    ds['cells'][0]['values']={'cost':2}
    req['task']['objectives']={'weights':{'distance':0},'cost_layers':[
        {'id':'layer','dataset_id':'cost','weight':1,'normalization':1,'basis':'time','unit':'cost*s'}]}
    if unknown=='nodata':ds['cells'][0]['values']={'nodata':True}
    if unknown=='height':ds['cells'][0]['bounds']['z_max']=10
    if unknown=='time':ds['valid_from']='2026-10-17T00:00:00Z'
    if unknown=='space':ds['cells'][0]['bounds']['x_min']=10
    route=plan_single_v2(**req)
    assert route['status']=='failed' and route['objectives'] is None


def test_reserved_and_duplicate_custom_layer_ids_rejected_and_defaults_unlimited():
    layer={'id':'distance','dataset_id':'c','weight':1,'normalization':1,'basis':'distance','unit':'cost*m'}
    with pytest.raises(ValidationError):ObjectiveConfig.model_validate({'cost_layers':[layer]})
    layer['id']='custom'
    with pytest.raises(ValidationError):ObjectiveConfig.model_validate({'cost_layers':[layer,layer]})
    assert ObjectiveConfig().max_solutions is None and ObjectiveConfig().max_iterations is None


def test_zero_duration_route_still_requires_known_field_value():
    req=request();compiled=CompiledScene(req['scene'])
    motion=build_motion([[0,0,20]],LIMITS)
    assert evaluate(compiled,ObjectiveConfig.model_validate(req['task']['objectives']),motion)['values']['risk']['raw']==0
    compiled.datasets[0].cells=[]
    with pytest.raises(ValueError,match='coverage'):evaluate(compiled,ObjectiveConfig.model_validate(req['task']['objectives']),motion)


def test_explicit_grid_alternatives_share_aggregate_search_report():
    req=request();req['task']['search']={'neighbors':26,'algorithm':'DIJKSTRA'}
    route=plan_single_v2(**req)
    assert route['status']=='planned' and route['alternatives']
    assert route['search_report']['expanded_nodes']==route['optimization']['iterations']
    assert route['search_report']['algorithm']=='DIJKSTRA'
    assert all(a['search_report']==route['search_report'] for a in route['alternatives'])


def test_global_reservation_deadline_also_marks_optimization_incomplete(monkeypatch):
    from uav_planner.v2 import engine,native
    clock=[100.]
    original=native.schedule_paths
    def expire(*args,**kwargs):
        result=original(*args,**kwargs)
        clock[0]+=100
        return result
    monkeypatch.setattr(engine.time,'perf_counter',lambda:clock[0])
    monkeypatch.setattr(native,'schedule_paths',expire)
    route=plan_single_v2(**request(),time_budget_s=60)
    assert route['status']=='timeout' and route['code']=='TIME_BUDGET_EXCEEDED'
    assert route['optimization']['status']=='deadline_exceeded'


def extreme_request(case):
    req=request()
    if case=='normalization':req['task']['objectives']['normalization']['distance_m']=1e-320
    elif case=='weighted_score':req['task']['objectives']['weights']['distance']=1e308
    elif case=='risk_integral':req['scene']['datasets'][0]['cells'][0]['values']['risk']=1e308
    elif case.startswith('cost_'):
        req['scene']['datasets']=[dict(id='cost',kind='cost',version='v1',cells=[
            dict(bounds=req['scene']['bounds'],values={'cost':1e308})])]
        req['task']['objectives']={'weights':{'distance':0},'cost_layers':[
            dict(id='custom',dataset_id='cost',weight=1,normalization=1,basis=case[5:],unit='custom-unit')]}
    elif case=='energy':
        req['task']['goal_wkt']=geo(Point(80,0,25))
        req['task']['objectives']={'weights':{'energy':1},'energy_model':
            {'mass_kg':1e308,'hover_power_w':1e308,'cruise_power_w':1e308,'climb_efficiency':1e-300}}
    return req


@pytest.mark.parametrize('case',['normalization','weighted_score','risk_integral','cost_distance','cost_time','energy'])
def test_extreme_finite_objectives_raise_typed_errors_instead_of_nonfinite_values(case):
    with pytest.raises(PlanningError,match='nonfinite') as caught:
        plan_single_v2(**extreme_request(case))
    assert caught.value.code=='INVALID_INPUT'
    assert caught.value.detail['operation']


@pytest.mark.parametrize('large_powers',[False,True])
def test_representable_energy_with_extreme_finite_inputs_is_not_arbitrarily_capped(large_powers):
    req=request();req['task']['objectives']={'weights':{'distance':0,'energy':1},'max_solutions':1,
        'energy_model':{'mass_kg':1e308,'hover_power_w':1e308 if large_powers else 1,
                        'cruise_power_w':1e308 if large_powers else 1,'climb_efficiency':1e-300}}
    route=plan_single_v2(**req)
    assert route['status']=='planned'
    value=route['objectives']['values']['energy']['raw']
    assert math.isfinite(value)
    assert value==pytest.approx(route['metrics']['flight_time_s']/3600*(1e308 if large_powers else 1))
