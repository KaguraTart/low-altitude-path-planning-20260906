"""Explicit search choices, continuous slope/visibility and analytic grid export."""
import math
from copy import deepcopy
import numpy as np
import pytest
from pydantic import ValidationError
from shapely.geometry import Point
from uav_planner.errors import PlanningError
from uav_planner.v2 import native
from uav_planner.v2.models import Scene, SearchOptions
from uav_planner.v2.engine import plan_single_v2, validate_route_v2
from uav_planner.v2.grid_traversal import traversed_grids
from uav_planner.v2.motion import build_motion, Primitive, Motion
from tests.planning.test_rules import base_scene, task, geo, dataset, check, rule_fails

LIMITS=dict(max_speed_mps=12,cruise_speed_mps=6,climb_speed_mps=3,descent_speed_mps=3,acceleration_mps2=2,deceleration_mps2=2,max_yaw_rate_deg_s=60,min_turn_radius_m=0)


def request():
    scene=base_scene()
    scene['bounds']=dict(x_min=-30,x_max=150,y_min=-60,y_max=60,z_min=0,z_max=100)
    scene['resolution']=[10,10,10]
    scene['rules']=[dict(id='turn',rule_type='TURN',content={'minTurnRadius':0,'maxTurnAngle':180})]
    return dict(scene=scene,task=task(start_wkt=geo(Point(0,0,20)),goal_wkt=geo(Point(100,30,20))))


@pytest.mark.parametrize('value',[0,True,6.0,'6',7])
def test_strict_neighbor_contract(value):
    with pytest.raises(ValidationError): SearchOptions(neighbors=value)


@pytest.mark.parametrize('algorithm',['ASTAR','DIJKSTRA'])
def test_grid_algorithm_and_neighborhood_are_real(algorithm):
    result=native.grid_search([0,0,0],[3,3,3],[[0,0,0],[4,4,4]],[1,1,1],algorithm=algorithm,neighbors=6)
    assert result['status']=='ok'
    assert result['cost']==pytest.approx(9)
    assert all(np.count_nonzero(np.subtract(a,b))==1 for a,b in zip(result['path'],result['path'][1:]))
    diagonal=native.grid_search([0,0,0],[3,3,3],[[0,0,0],[4,4,4]],[1,1,1],algorithm=algorithm,neighbors=26)
    assert diagonal['cost']==pytest.approx(math.sqrt(27))


def test_explicit_search_bypasses_direct_and_reports_budget():
    req=request();req['task']['search']={'neighbors':6}
    route=plan_single_v2(**req)
    assert route['status']=='planned',route
    assert route['metrics']['distance_m']>125
    assert route['search_report']['neighbors']==6
    assert route['search_report']['expanded_nodes']>1
    req['task']['search']['max_expanded_nodes']=1
    route=plan_single_v2(**req)
    assert route['status']=='failed' and route['code']=='SEARCH_BUDGET_EXCEEDED'
    assert route['search_report']['expanded_nodes']==1
    req['task'].pop('search')
    assert plan_single_v2(**req)['metrics']['distance_m']<110


def test_layer_mismatch_rejected():
    req=request();req['task']['search']={'grid_level':4}
    with pytest.raises(PlanningError,match='grid_level'):plan_single_v2(**req)


@pytest.mark.parametrize('field',['maxClimbAngle','maxDescentAngle'])
@pytest.mark.parametrize('value',[0,-1,91,True,'30'])
def test_bad_angles_rejected(field,value):
    with pytest.raises(PlanningError):check(base_scene('CLIMB',{field:value}))


def test_slope_checks_include_vertical_and_do_not_confuse_rates():
    scene=base_scene('CLIMB',{'maxClimbAngle':30,'maxDescentAngle':20})
    assert rule_fails(check(scene,[[0,0,20],[1,0,21]],[0,100]),'CLIMB')
    assert rule_fails(check(scene,[[0,0,20],[0,0,21]],[0,100]),'CLIMB')
    assert not rule_fails(check(scene,[[0,0,20],[10,0,21]],[0,100]),'CLIMB')


@pytest.mark.parametrize("explicit_search",[False,True])
def test_angle_changes_route_and_independent_validation(explicit_search):
    req=request();req['task']['goal_wkt']=geo(Point(20,0,40))
    if explicit_search:
        req['task']['search']={"neighbors":26}
    baseline=plan_single_v2(**req);assert baseline['status']=='planned'
    req['scene']['rules'].append(dict(id='angle',rule_type='CLIMB',content={'maxClimbAngle':36}))
    route=plan_single_v2(**req)
    assert route['status']=='planned',route
    assert route['metrics']['distance_m']>baseline['metrics']['distance_m']+5
    result=validate_route_v2(**req,trajectory_wkt=baseline['trajectory_wkt'],vertex_times_s=baseline['vertex_times_s'],motion_segments=baseline['motion_segments'])
    assert not result['valid'] and any(v['code']=='CLIMB' for v in result['violations'])


def test_visibility_continuous_and_missing_data_fail_closed():
    scene=base_scene('WEATHER',{'minVisibility':500})
    with pytest.raises(PlanningError):check(scene)
    scene['datasets']=[dataset('weather',{'visibility_m':600})]
    assert not rule_fails(check(scene),'WEATHER')
    scene['datasets'].append(dataset('weather',{'visibility_m':200},dict(x_min=49,x_max=51,y_min=-5,y_max=5,z_min=0,z_max=200),id='fog'))
    assert rule_fails(check(scene),'WEATHER')
    scene['datasets'][0]['cells'][0]['values']={}
    with pytest.raises(PlanningError) as exc:check(scene)
    assert exc.value.detail['field']=='visibility_m'


def test_line_corner_vertical_hover_and_external_grid_code():
    model=Scene.model_validate(base_scene())
    motion=build_motion([[0,0,100],[100,100,100]],LIMITS)
    model.resolution=(25,25,25)
    cells,context=traversed_grids(model,motion)
    assert [c['grid_indices'][:2] for c in cells]==[[20,20],[21,21],[22,22],[23,23]]
    assert cells[0]['entry_time_s']==0 and cells[-1]['exit_time_s']==pytest.approx(motion.times[-1])
    assert all(a['exit_time_s']==pytest.approx(b['entry_time_s']) for a,b in zip(cells,cells[1:]))
    # Descending half-open ownership follows interval interior at the face.
    motion=build_motion([[0,0,100],[0,0,50]],LIMITS)
    cells,_=traversed_grids(model,motion)
    assert [c['grid_indices'][2] for c in cells]==[7,6]
    from uav_planner.v2.models import GridSpec
    model.grid=GridSpec(origin=(0,0,0),resolution=(50,50,50),shape=(10,10,10),codes={'0,0,2':'regular'},cells=[dict(bounds=dict(x_min=10,x_max=20,y_min=0,y_max=50,z_min=100,z_max=150),code='explicit',occupied=False)])
    motion=build_motion([[0,0,100],[40,0,100]],LIMITS)
    cells,context=traversed_grids(model,motion)
    assert [c['grid_code'] for c in cells]==['regular','explicit','regular']
    assert all(c['grid_indices']==[0,0,2] for c in cells)
    p=Primitive('hover',np.array([10.,10.,110.]),np.array([10.,10.,110.]),0,0,duration=20)
    cells,_=traversed_grids(model,Motion(np.array([p.start,p.end]),np.array([3,23]),[p],[3]))
    assert cells[0]['entry_time_s']==3 and cells[0]['exit_time_s']==23


def test_arc_grid_traversal_uses_analytic_crossing_times():
    model=Scene.model_validate(base_scene());model.resolution=(10,10,10)
    limits={**LIMITS,'min_turn_radius_m':20}
    motion=build_motion([[0,0,100],[100,0,100],[100,100,100]],limits)
    cells,_=traversed_grids(model,motion)
    arc=next(p for p in motion.primitives if p.kind=='arc')
    start=next(t for p,t in zip(motion.primitives,motion.starts) if p is arc)
    crossing=start+arc.time_at(arc.length*(math.pi/6)/(math.pi/2))
    assert any(abs(c['entry_time_s']-crossing)<1e-7 or abs(c['exit_time_s']-crossing)<1e-7 for c in cells)


def test_dijkstra_really_disables_heuristic_and_18_has_no_corner_cutting():
    kwargs=dict(start=[0,0,0],goal=[4,4,0],bounds=[[0,0,0],[8,8,3]],resolution=[1,1,1])
    a=native.grid_search(**kwargs,algorithm='ASTAR')
    d=native.grid_search(**kwargs,algorithm='DIJKSTRA')
    assert d['expanded']>a['expanded'] and d['cost']==pytest.approx(a['cost'])
    for neighbors in [6,18,26]:
        result=native.grid_search([0,0,0],[1,1,0],[[0,0,0],[1,1,0]],[1,1,1],blocked=[[1,0,0],[0,1,0]],neighbors=neighbors)
        assert result['status']=='no_path'


def test_analytic_slope_certificate_uses_arc_length():
    # A helix quarter-turn has less slope than its endpoint chord.
    # Use actual acceleration-limited arc from build_motion for valid timing.
    motion=build_motion([[0,0,20],[100,0,40],[100,100,60]],{**LIMITS,'min_turn_radius_m':20})
    arc=next(p for p in motion.primitives if p.kind=='arc')
    angle=math.degrees(math.atan2(abs(arc.end[2]-arc.start[2]),arc.radius*abs(arc.sweep)))
    proof=native.validate_motion([arc],{**LIMITS,'max_climb_angle_deg':angle+1e-6})
    assert not any(v['code']=='CLIMB_ANGLE' for v in proof['violations'])
    proof=native.validate_motion([arc],{**LIMITS,'max_climb_angle_deg':angle-1})
    assert any(v['code']=='CLIMB_ANGLE' for v in proof['violations'])


def test_stationary_point_export_has_grid_and_zero_dwell():
    model=Scene.model_validate(base_scene())
    motion=build_motion([[0,0,100]],LIMITS,start_time=5)
    cells,_=traversed_grids(model,motion)
    assert len(cells)==1 and cells[0]['entry_time_s']==cells[0]['exit_time_s']==5


@pytest.mark.parametrize('regular_code',[None,'regular'])
def test_legacy_keypoint_codes_keep_regular_priority_and_closed_cell_faces(regular_code):
    from uav_planner.v2.engine import _result
    from uav_planner.v2.models import Task, AircraftProfile
    from uav_planner.v2.scene import CompiledScene
    req=request()
    req['scene']['grid']=dict(origin=[0,0,0],resolution=[50,50,50],shape=[10,10,10],
        codes={'0,0,0':regular_code} if regular_code else {},
        cells=[dict(bounds=dict(x_min=10,x_max=20,y_min=0,y_max=50,z_min=0,z_max=50),code='explicit',occupied=False)])
    req['task'].update(start_wkt=geo(Point(20,10,20)),goal_wkt=geo(Point(40,10,20)))
    compiled=CompiledScene(req['scene'])
    motion=build_motion([[20,10,20],[40,10,20]],LIMITS)
    route=_result(compiled,compiled.model,Task.model_validate(req['task']),AircraftProfile(),motion)
    start=next(k for k in route['keypoints'] if 'START' in k['types'])
    assert start['grid_code']==(regular_code or 'explicit')
    assert route['traversed_grids'][0]['grid_code']==regular_code


def _validate_existing_route(req,route):
    return validate_route_v2(**req,trajectory_wkt=route['trajectory_wkt'],vertex_times_s=route['vertex_times_s'],motion_segments=route['motion_segments'])


@pytest.mark.parametrize("explicit_search",[False,True])
def test_future_angle_rule_does_not_reject_finished_flight_or_analytic_proof(explicit_search):
    from datetime import timedelta
    from tests.planning.test_rules import ORIGIN
    req=request();req['task']['goal_wkt']=geo(Point(20,0,40))
    if explicit_search:
        req['task']['search']={'neighbors':26}
    baseline=plan_single_v2(**req)
    assert baseline['status']=='planned' and baseline['vertex_times_s'][-1]<500
    req['scene']['rules'].append(dict(id='future',rule_type='CLIMB',content={'maxClimbAngle':1},
        effective_start=ORIGIN+timedelta(seconds=500),effective_end=ORIGIN+timedelta(seconds=600)))
    route=plan_single_v2(**req)
    assert route['status']=='planned',route
    assert route['metrics']==baseline['metrics']
    check=_validate_existing_route(req,baseline)
    assert check['valid'],check
    assert next(r for r in check['rule_report'] if r['rule_id']=='future')['status']=='not_applicable'


def test_angle_rule_partial_time_overlap_checks_only_active_motion():
    from datetime import timedelta
    from tests.planning.test_rules import ORIGIN
    req=request()
    req['task']['goal_wkt']=geo(Point(120,0,40))
    req['task']['waypoints']=[dict(wkt=geo(Point(100,0,20)))]
    baseline=plan_single_v2(**req)
    assert baseline['status']=='planned',baseline
    # Rule is active on the horizontal launch, and expires before the climb.
    req['scene']['rules'].append(dict(id='early',rule_type='CLIMB',content={'maxClimbAngle':1},
        effective_start=ORIGIN,effective_end=ORIGIN+timedelta(seconds=2)))
    check=_validate_existing_route(req,baseline)
    assert check['valid'],check
    # A window lying inside the later climb must reject the same analytic law.
    climb=next(p for p in baseline['motion_segments'] if p['end'][2]-p['start'][2]>1)
    middle=(climb['start_time_s']+climb['end_time_s'])/2
    req['scene']['rules'][-1].update(effective_start=ORIGIN+timedelta(seconds=middle-.1),effective_end=ORIGIN+timedelta(seconds=middle+.1))
    check=_validate_existing_route(req,baseline)
    assert not check['valid'] and any(v['code']=='CLIMB' for v in check['violations'])
    assert not any(v['code']=='MOTION_MODEL' for v in check['violations'])


@pytest.mark.parametrize('explicit_search',[False,True])
def test_resource_angle_outside_route_does_not_prune_search(explicit_search):
    from shapely.geometry import box
    from tests.planning.test_rules import zone
    req=request();req['task']['goal_wkt']=geo(Point(20,0,40))
    if explicit_search:
        req['task']['search']={'neighbors':26}
    baseline=plan_single_v2(**req)
    assert baseline['status']=='planned'
    req['scene']['control_zones']=[zone(id='z',geom=geo(box(110,40,120,50)),minAlt=0,maxAlt=100)]
    req['scene']['rules'].append(dict(id='outside',rule_type='CLIMB',content={'maxClimbAngle':1},resource_ids=['z']))
    route=plan_single_v2(**req)
    assert route['status']=='planned',route
    assert route['metrics']==baseline['metrics']
    assert _validate_existing_route(req,baseline)['valid']


@pytest.mark.parametrize('explicit_search',[False,True])
def test_resource_angle_inside_route_is_enforced_on_actual_motion(explicit_search):
    from uav_planner.v2.engine import _angle_ok
    from uav_planner.v2.models import Task
    from uav_planner.v2.scene import CompiledScene
    req=request();req['task']['goal_wkt']=geo(Point(20,0,40))
    if explicit_search:
        req['task']['search']={'neighbors':26}
    baseline=plan_single_v2(**req)
    assert baseline['status']=='planned'
    # An area dataset supplies applicability geometry without adding an obstacle.
    req['scene']['datasets']=[dataset('area',{'code':'slope'},dict(x_min=-30,x_max=150,y_min=-60,y_max=60,z_min=0,z_max=100),id='region')]
    req['scene']['rules'].append(dict(id='inside',rule_type='CLIMB',content={'maxClimbAngle':1},resource_ids=['region']))
    compiled=CompiledScene(req['scene'])
    assert not _angle_ok(compiled,np.array([0,0,20]),np.array([20,0,40]),0,10,Task.model_validate(req['task']))
    check=_validate_existing_route(req,baseline)
    assert not check['valid'] and any(v['code']=='CLIMB' and v['rule_id']=='inside' for v in check['violations'])
    route=plan_single_v2(**req)
    assert route['status']=='failed',route
    assert any(v['code']=='CLIMB' and v['rule_id']=='inside' for v in route['validation']['violations'])
