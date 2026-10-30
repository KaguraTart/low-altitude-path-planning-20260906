"""Caller-driven incremental ENG-07 replanning with independently certified reuse."""
from __future__ import annotations

import copy
import math
import time
from datetime import timedelta
from typing import Literal

import numpy as np
from pydantic import Field, field_validator, model_validator

from uav_planner.errors import ErrorCode, PlanningError
from .models import (StrictModel, Number, NonNegative, Positive, Timestamp, GeographicWKT,
                     Scene, Task, RouteResult, Violation, validated_wkt)
from .motion import Motion, Primitive, primitive_tangent as tangent
from . import engine as e, native


class CurrentState(StrictModel):
    position_wkt: GeographicWKT
    velocity_xyz_mps: tuple[Number, Number, Number]
    at: Timestamp

    @field_validator('position_wkt')
    @classmethod
    def point(cls, value):
        return validated_wkt(value, kind='Point', require_z=True)


class ReplanEvent(StrictModel):
    id: str = Field(min_length=1)
    type: Literal['obstacle', 'dynamic_obstacle', 'rule', 'dataset', 'aircraft_state', 'mission', 'scene', 'weather', 'control_zone', 'airspace']
    at: Timestamp
    old_value: Number | None = None
    new_value: Number | None = None
    changed_resource_ids: list[str] = Field(default_factory=list)

    @model_validator(mode='after')
    def quantities(self):
        if (self.old_value is None) != (self.new_value is None):
            raise ValueError('old_value and new_value must be supplied together')
        if self.old_value is None and not self.changed_resource_ids:
            raise ValueError('event requires numeric old/new values or changed_resource_ids')
        return self


class ReplanTrigger(StrictModel):
    threshold: NonNegative | None = None
    min_interval_s: NonNegative | None = None
    max_frequency_hz: Positive | None = None
    last_event_time: Timestamp | None = None
    last_replan_time: Timestamp | None = None


class ReplanRequest(StrictModel):
    current_state: CurrentState
    previous_route: RouteResult
    previous_task: Task
    previous_scene: Scene
    remaining_task: Task
    scene: Scene | None = None
    scene_id: str | None = None
    event: ReplanEvent
    trigger: ReplanTrigger = Field(default_factory=ReplanTrigger)
    time_budget_s: Positive | None = None

    @model_validator(mode='after')
    def selection(self):
        if (self.scene is None) == (self.scene_id is None):
            raise ValueError('Provide exactly one of scene_id or scene')
        if any(t.engine_id != 'ENG-07' or t.route_mode != 'free' for t in (self.previous_task,self.remaining_task)):
            raise ValueError('Replanning supports ENG-07/free only')
        return self


class ReplanResponse(StrictModel):
    decision: Literal['reused', 'replanned', 'ignored', 'failed']
    reason: str
    next_eligible_time: Timestamp | None = None
    current_state: CurrentState
    route: RouteResult | None = None
    reused_primitives: int = 0
    reused_distance_m: NonNegative = 0
    reused_time_s: NonNegative = 0
    new_search_calls: int = 0
    recomputed_from_s: NonNegative | None = None
    violations: list[Violation] = Field(default_factory=list)


def _speed(p,t):
    if t <= p.ta:
        return p.v0+p.accel*t
    if t <= p.ta+p.tc:
        return p.peak
    return max(0,p.peak-p.decel*(t-p.ta-p.tc))


def _clip(motion, begin, end):
    """Split at analytic phase boundaries so every retained law is exact."""
    primitives=[]; starts=[]
    for p,start in zip(motion.primitives,motion.starts):
        cuts=sorted(set([0.,p.ta,p.ta+p.tc,p.duration]))
        for a,b in zip(cuts,cuts[1:]):
            lo=max(a,begin-start); hi=min(b,end-start)
            if hi-lo <= 1e-8:
                continue
            q=copy.copy(p)
            d0,d1=p.distance_at(lo),p.distance_at(hi)
            q.start=p.position(d0);q.end=p.position(d1);q.length=d1-d0
            q.sweep=p.sweep*q.length/p.length if p.length else p.sweep*(hi-lo)/max(p.duration,1e-30)
            q.duration=hi-lo;q.v0=_speed(p,lo);q.v1=_speed(p,hi)
            q.peak=max(q.v0,q.v1);q.ta=q.tc=q.td=0.
            if q.length <= 1e-12:
                q.v0=q.v1=q.peak=0.;q.tc=q.duration
            elif hi <= p.ta+1e-8:
                q.ta=q.duration
            elif lo >= p.ta+p.tc-1e-8:
                q.td=q.duration
            else:
                q.tc=q.duration
            primitives.append(q); starts.append(start+lo)
    return _motion(primitives,starts)


def _motion(primitives,starts):
    if not primitives:
        return None
    result=Motion(np.empty((0,3)),np.empty(0),primitives,starts,distance=sum(p.length for p in primitives))
    result.points,result.times=result.collision_samples(error=.01)
    result.events=[(0,['START']),(len(result.times)-1,['GOAL'])]
    return result


def _join(*motions):
    present=[m for m in motions if m is not None]
    primitives=[p for m in present for p in m.primitives]
    if present and not primitives:
        # A zero-distance remaining mission still has an authoritative position
        # and timestamp and must go through the ordinary independent checker.
        return e._join_motions(present)
    return _motion(primitives,[t for m in present for t in m.starts])


def _brake(position,velocity,at,limits):
    speed=float(np.linalg.norm(velocity))
    if speed <= 1e-8:
        return None
    decel=limits['deceleration_mps2']; duration=speed/decel
    end=position+velocity/speed*(speed*duration/2)
    delta=end-position
    kind='climb' if delta[2]>1e-8 else 'descent' if delta[2]<-1e-8 else 'line'
    p=Primitive(kind,position,end,float(np.linalg.norm(delta)),speed,duration=duration,
                v0=speed,v1=0,peak=speed,accel=limits['acceleration_mps2'],decel=decel,td=duration)
    return _motion([p],[at])


def _assemble(candidate,prefix,position,now,limits):
    handoff=float(prefix.times[-1]) if prefix else now
    point=prefix.points[-1] if prefix else position
    tail=candidate['motion'].shifted(max(0,candidate['lo']-candidate['motion'].times[0]))
    if prefix is not None:
        moving=[p for p in prefix.primitives if p.length>1e-9 and np.linalg.norm(tangent(p,True)[:2])>1e-9]
        following=next((i for i,p in enumerate(tail.primitives)
                        if p.length>1e-9 and np.linalg.norm(tangent(p)[:2])>1e-9),None)
        if moving and following is not None:
            a=tangent(moving[-1],True); b=tangent(tail.primitives[following])
            angle=math.atan2(a[0]*b[1]-a[1]*b[0],float(np.dot(a[:2],b[:2])))
            if abs(angle)>1e-5:
                yaw_at=tail.starts[following]; yaw_point=tail.primitives[following].start
                yaw=Primitive('yaw',yaw_point,yaw_point,0,0,sweep=angle,
                    duration=abs(angle)/math.radians(limits['max_yaw_rate_deg_s']))
                tail_before=_clip(tail,float(tail.times[0]),yaw_at)
                tail_after=_clip(tail,yaw_at,float(tail.times[-1]))
                tail=_join(tail_before,_motion([yaw],[yaw_at]),tail_after.shifted(yaw.duration))
    if tail.times[0]>handoff+1e-7:
        # Waiting at the handoff is a physical hover, never a gap in the law.
        wait=Primitive('hover',point,point,0,0,duration=float(tail.times[0]-handoff))
        prefix=_join(prefix,_motion([wait],[handoff]))
    complete=_join(prefix,tail)
    return complete


def _attach_search_report(route,task,model,expanded):
    if task.search is not None:
        route['search_report']={'algorithm':task.search.algorithm,'neighbors':task.search.neighbors,
            'grid_level':model.grid.level if model.grid else None,'expanded_nodes':expanded,
            'max_expanded_nodes':task.search.max_expanded_nodes}


def _bad(message):
    raise PlanningError(ErrorCode.INVALID_INPUT,message)


def _validate(motion,task,model,compiled,scene_id,velocity=None,*,partial=False):
    if motion is None:
        return []
    proof_task=task
    if partial:
        goal=e._timed_geographic(compiled,[motion.points[-1]],[motion.times[-1]],task.alt_reference,model)
        proof_task=task.model_copy(update={'goal_wkt':goal,'goal_grid':None,'waypoints':[],
            'earliest_arrival':None,'latest_arrival':None})
    route=e._result(compiled,model,proof_task,e._profile(model,proof_task),motion)
    proof=e.validate_route_v2(task=proof_task,scene_id=scene_id,trajectory_wkt=route['trajectory_wkt'],
        vertex_times_s=route['vertex_times_s'],motion_segments=route['motion_segments'],
        initial_velocity_xyz_mps=velocity,_partial_goal_wkt=proof_task.goal_wkt if partial else None)
    errors=proof['violations']
    profile=e._profile(model,task)
    remaining_time=task.state.remaining_flight_time_s if task.state else None
    remaining_range=task.state.remaining_range_m if task.state else None
    if motion.distance > min(profile.max_range_m,remaining_range if remaining_range is not None else math.inf)+1e-7 or motion.times[-1]-motion.times[0] > min(profile.endurance_s,remaining_time if remaining_time is not None else math.inf)+1e-7:
        errors.append({'code':'AIRCRAFT_ENDURANCE','message':'Analytic remaining mission exceeds exact distance or duration budget'})
    if motion.times[-1] > model.horizon_s+1e-7:
        errors.append({'code':'SCENE_HORIZON','message':'Analytic motion exceeds scene horizon'})
    return errors


def replan_v2(**kwargs):
    """Recheck clipped old motion; search only the remainder after a safe stop."""
    start=time.perf_counter()
    request=e._parse(ReplanRequest,kwargs)
    deadline=e._planning_deadline(start,request.time_budget_s)
    def budget():
        if time.perf_counter() >= deadline:
            raise PlanningError(ErrorCode.TIMEOUT,'Replanning time budget exhausted')
    if request.scene is not None:
        scene_id=e.register_scene(request.scene)['scene_id']
    else:
        scene_id=request.scene_id
    model,compiled,_,_=e.registry.get(scene_id)
    old_id=e.register_scene(request.previous_scene)['scene_id']
    old_model,old_compiled,_,_=e.registry.get(old_id)
    old=request.previous_route; previous=request.previous_task; task=request.remaining_task
    from .network import normalize_task
    previous=normalize_task(old_compiled,previous)
    previous,_=e._rtl_task(old_compiled,previous,e._profile(old_model,previous))
    state=request.current_state; event=request.event; trigger=request.trigger
    if model.time_origin != old_model.time_origin or compiled.frame.context != old_compiled.frame.context:
        _bad('Previous and current scenes require the same canonical projection, vertical context and time_origin')
    if old.coordinate_context != old_compiled.frame.context or old.time_origin != old_model.time_origin:
        _bad('Previous route coordinate/time context does not match previous_scene')
    if old.status != 'planned' or old.trajectory_wkt is None:
        _bad('previous_route requires a complete planned result')
    if old.task_id != previous.id or old.departure_time != previous.departure_time or old.alt_reference != previous.alt_reference or old.engine_id != previous.engine_id or old.route_mode != previous.route_mode:
        _bad('Previous route identity/time/height reference does not match previous_task')
    if np.linalg.norm(e._local(old_compiled,old.start_wkt,previous)-e._local(old_compiled,previous.start_wkt,previous))>.01 or np.linalg.norm(e._local(old_compiled,old.goal_wkt,previous)-e._local(old_compiled,previous.goal_wkt,previous))>.01:
        _bad('Previous route endpoints do not match previous_task')
    original=e.validate_route_v2(task=previous,scene_id=old_id,trajectory_wkt=old.trajectory_wkt,
        vertex_times_s=old.vertex_times_s,motion_segments=old.motion_segments,
        initial_velocity_xyz_mps=old.initial_velocity_xyz_mps)
    if not original['valid']:
        _bad('Previous route failed independent validation: '+str(original['violations']))
    budget()
    now=e._seconds(state.at,model)
    if not 0 <= now <= model.horizon_s:
        _bad('Current state time is outside scene horizon')
    if event.at > state.at:
        _bad('event.at must not follow current_state.at')
    if task.state is not None and task.state.at is not None and task.state.at != state.at:
        _bad('remaining_task.state.at must equal current_state.at')
    if task.departure_time != state.at:
        _bad('remaining_task.departure_time must equal current_state.at')
    from .network import normalize_task
    task=normalize_task(compiled,task)
    position=e._local(compiled,state.position_wkt,task)
    if np.linalg.norm(position-e._local(compiled,task.start_wkt,task))>.01:
        _bad('remaining_task.start_wkt must match current_state.position_wkt')
    old_passages=e._waypoint_passages(e._reconstruct_motion(old.motion_segments,
        e._timed_local(old_compiled,old.trajectory_wkt,np.asarray(old.vertex_times_s),previous.alt_reference,old_model),
        np.asarray(old.vertex_times_s)),previous,old_compiled) or []
    for waypoint in task.waypoints:
        local=e._local(compiled,waypoint.wkt,task)
        for old_waypoint,passed in zip(previous.waypoints,old_passages):
            if np.linalg.norm(local-e._local(old_compiled,old_waypoint.wkt,previous))<.01 and passed < now-1e-7:
                _bad('remaining_task contains a waypoint already passed before current_state.at')
    if any(w.latest is not None and w.latest < state.at for w in task.waypoints):
        _bad('remaining_task waypoints must be future only')
    # Resolve references even when the trigger will be ignored.
    profile=e._profile(model,task); limits=e._limits(compiled,task,profile)
    task,rtl=e._rtl_task(compiled,task,profile)
    if task.objectives is not None:
        from .objectives import datasets
        datasets(compiled,task.objectives)
    velocity=np.asarray(state.velocity_xyz_mps,dtype=float)
    if np.linalg.norm(velocity) > limits['max_total_speed_mps']+1e-7 or velocity[2]>limits['climb_speed_mps']+1e-7 or -velocity[2]>limits['descent_speed_mps']+1e-7:
        _bad('Current velocity exceeds aircraft or rule envelope')
    resources={v.id for m in (model,old_model) for name in ('obstacles','dynamic_obstacles','rules','datasets','airways','control_zones','connectors','profiles') for v in getattr(m,name,[])}
    if any(ref not in resources for ref in event.changed_resource_ids):
        _bad('event.changed_resource_ids contains an unknown resource')
    if trigger.threshold is not None and event.old_value is None:
        _bad('trigger.threshold requires numeric event old_value/new_value')
    for value in (trigger.last_event_time,trigger.last_replan_time):
        if value is not None and value > state.at:
            _bad('Trigger history time must not follow current_state.at')
    answer=dict(current_state=state,decision='ignored',reason='',new_search_calls=0)
    if trigger.last_event_time is not None and event.at <= trigger.last_event_time:
        answer['reason']='stale_event'
    elif trigger.threshold is not None and abs(event.new_value-event.old_value)<trigger.threshold:
        answer['reason']='below_threshold'
    elif trigger.last_replan_time is not None:
        try:
            interval=max(trigger.min_interval_s or 0,1/trigger.max_frequency_hz if trigger.max_frequency_hz else 0)
            eligible=trigger.last_replan_time+timedelta(seconds=interval)
        except (OverflowError,ValueError) as exc:
            raise PlanningError(ErrorCode.INVALID_INPUT,
                'trigger rate interval or next eligible timestamp is not representable',
                {'min_interval_s':trigger.min_interval_s,'max_frequency_hz':trigger.max_frequency_hz,
                 'last_replan_time':trigger.last_replan_time.isoformat()}) from exc
        if state.at < eligible:
            answer.update(reason='frequency_limited',next_eligible_time=eligible)
    if answer['reason']:
        budget()
        return ReplanResponse(**answer).model_dump(mode='json')
    old_points=e._timed_local(old_compiled,old.trajectory_wkt,np.asarray(old.vertex_times_s),previous.alt_reference,old_model)
    motion=e._reconstruct_motion(old.motion_segments,old_points,np.asarray(old.vertex_times_s))
    suffix=_clip(motion,now,float(motion.times[-1]))
    p=suffix.primitives[0] if suffix is not None else None
    matches=p is not None and abs(suffix.times[0]-now)<=1e-7 and np.linalg.norm(p.start-position)<=1e-5 and np.linalg.norm(tangent(p)*p.v0-velocity)<=1e-5
    # Old future motion may serve a changed mission only after checking the entire
    # remaining task, including ordered waypoint/window/budget constraints.
    if matches and task.objectives is None:
        errors=_validate(suffix,task,model,compiled,scene_id,velocity)
        budget()
        if not errors:
            route=e._result(compiled,model,task,profile,suffix)
            route['initial_velocity_xyz_mps']=velocity.tolist()
            _attach_search_report(route,task,model,0)
            budget()
            return ReplanResponse(**{**answer,'decision':'reused','reason':'unaffected_suffix',
                'route':route,'reused_primitives':len(suffix.primitives),'reused_distance_m':suffix.distance,
                'reused_time_s':float(suffix.times[-1]-now)}).model_dump(mode='json')
    # Search possible safe handoffs along the old law. Each retained prefix plus
    # tangent braking is independently checked under the latest scene. Stop at
    # the first unsafe candidate; no old unsafe motion can leak into the result.
    retained=None; braking=_brake(position,velocity,now,limits)
    initial_errors=_validate(braking,task,model,compiled,scene_id,velocity,partial=True)
    budget()
    if initial_errors:
        return ReplanResponse(**{**answer,'decision':'failed','reason':'unsafe_braking','violations':initial_errors}).model_dump(mode='json')
    if matches and task.objectives is None:
        end=float(suffix.times[-1])
        candidates=sorted(set([now+(end-now)*i/16 for i in range(1,16)]+[s for s in suffix.starts if s>now+1e-7]))
        for at in candidates:
            budget()
            prefix=_clip(suffix,now,at)
            q=prefix.primitives[-1]; at_velocity=tangent(q,True)*q.v1
            stop=_brake(q.end,at_velocity,at,limits)
            joined=_join(prefix,stop)
            if _validate(joined,task,model,compiled,scene_id,velocity,partial=True):
                break
            # Never fly past a still-required waypoint then lose its obligations
            # from a separately planned tail. Whole-route validation remains final.
            retained,braking=prefix,stop
    prefix=_join(retained,braking)
    handoff=float(prefix.times[-1]) if prefix else now
    point=prefix.points[-1] if prefix else position
    point_wkt=e._timed_geographic(compiled,[point],[handoff],task.alt_reference,model)
    tail_task=task.model_copy(deep=True,update={'start_wkt':point_wkt,'start_grid':None,
        'departure_time':model.time_origin+timedelta(seconds=handoff)})
    # Waypoints already fully met by the retained prefix are removed only from
    # the tail search; final certification uses the original remaining task.
    if prefix:
        passed=[]
        for waypoint in task.waypoints:
            probe=task.model_copy(update={'waypoints':[waypoint]})
            if e._waypoint_errors(prefix,probe,compiled):
                break
            passed.append(waypoint)
        tail_task=tail_task.model_copy(update={'waypoints':task.waypoints[len(passed):]})
    search_state={'remaining':task.search.max_expanded_nodes if task.search else None,'expanded':0,'calls':0}
    if task.objectives is not None:
        from .objectives import prepare
        candidate,failure=prepare(0,tail_task,model,compiled,e._dynamic_tracks(compiled,model),
                                  deadline,search_state,retain_all_candidates=True)
    else:
        candidate,failure=e._prepare_candidate(0,tail_task,model,compiled,e._dynamic_tracks(compiled,model),deadline,search_state)
    answer['new_search_calls']=search_state['calls']
    budget()
    if failure is not None:
        return ReplanResponse(**{**answer,'decision':'failed','reason':failure.get('code','recompute_failed'),
            'violations':[{'code':failure.get('code','REPLAN'),'message':failure.get('message','Recompute failed')}]}).model_dump(mode='json')
    choices=candidate.get('objective_candidates',[candidate]) if task.objectives is not None else [candidate]
    completed=[]; last_violations=[]
    from .objectives import evaluate,frontier,vector,ObjectiveCoverageError
    for choice in choices:
        budget()
        complete=_assemble(choice,prefix,position,now,limits)
        violations=_validate(complete,task,model,compiled,scene_id,velocity)
        budget()
        if violations:
            last_violations=violations
            continue
        route=e._result(compiled,model,task,profile,complete)
        route['initial_velocity_xyz_mps']=velocity.tolist()
        _attach_search_report(route,task,model,search_state['expanded'])
        if task.objectives is not None:
            try:
                route['objectives']=evaluate(compiled,task.objectives,complete,deadline)
            except TimeoutError as exc:
                raise PlanningError(ErrorCode.TIMEOUT,'Replanning objective scoring exhausted time budget') from exc
            except ObjectiveCoverageError as exc:
                last_violations=[{'code':'OBJECTIVE_COVERAGE','message':str(exc)}]
                continue
        completed.append({'route':route,'objective':route.get('objectives')})
    budget()
    if not completed:
        return ReplanResponse(**{**answer,'decision':'failed','reason':'final_validation_failed',
            'violations':last_violations}).model_dump(mode='json')
    if task.objectives is not None:
        validated=len(completed)
        completed=sorted(frontier(completed),key=lambda item:(item['objective']['weighted_score'],vector(item['objective'])))
        report=copy.deepcopy(candidate['optimization'])
        report.update(validated_candidates=validated,returned_solutions=len(completed))
        for item in completed:
            item['route']['optimization']=copy.deepcopy(report)
    route=completed[0]['route']
    route['alternatives']=[item['route'] for item in completed[1:]]
    return ReplanResponse(**{**answer,'decision':'replanned','reason':'objective_reoptimization' if task.objectives is not None else 'affected_suffix' if matches else 'current_state_mismatch',
        'route':route,'recomputed_from_s':handoff,'reused_primitives':len(retained.primitives) if retained else 0,
        'reused_distance_m':retained.distance if retained else 0,'reused_time_s':float(retained.times[-1]-now) if retained else 0}).model_dump(mode='json')
