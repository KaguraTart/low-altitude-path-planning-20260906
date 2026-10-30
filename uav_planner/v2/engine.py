"""Versioned scenes, geographic planning and independent output validation."""
from __future__ import annotations

import hashlib
import json
import math
import sys
import threading
import time
import uuid
from collections import OrderedDict
from datetime import datetime, timedelta, timezone

import numpy as np
from pydantic import ValidationError
from shapely.geometry import LineString, Point
from shapely.ops import nearest_points

from uav_planner.errors import ErrorCode, PlanningError
from .models import AircraftProfile, BatchResult, ExecutionOptions, MotionSegment, Scene, Task
from .motion import Motion, MotionError, Primitive, build_motion


class SceneRegistry:
    """Immutable compiled scenes retained until restart; no task answers are cached."""

    def __init__(self) -> None:
        self._scenes: OrderedDict[str, tuple] = OrderedDict()
        self._lock = threading.RLock()
        self._compile_lock = threading.Lock()
        self.max_scenes = None
        self.max_bytes = None

    def register(self, value: Scene | dict) -> dict:
        from .scene import CompiledScene
        model = _parse(Scene, value)
        canonical = model.model_dump_json(exclude_none=True)
        version = hashlib.sha256(canonical.encode()).hexdigest()
        key = "scene-" + version[:20]
        with self._lock:
            if key in self._scenes:
                self._scenes.move_to_end(key)
                return self.describe(key)
        with self._compile_lock:
            with self._lock:
                if key in self._scenes:
                    return self.describe(key)
            start = time.perf_counter()
            try:
                compiled = CompiledScene(model)
                from .safety import temporal_breakpoints
                temporal_breakpoints(compiled, model)
            except PlanningError:
                raise
            except ValueError as exc:
                raise PlanningError(ErrorCode.INVALID_INPUT, str(exc)) from exc
            record = (model, compiled, version, (time.perf_counter()-start)*1000)
            with self._lock:
                self._scenes[key] = record
        return self.describe(key)

    def get(self, key: str):
        with self._lock:
            if key not in self._scenes:
                raise PlanningError(ErrorCode.TASK_NOT_FOUND, "Scene not found; register it after restart", {"scene_id": key})
            self._scenes.move_to_end(key)
            return self._scenes[key]

    def describe(self, key: str) -> dict:
        model, compiled, version, elapsed = self.get(key)
        return {"scene_id": key, "scene_version": version, "network_version": model.airway_graph.version if model.airway_graph else model.network_version, "scene_type": model.scene_type, "status": "ready", "compilation_ms": elapsed,
                "coordinate_context": compiled.frame.context, "warnings": compiled.warnings,
                "counts": {**{name:len(getattr(model,name)) for name in ("airways","obstacles","control_zones","rules","datasets")}, **({"network_nodes": len(model.airway_graph.nodes), "network_edges": len(model.airway_graph.edges)} if model.airway_graph else {})}}


registry = SceneRegistry()


class _SearchBudgetExceeded(MotionError):
    pass


class _DeadlineExceeded(TimeoutError):
    """A caller-supplied planning budget has expired."""


def _planning_deadline(start: float, time_budget_s: float | None) -> float:
    if time_budget_s is None:
        return math.inf
    try:
        valid = (not isinstance(time_budget_s, bool) and isinstance(time_budget_s, (int, float))
                 and math.isfinite(time_budget_s) and time_budget_s > 0)
    except OverflowError:
        valid = False
    if not valid:
        raise PlanningError(ErrorCode.INVALID_INPUT, "time_budget_s must be a positive finite number or None")
    return start + time_budget_s


def _native_deadline_ms(deadline: float) -> float:
    """Translate an optional wall-clock deadline without disabling expired budgets."""
    if math.isinf(deadline):
        return 0.0
    remaining = deadline - time.perf_counter()
    if remaining <= 0:
        raise _DeadlineExceeded("Planning deadline exhausted")
    remaining_ms = remaining * 1000
    return remaining_ms if math.isfinite(remaining_ms) else sys.float_info.max


def _parse(cls, value):
    try:
        return value if isinstance(value,cls) else cls.model_validate(value)
    except ValidationError as exc:
        raise PlanningError(ErrorCode.INVALID_INPUT, "v2 input validation failed: "+exc.errors()[0]["msg"], {"errors":json.loads(exc.json(include_url=False))}) from exc


def register_scene(scene: Scene | dict) -> dict:
    return registry.register(scene)


def _profile(model: Scene, task: Task) -> AircraftProfile:
    if task.aircraft is not None:
        return task.aircraft
    for item in model.profiles:
        if item.id == task.profile_id:
            return item
    if task.profile_id == "dji_mini_4_pro":
        return AircraftProfile()
    raise PlanningError(ErrorCode.AIRCRAFT_NOT_FOUND,"Unknown v2 aircraft profile",{"profile_id":task.profile_id})


def _limits(compiled, task, profile):
    result = {name:getattr(profile,name) for name in (
        "max_speed_mps","cruise_speed_mps","climb_speed_mps","descent_speed_mps",
        "max_yaw_rate_deg_s","acceleration_mps2","deceleration_mps2","min_turn_radius_m")}
    result["min_speed_mps"] = 0.0
    result.update(compiled.rules.limits(task,profile))
    # Departure may be delayed, so launch-active hints alone cannot certify a
    # flight. Use the strictest envelope that can become applicable to this task.
    begin = _seconds(task.departure_time, compiled.model)
    end = _seconds(task.latest_arrival, compiled.model) if task.latest_arrival else compiled.model.horizon_s
    for rule in compiled.rules.selected(task):
        if compiled.rules.interval(rule, begin, end) is None:
            continue
        c = rule.content
        if rule.rule_type == "SPEED":
            result["max_speed_mps"] = min(result["max_speed_mps"], c.get("maxSpeed", math.inf))
            result["cruise_speed_mps"] = min(result["cruise_speed_mps"], c.get("cruiseSpeed", math.inf))
            result["min_speed_mps"] = max(result["min_speed_mps"], c.get("minSpeed", 0))
        elif rule.rule_type == "CLIMB":
            result["climb_speed_mps"] = min(result["climb_speed_mps"], c.get("maxClimbRate", math.inf))
            result["descent_speed_mps"] = min(result["descent_speed_mps"], c.get("maxDescentRate", math.inf))
        elif rule.rule_type == "TURN":
            result["min_turn_radius_m"] = max(result["min_turn_radius_m"], c.get("minTurnRadius", 0))
            result["max_turn_angle_deg"] = min(result.get("max_turn_angle_deg", 180), c.get("maxTurnAngle", 180))
    result["cruise_speed_mps"] = min(result["cruise_speed_mps"], result["max_speed_mps"])
    # Public SPEED and aircraft checks use full XYZ speed, so the analytic
    # certificate must also bound a climbing segment's unsampled vector peak.
    result["max_total_speed_mps"] = result["max_speed_mps"]
    return result


def _local(compiled, wkt, task):
    return np.asarray(compiled.frame.to_local(wkt,task.alt_reference,task.departure_time).coords[0],dtype=float)


def _airway_motion_limits(compiled,task,limits,points):
    """Include airway kinematics in the analytic envelope, not chord averages."""
    result=dict(limits)
    xy=np.asarray(points)[:,:2]
    geometry=Point(xy[0]) if len(xy)==1 or np.max(np.linalg.norm(xy-xy[0],axis=1))<1e-9 else LineString(xy)
    departure=_seconds(task.departure_time,compiled.model)
    latest=_seconds(task.latest_arrival,compiled.model) if task.latest_arrival else compiled.model.horizon_s
    for airway in compiled.airways:
        if airway.status!="ACTIVE" or airway.valid_to_s<departure or airway.valid_from_s>latest or not airway.geometry.intersects(geometry):
            continue
        climb=airway.source.climbConstraint
        if climb is not None:
            result["climb_speed_mps"]=min(result["climb_speed_mps"],climb.maxClimbRate)
            result["descent_speed_mps"]=min(result["descent_speed_mps"],climb.maxDescentRate)
        turn=airway.source.turnConstraint
        if turn is not None:
            result["min_turn_radius_m"]=max(result["min_turn_radius_m"],turn.minTurnRadius)
            result["max_turn_angle_deg"]=min(result.get("max_turn_angle_deg",180),turn.maxTurnAngle)
    return result


def _seconds(value, model):
    return (value-model.time_origin).total_seconds()


def _geometry_ok(compiled, a,b,t,task,profile):
    # Geometric edges are screened before timing; full rules run on real timing.
    duration=max(1e-6,float(np.linalg.norm(b-a))/profile.cruise_speed_mps)
    if not _angle_ok(compiled, a, b, t, t+duration, task):
        return False
    return not compiled.check_geometry(a,b,t,t+duration,task,profile)


def _angle_limits(compiled, task, t0, t1, *, cover_interval=False, edge=None):
    climb, descent = 90.0, 90.0
    for rule in compiled.rules.selected(task):
        if rule.rule_type != "CLIMB":
            continue
        active = compiled.rules.interval(rule, t0, t1)
        if active is None:
            continue
        # A spatial search has no arrival times for its edges. A native global
        # slope bound is valid only for a scene-wide rule covering the entire
        # possible leg window. Resource/time scopes use actual edges and timed
        # primitives, since the native spatial search has no scope predicate.
        if cover_interval and (active[0] > t0 or active[1] < t1 or rule.resource_ids or rule.scope is not None):
            continue
        if edge is not None and not compiled.rule_parts(rule, np.asarray(edge[0]), np.asarray(edge[1]), t0, t1):
            continue
        climb = min(climb, rule.content.get("maxClimbAngle", 90))
        descent = min(descent, rule.content.get("maxDescentAngle", 90))
    return climb, descent


def _motion_slope_angles(motion, times):
    angles=[]
    for a,b in zip(times,times[1:]):
        middle=(a+b)/2
        i=max(0,min(len(motion.primitives)-1,int(np.searchsorted(motion.starts,middle,side="right")-1)))
        p=motion.primitives[i]
        horizontal=abs(p.sweep)*p.radius if p.kind=="arc" else np.linalg.norm(p.end[:2]-p.start[:2])
        angles.append(math.degrees(math.atan2(abs(p.end[2]-p.start[2]),horizontal)))
    return angles


def _angle_ok(compiled, a, b, t0, t1, task):
    delta = np.asarray(b)-a
    angle = math.degrees(math.atan2(abs(delta[2]), np.linalg.norm(delta[:2])))
    limits = _angle_limits(compiled, task, t0, t1, edge=(a,b))
    return angle <= limits[0 if delta[2]>=0 else 1]+1e-7


def _cost_floor(compiled,task,t):
    active=[r for r in compiled.rules.selected(task) if r.rule_type=="PRIORITY" and compiled.rules.active(r,t)]
    signature=tuple(r.id for r in active)
    cache=getattr(compiled,"_cost_floor_cache",None)
    if cache is None:
        cache=compiled._cost_floor_cache={}
    if signature in cache:
        return cache[signature]
    floor=1.0
    for rule in active:
        content=rule.content
        if content["priorityScene"] in {"FAST", "PLANNING"}:
            floor*=1-content["costBonus"]
        else:
            kind,key=("noise","noise_db") if content["priorityScene"]=="QUIET" else ("risk","risk")
            values=[float(c.values[key]) for d in compiled.datasets_for(rule,kind) for c in d.cells
                    if key in c.values and not c.values.get("nodata",False)]
            best=min(values,default=0)
            best=min(1,max(0,best/100 if kind=="noise" else best))
            floor*=1-content["costBonus"]*(1-best)
    cache[signature]=max(.001,floor)
    return cache[signature]


def _edge_cost(compiled,a,b,t,task,profile):
    # Soft cost integration is independent from the exact hard geometry checks.
    samples=[a+(b-a)*f for f in (.125,.375,.625,.875)]
    cost=sum(compiled.cost_multiplier(p,t,task,profile) for p in samples)/len(samples)
    return float(np.linalg.norm(b-a))*cost


def _roadmap(compiled, start,goal,task,profile,t,deadline,reference_altitude=None):
    """Exact-coordinate roadmap with airway anchors and obstacle bypass anchors."""
    from . import native
    nodes=[start,goal]
    for airway in compiled.airways:
        line=getattr(airway,"line",None)
        if line is None:
            line=LineString(compiled.airway_lines[airway.id])
        coords=np.asarray(line.coords).copy()
        if reference_altitude is not None:
            coords[:,2]=reference_altitude
        nodes.extend(coords)
        for point in (start,goal):
            # The horizontal foot is an anchor, not a replacement for the endpoint.
            flat=LineString(coords[:,:2])
            s=flat.project(Point(point[:2]))
            q=flat.interpolate(s)
            z=float(np.interp(s,np.r_[0,np.cumsum(np.linalg.norm(np.diff(coords[:,:2],axis=0),axis=1))],coords[:,2]))
            nodes.append(np.array([q.x,q.y,z]))
            nodes.append(np.array([q.x,q.y,point[2]]))
    if task.route_mode != "strict":
        # Visibility anchors around obstacles, with clearance applied before use.
        margin=profile.radius_m+profile.safety_margin_m+.2
        for volume in [*compiled.obstacles, *compiled.zones, *(v for r in compiled.rules.selected(task) if r.rule_type in {"BUILDING","NO_FLY","ZONE"} for v in compiled.rule_scopes.get(r.id, []))]:
            geom=getattr(volume,"geometry",getattr(volume,"geom",None))
            if geom is None:
                continue
            xmin,ymin,xmax,ymax=geom.bounds
            for x,y in [(xmin-margin,ymin-margin),(xmin-margin,ymax+margin),(xmax+margin,ymin-margin),(xmax+margin,ymax+margin)]:
                nodes.append(np.array([x,y,start[2]]))
        for rule in compiled.rules.selected(task):
            if rule.rule_type == "PRIORITY" and compiled.rules.active(rule,t):
                for volume in compiled.rule_scopes.get(rule.id, []):
                    center=volume.geometry.representative_point()
                    nodes.append(np.array([center.x,center.y,start[2]]))
                    xmin,ymin,xmax,ymax=volume.geometry.bounds
                    for x,y in ((xmin,ymin),(xmin,ymax),(xmax,ymin),(xmax,ymax)):
                        nodes.append(np.array([x,y,start[2]]))
            if rule.rule_type!="PRIORITY" or rule.content["priorityScene"] in {"FAST", "PLANNING"}:
                continue
            kind,key=("noise","noise_db") if rule.content["priorityScene"]=="QUIET" else ("risk","risk")
            cells=[c for d in compiled.datasets_for(rule,kind) for c in d.cells if key in c.values and not c.values.get("nodata",False)]
            best=min((float(c.values[key]) for c in cells),default=0)
            for cell in cells:
                if float(cell.values[key])<=best+1e-9:
                    continue
                xmin,ymin,xmax,ymax=cell.geometry.bounds
                for x,y in [(xmin-margin,ymin-margin),(xmin-margin,ymax+margin),(xmax+margin,ymin-margin),(xmax+margin,ymax+margin)]:
                    nodes.append(np.array([x,y,start[2]]))
    nodes=np.unique(np.round(np.asarray(nodes),9),axis=0)
    si=int(np.argmin(np.linalg.norm(nodes-start,axis=1)))
    gi=int(np.argmin(np.linalg.norm(nodes-goal,axis=1)))
    nodes[si],nodes[gi]=start,goal
    edges,costs=[],[]
    # Sparse nearest-neighbour links plus original graph vertices. Small scenes
    # retain all links; large scenes never construct an all-pairs dense graph.
    from scipy.spatial import cKDTree
    tree=cKDTree(nodes)
    candidates=set()
    for i in range(len(nodes)):
        _,indices=tree.query(nodes[i],k=min(16,len(nodes)))
        for j in np.atleast_1d(indices):
            if i!=j:
                candidates.add((i,int(j)))
    for i,j in sorted(candidates):
        if time.perf_counter()>deadline:
            return None
        if _geometry_ok(compiled,nodes[i],nodes[j],t,task,profile):
            cost=_edge_cost(compiled,nodes[i],nodes[j],t,task,profile)
            edges.append((i,j));costs.append(cost)
    if not edges:
        return None
    result=native.graph_search(nodes,edges,costs,si,gi,deadline_ms=_native_deadline_ms(deadline),directed=True)
    if result["status"]=="deadline_exceeded":
        raise _DeadlineExceeded("Planning deadline exhausted during graph search")
    return nodes[result["path"]] if result["status"]=="ok" else None


def _grid_path(compiled,start,goal,task,profile,t,deadline,search_state=None):
    """Local 3D expansion, with exact final path verification."""
    from . import native
    model=compiled.scene if hasattr(compiled,"scene") else compiled.model
    b=model.bounds
    low=np.array([b.x_min,b.y_min,b.z_min]);high=np.array([b.x_max,b.y_max,b.z_max])
    resolution=np.array(model.resolution)
    blocked=[]
    # Geometry screening at cells is conservative; the chosen edges are still
    # verified against original geometry, so raster approximation cannot pass unsafe edges.
    if hasattr(compiled,"blocked_indices"):
        blocked=compiled.blocked_indices(task,profile,t)
    elif model.grid:
        blocked=model.grid.occupied
    options = task.search
    climb, descent = _angle_limits(compiled, task, t, compiled.model.horizon_s, cover_interval=True) if hasattr(compiled,"rules") else (90.0,90.0)
    budget = search_state["remaining"] if search_state is not None else options.max_expanded_nodes if options else None
    if budget is not None and budget <= 0:
        raise _SearchBudgetExceeded("Caller search expansion budget exhausted")
    search_kwargs={"deadline_ms":_native_deadline_ms(deadline)}
    if options is not None:
        search_kwargs.update(algorithm=options.algorithm,neighbors=options.neighbors,max_iterations=budget)
    if climb<90 or descent<90:
        search_kwargs.update(max_climb_angle=climb,max_descent_angle=descent)
    result=native.grid_search(start,goal,np.array([low,high]),resolution,blocked,**search_kwargs)
    if search_state is not None:
        search_state["expanded"] += result["expanded"]
    if search_state is not None and search_state["remaining"] is not None:
        search_state["remaining"] -= result["expanded"]
    if result["status"] == "iteration_limit":
        raise _SearchBudgetExceeded("Caller search expansion budget exhausted")
    if result["status"]=="deadline_exceeded":
        raise _DeadlineExceeded("Planning deadline exhausted during grid search")
    if result["status"] != "ok":
        return None
    points=np.asarray(result["path"])
    # Geographic round trips can differ from a lattice vertex by nanometres.
    # Keep exact task endpoints without introducing microscopic turn legs.
    if len(points)>2:
        interior=[p for p in points[1:-1] if np.linalg.norm(p-start)>1e-6 and np.linalg.norm(p-goal)>1e-6]
        points=np.asarray([start,*interior,goal])
    return points


def _path_for_leg(compiled,start,goal,task,profile,t,deadline,search_state=None):
    if task.search is not None:
        return _grid_path(compiled,start,goal,task,profile,t,deadline,search_state)
    direct=_geometry_ok(compiled,start,goal,t,task,profile)
    direct_cost=_edge_cost(compiled,start,goal,t,task,profile) if direct else float("inf")
    if direct and direct_cost<=np.linalg.norm(goal-start)*_cost_floor(compiled,task,t)*(1+1e-6):
        return np.asarray([start,goal])
    path=_roadmap(compiled,start,goal,task,profile,t,deadline)
    if path is not None:
        cost=sum(_edge_cost(compiled,a,b,t,task,profile) for a,b in zip(path,path[1:]))
        if cost<direct_cost-1e-7:
            return path
    if direct:
        return np.asarray([start,goal])
    if task.route_mode=="strict":
        # Centerline Z is a reference profile, not a mandatory flight height.
        # Try deterministic alternative layers while every edge still passes
        # the exact corridor, direction, obstacle and altitude-band checks.
        vertical=profile.height_m/2+profile.safety_margin_m+.2
        levels=set()
        step=max(1.0,float(compiled.model.resolution[2]))
        for airway in compiled.airways:
            for point in (start,goal):
                lon,lat=compiled.frame._inverse.transform(float(point[0]),float(point[1]))
                low=compiled.frame.altitude_to_local(airway.source.minAltitude,airway.alt_reference,lon,lat,task.departure_time)+vertical
                high=compiled.frame.altitude_to_local(airway.source.maxAltitude,airway.alt_reference,lon,lat,task.departure_time)-vertical
                low=max(low,compiled.model.bounds.z_min+vertical)
                high=min(high,compiled.model.bounds.z_max-vertical)
                if high>=low:
                    levels.update((low,high))
                    levels.update(np.arange(math.ceil(low/step)*step,high+1e-8,step))
        for altitude in sorted(levels,key=lambda z:(abs(z-start[2]),z)):
            if time.perf_counter()>=deadline:
                break
            path=_roadmap(compiled,start,goal,task,profile,t,deadline,altitude)
            if path is not None:
                return path
    if task.route_mode!="strict" and time.perf_counter()<deadline:
        return _grid_path(compiled,start,goal,task,profile,t,deadline,search_state)
    return None


def _join_motions(parts: list[Motion]) -> Motion:
    points=[];times=[];primitives=[];starts=[];events=[]
    for part in parts:
        skip=bool(times and abs(times[-1]-part.times[0])<1e-8)
        offset=len(points)-int(skip)
        points.extend(part.points[1:] if skip else part.points)
        times.extend(part.times[1:] if skip else part.times)
        events.extend((i+offset,types) for i,types in part.events)
        primitives.extend(part.primitives);starts.extend(part.starts)
    return Motion(np.asarray(points),np.asarray(times),primitives,starts,events,sum(p.distance for p in parts))


def _waypoint_time(motion: Motion, point: np.ndarray, not_before: float = -math.inf) -> float | None:
    encounters=[]
    for p,t in zip(motion.primitives,motion.starts):
        if t+p.duration < not_before-1e-7:
            continue
        if p.length <= 1e-12:
            if np.linalg.norm(p.start-point) <= .01:
                encounters.append(max(t,not_before))
            continue
        if p.kind=="arc":
            theta0=math.atan2(p.start[1]-p.center[1],p.start[0]-p.center[0])
            theta=math.atan2(point[1]-p.center[1],point[0]-p.center[0])
            angle=(theta-theta0)%(2*math.pi) if p.sweep>0 else -((theta0-theta)%(2*math.pi))
            f=np.clip(angle/p.sweep,0,1)
        else:
            d=p.end-p.start
            f=np.clip(np.dot(point-p.start,d)/max(np.dot(d,d),1e-30),0,1)
        s=float(f)*p.length
        error=float(np.linalg.norm(p.position(s)-point))
        passed=t+p.time_at(s)
        if error<=.01 and passed>=not_before-1e-7:
            encounters.append(max(not_before,passed))
    if not motion.primitives and np.linalg.norm(motion.points[0]-point)<=.01 and motion.times[0]>=not_before-1e-7:
        encounters.append(float(motion.times[0]))
    return min(encounters,default=None)


def _hover_duration(waypoint, task, compiled):
    duration=waypoint.hover_s
    if waypoint.hover and not duration:
        duration=max((r.content.get("defaultHoverTime",0) for r in compiled.rules.selected(task)
                      if r.rule_type=="HOVER"),default=0)
    return duration


def _waypoint_passages(motion, task, compiled):
    passages=[];not_before=float(motion.times[0])
    for waypoint in task.waypoints:
        passed=_waypoint_time(motion,_local(compiled,waypoint.wkt,task),not_before)
        if passed is None:
            return None
        passages.append(passed)
        not_before=passed+_hover_duration(waypoint,task,compiled)
    return passages


def _rtl_task(compiled, task, profile):
    rtl=compiled.rules.rtl_requirements(task,profile)
    if not rtl or not rtl["required"]:
        return task,rtl
    if task.state is None or task.state.home_wkt is None:
        raise PlanningError(ErrorCode.INVALID_INPUT,"Triggered RTL needs home_wkt",{"task_id":task.id})
    home=compiled.frame.to_local(task.state.home_wkt,task.state.alt_reference or task.alt_reference,
                                 task.state.at or task.departure_time)
    converted=compiled.frame.to_wgs84(home,task.alt_reference,task.departure_time)
    return task.model_copy(update={"goal_wkt":converted,"waypoints":[]}),rtl


def _dynamic_tracks(compiled, model):
    from .safety import dynamic_tracks
    try:
        return dynamic_tracks(compiled,model)
    except ValueError as exc:
        raise PlanningError(ErrorCode.INVALID_INPUT,str(exc)) from exc


def _transient_segments(motion, times):
    moving=[(p,t) for p,t in zip(motion.primitives,motion.starts) if p.length>1e-9]
    if not moving:
        return []
    first,tf=moving[0];last,tl=moving[-1]
    return [j for j,(a,b) in enumerate(zip(times,times[1:]))
            if b<=tf+first.ta+1e-7 or a>=tl+last.ta+last.tc-1e-7]


def _final_samples(compiled, model, motion):
    from .safety import temporal_breakpoints
    boundaries=temporal_breakpoints(compiled,model)
    a=float(motion.times[0]);b=float(motion.times[-1])
    i=np.searchsorted(boundaries,a,side="right")
    if i<len(boundaries) and boundaries[i]<b:
        return motion.collision_samples()
    return motion.points,motion.times


def _timed_geographic(compiled, points, times, reference, model):
    """Export every altitude against the evidence at its own timestamp."""
    from shapely import to_wkt
    rows=[]
    for p,t in zip(points,times):
        lon,lat=compiled.frame._inverse.transform(float(p[0]),float(p[1]),errcheck=True)
        z=compiled.frame.altitude_from_ellipsoid(float(p[2])+compiled.frame.spec.origin_alt_ellipsoid,
            reference,lon,lat,model.time_origin+timedelta(seconds=float(t)))
        rows.append((lon,lat,z))
    geom=Point(rows[0]) if len(rows)==1 else LineString(rows)
    return to_wkt(geom,rounding_precision=12,trim=True)


def _timed_local(compiled, wkt, times, reference, model):
    from shapely import from_wkt
    geometry=from_wkt(wkt)
    if geometry.geom_type not in {"Point","LineString"} or not geometry.has_z:
        raise PlanningError(ErrorCode.INVALID_INPUT,"Trajectory must be POINT Z or LINESTRING Z")
    rows=list(geometry.coords)
    if len(rows)!=len(times):
        raise PlanningError(ErrorCode.INVALID_INPUT,"Trajectory coordinates/times must match")
    return np.asarray([compiled.frame.to_local(Point(row).wkt,reference,
        model.time_origin+timedelta(seconds=float(t))).coords[0] for row,t in zip(rows,times)])


def _task_motion(legs,task,compiled,limits,departure):
    durations=[]
    for waypoint in task.waypoints:
        duration=waypoint.hover_s
        if waypoint.hover and duration==0:
            for rule in compiled.rules.selected(task):
                if rule.rule_type=="HOVER":
                    duration=max(duration,rule.content.get("defaultHoverTime",0))
        durations.append(duration)
    if not any(durations):
        full=list(legs[0])
        for leg in legs[1:]:
            full.extend(leg[1:])
        return build_motion(full,limits,departure)
    parts=[];t=departure
    for i,leg in enumerate(legs):
        moving=build_motion(leg,limits,t)
        parts.append(moving);t=float(moving.times[-1])
        if i>=len(durations):
            continue
        duration=durations[i]
        if duration<=0:
            continue
        position=leg[-1]
        delta=0.0
        if i+1<len(legs) and len(leg)>=2 and len(legs[i+1])>=2:
            a=leg[-1]-leg[-2];b=legs[i+1][1]-legs[i+1][0]
            if min(np.linalg.norm(a[:2]),np.linalg.norm(b[:2]))>1e-8:
                delta=math.atan2(a[0]*b[1]-a[1]*b[0],np.dot(a[:2],b[:2]))
        if abs(delta)>1e-8 and limits.get("min_turn_radius_m",0)>0:
            raise MotionError("A mandatory hover point cannot bypass positive turning radius")
        if abs(delta)>math.radians(limits["max_yaw_rate_deg_s"])*duration+1e-8:
            raise MotionError("Declared hover duration is shorter than the required yaw time")
        hold=Primitive("yaw" if abs(delta)>1e-8 else "hover",position.copy(),position.copy(),0,0,
                       duration=duration,sweep=delta)
        parts.append(Motion(np.asarray([position,position]),np.array([t,t+duration]),[hold],[t],
                            [(0,["WAYPOINT","HOVER_START"]),(1,["HOVER_END"])],0))
        t+=duration
    return _join_motions(parts)


def _sipp_motion(motion, reservations, reservation_times, task, profile, limits, latest, deadline):
    """Search safe intervals on shaped, dynamically timed macro edges.

    A macro ends only at an existing zero-speed state. Acceleration phases and
    continuous-speed arcs remain inside an indivisible edge, so waiting can
    never create an instantaneous stop or change the time law in flight.
    """
    from . import native
    if not reservations or not motion.primitives:
        return motion
    cp,ct=motion.collision_samples()
    groups=[];first=0
    for i,p in enumerate(motion.primitives):
        if p.v1<=1e-9 and (i+1==len(motion.primitives) or motion.primitives[i+1].v0<=1e-9):
            groups.append((first,i+1));first=i+1
    if first<len(motion.primitives):
        groups.append((first,len(motion.primitives)))
    nodes=[];edge_paths=[];edge_times=[];durations=[];submotions=[]
    for a,b in groups:
        begin=motion.starts[a]
        finish=motion.starts[b-1]+motion.primitives[b-1].duration
        selected=np.flatnonzero((ct>=begin-1e-8)&(ct<=finish+1e-8))
        if len(selected)<2 or finish<=begin:
            continue
        path=cp[selected]
        times=ct[selected]
        if not nodes:
            nodes.append(path[0])
        nodes.append(path[-1])
        edge_paths.append(path);edge_times.append(times-begin);durations.append(finish-begin)
        output=np.flatnonzero((motion.times>=begin-1e-8)&(motion.times<=finish+1e-8))
        events=[(int(np.searchsorted(output,i)),types) for i,types in motion.events if i in output]
        submotions.append(Motion(motion.points[output],motion.times[output],motion.primitives[a:b],motion.starts[a:b],events,
                                 sum(p.length for p in motion.primitives[a:b])))
    if not durations:
        return motion
    h,v=0.0,0.0
    # Caller passes certified separations through limits; no safety rule is weakened.
    h=max(limits.get("horizontal_separation_m",0),2*(profile.radius_m+profile.safety_margin_m))+.1
    v=max(limits.get("vertical_separation_m",0),profile.height_m+2*profile.safety_margin_m)+.1
    can_wait=profile.can_hover and limits.get("min_speed_mps",0)==0
    answer=native.solve_sipp(nodes,[(i,i+1) for i in range(len(durations))],durations,0,len(nodes)-1,
        edge_paths=edge_paths,edge_times=edge_times,wait_allowed=[can_wait]*len(nodes),
        reservations_paths=reservations,reservations_times=reservation_times,
        horizontal_separation=h,vertical_separation=v,start_time=float(motion.times[0]),latest_arrival=latest,
        directed=True,deadline_ms=_native_deadline_ms(deadline))
    if answer["status"]=="deadline_exceeded":
        raise _DeadlineExceeded("Planning deadline exhausted during safe-interval search")
    if answer["status"]!="ok":
        raise MotionError("No safe interval on the current dynamically feasible candidate")
    parts=[]
    for j,(a,b) in enumerate(zip(answer["path"],answer["path"][1:])):
        start,end=answer["times"][j:j+2]
        if a==b:
            if not parts:
                continue  # pre-launch delay is not airborne occupancy
            point=np.asarray(nodes[a])
            hold=Primitive("hover",point.copy(),point.copy(),0,0,duration=end-start)
            parts.append(Motion(np.asarray([point,point]),np.array([start,end]),[hold],[start],
                                [(0,["HOVER_START"]),(1,["HOVER_END"])],0))
        else:
            part=submotions[a]
            parts.append(part.shifted(start-float(part.times[0])))
    return _join_motions(parts)


def _failure(task,code,message,violations=(),report=()):
    return {"task_id":task.id,"engine_id":task.engine_id,"status":"timeout" if code=="TIME_BUDGET_EXCEEDED" else "failed",
            "code":code,"message":message,"route_mode":task.route_mode,"alt_reference":task.alt_reference,
            "start_wkt":task.start_wkt,"goal_wkt":task.goal_wkt,"departure_time":task.departure_time,
            "validation":{"valid":False,"violations":list(violations)},"rule_report":list(report)}


def _report(compiled,task,violations,times=None,points=None):
    reports=compiled.rules.report(task,violations,times=times,points=points)
    return [r.model_dump(mode="json") if hasattr(r,"model_dump") else r for r in reports]


def _arc_boundary_fractions(p, geometry):
    """Intersect the actual horizontal circle with a polygon's linear boundary."""
    fractions=[]
    if geometry is None or geometry.is_empty:
        return fractions
    def lines(part):
        if hasattr(part,"geoms"):
            for child in part.geoms:
                yield from lines(child)
        elif hasattr(part,"coords"):
            yield np.asarray(part.coords,dtype=float)[:,:2]
    angle0=math.atan2(p.start[1]-p.center[1],p.start[0]-p.center[0])
    for line in lines(geometry.boundary):
        for a,b in zip(line,line[1:]):
            d=b-a;relative=a-p.center[:2]
            aa=float(d@d)
            if aa<=1e-24:
                continue
            bb=2*float(relative@d);cc=float(relative@relative)-p.radius**2
            discriminant=bb*bb-4*aa*cc
            if discriminant<0:
                continue
            for u in ((-bb-math.sqrt(discriminant))/(2*aa),(-bb+math.sqrt(discriminant))/(2*aa)):
                if not -1e-9<=u<=1+1e-9:
                    continue
                point=a+u*d
                angle=math.atan2(point[1]-p.center[1],point[0]-p.center[0])
                delta=(angle-angle0)%(2*math.pi) if p.sweep>0 else -((angle0-angle)%(2*math.pi))
                f=delta/p.sweep
                if -1e-9<=f<=1+1e-9:
                    fractions.append(float(np.clip(f,0,1)))
    return fractions


def _primitive_airway_intervals(compiled, p, start, airway, profile, boundaries):
    horizontal,vertical=compiled.clearance(profile)
    end=start+p.duration
    cuts={0.0,p.duration}
    cuts.update(b-start for b in boundaries if start<b<end)
    if p.kind=="arc" and p.length>1e-12:
        # Bound the preliminary geometric chords, then partition them at exact
        # circle/footprint crossings and classify actual analytic positions.
        step=2*math.acos(max(-1,min(1,1-.05/max(p.radius,.05))))
        count=max(1,math.ceil(abs(p.sweep)/max(step,1e-8)))
        fractions={i/count for i in range(count+1)}
        fractions.update(_arc_boundary_fractions(p,compiled._buffered(airway,-horizontal)))
        offsets={0.0}
        if airway.alt_reference in {"AGL","QNE","QNH"}:
            from .safety import _datum_regions
            regions=_datum_regions(compiled,compiled.model,airway.alt_reference)
            offsets={r.offset-compiled.origin_alt_ellipsoid for r in regions if r.offset is not None}
            for region in regions:
                if region.start<=end and region.end>=start:
                    fractions.update(_arc_boundary_fractions(p,region.geometry))
        dz=float(p.end[2]-p.start[2])
        if abs(dz)>1e-12:
            for offset in offsets:
                for height in (airway.min_altitude+vertical+offset,airway.max_altitude-vertical+offset):
                    f=(height-p.start[2])/dz
                    if 0<f<1:
                        fractions.add(float(f))
        cuts.update(p.time_at(f*p.length) for f in fractions)
    output=[]
    for a,b in zip(sorted(cuts),sorted(cuts)[1:]):
        if b-a<=1e-9:
            continue
        middle=start+(a+b)/2
        s0,s1=p.distance_at(a),p.distance_at(b)
        x,y=p.position(s0),p.position(s1)
        if airway.direction!="BOTH" and np.linalg.norm(y[:2]-x[:2])>1e-7:
            first,last=airway.centerline.project(Point(x[:2])),airway.centerline.project(Point(y[:2]))
            if ((airway.direction=="FORWARD" and last<first-1e-7)
                    or (airway.direction=="REVERSE" and last>first+1e-7)):
                continue
        if p.kind=="arc" or p.length<=1e-12:
            point=p.position(p.distance_at((a+b)/2))
            if compiled.volume_intervals(airway,point,point,middle,middle,
                    horizontal=horizontal,vertical=vertical,inside=True):
                output.append((start+a,start+b))
            continue
        for lo,hi in compiled.volume_intervals(airway,x,y,middle,middle,
                horizontal=horizontal,vertical=vertical,inside=True):
            if hi-lo>1e-9:
                output.append((start+p.time_at(s0+lo*(s1-s0)),start+p.time_at(s0+hi*(s1-s0))))
    return output


def _airway_switch_events(compiled, motion, profile, segments):
    """Retain a chosen airway until coverage ends; hand off by stable id.

    Handoffs are evaluated on the analytic law and can fall between exported
    WKT vertices. This annotates the route without changing its geometry/timing.
    """
    used={rid for segment in segments for rid in segment["airway_ids"]}
    if any(p.kind=="arc" for p in motion.primitives):
        for p in motion.primitives:
            if p.kind=="arc":
                lo=p.center.copy();hi=p.center.copy()
                lo[:2]-=p.radius;hi[:2]+=p.radius
                used.update(a.id for a in compiled._query(compiled._airway_tree,compiled.airways,lo,hi)
                            if a.status=="ACTIVE")
    if len(used)<=1:
        return []
    from .safety import temporal_breakpoints
    boundaries=temporal_breakpoints(compiled,compiled.model)
    coverage={rid:[] for rid in used}
    for p,start in zip(motion.primitives,motion.starts):
        for rid in used:
            airway=compiled.resources[rid]
            if airway.valid_to_s<start or airway.valid_from_s>start+p.duration:
                continue
            coverage[rid].extend(_primitive_airway_intervals(compiled,p,start,airway,profile,boundaries))
    cuts=sorted({t for intervals in coverage.values() for interval in intervals for t in interval})
    current=None;events=[]
    for a,b in zip(cuts,cuts[1:]):
        if b-a<=1e-8:
            continue
        middle=(a+b)/2
        active=sorted(rid for rid,intervals in coverage.items() if any(lo<=middle<=hi for lo,hi in intervals))
        if current in active:
            continue
        previous=current;current=active[0] if active else None
        if previous is not None and current is not None:
            i=max(0,min(len(motion.primitives)-1,int(np.searchsorted(motion.starts,a,side="right")-1)))
            p=motion.primitives[i]
            events.append((float(a),p.position(p.distance_at(a-motion.starts[i]))))
    return events


def _result(compiled,model,task,profile,motion,wait=0):
    from .models import RouteResult
    wkt=_timed_geographic(compiled,motion.points,motion.times,task.alt_reference,model)
    segments=[]
    for p,t in zip(motion.primitives,motion.starts):
        segments.append({"kind":p.kind,"start_time_s":t,"end_time_s":t+p.duration,
            "airway_ids":compiled.airway_ids_for_segment(p.start,p.end,t,t+p.duration,profile),
            "start":p.start.tolist(),"end":p.end.tolist(),"length_m":p.length,
            "radius_m":p.radius if p.kind=="arc" else None,
            "center":p.center.tolist() if p.center is not None else None,
            "start_speed_mps":p.v0,"end_speed_mps":p.v1,"acceleration_mps2":p.accel,
            "profile":{"peak_speed_mps":p.peak,"deceleration_mps2":p.decel,"accel_time_s":p.ta,
                       "cruise_time_s":p.tc,"decel_time_s":p.td,"sweep_angle_deg":math.degrees(p.sweep)}})
    events=[]
    for index,types in motion.events:
        events.append((float(motion.times[index]),motion.points[index],list(types),None))
    passages=_waypoint_passages(motion,task,compiled) or []
    for waypoint_index,(waypoint,passed) in enumerate(zip(task.waypoints,passages)):
        events.append((passed,_local(compiled,waypoint.wkt,task),["WAYPOINT"],waypoint_index))
    for passed,point in _airway_switch_events(compiled,motion,profile,segments):
        events.append((passed,point,["AIRWAY_SWITCH"],None))
    merged=[]
    for passed,point,types,waypoint_index in sorted(events,key=lambda item:item[0]):
        previous=merged[-1] if merged else None
        if (previous is not None and abs(previous[0]-passed)<=1e-7
                and np.linalg.norm(previous[1]-point)<=1e-6
                and (previous[3] is None or waypoint_index is None or previous[3]==waypoint_index)):
            previous[2].update(types)
            if waypoint_index is not None:
                previous[3]=waypoint_index
        else:
            merged.append([passed,point,set(types),waypoint_index])
    keypoints=[]
    from .grid_traversal import traversed_grids
    traversal, grid_context = traversed_grids(model, motion)
    for passed,point,types,waypoint_index in merged:
        code=None
        if model.grid:
            q=(point-np.asarray(model.grid.origin))/np.asarray(model.grid.resolution)
            nearest=np.rint(q)
            external_index=np.floor(np.where(np.abs(q-nearest)<=1e-7,nearest,q)).astype(int)
            code=model.grid.codes.get(",".join(map(str,external_index)))
            if code is None:
                for cell in model.grid.cells:
                    b=cell.bounds
                    if (cell.code is not None and b.x_min<=point[0]<=b.x_max
                            and b.y_min<=point[1]<=b.y_max and b.z_min<=point[2]<=b.z_max):
                        code=cell.code
                        break
        # Existing keypoint indices name the planner grid; traversal explicitly
        # identifies its external frame separately in grid_context.
        origin=np.asarray([model.bounds.x_min,model.bounds.y_min,model.bounds.z_min])
        q=(point-origin)/np.asarray(model.resolution)
        nearest=np.rint(q)
        index=np.floor(np.where(np.abs(q-nearest)<=1e-7,nearest,q)).astype(int).tolist()
        keypoints.append({"types":sorted(types),"wkt":_timed_geographic(compiled,[point],[passed],task.alt_reference,model),
                          "time_s":passed,"grid_indices":index,"grid_code":code,"waypoint_index":waypoint_index})
    result={"task_id":task.id,"engine_id":task.engine_id,"status":"planned","route_mode":task.route_mode,
        "alt_reference":task.alt_reference,"coordinate_context":compiled.frame.context,
        "start_wkt":task.start_wkt,"goal_wkt":task.goal_wkt,"departure_time":task.departure_time,
        "time_origin":model.time_origin,"action":"return_to_home" if (compiled.rules.rtl_requirements(task,profile) or {}).get("required") else "mission",
        "trajectory_wkt":wkt,"vertex_times_s":motion.times.tolist(),"motion_segments":segments,"keypoints":keypoints,
        "traversed_grids":traversal,"grid_context":grid_context,
        "metrics":{"distance_m":motion.distance,"flight_time_s":float(motion.times[-1]-motion.times[0]),"wait_time_s":wait},
        "validation":{"valid":True,"violations":[]},"rule_report":_report(compiled,task,[],motion.times,motion.points)}
    return RouteResult.model_validate(result).model_dump(mode="json")


def _prepare_candidate(index, task, model, compiled, dynamic_tracks, deadline, search_state, *, objective_legs=None):
    """Prepare one independent route; reservation and final checks stay global."""
    from . import native
    if task.objectives is not None and objective_legs is None:
        from .objectives import prepare
        return prepare(index,task,model,compiled,dynamic_tracks,deadline,search_state)
    if time.perf_counter()>=deadline:
        return None, _failure(task,"TIME_BUDGET_EXCEEDED","Planning deadline exhausted")
    profile=_profile(model,task)
    limits=_limits(compiled,task,profile)
    departure=_seconds(task.departure_time,model)
    if departure<0 or departure>model.horizon_s:
        raise PlanningError(ErrorCode.INVALID_INPUT,"Departure is outside scene horizon",{"task_id":task.id})
    if task.route_mode=="strict" and not compiled.airways:
        raise PlanningError(ErrorCode.INVALID_INPUT,"strict requires at least one active airway",{"task_id":task.id})
    task,rtl=_rtl_task(compiled,task,profile)
    if task.network and rtl and rtl["required"]:
        raise PlanningError(ErrorCode.INVALID_INPUT, "Network selection cannot be combined with an RTL diversion")
    anchors=[_local(compiled,task.start_wkt,task)]+[_local(compiled,w.wkt,task) for w in task.waypoints]+[_local(compiled,task.goal_wkt,task)]
    if rtl and rtl["required"]:
        altitude=rtl["rtl_altitude_m"]
        a,b=anchors[0].copy(),anchors[-1].copy()
        a[2]=b[2]=compiled.frame.altitude_to_local(altitude,"EG",
            *compiled.frame._inverse.transform(float(a[0]),float(a[1])),task.departure_time)
        anchors=[anchors[0],a,b,anchors[-1]]
    try:
        legs=[]; network_trace=None
        if objective_legs is not None:
            legs=objective_legs
        elif task.network:
            selected=compiled.network.select(task,profile,departure,deadline)
            if selected is None:
                return None, _failure(task,"NETWORK_DISCONNECTED","No active, admissible directed network path connects the requested nodes")
            legs,network_trace=selected
        else:
            for a,b in zip(anchors,anchors[1:]):
                leg=_path_for_leg(compiled,a,b,task,profile,departure,deadline,search_state)
                if leg is None:
                    raise MotionError("No route found at the current resolution and constraints")
                legs.append(leg)
        limits=_airway_motion_limits(compiled,task,limits,np.concatenate(legs))
        motion=_task_motion(legs,task,compiled,limits,departure)
        latest=min(model.horizon_s,_seconds(task.latest_arrival,model)) if task.latest_arrival else model.horizon_s
        if motion.times[-1]>latest+1e-7:
            raise MotionError("Candidate exceeds arrival or scene time horizon")
        if motion.times[-1]-motion.times[0]>profile.endurance_s+1e-7 or motion.distance>profile.max_range_m+1e-7:
            raise MotionError("Candidate exceeds aircraft endurance or range")
        if model.dynamic_obstacles:
            reservations=[p for p,t in dynamic_tracks]
            reservation_times=[t for p,t in dynamic_tracks]
            sh,sv=compiled.rules.separation(task)
            limits["horizontal_separation_m"]=max(sh,profile.radius_m+profile.safety_margin_m+
                max(d.horizontal_separation_m for d in model.dynamic_obstacles))
            limits["vertical_separation_m"]=max(sv,profile.height_m/2+profile.safety_margin_m+
                max(d.vertical_separation_m for d in model.dynamic_obstacles))
            motion=_sipp_motion(motion,reservations,reservation_times,task,profile,limits,latest,deadline)
        certificate=native.validate_motion(motion.primitives,limits,starts=motion.starts)
        if not certificate["valid"]:
            raise MotionError("Independent motion validation rejected candidate: "+str(certificate["violations"]))
        # Reject impossible durations before allocating the certified cover.
        cp,ct=motion.collision_samples()
    except _SearchBudgetExceeded as exc:
        return None, _failure(task,"SEARCH_BUDGET_EXCEEDED",str(exc))
    except (_DeadlineExceeded,TimeoutError) as exc:
        return None, _failure(task,"TIME_BUDGET_EXCEEDED",str(exc))
    except (MotionError,ValueError) as exc:
        code="TIME_BUDGET_EXCEEDED" if time.perf_counter()>=deadline else "SEARCH_NOT_FOUND"
        return None, _failure(task,code,str(exc))
    # Dynamic reservations are checked on a certified space-time chord cover.
    h,v=compiled.rules.separation(task)
    h=max(h,2*(profile.radius_m+profile.safety_margin_m))+.1
    v=max(v,profile.height_m+2*profile.safety_margin_m)+.1
    earliest_delay=max(0,_seconds(task.earliest_arrival,model)-float(ct[-1])) if task.earliest_arrival else 0
    window=latest-float(ct[-1])
    passages=_waypoint_passages(motion,task,compiled)
    if passages is None:
        window=-1
    for waypoint,passed in zip(task.waypoints,passages or []):
        if waypoint.earliest:
            earliest_delay=max(earliest_delay,_seconds(waypoint.earliest,model)-passed)
        if waypoint.latest:
            window=min(window,_seconds(waypoint.latest,model)-passed)
    if window<earliest_delay-1e-7:
        return None, _failure(task,"TIME","No departure can satisfy the arrival window")
    from .time_windows import departure_windows
    alternatives=departure_windows(compiled,task,motion,float(ct[0])+earliest_delay,float(ct[0])+window)
    if not alternatives:
        return None, _failure(task,"TIME","No permitted TIME-rule departure window")
    return {"index":index,"task":task,"profile":profile,"motion":motion,
                       "network_trace":network_trace,"points":cp,"times":ct,"h":h,"v":v,"lo":alternatives[0][0],
                       "hi":alternatives[0][1],"windows":alternatives,"latest":latest}, None


def plan_batch_v2(*,tasks,scene_id=None,scene=None,request_id=None,time_budget_s: float | None=None,execution=None):
    from . import native
    native.require_native()
    start=time.perf_counter();deadline=_planning_deadline(start,time_budget_s)
    execution_options = _parse(ExecutionOptions, execution if execution is not None else {})
    if (scene_id is None)==(scene is None):
        raise PlanningError(ErrorCode.INVALID_INPUT,"Provide exactly one of scene_id or scene")
    if scene is not None:
        scene_id=register_scene(scene)["scene_id"]
    model,compiled,version,_=registry.get(scene_id)
    dynamic_tracks=_dynamic_tracks(compiled,model)
    from .network import normalize_task
    task_models=[normalize_task(compiled, _parse(Task,t)) for t in tasks]
    for task in task_models:
        if task.search and task.search.grid_level is not None and (model.grid is None or model.grid.level != task.search.grid_level):
            raise PlanningError(ErrorCode.INVALID_INPUT, "search.grid_level must match scene.grid.level", {"task_id": task.id})
    if not task_models or len({t.id for t in task_models})!=len(task_models):
        raise PlanningError(ErrorCode.INVALID_INPUT,"Tasks must be nonempty and have unique ids")
    ordered=sorted(enumerate(task_models),key=lambda it:(-it[1].priority,it[1].latest_arrival or datetime.max.replace(tzinfo=timezone.utc),it[1].id))
    parsed_at=time.perf_counter()
    results=[None]*len(task_models)
    candidates=[]
    search_states={t.id:{"remaining":t.search.max_expanded_nodes,"expanded":0} for t in task_models if t.search}
    from .execution import run_candidates
    outcomes, execution_report = run_candidates(
        ordered, compiled, execution_options,
        lambda index, task, worker_scene: _prepare_candidate(
            index, task, model, worker_scene, dynamic_tracks, deadline, search_states.get(task.id)))
    for (index, task), (candidate, failure) in zip(ordered, outcomes):
        if failure is not None:
            results[index] = failure
        else:
            candidates.append(candidate)
    execution_report.candidate_count = len(candidates)
    # A single native batch shares its temporal index across every reservation.
    # Rebuilding that index once per task is cubic and defeats scene preloading.
    searched_at=time.perf_counter()
    paths=[];timings=[];windows=[];priorities=[]
    for points,times in dynamic_tracks:
        paths.append(points);timings.append(times)
        windows.append([float(times[0])]*2);priorities.append(2_000_000_000)
    dynamic_count=len(paths)
    for c in candidates:
        paths.append(c["points"]);timings.append(c["times"])
        windows.append([c["lo"],c["hi"]]);priorities.append(len(candidates)-len(priorities)+dynamic_count)
    h=max((c["h"] for c in candidates),default=10.7)
    v=max((c["v"] for c in candidates),default=10.3)
    radius=max((c["profile"].radius_m+c["profile"].safety_margin_m for c in candidates),default=0)
    height=max((c["profile"].height_m/2+c["profile"].safety_margin_m for c in candidates),default=0)
    h=max([h]+[radius+d.horizontal_separation_m+.1 for d in model.dynamic_obstacles])
    v=max([v]+[height+d.vertical_separation_m+.1 for d in model.dynamic_obstacles])
    try:
        scheduled=native.schedule_paths(paths,timings,windows,horizontal_separation=h,vertical_separation=v,
            priorities=priorities,fixed_count=dynamic_count,
            deadline_ms=_native_deadline_ms(deadline)) if paths else {"delays":[]}
    except _DeadlineExceeded:
        scheduled={"delays":[None]*len(paths)}
    # Most tasks have one interval. Only unsuccessful tasks with additional
    # calendar openings need another search; all previous reservations stay fixed.
    retry=[k for k,c in enumerate(candidates) if scheduled["delays"][dynamic_count+k] is None and len(c["windows"])>1]
    if retry:
        fixed_paths=list(paths[:dynamic_count]);fixed_times=list(timings[:dynamic_count])
        for k,c in enumerate(candidates):
            delay=scheduled["delays"][dynamic_count+k]
            if delay is not None:
                fixed_paths.append(c["points"]);fixed_times.append(c["times"]+delay)
        for k in retry:
            c=candidates[k]
            for interval in c["windows"][1:]:
                if time.perf_counter()>=deadline:
                    break
                fixed_windows=[[float(ts[0])]*2 for ts in fixed_times]
                try:
                    answer=native.schedule_paths(fixed_paths+[c["points"]],fixed_times+[c["times"]],fixed_windows+[interval],
                        horizontal_separation=h,vertical_separation=v,fixed_count=len(fixed_paths),
                        deadline_ms=_native_deadline_ms(deadline))
                except _DeadlineExceeded:
                    break
                delay=answer["delays"][-1]
                if delay is not None:
                    scheduled["delays"][dynamic_count+k]=delay
                    fixed_paths.append(c["points"]);fixed_times.append(c["times"]+delay)
                    break
    scheduled_at=time.perf_counter()
    accepted=[];accepted_times=[];accepted_profiles=[];accepted_tasks=[]
    for k,c in enumerate(candidates):
        task,profile,motion,index=c["task"],c["profile"],c["motion"],c["index"]
        delay=scheduled["delays"][dynamic_count+k]
        if delay is None:
            statuses=scheduled.get("statuses",[])
            timed_out=(time.perf_counter()>=deadline or scheduled.get("status")=="deadline_exceeded"
                       or (dynamic_count+k<len(statuses) and statuses[dynamic_count+k]=="deadline_exceeded"))
            results[index]=_failure(task,"TIME_BUDGET_EXCEEDED" if timed_out else "SEARCH_NOT_FOUND","No safe departure interval within arrival window");continue
        if delay:
            motion=motion.shifted(delay)
        cp,ct=c["points"],c["times"]+delay
        final_points,final_times=_final_samples(compiled,model,motion)
        transients=_transient_segments(motion,final_times)
        rtl=compiled.rules.rtl_requirements(task,profile)
        certified_profile=profile.model_copy(update={"safety_margin_m":profile.safety_margin_m+.1})
        violations=compiled.check_path(final_points,final_times,task,certified_profile,
            phase="rtl" if rtl and rtl["required"] else "route",speed_transient_segments=transients,
            slope_angles=_motion_slope_angles(motion,final_times))
        if task.network:
            violations.extend(compiled.network.validate(task,final_points,final_times,c["network_trace"],motion=motion))
        if ct[-1]>c["latest"]+1e-7:
            violations.append({"code":"TIME","message":"Latest arrival exceeded"})
        if task.earliest_arrival and ct[-1]<_seconds(task.earliest_arrival,model)-1e-7:
            violations.append({"code":"TIME","message":"Earliest arrival not satisfied"})
        if violations:
            results[index]=_failure(task,"CONSTRAINT_VIOLATION","Final independent validation failed",violations,_report(compiled,task,violations));continue
        results[index]=_result(compiled,model,task,profile,motion,delay)
        results[index]["network_route"]=c["network_trace"]
        c["accepted_motion"]=motion
        c["accepted_delay"]=delay
        accepted.append(cp);accepted_times.append(ct);accepted_profiles.append(profile);accepted_tasks.append(task)
    # Re-run independent pair validation, not only trust scheduling success.
    conflicts=[]
    if accepted:
        # Every upstream trajectory remains an obstacle, even if upstream tracks
        # intersect one another. Such intersections never remove an obstacle.
        try:
            check=native.continuous_conflicts(paths[:dynamic_count]+accepted,timings[:dynamic_count]+accepted_times,h,v,
                deadline_ms=_native_deadline_ms(deadline))
        except _DeadlineExceeded:
            check={"status":"deadline_exceeded","conflicts":[]}
        for c in check["conflicts"]:
            if c["first"]<dynamic_count and c["second"]<dynamic_count:
                continue
            a=(model.dynamic_obstacles[c["first"]].id if c["first"]<dynamic_count else accepted_tasks[c["first"]-dynamic_count].id)
            b=(model.dynamic_obstacles[c["second"]].id if c["second"]<dynamic_count else accepted_tasks[c["second"]-dynamic_count].id)
            conflicts.append({"task_ids":[a,b],"start_time_s":c["start"],"end_time_s":c["end"]})
            for i,t in enumerate(task_models):
                if t.id in (a,b):
                    results[i]=_failure(t,"SEPARATION","Residual continuous collision")
        if check["status"]!="ok":
            code="TIME_BUDGET_EXCEEDED" if check["status"]=="deadline_exceeded" else "VALIDATION_INCOMPLETE"
            for i,t in enumerate(task_models):
                if results[i]["status"]=="planned":
                    results[i]=_failure(t,code,"Continuous separation validation did not complete: "+check["status"])
    from .objectives import attach_results
    attach_results(compiled,model,candidates,results,dynamic_tracks,deadline,h,v)
    for task,result in zip(task_models,results):
        result["coordinate_context"]=compiled.frame.context
        if task.search:
            result["search_report"]={"algorithm":task.search.algorithm,"neighbors":task.search.neighbors,
                "grid_level":model.grid.level if model.grid else None,
                "expanded_nodes":search_states[task.id]["expanded"],"max_expanded_nodes":task.search.max_expanded_nodes}
            for alternative in result.get("alternatives",[]):
                alternative["search_report"]=dict(result["search_report"])
    count=sum(r["status"]=="planned" for r in results)
    validated_at=time.perf_counter()
    result=BatchResult.model_validate({"planned_at":datetime.now(timezone.utc),"request_id":request_id or uuid.uuid4().hex,
        "scene_id":scene_id,"scene_version":version,"routes":results,"success_count":count,
        "failed_count":len(results)-count,"conflicts":conflicts,"elapsed_ms":(time.perf_counter()-start)*1000,"engine":"native-v2",
        "coordinate_context":compiled.frame.context, "execution_report":execution_report,
        "stage_timings_ms":{"scene_and_task_parsing":(parsed_at-start)*1000,
            "search_motion_and_certificate":(searched_at-parsed_at)*1000,
            "reservation_scheduling":(scheduled_at-searched_at)*1000,
            "final_validation_and_wkt":(validated_at-scheduled_at)*1000}})
    output=result.model_dump(mode="json")
    output["stage_timings_ms"]["sdk_response_serialization"]=(time.perf_counter()-validated_at)*1000
    return output


def plan_single_v2(*,task,scene_id=None,scene=None,time_budget_s: float | None=None):
    return plan_batch_v2(tasks=[task],scene_id=scene_id,scene=scene,time_budget_s=time_budget_s)["routes"][0]


def _reconstruct_motion(segments, points, times):
    """Rebuild the public analytic law; never infer accelerations from chords."""
    primitives=[];starts=[]
    for value in segments:
        item=_parse(MotionSegment,value)
        profile=item.profile
        def parameter(name, nested=None, default=None):
            value=profile.get(nested or name,getattr(item,name,None))
            if value is None:
                if default is None:
                    raise ValueError("Motion primitive is missing "+name)
                return default
            return float(value)
        peak=parameter("peak_speed_mps")
        p=Primitive(item.kind,np.asarray(item.start,dtype=float),np.asarray(item.end,dtype=float),
            float(item.length_m),peak,
            center=None if item.center is None else np.asarray(item.center,dtype=float),
            radius=float(item.radius_m or 0),sweep=math.radians(parameter("sweep_angle_deg",default=0)),
            duration=float(item.end_time_s-item.start_time_s),
            v0=parameter("start_speed_mps"),v1=parameter("end_speed_mps"),peak=peak,
            accel=parameter("acceleration_mps2"),
            decel=parameter("deceleration_mps2",nested="deceleration_mps2"),
            ta=parameter("accel_time_s"),tc=parameter("cruise_time_s"),td=parameter("decel_time_s"))
        primitives.append(p);starts.append(float(item.start_time_s))
    return Motion(points,times,primitives,starts,distance=sum(p.length for p in primitives))


def _segment_distance(point, start, end):
    delta=end-start
    f=float(np.clip(np.dot(point-start,delta)/max(np.dot(delta,delta),1e-30),0,1))
    return float(np.linalg.norm(point-start-f*delta))


def _motion_view_errors(motion):
    """Certify both timestamp positions and the exported spatial approximation."""
    ps,starts=motion.primitives,motion.starts
    if not ps:
        return [{"code":"MOTION_MODEL","message":"A moving route needs nonempty analytic primitives"}]
    first,last=float(motion.times[0]),float(motion.times[-1])
    if abs(starts[0]-first)>1e-7 or abs(starts[-1]+ps[-1].duration-last)>1e-7:
        return [{"code":"MOTION_MODEL","message":"Motion and WKT must cover the same time interval"}]
    for point,t in zip(motion.points,motion.times):
        i=max(0,min(len(ps)-1,int(np.searchsorted(starts,t,side="right")-1)))
        p=ps[i]
        if t<starts[i]-1e-7 or t>starts[i]+p.duration+1e-7 or np.linalg.norm(p.position(p.distance_at(t-starts[i]))-point)>.100001:
            return [{"code":"MOTION_MODEL","message":"A WKT vertex does not follow the analytic motion at its timestamp"}]
    for a,b,t0,t1 in zip(motion.points,motion.points[1:],motion.times,motion.times[1:]):
        begin=max(0,int(np.searchsorted(starts,t0,side="right")-1))
        for i in range(begin,len(ps)):
            p=ps[i];start=starts[i];end=start+p.duration
            if start>t1+1e-8:
                break
            if end<t0-1e-8:
                continue
            lo=max(t0,start)-start;hi=min(t1,end)-start
            x=p.position(p.distance_at(lo));y=p.position(p.distance_at(hi))
            bound=max(_segment_distance(x,a,b),_segment_distance(y,a,b))
            if p.kind=="arc" and p.length>1e-12:
                angle=abs(p.sweep)*(p.distance_at(hi)-p.distance_at(lo))/p.length
                bound+=p.radius*(1-math.cos(angle/2)) if angle<=math.pi else 2*p.radius
            if bound>.100001:
                return [{"code":"MOTION_MODEL","message":"WKT chords exceed the 0.1 m analytic spatial approximation bound"}]
    return []


def _waypoint_errors(motion, task, compiled):
    passages=_waypoint_passages(motion,task,compiled)
    if passages is None:
        return [{"code":"WAYPOINT","message":"Trajectory does not encounter every waypoint in task order"}]
    errors=[]
    for index,(waypoint,passed) in enumerate(zip(task.waypoints,passages)):
        if waypoint.earliest and passed<_seconds(waypoint.earliest,compiled.model)-1e-7:
            errors.append({"code":"TIME","message":"Waypoint precedes its earliest passage time","details":{"waypoint_index":index}})
        if waypoint.latest and passed>_seconds(waypoint.latest,compiled.model)+1e-7:
            errors.append({"code":"TIME","message":"Waypoint exceeds its latest passage time","details":{"waypoint_index":index}})
        duration=_hover_duration(waypoint,task,compiled)
        if duration<=0:
            continue
        point=_local(compiled,waypoint.wkt,task)
        finish=passed+duration
        if finish>motion.times[-1]+1e-7 or any(
            p.length>1e-9 or np.linalg.norm(p.start-point)>.01
            for p,t in zip(motion.primitives,motion.starts)
            if t<finish-1e-7 and t+p.duration>passed+1e-7):
            errors.append({"code":"HOVER","message":"Waypoint hover duration is not present in the analytic motion","details":{"waypoint_index":index}})
    return errors


def validate_route_v2(*,task,trajectory_wkt,vertex_times_s,motion_segments=None,network_route=None,scene_id=None,scene=None):
    from . import native
    if (scene_id is None)==(scene is None):
        raise PlanningError(ErrorCode.INVALID_INPUT,"Provide exactly one of scene_id or scene")
    if scene is not None:
        scene_id=register_scene(scene)["scene_id"]
    model,compiled,_,_=registry.get(scene_id)
    from .network import normalize_task
    task=normalize_task(compiled,_parse(Task,task));profile=_profile(model,task)
    if network_route is not None and task.network is None:
        raise PlanningError(ErrorCode.INVALID_INPUT,"network_route requires task.network")
    task,rtl=_rtl_task(compiled,task,profile)
    times=np.asarray(vertex_times_s,dtype=float)
    if times.ndim!=1 or not len(times) or not np.isfinite(times).all() or np.any(np.diff(times)<=0):
        raise PlanningError(ErrorCode.INVALID_INPUT,"Trajectory times must be finite and strictly increase")
    points=_timed_local(compiled,trajectory_wkt,times,task.alt_reference,model)
    errors=[];motion=None
    if motion_segments:
        try:
            motion=_reconstruct_motion(motion_segments,points,times)
            limits=_airway_motion_limits(compiled,task,_limits(compiled,task,profile),points)
            proof=native.validate_motion(motion.primitives,limits,starts=motion.starts)
            if not proof["valid"]:
                errors.append({"code":"MOTION_MODEL","message":"Analytic motion violates aircraft or rule limits","details":{"violations":proof["violations"]}})
            errors.extend(_motion_view_errors(motion))
        except (ValueError,TypeError,KeyError) as exc:
            errors.append({"code":"MOTION_MODEL","message":"Invalid analytic motion: "+str(exc)})
            motion=None
    elif len(points)>1:
        errors.append({"code":"MOTION_MODEL_REQUIRED","message":"Moving route validation requires motion_segments"})
    # A supplied law is checked against the WKT before its certified dense cover
    # is used for scene and continuous upstream separation checks.
    certified=motion is not None and not any(v["code"]=="MOTION_MODEL" for v in errors)
    try:
        checked_points,checked_times=motion.collision_samples() if certified else (points,times)
    except MotionError as exc:
        errors.append({"code":"MOTION_MODEL","message":str(exc)})
        certified=False
        checked_points,checked_times=points,times
    certified_profile=profile.model_copy(update={"safety_margin_m":profile.safety_margin_m+.1}) if certified else profile
    errors.extend(compiled.check_path(checked_points,checked_times,task,certified_profile,
        phase="rtl" if rtl and rtl["required"] else "route",
        speed_transient_segments=_transient_segments(motion,checked_times) if certified else [],
        slope_angles=_motion_slope_angles(motion,checked_times) if certified else None))
    if task.network:
        if network_route is not None:
            from .models import NetworkRoute
            network_route=_parse(NetworkRoute,network_route).model_dump()
        errors.extend(compiled.network.validate(task,checked_points,checked_times,network_route,motion=motion if certified else None))
    if times[0]<_seconds(task.departure_time,model)-1e-7:
        errors.append({"code":"TIME","message":"Trajectory departs before the task departure window"})
    if task.latest_arrival and times[-1]>_seconds(task.latest_arrival,model)+1e-7:
        errors.append({"code":"TIME","message":"Trajectory exceeds latest arrival"})
    if task.earliest_arrival and times[-1]<_seconds(task.earliest_arrival,model)-1e-7:
        errors.append({"code":"TIME","message":"Trajectory precedes earliest arrival"})
    if certified:
        errors.extend(_waypoint_errors(motion,task,compiled))
    elif len(points)==1:
        stationary=Motion(points,times,[],[])
        errors.extend(_waypoint_errors(stationary,task,compiled))
    for dynamic,(other,other_times) in zip(model.dynamic_obstacles,_dynamic_tracks(compiled,model)):
        h,v=compiled.rules.separation(task)
        h=max(h,profile.radius_m+profile.safety_margin_m+dynamic.horizontal_separation_m)+.1
        v=max(v,profile.height_m/2+profile.safety_margin_m+dynamic.vertical_separation_m)+.1
        pair=native.continuous_conflicts([checked_points,other],[checked_times,other_times],h,v)
        if pair["status"]!="ok":
            errors.append({"code":"VALIDATION_INCOMPLETE","message":"Continuous upstream separation validation did not complete", "details":{"obstacle_id":dynamic.id,"status":pair["status"]}})
        if pair["conflicts"]:
            errors.append({"code":"SEPARATION","message":"Trajectory intersects an upstream dynamic obstacle", "details":{"obstacle_id":dynamic.id}})
    if np.linalg.norm(points[0]-_local(compiled,task.start_wkt,task))>.1 or np.linalg.norm(points[-1]-_local(compiled,task.goal_wkt,task))>.1:
        errors.append({"code":"ENDPOINT","message":"Trajectory endpoints differ from task"})
    return {"valid":not errors,"violations":errors,"rule_report":_report(compiled,task,errors,checked_times,checked_points)}
