"""Finite lattice multi-label candidate generation and analytic objective integrals.

Labels use rebuilt motion costs and incoming lattice edges. Their dominance is a
candidate-generation heuristic, especially with calendars and variable speed.
The reported Pareto set is explicitly over independently validated candidates.
"""
from __future__ import annotations

import heapq
from decimal import Decimal
import itertools
import math
import time

import numpy as np
from shapely.geometry import Point

from .models import RouteObjectives, OptimizationReport
from .motion import MotionError, Primitive
from .rules import invalid
from .scene import _geometry_intervals


class ObjectiveCoverageError(MotionError):
    pass


def _finite_result(value, objective, operation):
    """Reject only an actually unrepresentable computed result, without caps."""
    try:
        result = float(value)
        finite = math.isfinite(result)
    except (OverflowError, ValueError):
        finite = False
    if not finite:
        invalid("Objective arithmetic produced a nonfinite result", objective=objective, operation=operation)
    return result


def datasets(compiled, config):
    selected = []
    if config.weights.risk:
        risks = compiled.datasets_for(None, "risk")
        if not risks:
            invalid("Positive risk weight requires versioned risk datasets")
        selected.append(("risk", risks, "risk", "time", "risk*s", config.normalization.risk_s, config.weights.risk))
    lookup = {d.id: d for d in compiled.datasets}
    for layer in config.cost_layers:
        ds = lookup.get(layer.dataset_id)
        if ds is None or ds.kind != "cost":
            invalid("Objective cost layer requires an existing cost dataset", dataset_id=layer.dataset_id)
        selected.append((layer.id, [ds], "cost", layer.basis, layer.unit, layer.normalization, layer.weight))
    return selected


def field_integral(motion, resources, key, basis, deadline=math.inf):
    """Split at every analytic footprint/height/time crossing, then integrate.

    Overlapping measurements use the maximum (conservative exposure), including
    across risk datasets. A nodata overlap is unknown, never a zero measurement.
    """
    from .engine import _arc_boundary_fractions
    total = 0.0
    primitives = list(zip(motion.primitives, motion.starts))
    if not primitives and len(motion.points):
        # A zero-duration point route still needs a known field value.
        point = np.asarray(motion.points[0])
        primitives = [(Primitive("hover",point,point,0,0,duration=float(motion.times[-1]-motion.times[0])),float(motion.times[0]))]
    for primitive, start in primitives:
        if time.perf_counter() >= deadline:
            raise TimeoutError("Deadline exhausted during objective integration")
        p = primitive
        fractions = {0.0, 1.0}
        def spatial(geometry):
            if p.kind == "arc":
                fractions.update(_arc_boundary_fractions(p, geometry))
            else:
                for a, b in _geometry_intervals(geometry, p.start, p.end):
                    fractions.update((a, b))
        def temporal(t):
            if start < t < start + p.duration:
                fractions.add(p.distance_at(t-start)/p.length if p.length else (t-start)/p.duration)
        for ds in resources:
            spatial(ds.geometry)
            temporal(ds.valid_from_s)
            temporal(ds.valid_to_s)
            for cell in ds.cells:
                spatial(cell.geometry)
                if abs(p.end[2]-p.start[2]) > 1e-12:
                    for z in (cell.min_altitude, cell.max_altitude):
                        f = (z-p.start[2])/(p.end[2]-p.start[2])
                        if 0 < f < 1:
                            fractions.add(f)
                temporal(cell.valid_from_s)
                temporal(cell.valid_to_s)
        def offset(f):
            return p.time_at(p.length*f) if p.length else p.duration*f
        def value(f):
            position = p.position(p.length*f)
            at = start + offset(f)
            found = []
            for ds in resources:
                if not ds.valid_from_s <= at <= ds.valid_to_s or not ds.geometry.covers(Point(position[:2])):
                    continue
                for cell in ds.cells:
                    if (cell.valid_from_s <= at <= cell.valid_to_s
                            and cell.min_altitude-1e-9 <= position[2] <= cell.max_altitude+1e-9
                            and cell.geometry.covers(Point(position[:2]))):
                        v = cell.values.get(key)
                        if cell.values.get("nodata") or isinstance(v, bool) or not isinstance(v, (int,float)) or not math.isfinite(v) or v < 0:
                            raise ObjectiveCoverageError(f"Unknown/nodata {key} objective field along motion")
                        found.append(float(v))
            if not found:
                raise ObjectiveCoverageError(f"Missing spatial/height/time coverage for {key} objective field")
            return max(found)
        # Include endpoints: a gap at departure/arrival is not a zero exposure.
        value(0.0)
        value(1.0)
        ordered = sorted(fractions)
        for a, b in zip(ordered, ordered[1:]):
            if b-a <= 1e-14:
                continue
            measure = p.length*(b-a) if basis == "distance" else offset(b)-offset(a)
            contribution = _finite_result(value((a+b)/2)*float(measure),key,"field integral product")
            total = _finite_result(total+contribution,key,"field integral sum")
    return total


def evaluate(compiled, config, motion, deadline=math.inf):
    weights = config.weights
    scales = config.normalization
    values = {}
    def add(name, raw, scale, unit, weight):
        raw = _finite_result(raw,name,"raw value")
        normalized = _finite_result(raw/scale,name,"normalization")
        values[name] = dict(raw=raw, normalized=normalized, normalization=scale, unit=unit, weight=weight)
    add("distance", motion.distance, scales.distance_m, "m", weights.distance)
    add("time", float(motion.times[-1]-motion.times[0]), scales.time_s, "s", weights.time)
    formula = None
    if config.energy_model is not None:
        energy = config.energy_model
        # Compute in Wh using decimal intermediates: finite Wh must not fail
        # because its Joule intermediate overflows, nor may zero climb produce
        # inf*0 when the caller supplied a very large finite mass.
        watt_hours = Decimal(0)
        for p in motion.primitives:
            power = energy.hover_power_w if p.length <= 1e-12 else energy.cruise_power_w
            watt_hours += Decimal(power)*Decimal(float(p.duration))/Decimal(3600)
            climb = max(0.,float(p.end[2]-p.start[2]))
            if climb:
                watt_hours += (Decimal(energy.mass_kg)*Decimal("9.80665")*Decimal(climb)
                               /Decimal(energy.climb_efficiency)/Decimal(3600))
        add("energy", _finite_result(watt_hours,"energy","Wh integration"), scales.energy_wh, "Wh", weights.energy)
        formula = "Wh = (cruise_power_w*moving_seconds + hover_power_w*stationary_seconds + mass_kg*9.80665*sum_positive_climb_m/climb_efficiency)/3600; no descent regeneration"
    versions = {}
    for name, resources, key, basis, unit, scale, weight in datasets(compiled, config):
        add(name, field_integral(motion, resources, key, basis, deadline), scale, unit, weight)
        versions.update({d.id:d.version for d in resources})
    score = 0.0
    for name,v in values.items():
        weighted = _finite_result(v["normalized"]*v["weight"],name,"weighted value")
        score = _finite_result(score+weighted,name,"weighted score sum")
    return RouteObjectives(values=values, weighted_score=score, dataset_versions=versions, energy_formula=formula).model_dump(mode="json")


def vector(objective):
    return tuple(v["normalized"] for v in objective["values"].values())


def dominates(a, b):
    return all(x <= y+1e-9 for x,y in zip(a,b)) and any(x < y-1e-9 for x,y in zip(a,b))


def frontier(candidates):
    return [c for c in candidates if not any(dominates(vector(d["objective"]), vector(c["objective"])) for d in candidates if d is not c)]


def paths(compiled, task, profile, limits, deadline, search_state, report):
    """Yield distinct complete paths from a finite neighbour lattice label search.

    Each leg is simple (no revisiting lattice vertices). No implicit label or
    solution limit is used. Exact endpoints connect to their nearest lattice
    vertex. Caller neighbours and blocked diagonal intermediary cells apply.
    """
    if search_state is not None:
        search_state["calls"] = search_state.get("calls", 0) + 1
    from .engine import _local, _seconds, _geometry_ok, _task_motion, _rtl_task
    config = task.objectives
    model = compiled.model
    b = model.bounds
    low = np.array([b.x_min,b.y_min,b.z_min])
    high = np.array([b.x_max,b.y_max,b.z_max])
    spacing = np.array(model.resolution)
    shape = np.floor((high-low)/spacing).astype(int)+1
    task, rtl = _rtl_task(compiled,task,profile)
    anchors = [_local(compiled,task.start_wkt,task), *[_local(compiled,w.wkt,task) for w in task.waypoints], _local(compiled,task.goal_wkt,task)]
    if rtl and rtl["required"]:
        a,b = anchors[0].copy(), anchors[-1].copy()
        lon,lat = compiled.frame._inverse.transform(float(a[0]),float(a[1]))
        a[2] = b[2] = compiled.frame.altitude_to_local(rtl["rtl_altitude_m"],"EG",lon,lat,task.departure_time)
        anchors = [anchors[0],a,b,anchors[-1]]
    departure = _seconds(task.departure_time, model)
    blocked = set(map(tuple, compiled.blocked_indices(task,profile,departure)))
    neighbors = task.search.neighbors if task.search else 26
    offsets = [o for o in itertools.product((-1,0,1), repeat=3) if any(o)
               and (neighbors == 26 or sum(abs(x) for x in o) <= (1 if neighbors == 6 else 2))]
    def cell(point):
        return tuple(np.clip(np.floor((point-low)/spacing+.5).astype(int),0,shape-1))
    def position(node):
        return low + np.array(node)*spacing
    def free(node):
        return node not in blocked and all(0<=n<s for n,s in zip(node,shape))
    def permitted(a, o):
        axes = [i for i,x in enumerate(o) if x]
        return all(free(tuple(a[i]+(o[i] if i in subset else 0) for i in range(3)))
                   for size in range(1,len(axes)+1) for subset in itertools.combinations(axes,size))
    sequence = itertools.count()
    heap = []
    labels = {}
    seen_complete = set()
    # A label holds the current lattice vertex, visited vertices, completed legs,
    # and exact current-leg geometry. Pruning keys retain incoming direction.
    def submit(stage, node, visited, completed, current, previous):
        if time.perf_counter() >= deadline:
            raise TimeoutError("Deadline exhausted during objective label search")
        prefix_task = task.model_copy(update={"waypoints": task.waypoints[:stage]})
        try:
            motion = _task_motion([*completed,np.asarray(current)], prefix_task, compiled, limits, departure)
            obj = evaluate(compiled, config, motion, deadline)
        except (MotionError, ValueError):
            return
        costs = vector(obj)
        state = (stage,node,previous)
        existing = labels.setdefault(state, [])
        if any(dominates(v,costs) or all(abs(x-y)<=1e-9 for x,y in zip(v,costs)) for v in existing):
            return
        existing[:] = [v for v in existing if not dominates(costs,v)]
        existing.append(costs)
        # ASTAR uses a geometric admissible lower bound for distance; DIJKSTRA
        # uses the prefix score. This changes ordering, never hard admissibility.
        heuristic = 0.0
        if not task.search or task.search.algorithm == "ASTAR":
            heuristic = _finite_result(Decimal(config.weights.distance)*Decimal(float(np.linalg.norm(anchors[stage+1]-current[-1])))/Decimal(config.normalization.distance_m),"distance","search heuristic")
        heapq.heappush(heap,(_finite_result(obj["weighted_score"]+heuristic,"score","search priority"),next(sequence),stage,node,visited,completed,current,costs))
    def begin(stage, completed):
        a = anchors[stage]
        node = cell(a)
        p = position(node)
        if free(node) and _geometry_ok(compiled,a,p,departure,task,profile):
            submit(stage,node,frozenset([node]),completed,[a] if np.linalg.norm(a-p)<1e-6 else [a,p],None)
    # Direct routes are useful in the unselected-grid default; explicit search
    # requests always traverse neighbour edges of their selected lattice.
    if task.search is None and not task.waypoints and not (rtl and rtl["required"]):
        if _geometry_ok(compiled,anchors[0],anchors[-1],departure,task,profile):
            seen_complete.add(tuple(tuple(np.round(p,8)) for p in anchors))
            yield [np.asarray(anchors)]
    begin(0, [])
    while heap:
        if config.max_solutions is not None and report["generated_candidates"] >= config.max_solutions:
            report["status"] = "solution_limit"
            break
        if time.perf_counter() >= deadline:
            report["status"] = "deadline_exceeded"
            break
        if config.max_iterations is not None and report["iterations"] >= config.max_iterations:
            report["status"] = "iteration_limit"
            break
        if search_state is not None and search_state["remaining"] is not None and search_state["remaining"] <= 0:
            report["status"] = "search_budget_exceeded"
            break
        _,_,stage,node,visited,completed,current,costs = heapq.heappop(heap)
        report["iterations"] += 1
        if search_state is not None:
            search_state["expanded"] += 1
            if search_state["remaining"] is not None:
                search_state["remaining"] -= 1
        target = anchors[stage+1]
        at = departure  # full motion is independently rechecked at actual times.
        if node == cell(target) and _geometry_ok(compiled,current[-1],target,at,task,profile):
            final = list(current)
            if np.linalg.norm(final[-1]-target)>1e-6:
                final.append(target)
            else:
                final[-1] = target
            legs = [*completed,np.asarray(final)]
            if stage+1 == len(anchors)-1:
                signature = tuple(tuple(np.round(p,8)) for leg in legs for p in leg)
                if signature not in seen_complete:
                    seen_complete.add(signature)
                    yield legs
            else:
                begin(stage+1,legs)
            continue
        for offset in offsets:
            nxt = tuple(a+d for a,d in zip(node,offset))
            if nxt in visited or not permitted(node,offset):
                continue
            p = position(nxt)
            if not _geometry_ok(compiled,current[-1],p,at,task,profile):
                continue
            submit(stage,nxt,visited|{nxt},completed,[*current,p],node)


def prepare(index, task, model, compiled, dynamic_tracks, deadline, search_state, *, retain_all_candidates=False):
    from .engine import (_prepare_candidate, _profile, _limits, _final_samples,
                         _transient_segments, _motion_slope_angles, _failure)
    config = task.objectives
    datasets(compiled, config)  # References are request errors, not empty scores.
    profile = _profile(model,task)
    limits = _limits(compiled,task,profile)
    report = OptimizationReport(status="generator_exhausted").model_dump(mode="json")
    candidates = []
    last_failure = None
    try:
        for legs in paths(compiled,task,profile,limits,deadline,search_state,report):
            if config.max_solutions is not None and report["generated_candidates"] >= config.max_solutions:
                report["status"] = "solution_limit"
                break
            report["generated_candidates"] += 1
            candidate, failure = _prepare_candidate(index,task,model,compiled,dynamic_tracks,deadline,search_state,
                                                     objective_legs=legs)
            if failure is not None:
                last_failure = failure
                continue
            # Calendar departure shifts are included before scoring. The global
            # batch scheduler can still move these candidates later.
            delay = max(0.,candidate["lo"]-float(candidate["times"][0]))
            motion = candidate["motion"].shifted(delay)
            fp,ft = _final_samples(compiled,model,motion)
            certificate_profile = profile.model_copy(update={"safety_margin_m":profile.safety_margin_m+.1})
            violations = compiled.check_path(fp,ft,task,certificate_profile,
                speed_transient_segments=_transient_segments(motion,ft),
                slope_angles=_motion_slope_angles(motion,ft))
            if violations:
                continue
            try:
                obj = evaluate(compiled,config,motion,deadline)
            except ObjectiveCoverageError as exc:
                last_failure = _failure(task,"OBJECTIVE_COVERAGE",str(exc))
                continue
            candidate["objective"] = obj
            candidate["objective_shift"] = delay
            candidates.append(candidate)
            report["validated_candidates"] += 1
    except TimeoutError:
        report["status"] = "deadline_exceeded"
    if not candidates:
        failure = last_failure or _failure(task,"SEARCH_NOT_FOUND","No independently valid objective candidate with complete field coverage")
        if report["status"] == "deadline_exceeded":
            failure = _failure(task,"TIME_BUDGET_EXCEEDED","Objective exploration exhausted request deadline")
        elif report["status"] in {"iteration_limit","search_budget_exceeded"}:
            failure = _failure(task,"SEARCH_BUDGET_EXCEEDED","Objective exploration exhausted caller iteration/search budget")
        failure["optimization"] = report
        return None, failure
    # Incremental handoff yaw/wait depends on each tail heading. Tail-only
    # dominance cannot eliminate candidates before that full motion is built.
    candidates = sorted(candidates if retain_all_candidates else frontier(candidates),
                        key=lambda c:(c["objective"]["weighted_score"],vector(c["objective"])))
    primary = candidates[0]
    primary["objective_candidates"] = candidates
    primary["optimization"] = report
    return primary,None


def attach_results(compiled, model, candidates, results, dynamic_tracks, deadline, horizontal, vertical):
    """Finalize each Pareto choice against all other current batch primaries.

    Replacement primaries undergo fixed-reservation checks. A final second pass
    filters alternatives against the resulting primaries, never just their own
    task. Alternatives are options, not simultaneous reservations/executions.
    """
    from . import native
    from .engine import (_native_deadline_ms, _final_samples, _transient_segments,
                         _motion_slope_angles, _result, _failure, _seconds)
    objective_candidates = [c for c in candidates if c["task"].objectives is not None]
    if not objective_candidates:
        return
    def obstacles(excluded):
        tracks = list(dynamic_tracks)
        for c in candidates:
            if c is excluded or results[c["index"]]["status"] != "planned":
                continue
            motion = c.get("accepted_motion")
            if motion is not None:
                tracks.append(motion.collision_samples())
        return tracks
    def safe(motion, tracks):
        points,times = motion.collision_samples()
        check = native.continuous_conflicts([p for p,t in tracks]+[points], [t for p,t in tracks]+[times],
            horizontal,vertical,deadline_ms=_native_deadline_ms(deadline))
        return check["status"] == "ok" and not any(c["second"] == len(tracks) or c["first"] == len(tracks) for c in check["conflicts"])
    for primary in objective_candidates:
        index = primary["index"]
        task = primary["task"]
        if results[index]["status"] != "planned":
            report = dict(primary["optimization"])
            if results[index].get("code") == "TIME_BUDGET_EXCEEDED":
                report["status"] = "deadline_exceeded"
            results[index]["optimization"] = report
            continue
        valid = []
        tracks = obstacles(primary)
        try:
            for candidate in primary["objective_candidates"]:
                if time.perf_counter() >= deadline:
                    raise TimeoutError("Deadline exhausted finalizing objective alternatives")
                if candidate is primary:
                    motion = primary["accepted_motion"]
                    delay = primary.get("accepted_delay",0.)
                else:
                    motion = None
                    for window in candidate["windows"]:
                        fixed_paths = [p for p,t in tracks]
                        fixed_times = [t for p,t in tracks]
                        scheduled = native.schedule_paths(fixed_paths+[candidate["points"]],fixed_times+[candidate["times"]],
                            [[float(t[0])]*2 for t in fixed_times]+[window],
                            horizontal_separation=horizontal,vertical_separation=vertical,fixed_count=len(tracks),
                            deadline_ms=_native_deadline_ms(deadline))
                        delay = scheduled["delays"][-1]
                        if delay is not None:
                            motion = candidate["motion"].shifted(delay)
                            break
                    if motion is None:
                        continue
                if not safe(motion,tracks):
                    continue
                fp,ft = _final_samples(compiled,model,motion)
                profile = candidate["profile"]
                certified_profile = profile.model_copy(update={"safety_margin_m":profile.safety_margin_m+.1})
                violations = compiled.check_path(fp,ft,task,certified_profile,
                    phase="rtl" if (compiled.rules.rtl_requirements(task,profile) or {}).get("required") else "route",
                    speed_transient_segments=_transient_segments(motion,ft),
                    slope_angles=_motion_slope_angles(motion,ft))
                if violations or ft[-1] > candidate["latest"]+1e-7:
                    continue
                if task.earliest_arrival and ft[-1] < _seconds(task.earliest_arrival,model)-1e-7:
                    continue
                try:
                    obj = evaluate(compiled,task.objectives,motion,deadline)
                except ObjectiveCoverageError:
                    continue
                result = _result(compiled,model,task,profile,motion,delay)
                result["objectives"] = obj
                valid.append(dict(objective=obj,result=result,motion=motion))
            # Search-grid paths with redundant collinear vertices can produce
            # identical analytic geometry; retain a single deterministic route.
            distinct = {}
            for item in valid:
                key = item["result"]["trajectory_wkt"]
                old = distinct.get(key)
                if old is None or item["objective"]["weighted_score"] < old["objective"]["weighted_score"]:
                    distinct[key] = item
            valid = sorted(frontier(list(distinct.values())),
                           key=lambda c:(c["objective"]["weighted_score"],vector(c["objective"]),c["result"]["trajectory_wkt"]))
            if not valid:
                results[index] = _failure(task,"OBJECTIVE_COVERAGE","No final objective candidate with complete coverage and batch safety")
                results[index]["optimization"] = primary["optimization"]
                continue
            selected = valid[0]
            primary["accepted_motion"] = selected["motion"]
            primary["final_objective_alternatives"] = valid[1:]
            results[index] = selected["result"]
            results[index]["alternatives"] = [a["result"] for a in valid[1:]]
            results[index]["optimization"] = primary["optimization"]
        except TimeoutError:
            results[index] = _failure(task,"TIME_BUDGET_EXCEEDED","Deadline exhausted validating objective alternatives")
            results[index]["optimization"] = {**primary["optimization"],"status":"deadline_exceeded"}
    for primary in objective_candidates:
        index = primary["index"]
        if results[index]["status"] != "planned":
            continue
        try:
            tracks = obstacles(primary)
            alternatives = [a["result"] for a in primary.get("final_objective_alternatives",[]) if safe(a["motion"],tracks)]
            results[index]["alternatives"] = alternatives
            results[index]["optimization"]["returned_solutions"] = 1+len(alternatives)
        except TimeoutError:
            results[index] = _failure(primary["task"],"TIME_BUDGET_EXCEEDED","Deadline exhausted in final alternative pair validation")
            results[index]["optimization"] = {**primary["optimization"],"status":"deadline_exceeded"}
