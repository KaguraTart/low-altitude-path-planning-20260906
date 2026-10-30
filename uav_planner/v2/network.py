"""Explicit airway topology, endpoint normalization and structural connectivity."""
from __future__ import annotations

import heapq
import math
import time

import numpy as np
from shapely.geometry import Point

from uav_planner.errors import ErrorCode, PlanningError


def invalid(message):
    raise PlanningError(ErrorCode.INVALID_INPUT, message)


class Network:
    def __init__(self, compiled):
        self.compiled = compiled
        self.graph = compiled.model.airway_graph
        self.nodes = {n.id: np.asarray(compiled.frame.to_local(n.point_wkt, "EG", compiled.time_origin).coords[0]) for n in self.graph.nodes}
        self.edges = {e.id: e for e in self.graph.edges}
        self.airways = {a.id: a for a in compiled.model.airways}
        self.prepared_airways = {a.id: a for a in compiled.airways}
        self.lines = {}
        for edge in self.graph.edges:
            points = (np.asarray(compiled.frame.to_local(edge.geometry_wkt, "EG", compiled.time_origin).coords) if edge.geometry_wkt else np.asarray([self.nodes[edge.from_node], self.nodes[edge.to_node]]))
            if np.linalg.norm(points[0] - self.nodes[edge.from_node]) > .1 or np.linalg.norm(points[-1] - self.nodes[edge.to_node]) > .1:
                invalid(f"Graph edge {edge.id} geometry endpoints must match nodes within 0.1m")
            # Canonical nodes avoid tolerance gaps between adjacent edge geometries.
            points[0], points[-1] = self.nodes[edge.from_node], self.nodes[edge.to_node]
            self.lines[edge.id] = points

    def active(self, edge, start, end=None):
        end = start if end is None else end
        for item in (edge, self.airways[edge.airway_id]):
            if item.status != "ACTIVE":
                return False
            if item.valid_from and start < (item.valid_from-self.compiled.time_origin).total_seconds()-1e-7:
                return False
            if item.valid_to and end > (item.valid_to-self.compiled.time_origin).total_seconds()+1e-7:
                return False
        return True

    def arcs(self, at):
        for edge in sorted(self.graph.edges, key=lambda e: e.id):
            if not self.active(edge, at):
                continue
            if edge.direction != "REVERSE":
                yield edge.from_node, edge.to_node, edge, self.lines[edge.id]
            if edge.direction != "FORWARD":
                yield edge.to_node, edge.from_node, edge, self.lines[edge.id][::-1]

    def edge_geometry_allowed(self, edge, points, profile, at, deadline=math.inf):
        from .scene import _covers
        airway = self.prepared_airways[edge.airway_id]
        margin, vertical = self.compiled.clearance(profile)
        for a,b in zip(points,points[1:]):
            if time.perf_counter() >= deadline:
                raise TimeoutError("Network admissibility deadline exhausted")
            if airway.direction != "BOTH":
                start = airway.centerline.project(Point(a[:2]))
                end = airway.centerline.project(Point(b[:2]))
                if (airway.direction == "FORWARD" and end < start-1e-7) or (airway.direction == "REVERSE" and end > start+1e-7):
                    return False
            if not _covers(self.compiled.volume_intervals(airway,a,b,at,at,horizontal=margin,vertical=vertical,inside=True)):
                return False
        return True

    def weighted_cost(self, edge, points, task, profile, at, deadline=math.inf):
        """Integrate soft priority across exact scope and field transitions."""
        selected = [r for r in self.compiled.rules.selected(task) if r.rule_type == "PRIORITY"]
        raw = edge.weights[task.network.weight_field]
        if not selected or raw == 0:
            return raw
        weighted = 0.; length = 0.
        for a,b in zip(points,points[1:]):
            if time.perf_counter() >= deadline:
                raise TimeoutError("Network priority integration deadline exhausted")
            distance = float(np.linalg.norm(b-a))
            cuts = {0.,1.}
            for rule in selected:
                if time.perf_counter() >= deadline:
                    raise TimeoutError("Network priority integration deadline exhausted")
                for low,high in self.compiled.rule_parts(rule,a,b,at,at):
                    cuts.update((low,high))
                if rule.content['priorityScene'] in {'QUIET','LOW_RISK'}:
                    kind = 'noise' if rule.content['priorityScene'] == 'QUIET' else 'risk'
                    for low,high,_ in self.compiled._field_parts(rule,kind,a,b,at,at):
                        cuts.update((low,high))
            ordered = sorted(cuts)
            multiplier = sum((high-low)*self.compiled.cost_multiplier(a+(b-a)*((low+high)/2),at,task,profile) for low,high in zip(ordered,ordered[1:]) if high>low)
            weighted += distance*multiplier; length += distance
        multiplier = weighted/length if length>1e-12 else self.compiled.cost_multiplier(points[0],at,task,profile)
        return raw*multiplier

    def edge_intervals(self,points,times,node_ids,motion):
        starts = [float(times[0])]; progress = 0.
        for node in node_ids[1:-1]:
            distances,positions,fractions = _projections(self.nodes[node],points)
            distances = np.where(positions>=progress-1e-6,distances,math.inf)
            i = int(np.argmin(distances)); progress = float(positions[i])
            passed = float(times[0]) if len(points)==1 else float(times[i]+fractions[i]*(times[i+1]-times[i]))
            starts.append(passed)
        starts.append(float(times[-1]))
        edge_starts,edge_ends = starts[:-1],starts[1:]
        if len(node_ids)==1:
            return [],[]
        for index,node in enumerate(node_ids[1:-1]):
            position = self.nodes[node]
            for primitive,begin in zip(motion.primitives,motion.starts):
                if primitive.kind!='arc' or not begin-1e-7<=starts[index+1]<=begin+primitive.duration+1e-7:
                    continue
                incoming = primitive.start[:2]-primitive.center[:2]
                outgoing = primitive.end[:2]-primitive.center[:2]
                incoming = np.array([-incoming[1],incoming[0]])*np.sign(primitive.sweep)
                outgoing = np.array([-outgoing[1],outgoing[0]])*np.sign(primitive.sweep)
                parameters = np.linalg.lstsq(np.column_stack([incoming,outgoing]),primitive.end[:2]-primitive.start[:2],rcond=None)[0]
                corner = primitive.start[:2]+parameters[0]*incoming
                if np.linalg.norm(corner-position[:2])<=.1:
                    edge_ends[index] = max(edge_ends[index],begin+primitive.duration)
                    edge_starts[index+1] = min(edge_starts[index+1],begin)
        return edge_starts,edge_ends

    def departure_windows(self,motion,trace,windows):
        """Clip reservation launch windows by every edge's real occupancy."""
        starts,ends = self.edge_intervals(motion.points,motion.times,trace['node_ids'],motion)
        launch = float(motion.times[0]); minimum = -math.inf; maximum = math.inf
        for edge_id,start,end in zip(trace['edge_ids'],starts,ends):
            edge = self.edges[edge_id]
            for resource in (edge,self.airways[edge.airway_id]):
                if resource.valid_from:
                    minimum = max(minimum,launch+(resource.valid_from-self.compiled.time_origin).total_seconds()-start)
                if resource.valid_to:
                    maximum = min(maximum,launch+(resource.valid_to-self.compiled.time_origin).total_seconds()-end)
        return [(max(a,minimum),min(b,maximum)) for a,b in windows if max(a,minimum)<=min(b,maximum)+1e-7]

    def candidates(self,task,profile,departure,deadline):
        """Yield weighted, locally flyable expanded-state paths lazily.

        Local geometry is finite-state. Whole-route resource/timing constraints
        remain independent certificates and can request the next weighted path.
        """
        choices = _GraphChoices(self,task,profile,departure,deadline)
        yield from choices.candidates()

    def select(self,task,profile,departure,deadline):
        return next(self.candidates(task,profile,departure,deadline),None)

    def validate(self, task, points, times, trace=None, motion=None):
        """Independently reconstruct the selected walk's flyable geometry.

        Analytic corner arcs may round a graph junction. They must follow the
        canonical connected walk in order; an intersection creates no new edge.
        Normal motion and airway checks separately certify speed and clearance.
        """
        from .engine import _profile, _limits, _airway_motion_limits, _task_motion
        from .motion import MotionError, build_motion
        error = [{"code": "NETWORK_ROUTE", "message": "Trajectory does not follow connected, directed, active graph edges over its actual times"}]
        profile = _profile(self.compiled.model, task)
        departure = (task.departure_time-self.compiled.time_origin).total_seconds()
        def walk(edge_ids, node_ids):
            if len(node_ids) != len(edge_ids)+1 or node_ids[0] != task.network.start_node_id or node_ids[-1] != task.network.goal_node_id:
                return None
            via = 0; wanted = task.network.via_node_ids
            legs = []; cost = 0.; effective_cost = 0.
            for i, edge_id in enumerate(edge_ids):
                edge = self.edges.get(edge_id)
                if edge is None or task.network.weight_field not in edge.weights:
                    return None
                a,b = node_ids[i:i+2]
                if (a,b) == (edge.from_node,edge.to_node) and edge.direction != "REVERSE":
                    line = self.lines[edge.id]
                elif (a,b) == (edge.to_node,edge.from_node) and edge.direction != "FORWARD":
                    line = self.lines[edge.id][::-1]
                else:
                    return None
                if self.airways[edge.airway_id].allowed_profile_ids and profile.id not in self.airways[edge.airway_id].allowed_profile_ids:
                    return None
                legs.append(line); cost += edge.weights[task.network.weight_field]
                effective_cost += self.weighted_cost(edge,line,task,profile,departure)
            for node in node_ids:
                while via < len(wanted) and node == wanted[via]:
                    via += 1
            if via != len(wanted):
                return None
            return legs or [np.asarray([self.nodes[node_ids[0]]])], {"network_version":self.graph.version,"node_ids":node_ids,"edge_ids":edge_ids,"weight_field":task.network.weight_field,"total_cost":cost,"effective_cost":effective_cost}

        def projections(point, line):
            if len(line) == 1:
                return np.array([np.linalg.norm(point-line[0])]), np.array([0.]), np.array([0.])
            delta = np.diff(line, axis=0)
            lengths = np.linalg.norm(delta, axis=1)
            den = np.sum(delta*delta, axis=1)
            fraction = np.clip(np.sum((point-line[:-1])*delta, axis=1)/np.maximum(den,1e-30),0,1)
            distances = np.linalg.norm(point-(line[:-1]+fraction[:,None]*delta), axis=1)
            offsets = np.r_[0.,np.cumsum(lengths)]
            return distances, offsets[:-1]+fraction*lengths, fraction

        def motion_limits(legs):
            limits = _airway_motion_limits(self.compiled,task,_limits(self.compiled,task,profile),np.concatenate(legs))
            if motion is not None:
                radii = [p.radius for p in motion.primitives if p.kind == "arc"]
                if radii and max(radii)-min(radii) <= 1e-6:
                    limits["min_turn_radius_m"] = max(limits["min_turn_radius_m"],min(radii))
            return limits

        edge_motion_cache = {}
        def edge_motion_allowed(edge_id,node,line):
            key = edge_id,node
            if key not in edge_motion_cache:
                try:
                    from . import native
                    limits = motion_limits([line])
                    candidate = build_motion(line,limits,departure)
                    edge_motion_cache[key] = bool(native.validate_motion(candidate.primitives,limits,starts=candidate.starts)["valid"])
                except (ValueError,MotionError):
                    edge_motion_cache[key] = False
            return edge_motion_cache[key]

        def certify(legs, candidate):
            try:
                limits = motion_limits(legs)
                expected = _task_motion(legs, task, self.compiled, limits, departure)
                reference, _ = expected.collision_samples(error=.01)
            except (ValueError, MotionError):
                return False
            # Both directions matter: sparse chords cannot cut across turns.
            progress = -math.inf
            for point in points:
                distances, positions, _ = projections(point, reference)
                candidates = positions[(distances <= .21) & (positions >= progress-1e-6)]
                if not len(candidates):
                    return False
                progress = float(min(candidates))
            progress = -math.inf
            for point in reference:
                distances, positions, _ = projections(point, points)
                candidates = positions[(distances <= .21) & (positions >= progress-1e-6)]
                if not len(candidates):
                    return False
                progress = float(min(candidates))
            edge_starts,edge_ends = self.edge_intervals(points,times,candidate["node_ids"],motion if motion is not None else expected)
            if not all(self.active(self.edges[e],a,b) for e,a,b in zip(candidate["edge_ids"],edge_starts,edge_ends)):
                return False
            for edge_id,line,start in zip(candidate["edge_ids"],legs,edge_starts):
                if not self.edge_geometry_allowed(self.edges[edge_id],line,profile,start):
                    return False
            # Select corridor authorization by actual logical-edge time.
            # Shared junction arcs use exactly their two incident edges; an
            # unrelated earlier/later selected edge cannot mask a missing turn.
            from .scene import _covers
            margin, vertical = self.compiled.clearance(profile)
            margin += .1; vertical += .1
            boundaries = [*edge_starts,*edge_ends]
            for a,b,t0,t1 in zip(points,points[1:],times,times[1:]):
                cuts = sorted({float(t0),float(t1),*(float(t) for t in boundaries if t0<t<t1)})
                for low,high in zip(cuts,cuts[1:]):
                    start = a+(b-a)*((low-t0)/(t1-t0))
                    stop = a+(b-a)*((high-t0)/(t1-t0))
                    airway_ids = {self.edges[e].airway_id for e,begin,end in zip(candidate["edge_ids"],edge_starts,edge_ends) if begin<=low+1e-7 and end>=high-1e-7}
                    covered = []
                    for airway_id in airway_ids:
                        airway = self.prepared_airways[airway_id]
                        if airway.direction != "BOTH":
                            begin = airway.centerline.project(Point(start[:2]))
                            end = airway.centerline.project(Point(stop[:2]))
                            if (airway.direction == "FORWARD" and end < begin-1e-7) or (airway.direction == "REVERSE" and end > begin+1e-7):
                                continue
                        covered.extend(self.compiled.volume_intervals(airway,start,stop,low,high,horizontal=margin,vertical=vertical,inside=True))
                    if not _covers(covered):
                        return False
            return True

        if trace is not None:
            selected = walk(trace["edge_ids"],trace["node_ids"])
            if selected is None:
                return error
            expected_trace = selected[1].copy()
            supplied = trace.copy()
            if supplied.get("effective_cost") is None:
                supplied.pop("effective_cost",None); expected_trace.pop("effective_cost",None)
            if "effective_cost" in supplied:
                if not math.isclose(expected_trace.pop("effective_cost"), supplied.pop("effective_cost"),rel_tol=1e-12,abs_tol=1e-12):
                    return error
            if expected_trace != supplied:
                return error
            return [] if certify(*selected) else error
        # The optimizer's walk is a fast common case, not a validation mandate.
        selected = self.select(task, profile, departure, math.inf)
        if selected is not None and certify(*selected):
            return []
        # Recover the raw line/corner skeleton from the certified flight law.
        # A remote nearest projection is not a logical node passage: only a
        # flown line or a real arc's virtual tangent intersection can be one.
        skeleton = [np.asarray(points[0])]
        skeleton_bounds = [(float(times[0]),float(times[0]))]
        if motion is not None:
            for index, primitive in enumerate(motion.primitives):
                if primitive.kind == "arc":
                    incoming = primitive.start[:2]-primitive.center[:2]
                    outgoing = primitive.end[:2]-primitive.center[:2]
                    incoming = np.array([-incoming[1],incoming[0]]) * np.sign(primitive.sweep)
                    outgoing = np.array([-outgoing[1],outgoing[0]]) * np.sign(primitive.sweep)
                    parameters = np.linalg.lstsq(np.column_stack([incoming,outgoing]),primitive.end[:2]-primitive.start[:2],rcond=None)[0]
                    corner_xy = primitive.start[:2]+parameters[0]*incoming
                    z = (primitive.start[2]+primitive.end[2])/2
                    # A neighboring straight tangent recovers the original
                    # corner's altitude even for a climbing rounded junction.
                    if index:
                        previous = motion.primitives[index-1]
                        horizontal = float(np.linalg.norm((previous.end-previous.start)[:2]))
                        if previous.kind != "arc" and horizontal > 1e-8:
                            z = primitive.start[2]+float(np.linalg.norm(corner_xy-primitive.start[:2]))*(previous.end[2]-previous.start[2])/horizontal
                    skeleton.append(np.array([*corner_xy,z]))
                    skeleton_bounds.append((motion.starts[index],motion.starts[index]+primitive.duration))
                skeleton.append(primitive.end)
                end_time = motion.starts[index]+primitive.duration
                skeleton_bounds.append((end_time,end_time))
        else:
            skeleton.extend(points[1:])
            skeleton_bounds.extend((float(t),float(t)) for t in times[1:])
        cleaned = [skeleton[0]]; cleaned_bounds = [skeleton_bounds[0]]
        for point,bounds in zip(skeleton[1:],skeleton_bounds[1:]):
            if np.linalg.norm(point-cleaned[-1]) > 1e-8:
                cleaned.append(point); cleaned_bounds.append(bounds)
            else:
                cleaned_bounds[-1] = (min(cleaned_bounds[-1][0],bounds[0]),max(cleaned_bounds[-1][1],bounds[1]))
        skeleton = np.asarray(cleaned)
        offsets = np.r_[0.,np.cumsum(np.linalg.norm(np.diff(skeleton,axis=0),axis=1))]
        total = float(offsets[-1])
        events = {}
        for node, point in self.nodes.items():
            distances, positions, _ = projections(point,skeleton)
            events[node] = sorted(set(float(position) for distance,position in zip(distances,positions) if distance <= .21))
        events[task.network.start_node_id] = sorted(set([0.,*events[task.network.start_node_id]]))
        events[task.network.goal_node_id] = sorted(set([total,*events[task.network.goal_node_id]]))

        def interval(start,end):
            if len(skeleton) == 1:
                return skeleton
            def at(position):
                i = min(len(skeleton)-2,max(0,int(np.searchsorted(offsets,position,side="right")-1)))
                fraction = (position-offsets[i])/max(offsets[i+1]-offsets[i],1e-30)
                return skeleton[i]+fraction*(skeleton[i+1]-skeleton[i])
            return np.asarray([at(start),*skeleton[(offsets>start+1e-8)&(offsets<end-1e-8)],at(end)])

        def passage_bounds(position):
            closest = int(np.argmin(np.abs(offsets-position)))
            if abs(offsets[closest]-position) <= .21:
                return cleaned_bounds[closest]
            i = min(len(skeleton)-2,max(0,int(np.searchsorted(offsets,position,side="right")-1)))
            fraction = (position-offsets[i])/max(offsets[i+1]-offsets[i],1e-30)
            point = skeleton[i]+fraction*(skeleton[i+1]-skeleton[i])
            if motion is not None:
                from .engine import _waypoint_time
                actual = _waypoint_time(motion,point,not_before=cleaned_bounds[i][0]-1e-7)
                return None if actual is None else (actual,actual)
            actual = cleaned_bounds[i][1]+fraction*(cleaned_bounds[i+1][0]-cleaned_bounds[i][1])
            return actual,actual

        def ordered_match(first,second):
            progress = -math.inf
            for point in first:
                distances,positions,_ = projections(point,second)
                candidates = positions[(distances<=.21)&(positions>=progress-1e-6)]
                if not len(candidates):
                    return False
                progress = float(min(candidates))
            return True

        transition_cache = {}
        transition_samples = None
        def transition_allowed(previous_id,current_id,position):
            nonlocal transition_samples
            if previous_id is None or motion is None:
                return True
            key = previous_id,current_id,position
            if key in transition_cache:
                return transition_cache[key]
            bounds = passage_bounds(position)
            if bounds is None:
                return False
            begin,end = bounds
            if end <= begin+1e-7:
                return True
            # Certify the settled physical junction before histories can merge.
            # A later edge cannot rescue a turn unsupported by its two incident
            # corridors, and an invalid narrow prefix cannot suppress a broad one.
            if transition_samples is None:
                transition_samples = motion.collision_samples()
            actual,actual_times = transition_samples
            airway_ids = {self.edges[e].airway_id for e in (previous_id,current_id)}
            margin,vertical = self.compiled.clearance(profile)
            margin += .1; vertical += .1
            from .scene import _covers
            for a,b,t0,t1 in zip(actual,actual[1:],actual_times,actual_times[1:]):
                if t1 <= begin+1e-8 or t0 >= end-1e-8:
                    continue
                low,high = max(float(t0),begin),min(float(t1),end)
                start = a+(b-a)*((low-t0)/(t1-t0))
                stop = a+(b-a)*((high-t0)/(t1-t0))
                covered = []
                for airway_id in airway_ids:
                    airway = self.prepared_airways[airway_id]
                    if airway.direction != "BOTH":
                        p = airway.centerline.project(Point(start[:2]))
                        q = airway.centerline.project(Point(stop[:2]))
                        if (airway.direction == "FORWARD" and q < p-1e-7) or (airway.direction == "REVERSE" and q > p+1e-7):
                            continue
                    covered.extend(self.compiled.volume_intervals(airway,start,stop,low,high,horizontal=margin,vertical=vertical,inside=True))
                if not _covers(covered):
                    transition_cache[key] = False
                    return False
            transition_cache[key] = True
            return True

        adjacency = {n:[] for n in self.nodes}
        for edge in self.graph.edges:
            if edge.direction != "REVERSE": adjacency[edge.from_node].append((edge.to_node,edge.id,self.lines[edge.id]))
            if edge.direction != "FORWARD": adjacency[edge.to_node].append((edge.from_node,edge.id,self.lines[edge.id][::-1]))
        wanted = task.network.via_node_ids
        initial_via = 0
        while initial_via < len(wanted) and wanted[initial_via] == task.network.start_node_id:
            initial_via += 1
        stack = [(task.network.start_node_id,0.,initial_via,[task.network.start_node_id],[])]
        visited = set(); compatibility = {}
        while stack:
            node, position, via, node_ids, edge_ids = stack.pop()
            state = (node,position,via,edge_ids[-1] if edge_ids else None)
            if state in visited:
                continue
            visited.add(state)
            if node == task.network.goal_node_id and abs(position-total)<1e-6:
                candidate = walk(edge_ids,node_ids)
                if via == len(wanted) and candidate is not None and certify(*candidate):
                    return []
                continue
            for other, edge_id, line in adjacency[node]:
                for next_position in events[other]:
                    if next_position <= position+1e-6:
                        continue
                    key = (edge_id,node,position,next_position)
                    if key not in compatibility:
                        portion = interval(position,next_position)
                        edge = self.edges[edge_id]
                        start_bounds = passage_bounds(position)
                        end_bounds = passage_bounds(next_position)
                        allowed_profiles = self.airways[edge.airway_id].allowed_profile_ids
                        compatibility[key] = (
                            ordered_match(line,portion) and ordered_match(portion,line)
                            and start_bounds is not None and end_bounds is not None
                            and (not allowed_profiles or profile.id in allowed_profiles)
                            and self.active(edge,start_bounds[0],end_bounds[1])
                            and self.edge_geometry_allowed(edge,line,profile,start_bounds[0])
                            and edge_motion_allowed(edge_id,node,line)
                        )
                    if not compatibility[key]:
                        continue
                    if not transition_allowed(edge_ids[-1] if edge_ids else None,edge_id,position):
                        continue
                    next_via = via
                    while next_via < len(wanted) and other == wanted[next_via]:
                        next_via += 1
                    stack.append((other,next_position,next_via,[*node_ids,other],[*edge_ids,edge_id]))
        return error

    def analyze(self, at):
        arcs = list(self.arcs(at)); names = sorted(self.nodes)
        forward = {n: set() for n in names}; backward = {n: set() for n in names}
        for a, b, _, _ in arcs:
            forward[a].add(b); backward[b].add(a)
        def reach(start, adjacency):
            seen = {start}; stack = [start]
            while stack:
                node = stack.pop()
                for other in adjacency[node]-seen:
                    seen.add(other); stack.append(other)
            return seen
        def components(strong):
            if not strong:
                remaining = set(names); result = []
                undirected = {n: forward[n] | backward[n] for n in names}
                while remaining:
                    group = reach(min(remaining), undirected)
                    result.append(sorted(group)); remaining -= group
                return sorted(result)
            # Iterative Kosaraju: linear in nodes+arcs and independent of the
            # interpreter recursion limit for production-sized networks.
            visited = set(); finish = []
            for node in names:
                if node in visited:
                    continue
                visited.add(node); stack = [(node, iter(sorted(forward[node])))]
                while stack:
                    current, children = stack[-1]
                    other = next(children, None)
                    if other is None:
                        finish.append(current); stack.pop()
                    elif other not in visited:
                        visited.add(other); stack.append((other, iter(sorted(forward[other]))))
            assigned = set(); result = []
            for node in reversed(finish):
                if node in assigned:
                    continue
                group = set(); stack = [node]; assigned.add(node)
                while stack:
                    current = stack.pop(); group.add(current)
                    for other in backward[current]-assigned:
                        assigned.add(other); stack.append(other)
                result.append(sorted(group))
            return sorted(result)
        active = sorted({e.id for _, _, e, _ in arcs})
        return {"version": self.graph.version, "nodes": names, "active_edges": active,
                "weak_components": components(False), "strong_components": components(True),
                "inactive_edge_ids": sorted(set(self.edges)-set(active))}


def normalize_task(compiled, task):
    updates = {}
    if task.search and task.search.grid_level is not None and (compiled.model.grid is None or compiled.model.grid.level != task.search.grid_level):
        invalid("search.grid_level must match scene.grid.level")
    if task.network:
        network = getattr(compiled, "network", None)
        if network is None:
            invalid("Task network selection requires scene.airway_graph")
        if task.network.expected_version is not None and task.network.expected_version != network.graph.version:
            invalid("Task network expected_version does not match graph version")
        for node in [task.network.start_node_id, *task.network.via_node_ids, task.network.goal_node_id]:
            if node not in network.nodes:
                invalid(f"Unknown network node: {node}")
    for label in ("start", "goal"):
        explicit = getattr(task, label+"_wkt")
        index = getattr(task, label+"_grid")
        point = None
        if task.network:
            point = compiled.network.nodes[getattr(task.network, label+"_node_id")]
        elif index is not None:
            grid = compiled.model.grid
            if grid is None or grid.crs != "LOCAL_AEQD":
                invalid("Grid endpoint requires external scene.grid in LOCAL_AEQD")
            if any(i < 0 or i >= n for i, n in zip(index, grid.shape)):
                invalid(f"{label}_grid index is out of bounds")
            low = np.asarray(grid.origin)+np.asarray(index)*np.asarray(grid.resolution)
            point = low+np.asarray(grid.resolution)/2
            if explicit:
                actual = np.asarray(compiled.frame.to_local(explicit, task.alt_reference, task.departure_time).coords[0])
                q = (actual-np.asarray(grid.origin))/np.asarray(grid.resolution)
                nearest = np.rint(q)
                cell = np.floor(np.where(np.abs(q-nearest)<=1e-7,nearest,q)).astype(int)
                if tuple(cell) != tuple(index):
                    invalid(f"{label}_wkt contradicts {label}_grid")
        if point is not None:
            if explicit and task.network:
                actual = np.asarray(compiled.frame.to_local(explicit, task.alt_reference, task.departure_time).coords[0])
                if np.linalg.norm(point-actual) > .1:
                    invalid(f"{label}_wkt contradicts network node")
            if explicit is None:
                updates[label+"_wkt"] = compiled.frame.to_wgs84(Point(point), task.alt_reference, task.departure_time)
    return task.model_copy(update=updates)


def analyze_network_v2(*, scene_id=None, scene=None, at=None):
    """Return structural connectivity at an explicit instant in the scene."""
    from pydantic import TypeAdapter, ValidationError
    from .engine import register_scene, registry
    from .models import Timestamp
    if (scene_id is None) == (scene is None):
        invalid("Provide exactly one of scene_id or scene")
    if scene is not None:
        scene_id = register_scene(scene)["scene_id"]
    model, compiled, _, _ = registry.get(scene_id)
    if model.airway_graph is None:
        invalid("Scene has no airway_graph")
    try:
        instant = TypeAdapter(Timestamp).validate_python(at) if at is not None else model.time_origin
    except ValidationError as exc:
        invalid("Network analysis at must be a timezone-aware timestamp: " + str(exc.errors()[0]["msg"]))
    return compiled.network.analyze((instant-model.time_origin).total_seconds())


class _GraphChoices:
    """Task-local finite geometric states and lazy loopless weighted deviations."""
    envelope_keys = ('min_turn_radius_m','max_turn_angle_deg','climb_speed_mps','descent_speed_mps')

    def __init__(self,network,task,profile,departure,deadline):
        from .engine import _geometry_ok,_limits,_airway_motion_limits
        self.network,self.compiled,self.task,self.profile = network,network.compiled,task,profile
        self.departure,self.deadline = departure,deadline
        self.base = _limits(self.compiled,task,profile)
        self.base.setdefault('max_turn_angle_deg',180.)
        self.base_env = self.env(self.base)
        self.arcs = []; self.adjacency = {n:[] for n in network.nodes}
        self.cert_profile = profile.model_copy(update={'safety_margin_m':profile.safety_margin_m+.1})
        for edge in network.graph.edges:
            self.check_deadline()
            if task.network.weight_field not in edge.weights:
                invalid(f'Missing weight field {task.network.weight_field} on edge {edge.id}')
        for start,end,edge,line in network.arcs(departure):
            self.check_deadline()
            if network.airways[edge.airway_id].allowed_profile_ids and profile.id not in network.airways[edge.airway_id].allowed_profile_ids:
                continue
            if not network.edge_geometry_allowed(edge,line,profile,departure,deadline):
                continue
            allowed = True
            for a,b in zip(line,line[1:]):
                self.check_deadline()
                if not _geometry_ok(self.compiled,a,b,departure,task,profile):
                    allowed = False; break
            if not allowed:
                continue
            cost = network.weighted_cost(edge,line,task,profile,departure,deadline)
            if not math.isfinite(cost):
                continue
            envelope = self.env(_airway_motion_limits(self.compiled,task,self.base,line))
            index = len(self.arcs)
            self.arcs.append((start,end,edge,line,cost,envelope))
            self.adjacency[start].append(index)
        self.cache = {}

    def check_deadline(self):
        if time.perf_counter() >= self.deadline:
            raise TimeoutError('Network feasible-alternative deadline exhausted')

    def env(self,limits):
        return tuple(float(limits.get(key,180. if key=='max_turn_angle_deg' else self.base[key])) for key in self.envelope_keys)

    @staticmethod
    def combine(first,second):
        return (max(first[0],second[0]),*(min(a,b) for a,b in zip(first[1:],second[1:])))

    def limits(self,envelope):
        return {**self.base,**dict(zip(self.envelope_keys,envelope))}

    def advance_via(self,node,via):
        wanted = self.task.network.via_node_ids
        while via<len(wanted) and wanted[via]==node:
            via += 1
        return via

    @staticmethod
    def clean_tagged(values):
        result = []
        for point,tag in values:
            if result and np.linalg.norm(point-result[-1][0]) <= 1e-8:
                if tag is not None:
                    result[-1] = point,tag
                continue
            while len(result)>=2:
                a,b = result[-2][0],result[-1][0]
                u,v = b-a,point-b
                if np.linalg.norm(np.cross(u,v)) <= 1e-7*np.linalg.norm(u)*np.linalg.norm(v) and np.dot(u,v)>0:
                    result.pop()
                else:
                    break
            result.append((point,tag))
        return result

    @staticmethod
    def radius(limits):
        radius = limits['min_turn_radius_m']
        minimum = limits.get('min_speed_mps',0)
        if minimum>0:
            radius = max(radius,minimum*minimum/min(limits['acceleration_mps2'],limits['deceleration_mps2']),minimum/math.radians(limits['max_yaw_rate_deg_s']))
        return radius

    def padded(self,values,limits,pad_start=False,pad_end=True):
        """Leave the unfinished outgoing ray open until its next actual turn.

        Padding is used only for local proof. Returned mission geometry always
        uses the original vertices and the final stop is checked without padding.
        """
        points = np.asarray([p for p,_ in values]).copy()
        if len(points)<2:
            return points
        radius = self.radius(limits)
        minimum = limits.get('min_speed_mps',0)
        acceleration_distance = minimum*minimum/min(limits['acceleration_mps2'],limits['deceleration_mps2'])
        if pad_start:
            delta = points[0]-points[1]; horizontal = float(np.linalg.norm(delta[:2]))
            if horizontal>1e-8:
                extension = radius+acceleration_distance
                if len(points)>2:
                    a,b = points[1]-points[0],points[2]-points[1]
                    if min(np.linalg.norm(a[:2]),np.linalg.norm(b[:2]))>1e-8:
                        angle = math.atan2(a[0]*b[1]-a[1]*b[0],a[:2]@b[:2])
                        extension += radius*abs(math.tan(angle/2))
                points[0] += delta/horizontal*(extension+horizontal)
        delta = points[-1]-points[-2]; horizontal = float(np.linalg.norm(delta[:2]))
        if pad_end and horizontal>1e-8:
            extension = radius+acceleration_distance
            if len(points)>2:
                a,b = points[-2]-points[-3],delta
                if np.linalg.norm(a[:2])>1e-8:
                    angle = math.atan2(a[0]*b[1]-a[1]*b[0],a[:2]@b[:2])
                    extension += radius*abs(math.tan(angle/2))
            points[-1] += delta/horizontal*(extension+horizontal)
        return points

    def cover_arc(self,primitive,airway_ids):
        from .scene import _covers
        from .engine import _geometry_ok
        radius = primitive.radius
        step = 2*math.acos(max(-1.,min(1.,1-.025/max(radius,.025))))
        count = max(1,math.ceil(abs(primitive.sweep)/max(step,1e-12)))
        previous = primitive.start
        margin,vertical = self.compiled.clearance(self.cert_profile)
        for i in range(1,count+1):
            self.check_deadline()
            point = primitive.position(primitive.length*i/count)
            if not _geometry_ok(self.compiled,previous,point,self.departure,self.task,self.cert_profile):
                return False
            covered = []
            for airway_id in airway_ids:
                airway = self.network.prepared_airways[airway_id]
                if airway.direction!='BOTH':
                    a,b = airway.centerline.project(Point(previous[:2])),airway.centerline.project(Point(point[:2]))
                    if (airway.direction=='FORWARD' and b<a-1e-7) or (airway.direction=='REVERSE' and b>a+1e-7):
                        continue
                covered.extend(self.compiled.volume_intervals(airway,previous,point,self.departure,self.departure,horizontal=margin,vertical=vertical,inside=True))
            if not _covers(covered):
                return False
            previous = point
        return True

    def local(self,state,arc_id,envelope):
        from .motion import MotionError,build_motion
        from . import native
        key = state[1],state[2],state[5],arc_id,envelope
        if key in self.cache:
            return self.cache[key]
        self.check_deadline()
        _,previous,tail,_,_,anchored = state
        _,_,edge,line,_,_ = self.arcs[arc_id]
        values = [(np.asarray(p),None) for p in tail]
        if values:
            values[-1] = values[-1][0],('junction',previous,arc_id)
        values.extend((p,('internal',arc_id)) for p in line[1:-1])
        if len(line)>1:
            values.append((line[-1],None))
        values = self.clean_tagged(values)
        limits = self.limits(envelope)
        try:
            padded = self.padded(values,limits,pad_start=not anchored)
            candidate = build_motion(padded,limits,self.departure)
            if not native.validate_motion(candidate.primitives,limits,starts=candidate.starts)['valid']:
                raise MotionError('Local analytic motion is infeasible')
            turns = []
            for i in range(1,len(values)-1):
                a,b = padded[i]-padded[i-1],padded[i+1]-padded[i]
                if min(np.linalg.norm(a[:2]),np.linalg.norm(b[:2]))<=1e-8:
                    continue
                angle = math.atan2(a[0]*b[1]-a[1]*b[0],a[:2]@b[:2])
                if abs(angle)>1e-8:
                    turns.append(values[i][1])
            laws = [p for p in candidate.primitives if p.kind in {'arc','yaw'}]
            if len(turns)!=len(laws):
                raise MotionError('Local turn correspondence is incomplete')
            for tag,primitive in zip(turns,laws):
                if tag is None:
                    continue
                if primitive.kind=='yaw':
                    if not self.profile.can_hover:
                        raise MotionError('Aircraft cannot hover through a yaw transition')
                    continue
                if tag[0]=='junction':
                    incoming,outgoing = tag[1:]
                    ids = {self.arcs[outgoing][2].airway_id}
                    if incoming is not None:
                        ids.add(self.arcs[incoming][2].airway_id)
                else:
                    ids = {edge.airway_id}
                if not self.cover_arc(primitive,ids):
                    raise MotionError('Selected incident corridors do not cover the actual turn')
            result = (tuple(tuple(float(v) for v in p) for p,_ in values[-4:]),anchored and len(values)<=4)
        except (ValueError,MotionError):
            result = None
        self.cache[key] = result
        return result

    def candidates(self):
        import itertools
        # A full path's airway envelope is the componentwise fold of its edge
        # envelopes. Solve each finite envelope so future constraints cannot
        # invalidate a settled earlier corner unnoticed by state dominance.
        choices = [sorted({self.base_env[i],*(a[5][i] for a in self.arcs)}) for i in range(len(self.envelope_keys))]
        heap = []; serial = itertools.count()
        for envelope in itertools.product(*choices):
            self.check_deadline()
            solver = _ExpandedPaths(self,tuple(envelope))
            generator = solver.paths()
            first = next(generator,None)
            if first is not None:
                heapq.heappush(heap,(first[0],next(serial),first,generator))
        while heap:
            self.check_deadline()
            _,_,path,generator = heapq.heappop(heap)
            _,states,arc_ids = path
            chosen = [a for a in arc_ids if a is not None]
            node_ids = [self.task.network.start_node_id]+[self.arcs[a][1] for a in chosen]
            edge_ids = [self.arcs[a][2].id for a in chosen]
            legs = [self.arcs[a][3] for a in chosen]
            if not legs:
                legs = [np.asarray([self.network.nodes[node_ids[0]]])]
            yield legs,{'network_version':self.network.graph.version,'node_ids':node_ids,'edge_ids':edge_ids,
                        'weight_field':self.task.network.weight_field,
                        'total_cost':sum(self.arcs[a][2].weights[self.task.network.weight_field] for a in chosen),
                        'effective_cost':path[0]}
            following = next(generator,None)
            if following is not None:
                heapq.heappush(heap,(following[0],next(serial),following,generator))


class _ExpandedPaths:
    """Dijkstra over finite kinematic states; Yen deviations are lazy and uncapped."""
    goal = ('__network_goal__',)

    def __init__(self,context,envelope):
        self.context,self.envelope = context,envelope
        node = context.task.network.start_node_id
        self.start = (node,None,(tuple(context.network.nodes[node]),),context.advance_via(node,0),context.base_env,True)
        self.neighbor_cache = {}; self.terminal_cache = {}

    def terminal(self,state):
        from .motion import build_motion,MotionError
        from . import native
        if state in self.terminal_cache:
            return self.terminal_cache[state]
        c = self.context
        answer = False
        if state[0]==c.task.network.goal_node_id and state[3]==len(c.task.network.via_node_ids) and state[4]==self.envelope:
            try:
                values=[(np.asarray(p),None) for p in state[2]]
                points=c.padded(values,c.limits(self.envelope),pad_start=not state[5],pad_end=False)
                candidate = build_motion(points,c.limits(self.envelope),c.departure)
                answer = bool(native.validate_motion(candidate.primitives,c.limits(self.envelope),starts=candidate.starts)['valid'])
            except (ValueError,MotionError):
                pass
        self.terminal_cache[state] = answer
        return answer

    def neighbors(self,state):
        if state==self.goal:
            return []
        if state in self.neighbor_cache:
            return self.neighbor_cache[state]
        c = self.context; c.check_deadline()
        result = [(self.goal,None,0.)] if self.terminal(state) else []
        for arc_id in c.adjacency[state[0]]:
            c.check_deadline()
            _,node,_,_,cost,requirement = c.arcs[arc_id]
            actual = c.combine(state[4],requirement)
            if c.combine(actual,self.envelope)!=self.envelope:
                continue
            local = c.local(state,arc_id,self.envelope)
            if local is None:
                continue
            tail,anchored = local
            following = node,arc_id,tail,c.advance_via(node,state[3]),actual,anchored
            result.append((following,arc_id,cost))
        self.neighbor_cache[state] = result
        return result

    def shortest(self,start,banned_states=frozenset(),banned_arcs=frozenset()):
        import itertools
        c = self.context; serial = itertools.count()
        queue = [(0.,next(serial),start)]; distance = {start:0.}; previous = {}
        while queue:
            c.check_deadline()
            cost,_,state = heapq.heappop(queue)
            if cost!=distance[state]:
                continue
            if state==self.goal:
                states = [state]; arcs = []
                while state!=start:
                    parent,arc_id = previous[state]
                    states.append(parent); arcs.append(arc_id); state = parent
                return cost,list(reversed(states)),list(reversed(arcs))
            for other,arc_id,weight in self.neighbors(state):
                if other in banned_states or (state,other) in banned_arcs:
                    continue
                candidate = cost+weight
                if not math.isfinite(candidate):
                    invalid('Network cumulative cost exceeds finite numeric range')
                if candidate < distance.get(other,math.inf):
                    distance[other] = candidate; previous[other] = state,arc_id
                    heapq.heappush(queue,(candidate,next(serial),other))
        return None

    def paths(self):
        import itertools
        first = self.shortest(self.start)
        if first is None:
            return
        accepted = []; pending = []; seen = {tuple(first[1])}; serial = itertools.count()
        current = first
        while current is not None:
            self.context.check_deadline()
            yield current
            accepted.append(current)
            cost,states,arcs = current
            root_cost = 0.
            for i in range(len(states)-1):
                self.context.check_deadline()
                root = states[:i+1]
                banned = {(p[1][i],p[1][i+1]) for p in accepted if len(p[1])>i+1 and p[1][:i+1]==root}
                spur = self.shortest(root[-1],frozenset(root[:-1]),frozenset(banned))
                if spur is not None:
                    combined_states = root[:-1]+spur[1]
                    signature = tuple(combined_states)
                    if signature not in seen:
                        seen.add(signature)
                        combined = root_cost+spur[0],combined_states,arcs[:i]+spur[2]
                        heapq.heappush(pending,(combined[0],next(serial),combined))
                if arcs[i] is not None:
                    root_cost += self.context.arcs[arcs[i]][4]
            current = heapq.heappop(pending)[2] if pending else None


def _projections(point,line):
    if len(line)==1:
        return np.array([np.linalg.norm(point-line[0])]),np.array([0.]),np.array([0.])
    delta = np.diff(line,axis=0); lengths = np.linalg.norm(delta,axis=1)
    fraction = np.clip(np.sum((point-line[:-1])*delta,axis=1)/np.maximum(np.sum(delta*delta,axis=1),1e-30),0,1)
    distances = np.linalg.norm(point-(line[:-1]+fraction[:,None]*delta),axis=1)
    offsets = np.r_[0.,np.cumsum(lengths)]
    return distances,offsets[:-1]+fraction*lengths,fraction
