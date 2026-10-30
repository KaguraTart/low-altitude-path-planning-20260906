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

    def select(self, task, profile, departure, deadline):
        from .engine import _geometry_ok
        selection = task.network
        for edge in self.graph.edges:
            if time.perf_counter() >= deadline:
                raise TimeoutError("Network preparation deadline exhausted")
            if selection.weight_field not in edge.weights:
                invalid(f"Missing weight field {selection.weight_field} on edge {edge.id}")
        adjacency = {n: [] for n in self.nodes}
        for a, b, edge, points in self.arcs(departure):
            if time.perf_counter() >= deadline:
                raise TimeoutError("Network preparation deadline exhausted")
            if self.airways[edge.airway_id].allowed_profile_ids and profile.id not in self.airways[edge.airway_id].allowed_profile_ids:
                continue
            if not self.edge_geometry_allowed(edge,points,profile,departure,deadline):
                continue
            admissible = True
            for p,q in zip(points,points[1:]):
                if time.perf_counter() >= deadline:
                    raise TimeoutError("Network admissibility deadline exhausted")
                if not _geometry_ok(self.compiled,p,q,departure,task,profile):
                    admissible = False; break
            if admissible:
                effective = self.weighted_cost(edge,points,task,profile,departure,deadline)
                if math.isfinite(effective):
                    adjacency[a].append((b, edge, points, effective))
        nodes = [selection.start_node_id]; edges = []; legs = []; cost = 0.
        for goal in [*selection.via_node_ids, selection.goal_node_id]:
            start = nodes[-1]
            queue = [(0., start)]; distances = {start: 0.}; previous = {}
            while queue:
                if time.perf_counter() >= deadline:
                    raise TimeoutError("Network search deadline exhausted")
                d, node = heapq.heappop(queue)
                if d != distances[node]:
                    continue
                if node == goal:
                    break
                for other, edge, points, effective in adjacency[node]:
                    nd = d + effective
                    if not math.isfinite(nd):
                        invalid("Network cumulative cost exceeds finite numeric range")
                    if nd < distances.get(other, math.inf):
                        distances[other] = nd
                        previous[other] = (node, edge, points)
                        heapq.heappush(queue, (nd, other))
            if goal not in distances:
                return None
            chain = []; current = goal
            while current != start:
                parent, edge, points = previous[current]
                chain.append((current, edge, points)); current = parent
            for node, edge, points in reversed(chain):
                nodes.append(node); edges.append(edge.id); legs.append(points)
            cost += distances[goal]
        if not legs:
            legs = [np.asarray([self.nodes[nodes[0]]])]
        return legs, {"network_version": self.graph.version, "node_ids": nodes, "edge_ids": edges, "weight_field": selection.weight_field, "total_cost": sum(self.edges[e].weights[selection.weight_field] for e in edges), "effective_cost": cost}

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
            junction_times = [float(times[0])]; after = 0.
            for node in candidate["node_ids"][1:-1]:
                distances, positions, fractions = projections(self.nodes[node], points)
                distances = np.where(positions >= after-1e-6, distances, math.inf)
                i = int(np.argmin(distances))
                if not math.isfinite(float(distances[i])):
                    return False
                after = float(positions[i])
                junction_times.append(float(times[i]+fractions[i]*(times[i+1]-times[i])))
            junction_times.append(float(times[-1]))
            edge_starts = junction_times[:-1].copy()
            edge_ends = junction_times[1:].copy()
            turn_motion = motion if motion is not None else expected
            for index,node in enumerate(candidate["node_ids"][1:-1]):
                position = self.nodes[node]
                for primitive,start_time in zip(turn_motion.primitives,turn_motion.starts):
                    if primitive.kind != "arc" or not start_time-1e-7 <= junction_times[index+1] <= start_time+primitive.duration+1e-7:
                        continue
                    incoming = primitive.start[:2]-primitive.center[:2]
                    outgoing = primitive.end[:2]-primitive.center[:2]
                    incoming = np.array([-incoming[1],incoming[0]]) * np.sign(primitive.sweep)
                    outgoing = np.array([-outgoing[1],outgoing[0]]) * np.sign(primitive.sweep)
                    parameters = np.linalg.lstsq(np.column_stack([incoming,outgoing]),primitive.end[:2]-primitive.start[:2],rcond=None)[0]
                    corner = primitive.start[:2]+parameters[0]*incoming
                    if np.linalg.norm(corner-position[:2]) > .1:
                        continue
                    edge_ends[index] = max(edge_ends[index],start_time+primitive.duration)
                    edge_starts[index+1] = min(edge_starts[index+1],start_time)
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
