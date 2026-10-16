"""Geometric motion primitives with an analytic, acceleration-limited time law.

The exported polyline is a bounded-error view of the primitives. Timing and
collision sampling are derived from the same time law, never by moving endpoints.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


MAX_MOTION_SAMPLES = 100_000


class MotionError(ValueError):
    """A candidate cannot satisfy its geometric or dynamic constraints."""


@dataclass
class Primitive:
    kind: str
    start: np.ndarray
    end: np.ndarray
    length: float
    cap: float
    center: np.ndarray | None = None
    radius: float = 0.0
    sweep: float = 0.0
    duration: float = 0.0
    v0: float = 0.0
    v1: float = 0.0
    peak: float = 0.0
    accel: float = 0.0
    decel: float = 0.0
    ta: float = 0.0
    tc: float = 0.0
    td: float = 0.0

    def position(self, distance: float) -> np.ndarray:
        f = min(1.0, max(0.0, distance / self.length)) if self.length else 0.0
        if self.kind != "arc":
            return self.start + f * (self.end - self.start)
        angle = math.atan2(self.start[1] - self.center[1], self.start[0] - self.center[0])
        angle += f * self.sweep
        return np.array([self.center[0] + self.radius * math.cos(angle),
                         self.center[1] + self.radius * math.sin(angle),
                         self.start[2] + f * (self.end[2] - self.start[2])])

    def distance_at(self, t: float) -> float:
        t = max(0.0, min(t, self.duration))
        if t <= self.ta:
            return self.v0 * t + .5 * self.accel * t * t
        sa = (self.v0 + self.peak) * self.ta * .5
        if t <= self.ta + self.tc:
            return sa + self.peak * (t - self.ta)
        q = t - self.ta - self.tc
        return min(self.length, sa + self.peak * self.tc + self.peak * q - .5 * self.decel * q * q)

    def time_at(self, s: float) -> float:
        if self.length <= 1e-12:
            return 0.0
        s = max(0.0, min(s, self.length))
        sa = (self.v0 + self.peak) * self.ta * .5
        if s <= sa + 1e-10:
            return (math.sqrt(max(0.0, self.v0 ** 2 + 2 * self.accel * s)) - self.v0) / self.accel
        if s <= sa + self.peak * self.tc:
            return self.ta + (s - sa) / self.peak
        remaining = self.length - s
        return self.duration - (math.sqrt(max(0.0, self.v1 ** 2 + 2 * self.decel * remaining)) - self.v1) / self.decel


@dataclass
class Motion:
    points: np.ndarray
    times: np.ndarray
    primitives: list[Primitive]
    starts: list[float]
    events: list[tuple[int, list[str]]] = field(default_factory=list)
    distance: float = 0.0

    def shifted(self, seconds: float) -> Motion:
        return Motion(self.points, self.times + seconds, self.primitives,
                      [t + seconds for t in self.starts], self.events, self.distance)

    def collision_samples(self, error: float = .025) -> tuple[np.ndarray, np.ndarray]:
        """Bound each chord's space-time error under acceleration and curvature."""
        if not math.isfinite(error) or error <= 0:
            raise MotionError("Collision approximation error must be finite and positive")
        points, times = [], []
        for p, start in zip(self.primitives, self.starts):
            if not math.isfinite(p.duration) or p.duration < 0:
                raise MotionError("Motion duration must be finite and nonnegative")
            if p.length <= 1e-12:
                ts = [0.0, p.duration]
            else:
                # |x(t)-chord| <= max|x''| dt²/8; split also at phase boundaries.
                acc = max(p.accel, p.decel, p.peak ** 2 / max(p.radius, 1e-9) if p.kind == "arc" else 0)
                step = min(2.0, math.sqrt(8 * error / max(acc * 2, 1e-9)))
                requested = p.duration / step if step > 0 else math.inf
                if not math.isfinite(requested) or requested + len(points) + 4 > MAX_MOTION_SAMPLES:
                    raise MotionError("Certified motion sample budget exceeded (100000 samples)")
                ts = sorted(set([0.0, p.ta, p.ta + p.tc, p.duration] +
                                [min(p.duration, i * step) for i in range(1, math.ceil(p.duration / step))]))
            if len(points) + len(ts) > MAX_MOTION_SAMPLES:
                raise MotionError("Certified motion sample budget exceeded (100000 samples)")
            for t in ts:
                if times and abs(start + t - times[-1]) < 1e-9:
                    continue
                points.append(p.position(p.distance_at(t)))
                times.append(start + t)
        if not points:
            return self.points, self.times
        return np.asarray(points), np.asarray(times)


def _clean(points: np.ndarray) -> np.ndarray:
    out = [np.asarray(points[0], dtype=float)]
    for point in points[1:]:
        point = np.asarray(point, dtype=float)
        if np.linalg.norm(point - out[-1]) > 1e-8:
            out.append(point)
    # Remove collinear search samples, retaining actual turns and elevation changes.
    result = []
    for point in out:
        while len(result) >= 2:
            a, b = result[-2:]
            u, v = b - a, point - b
            if np.linalg.norm(np.cross(u, v)) <= 1e-7 * np.linalg.norm(u) * np.linalg.norm(v) and np.dot(u, v) > 0:
                result.pop()
            else:
                break
        result.append(point)
    return np.asarray(result)


def build_motion(points, limits: dict, start_time: float = 0.0, *, tolerance: float = .1) -> Motion:
    if not math.isfinite(tolerance) or tolerance <= 0:
        raise MotionError("Motion approximation tolerance must be finite and positive")
    points = _clean(np.asarray(points, dtype=float))
    vmax = float(limits["max_speed_mps"])
    cruise = min(vmax, float(limits.get("cruise_speed_mps", vmax)))
    vmin = float(limits.get("min_speed_mps", 0))
    accel = float(limits["acceleration_mps2"])
    decel = float(limits["deceleration_mps2"])
    yaw = math.radians(float(limits["max_yaw_rate_deg_s"]))
    radius = float(limits.get("min_turn_radius_m", 0))
    if min(vmax, cruise, accel, decel, yaw) <= 0 or vmin > vmax:
        raise MotionError("Empty speed or acceleration envelope")
    if vmin > 0:
        radius = max(radius, vmin * vmin / min(accel, decel), vmin / yaw)
    if len(points) == 1:
        return Motion(points, np.array([start_time]), [], [], [(0, ["START", "GOAL"])])

    turns: dict[int, tuple] = {}
    for i in range(1, len(points) - 1):
        u, v = points[i] - points[i-1], points[i+1] - points[i]
        lu, lv = np.linalg.norm(u[:2]), np.linalg.norm(v[:2])
        if min(lu, lv) <= 1e-8:
            continue
        uxy, vxy = u[:2]/lu, v[:2]/lv
        delta = math.atan2(float(uxy[0]*vxy[1] - uxy[1]*vxy[0]), float(np.dot(uxy, vxy)))
        if abs(delta) < 1e-8:
            continue
        if abs(math.degrees(delta)) > limits.get("max_turn_angle_deg", 180) + 1e-7:
            raise MotionError("TURN.maxTurnAngle exceeded by an actual transition")
        if radius > 0:
            if abs(delta) >= math.pi - 1e-7:
                raise MotionError("A positive-radius U-turn requires a spatial detour")
            cut = radius * math.tan(abs(delta)/2)
            if cut >= min(lu, lv) - 1e-7:
                raise MotionError("Insufficient edge length for the required turning radius")
            entry = points[i] - u * (cut/lu)
            exit_ = points[i] + v * (cut/lv)
            normal = np.array([-uxy[1], uxy[0]]) * (1 if delta > 0 else -1)
            center = np.array([*(entry[:2] + normal*radius), entry[2]])
            turns[i] = entry, exit_, center, delta, cut
        else:
            turns[i] = points[i], points[i], None, delta, 0
    for i in range(1, len(points)-2):
        if i in turns and i+1 in turns:
            length = np.linalg.norm((points[i+1]-points[i])[:2])
            if turns[i][4] + turns[i+1][4] >= length - 1e-7:
                raise MotionError("Adjacent turning arcs overlap")

    primitives: list[Primitive] = []
    current = points[0]

    def line(end):
        nonlocal current
        d = end - current
        length = float(np.linalg.norm(d))
        if length > 1e-8:
            horizontal = float(np.linalg.norm(d[:2]))
            cap = cruise * length/horizontal if horizontal > 1e-9 else float("inf")
            if abs(d[2]) > 1e-9:
                vz = limits["climb_speed_mps"] if d[2] > 0 else limits["descent_speed_mps"]
                cap = min(cap, vz * length/abs(d[2]))
            kind = "climb" if d[2] > 1e-8 else "descent" if d[2] < -1e-8 else "line"
            primitives.append(Primitive(kind, current.copy(), end.copy(), length, cap))
        current = end.copy()

    for i in range(1, len(points)-1):
        if i not in turns:
            line(points[i])
            continue
        entry, exit_, center, delta, _ = turns[i]
        line(entry)
        if center is None:
            p = Primitive("yaw", current.copy(), current.copy(), 0, 0)
            p.sweep = delta
            p.duration = abs(delta)/yaw
            primitives.append(p)
        else:
            horizontal = abs(delta)*radius
            length = math.hypot(horizontal, exit_[2]-entry[2])
            # Split the available vector acceleration between curvature and timing.
            horizontal_cap = min(cruise, radius*yaw, math.sqrt(min(accel,decel)*radius/math.sqrt(2)))
            cap = horizontal_cap * length/horizontal
            if abs(exit_[2]-entry[2]) > 1e-9:
                vz = limits["climb_speed_mps"] if exit_[2] > entry[2] else limits["descent_speed_mps"]
                cap = min(cap, vz*length/abs(exit_[2]-entry[2]))
            primitives.append(Primitive("arc", entry, exit_, length, cap, center, radius, delta))
            current = exit_.copy()
    line(points[-1])

    # Forward/backward propagation preserves continuous speed at tangent joins.
    n = len(primitives)
    velocities = np.zeros(n+1)
    for i in range(1,n):
        left,right = primitives[i-1:i+1]
        if min(left.length,right.length) > 0:
            def tangent_at(p, at_end):
                if p.kind != "arc":
                    return (p.end-p.start)/p.length
                point=p.end if at_end else p.start
                r=point[:2]-p.center[:2]
                horizontal=np.array([-r[1],r[0]]) * (1 if p.sweep>0 else -1) / p.radius
                return np.array([*(horizontal*(abs(p.sweep)*p.radius/p.length)),(p.end[2]-p.start[2])/p.length])
            a,b = tangent_at(left,True),tangent_at(right,False)
            tangent = np.dot(a,b) >= .999999999
            if tangent:
                velocities[i] = min(left.cap,right.cap)
    for i,p in enumerate(primitives):
        a = accel/math.sqrt(2) if p.kind == "arc" else accel
        velocities[i+1] = min(velocities[i+1], math.sqrt(velocities[i]**2+2*a*p.length))
    for i in range(n-1,-1,-1):
        p=primitives[i]
        d = decel/math.sqrt(2) if p.kind == "arc" else decel
        velocities[i] = min(velocities[i], math.sqrt(velocities[i+1]**2+2*d*p.length))
    vertices, times, starts, events = [], [], [], []
    t = start_time
    for i,p in enumerate(primitives):
        starts.append(t)
        start_index=max(0,len(vertices)-1) if times and abs(times[-1]-t)<1e-8 else len(vertices)
        if p.length > 1e-9:
            p.accel = accel/math.sqrt(2) if p.kind == "arc" else accel
            p.decel = decel/math.sqrt(2) if p.kind == "arc" else decel
            p.v0,p.v1 = float(velocities[i]),float(velocities[i+1])
            p.peak = min(p.cap, math.sqrt(max(0, (2*p.length+p.v0**2/p.accel+p.v1**2/p.decel)/(1/p.accel+1/p.decel))))
            if vmin > 0 and p.peak < vmin - 1e-8:
                raise MotionError("The segment cannot attain the minimum cruise speed")
            if vmin > 0 and i > 0 and primitives[i-1].length > 0 and p.v0 < vmin - 1e-8:
                raise MotionError("Minimum speed prohibits stopping at an intermediate transition")
            p.ta=max(0,(p.peak-p.v0)/p.accel)
            p.td=max(0,(p.peak-p.v1)/p.decel)
            p.tc=max(0,(p.length-(p.v0+p.peak)*p.ta*.5-(p.v1+p.peak)*p.td*.5)/p.peak)
            p.duration=p.ta+p.tc+p.td
            cuts=[0.0,p.length,p.distance_at(p.ta),p.distance_at(p.ta+p.tc)]
            if p.kind == "arc":
                angle_step=2*math.acos(max(-1, min(1,1-tolerance/max(p.radius, tolerance))))
                count=max(1,math.ceil(abs(p.sweep)/max(angle_step,1e-8)))
                if count + len(vertices) + len(cuts) > MAX_MOTION_SAMPLES:
                    raise MotionError("Exported motion sample budget exceeded (100000 samples)")
                cuts += [p.length*j/count for j in range(1,count)]
            if len(vertices) + len(cuts) > MAX_MOTION_SAMPLES:
                raise MotionError("Exported motion sample budget exceeded (100000 samples)")
            for s in sorted(set(cuts)):
                ts=t+p.time_at(s)
                if times and abs(ts-times[-1])<1e-8:
                    continue
                vertices.append(p.position(s));times.append(ts)
        else:
            if len(vertices) + 2 > MAX_MOTION_SAMPLES:
                raise MotionError("Exported motion sample budget exceeded (100000 samples)")
            if not times or abs(t-times[-1])>1e-8:
                vertices.append(p.start);times.append(t)
            vertices.append(p.end);times.append(t+p.duration)
        label={"arc":["TURN_START"],"climb":["CLIMB_START"],"descent":["DESCENT_START"],"yaw":["TURN_START","HOVER_START"]}.get(p.kind,[])
        if label:
            events.append((start_index,label))
            events.append((len(vertices)-1,[s.replace("START","END") for s in label]))
        t+=p.duration
    if not vertices:
        vertices,times=[points[0]],[start_time]
    events.extend([(0,["START"]),(len(vertices)-1,["GOAL"])])
    return Motion(np.asarray(vertices),np.asarray(times),primitives,starts,events,sum(p.length for p in primitives))
