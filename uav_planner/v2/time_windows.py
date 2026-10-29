"""Conservative TIME-rule departure alternatives for an analytic candidate.

All returned timestamps are absolute seconds relative to ``scene.time_origin``.
The path and its internal timing remain unchanged; each alternative is a launch
window for shifting that whole candidate. Exact scoped checks still run after
scheduling. At most the first ten disjoint alternatives are returned.
"""
from __future__ import annotations

from datetime import datetime, time, timedelta
import math

import numpy as np
from shapely.geometry import LineString, Point

from .rules import LOCAL_TZ


_EDGE_GUARD = 1e-6


def _merge(windows: list[tuple[float, float]]) -> list[tuple[float, float]]:
    result: list[tuple[float, float]] = []
    for lo, hi in sorted(windows):
        if lo > hi:
            continue
        if result and lo <= result[-1][1]:
            result[-1] = (result[-1][0], max(result[-1][1], hi))
        else:
            result.append((lo, hi))
    return result


def _intersect(first, second):
    return _merge([(max(a,c),min(b,d)) for a,b in first for c,d in second
                   if max(a,c)<=min(b,d)])


def _scope_intersects(compiled, rule, motion) -> bool:
    if not rule.resource_ids:
        return True
    points=np.asarray(motion.points,dtype=float)[:,:2]
    geometry=Point(points[0]) if len(points)==1 or np.all(points==points[0]) else LineString(points)
    # Ignore altitude and validity here: an extra scheduling restriction is
    # conservative, while skipping a possibly applicable rule is unsafe.
    return any(compiled.resources[str(ref)].geometry.intersects(geometry.buffer(.100001))
               for ref in rule.resource_ids)


def _calendar(compiled, content, lo: float, hi: float, duration: float):
    origin=compiled.model.time_origin
    first=(origin+timedelta(seconds=lo)).astimezone(LOCAL_TZ).date()-timedelta(days=1)
    last=(origin+timedelta(seconds=hi+duration)).astimezone(LOCAL_TZ).date()+timedelta(days=1)
    weekdays=set(content.get("weekdays",range(7)))
    daily=[];departures=[];arrivals=[]
    begin=time.fromisoformat(content.get("dailyStart","00:00"))
    finish=time.fromisoformat(content.get("dailyEnd","00:00"))
    earliest=time.fromisoformat(content.get("earliestDeparture","00:00"))
    latest=time.fromisoformat(content["latestArrival"]) if "latestArrival" in content else None
    def seconds(value):
        return float((value-origin).total_seconds())
    day=first
    while day<=last:
        midnight=datetime.combine(day,time(),LOCAL_TZ)
        next_midnight=midnight+timedelta(days=1)
        if (day.isoweekday() % 7) in weekdays:
            if "dailyStart" in content:
                a=datetime.combine(day,begin,LOCAL_TZ)
                b=datetime.combine(day+timedelta(days=finish<=begin),finish,LOCAL_TZ)
                daily.append((seconds(a),seconds(b)))
            else:
                end=seconds(next_midnight)
                # The weekday-only evaluator examines the arrival date too,
                # including a flight ending exactly at midnight.
                if (next_midnight.isoweekday() % 7) not in weekdays:
                    end-=_EDGE_GUARD
                daily.append((seconds(midnight),end))
        departures.append((seconds(datetime.combine(day,earliest,LOCAL_TZ)),
                           seconds(next_midnight)-(_EDGE_GUARD if earliest!=time() else 0)))
        if latest is not None:
            arrivals.append((seconds(midnight)-duration,
                             seconds(datetime.combine(day,latest,LOCAL_TZ))-duration))
        day+=timedelta(days=1)
    return _merge(daily),_merge(departures),_merge(arrivals)


def departure_windows(compiled, task, motion, lower_abs: float, upper_abs: float) -> list[tuple[float,float]]:
    """Intersect launch alternatives with all selected, potentially scoped TIME rules.

    Daily windows constrain the part of a flight overlapping the rule's effective
    period. ``earliestDeparture`` and ``latestArrival`` retain their route-level
    clock meanings whenever that effective period overlaps the flight. Inactive
    periods remain valid alternatives. Scoped rules constrain the whole candidate
    when its XY geometry intersects their scope, so final exact checks can only
    narrow these safe alternatives.
    """
    lower=float(lower_abs);upper=float(upper_abs)
    duration=float(motion.times[-1]-motion.times[0])
    if not all(math.isfinite(value) for value in (lower,upper,duration)) or duration<0:
        raise ValueError("Departure bounds and motion duration must be finite")
    origin=compiled.model.time_origin
    lower=max(lower,0.0,float((task.departure_time-origin).total_seconds()))
    upper=min(upper,float(compiled.model.horizon_s)-duration)
    if lower>upper:
        return []
    windows=[(lower,upper)]
    for rule in compiled.rules.selected(task):
        if rule.rule_type!="TIME" or not _scope_intersects(compiled,rule,motion):
            continue
        start=float((rule.effective_start-origin).total_seconds()) if rule.effective_start else -math.inf
        end=float((rule.effective_end-origin).total_seconds()) if rule.effective_end else math.inf
        active_lo=max(lower,start-duration-2*_EDGE_GUARD);active_hi=min(upper,end+2*_EDGE_GUARD)
        if active_lo>active_hi:
            continue
        daily,departures,arrivals=_calendar(compiled,rule.content,active_lo,active_hi,duration)
        active=[]
        for a,b in daily:
            if b<start or a>end:
                continue
            lo=active_lo if start>=a else max(active_lo,a)
            hi=active_hi if end<=b else min(active_hi,b-duration)
            if lo<=hi:
                active.append((lo,hi))
        active=_intersect(_merge(active),departures)
        if "latestArrival" in rule.content:
            active=_intersect(active,arrivals)
        allowed=active
        if math.isfinite(start):
            allowed.append((lower,min(upper,start-duration-2*_EDGE_GUARD)))
        if math.isfinite(end):
            allowed.append((max(lower,end+2*_EDGE_GUARD),upper))
        windows=_intersect(windows,_merge(allowed))
        if not windows:
            break
    return windows
