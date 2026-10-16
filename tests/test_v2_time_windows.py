"""Calendar launch alternatives preserve the exact TIME final-check semantics."""
from datetime import datetime, timedelta
from types import SimpleNamespace

import numpy as np
import pytest
from shapely.geometry import box

from uav_planner.v2.coordinates import CoordinateFrame
from uav_planner.v2.models import Task
from uav_planner.v2.rules import _daily_interval_allowed
from uav_planner.v2.scene import CompiledScene
from uav_planner.v2.time_windows import departure_windows


def setup(content, *, origin="2026-10-16T09:00:00+08:00", duration=60, rules=None, **scene_updates):
    at=datetime.fromisoformat(origin)
    scene={"coordinates":{"origin_lon":104.77,"origin_lat":29.33},
        "time_origin":at,"bounds":{"x_min":0,"x_max":1000,"y_min":0,"y_max":1000,"z_min":0,"z_max":300},
        "horizon_s":86400,"rules":rules if rules is not None else [{"id":"time","rule_type":"TIME","content":content}]}
    scene.update(scene_updates)
    compiled=CompiledScene(scene)
    frame=CoordinateFrame(scene["coordinates"])
    from shapely.geometry import Point
    task=Task.model_validate({"id":"task","start_wkt":frame.to_wgs84(Point(100,100,80)),
        "goal_wkt":frame.to_wgs84(Point(200,100,80)),"departure_time":at,"alt_reference":"MSL","route_mode":"free"})
    motion=SimpleNamespace(points=np.asarray([[100,100,80],[200,100,80]]),times=np.asarray([125,125+duration]))
    return compiled,task,motion


def test_future_earliest_departure_becomes_a_search_window():
    compiled,task,motion=setup({"earliestDeparture":"09:05"})
    assert departure_windows(compiled,task,motion,0,600)==[(300,600)]


def test_future_daily_start_and_latest_arrival_include_motion_duration():
    compiled,task,motion=setup({"dailyStart":"09:05","dailyEnd":"09:10","latestArrival":"09:09"})
    assert departure_windows(compiled,task,motion,0,1000)==[(300,480)]


def test_time_windows_use_duration_independent_of_current_motion_start():
    compiled,task,motion=setup({"earliestDeparture":"09:05","latestArrival":"09:10"})
    first=departure_windows(compiled,task,motion,0,600)
    motion.times+=500
    assert departure_windows(compiled,task,motion,0,600)==first==[(300,540)]


def test_cross_midnight_window_belongs_to_its_start_weekday():
    compiled,task,motion=setup({"dailyStart":"23:00","dailyEnd":"01:00","weekdays":[5]},
        origin="2026-10-16T22:00:00+08:00",duration=1800)
    assert departure_windows(compiled,task,motion,0,14400)==[(3600,9000)]


def test_cross_midnight_previous_day_window_is_available():
    compiled,task,motion=setup({"dailyStart":"23:00","dailyEnd":"01:00","weekdays":[5]},
        origin="2026-10-17T00:00:00+08:00",duration=1800)
    assert departure_windows(compiled,task,motion,0,7200)==[(0,1800)]


def test_weekday_only_avoids_arrival_at_disallowed_midnight():
    compiled,task,motion=setup({"weekdays":[5]},origin="2026-10-16T23:58:00+08:00",duration=60)
    windows=departure_windows(compiled,task,motion,0,180)
    assert windows[0][0]==0
    assert windows[0][1]<60
    assert windows[0][1]==pytest.approx(60,abs=2e-6)


def test_effective_period_retains_safe_before_and_after_alternatives():
    origin=datetime.fromisoformat("2026-10-16T09:00:00+08:00")
    rules=[{"id":"time","rule_type":"TIME","content":{"earliestDeparture":"09:10"},
        "effective_start":origin+timedelta(seconds=300),"effective_end":origin+timedelta(seconds=500)}]
    compiled,task,motion=setup({},rules=rules)
    windows=departure_windows(compiled,task,motion,0,1000)
    assert len(windows)==2
    assert windows[0][0]==0 and windows[0][1]<240
    assert windows[1][0]>500 and windows[1][1]==1000


def test_daily_rule_only_constrains_the_overlapping_effective_part():
    origin=datetime.fromisoformat("2026-10-16T09:00:00+08:00")
    rules=[{"id":"time","rule_type":"TIME","content":{"dailyStart":"09:05","dailyEnd":"09:06"},
        "effective_start":origin+timedelta(seconds=300),"effective_end":origin+timedelta(seconds=360)}]
    compiled,task,motion=setup({},rules=rules,duration=180)
    assert departure_windows(compiled,task,motion,0,600)==[(0,600)]


def test_selected_rules_intersect_and_unselected_rules_do_not_apply():
    rules=[{"id":"early","rule_type":"TIME","content":{"earliestDeparture":"09:05"}},
        {"id":"late","rule_type":"TIME","content":{"latestArrival":"09:09"}}]
    compiled,task,motion=setup({},rules=rules)
    assert departure_windows(compiled,task,motion,0,1000)==[(300,480)]
    task=task.model_copy(update={"rule_ids":["early"]})
    assert departure_windows(compiled,task,motion,0,1000)==[(300,1000)]


def test_scoped_rule_is_skipped_only_when_xy_candidate_misses_scope():
    frame=CoordinateFrame({"origin_lon":104.77,"origin_lat":29.33})
    obstacle={"id":"scope","geometry_wkt":frame.to_wgs84(box(500,500,600,600)),
        "min_altitude":0,"max_altitude":300,"alt_reference":"MSL"}
    rules=[{"id":"scoped","rule_type":"TIME","resource_ids":["scope"],"content":{"earliestDeparture":"09:05"}}]
    compiled,task,motion=setup({},rules=rules,obstacles=[obstacle])
    assert departure_windows(compiled,task,motion,0,600)==[(0,600)]
    motion.points=np.asarray([[500,550,80],[600,550,80]])
    assert departure_windows(compiled,task,motion,0,600)==[(300,600)]


def test_every_sampled_allowed_departure_passes_calendar_evaluator():
    content={"dailyStart":"23:30","dailyEnd":"01:00","weekdays":[5,6],"earliestDeparture":"00:15","latestArrival":"23:50"}
    compiled,task,motion=setup(content,origin="2026-10-16T22:00:00+08:00",duration=600)
    for lo,hi in departure_windows(compiled,task,motion,0,95000):
        for departure in np.linspace(lo,hi,11):
            start=task.departure_time+timedelta(seconds=float(departure))
            end=start+timedelta(seconds=600)
            assert _daily_interval_allowed(start,end,content)
            assert start.strftime("%H:%M")>=content["earliestDeparture"]
            assert end.strftime("%H:%M")<=content["latestArrival"]


def test_horizon_and_maximum_alternative_count_are_bounded():
    origin=datetime.fromisoformat("2026-10-16T09:00:00+08:00")
    rules=[{"id":f"time-{i}","rule_type":"TIME","content":{"earliestDeparture":"23:59"},
        "effective_start":origin+timedelta(seconds=i*600+300),
        "effective_end":origin+timedelta(seconds=i*600+400)} for i in range(20)]
    compiled,task,motion=setup({},rules=rules,horizon_s=20000)
    windows=departure_windows(compiled,task,motion,0,20000)
    assert len(windows)==10
    assert windows==sorted(windows)
    compiled,task,motion=setup({"dailyStart":"09:05","dailyEnd":"09:10"},horizon_s=360)
    assert departure_windows(compiled,task,motion,0,1000)==[(300,300)]
