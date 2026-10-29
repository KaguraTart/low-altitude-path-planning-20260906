#!/usr/bin/env python3
"""Generate the current ENG-06/ENG-07 delivery fixtures and integrity manifest."""
from __future__ import annotations

import hashlib
import json
import sys
from copy import deepcopy
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from shapely.geometry import Polygon, box
from server.v2 import example_request
from uav_planner.v2.coordinates import CoordinateFrame
from uav_planner.v2.engine import plan_single_v2
from tools.benchmark_v2 import load_frozen


def write(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def rule(kind, content, scope=None):
    value = {"id": f"rule-{kind}", "rule_type": kind, "content": content, "resource_ids": []}
    if scope:
        value["scope"] = scope
    return value


def main():
    root = ROOT / "test_api" / "data"
    strict = example_request()
    strict["task"].update(engine_id="ENG-06", alt_reference="EG")
    strict["scene"].update(scene_type="LOW_ALTITUDE", network_version="example-network-20261029")
    strict["scene"]["airways"][0]["allowed_profile_ids"] = ["dji_mini_4_pro"]
    frame = CoordinateFrame(strict["scene"]["coordinates"])
    from shapely import from_wkt
    minlon,minlat,maxlon,maxlat = from_wkt(frame.to_wgs84(box(-100,-100,1500,300))).bounds
    strict["scene"]["rules"] = [
        rule("HEIGHT_LAYER", {"minHeight":60,"maxHeight":140,"layerStep":20,"altDatum":"EG"}),
        rule("WIDTH", {"width":100,"bufferWidth":10}),
        rule("DIRECTION", {"direction":"ONE_WAY"}),
        rule("SPEED", {"minSpeed":0,"maxSpeed":12,"cruiseSpeed":6}),
        rule("TIME", {"dailyStart":"07:00","dailyEnd":"20:00","weekdays":[0,1,2,3,4,5,6]}),
        rule("AREA", {"scope":"BBOX","minLon":minlon,"maxLon":maxlon,"minLat":minlat,"maxLat":maxlat}),
        rule("CLIMB", {"maxClimbRate":5,"maxDescentRate":3}),
        rule("TURN", {"maxTurnAngle":90,"minTurnRadius":10}),
        rule("PRIORITY", {"priorityScene":"PLANNING","costBonus":.15}),
        rule("WAYPOINT", {"maxWaypoints":10,"minWaypointInterval":500}),
    ]
    write(root / "examples/single_strict_request.json", strict)
    prefer = deepcopy(strict); prefer["task"]["route_mode"] = "prefer"
    write(root / "examples/single_prefer_request.json", prefer)
    free = example_request(); free["scene"]["airways"] = []
    free["scene"]["bounds"]["y_max"] = 500
    free["task"].update(route_mode="free",engine_id="ENG-07",alt_reference="EG",
        aircraft={"id":"example-5kg","name":"Example aircraft with 5 kg payload capacity","payload_kg":5},
        state={"battery_percent":80,"remaining_flight_time_s":2000,"remaining_range_m":15000,"home_wkt":free["task"]["goal_wkt"],"alt_reference":"EG"})
    def volume(x0,y0,x1,y1,z0=0,z1=150):
        return frame.to_wgs84(Polygon([(x0,y0,z0),(x1,y0,z0),(x1,y1,z1),(x0,y1,z1),(x0,y0,z0)]))
    field_scope = {"grid":[{"geometry":volume(0,0,1400,500,0,200)}]}
    free["scene"]["rules"] = [
        rule("HEIGHT_LAYER", {"minAltitude":60,"maxAltitude":140,"altDatum":"EG"}),
        rule("BUILDING", {"avoidMode":"BUFFER","safeDistance":30,"bufferRadius":15}, {"grid":[{"geometry":volume(200,300,230,330)}]}),
        rule("NO_FLY", {"noFlyMode":"STANDARD","heightCeiling":300,"includeBuffer":True}, {"geom":[{"type":"NO_FLY_ZONE","geom":volume(400,300,430,330)}]}),
        rule("ZONE", {"zoneLevels":["A","B","C"],"coverageThreshold":.3}, {"geom":[{"type":"CONTROL_ZONE","geom":volume(600,300,630,330)}]}),
        rule("WEATHER", {"maxWindSpeed":10.8,"noPrecipitation":True,"noIcing":True}, field_scope),
        rule("SIGNAL", {"minSignalStrength":-100,"bands":["2.4GHz","5.8GHz"]}, field_scope),
        rule("CLIMB", {"maxClimbRate":5,"maxDescentRate":3}),
        rule("TURN", {"maxTurnAngle":90,"minTurnRadius":10}),
        rule("HOVER", {"maxHoverTime":300,"defaultHoverTime":60}),
        rule("ENDURANCE", {"maxRange":15000,"minEndurance":1800}),
        rule("PAYLOAD", {"maxPayloadKg":5}),
        rule("RTL", {"rtlAltitude":100,"rtlOnLowBattery":True,"lowBatteryPercent":20}),
        rule("PRIORITY", {"priorityScene":"PLANNING","costBonus":.1}),
        rule("WAYPOINT", {"maxWaypoints":10,"minWaypointInterval":500}),
    ]
    free["scene"]["datasets"] = [
        {"id":"weather","kind":"weather","version":"example-20261029","cells":[{"bounds":free["scene"]["bounds"],"values":{"wind_speed_mps":2,"precipitation":0,"icing":False}}]},
        {"id":"signal","kind":"signal","version":"example-20261029","cells":[{"bounds":free["scene"]["bounds"],"values":{"bands":{"2.4GHz":-60,"5.8GHz":-70}}}]},
    ]
    write(root / "examples/single_free_request.json", free)
    for suite in ("strict","mixed","free"):
        data,_ = load_frozen(suite)
        write(root / "benchmarks" / f"{suite}_scene.json", data["scene"])
        write(root / "benchmarks" / f"{suite}_tasks.json", data["tasks"])
    # Store an actual current motion law for independent validation.
    route = plan_single_v2(**strict)
    if route["status"] != "planned":
        raise RuntimeError(route)
    payload = {**strict, **{key:route[key] for key in ("trajectory_wkt","vertex_times_s","motion_segments")}}
    payload.pop("time_budget_s",None)
    write(root / "examples/validate_request.json", payload)
    manifest = json.loads((root / "manifest.json").read_text())
    for entry in manifest["files"]:
        content = (root / entry["path"]).read_bytes()
        entry.update(bytes=len(content),sha256=hashlib.sha256(content).hexdigest())
    write(root / "manifest.json",manifest)
    for payload in (strict,prefer,free):
        result = plan_single_v2(**payload)
        if result["status"] != "planned":
            raise RuntimeError(result)
    print("Generated fixtures; ENG-06 strict/prefer and ENG-07 free plans passed.")


if __name__ == "__main__":
    main()
