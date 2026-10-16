"""Explicit operational scene bounds checked before geometric compilation."""
from __future__ import annotations

from shapely import from_wkt, get_num_coordinates

from uav_planner.errors import ErrorCode, PlanningError
from .models import Scene


SCENE_LIMITS = {
    "airways": 200, "obstacles": 1000, "control_zones": 20,
    "connectors": 1000, "rules": 1000, "profiles": 32, "datasets": 100,
    "geometry_vertices": 50000, "field_cells": 1_200_000,
    "grid_entries": 1_200_000, "grid_nodes": 4_000_000,
    "estimated_scene_bytes": 512*1024*1024,
}


def validate_scene_budget(scene: Scene) -> int:
    """Return a conservative storage estimate or reject an oversized scene."""
    def check(name: str, count: int) -> None:
        if count > SCENE_LIMITS[name]:
            raise PlanningError(ErrorCode.INVALID_INPUT,"Scene resource budget exceeded",
                                {"resource":name,"actual":count,"limit":SCENE_LIMITS[name]})
    for name in ("airways","obstacles","control_zones","connectors","rules","profiles","datasets"):
        check(name,len(getattr(scene,name)))
    vertices=0
    def walk(value):
        nonlocal vertices
        if isinstance(value,dict):
            for name,item in value.items():
                if isinstance(item,str) and (name.endswith("wkt") or name in {"centerLine","geom","boundary2d","centerPoint"}):
                    vertices+=int(get_num_coordinates(from_wkt(item)))
                else:
                    walk(item)
        elif isinstance(value,list):
            for item in value:
                walk(item)
    walk(scene.model_dump(mode="json",exclude_none=True))
    check("geometry_vertices",vertices)
    field_cells=sum(len(d.cells) for d in scene.datasets)
    check("field_cells",field_cells)
    entries=0
    if scene.grid:
        grid=scene.grid
        entries=len(grid.occupied)+len(grid.occupied_ranges)+len(grid.cells)
        check("grid_entries",entries)
        check("grid_nodes",grid.shape[0]*grid.shape[1]*grid.shape[2])
    # Includes Python models, GEOS/prepared objects, indexes and a raster cache.
    estimated=len(scene.model_dump_json())*4+vertices*128+(field_cells+entries)*2048+24*1024*1024
    check("estimated_scene_bytes",estimated)
    return estimated
