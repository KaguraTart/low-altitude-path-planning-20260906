"""Compile supplied rule geometry into local ellipsoid-height volumes."""
from __future__ import annotations

import numpy as np
from shapely import force_2d, from_wkt, get_coordinates
from shapely.geometry import box

from .rules import invalid


def compile_scopes(scene):
    from .scene import PreparedVolume

    result = {}
    for rule in scene.model.rules:
        if rule.scope is None:
            continue
        volumes = []

        def append(geometry, lo, hi):
            volumes.append(PreparedVolume(
                id=f"scope:{rule.id}:{len(volumes)}", geometry=geometry,
                min_altitude=float(lo), max_altitude=float(hi), alt_reference="EG",
                kind="rule_scope", source=rule,
                valid_from_s=scene._seconds(rule.effective_start, -float("inf")),
                valid_to_s=scene._seconds(rule.effective_end, float("inf")),
            ))

        def geometry_volume(wkt, boundary=None):
            source = from_wkt(wkt)
            local = scene.frame.to_local(wkt, "EG", scene.time_origin)
            footprint = force_2d(local)
            if boundary:
                boundary_shape = scene._footprint(boundary)
                if source.geom_type != "Point" and footprint.hausdorff_distance(boundary_shape) > .1:
                    invalid("scope geom and boundary_2d footprints disagree", rule_id=rule.id)
                if source.geom_type == "Point" and not boundary_shape.covers(footprint):
                    invalid("scope landing point is outside boundary_2d", rule_id=rule.id)
                if source.geom_type == "Point":
                    footprint = boundary_shape
            if source.has_z:
                z = get_coordinates(local, include_z=True)[:, 2]
                lo, hi = float(z.min()), float(z.max())
            else:
                lo, hi = scene.bounds.z_min, scene.bounds.z_max
            append(footprint, lo, hi)

        for item in rule.scope.grid:
            if item.geometry:
                geometry_volume(item.geometry)
            else:
                grid = scene.model.grid
                if grid is None:
                    invalid("scope.grid indices require scene.grid origin, resolution and shape", rule_id=rule.id)
                index = np.array([item.x, item.y, item.z])
                if np.any(index >= grid.shape):
                    invalid("scope.grid index exceeds scene.grid shape", rule_id=rule.id)
                lo = np.asarray(grid.origin) + index * np.asarray(grid.resolution)
                hi = lo + np.asarray(grid.resolution)
                append(box(lo[0], lo[1], hi[0], hi[1]), lo[2], hi[2])
        for item in rule.scope.geom:
            # A center is metadata when a boundary exists. It does not set a
            # volume's height. Only a supplied 3D geom sets its height interval.
            if item.geom:
                geometry_volume(item.geom, item.boundary_2d)
            elif item.boundary_2d:
                geometry_volume(item.boundary_2d)
            else:
                geometry_volume(item.center_point)
        result[rule.id] = volumes
    return result
