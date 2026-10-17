"""Compatibility metadata for scene resources, which have no service quotas."""
from __future__ import annotations

from .models import Scene


SCENE_LIMITS: dict[str, None] = dict.fromkeys((
    "airways", "obstacles", "control_zones", "connectors", "rules", "profiles",
    "datasets", "geometry_vertices", "field_cells", "grid_entries", "grid_nodes",
    "estimated_scene_bytes",
))


def validate_scene_budget(scene: Scene) -> None:
    """Retain the former hook; correctness is validated by the Scene model."""
