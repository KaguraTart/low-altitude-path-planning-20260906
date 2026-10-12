"""REST HTTP 服务包（FastAPI），封装自 uav_planner SDK"""

from .app import app  # noqa: F401

__all__ = ["app"]
