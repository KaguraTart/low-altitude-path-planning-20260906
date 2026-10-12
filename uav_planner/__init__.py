"""
低空无人机路径规划算法 —— 对外 Python SDK

把项目内部的 ``src.*`` 实现封装成一组稳定的函数式接口，让调用方无需了解
内部目录结构与 dataclass 细节即可使用算法：

    from uav_planner import plan_single

    result = plan_single(
        start={"x": 500, "y": 500, "z": 100},
        goal={"x": 2500, "y": 2500, "z": 150},
    )
    print(result["status"], result["route"]["metrics"])

所有入参与返回值都是纯 JSON 结构，可直接 ``json.dumps``。
字段级契约见 ``docs/API.md``；起 HTTP 服务见 ``serve.py``。
"""

from __future__ import annotations

import os as _os
import sys as _sys

# 项目根目录入 sys.path，使 `import src.*`（以及复用顶层 CLI 脚本的编排函数）
# 在「被别处 import」时同样可用，而不必依赖调用方先设好 PYTHONPATH。
_ROOT = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
if _ROOT not in _sys.path:
    _sys.path.insert(0, _ROOT)

from .api import (  # noqa: E402
    ALGORITHMS,
    get_version,
    plan_batch,
    plan_from_dir,
    plan_single,
    validate_route,
)
from .adapters import DEFAULT_AIRCRAFT, DEFAULT_CONFIG  # noqa: E402
from .errors import ErrorCode, PlanningError  # noqa: E402
from .serializer import dumps_safe, sanitize  # noqa: E402

__version__ = "1.0.0"

__all__ = [
    # 核心接口
    "plan_single",
    "plan_batch",
    "plan_from_dir",
    "validate_route",
    "get_version",
    # 错误体系
    "PlanningError",
    "ErrorCode",
    # 默认值与工具
    "DEFAULT_CONFIG",
    "DEFAULT_AIRCRAFT",
    "ALGORITHMS",
    "sanitize",
    "dumps_safe",
    "__version__",
]
