"""
内部数据模型 → 对外 JSON 结构 的序列化层

对外接口只承诺返回**纯 JSON 可序列化**的结构（dict / list / str / int /
float / bool / None）。本模块把 ``src.models`` 的 dataclass 转成这种结构，
并做一次「JSON 安全性」清洗。

为什么要清洗：轨迹指标里可能出现 ``NaN`` / ``Infinity``（例如空轨迹、
除零），标准 JSON 不允许这两个字面量。若直接交给 Web 框架，响应会带着
非法字面量发出去，对方的 JSON 解析器会报错——这是联调时最难排查的一类问题，
所以在边界上一次性挡掉。
"""

from __future__ import annotations

import json
import math
from typing import Any, List, Optional

import numpy as np

from src.models.task import PlannedRoute, PlanningResult
from src.validator import ValidationResult


def sanitize(obj: Any) -> Any:
    """
    递归把内部结构转成原生 Python 类型，并把非有限浮点数（``NaN`` /
    ``Inf``）替换为 ``None``

    做两件事，都是为了让返回值能被任何 JSON 库安全消费：

    1. **numpy 标量 → 原生类型**。内部指标由 numpy 算出，``np.float64``
       虽然能骗过 ``isinstance(x, float)``（它确实继承自 float），但
       ``repr`` 出来是 ``np.float64(4037.1)``，调用方做类型判断、格式化或
       写入某些严格序列化器时会出意外。这里统一降级成 ``float`` / ``int`` /
       ``bool``。
    2. **非有限值 → None**。标准 JSON 没有 ``NaN`` / ``Infinity`` 字面量。

    返回新对象，不修改入参。
    """
    if isinstance(obj, np.generic):
        # np.float64 -> float, np.int64 -> int, np.bool_ -> bool
        obj = obj.item()

    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, dict):
        return {k: sanitize(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [sanitize(v) for v in obj]
    if isinstance(obj, (str, int, bool)) or obj is None:
        return obj
    if isinstance(obj, np.ndarray):
        # 数组只可能出现在内部结构里，不应出现在对外契约中；
        # 显式暴露而非静默转字符串，便于尽早发现契约泄漏。
        raise TypeError(
            f"对外结构中出现 numpy 数组（shape={obj.shape}），"
            "序列化层不应透出数组，请检查调用方是否漏取了标量"
        )
    return str(obj)


def route_to_dict(route: PlannedRoute) -> dict:
    """单条航线 → dict（含指标、轨迹点、规划解释）"""
    return sanitize(route.to_dict())


def routes_to_list(routes: List[PlannedRoute]) -> List[dict]:
    """航线列表 → dict 列表"""
    return [route_to_dict(r) for r in routes]


def result_to_dict(result: PlanningResult) -> dict:
    """
    批量规划结果 → dict

    ``PlanningResult.summary()`` 已包含 request_id / 计数 / 航线列表；
    这里补上冲突列表，并做 JSON 清洗。
    """
    out = dict(result.summary())
    out["conflicts"] = sanitize(list(getattr(result, "conflicts", []) or []))
    out["conflict_count"] = len(out["conflicts"])
    return sanitize(out)


def validation_to_dict(validation: Optional[ValidationResult]) -> Optional[dict]:
    """验证结果 → dict；``None`` 原样返回"""
    if validation is None:
        return None
    return sanitize(validation.to_dict())


def dumps_safe(obj: Any, **kwargs) -> str:
    """
    ``json.dumps`` 的严格版：禁止 ``NaN`` / ``Infinity`` 字面量

    经过 :func:`sanitize` 之后本不该再出现非有限值；这里 ``allow_nan=False``
    作为最后一道防线，一旦出现就抛错而不是发出去一个对方解析不了的响应。
    """
    kwargs.setdefault("ensure_ascii", False)
    return json.dumps(sanitize(obj), allow_nan=False, **kwargs)
