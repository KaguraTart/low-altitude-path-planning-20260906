"""
统一错误体系

对外接口的错误分两类，调用方需要区分对待：

1. **业务失败**（如"起点被障碍物占用"、"未找到可行路径"）
   —— 不抛异常，正常返回 ``status="failed"``，并在 ``message`` 里说明原因。
   这类"规划不出来"是路径规划的常态结果，不是程序错误。

2. **调用错误**（如参数类型不对、引用了不存在的飞行器）
   —— 抛 :class:`PlanningError`，携带稳定的 ``code`` 供程序判断，
   不需要调用方去解析错误文本。

HTTP 层会把 ``PlanningError.code`` 映射为合适的 HTTP 状态码，
见 ``server.app``。
"""

from __future__ import annotations

from typing import Any, Dict, Optional


class ErrorCode:
    """稳定的错误码常量（字符串值即对外契约，不要随意改动）"""

    INVALID_INPUT = "INVALID_INPUT"
    """入参不合法：字段缺失、类型错误、数值越界、权重和不为 1 等"""

    TASK_NOT_FOUND = "TASK_NOT_FOUND"
    """按 ID 找不到任务"""

    AIRCRAFT_NOT_FOUND = "AIRCRAFT_NOT_FOUND"
    """任务引用的飞行器未在配置中定义"""

    PLANNING_FAILED = "PLANNING_FAILED"
    """规划整体无法进行（区别于单条航线 status=failed）"""

    TIMEOUT = "TIMEOUT"
    """超过调用方给定的时间预算"""

    INTERNAL_ERROR = "INTERNAL_ERROR"
    """未预期的内部异常"""


class PlanningError(Exception):
    """
    调用方错误，携带机器可读的错误码

    Attributes:
        code: 见 :class:`ErrorCode`
        message: 人类可读的描述（中文）
        detail: 附加上下文，如出错字段名、非法值、期望范围
    """

    def __init__(
        self,
        code: str,
        message: str,
        detail: Optional[Dict[str, Any]] = None,
    ):
        super().__init__(message)
        self.code = code
        self.message = message
        self.detail = detail or {}

    def to_dict(self) -> Dict[str, Any]:
        """序列化为统一错误信封"""
        return {
            "error": {
                "code": self.code,
                "message": self.message,
                "detail": self.detail,
            }
        }

    def __repr__(self) -> str:  # pragma: no cover - 仅调试用
        return f"PlanningError(code={self.code!r}, message={self.message!r})"
