"""validator 包 - 路径合法性验证模块（任务书 §六 模块架构）

提供：
- PathValidator: 路径合法性校验（连续性、约束、冲突）
- ValidationResult: 验证结果结构化输出
- ValidationIssue / Severity: 单条问题 + 严重等级

任务书 §十二 验收标准：
"路径无静态障碍冲突 / 满足飞行器性能约束 / 多机情况下无航线冲突"
这些都需要独立的验证模块支撑。
"""
from .path_validator import (
    PathValidator, ValidationResult, ValidationIssue, Severity,
)

__all__ = ["PathValidator", "ValidationResult", "ValidationIssue", "Severity"]