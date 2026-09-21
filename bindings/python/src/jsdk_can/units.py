"""输出端换算系数（``jsdk_unit_scale_t``）—— 与 EtherCAT 家族同名同义。

CAN 后端里线上量**已经是输出端物理量**（rad / rad/s / N·m），所以
:func:`unit_scale_default` 给的是**恒等映射**；这组 API 主要给两类人用：

1. 从 EtherCAT 版迁移过来的代码（那边 ``counts`` → ``rad`` 需要非平凡的系数）；
2. 设备标定不可用、但手里有权威换算表时，用 :meth:`Joint.set_scale` 显式覆盖。

⚠ 语义差异是真实的：**同名不同义**（CAN 版恒等，EtherCAT 版是 counts→rad）。
需要按名字推断前请先读 :attr:`UnitScale.valid` 与本节说明。
"""

from __future__ import annotations

import ctypes

from . import _abi

__all__ = ["UnitScale", "unit_scale_default", "unit_scale_calc"]


class UnitScale:
    """``jsdk_unit_scale_t`` 的 Python 视图。

    :param pos_counts_to_rad: 位置：1 命令单位 = N rad
    :param vel_counts_to_rad_s: 速度：1 命令单位/s = N rad/s
    :param trq_to_Nm: 力矩：1 命令单位 = N N·m
    :param valid: 0 = 尚未从设备取得标定参数
    """

    __slots__ = ("_c",)

    def __init__(self, pos_counts_to_rad: float = 0.0,
                 vel_counts_to_rad_s: float = 0.0,
                 trq_to_Nm: float = 0.0, valid: int = 0) -> None:
        c = _abi.UnitScale()
        c.pos_counts_to_rad = float(pos_counts_to_rad)
        c.vel_counts_to_rad_s = float(vel_counts_to_rad_s)
        c.trq_to_Nm = float(trq_to_Nm)
        c.valid = int(valid)
        self._c = c

    # --- 构造 -------------------------------------------------------------

    @classmethod
    def _from_c(cls, c: _abi.UnitScale) -> "UnitScale":
        return cls(c.pos_counts_to_rad, c.vel_counts_to_rad_s, c.trq_to_Nm,
                   c.valid)

    @classmethod
    def from_dict(cls, d) -> "UnitScale":
        """从映射/对象（含 ``pos_counts_to_rad`` 等字段）构造。"""
        get = d.get if isinstance(d, dict) else (lambda k, dv=None: getattr(d, k, dv))
        return cls(get("pos_counts_to_rad", 0.0), get("vel_counts_to_rad_s", 0.0),
                   get("trq_to_Nm", 0.0), get("valid", 1))

    # --- 字段 -------------------------------------------------------------

    @property
    def pos_counts_to_rad(self) -> float:
        return float(self._c.pos_counts_to_rad)

    @property
    def vel_counts_to_rad_s(self) -> float:
        return float(self._c.vel_counts_to_rad_s)

    @property
    def trq_to_Nm(self) -> float:
        return float(self._c.trq_to_Nm)

    @property
    def valid(self) -> int:
        return int(self._c.valid)

    def as_dict(self) -> dict:
        return {"pos_counts_to_rad": self.pos_counts_to_rad,
                "vel_counts_to_rad_s": self.vel_counts_to_rad_s,
                "trq_to_Nm": self.trq_to_Nm, "valid": self.valid}

    def __repr__(self) -> str:  # pragma: no cover - 人读
        return (f"UnitScale(pos={self.pos_counts_to_rad}, "
                f"vel={self.vel_counts_to_rad_s}, trq={self.trq_to_Nm}, "
                f"valid={self.valid})")


def unit_scale_default(rated_trq: int = 0) -> UnitScale:
    """``jsdk_unit_scale_default()``：CAN 后端的**恒等映射**（线上量已是物理量）。

    :param rated_trq: 额定力矩（N·m）；CAN 后端只用于内部自洽检查，不参与换算。
    """
    c = _abi.UnitScale()
    _abi.lib().jsdk_unit_scale_default(ctypes.byref(c), int(rated_trq))
    return UnitScale._from_c(c)


def unit_scale_calc(encoder_resolution: int, motor_rev: int, shaft_rev: int,
                    rated_torque: int) -> UnitScale:
    """``jsdk_unit_scale_calc()``：按编码器分辨率与齿轮比算出系数。

    :param encoder_resolution: 电机端每圈计数
    :param motor_rev: 电机转数
    :param shaft_rev: 输出轴转数（``motor_rev / shaft_rev`` = 减速比）
    :param rated_torque: 额定力矩（N·m）
    """
    c = _abi.UnitScale()
    _abi.lib().jsdk_unit_scale_calc(
        ctypes.byref(c), int(encoder_resolution), int(motor_rev),
        int(shaft_rev), int(rated_torque))
    return UnitScale._from_c(c)
