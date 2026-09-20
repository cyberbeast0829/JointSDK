"""异常映射：C 返回码 → Python 异常，并附上 ``jsdk_context_last_error()`` 文本。

原则
----

* **每个非 0 返回码都抛异常**，绝不静默忽略（C 侧"不闪失败"的承诺在这里延续）。
* 异常里一定带上 **C 侧的可读文本**（含关节号、给定量、量程），因为那句话是写给
  现场工程师看的，Python 侧再包装一遍反而丢信息。
* 用**不同的异常类**区分"你参数写错了"与"设备/总线不行了"，让调用方可以：
  ``except JsdkTransportError:`` 重连，而 ``except JsdkUsageError:`` 说明这是 bug。
"""

from __future__ import annotations

from .enums import Status

__all__ = [
    "JsdkError",
    "JsdkUsageError",
    "JsdkStateError",
    "JsdkTransportError",
    "JsdkTimeoutError",
    "JsdkProtocolError",
    "JsdkDeviceNotFoundError",
    "JsdkUnsupportedError",
    "JsdkMemoryError",
    "JsdkParseError",
    "JsdkBusyError",
    "raise_for_status",
    "error_for_status",
    "LibraryNotFoundError",
    "AbiMismatchError",
]


class JsdkError(RuntimeError):
    """所有 SDK 错误的基类。

    :ivar status: :class:`~jsdk_can.enums.Status` 值
    :ivar op: 触发错误的操作名（便于定位）
    :ivar detail: C 侧 ``jsdk_context_last_error()`` 文本（可能为空串）
    """

    def __init__(self, status: int | Status, op: str = "", detail: str = ""):
        self.status = Status(int(status)) if int(status) in set(Status) else int(status)
        self.op = op
        self.detail = detail or ""

        msg = f"{op + ': ' if op else ''}{self.name()} (status={int(status)})"
        if self.detail:
            msg += f" — {self.detail}"
        super().__init__(msg)

    @staticmethod
    def name_for(status: int) -> str:
        try:
            return Status(int(status)).name.lower()
        except ValueError:
            return f"unknown({status})"

    def name(self) -> str:
        return self.name_for(int(self.status) if isinstance(self.status, Status)
                             else self.status)


class _LoadError(JsdkError):
    """加载期错误的基类（库找不到 / ABI 不匹配）。

    ⚠ 这两个错误**没有 C 返回码** —— 它们发生在能调用 C 之前，参数是一句话而不是
    ``status``。所以必须覆盖 :meth:`JsdkError.__init__`：直接用基类签名会把消息当
    状态码去 ``int()``，抛出 ``ValueError: invalid literal for int()``，
    把"库没找到"这种一眼能看懂的问题变成看不懂的问题（实测踩过）。

    ``status`` 仍保留 :attr:`Status.NOT_FOUND` 占位，以免调用方读属性时炸掉；
    判断"是不是加载期问题"请用 ``isinstance(e, _LoadError)`` 或具体类。
    """

    def __init__(self, message: str) -> None:
        RuntimeError.__init__(self, str(message))     # args=(msg,) → str(e) 即原文
        self.status = Status.NOT_FOUND
        self.op = "load_library"
        self.detail = str(message)


class LibraryNotFoundError(_LoadError, OSError):
    """找不到 ``libjsdk_can``。

    同时是 ``OSError``（"加载器/文件问题"仍能被常规写法捕获）与
    :class:`JsdkError`（这样 ``except JsdkError`` 能兜住 SDK 抛出的**一切**，
    不会漏掉最外层的部署问题）。
    """


class AbiMismatchError(_LoadError):
    """加载到的库与绑定不是一套（结构体布局、后端标签或必需符号不符）。

    ⚠ 归入 :class:`JsdkError` 家族是刻意的：它不属于"设备出错了"，而是**部署错了**，
    但用户写 ``except JsdkError`` 兜底时不应该被它穿过去。

    ⚠ 为什么这两个类定义在这里（而不是 ``_abi.py``）：
    ``enums.py`` 依赖 ``_abi`` 里的常量，若 ``_abi`` 再在模块级 import ``errors``
    就成环。异常类本身只依赖 :class:`JsdkError`，放在这一层没有任何代价。
    """


class JsdkUsageError(JsdkError):
    """调用方参数非法（``INVALID_ARG`` / ``NOT_FOUND``）。属**编程错误**。"""


class JsdkStateError(JsdkError):
    """当前状态不允许该操作（``BAD_STATE``）。

    典型：在关节使能时下载描述符、未 ``configure()`` 就用物理量 API。
    """


class JsdkTransportError(JsdkError):
    """链路/HAL 层错误（``TRANSPORT``）。总线掉线、驱动异常。"""


class JsdkTimeoutError(JsdkError):
    """等待响应/状态变化超时（``TIMEOUT``）。"""


class JsdkProtocolError(JsdkError):
    """对端返回非法值或错误码（``PROTOCOL``）。"""


class JsdkDeviceNotFoundError(JsdkError):
    """设备/接口不存在（``NOT_FOUND``）。

    HAL 层也用它表示"没装驱动"（如 PCANBasic 缺失）。
    """


class JsdkUnsupportedError(JsdkError):
    """当前构建/后端不支持（``UNSUPPORTED``）。

    例如没编译某个 HAL 后端、位定时组合不在表里、slcan 发 FD 帧。
    """


class JsdkMemoryError(JsdkError):
    """内存不足（``NO_MEMORY``）。多数情况是 arena 给小了。"""


class JsdkParseError(JsdkError):
    """JSON 描述符解析失败（``PARSE``）。"""


class JsdkBusyError(JsdkError):
    """请求进行中 / 队列满（``BUSY``）。"""


_MAP: dict[Status, type[JsdkError]] = {
    Status.INVALID_ARG: JsdkUsageError,
    Status.NO_MEMORY: JsdkMemoryError,
    Status.NOT_FOUND: JsdkDeviceNotFoundError,
    Status.BAD_STATE: JsdkStateError,
    Status.TRANSPORT: JsdkTransportError,
    Status.UNSUPPORTED: JsdkUnsupportedError,
    Status.TIMEOUT: JsdkTimeoutError,
    Status.PROTOCOL: JsdkProtocolError,
    Status.BUSY: JsdkBusyError,
    Status.PARSE: JsdkParseError,
}


def error_for_status(status: int, op: str = "", detail: str = "") -> JsdkError:
    """构造（但不抛出）对应异常 —— 供需要自己处理的分支使用。"""
    try:
        st = Status(int(status))
    except ValueError:
        return JsdkError(status, op, detail)
    return _MAP.get(st, JsdkError)(st, op, detail)


def raise_for_status(status: int, op: str = "", detail: str = "") -> None:
    """``status != 0`` 时抛出对应异常。"""
    if int(status) == 0:
        return
    raise error_for_status(status, op, detail)
