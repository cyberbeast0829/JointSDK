cmake_minimum_required(VERSION 3.16)

project(joint_sdk_can
        VERSION 0.1.0
        DESCRIPTION "CyberBeast Joint SDK - CYBERBEAST CAN/CAN-FD backend"
        LANGUAGES C)

# ---------------------------------------------------------------------------
# 编译标准与告警
#
# C99：与两个 EtherCAT 后端一致，覆盖绝大多数 MCU 工具链（Keil ARMCC5 / IAR / GCC）。
# 不使用 VLA、不使用 C11 atomics/threads、不使用编译器扩展。
# ---------------------------------------------------------------------------
# ⚠ 必须**在目标定义之前** include：下面目标的 `$<INSTALL_INTERFACE:...>` 里
#   用到了 `${CMAKE_INSTALL_INCLUDEDIR}`，而这个变量由本模块定义 —— 放晚了
#   会直接拼出一个 `$<INSTALL_INTERFACE:/joint_sdk>` 这种空路径。
include(GNUInstallDirs)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

option(JSDK_BUILD_TESTS  "Build unit tests" ON)
option(JSDK_WERROR        "Treat warnings as errors" OFF)
option(JSDK_BUILD_HAL_VIRTUAL "Build the virtual CAN backend + device model" ON)
# 真实后端：默认按平台开启。全都需要操作系统句柄，MCU 构建关掉即可。
if(WIN32)
  set(_jsdk_hal_socketcan_default OFF)
  set(_jsdk_hal_pcan_default     ON)
  set(_jsdk_hal_slcan_default    ON)
elseif(APPLE)
  set(_jsdk_hal_socketcan_default OFF)
  set(_jsdk_hal_pcan_default     ON)
  set(_jsdk_hal_slcan_default    ON)
else()
  set(_jsdk_hal_socketcan_default ON)
  set(_jsdk_hal_pcan_default      OFF)
  set(_jsdk_hal_slcan_default     ON)
endif()
option(JSDK_BUILD_HAL_SOCKETCAN "Build the Linux SocketCAN backend" ${_jsdk_hal_socketcan_default})
option(JSDK_BUILD_HAL_PCAN      "Build the PEAK PCAN-Basic backend (Windows/macOS)" ${_jsdk_hal_pcan_default})
option(JSDK_BUILD_HAL_SLCAN     "Build the serial slcan backend" ${_jsdk_hal_slcan_default})
# 堆模式（jsdk_context_create/free）。**MCU 构建不要打开**。
option(JSDK_ENABLE_HEAP   "Build the optional heap-based context helpers" OFF)
# PC 诊断 CLI（tools/jsdk_cli）。它用堆分配上下文与 arena，所以**默认跟随
# JSDK_ENABLE_HEAP**：堆关着时不冒然打开，免得默认 configure 就报错。
if(JSDK_ENABLE_HEAP)
  set(_jsdk_cli_default ON)
else()
  set(_jsdk_cli_default OFF)
endif()
option(JSDK_BUILD_CLI     "Build jsdk-cli (requires JSDK_ENABLE_HEAP=ON)" ${_jsdk_cli_default})
# 共享库（Python 绑定 / 桌面工具通过 FFI 加载）。客户侧静态链接时保持 OFF。
option(JSDK_BUILD_SHARED  "Build jsdk_can as a SHARED library (needed by bindings/python)" OFF)

set(JSDK_WARN_FLAGS "")
if(MSVC)
  # ⚠ MSVC 的 C 默认是 **C89 + 扩展**："声明必须在语句之前" 会被当成语法错误
  #   （真在 `cb_ctrl.c` 的 `if (classic) { int16_t vel_raw = …; }` 上撞到：
  #    `error C2059: 语法错误:"if"`）→ 必须显式 /std:c11。
  # ⚠ `/utf-8`：源码是 UTF-8（注释里有中文），而中文 Windows 的默认代码页是 936，
  #   不给它就会逐文件刷 C4819（"不能在当前代码页中表示的字符"）。
  list(APPEND JSDK_WARN_FLAGS /W4 /std:c11 /utf-8)
  # C4127“条件表达式是常量”：`do { … } while (0)` 宏（测试里的 CHECK 系列全是这种）
  # 在 /W4 下必然误报，且是社区公认的假阳性（C11 后依然如此）→ 明确关闭。
  list(APPEND JSDK_WARN_FLAGS /wd4127)
  if(JSDK_WERROR)
    list(APPEND JSDK_WARN_FLAGS /WX)
  endif()
else()
  list(APPEND JSDK_WARN_FLAGS -Wall -Wextra -Wpedantic -Wconversion -Wshadow
                              -Wstrict-prototypes -Wmissing-prototypes)
  if(JSDK_WERROR)
    list(APPEND JSDK_WARN_FLAGS -Werror)
  endif()
endif()

# C++ 版本：去掉两个**只对 C 有意义**的告警，否则 g++ 会刷
#   "command-line option '-Wstrict-prototypes' is valid for C/ObjC but not for C++"
# 这类噪声（它不报错，但会让日志里混进 2 行永久警告）。
# MSVC：C 运行时把 fopen/strcpy 等标为“不安全”（C4996）。本库是**纯 C99 可移植**代码，
# 那些 `_s` 变体在 gcc/clang 上不存在，且用法都是定长缓冲，所以按社区通行做法关掉该提示。
#
# ⚠ `_USE_MATH_DEFINES`：MSVC 的 `<math.h>` **默认不定义 `M_PI`**，而 gcc/clang 有。
#   不给它，测试里用 `M_PI` 的地方全部报 C2065（真撞过）。
if(MSVC)
  add_compile_definitions(_CRT_SECURE_NO_WARNINGS _USE_MATH_DEFINES)
endif()

set(JSDK_WARN_FLAGS_CXX "")
if(MSVC)
  list(APPEND JSDK_WARN_FLAGS_CXX /W4 /utf-8)
  if(JSDK_WERROR)
    list(APPEND JSDK_WARN_FLAGS_CXX /WX)
  endif()
else()
  list(APPEND JSDK_WARN_FLAGS_CXX -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
  if(JSDK_WERROR)
    list(APPEND JSDK_WARN_FLAGS_CXX -Werror)
  endif()
endif()

# ---------------------------------------------------------------------------
# 库：jsdk_can
#
# 分层：
#   src/proto_cyberbeast/*   L2 协议层（帧编解码 / MIT / 控制 / 查询 / 心跳 /
#                            参数 / JSON 描述符收发与解析 / 缓存）
#   src/core/*               L3 关节层（上下文 / 关节 / 看门狗 / 运维 / 分组）
#   src/hal/*                内置 HAL 后端（可选：virtual / socketcan / pcan / slcan）
#
# **为什么源文件清单要先算出来、再建目标**
#   需要两种编译产物，且**编译参数必须不同**：
#     - `jsdk_can`（静态，默认）：给客户与测试用。测试会直接使用内部接口
#       （`jsdk_jsondesc_error`、`cb_*`、`sim_*`），所以它**必须**保留全部符号。
#     - `jsdk_can_shared`（`JSDK_BUILD_SHARED=ON`）：给 Python 绑定等 FFI 用。
#       定义 `JSDK_SHARED` → 公共头的 `JSDK_API` 展开为 dllexport /
#       `visibility("default")`，配合 `C_VISIBILITY_PRESET hidden`
#       **只导出公共 ABI**；内部符号不外泄（这正是"符号同名、互斥链接"
#       那条家族约束要的：`cb_*` 不该和 EtherCAT 版后端在一个进程里撞名）。
#   两者编译同一批源文件两次，代价是构建时间；换来的是"内部符号确实被藏住"
#   这一条能被链接器**证明**，而不是靠约定。
# ---------------------------------------------------------------------------

set(JSDK_SOURCES
  src/proto_cyberbeast/cb_jsondesc_parse.c
  src/proto_cyberbeast/cb_frame.c
  src/proto_cyberbeast/cb_mit.c
  src/proto_cyberbeast/cb_ctrl.c
  src/proto_cyberbeast/cb_query.c
  src/proto_cyberbeast/cb_heartbeat.c
  src/proto_cyberbeast/cb_param.c
  src/proto_cyberbeast/cb_jsondesc_fetch.c
  src/proto_cyberbeast/cb_desc_cache.c
  src/core/jsdk_text.c
  src/core/jsdk_units.c
  src/core/jsdk_context.c
  src/core/jsdk_joint.c
  src/core/jsdk_watchdog.c
  src/core/jsdk_desc.c
  src/core/jsdk_config.c
  src/core/jsdk_fault.c
  src/core/jsdk_ops.c
  src/core/jsdk_param.c
  src/core/jsdk_group.c

  # slcan 的 ASCII 编解码是**纯逻辑**（只依赖 string.h），与串口 I/O 无关，
  # 所以无论是否构建 slcan 后端都编进来：它的单元测试因此能在**每个**配置下
  # 跑到（而且关掉后端时 test_hal 不会因为链接不到 jsdk_slcan_encode 而挂）。
  src/hal/hal_slcan_codec.c
)

set(JSDK_DEFINES "")

# 内置 HAL：虚拟后端 + 设备模型（桌面平台；位于零 malloc 核心之外）
if(JSDK_BUILD_HAL_VIRTUAL)
  list(APPEND JSDK_SOURCES src/hal/hal_virtual.c src/hal/sim_device.c)
  list(APPEND JSDK_DEFINES JSDK_HAL_HAVE_VIRTUAL=1)
endif()

# 内置 HAL：真实后端。
#   hal_common.c 提供 jsdk_hal_close() 与"未编译后端"的占位，只要有一个后端
#   启用就必须编进去 —— 否则客户只开 socketcan 时链接不到 jsdk_hal_close()。
set(JSDK_HAL_COMMON_NEEDED OFF)
if(JSDK_BUILD_HAL_SOCKETCAN OR JSDK_BUILD_HAL_PCAN OR
   JSDK_BUILD_HAL_SLCAN OR JSDK_BUILD_HAL_VIRTUAL)
  set(JSDK_HAL_COMMON_NEEDED ON)
  list(APPEND JSDK_SOURCES src/hal/hal_common.c)
endif()

if(JSDK_BUILD_HAL_SOCKETCAN)
  if(NOT UNIX OR APPLE)
    message(FATAL_ERROR "JSDK_BUILD_HAL_SOCKETCAN=ON requires Linux "
                        "(SocketCAN is a Linux kernel interface)")
  endif()
  list(APPEND JSDK_SOURCES src/hal/hal_socketcan.c)
  list(APPEND JSDK_DEFINES JSDK_HAL_HAVE_SOCKETCAN=1)
endif()

if(JSDK_BUILD_HAL_PCAN)
  list(APPEND JSDK_SOURCES src/hal/hal_pcan.c)
  list(APPEND JSDK_DEFINES JSDK_HAL_HAVE_PCAN=1)
endif()

if(JSDK_BUILD_HAL_SLCAN)
  list(APPEND JSDK_SOURCES src/hal/hal_slcan.c)
  list(APPEND JSDK_DEFINES JSDK_HAL_HAVE_SLCAN=1)
endif()

if(JSDK_ENABLE_HEAP)
  list(APPEND JSDK_SOURCES src/hal/heap_optional.c)
  list(APPEND JSDK_DEFINES JSDK_ENABLE_HEAP=1)
endif()

# 把"所有目标都需要的设置"集中一处，避免静态/共享两份配置漂移。
function(jsdk_configure_target tgt)
  # 公开头之间用**引号形式**互相引用（`#include "joint_sdk.h"`，见 jsdk_hal_builtin.h），
  # 所以两个 include 目录都要暴露给使用者；安装后对应 <prefix>/include 与
  # <prefix>/include/joint_sdk（pkg-config 的 `Cflags` 里也是两个 `-I`）。
  target_include_directories(${tgt}
    PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/joint_sdk>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/joint_sdk>
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
            ${CMAKE_CURRENT_SOURCE_DIR}/src/proto_cyberbeast
            ${CMAKE_CURRENT_SOURCE_DIR}/src/hal)
  target_compile_options(${tgt} PRIVATE ${JSDK_WARN_FLAGS})
  if(UNIX)
    # PCAN 是运行期 dlopen 的，需要 libdl
    target_link_libraries(${tgt} PUBLIC ${CMAKE_DL_LIBS})

    # ⚠ 必须链 `libm`：核心用到 `floor()`（jsdk_units.c 的位置展开）。
    #   glibc 把数学函数放在**独立的** libm 里，而 MinGW 的 libm 属于 C 运行时、
    #   自动就能链上 —— 所以这个缺口在 Windows 上永远看不到，直到第一次 Linux
    #   构建：`undefined reference to 'floor'`，而且**连库带 CLI/测试一起失败**。
    #   用 PUBLIC 传递：静态库的使用者（CLI、测试、客户程序）也就自动拿到了。
    target_link_libraries(${tgt} PUBLIC m)
  endif()
endfunction()

# --- 静态库（默认；客户与测试都用它，保留全部内部符号） ---
add_library(jsdk_can STATIC ${JSDK_SOURCES})
jsdk_configure_target(jsdk_can)
target_compile_definitions(jsdk_can PRIVATE ${JSDK_DEFINES})
set_target_properties(jsdk_can PROPERTIES
  C_VISIBILITY_PRESET hidden
  POSITION_INDEPENDENT_CODE ON
  # `find_package(jsdk_can)` 后客户写的是 `jsdk::can`（不是 jsdk::jsdk_can）
  EXPORT_NAME can)
add_library(jsdk_can::jsdk_can ALIAS jsdk_can)

# --- 共享库（可选；Python 绑定等 FFI 用，只导出公共 ABI） ---
if(JSDK_BUILD_SHARED)
  add_library(jsdk_can_shared SHARED ${JSDK_SOURCES})
  jsdk_configure_target(jsdk_can_shared)
  target_compile_definitions(jsdk_can_shared PRIVATE ${JSDK_DEFINES} JSDK_SHARED=1)
  set_target_properties(jsdk_can_shared PROPERTIES
    OUTPUT_NAME       jsdk_can            # → libjsdk_can.so / jsdk_can.dll
    C_VISIBILITY_PRESET hidden
    POSITION_INDEPENDENT_CODE ON
    VERSION   ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR})

  # ⚠⚠ Windows 上**导入库与静态库会同名**：静态库是 `jsdk_can.lib`，
  #   而 MSVC 给 DLL 生成的导入库默认也叫 `jsdk_can.lib` —— 两者落在同一个
  #   输出目录里，谁后写谁覆盖谁。于是 `-DJSDK_BUILD_SHARED=ON` 时，测试/客户
  #   链接到的其实是**导入库**，内部符号（`jsdk_jsondesc_*`、`jsdk_ep_store_*`…）
  #   全部 LNK2019（真撞过：只有 MSVC 会这样，MinGW 的 `libjsdk_can.a` 与
  #   `libjsdk_can.dll.a` 名字不同，所以一直没暴露）。
  #   Python 绑定按**文件路径**加载 DLL，不依赖导入库名字，所以把导入库改名即可。
  if(WIN32)
    set_target_properties(jsdk_can_shared PROPERTIES ARCHIVE_OUTPUT_NAME jsdk_can_dll)
  endif()

  # 构建完成时把共享库路径报出来：Python 侧要靠它（或 JSDK_LIB_PATH）找到库
  add_custom_command(TARGET jsdk_can_shared POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo
            "jsdk_can shared library: $<TARGET_FILE:jsdk_can_shared>")
endif()

# ---------------------------------------------------------------------------
# Python 绑定（bindings/python）—— 仅在构建共享库时可用
#
# 设计取舍：**不**用 scikit-build / maturin 之类把 ctypes 包"编译"进 CMake。
# 包本身是纯 Python（ctypes），唯一的构建产物就是共享库；`pip install -e .`
# 与 `cmake --build` 谁先谁后都行，只要库能被找到（JSDK_LIB_PATH 或包内 lib/）。
# 这里只做两件事：把测试挂进 ctest，并把共享库复制进包内 lib/ 便于分发。
# ---------------------------------------------------------------------------
option(JSDK_BUILD_PYTHON "Install the ctypes bindings into the Python bindings dir"
       OFF)
set(JSDK_PYTHON_DIR "${CMAKE_CURRENT_SOURCE_DIR}/bindings/python"
    CACHE PATH "Path to the Python bindings directory")

if(JSDK_BUILD_SHARED AND JSDK_BUILD_PYTHON)
  # 把共享库拷进 <bindings/python>/src/jsdk_can/lib/，这样 `pip install .` 出来的
  # 包自带库，不需要用户再设 JSDK_LIB_PATH。
  #
  # ⚠ **必须显式指定目标文件名**（曾经踩过，B9）：
  #   `$<TARGET_FILE:...>` 在 Linux/macOS 上解析为**带完整版本号**的实体文件
  #   （`libjsdk_can.so.0.1.0` / `libjsdk_can.0.1.0.dylib`），而包内加载器只认
  #   无版本号的名字（见 bindings/python/src/jsdk_can/_abi.py 的 _LIB_CANDIDATES）。
  #   结果：**Linux 上装出来的包找不到自己带的库**，而 Windows 因为
  #   `libjsdk_can.dll` 本来就不带版本号，一直没暴露。
  if(WIN32)
    set(_jsdk_py_libname "$<TARGET_FILE_NAME:jsdk_can_shared>")
    # 别的平台的库（lib/ 里全是构建产物；混一份用不上的进来会让打出的 wheel 变脏，
    # 也会让"包内库叫什么名字"同时存在两个真相）
    set(_jsdk_py_stale "libjsdk_can.so" "jsdk_can.so" "libjsdk_can.dylib")
  elseif(APPLE)
    set(_jsdk_py_libname "libjsdk_can.dylib")
    set(_jsdk_py_stale "libjsdk_can.dll" "jsdk_can.dll" "libjsdk_can.so")
  else()
    set(_jsdk_py_libname "libjsdk_can.so")
    set(_jsdk_py_stale "libjsdk_can.dll" "jsdk_can.dll" "libjsdk_can.dylib")
  endif()
  set(_jsdk_py_libdir "${JSDK_PYTHON_DIR}/src/jsdk_can/lib")
  set(_jsdk_py_stale_full "")
  foreach(_n IN LISTS _jsdk_py_stale)
    list(APPEND _jsdk_py_stale_full "${_jsdk_py_libdir}/${_n}")
  endforeach()

  add_custom_command(TARGET jsdk_can_shared POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${_jsdk_py_libdir}"
    COMMAND ${CMAKE_COMMAND} -E rm -f ${_jsdk_py_stale_full}
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_FILE:jsdk_can_shared>"
            "${_jsdk_py_libdir}/${_jsdk_py_libname}"
    COMMENT "Copying shared library into bindings/python/src/jsdk_can/lib/${_jsdk_py_libname}")
endif()

# 把 Python 测试挂进 ctest：只有"共享库 + pytest 都在"时才注册，否则安静跳过
# （CI 多数镜像没装 pytest，不能因此让 configure 失败）。
if(JSDK_BUILD_TESTS AND JSDK_BUILD_SHARED)
  find_package(Python3 COMPONENTS Interpreter QUIET)
  if(Python3_Interpreter_FOUND)
    execute_process(
      COMMAND ${Python3_EXECUTABLE} -c "import pytest"
      RESULT_VARIABLE _jsdk_pytest_rc OUTPUT_QUIET ERROR_QUIET)
    if(_jsdk_pytest_rc EQUAL 0)
      add_test(NAME python_bindings
        COMMAND ${Python3_EXECUTABLE} -m pytest -q
                "${JSDK_PYTHON_DIR}/tests")
      set_tests_properties(python_bindings PROPERTIES
        WORKING_DIRECTORY "${JSDK_PYTHON_DIR}"
        ENVIRONMENT
          "JSDK_LIB_PATH=$<TARGET_FILE_DIR:jsdk_can_shared>;PYTHONPATH=${JSDK_PYTHON_DIR}/src;PYTHONIOENCODING=utf-8")
      message(STATUS "Python bindings: 'python_bindings' ctest registered")
    else()
      message(STATUS
        "Python bindings: pytest not found → 'python_bindings' ctest skipped "
        "(pip install pytest 后重新 configure)")
    endif()
  endif()
endif()

# ---------------------------------------------------------------------------
# 测试
# ---------------------------------------------------------------------------
if(JSDK_BUILD_TESTS)
  enable_testing()

  add_executable(test_jsondesc tests/test_jsondesc.c)
  target_link_libraries(test_jsondesc PRIVATE jsdk_can)
  # 测试直接使用内部接口（src/jsdk_internal.h），故需 src 在包含路径中
  target_include_directories(test_jsondesc PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_compile_options(test_jsondesc PRIVATE ${JSDK_WARN_FLAGS})
  target_compile_definitions(test_jsondesc PRIVATE
    JSDK_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/data")

  add_test(NAME jsondesc COMMAND test_jsondesc)

  add_executable(test_codec tests/test_codec.c)
  target_link_libraries(test_codec PRIVATE jsdk_can)
  # 测试直接用协议层内部头（cb_frame.h / cb_mit.h）与黄金向量
  target_include_directories(test_codec PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/proto_cyberbeast
    ${CMAKE_CURRENT_SOURCE_DIR}/tests)
  target_compile_options(test_codec PRIVATE ${JSDK_WARN_FLAGS})

  add_test(NAME codec COMMAND test_codec)

  add_executable(test_proto tests/test_proto.c)
  target_link_libraries(test_proto PRIVATE jsdk_can)
  target_include_directories(test_proto PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/proto_cyberbeast
    ${CMAKE_CURRENT_SOURCE_DIR}/tests)
  target_compile_options(test_proto PRIVATE ${JSDK_WARN_FLAGS})

  add_test(NAME proto COMMAND test_proto)

  add_executable(test_desc_fetch tests/test_desc_fetch.c)
  target_link_libraries(test_desc_fetch PRIVATE jsdk_can)
  target_include_directories(test_desc_fetch PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/proto_cyberbeast
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hal
    ${CMAKE_CURRENT_SOURCE_DIR}/tests)
  target_compile_options(test_desc_fetch PRIVATE ${JSDK_WARN_FLAGS})
  target_compile_definitions(test_desc_fetch PRIVATE
    JSDK_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/data")

  if(JSDK_BUILD_HAL_VIRTUAL)
    add_executable(test_hal_virtual tests/test_hal_virtual.c)
    target_link_libraries(test_hal_virtual PRIVATE jsdk_can)
    target_include_directories(test_hal_virtual PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src/proto_cyberbeast
      ${CMAKE_CURRENT_SOURCE_DIR}/src/hal
      ${CMAKE_CURRENT_SOURCE_DIR}/tests)
    target_compile_options(test_hal_virtual PRIVATE ${JSDK_WARN_FLAGS})
    target_compile_definitions(test_hal_virtual PRIVATE
      JSDK_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/data")

    add_test(NAME hal_virtual COMMAND test_hal_virtual)
    target_link_libraries(test_desc_fetch PRIVATE jsdk_can)
    add_test(NAME desc_fetch COMMAND test_desc_fetch)

    add_executable(test_joint tests/test_joint.c)
    target_link_libraries(test_joint PRIVATE jsdk_can)
    target_include_directories(test_joint PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/src/proto_cyberbeast
      ${CMAKE_CURRENT_SOURCE_DIR}/src/hal
      ${CMAKE_CURRENT_SOURCE_DIR}/tests)
    target_compile_options(test_joint PRIVATE ${JSDK_WARN_FLAGS})
    target_compile_definitions(test_joint PRIVATE
      JSDK_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/data")

    add_test(NAME joint COMMAND test_joint)

    add_executable(test_ops tests/test_ops.c)
    target_link_libraries(test_ops PRIVATE jsdk_can)
    target_include_directories(test_ops PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/src/proto_cyberbeast
      ${CMAKE_CURRENT_SOURCE_DIR}/src/hal
      ${CMAKE_CURRENT_SOURCE_DIR}/tests)
    target_compile_options(test_ops PRIVATE ${JSDK_WARN_FLAGS})
    target_compile_definitions(test_ops PRIVATE
      JSDK_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/data")

    add_test(NAME ops COMMAND test_ops)

    add_executable(test_group tests/test_group.c)
    target_link_libraries(test_group PRIVATE jsdk_can)
    target_include_directories(test_group PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/src/proto_cyberbeast
      ${CMAKE_CURRENT_SOURCE_DIR}/src/hal
      ${CMAKE_CURRENT_SOURCE_DIR}/tests)
    target_compile_options(test_group PRIVATE ${JSDK_WARN_FLAGS})
    target_compile_definitions(test_group PRIVATE
      JSDK_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/data")

    add_test(NAME group COMMAND test_group)
  endif()

  # 内置 HAL 后端的可离线测试部分（slcan 编解码 + 句柄生命周期 + 错误码契约）。
  # 真机收发无法在 CI 断言，见 docs/PORTING.zh-CN.md 的手工冒烟清单。
  if(JSDK_HAL_COMMON_NEEDED)
    add_executable(test_hal tests/test_hal.c)
    target_link_libraries(test_hal PRIVATE jsdk_can)
    target_include_directories(test_hal PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/src/hal
      ${CMAKE_CURRENT_SOURCE_DIR}/tests)
    target_compile_options(test_hal PRIVATE ${JSDK_WARN_FLAGS})

    add_test(NAME hal COMMAND test_hal)
  endif()

  # SocketCAN **真链路**冒烟（走内核，对端是裸 socket）—— 仅 Linux + socketcan 后端。
  # 需要一个可用的 CAN 接口：vcan（`modprobe vcan`）或真硬件。
  # 只有显式给了 -DJSDK_LIVE_CAN_IFACE=vcan0 才注册成 ctest（否则会因缺接口而红）。
  if(UNIX AND JSDK_BUILD_HAL_SOCKETCAN)
    add_executable(test_socketcan_live tests/test_socketcan_live.c)
    target_link_libraries(test_socketcan_live PRIVATE jsdk_can)
    target_include_directories(test_socketcan_live PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_compile_options(test_socketcan_live PRIVATE ${JSDK_WARN_FLAGS})

    set(JSDK_LIVE_CAN_IFACE "" CACHE STRING
        "可选：把 SocketCAN 真链路冒烟注册成 ctest 并绑定到这个接口（如 vcan0）")
    if(JSDK_LIVE_CAN_IFACE)
      add_test(NAME socketcan_live COMMAND test_socketcan_live)
      set_tests_properties(socketcan_live PROPERTIES
        ENVIRONMENT "JSDK_SC_IFACE=${JSDK_LIVE_CAN_IFACE};JSDK_SC_IFACE_CLASSIC=${JSDK_LIVE_CAN_CLASSIC_IFACE}")
      message(STATUS "SocketCAN live smoke: bound to '${JSDK_LIVE_CAN_IFACE}'")
    endif()
  endif()
endif()

# ---------------------------------------------------------------------------
# WP7：PC 诊断 CLI（jsdk-cli）
#
# 逻辑全部放在静态库 `jsdk_cli_core` 里，可执行文件只是个壳（装 SIGINT 并转调）。
# 这样测试能**同进程**调用 `jsdk_cli_run(argc, argv, out, err)` 断言输出，
# 不必起子进程抓 stdout —— 在 Windows 上那还要处理路径与引号。
#
# ⚠ CLI 需要堆模式（上下文与 arena 都是 calloc 出来的），因此 JSDK_ENABLE_HEAP
#   必须为 ON；否则报错而不是给出一个"能编但一跑就崩"的目标。
# ---------------------------------------------------------------------------
if(JSDK_BUILD_CLI)
  if(NOT JSDK_ENABLE_HEAP)
    message(FATAL_ERROR
      "JSDK_BUILD_CLI=ON 需要同时打开堆模式：-DJSDK_ENABLE_HEAP=ON\n"
      "（CLI 用堆分配上下文与描述符 arena；MCU 构建不需要 CLI）")
  endif()
  if(NOT JSDK_BUILD_HAL_VIRTUAL AND NOT JSDK_BUILD_HAL_SOCKETCAN
     AND NOT JSDK_BUILD_HAL_PCAN AND NOT JSDK_BUILD_HAL_SLCAN)
    message(FATAL_ERROR "JSDK_BUILD_CLI=ON 需要至少一个 HAL 后端")
  endif()

  add_library(jsdk_cli_core STATIC
    tools/jsdk_cli/cli_text.c
    tools/jsdk_cli/cli_json.c
    tools/jsdk_cli/cli_stop.c
    tools/jsdk_cli/cli_app.c
    tools/jsdk_cli/cli_cmd.c)
  target_link_libraries(jsdk_cli_core PUBLIC jsdk_can)
  target_include_directories(jsdk_cli_core
    PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}/tools/jsdk_cli
            ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_options(jsdk_cli_core PRIVATE ${JSDK_WARN_FLAGS})

  add_executable(jsdk-cli tools/jsdk_cli/cli_main.c)
  target_link_libraries(jsdk-cli PRIVATE jsdk_cli_core)
  target_compile_options(jsdk-cli PRIVATE ${JSDK_WARN_FLAGS})

  if(JSDK_BUILD_TESTS AND JSDK_BUILD_HAL_VIRTUAL)
    add_executable(test_cli tests/test_cli.c)
    target_link_libraries(test_cli PRIVATE jsdk_cli_core)
    target_compile_options(test_cli PRIVATE ${JSDK_WARN_FLAGS})

    add_test(NAME cli COMMAND test_cli)

    # 终端文本输出（代码页转换）：纯逻辑 + 注入式转换器，不需要真控制台。
    # 背景：Windows 控制台默认 CP936，而工程里的中文是 UTF-8 字面量。
    add_executable(test_cli_text tests/test_cli_text.c)
    target_link_libraries(test_cli_text PRIVATE jsdk_cli_core)
    target_compile_options(test_cli_text PRIVATE ${JSDK_WARN_FLAGS})

    add_test(NAME cli_text COMMAND test_cli_text)
  endif()

  # 静态检查（防"乱码"复发）：CLI 的输出必须走 cli_text.h 的包装，
  # 而且输出字符串必须在 CP936 里可表示（⚠/✓ 这种会变成 '?'）。
  # 只在有 Python 时注册 —— 与 python_bindings 同一原则：缺工具不能弄挂 configure。
  if(JSDK_BUILD_TESTS)
    find_package(Python3 COMPONENTS Interpreter QUIET)
    if(Python3_Interpreter_FOUND)
      add_test(NAME cli_text_lint
        COMMAND ${Python3_EXECUTABLE}
                "${CMAKE_CURRENT_SOURCE_DIR}/tools/check_cli_text.py"
                "${CMAKE_CURRENT_SOURCE_DIR}")
      message(STATUS "CLI text: 'cli_text_lint' ctest registered")
    else()
      message(STATUS "CLI text: Python3 未找到 → 'cli_text_lint' 跳过")
    endif()
  endif()

  # 真机自检脚本的**无硬件**回归：`--if virtual` 跑一层，证明脚本本身没坏
  # （分层判据、JSON 解析、退出码）。真机那部分只能人工 —— 见 CLI.zh-CN.md §8。
  # 缺 bash（Windows 上是 Git-Bash）就跳过，不弄挂 configure。
  if(JSDK_BUILD_TESTS)
    find_program(JSDK_BASH_EXECUTABLE bash)
    if(JSDK_BASH_EXECUTABLE AND TARGET jsdk-cli)
      add_test(NAME hw_verify_virtual
        COMMAND ${CMAKE_COMMAND} -E env "JSDK_CLI=$<TARGET_FILE:jsdk-cli>"
                "${JSDK_BASH_EXECUTABLE}"
                "${CMAKE_CURRENT_SOURCE_DIR}/tools/hw_verify.sh"
                --if virtual --runs 1)
      set_tests_properties(hw_verify_virtual PROPERTIES
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}")
      message(STATUS "hw_verify: 'hw_verify_virtual' ctest registered")
    else()
      message(STATUS "hw_verify: 缺 bash 或缺 jsdk-cli → 'hw_verify_virtual' 跳过")
    endif()
  endif()
endif()

# ---------------------------------------------------------------------------
# 示例（A1）：7 个可编译可运行的独立程序
#
# 默认跟随 JSDK_BUILD_TESTS：示例同时是**回归的一部分**（在虚拟后端上跑，不需要硬件）。
# 只跑一遍看退出码是故意设计得这么粗 —— 它们演示的是"客户能不能照抄成功"，
# 断言细节交给 tests/（那里有 2.6 万条断言）。
# ---------------------------------------------------------------------------
option(JSDK_BUILD_EXAMPLES "Build the examples/ programs" ${JSDK_BUILD_TESTS})

if(JSDK_BUILD_EXAMPLES)
  # 01~06 用内置虚拟后端；每个示例都要用到那个 HAL。
  if(NOT JSDK_BUILD_HAL_VIRTUAL)
    message(WARNING
      "JSDK_BUILD_EXAMPLES=ON 但 JSDK_BUILD_HAL_VIRTUAL=OFF："
      "01~06 号示例需要虚拟后端，本次只构建 07_custom_hal（它只用公共 ABI）。")
  endif()

  set(_jsdk_examples "")
  if(JSDK_BUILD_HAL_VIRTUAL)
    set(_jsdk_examples
        01_hello_virtual 02_enable_mit 03_control_loop 04_param_rw
        05_desc_cache 06_group_broadcast)
  endif()
  # 07 不用任何内置后端（自己实现 jsdk_can_hal_t）→ 永远可以构建
  list(APPEND _jsdk_examples 07_custom_hal)

  foreach(_ex IN LISTS _jsdk_examples)
    set(_tgt "ex_${_ex}")
    add_executable(${_tgt} examples/${_ex}.c)
    target_link_libraries(${_tgt} PRIVATE jsdk_can)
    target_compile_options(${_tgt} PRIVATE ${JSDK_WARN_FLAGS})
    set_target_properties(${_tgt} PROPERTIES
      OUTPUT_NAME ${_ex}
      # 落到 build/examples/ 下（与 examples/README.md 里写的路径一致）
      RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/examples)
    if(JSDK_BUILD_TESTS)
      add_test(NAME example_${_ex} COMMAND ${_tgt})
    endif()
  endforeach()

  # --- C++ 包装示例（A4）：需要 C++ 编译器 --------------------------------
  # 包装头 `joint_group.hpp` 只依赖公共 C ABI；这个示例额外用到虚拟后端。
  include(CheckLanguage)
  check_language(CXX)
  if(CMAKE_CXX_COMPILER)
    enable_language(CXX)
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    add_executable(ex_08_cpp_wrapper examples/08_cpp_wrapper.cpp)
    target_link_libraries(ex_08_cpp_wrapper PRIVATE jsdk_can)
    target_compile_options(ex_08_cpp_wrapper PRIVATE ${JSDK_WARN_FLAGS_CXX})
    set_target_properties(ex_08_cpp_wrapper PROPERTIES
      OUTPUT_NAME 08_cpp_wrapper
      RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/examples)
    if(JSDK_BUILD_TESTS)
      add_test(NAME example_08_cpp_wrapper COMMAND ex_08_cpp_wrapper)
    endif()
  else()
    message(WARNING "找不到 C++ 编译器：跳过 examples/08_cpp_wrapper（包装头本身不受影响）")
  endif()
endif()

# ---------------------------------------------------------------------------
# 安装与导出（A2 pkg-config + A6 find_package）
#
# 目标：客户两种方式都能用，且**不需要猜 include 路径**：
#   cmake:      find_package(jsdk_can REQUIRED) + target_link_libraries(app jsdk::can)
#   pkg-config: pkg-config --cflags --libs joint-sdk-can
#
# ⚠ 为什么不写 `ARCH_INDEPENDENT`（版本文件）：本包装的是**编译好的二进制**，
#   架构必须参与校验 —— 与 Python wheel 那边（B3：`py3-none-any` → `py3-none-win_amd64`）
#   是同一类错误：把平台相关的产物说成"哪个平台都能装"。
# ---------------------------------------------------------------------------
include(CMakePackageConfigHelpers)

install(TARGETS jsdk_can EXPORT jsdk_canTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/joint_sdk DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# 共享库（可选）与 CLI（可选）也一并装上：现场排障就是把 jsdk-cli 拷过去
if(TARGET jsdk_can_shared)
  install(TARGETS jsdk_can_shared EXPORT jsdk_canTargets
          LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
          RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
if(TARGET jsdk-cli)
  install(TARGETS jsdk-cli RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

# --- CMake 包配置：find_package(jsdk_can) ---
install(EXPORT jsdk_canTargets
        FILE      jsdk_canTargets.cmake
        NAMESPACE jsdk::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/jsdk_can)

configure_package_config_file(
    cmake/jsdk_canConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/jsdk_canConfig.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/jsdk_can)

# ⚠ `SameMinorVersion` 而不是 `SameMajorVersion`：本库还在 0.x，语义上
#   **minor 才是兼容轴**（0.1 → 0.2 可能有破坏性变更）。等 1.0 之后再改宽。
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/jsdk_canConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMinorVersion)

install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/jsdk_canConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/jsdk_canConfigVersion.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/jsdk_can)

# --- pkg-config：joint-sdk-can ---
# `Libs.private` 是静态库使用者要补的。实测清单：
#   -lm   : glibc 把 floor() 放在**独立的** libm 里（MinGW 不需要，所以 Windows 上看不到）；
#   -ldl  : hal_pcan.c 运行期 dlopen 加载 PCAN-Basic（macOS 无此库，CMAKE_DL_LIBS 为空）。
#   **没有 pthread**：本库不创建线程（对照：设计文档 §7 早期写的 -lpthread 是不对的，已改）。
set(JSDK_PC_LIBS_PRIVATE "")
if(UNIX)
  string(APPEND JSDK_PC_LIBS_PRIVATE " -lm")
  if(CMAKE_DL_LIBS)
    string(APPEND JSDK_PC_LIBS_PRIVATE " -l${CMAKE_DL_LIBS}")
  endif()
endif()
string(STRIP "${JSDK_PC_LIBS_PRIVATE}" JSDK_PC_LIBS_PRIVATE)

configure_file(cmake/joint-sdk-can.pc.in joint-sdk-can.pc @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/joint-sdk-can.pc"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)
