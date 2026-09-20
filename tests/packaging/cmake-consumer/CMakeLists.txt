# 打包冒烟的第二个消费者：只用 `find_package`（不碰 pkg-config）。
#
# 它故意**不写**任何 include_directories / link_directories ——
# 客户应该只需要 `target_link_libraries(app PRIVATE jsdk::can)` 就拿到
# 头文件路径（两个 -I）、库、以及传递依赖（UNIX 下的 -lm/-ldl）。
# 若哪天有人把 `$<INSTALL_INTERFACE:...>` 或 EXPORT_NAME 改坏，这里会立刻失败。
cmake_minimum_required(VERSION 3.16)
project(jsdk_consumer C)

# 允许调用方指定一个版本要求：给了就走“版本必须兼容”的路径。
# `tests/packaging` 的负向用例靠它验证 `SameMinorVersion` 真的生效
# （只声明变量而不写进 find_package 是无效的 —— 版本文件根本不会被读）。
if(DEFINED JSDK_CONSUMER_REQUIRE_VERSION)
  find_package(jsdk_can ${JSDK_CONSUMER_REQUIRE_VERSION} REQUIRED)
else()
  find_package(jsdk_can REQUIRED)
endif()
add_executable(consumer ../consumer.c)
target_link_libraries(consumer PRIVATE jsdk::can)

# 把安装前缀打出来，方便在日志里确认"找的确实是刚装的那一份"
get_target_property(_inc jsdk::can INTERFACE_INCLUDE_DIRECTORIES)
message(STATUS "jsdk_can found; interface includes: ${_inc}")
