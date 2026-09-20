/*
 * 这个 shim 只有一个用途：**证明 amalgamation 是公开头的即插即用替代**。
 *
 * 客户程序照常写 `#include "joint_sdk.h"` —— 只要把本目录放在 include 路径
 * **前面**（并让 dist/ 可被找到），编译到的就是单文件版。这样"合并件与原件
 * 行为一致"这件事可以被自动化验证，而不是靠读生成的代码。
 *
 * 见 tools/amalgam_smoke.sh。
 */
#include "jsdk_can_amalgam.h"
