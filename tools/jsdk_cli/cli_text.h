/**
 * @file    cli_text.h
 * @brief   终端文本输出：把工程里的 **UTF-8** 变成"目标流看得懂"的字节
 *
 * @par 为什么需要它
 * 全工程的中文都是 **UTF-8 字面量**（源码、文档、错误消息），而 Windows 控制台
 * 的默认代码页是 **CP936(GBK)**（本机实测 `chcp` → 936）。C 运行时不看代码页，
 * 只会把字节原样丢给控制台，控制台按**自己的**代码页解释 → 用户看到
 *
 *     $ jsdk-cli --if slcan scan
 *     鍙戠幇 0 涓�鑺傜偣锛堣��鍔� 200 ms + ...     ← 实际想显示的是"发现 0 个节点"
 *
 * 这不是 `jsdk-cli` 的问题，而是**任何**往下写 UTF-8 的程序在这台机器上都会有的
 * 问题（客户程序 `printf` 一行 `jsdk_context_last_error()` 也一样）。所以修在
 * 边界上，而不是修某一条消息。
 *
 * @par 规则（只需记两条）
 *  - `f` **是控制台** → UTF-8 先转成 `GetConsoleOutputCP()` 再写；
 *  - `f` 是管道/文件 → **原样写 UTF-8**（重定向给脚本、`| jq`、编辑器都期望 UTF-8）。
 *
 * 这与 git for Windows 的 `mingw_ansi_fputs()` 思路一致：**输出端的字节要符合
 * 接收端的约定**。反过来做（启动时把控制台设成 CP65001）改的是**共享状态**：
 * 程序被 kill 就回不去，同控制台的其它进程跟着遭殃，还得碰运气控制台字体有没有
 * CJK 字形 —— 所以不采用。
 *
 * @par 用法
 * 把 stdio 的同名调用换成 `cli_` 前缀版本即可，**参数顺序完全一致**，例如
 * `printf(fmt, ...)` → `cli_printf(fmt, ...)`、`fprintf(f, fmt, ...)` →
 * `cli_fprintf(f, fmt, ...)`、`fputc(c, f)` → `cli_fputc(c, f)`。
 * 变参函数都带 `__attribute__((format(printf, ...)))`，所以 `-Wformat` 的
 * `%d/%u` 检查一点没少。
 *
 * @warning **不要**用它们写二进制数据（`desc-export` 的 `fwrite` 保持原样）：
 *          它们只保证"文本看得懂"，不保证字节不变（控制台上会被转码）。
 *
 * @par 环境变量 `JSDK_CLI_TEXT`（排查与特殊管道用）
 *  - **不设**（默认）→ 自动判别：控制台转码、管道/文件原样 UTF-8；
 *  - `utf8`（或 `off`）→ 一律原样写 UTF-8。给"本来就吃 UTF-8"的终端
 *    （MinTTY / Windows Terminal）或管道另一端期待 UTF-8 的场合；
 *  - 代码页数字（如 `936`）→ 一律按"控制台 + 该代码页"处理。CI 里用它
 *    在没有真控制台的机器上验证转码路径（不必造 ConPTY）。
 */

#ifndef JSDK_CLI_TEXT_H
#define JSDK_CLI_TEXT_H

#include <stdio.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 让 GCC/Clang 继续检查格式串（丢了它等于丢掉 `-Wformat` 的保护）。 */
#if defined(__GNUC__) || defined(__clang__)
#  define CLI_PRINTF_ATTR(fmt_idx, first_va) \
        __attribute__((format(printf, fmt_idx, first_va)))
#else
#  define CLI_PRINTF_ATTR(fmt_idx, first_va)
#endif

/* --------------------------------------------------------------------------
 * 文本输出（stdio 的 `cli_` 版本，参数顺序一致）
 * ------------------------------------------------------------------------ */

/** `printf`：写 stdout。 */
int cli_printf(const char *fmt, ...) CLI_PRINTF_ATTR(1, 2);

/** `fprintf`：写指定流。 */
int cli_fprintf(FILE *f, const char *fmt, ...) CLI_PRINTF_ATTR(2, 3);

/** `puts`：写 stdout 并补一个 `\n`。 */
int cli_puts(const char *s);

/** `fputs`：写指定流。 */
int cli_fputs(const char *s, FILE *f);

/** `fputc`：写一个字节（逐字节写 UTF-8 也是安全的，见下）。 */
int cli_fputc(int c, FILE *f);

/**
 * 刷缓冲。`f==NULL` → 同时刷 stdout 与 stderr。
 * @note 本模块自己**不做行缓冲**（写下去就落盘），保留这个函数是为了
 *       让调用方语义完整、以后加缓冲不必改调用点。
 */
void cli_text_flush(FILE *f);

/* --------------------------------------------------------------------------
 * 纯逻辑（不碰 Windows API，可离线单测）
 * ------------------------------------------------------------------------ */

/**
 * 从 UTF-8 字节串里切出"到最后一个完整码点为止"的前缀长度。
 *
 * 存在的理由：转换必须**按完整码点**做，绝不能把一个 3 字节汉字从中切开。
 * 调用方拿到 @c 返回值 后，把剩下的 0~3 字节留到下一次（本模块内部就这么做，
 * 所以逐字节写也是安全的）。
 *
 * 非法字节不会导致死循环：非法前导/续字节一律按"1 个字节"前进。
 *
 * @return `0..n`。等于 @p n 表示整串都是完整码点。
 */
size_t cli_utf8_complete_prefix(const char *s, size_t n);

/* --------------------------------------------------------------------------
 * 代码页转换（可单测；非 Windows 上是直通）
 * ------------------------------------------------------------------------ */

/**
 * UTF-8 → 代码页 @p cp。
 *
 * @param cp 目标代码页。**0 或 65001 表示不转换**（UTF-8 直通）。
 * @param n  UTF-8 输入长度，必须 ≤ `CLI_TEXT_CHUNK`（调用方先切块）。
 * @param out 输出缓冲；@param cap 其容量。
 * @return 写入 @p out 的字节数；`0` = 转换不可行（非法 UTF-8 / 缓冲不足 /
 *         非 Windows），**调用方应当降级为原样写**而不是丢弃内容。
 *
 * 目标代码页里**表示不了**的字符会变成 `?`（`WC_NO_BEST_FIT_CHARS`，
 * 不用"最佳拟合"——那会把 `⚠` 悄悄变成别的字，更难查）。
 */
size_t cli_text_conv(unsigned cp, const char *utf8, size_t n, char *out, size_t cap);

/** 单块 UTF-8 输入的最大长度（转换缓冲按它定尺寸）。 */
#define CLI_TEXT_CHUNK 512u

/* --------------------------------------------------------------------------
 * 测试注入点（生产代码不调用）
 * ------------------------------------------------------------------------ */

/** 转换器签名（测试可替换）。 */
typedef size_t (*cli_text_conv_fn)(unsigned cp, const char *utf8, size_t n,
                                   char *out, size_t cap);

/** 替换转换器；`NULL` = 恢复默认实现。 */
void cli_text_set_conv(cli_text_conv_fn fn);

/**
 * 把所有流都当成"控制台 + 代码页 @p cp"（测试用，便于在没有真控制台的
 * CI 里验证转换路径）。@p on = 0 恢复正常判别。
 */
void cli_text_set_console_force(int on, unsigned cp);

#ifdef __cplusplus
}
#endif

#endif /* JSDK_CLI_TEXT_H */
