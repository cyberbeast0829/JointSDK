/**
 * @file    cli_json.h
 * @brief   极简 JSON 输出器（`--json` 用）
 *
 * @par 为什么自己写而不是引一个库
 *  CLI 的目标之一是"零运行时依赖、现场直接跑"。为 200 行输出格式引一个 JSON
 *  库不划算，而且 **MCU 客户会读这些字段名**，格式必须由我们完全控制。
 *
 * @par 用途边界
 *  只负责**打印**，不解析。所有数值都以十进制打印；浮点用 `%.6g`，
 *  既能读（`0.012`）也不会把 `12.5` 打成 `12.499999`。
 *  转义只处理 JSON 必需的那几个字符（`"` `\` 和控制字符）。
 */

#ifndef JSDK_CLI_JSON_H
#define JSDK_CLI_JSON_H

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CLI_JSON_MAX_DEPTH 6

typedef struct {
    FILE *f;
    int   depth;                       /**< 当前嵌套层数 */
    int   need_comma[CLI_JSON_MAX_DEPTH];
    int   pretty;                      /**< 1 = 换行缩进（人看）；0 = 紧凑（机器看） */
} cli_json_t;

/** 初始化。@param pretty 非 0 时输出带缩进，便于人读。 */
void cli_json_init(cli_json_t *j, FILE *f, int pretty);

/** 结束（补齐所有未闭合的花括号并换行）。 */
void cli_json_finish(cli_json_t *j);

void cli_json_obj_begin(cli_json_t *j, const char *key);   /**< key = NULL → 匿名对象 */
void cli_json_obj_end(cli_json_t *j);
void cli_json_arr_begin(cli_json_t *j, const char *key);
void cli_json_arr_end(cli_json_t *j);

void cli_json_num  (cli_json_t *j, const char *key, double v);
void cli_json_i64  (cli_json_t *j, const char *key, long long v);
void cli_json_str  (cli_json_t *j, const char *key, const char *v);
void cli_json_bool (cli_json_t *j, const char *key, int v);
void cli_json_null (cli_json_t *j, const char *key);
/** 裸值（数组元素）：调用方自己控制逗号（库会处理）。 */
void cli_json_raw_num(cli_json_t *j, double v);

/** 键值分隔与转义（供需要自己拼的地方用）。 */
void cli_json_escape(FILE *f, const char *s);

#ifdef __cplusplus
}
#endif

#endif /* JSDK_CLI_JSON_H */
