/**
 * @file    cli_json.c
 * @brief   极简 JSON 输出器实现
 */

#include "cli_json.h"

#include <string.h>
#include "cli_text.h"

/* --------------------------------------------------------------------------
 * 内部：逗号与缩进
 * ------------------------------------------------------------------------ */

/** 在写下一个成员/元素**之前**调用：决定是否先补逗号。 */
static void j_pre(cli_json_t *j)
{
    if (j->depth <= 0) return;
    if (j->need_comma[j->depth - 1]) {
        cli_fputc(',', j->f);
    }
    j->need_comma[j->depth - 1] = 1;    /* 这一次之后就是必需的了 */
    if (j->pretty) cli_fputc('\n', j->f);
    if (j->pretty) {
        int k;
        for (k = 0; k < j->depth; ++k) cli_fputs("  ", j->f);
    }
}

/** 只在"数组元素/无键值"的场合补逗号与缩进（key == NULL 的分支共用）。 */
static void j_key(cli_json_t *j, const char *key)
{
    j_pre(j);
    if (key) {
        cli_json_escape(j->f, key);
        cli_fputc(':', j->f);
        if (j->pretty) cli_fputc(' ', j->f);
    }
}

static void j_push(cli_json_t *j)
{
    if (j->depth < CLI_JSON_MAX_DEPTH) {
        j->need_comma[j->depth] = 0;
    }
    j->depth++;
}

/* --------------------------------------------------------------------------
 * 公开
 * ------------------------------------------------------------------------ */

void cli_json_init(cli_json_t *j, FILE *f, int pretty)
{
    if (!j) return;
    memset(j, 0, sizeof *j);
    j->f = f;
    j->pretty = pretty ? 1 : 0;
    j_push(j);                    /* 根对象 */
    cli_fputc('{', f);
}

void cli_json_finish(cli_json_t *j)
{
    if (!j || !j->f) return;
    while (j->depth > 0) {
        j->depth--;
        if (j->pretty && j->depth > 0) {
            int k;
            cli_fputc('\n', j->f);
            for (k = 0; k < j->depth; ++k) cli_fputs("  ", j->f);
        }
        cli_fputc('}', j->f);
    }
    cli_fputc('\n', j->f);
}

void cli_json_obj_begin(cli_json_t *j, const char *key)
{
    j_key(j, key);
    cli_fputc('{', j->f);
    j_push(j);
}

void cli_json_obj_end(cli_json_t *j)
{
    if (j->depth > 0) j->depth--;
    if (j->pretty && j->depth > 0 &&
        (j->depth < CLI_JSON_MAX_DEPTH && j->need_comma[j->depth])) {
        /* 只有当对象非空时才换行 */
        int k;
        cli_fputc('\n', j->f);
        for (k = 0; k < j->depth; ++k) cli_fputs("  ", j->f);
    }
    cli_fputc('}', j->f);
    if (j->depth < CLI_JSON_MAX_DEPTH) j->need_comma[j->depth] = 1;
}

void cli_json_arr_begin(cli_json_t *j, const char *key)
{
    j_key(j, key);
    cli_fputc('[', j->f);
    j_push(j);
}

void cli_json_arr_end(cli_json_t *j)
{
    if (j->depth > 0) j->depth--;
    if (j->pretty) cli_fputc('\n', j->f);
    cli_fputc(']', j->f);
    if (j->depth < CLI_JSON_MAX_DEPTH) j->need_comma[j->depth] = 1;
}

void cli_json_num(cli_json_t *j, const char *key, double v)
{
    j_key(j, key);
    cli_fprintf(j->f, "%.6g", v);
}

void cli_json_i64(cli_json_t *j, const char *key, long long v)
{
    j_key(j, key);
    cli_fprintf(j->f, "%lld", v);
}

void cli_json_bool(cli_json_t *j, const char *key, int v)
{
    j_key(j, key);
    cli_fputs(v ? "true" : "false", j->f);
}

void cli_json_null(cli_json_t *j, const char *key)
{
    j_key(j, key);
    cli_fputs("null", j->f);
}

void cli_json_raw_num(cli_json_t *j, double v)
{
    j_key(j, NULL);
    cli_fprintf(j->f, "%.6g", v);
}

void cli_json_escape(FILE *f, const char *s)
{
    cli_fputc('"', f);
    if (s) {
        const unsigned char *p = (const unsigned char *)s;
        for (; *p; ++p) {
            unsigned char c = *p;
            switch (c) {
            case '"':  cli_fputs("\\\"", f); break;
            case '\\': cli_fputs("\\\\", f); break;
            case '\n': cli_fputs("\\n", f);  break;
            case '\r': cli_fputs("\\r", f);  break;
            case '\t': cli_fputs("\\t", f);  break;
            default:
                if (c < 0x20u) {
                    cli_fprintf(f, "\\u%04x", (unsigned)c);
                } else {
                    cli_fputc((int)c, f);
                }
                break;
            }
        }
    }
    cli_fputc('"', f);
}

void cli_json_str(cli_json_t *j, const char *key, const char *v)
{
    j_key(j, key);
    cli_json_escape(j->f, v);
}
