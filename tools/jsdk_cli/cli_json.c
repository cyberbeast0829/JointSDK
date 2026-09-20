/**
 * @file    cli_json.c
 * @brief   极简 JSON 输出器实现
 */

#include "cli_json.h"

#include <string.h>

/* --------------------------------------------------------------------------
 * 内部：逗号与缩进
 * ------------------------------------------------------------------------ */

/** 在写下一个成员/元素**之前**调用：决定是否先补逗号。 */
static void j_pre(cli_json_t *j)
{
    if (j->depth <= 0) return;
    if (j->need_comma[j->depth - 1]) {
        fputc(',', j->f);
    }
    j->need_comma[j->depth - 1] = 1;    /* 这一次之后就是必需的了 */
    if (j->pretty) fputc('\n', j->f);
    if (j->pretty) {
        int k;
        for (k = 0; k < j->depth; ++k) fputs("  ", j->f);
    }
}

/** 只在"数组元素/无键值"的场合补逗号与缩进（key == NULL 的分支共用）。 */
static void j_key(cli_json_t *j, const char *key)
{
    j_pre(j);
    if (key) {
        cli_json_escape(j->f, key);
        fputc(':', j->f);
        if (j->pretty) fputc(' ', j->f);
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
    fputc('{', f);
}

void cli_json_finish(cli_json_t *j)
{
    if (!j || !j->f) return;
    while (j->depth > 0) {
        j->depth--;
        if (j->pretty && j->depth > 0) {
            int k;
            fputc('\n', j->f);
            for (k = 0; k < j->depth; ++k) fputs("  ", j->f);
        }
        fputc('}', j->f);
    }
    fputc('\n', j->f);
}

void cli_json_obj_begin(cli_json_t *j, const char *key)
{
    j_key(j, key);
    fputc('{', j->f);
    j_push(j);
}

void cli_json_obj_end(cli_json_t *j)
{
    if (j->depth > 0) j->depth--;
    if (j->pretty && j->depth > 0 &&
        (j->depth < CLI_JSON_MAX_DEPTH && j->need_comma[j->depth])) {
        /* 只有当对象非空时才换行 */
        int k;
        fputc('\n', j->f);
        for (k = 0; k < j->depth; ++k) fputs("  ", j->f);
    }
    fputc('}', j->f);
    if (j->depth < CLI_JSON_MAX_DEPTH) j->need_comma[j->depth] = 1;
}

void cli_json_arr_begin(cli_json_t *j, const char *key)
{
    j_key(j, key);
    fputc('[', j->f);
    j_push(j);
}

void cli_json_arr_end(cli_json_t *j)
{
    if (j->depth > 0) j->depth--;
    if (j->pretty) fputc('\n', j->f);
    fputc(']', j->f);
    if (j->depth < CLI_JSON_MAX_DEPTH) j->need_comma[j->depth] = 1;
}

void cli_json_num(cli_json_t *j, const char *key, double v)
{
    j_key(j, key);
    fprintf(j->f, "%.6g", v);
}

void cli_json_i64(cli_json_t *j, const char *key, long long v)
{
    j_key(j, key);
    fprintf(j->f, "%lld", v);
}

void cli_json_bool(cli_json_t *j, const char *key, int v)
{
    j_key(j, key);
    fputs(v ? "true" : "false", j->f);
}

void cli_json_null(cli_json_t *j, const char *key)
{
    j_key(j, key);
    fputs("null", j->f);
}

void cli_json_raw_num(cli_json_t *j, double v)
{
    j_key(j, NULL);
    fprintf(j->f, "%.6g", v);
}

void cli_json_escape(FILE *f, const char *s)
{
    fputc('"', f);
    if (s) {
        const unsigned char *p = (const unsigned char *)s;
        for (; *p; ++p) {
            unsigned char c = *p;
            switch (c) {
            case '"':  fputs("\\\"", f); break;
            case '\\': fputs("\\\\", f); break;
            case '\n': fputs("\\n", f);  break;
            case '\r': fputs("\\r", f);  break;
            case '\t': fputs("\\t", f);  break;
            default:
                if (c < 0x20u) {
                    fprintf(f, "\\u%04x", (unsigned)c);
                } else {
                    fputc((int)c, f);
                }
                break;
            }
        }
    }
    fputc('"', f);
}

void cli_json_str(cli_json_t *j, const char *key, const char *v)
{
    j_key(j, key);
    cli_json_escape(j->f, v);
}
