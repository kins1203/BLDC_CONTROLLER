#include "command.h"
#include <string.h>
#include <ctype.h>

/* ================= INTERNAL UTILS ================= */

static void trim(char *s)
{
    char *p = s;
    char *end;

    /* bỏ space đầu */
    while (*p == ' ' || *p == '\r' || *p == '\n')
        p++;

    memmove(s, p, strlen(p) + 1);

    /* bỏ space cuối */
    end = s + strlen(s) - 1;
    while (end >= s && (*end == ' ' || *end == '\r' || *end == '\n'))
    {
        *end = '\0';
        end--;
    }
}

/* parse float bằng cách tách dấu '.' */
static int parse_float_dot(const char *s, float *out)
{
    int sign = 1;
    int ip = 0;
    int dp = 0;
    float scale = 1.0f;
    int has_digit = 0;

    while (*s == ' ')
        s++;

    if (*s == '-')
    {
        sign = -1;
        s++;
    }
    else if (*s == '+')
    {
        s++;
    }

    while (isdigit((unsigned char)*s))
    {
        ip = ip * 10 + (*s - '0');
        has_digit = 1;
        s++;
    }

    if (*s == '.')
    {
        s++;
        while (isdigit((unsigned char)*s))
        {
            dp = dp * 10 + (*s - '0');
            scale *= 0.1f;
            has_digit = 1;
            s++;
        }
    }

    if (!has_digit)
        return 0;

    *out = sign * (ip + dp * scale);
    return 1;
}

/* ================= PUBLIC API ================= */

int Command_Parse(char *input, CmdParsed_t *out)
{
    char *comma;
    char *eq;

    if (!input || !out)
        return 0;

    memset(out, 0, sizeof(CmdParsed_t));

    trim(input);

    /* tìm dấu ',' */
    comma = strchr(input, ',');

    /* ---- chỉ có CMD ---- */
    if (!comma)
    {
        strncpy(out->cmd, input, sizeof(out->cmd) - 1);
        out->has_key = 0;
        out->has_value = 0;
        return 1;
    }

    /* ---- CMD ---- */
    *comma = '\0';
    trim(input);
    strncpy(out->cmd, input, sizeof(out->cmd) - 1);

    /* ---- KEY = VALUE ---- */
    char *kv = comma + 1;
    trim(kv);

    eq = strchr(kv, '=');
    if (!eq)
    {
        out->has_key = 0;
        out->has_value = 0;
        return 1;
    }

    *eq = '\0';
    trim(kv);
    trim(eq + 1);

    strncpy(out->key, kv, sizeof(out->key) - 1);
    out->has_key = 1;

    if (parse_float_dot(eq + 1, &out->value))
        out->has_value = 1;
    else
        out->has_value = 0;

    return 1;
}
