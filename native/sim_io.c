#include "sim_io.h"

#include <ctype.h>
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void set_err(char *err, size_t err_size, const char *fmt, const char *arg)
{
    if (err && err_size) snprintf(err, err_size, fmt, arg);
}

static char *copy_string(const char *s)
{
    const size_t n = strlen(s) + 1;
    char *c = malloc(n);
    if (c) memcpy(c, s, n);
    return c;
}

/* ------------------------------------------------------------------- PGM */

/* Next header integer, skipping whitespace and # comments. Consumes the one
 * whitespace byte after the number, as the PGM format requires. */
static int pgm_next_int(FILE *f)
{
    int c = fgetc(f);
    for (;;) {
        while (c != EOF && isspace(c)) c = fgetc(f);
        if (c != '#') break;
        while (c != EOF && c != '\n') c = fgetc(f);
    }
    if (c == EOF || !isdigit(c)) return -1;
    int v = 0;
    while (c != EOF && isdigit(c)) {
        v = v * 10 + (c - '0');
        c = fgetc(f);
    }
    return v;
}

bool sim_read_pgm(const char *path, uint8_t *gray_out, char *err, size_t err_size)
{
    FILE *f = fopen(path, "rb");
    if (!f) {
        set_err(err, err_size, "cannot open %s", path);
        return false;
    }
    char magic[2];
    const bool is_p5 = fread(magic, 1, 2, f) == 2 && magic[0] == 'P' && magic[1] == '5';
    const int w = is_p5 ? pgm_next_int(f) : -1;
    const int h = is_p5 ? pgm_next_int(f) : -1;
    const int maxval = is_p5 ? pgm_next_int(f) : -1;
    if (!is_p5 || w != SIM_FRAME_W || h != SIM_FRAME_H || maxval != 255) {
        fclose(f);
        set_err(err, err_size, "%s is not a 1920x1080 8-bit binary PGM", path);
        return false;
    }
    const size_t n = (size_t)SIM_FRAME_W * SIM_FRAME_H;
    const bool ok = fread(gray_out, 1, n, f) == n;
    fclose(f);
    if (!ok) set_err(err, err_size, "%s: truncated pixel data", path);
    return ok;
}

/* ------------------------------------------------------------ frame list */

typedef struct {
    char **items;
    int    n, cap;
} strlist_t;

static bool strlist_push(strlist_t *l, const char *s)
{
    if (l->n == l->cap) {
        const int cap = l->cap ? l->cap * 2 : 64;
        char **grown = realloc(l->items, (size_t)cap * sizeof(char *));
        if (!grown) return false;
        l->items = grown;
        l->cap = cap;
    }
    l->items[l->n] = copy_string(s);
    return l->items[l->n++] != NULL;
}

static void strlist_free(strlist_t *l)
{
    for (int i = 0; i < l->n; ++i) free(l->items[i]);
    free(l->items);
}

static int cmp_strings(const void *a, const void *b)
{
    return strcmp(*(char *const *)a, *(char *const *)b);
}

static bool has_pgm_suffix(const char *name)
{
    const size_t n = strlen(name);
    if (n < 4) return false;
    const char *s = name + n - 4;
    return s[0] == '.' && tolower((unsigned char)s[1]) == 'p' &&
           tolower((unsigned char)s[2]) == 'g' && tolower((unsigned char)s[3]) == 'm';
}

/* f<idx>_t<ms>.pgm as written by `floodsim export`; either part optional. */
static void parse_frame_name(const char *path, int32_t *idx, long long *t_ms, bool *has_t)
{
    const char *base = path;
    for (const char *p = path; *p; ++p)
        if (*p == '/' || *p == '\\') base = p + 1;
    *idx = -1;
    *has_t = false;
    if (base[0] == 'f' && isdigit((unsigned char)base[1])) *idx = (int32_t)strtol(base + 1, NULL, 10);
    const char *t = strstr(base, "_t");
    if (t && isdigit((unsigned char)t[2])) {
        *t_ms = strtoll(t + 2, NULL, 10);
        *has_t = true;
    }
}

static int cmp_frames(const void *a, const void *b)
{
    const sim_frame_t *x = a, *y = b;
    if (x->t_sec != y->t_sec) return x->t_sec < y->t_sec ? -1 : 1;
    return (x->frame_idx > y->frame_idx) - (x->frame_idx < y->frame_idx);
}

int sim_list_frames(const char *spec, double fps, sim_frame_t **out, char *err, size_t err_size)
{
    strlist_t names = { 0 };
    DIR *dir = opendir(spec);
    if (dir) {
        struct dirent *e;
        char full[SIM_PATH_MAX];
        while ((e = readdir(dir)) != NULL) {
            if (!has_pgm_suffix(e->d_name)) continue;
            snprintf(full, sizeof full, "%s/%s", spec, e->d_name);
            if (!strlist_push(&names, full)) break;
        }
        closedir(dir);
        if (names.n > 1) qsort(names.items, (size_t)names.n, sizeof(char *), cmp_strings);
    } else {
        FILE *f = fopen(spec, "r");
        if (!f) {
            set_err(err, err_size, "cannot open frames %s", spec);
            return -1;
        }
        char line[SIM_PATH_MAX];
        while (fgets(line, sizeof line, f)) {
            size_t len = strlen(line);
            while (len && isspace((unsigned char)line[len - 1])) line[--len] = '\0';
            if (len == 0 || line[0] == '#') continue;
            if (!strlist_push(&names, line)) break;
        }
        fclose(f);
    }
    if (names.n == 0) {
        strlist_free(&names);
        set_err(err, err_size, "no .pgm frames in %s", spec);
        return -1;
    }

    sim_frame_t *frames = calloc((size_t)names.n, sizeof(*frames));
    if (!frames) {
        strlist_free(&names);
        set_err(err, err_size, "out of memory listing %s", spec);
        return -1;
    }
    for (int i = 0; i < names.n; ++i) {
        int32_t idx;
        long long t_ms = 0;
        bool has_t;
        snprintf(frames[i].path, sizeof frames[i].path, "%s", names.items[i]);
        parse_frame_name(names.items[i], &idx, &t_ms, &has_t);
        frames[i].frame_idx = idx >= 0 ? idx : i;
        frames[i].t_sec = has_t ? (double)t_ms / 1000.0 : (double)i / fps;
    }
    const int n = names.n;
    strlist_free(&names);
    qsort(frames, (size_t)n, sizeof(*frames), cmp_frames);
    *out = frames;
    return n;
}

int sim_frame_at(const sim_frame_t *frames, int n, double t)
{
    for (int i = n - 1; i >= 0; --i)
        if (frames[i].t_sec <= t + 1e-9) return i;
    return -1;
}

/* ----------------------------------------------------------------- boxes */

int sim_read_boxes(const char *path, sim_box_rec_t **out, char *err, size_t err_size)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        set_err(err, err_size, "cannot open boxes %s", path);
        return -1;
    }
    sim_box_rec_t *recs = NULL;
    int n = 0, cap = 0;
    char line[512];
    while (fgets(line, sizeof line, f)) {
        if (line[0] == '#' || strncmp(line, "frame_idx", 9) == 0) continue;
        int idx, cls;
        double t;
        float x0, y0, x1, y1, conf;
        char coco[64];
        /* A no-detection marker row has empty fields and fails here: skipped. */
        if (sscanf(line, "%d,%lf,%f,%f,%f,%f,%f,%63[^,],%d",
                   &idx, &t, &x0, &y0, &x1, &y1, &conf, coco, &cls) != 9) continue;
        if (n == cap) {
            cap = cap ? cap * 2 : 64;
            sim_box_rec_t *grown = realloc(recs, (size_t)cap * sizeof(*recs));
            if (!grown) break;
            recs = grown;
        }
        recs[n].frame_idx = idx;
        recs[n].box.x0 = x0; recs[n].box.y0 = y0;
        recs[n].box.x1 = x1; recs[n].box.y1 = y1;
        recs[n].box.confidence = conf;
        recs[n].box.cls = cls;
        ++n;
    }
    fclose(f);
    *out = recs;
    return n;
}

int sim_boxes_for(const sim_box_rec_t *recs, int n_recs, int32_t frame_idx,
                  sim_box_t *out, int max_out)
{
    int n = 0;
    for (int i = 0; i < n_recs && n < max_out; ++i)
        if (recs[i].frame_idx == frame_idx) out[n++] = recs[i].box;
    return n;
}

/* --------------------------------------------------------------- geometry */

static const char *find_object(const char *json, const char *key)
{
    char pat[32];
    snprintf(pat, sizeof pat, "\"%s\"", key);
    const char *p = strstr(json, pat);
    return p ? strchr(p + strlen(pat), '{') : NULL;
}

static bool read_number(const char *obj, const char *key, float *out)
{
    const char *end = strchr(obj, '}');
    char pat[16];
    snprintf(pat, sizeof pat, "\"%s\"", key);
    const char *p = strstr(obj, pat);
    if (!end || !p || p > end) return false;
    p = strchr(p + strlen(pat), ':');
    if (!p || p > end) return false;
    char *num_end;
    const double v = strtod(p + 1, &num_end);
    if (num_end == p + 1) return false;
    *out = (float)v;
    return true;
}

bool sim_read_geometry_json(const char *path, float out[SIM_GEOMETRY_LEN], char *err, size_t err_size)
{
    FILE *f = fopen(path, "rb");
    if (!f) {
        set_err(err, err_size, "cannot open geometry %s", path);
        return false;
    }
    fseek(f, 0, SEEK_END);
    const long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    char *buf = size > 0 ? malloc((size_t)size + 1) : NULL;
    const bool read_ok = buf && fread(buf, 1, (size_t)size, f) == (size_t)size;
    fclose(f);
    if (!read_ok) {
        free(buf);
        set_err(err, err_size, "cannot read geometry %s", path);
        return false;
    }
    buf[size] = '\0';

    static const char *const lines[2] = { "curb", "lane" };
    static const char *const keys[4] = { "x0", "y0", "x1", "y1" };
    bool ok = true;
    for (int l = 0; l < 2 && ok; ++l) {
        const char *obj = find_object(buf, lines[l]);
        if (!obj) {
            set_err(err, err_size, "geometry: missing \"%s\" object", lines[l]);
            ok = false;
            break;
        }
        for (int k = 0; k < 4 && ok; ++k) {
            if (!read_number(obj, keys[k], &out[l * 4 + k])) {
                set_err(err, err_size, "geometry: %s is missing a number", lines[l]);
                ok = false;
            }
        }
    }
    free(buf);
    return ok;
}
