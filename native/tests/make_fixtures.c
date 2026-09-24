/* Writes the golden-test inputs: reference.pgm, frames/f<idx>_t<ms>.pgm,
 * boxes.csv, geometry.json (firmware defaults) and bad_geometry.json. */
#include "synth.h"

#include <stdio.h>
#include <stdlib.h>
#ifdef _WIN32
#include <direct.h>
#define MAKE_DIR(p) _mkdir(p)
#else
#include <sys/stat.h>
#define MAKE_DIR(p) mkdir((p), 0755)
#endif

static int write_pgm(const char *path, const uint8_t *gray)
{
    FILE *f = fopen(path, "wb");
    if (!f) return 0;
    fprintf(f, "P5\n%d %d\n255\n", SYNTH_W, SYNTH_H);
    const size_t n = (size_t)SYNTH_W * SYNTH_H;
    const int ok = fwrite(gray, 1, n, f) == n;
    fclose(f);
    return ok;
}

static int write_text(const char *path, const char *text)
{
    FILE *f = fopen(path, "wb");
    if (!f) return 0;
    const int ok = fputs(text, f) >= 0;
    fclose(f);
    return ok;
}

int main(int argc, char **argv)
{
    if (argc != 2) {
        fputs("usage: make_fixtures <out_dir>\n", stderr);
        return 2;
    }
    const char *dir = argv[1];
    char path[1024];
    MAKE_DIR(dir);
    snprintf(path, sizeof path, "%s/frames", dir);
    MAKE_DIR(path);

    uint8_t *dry = malloc((size_t)SYNTH_W * SYNTH_H);
    uint8_t *flood = malloc((size_t)SYNTH_W * SYNTH_H);
    if (!dry || !flood) return 2;
    synth_dry(dry);
    synth_flooded(flood);

    const struct { int idx; long long t_ms; const uint8_t *gray; } frames[] = {
        { 0, 0, dry }, { 1, 1000, dry }, { 2, 60000, flood },
        { 3, 61000, flood }, { 4, 120000, dry }, { 5, 121000, dry },
    };

    int ok = 1;
    snprintf(path, sizeof path, "%s/reference.pgm", dir);
    ok &= write_pgm(path, dry);
    for (size_t i = 0; i < sizeof frames / sizeof frames[0]; ++i) {
        snprintf(path, sizeof path, "%s/frames/f%06d_t%010lld.pgm", dir, frames[i].idx, frames[i].t_ms);
        ok &= write_pgm(path, frames[i].gray);
    }
    snprintf(path, sizeof path, "%s/boxes.csv", dir);
    ok &= write_text(path,
        "# fixture: frame 4 has a car across the whole kerb\n"
        "frame_idx,t_sec,x0,y0,x1,y1,conf,coco_class,flood_class\n"
        "4,120.000,0,0,1919,1079,0.9,car,2\n"
        "5,121.000,,,,,,,\n");
    snprintf(path, sizeof path, "%s/geometry.json", dir);
    ok &= write_text(path,
        "{\"curb\": {\"x0\": 560, \"y0\": 230, \"x1\": 1920, \"y1\": 355},\n"
        " \"lane\": {\"x0\": 560, \"y0\": 378, \"x1\": 1920, \"y1\": 446}}\n");
    snprintf(path, sizeof path, "%s/bad_geometry.json", dir);
    ok &= write_text(path,
        "{\"curb\": {\"x0\": 560, \"y0\": 230, \"x1\": 560, \"y1\": 355},\n"
        " \"lane\": {\"x0\": 560, \"y0\": 378, \"x1\": 1920, \"y1\": 446}}\n");

    free(dry);
    free(flood);
    if (!ok) {
        fprintf(stderr, "make_fixtures: failed writing into %s\n", dir);
        return 1;
    }
    return 0;
}
