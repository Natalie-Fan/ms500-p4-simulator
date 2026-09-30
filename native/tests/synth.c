#include "synth.h"

#include <math.h>
#include <string.h>

static uint8_t texture(int x, int y)
{
    uint32_t h = (uint32_t)x * 73856093u ^ (uint32_t)y * 19349663u;
    h ^= h >> 13;
    h *= 0x5bd1e995u;
    h ^= h >> 15;
    return (uint8_t)(h >> 8);
}

void synth_dry(uint8_t *gray)
{
    for (int y = 0; y < SYNTH_H; ++y)
        for (int x = 0; x < SYNTH_W; ++x)
            gray[(size_t)y * SYNTH_W + x] = texture(x, y);
}

void synth_flooded(uint8_t *gray)
{
    synth_dry(gray);
    for (int y = SYNTH_FLOOD_Y0; y < SYNTH_FLOOD_Y1; ++y)
        memset(gray + (size_t)y * SYNTH_W, 128, SYNTH_W);
}

void synth_relit(uint8_t *gray, int dry_weight, int other_weight, int seed)
{
    synth_dry(gray);
    const int total = dry_weight + other_weight;
    for (int y = SYNTH_FLOOD_Y0; y < SYNTH_FLOOD_Y1; ++y)
        for (int x = 0; x < SYNTH_W; ++x) {
            uint8_t *p = gray + (size_t)y * SYNTH_W + x;
            *p = (uint8_t)((dry_weight * *p + other_weight * texture(x + seed, y + 2 * seed)) / total);
        }
}

void synth_water(uint8_t *gray, const float *xs, const float *y_lo, const float *y_hi, int n,
                 float level, int seed, int amplitude)
{
    const int base = 128 - amplitude / 2;
    for (int c = 0; c < n; ++c) {
        const float top = y_hi[c] - level * (y_hi[c] - y_lo[c]);
        const int y0 = (int)ceilf(top) < 0 ? 0 : (int)ceilf(top);
        const int y1 = (int)y_hi[c] + 1;   /* the strip samples rows up to (int)y_hi */
        const int x0 = (int)xs[c];
        for (int y = y0; y <= y1 && y < SYNTH_H; ++y)
            for (int x = x0; x < x0 + 2 && x < SYNTH_W; ++x)   /* FLOOD_STRIP_XSTEP */
                gray[(size_t)y * SYNTH_W + x] =
                    (uint8_t)(base + texture(x + seed, y + 2 * seed) % amplitude);
    }
}
