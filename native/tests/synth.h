/* Deterministic synthetic frames. Dry: a hashed texture everywhere, so the
 * kerb strip and the alignment patch both have structure. Flooded: the same
 * frame with rows [SYNTH_FLOOD_Y0, SYNTH_FLOOD_Y1) set to a constant -- the
 * whole kerb strip (y ~200..300 under the default geometry) loses its
 * structure, while the alignment patch (y 0..190) is untouched. */
#ifndef SYNTH_H_INCLUDED
#define SYNTH_H_INCLUDED

#include <stdint.h>

#define SYNTH_W 1920
#define SYNTH_H 1080
#define SYNTH_FLOOD_Y0 195
#define SYNTH_FLOOD_Y1 500

void synth_dry(uint8_t *gray);
void synth_flooded(uint8_t *gray);

/* Dry, but the kerb rows blended with an unrelated texture (a stand-in for a
 * lighting change): correlation against synth_dry is about
 * dry_weight / sqrt(dry_weight^2 + other_weight^2). */
void synth_relit(uint8_t *gray, int dry_weight, int other_weight, int seed);

/* Water in the kerb strip, measured up from the strip's bottom edge: level 0 = none,
 * 1/7 = the gutter lip only (the strip spans 30 % above to 5 % below the curb foot),
 * 1 = the whole strip. The outline comes from sim_strip_outline() (columns 2 px apart).
 * Water pixels are (128 - amplitude / 2) + texture % amplitude: amplitude 8 = still,
 * almost featureless water; 256 = moving water that differs with every `seed`. */
void synth_water(uint8_t *gray, const float *xs, const float *y_lo, const float *y_hi, int n,
                 float level, int seed, int amplitude);

#endif
