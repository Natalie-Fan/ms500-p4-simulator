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

#endif
