/* Deterministic synthetic frames. Dry: a hashed texture everywhere, so the
 * kerb strip and the alignment patch both have structure. Flooded: the same
 * frame with rows [SYNTH_FLOOD_Y0, SYNTH_FLOOD_Y1) set to a constant -- the
 * whole kerb strip (y ~200..300 under the default geometry) loses its
 * structure, while the alignment patch (y 0..190) is untouched. */
#ifndef SYNTH_H
#define SYNTH_H

#include <stdint.h>

#define SYNTH_W 1920
#define SYNTH_H 1080
#define SYNTH_FLOOD_Y0 195
#define SYNTH_FLOOD_Y1 500

void synth_dry(uint8_t *gray);
void synth_flooded(uint8_t *gray);

#endif
