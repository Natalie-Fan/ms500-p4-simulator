/* File readers for flood_sim.exe. Standard C library only. */
#ifndef SIM_IO_H
#define SIM_IO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sim_shim.h"

#define SIM_PATH_MAX 1024

typedef struct {
    char    path[SIM_PATH_MAX];
    int32_t frame_idx;
    double  t_sec;
} sim_frame_t;

typedef struct {
    int32_t   frame_idx;
    sim_box_t box;
} sim_box_rec_t;

/* Reads a 1920x1080 8-bit binary PGM (P5, maxval 255) into gray_out. */
bool sim_read_pgm(const char *path, uint8_t *gray_out, char *err, size_t err_size);

/* `spec` is a directory of *.pgm or a text file with one path per line.
 * Returns the frame count (sorted by t_sec, then frame_idx) or -1; caller
 * frees *out. */
int sim_list_frames(const char *spec, double fps, sim_frame_t **out, char *err, size_t err_size);

/* Latest frame with t_sec <= t, or -1. */
int sim_frame_at(const sim_frame_t *frames, int n, double t);

/* Returns the number of box rows (malformed rows skipped) or -1; caller frees *out. */
int sim_read_boxes(const char *path, sim_box_rec_t **out, char *err, size_t err_size);

/* Copies the boxes of frame_idx into out (at most max_out); returns the count. */
int sim_boxes_for(const sim_box_rec_t *recs, int n_recs, int32_t frame_idx,
                  sim_box_t *out, int max_out);

/* Reads {"curb":{"x0","y0","x1","y1"},"lane":{...}} (the firmware web API shape). */
bool sim_read_geometry_json(const char *path, float out[SIM_GEOMETRY_LEN], char *err, size_t err_size);

#endif /* SIM_IO_H */
