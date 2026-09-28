/* Flat, stable C API over the firmware flood-detection core. This is the only
 * code that touches firmware structs: if the firmware changes a struct or a
 * signature, the compiler flags it here and Python is unaffected. */
#ifndef SIM_SHIM_H
#define SIM_SHIM_H

#include <stdint.h>

#ifdef _WIN32
#define SIM_API __declspec(dllexport)
#else
#define SIM_API
#endif

#define SIM_ABI_VERSION        3
#define SIM_FRAME_W            1920
#define SIM_FRAME_H            1080
#define SIM_MAX_BOXES          32     /* FLOOD_VETO_MAX_BOXES */
#define SIM_GEOMETRY_LEN       8
#define SIM_OUT_SKIPPED_NO_REF 4      /* after FLOOD_OUT_UPLOAD..HEARTBEAT (0..3) */

typedef struct {
    float   x0, y0, x1, y1;           /* frame coordinates, 1920x1080 */
    float   confidence;
    int32_t cls;                      /* flood_class_t value */
} sim_box_t;

typedef struct {
    int32_t veto_usable;
    float   veto_coverage;
    int32_t veto_worst_class;
    float   curb_correlation;
    int32_t submerged;
    int32_t severity;
    int32_t shift_dx;
    int32_t shift_dy;
    int32_t gallery_slot;             /* slot that scored best, -1 if none */
} sim_score_t;

typedef struct {
    int32_t outcome;                  /* flood_outcome_t, or SIM_OUT_SKIPPED_NO_REF */
    int32_t frames_taken;
    int32_t has_reading;
    float   curb_correlation;
    int32_t severity;
    int32_t shift_dx;
    int32_t shift_dy;
    int32_t veto_usable;              /* veto of the last attempt that ran detect */
    float   veto_coverage;
    int32_t veto_worst_class;
    int32_t since_image;              /* capture state after this sample */
    int32_t frame_idx;                /* last frame grabbed successfully, -1 if none */
    int32_t gallery_slot;             /* slot that matched, -1 unless has_reading */
    int32_t gallery_source;           /* flood_ref_source_t of that slot, -1 if none */
    int32_t admitted;                 /* this sample auto-admitted a new reference */
    int32_t notice;                   /* this sample raised the "no confident match" notice */
    int32_t gallery_count;            /* in-use slots after this sample */
} sim_result_t;

/* Fills the frame for `attempt` (0 = sample time, 1 = retry). gray_out holds
 * SIM_FRAME_W*SIM_FRAME_H bytes, boxes_out SIM_MAX_BOXES entries. Returns 1 if
 * a frame was delivered, 0 if none (the firmware treats that as a failed grab). */
typedef int32_t (*sim_provider_fn)(int32_t attempt, uint8_t *gray_out, sim_box_t *boxes_out,
                                   int32_t *n_boxes, int32_t *frame_idx, void *user);

/* ---- ABI, names, constants ---- */
SIM_API int32_t     sim_abi_version(void);
SIM_API int32_t     sim_sizeof_box(void);
SIM_API int32_t     sim_sizeof_score(void);
SIM_API int32_t     sim_sizeof_result(void);
SIM_API const char *sim_outcome_name(int32_t outcome);
SIM_API const char *sim_severity_name(int32_t severity);
SIM_API const char *sim_class_name(int32_t cls);
SIM_API float       sim_submerged_below(void);
SIM_API float       sim_veto_threshold(void);
SIM_API float       sim_sample_period_s(void);
SIM_API int32_t     sim_upload_downscale(void);

/* ---- geometry ---- */
SIM_API void    sim_default_geometry(float *out8);
SIM_API void    sim_get_geometry(float *out8);
SIM_API void    sim_set_geometry(const float *in8);
SIM_API int32_t sim_validate_geometry(const float *in8, char *err, int32_t err_size);
SIM_API float   sim_domain_x0(void);
SIM_API float   sim_domain_x1(void);
SIM_API void    sim_align_patch(int32_t *out4);
SIM_API int32_t sim_strip_outline(float *xs, float *y_lo, float *y_hi, int32_t max_points);

/* ---- frames ---- */
SIM_API void sim_rgb_to_gray(const uint8_t *rgb, int32_t width, int32_t height, uint8_t *gray_out);

/* ---- reference gallery and scoring ----
 * A gallery wraps the firmware flood_gallery_t. Manual references are built
 * from a dry frame; AUTO references are only ever admitted by
 * sim_device_sample(), as flood_task.c does once per sampling cycle. */
SIM_API void   *sim_gallery_new(void);
SIM_API void    sim_gallery_free(void *gallery);
SIM_API void    sim_gallery_clear(void *gallery);
SIM_API int32_t sim_gallery_max_slots(void);
SIM_API int32_t sim_gallery_count(const void *gallery);
/* 1 = added; 0 = geometry leaves no usable sampling window; -1 = gallery full
 * of manual references (flood_task.c's flood_do_calibration() failure). */
SIM_API int32_t sim_gallery_add_manual(void *gallery, const uint8_t *gray);
/* Slot `index`: returns 1 if in use, filling source (flood_ref_source_t) and
 * the cycle it last matched; 0 if unused or out of range. */
SIM_API int32_t sim_gallery_slot(const void *gallery, int32_t index, int32_t *source,
                                 uint32_t *last_matched_cycle);
SIM_API float   sim_correlation_at(const void *gallery, int32_t slot, const uint8_t *gray,
                                   int32_t dx, int32_t dy);
SIM_API void    sim_box_from_accelerator(float x0, float y0, float x1, float y1, float confidence,
                                         int32_t cls, int32_t input_w, int32_t input_h,
                                         sim_box_t *out);
/* Read-only: scores against every slot, keeps the best; never admits. */
SIM_API int32_t sim_score(const void *gallery, const uint8_t *gray, const sim_box_t *boxes,
                          int32_t n_boxes, sim_score_t *out);
SIM_API float   sim_band_coverage(const sim_box_t *box);
SIM_API int32_t sim_heartbeat_every(void);

/* ---- device loop (Task 2) ---- */
SIM_API void *sim_device_new(void);
SIM_API void  sim_device_free(void *dev);
SIM_API void  sim_device_reset(void *dev);
/* One firmware sampling cycle; may admit an AUTO reference into `gallery`. */
SIM_API void  sim_device_sample(void *dev, void *gallery, sim_provider_fn provider,
                                void *user, sim_result_t *out);

#endif /* SIM_SHIM_H */
