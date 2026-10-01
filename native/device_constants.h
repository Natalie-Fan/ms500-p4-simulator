/* Values mirrored from firmware device glue that the simulator cannot compile
 * (it needs FreeRTOS, the camera, SPIFFS). Each names the firmware file and
 * line it was copied from, at firmware commit 58d8b5d. When one of those
 * files changes, re-check the matching value here -- nothing else notices.
 *
 * Mirrored on the Python side, listed here so this stays the one index:
 *   floodsim/boxes.py  COCO_TO_FLOOD  <- main/ai_process/flood_task.c:117-127 map_traffic_class
 *   floodsim/modes.py  UploadWriter label "%06X", sequence per upload
 *                                       <- main/ai_process/flood_task.c:248
 *   sim_shim.c sim_device_sample() empty-gallery gate + flood_gallery_after_sample()
 *                                       <- main/ai_process/flood_task.c:365-377
 *   sim_shim.c sim_validate_geometry()  <- main/ai_process/flood_geometry_config.c:118-160
 */
#ifndef DEVICE_CONSTANTS_H
#define DEVICE_CONSTANTS_H

/* main/ai_process/flood_task.c:30  FLOOD_SAMPLE_PERIOD_MS 60000 */
#define SIM_SAMPLE_PERIOD_S 60.0f

/* main/bsp_video/flood_frame_tap.c:112  (r * 77 + g * 151 + b * 28) >> 8 */
#define SIM_GRAY_R     77
#define SIM_GRAY_G     151
#define SIM_GRAY_B     28
#define SIM_GRAY_SHIFT 8

/* main/ai_process/flood_geometry_config.c:28-31  g_flood_geometry defaults,
 * in flood_geometry_t field order. */
#define SIM_GEOMETRY_DEFAULT_INIT \
    { 560.0f, 230.0f, 1920.0f, 355.0f, 560.0f, 378.0f, 1920.0f, 446.0f }

/* main/ai_process/flood_geometry_config.c:22  FLOOD_GEOMETRY_MIN_DOMAIN_WIDTH
 * (needs flood_geometry.h included first). */
#define SIM_GEOMETRY_MIN_DOMAIN_WIDTH (FLOOD_STRIP_OFFSET + 40.0f)

/* main/bsp_video/flood_roi.h:38  FLOOD_ROI_DOWNSCALE */
#define SIM_UPLOAD_DOWNSCALE 2

#endif /* DEVICE_CONSTANTS_H */
