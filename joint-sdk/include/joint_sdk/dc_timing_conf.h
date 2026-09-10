#ifndef JSDK_DC_TIMING_CONF_H
#define JSDK_DC_TIMING_CONF_H

#include <stddef.h>
#include <stdint.h>

#include <joint_sdk/drive_model.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Persistent DC timing file (PREOP / configure), not motion CLI.
 * Default path: ./config/dc_timing.conf (cwd) or $JSDK_DC_TIMING_CONF.
 *
 * jsdk_dc_timing_conf_apply() is declared on jsdk_context_config_t
 * in joint_sdk.h (avoids include cycles).
 */
#define JSDK_DC_TIMING_CONF_DEFAULT "config/dc_timing.conf"

const char *jsdk_dc_timing_conf_path(void);

/**
 * Per-fw override from conf (PER_JOINT). Returns 1 if found.
 */
int jsdk_dc_timing_conf_lookup_fw(const char *fw,
        int *use_dc,
        int32_t *sync0_shift_ns,
        uint32_t *wait_ms,
        uint32_t *preferred_period_ns,
        uint32_t *sync0_cycle_ns);

/** Global task period from conf (0 if unset). */
uint32_t jsdk_dc_timing_conf_period_ns(void);

/** 1 if a [fw] section exists (substring or exact). */
int jsdk_dc_timing_conf_has_fw(const char *fw);

/**
 * Build a drive_model row from conf [fw] for unknown firmware.
 * name/fw_match/notes point into the caller buffers. Returns 1 if filled.
 */
int jsdk_dc_timing_conf_make_model(const char *fw,
        jsdk_drive_model_t *out,
        char *name_buf, size_t name_sz,
        char *fw_buf, size_t fw_sz,
        char *notes_buf, size_t notes_sz);

#ifdef __cplusplus
}
#endif

#endif /* JSDK_DC_TIMING_CONF_H */
