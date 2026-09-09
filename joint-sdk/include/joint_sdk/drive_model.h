#ifndef JSDK_DRIVE_MODEL_H
#define JSDK_DRIVE_MODEL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Multi-axis DC timing policy (also on jsdk_context_config_t.dc_timing_mode).
 *
 * JSDK_DC_TIMING_SHARED (0) — step 1: every slave gets the same Sync0 /
 *   shift / wait from context (usually filled from --fw0 / first model).
 * JSDK_DC_TIMING_PER_JOINT (1) — step 2: each slave uses its own
 *   drive_model row (sync0_cycle_ns, sync0_shift_ns, wait, use_dc,
 *   AssignActivate). Identical table rows ⇒ same timing as SHARED.
 *
 * Default compile-time switch (no motion CLI):
 *   edit JSDK_DC_TIMING_MODE_DEFAULT, or use tool dc_timing_setup
 *   which writes config/dc_timing.conf (loaded at PREOP configure).
 *
 * New motor / fw: one call
 *   ./build/dc_timing_setup --per-joint --fw X.Y.Z --sync0-cycle ...
 * upserts that section only (other [fw] kept). Runtime find() uses the
 * conf row if missing from g_models[]; add a static row here only when
 * you need csp_sw_invert or a rebuild-time default.
 *
 * Note: application task period (period_ns) is always one per master
 * context from conf global period_ns / caller — not per-fw
 * preferred_period_ns (that field only feeds Sync0 when sync0_cycle_ns=0).
 * Only Sync0/shift/wait/AssignActivate can differ per joint.
 */
#define JSDK_DC_TIMING_SHARED     0
#define JSDK_DC_TIMING_PER_JOINT  1
#ifndef JSDK_DC_TIMING_MODE_DEFAULT
#define JSDK_DC_TIMING_MODE_DEFAULT JSDK_DC_TIMING_SHARED
#endif

/**
 * Drive-model (型号 + 固件) DC / timing parameters.
 *
 * Engineering practice: keep DC AssignActivate / Sync0 shift / settle time
 * per drive firmware, not as one global hard-code. After FreeRun proves
 * PDO+CiA402, enable DC and raise sync0_shift from 100000 ns until OP is
 * stable; freeze that value for the model row (scheme B: edit this table
 * and rebuild).
 */
typedef struct jsdk_drive_model {
    const char *name;              /* e.g. "ISVD90RC-v8.1.50" */
    uint32_t vendor_id;
    uint32_t product_code;
    /** Match substring of CoE 0x100A (Manufacturer software version). */
    const char *fw_match;
    int use_dc;                    /* 1 = Sync0, 0 = FreeRun */
    uint16_t dc_assign_activate;   /* typically 0x0300 */
    int32_t sync0_shift_ns;        /* Sync0 shift; -1 = period/4 in transport */
    uint32_t wait_before_safeop_ms;
    /**
     * Preferred Sync0 cycle fallback when sync0_cycle_ns == 0.
     * Not the application task period — that is conf/context period_ns.
     * 0 = use caller / conf global period_ns for Sync0 fallback.
     */
    uint32_t preferred_period_ns;
    /**
     * Sync0 cycle time for ecrt_slave_config_dc().
     * 0 = same as preferred_period_ns / period_ns (legacy 1:1).
     * TwinCAT may use Sync Unit Cycle × N (e.g. 2 ms × 3 = 6 ms).
     */
    uint32_t sync0_cycle_ns;
    /**
     * CSP software position invert for this plant (write = 2*act-cmd).
     * 1 = enable soft-invert in diags when --fw matches this model.
     * (Not CoE 0x607E.) Dual CyberBeast bench: 8.1.50=1, 8.1.60=0.
     */
    int csp_sw_invert;
    const char *notes;
} jsdk_drive_model_t;

/** Default model used when firmware is unknown (conservative FreeRun). */
const jsdk_drive_model_t *jsdk_drive_model_default(void);

/**
 * Lookup by firmware string (0x100A) and optional product code.
 * product_code 0 = ignore product filter.
 */
const jsdk_drive_model_t *jsdk_drive_model_find(
        const char *firmware_version,
        uint32_t product_code);

/** Lookup by table name, e.g. "ISVD90RC-v8.1.50". */
const jsdk_drive_model_t *jsdk_drive_model_by_name(const char *name);

/** Number of built-in models (for diagnostics). */
unsigned int jsdk_drive_model_count(void);
const jsdk_drive_model_t *jsdk_drive_model_at(unsigned int index);

/** CSP soft-invert recommended for this model (0/1). Unknown → 0. */
int jsdk_drive_model_csp_sw_invert(const jsdk_drive_model_t *model);

#ifdef __cplusplus
}
#endif

#endif /* JSDK_DRIVE_MODEL_H */
