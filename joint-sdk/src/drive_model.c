#include <stdio.h>
#include <string.h>

#include <joint_sdk/dc_timing_conf.h>
#include <joint_sdk/drive_model.h>

#define CB_VID 0x000C0B00u
#define CB_PID 0x00080117u
#define CONF_MODEL_SLOTS 16

/*
 * Motor-app iteration tags (NOT CoE 0x100A / EtherCAT stack version).
 * CoE 0x100A on this board often reads "0.6.0" — ignore for model select.
 *
 * DC timing (bench / ESC readout):
 *   Sync Unit Cycle = 1 ms, Sync0 = 1 ms, Shift = 250 µs,
 *   WaitBeforeSAFEOP = 100 ms, AssignActivate = 0x0300.
 *
 * New motors: upsert config/dc_timing.conf via dc_timing_setup --per-joint
 * --fw ... (one call per fw). Runtime also accepts conf-only rows when not
 * listed here; add a static row when you need csp_sw_invert / a code default.
 */
static const jsdk_drive_model_t g_models[] = {
    {
        "ISVD90RC-v8.1.44",
        CB_VID,
        CB_PID,
        "8.1.44",
        1,
        0x0300u,
        250000,     /* Sync0 shift = 250 µs */
        100,        /* WaitBeforeSAFEOP */
        1000000u,   /* Sync Unit Cycle = 1 ms */
        1000000u,   /* Sync0 Cycle = 1 ms */
        0,          /* csp_sw_invert */
        "DC: task 1 ms, Sync0 1 ms, shift 250 us, wait 100 ms",
    },
    {
        "ISVD90RC-v8.1.50",
        CB_VID,
        CB_PID,
        "8.1.50",
        1,
        0x0300u,
        250000,     /* Sync0 shift = 250 µs */
        100,        /* WaitBeforeSAFEOP */
        1000000u,   /* Sync Unit Cycle = 1 ms */
        1000000u,   /* Sync0 Cycle = 1 ms */
        1,          /* csp_sw_invert (unused by dual CLI invert) */
        "DC: task 1 ms, Sync0 1 ms, shift 250 us, wait 100 ms",
    },
    {
        "ISVD90RC-v8.1.60",
        CB_VID,
        CB_PID,
        "8.1.60",
        1,
        0x0300u,
        250000,     /* Sync0 shift = 250 µs */
        100,        /* WaitBeforeSAFEOP */
        1000000u,   /* Sync Unit Cycle = 1 ms */
        1000000u,   /* Sync0 Cycle = 1 ms */
        0,          /* csp_sw_invert */
        "DC: task 1 ms, Sync0 1 ms, shift 250 us, wait 100 ms",
    },
};

static const jsdk_drive_model_t g_default = {
    "ISVD90RC-default-freerun",
    CB_VID,
    CB_PID,
    "",
    0,
    0x0000u,
    0,
    100,
    1000000u,
    0,
    0,          /* csp_sw_invert */
    "Unknown FW: FreeRun only (PDO/CiA402 bring-up)",
};

/* Runtime models synthesized from dc_timing.conf [fw] sections. */
static jsdk_drive_model_t g_conf_models[CONF_MODEL_SLOTS];
static char g_conf_names[CONF_MODEL_SLOTS][48];
static char g_conf_fws[CONF_MODEL_SLOTS][32];
static char g_conf_notes[CONF_MODEL_SLOTS][96];
static int g_conf_valid[CONF_MODEL_SLOTS];

static const jsdk_drive_model_t *model_from_conf(const char *firmware_version)
{
    int i;
    int free_slot = -1;

    if (!firmware_version || !firmware_version[0])
        return NULL;
    if (!jsdk_dc_timing_conf_has_fw(firmware_version))
        return NULL;

    for (i = 0; i < CONF_MODEL_SLOTS; i++) {
        if (g_conf_valid[i] &&
                (strstr(firmware_version, g_conf_fws[i]) ||
                 !strcmp(firmware_version, g_conf_fws[i])))
            return &g_conf_models[i];
        if (!g_conf_valid[i] && free_slot < 0)
            free_slot = i;
    }
    if (free_slot < 0)
        return NULL;

    if (!jsdk_dc_timing_conf_make_model(firmware_version,
            &g_conf_models[free_slot],
            g_conf_names[free_slot], sizeof(g_conf_names[free_slot]),
            g_conf_fws[free_slot], sizeof(g_conf_fws[free_slot]),
            g_conf_notes[free_slot], sizeof(g_conf_notes[free_slot])))
        return NULL;

    g_conf_valid[free_slot] = 1;
    return &g_conf_models[free_slot];
}

const jsdk_drive_model_t *jsdk_drive_model_default(void)
{
    return &g_default;
}

unsigned int jsdk_drive_model_count(void)
{
    return (unsigned int)(sizeof(g_models) / sizeof(g_models[0]));
}

const jsdk_drive_model_t *jsdk_drive_model_at(unsigned int index)
{
    if (index >= jsdk_drive_model_count())
        return NULL;
    return &g_models[index];
}

const jsdk_drive_model_t *jsdk_drive_model_by_name(const char *name)
{
    unsigned int i;
    const jsdk_drive_model_t *from_conf;

    if (!name || !name[0])
        return NULL;
    for (i = 0; i < jsdk_drive_model_count(); i++) {
        if (!strcmp(g_models[i].name, name))
            return &g_models[i];
    }
    /* Accept "ISVD90RC-vX.Y.Z" / raw "X.Y.Z" via conf. */
    from_conf = model_from_conf(name);
    if (from_conf)
        return from_conf;
    if (!strncmp(name, "ISVD90RC-v", 10))
        return model_from_conf(name + 10);
    return NULL;
}

const jsdk_drive_model_t *jsdk_drive_model_find(
        const char *firmware_version,
        uint32_t product_code)
{
    unsigned int i;
    const jsdk_drive_model_t *from_conf;

    if (!firmware_version || !firmware_version[0])
        return jsdk_drive_model_default();

    for (i = 0; i < jsdk_drive_model_count(); i++) {
        const jsdk_drive_model_t *m = &g_models[i];

        if (product_code && m->product_code &&
                product_code != m->product_code)
            continue;
        if (!m->fw_match || !m->fw_match[0])
            continue;
        if (strstr(firmware_version, m->fw_match))
            return m;
    }

    /* New motor: conf [fw] alone is enough for DC (no rebuild). */
    from_conf = model_from_conf(firmware_version);
    if (from_conf) {
        if (product_code && from_conf->product_code &&
                product_code != from_conf->product_code)
            return jsdk_drive_model_default();
        return from_conf;
    }

    return jsdk_drive_model_default();
}

int jsdk_drive_model_csp_sw_invert(const jsdk_drive_model_t *model)
{
    if (!model)
        return 0;
    return model->csp_sw_invert ? 1 : 0;
}
