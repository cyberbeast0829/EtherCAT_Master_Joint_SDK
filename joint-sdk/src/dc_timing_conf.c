#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <joint_sdk/dc_timing_conf.h>
#include <joint_sdk/drive_model.h>
#include <joint_sdk/joint_sdk.h>

#define JSDK_DC_FW_SLOTS 16

typedef struct {
    char fw[32];
    int use_dc;
    int32_t sync0_shift_ns;
    uint32_t wait_ms;
    uint32_t preferred_period_ns;
    uint32_t sync0_cycle_ns;
    int valid;
} jsdk_dc_fw_slot_t;

typedef struct {
    int loaded;
    int mode; /* JSDK_DC_TIMING_* */
    int has_shared;
    int set_period;
    int set_sync0_cycle;
    int set_sync0_shift;
    int set_wait_ms;
    int set_use_dc;
    uint32_t period_ns;
    uint32_t sync0_cycle_ns;
    int32_t sync0_shift_ns;
    uint32_t wait_ms;
    int use_dc;
    jsdk_dc_fw_slot_t slots[JSDK_DC_FW_SLOTS];
} jsdk_dc_timing_file_t;

static jsdk_dc_timing_file_t g_dc_file;

const char *jsdk_dc_timing_conf_path(void)
{
    const char *e = getenv("JSDK_DC_TIMING_CONF");

    if (e && e[0])
        return e;
    return JSDK_DC_TIMING_CONF_DEFAULT;
}

static char *trim_inplace(char *s)
{
    char *e;
    char *start;

    while (*s && isspace((unsigned char)*s))
        s++;
    start = s;
    if (!*start)
        return start;
    e = start + strlen(start);
    while (e > start && isspace((unsigned char)e[-1]))
        *--e = '\0';
    return start;
}

static jsdk_dc_fw_slot_t *slot_for_fw(const char *fw, int create)
{
    unsigned int i;
    jsdk_dc_fw_slot_t *empty = NULL;

    if (!fw || !fw[0])
        return NULL;
    for (i = 0; i < JSDK_DC_FW_SLOTS; i++) {
        if (g_dc_file.slots[i].valid &&
                !strcmp(g_dc_file.slots[i].fw, fw))
            return &g_dc_file.slots[i];
        if (!g_dc_file.slots[i].valid && !empty)
            empty = &g_dc_file.slots[i];
    }
    if (!create || !empty)
        return NULL;
    memset(empty, 0, sizeof(*empty));
    strncpy(empty->fw, fw, sizeof(empty->fw) - 1);
    empty->use_dc = 1;
    empty->sync0_shift_ns = 0;
    empty->valid = 1;
    return empty;
}

static int load_file(void)
{
    FILE *fp;
    char line[256];
    char cur_fw[32];

    if (g_dc_file.loaded)
        return 0;

    memset(&g_dc_file, 0, sizeof(g_dc_file));
    g_dc_file.mode = JSDK_DC_TIMING_MODE_DEFAULT;
    g_dc_file.use_dc = 1;
    cur_fw[0] = '\0';

    fp = fopen(jsdk_dc_timing_conf_path(), "r");
    if (!fp) {
        g_dc_file.loaded = 1;
        return 0;
    }

    while (fgets(line, sizeof(line), fp)) {
        char *p = trim_inplace(line);
        char *eq;
        char key[64];
        char val[128];

        if (!p[0] || p[0] == '#')
            continue;
        if (p[0] == '[' && p[strlen(p) - 1] == ']') {
            size_t n = strlen(p) - 2;
            if (n >= sizeof(cur_fw))
                n = sizeof(cur_fw) - 1;
            memcpy(cur_fw, p + 1, n);
            cur_fw[n] = '\0';
            slot_for_fw(cur_fw, 1);
            continue;
        }
        eq = strchr(p, '=');
        if (!eq)
            continue;
        *eq = '\0';
        strncpy(key, trim_inplace(p), sizeof(key) - 1);
        key[sizeof(key) - 1] = '\0';
        strncpy(val, trim_inplace(eq + 1), sizeof(val) - 1);
        val[sizeof(val) - 1] = '\0';

        if (!strcmp(key, "mode")) {
            if (!strcmp(val, "shared") || !strcmp(val, "0"))
                g_dc_file.mode = JSDK_DC_TIMING_SHARED;
            else if (!strcmp(val, "per_joint") || !strcmp(val, "1"))
                g_dc_file.mode = JSDK_DC_TIMING_PER_JOINT;
        } else if (!cur_fw[0]) {
            g_dc_file.has_shared = 1;
            if (!strcmp(key, "period_ns")) {
                g_dc_file.period_ns = (uint32_t)strtoul(val, NULL, 0);
                g_dc_file.set_period = 1;
            } else if (!strcmp(key, "sync0_cycle_ns")) {
                g_dc_file.sync0_cycle_ns = (uint32_t)strtoul(val, NULL, 0);
                g_dc_file.set_sync0_cycle = 1;
            } else if (!strcmp(key, "sync0_shift_ns")) {
                g_dc_file.sync0_shift_ns = (int32_t)strtol(val, NULL, 0);
                g_dc_file.set_sync0_shift = 1;
            } else if (!strcmp(key, "wait_ms")) {
                g_dc_file.wait_ms = (uint32_t)strtoul(val, NULL, 0);
                g_dc_file.set_wait_ms = 1;
            } else if (!strcmp(key, "use_dc")) {
                g_dc_file.use_dc = (int)strtol(val, NULL, 0);
                g_dc_file.set_use_dc = 1;
            }
        } else {
            jsdk_dc_fw_slot_t *s = slot_for_fw(cur_fw, 1);
            if (!s)
                continue;
            if (!strcmp(key, "period_ns") ||
                    !strcmp(key, "preferred_period_ns"))
                s->preferred_period_ns = (uint32_t)strtoul(val, NULL, 0);
            else if (!strcmp(key, "sync0_cycle_ns"))
                s->sync0_cycle_ns = (uint32_t)strtoul(val, NULL, 0);
            else if (!strcmp(key, "sync0_shift_ns"))
                s->sync0_shift_ns = (int32_t)strtol(val, NULL, 0);
            else if (!strcmp(key, "wait_ms"))
                s->wait_ms = (uint32_t)strtoul(val, NULL, 0);
            else if (!strcmp(key, "use_dc"))
                s->use_dc = (int)strtol(val, NULL, 0);
        }
    }
    fclose(fp);
    g_dc_file.loaded = 1;
    return 0;
}

int jsdk_dc_timing_conf_apply(jsdk_context_config_t *cfg)
{
    if (!cfg)
        return -1;
    load_file();

    cfg->dc_timing_mode = g_dc_file.mode;
    if (g_dc_file.has_shared) {
        /*
         * Only overwrite fields present in the file. A PER_JOINT conf often
         * sets global period_ns/use_dc only — blindly writing unset
         * sync0_shift_ns=0 wiped drive_model's 250 µs and broke Sync0 (AL
         * status 0x001A / settle fail), especially on 8.1.44 single.
         */
        if (g_dc_file.set_period && g_dc_file.period_ns)
            cfg->period_ns = g_dc_file.period_ns;
        if (g_dc_file.set_sync0_cycle)
            cfg->sync0_cycle_ns = g_dc_file.sync0_cycle_ns;
        else if (g_dc_file.mode == JSDK_DC_TIMING_SHARED &&
                g_dc_file.set_period && g_dc_file.period_ns)
            cfg->sync0_cycle_ns = g_dc_file.period_ns;
        if (g_dc_file.set_sync0_shift)
            cfg->sync0_shift_ns = g_dc_file.sync0_shift_ns;
        if (g_dc_file.set_wait_ms && g_dc_file.wait_ms)
            cfg->wait_before_safeop_ms = g_dc_file.wait_ms;
        if (g_dc_file.set_use_dc)
            cfg->use_dc = g_dc_file.use_dc;
    }
    return 0;
}

int jsdk_dc_timing_conf_apply_fw(jsdk_context_config_t *cfg, const char *fw)
{
    int use_dc;
    int32_t shift;
    uint32_t wait_ms;
    uint32_t preferred;
    uint32_t sync0;

    if (!cfg)
        return -1;
    if (jsdk_dc_timing_conf_apply(cfg) < 0)
        return -1;
    if (!fw || !fw[0])
        return 0;

    use_dc = cfg->use_dc;
    shift = cfg->sync0_shift_ns;
    wait_ms = cfg->wait_before_safeop_ms;
    preferred = 0;
    sync0 = cfg->sync0_cycle_ns;

    if (!jsdk_dc_timing_conf_lookup_fw(fw, &use_dc, &shift, &wait_ms,
            &preferred, &sync0))
        return 0;

    cfg->use_dc = use_dc;
    cfg->auto_reference_clock = use_dc ? 1 : 0;
    cfg->sync0_shift_ns = shift;
    if (wait_ms)
        cfg->wait_before_safeop_ms = wait_ms;
    if (sync0)
        cfg->sync0_cycle_ns = sync0;
    /*
     * Task period stays global conf period_ns (file top). Do NOT apply
     * per-fw preferred_period_ns here — that field is Sync0 fallback only.
     */
    (void)preferred;
    return 1;
}

int jsdk_dc_timing_conf_lookup_fw(const char *fw,
        int *use_dc,
        int32_t *sync0_shift_ns,
        uint32_t *wait_ms,
        uint32_t *preferred_period_ns,
        uint32_t *sync0_cycle_ns)
{
    unsigned int i;

    load_file();
    if (!fw || !fw[0])
        return 0;
    for (i = 0; i < JSDK_DC_FW_SLOTS; i++) {
        const jsdk_dc_fw_slot_t *s = &g_dc_file.slots[i];

        if (!s->valid)
            continue;
        if (!strstr(fw, s->fw) && strcmp(fw, s->fw))
            continue;
        if (use_dc)
            *use_dc = s->use_dc;
        if (sync0_shift_ns)
            *sync0_shift_ns = s->sync0_shift_ns;
        if (wait_ms && s->wait_ms)
            *wait_ms = s->wait_ms;
        if (preferred_period_ns && s->preferred_period_ns)
            *preferred_period_ns = s->preferred_period_ns;
        if (sync0_cycle_ns && s->sync0_cycle_ns)
            *sync0_cycle_ns = s->sync0_cycle_ns;
        return 1;
    }
    return 0;
}

uint32_t jsdk_dc_timing_conf_period_ns(void)
{
    load_file();
    if (g_dc_file.set_period && g_dc_file.period_ns)
        return g_dc_file.period_ns;
    return 0;
}

int jsdk_dc_timing_conf_has_fw(const char *fw)
{
    return jsdk_dc_timing_conf_lookup_fw(fw, NULL, NULL, NULL, NULL, NULL);
}

int jsdk_dc_timing_conf_make_model(const char *fw,
        jsdk_drive_model_t *out,
        char *name_buf, size_t name_sz,
        char *fw_buf, size_t fw_sz,
        char *notes_buf, size_t notes_sz)
{
    unsigned int i;
    const jsdk_dc_fw_slot_t *s = NULL;
    uint32_t period;

    if (!fw || !fw[0] || !out || !name_buf || !fw_buf || name_sz < 8 ||
            fw_sz < 8)
        return 0;

    load_file();
    for (i = 0; i < JSDK_DC_FW_SLOTS; i++) {
        const jsdk_dc_fw_slot_t *cand = &g_dc_file.slots[i];

        if (!cand->valid)
            continue;
        if (!strstr(fw, cand->fw) && strcmp(fw, cand->fw))
            continue;
        s = cand;
        break;
    }
    if (!s)
        return 0;

    period = jsdk_dc_timing_conf_period_ns();
    if (!period)
        period = 1000000u;

    snprintf(name_buf, name_sz, "ISVD90RC-v%s", s->fw);
    strncpy(fw_buf, s->fw, fw_sz - 1);
    fw_buf[fw_sz - 1] = '\0';
    if (notes_buf && notes_sz) {
        snprintf(notes_buf, notes_sz,
                "from %s [%s] (conf-only; add drive_model.c for "
                "csp_sw_invert)",
                jsdk_dc_timing_conf_path(), s->fw);
    }

    memset(out, 0, sizeof(*out));
    out->name = name_buf;
    out->vendor_id = 0x000C0B00u;
    out->product_code = 0x00080117u;
    out->fw_match = fw_buf;
    out->use_dc = s->use_dc;
    out->dc_assign_activate = s->use_dc ? 0x0300u : 0;
    out->sync0_shift_ns = s->sync0_shift_ns;
    out->wait_before_safeop_ms = s->wait_ms ? s->wait_ms : 100;
    out->preferred_period_ns = s->preferred_period_ns
            ? s->preferred_period_ns : period;
    out->sync0_cycle_ns = s->sync0_cycle_ns
            ? s->sync0_cycle_ns : out->preferred_period_ns;
    out->csp_sw_invert = 0;
    out->notes = (notes_buf && notes_sz) ? notes_buf : "";
    return 1;
}
