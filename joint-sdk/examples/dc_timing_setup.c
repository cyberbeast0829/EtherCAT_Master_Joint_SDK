/*****************************************************************************
 *
 *  DC 时序单独配置（PREOP / configure 用），与 CSP/CSV/CST 运动指令分离。
 *
 *  相同 DC（SHARED）:
 *    ./build/dc_timing_setup --shared \
 *        --period 1000000 --sync0-cycle 1000000 \
 *        --sync0-shift 250000 --wait-ms 100
 *
 *  只改 task 周期（其它项保留）:
 *    ./build/dc_timing_setup --shared --period 1000000
 *    ./build/dc_timing_setup --period 2000000
 *
 *  不同 DC（PER_JOINT）— 每次一条固件，upsert，其它 [fw] 保留:
 *    ./build/dc_timing_setup --per-joint --fw 8.1.50 \
 *        --sync0-cycle 1000000 --sync0-shift 250000 --wait-ms 100
 *    ./build/dc_timing_setup --per-joint --fw 8.1.60 \
 *        --sync0-cycle 1000000 --sync0-shift 0 --wait-ms 100
 *    # 新电机只需再跑一次，例如:
 *    ./build/dc_timing_setup --per-joint --fw 8.2.71 \
 *        --sync0-cycle 1000000 --sync0-shift 250000 --wait-ms 100
 *
 *  查看: ./build/dc_timing_setup --show | --list
 *  配置文件: config/dc_timing.conf（或 $JSDK_DC_TIMING_CONF）
 *  新 fw 写进 conf 即可用于 DC（无需改 drive_model.c）；可选把打印的
 *  C 片段粘进 drive_model.c（csp_sw_invert 等）。
 *
 ****************************************************************************/

#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include <joint_sdk/dc_timing_conf.h>
#include <joint_sdk/drive_model.h>
#include <joint_sdk/joint_sdk.h>

#define MAX_FW 16
#define FW_LEN 32

typedef struct {
    char fw[FW_LEN];
    int use_dc;
    uint32_t preferred_period_ns;
    uint32_t sync0_cycle_ns;
    int32_t sync0_shift_ns;
    uint32_t wait_ms;
    int used;
} fw_entry_t;

typedef struct {
    int mode; /* JSDK_DC_TIMING_* ; -1 = unknown */
    int use_dc;
    uint32_t period_ns;
    uint32_t sync0_cycle_ns;
    int32_t sync0_shift_ns;
    uint32_t wait_ms;
    int has_sync0;
    int has_shift;
    int has_wait;
    int has_use_dc;
    fw_entry_t ents[MAX_FW];
    int n_ents;
} conf_state_t;

static int ensure_dir(const char *path)
{
    char tmp[256];
    char *slash;
    size_t n;

    n = strlen(path);
    if (n >= sizeof(tmp))
        return -1;
    memcpy(tmp, path, n + 1);
    slash = strrchr(tmp, '/');
    if (!slash)
        return 0;
    *slash = '\0';
    if (!tmp[0])
        return 0;
    if (mkdir(tmp, 0755) == 0 || errno == EEXIST)
        return 0;
    return -1;
}

static void conf_state_init(conf_state_t *st)
{
    memset(st, 0, sizeof(*st));
    st->mode = -1;
    st->use_dc = 1;
    st->period_ns = 1000000u;
    st->sync0_cycle_ns = 1000000u;
    st->sync0_shift_ns = 250000;
    st->wait_ms = 100;
}

static void load_conf_state(const char *text, conf_state_t *st)
{
    const char *p = text;
    fw_entry_t *cur = NULL;

    conf_state_init(st);

    while (p && *p) {
        char line[256];
        char *nl = strchr(p, '\n');
        size_t len;

        if (nl) {
            len = (size_t)(nl - p);
            if (len >= sizeof(line))
                len = sizeof(line) - 1;
            memcpy(line, p, len);
            line[len] = '\0';
            p = nl + 1;
        } else {
            strncpy(line, p, sizeof(line) - 1);
            line[sizeof(line) - 1] = '\0';
            p = NULL;
        }
        if (!strncmp(line, "mode=", 5)) {
            if (!strcmp(line + 5, "shared") || !strcmp(line + 5, "0"))
                st->mode = JSDK_DC_TIMING_SHARED;
            else if (!strcmp(line + 5, "per_joint") ||
                    !strcmp(line + 5, "1"))
                st->mode = JSDK_DC_TIMING_PER_JOINT;
            continue;
        }
        if (line[0] == '[') {
            char *end = strchr(line, ']');
            if (!end || st->n_ents >= MAX_FW) {
                cur = NULL;
                continue;
            }
            *end = '\0';
            cur = &st->ents[st->n_ents++];
            memset(cur, 0, sizeof(*cur));
            strncpy(cur->fw, line + 1, FW_LEN - 1);
            cur->use_dc = 1;
            cur->used = 1;
            if (st->mode < 0)
                st->mode = JSDK_DC_TIMING_PER_JOINT;
            continue;
        }
        if (!cur) {
            if (!strncmp(line, "period_ns=", 10))
                st->period_ns = (uint32_t)strtoul(line + 10, NULL, 0);
            else if (!strncmp(line, "sync0_cycle_ns=", 15)) {
                st->sync0_cycle_ns =
                        (uint32_t)strtoul(line + 15, NULL, 0);
                st->has_sync0 = 1;
            } else if (!strncmp(line, "sync0_shift_ns=", 15)) {
                st->sync0_shift_ns =
                        (int32_t)strtol(line + 15, NULL, 0);
                st->has_shift = 1;
            } else if (!strncmp(line, "wait_ms=", 8)) {
                st->wait_ms = (uint32_t)strtoul(line + 8, NULL, 0);
                st->has_wait = 1;
            } else if (!strncmp(line, "use_dc=", 7)) {
                st->use_dc = (int)strtol(line + 7, NULL, 0);
                st->has_use_dc = 1;
            }
            continue;
        }
        if (!strncmp(line, "use_dc=", 7))
            cur->use_dc = (int)strtol(line + 7, NULL, 0);
        else if (!strncmp(line, "preferred_period_ns=", 20))
            cur->preferred_period_ns =
                    (uint32_t)strtoul(line + 20, NULL, 0);
        else if (!strncmp(line, "sync0_cycle_ns=", 15))
            cur->sync0_cycle_ns =
                    (uint32_t)strtoul(line + 15, NULL, 0);
        else if (!strncmp(line, "sync0_shift_ns=", 15))
            cur->sync0_shift_ns =
                    (int32_t)strtol(line + 15, NULL, 0);
        else if (!strncmp(line, "wait_ms=", 8))
            cur->wait_ms = (uint32_t)strtoul(line + 8, NULL, 0);
    }
    if (st->mode < 0)
        st->mode = st->n_ents ? JSDK_DC_TIMING_PER_JOINT
                              : JSDK_DC_TIMING_SHARED;
}

static int read_conf_file(const char *path, conf_state_t *st)
{
    FILE *fp;
    char buf[4096];
    size_t nr;

    conf_state_init(st);
    fp = fopen(path, "r");
    if (!fp)
        return 0;
    nr = fread(buf, 1, sizeof(buf) - 1, fp);
    buf[nr] = '\0';
    fclose(fp);
    if (nr)
        load_conf_state(buf, st);
    return 1;
}

static int write_shared(const char *path, uint32_t period, uint32_t sync0,
        int32_t shift, uint32_t wait_ms, int use_dc)
{
    FILE *fp;

    if (ensure_dir(path)) {
        fprintf(stderr, "mkdir for %s failed: %s\n", path, strerror(errno));
        return 1;
    }
    fp = fopen(path, "w");
    if (!fp) {
        fprintf(stderr, "open %s: %s\n", path, strerror(errno));
        return 1;
    }
    fprintf(fp,
            "# joint-sdk DC timing (SHARED) — PREOP configure\n"
            "mode=shared\n"
            "use_dc=%d\n"
            "period_ns=%u\n"
            "sync0_cycle_ns=%u\n"
            "sync0_shift_ns=%d\n"
            "wait_ms=%u\n",
            use_dc, period, sync0, (int)shift, wait_ms);
    fclose(fp);
    printf("wrote SHARED DC → %s\n", path);
    printf("  period=%u sync0=%u shift=%d wait_ms=%u use_dc=%d\n",
           period, sync0, (int)shift, wait_ms, use_dc);
    return 0;
}

static int write_per_joint_file(const char *path, const conf_state_t *st)
{
    FILE *fp;
    int i;

    if (ensure_dir(path)) {
        fprintf(stderr, "mkdir for %s failed: %s\n", path, strerror(errno));
        return 1;
    }
    fp = fopen(path, "w");
    if (!fp) {
        fprintf(stderr, "open %s: %s\n", path, strerror(errno));
        return 1;
    }
    fprintf(fp,
            "# joint-sdk DC timing (PER_JOINT) — PREOP configure\n"
            "mode=per_joint\n"
            "use_dc=%d\n"
            "period_ns=%u\n",
            st->use_dc, st->period_ns);
    for (i = 0; i < st->n_ents; i++) {
        const fw_entry_t *e = &st->ents[i];

        if (!e->used)
            continue;
        fprintf(fp,
                "\n[%s]\n"
                "use_dc=%d\n"
                "preferred_period_ns=%u\n"
                "sync0_cycle_ns=%u\n"
                "sync0_shift_ns=%d\n"
                "wait_ms=%u\n",
                e->fw, e->use_dc,
                e->preferred_period_ns ? e->preferred_period_ns
                                       : st->period_ns,
                e->sync0_cycle_ns,
                (int)e->sync0_shift_ns,
                e->wait_ms);
    }
    fclose(fp);
    return 0;
}

/** Only change global period_ns; keep mode / Sync0 / fw sections. */
static int patch_period(const char *path, uint32_t period)
{
    conf_state_t st;
    int existed;

    existed = read_conf_file(path, &st);
    st.period_ns = period;

    if (st.mode == JSDK_DC_TIMING_PER_JOINT || st.n_ents > 0) {
        if (write_per_joint_file(path, &st))
            return 1;
        printf("%s period_ns=%u → %s\n",
               existed ? "updated" : "wrote", period, path);
        printf("  (kept mode=per_joint and %d fw section(s))\n", st.n_ents);
        return 0;
    }

    if (!st.has_sync0)
        st.sync0_cycle_ns = period;
    if (write_shared(path, st.period_ns, st.sync0_cycle_ns,
            st.sync0_shift_ns, st.wait_ms, st.use_dc))
        return 1;
    if (existed && !st.has_sync0)
        printf("  (sync0_cycle_ns followed period; other fields kept)\n");
    else if (existed)
        printf("  (only period_ns changed; sync0/shift/wait kept)\n");
    return 0;
}

static int upsert_per_joint_fw(const char *path, const char *fw,
        uint32_t period, int period_set, uint32_t sync0, int32_t shift,
        uint32_t wait_ms, int use_dc)
{
    conf_state_t st;
    int i;
    int found = 0;

    if (!fw || !fw[0]) {
        fprintf(stderr, "--per-joint needs --fw\n");
        return 1;
    }

    read_conf_file(path, &st);
    st.mode = JSDK_DC_TIMING_PER_JOINT;
    if (period_set)
        st.period_ns = period;

    for (i = 0; i < st.n_ents; i++) {
        if (!strcmp(st.ents[i].fw, fw)) {
            st.ents[i].use_dc = use_dc;
            st.ents[i].preferred_period_ns = st.period_ns;
            st.ents[i].sync0_cycle_ns = sync0;
            st.ents[i].sync0_shift_ns = shift;
            st.ents[i].wait_ms = wait_ms;
            st.ents[i].used = 1;
            found = 1;
            break;
        }
    }
    if (!found) {
        if (st.n_ents >= MAX_FW) {
            fprintf(stderr, "too many fw sections (max %d)\n", MAX_FW);
            return 1;
        }
        memset(&st.ents[st.n_ents], 0, sizeof(st.ents[0]));
        strncpy(st.ents[st.n_ents].fw, fw, FW_LEN - 1);
        st.ents[st.n_ents].use_dc = use_dc;
        st.ents[st.n_ents].preferred_period_ns = st.period_ns;
        st.ents[st.n_ents].sync0_cycle_ns = sync0;
        st.ents[st.n_ents].sync0_shift_ns = shift;
        st.ents[st.n_ents].wait_ms = wait_ms;
        st.ents[st.n_ents].used = 1;
        st.n_ents++;
    }

    if (write_per_joint_file(path, &st))
        return 1;
    printf("%s PER_JOINT [%s] → %s\n",
           found ? "updated" : "added", fw, path);
    printf("  period=%u sync0=%u shift=%d wait_ms=%u use_dc=%d "
           "(%d fw section(s) total)\n",
           st.period_ns, sync0, (int)shift, wait_ms, use_dc, st.n_ents);
    {
        unsigned int mi;
        int in_static = 0;

        for (mi = 0; mi < jsdk_drive_model_count(); mi++) {
            const jsdk_drive_model_t *m = jsdk_drive_model_at(mi);

            if (m && m->fw_match && strstr(fw, m->fw_match)) {
                in_static = 1;
                break;
            }
        }
        if (!in_static) {
            printf("\n  note: [%s] not in drive_model.c g_models[] —\n"
                   "  DC/runtime OK via conf (no rebuild). Optional paste "
                   "into src/drive_model.c:\n\n", fw);
            printf("    {\n"
                   "        \"ISVD90RC-v%s\",\n"
                   "        0x000C0B00u, 0x00080117u, \"%s\",\n"
                   "        %d, 0x%04xu,\n"
                   "        %d, %u, %uu, %uu,\n"
                   "        0,  /* csp_sw_invert: set 1 if needed */\n"
                   "        \"DC from dc_timing_setup\",\n"
                   "    },\n",
                   fw, fw, use_dc, use_dc ? 0x0300u : 0u,
                   (int)shift, wait_ms, st.period_ns, sync0);
        } else {
            printf("  (also present in drive_model.c — conf overrides "
                   "Sync0/shift/wait in PER_JOINT)\n");
        }
    }
    return 0;
}

static int show_conf(void)
{
    const char *path = jsdk_dc_timing_conf_path();
    FILE *fp;
    char line[256];

    printf("conf path: %s\n", path);
    printf("code default mode: %s (%d)  [JSDK_DC_TIMING_MODE_DEFAULT "
           "in drive_model.h]\n",
           JSDK_DC_TIMING_MODE_DEFAULT == JSDK_DC_TIMING_PER_JOINT
                   ? "PER_JOINT" : "SHARED",
           JSDK_DC_TIMING_MODE_DEFAULT);
    fp = fopen(path, "r");
    if (!fp) {
        printf("(no conf file — motion apps use drive_model.c + "
               "JSDK_DC_TIMING_MODE_DEFAULT)\n");
        return 0;
    }
    printf("----- %s -----\n", path);
    while (fgets(line, sizeof(line), fp))
        fputs(line, stdout);
    fclose(fp);
    return 0;
}

static int list_models(void)
{
    unsigned int i;
    conf_state_t st;
    int existed;

    printf("Built-in drive_model.c (%u):\n", jsdk_drive_model_count());
    for (i = 0; i < jsdk_drive_model_count(); i++) {
        const jsdk_drive_model_t *m = jsdk_drive_model_at(i);

        printf("  %-12s use_dc=%d sync0=%u shift=%d wait=%u invert=%d\n",
               m->fw_match ? m->fw_match : "?",
               m->use_dc, m->sync0_cycle_ns, (int)m->sync0_shift_ns,
               m->wait_before_safeop_ms, m->csp_sw_invert);
    }

    existed = read_conf_file(jsdk_dc_timing_conf_path(), &st);
    printf("\nConf %s%s:\n", jsdk_dc_timing_conf_path(),
           existed ? "" : " (missing)");
    if (!existed) {
        printf("  (none — add with --per-joint --fw ...)\n");
        return 0;
    }
    printf("  mode=%s period_ns=%u\n",
           st.mode == JSDK_DC_TIMING_PER_JOINT ? "per_joint" : "shared",
           st.period_ns);
    for (i = 0; (int)i < st.n_ents; i++) {
        const fw_entry_t *e = &st.ents[i];
        unsigned int mi;
        int in_static = 0;

        for (mi = 0; mi < jsdk_drive_model_count(); mi++) {
            const jsdk_drive_model_t *m = jsdk_drive_model_at(mi);

            if (m && m->fw_match && strstr(e->fw, m->fw_match)) {
                in_static = 1;
                break;
            }
        }
        printf("  [%-8s] sync0=%u shift=%d wait=%u  %s\n",
               e->fw, e->sync0_cycle_ns, (int)e->sync0_shift_ns,
               e->wait_ms,
               in_static ? "(in drive_model.c)" : "(conf-only OK for DC)");
    }
    return 0;
}

int main(int argc, char **argv)
{
    int shared = 0;
    int per_joint = 0;
    int show = 0;
    int list = 0;
    const char *fw = NULL;
    uint32_t period = 1000000u;
    uint32_t sync0 = 1000000u;
    int32_t shift = 250000;
    uint32_t wait_ms = 100;
    int use_dc = 1;
    int period_set = 0;
    int sync0_set = 0;
    int shift_set = 0;
    int wait_set = 0;
    int use_dc_set = 0;
    int i;

    for (i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--shared")) {
            shared = 1;
        } else if (!strcmp(argv[i], "--per-joint")) {
            per_joint = 1;
        } else if (!strcmp(argv[i], "--show")) {
            show = 1;
        } else if (!strcmp(argv[i], "--list")) {
            list = 1;
        } else if (!strcmp(argv[i], "--fw") && i + 1 < argc) {
            fw = argv[++i];
        } else if (!strcmp(argv[i], "--period") && i + 1 < argc) {
            period = (uint32_t)strtoul(argv[++i], NULL, 0);
            period_set = 1;
        } else if (!strcmp(argv[i], "--sync0-cycle") && i + 1 < argc) {
            sync0 = (uint32_t)strtoul(argv[++i], NULL, 0);
            sync0_set = 1;
        } else if (!strcmp(argv[i], "--sync0-shift") && i + 1 < argc) {
            shift = (int32_t)strtol(argv[++i], NULL, 0);
            shift_set = 1;
        } else if (!strcmp(argv[i], "--wait-ms") && i + 1 < argc) {
            wait_ms = (uint32_t)strtoul(argv[++i], NULL, 0);
            wait_set = 1;
        } else if (!strcmp(argv[i], "--use-dc") && i + 1 < argc) {
            use_dc = (int)strtol(argv[++i], NULL, 0);
            use_dc_set = 1;
        } else if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) {
            fprintf(stderr,
                    "Usage:\n"
                    "  %s --show | --list\n"
                    "  %s --period N\n"
                    "      (only update task period_ns; keep Sync0/fw)\n"
                    "  %s --shared --period N --sync0-cycle N "
                    "--sync0-shift N --wait-ms N\n"
                    "  %s --per-joint --fw X.Y.Z --sync0-cycle N "
                    "--sync0-shift N --wait-ms N\n"
                    "      (one motor/fw per call; upserts that [fw] only)\n"
                    "Writes %s. New fw in conf is enough for DC at runtime;\n"
                    "paste printed C into drive_model.c only if needed.\n",
                    argv[0], argv[0], argv[0], argv[0],
                    jsdk_dc_timing_conf_path());
            return 0;
        } else {
            fprintf(stderr, "unknown arg: %s\n", argv[i]);
            return 1;
        }
    }

    if (list)
        return list_models();
    if (show)
        return show_conf();

    if (shared && per_joint) {
        fprintf(stderr, "use only one of --shared / --per-joint\n");
        return 1;
    }

    /* --period alone (optional --shared) — do not wipe Sync0 / fw. */
    if (period_set && !sync0_set && !shift_set && !wait_set &&
            !use_dc_set && !fw && !per_joint) {
        return patch_period(jsdk_dc_timing_conf_path(), period);
    }

    if (!shared && !per_joint)
        return show_conf();

    if (shared) {
        conf_state_t st;

        read_conf_file(jsdk_dc_timing_conf_path(), &st);
        if (period_set)
            st.period_ns = period;
        if (sync0_set)
            st.sync0_cycle_ns = sync0;
        else if (period_set && !st.has_sync0)
            st.sync0_cycle_ns = st.period_ns;
        if (shift_set)
            st.sync0_shift_ns = shift;
        if (wait_set)
            st.wait_ms = wait_ms;
        if (use_dc_set)
            st.use_dc = use_dc;
        return write_shared(jsdk_dc_timing_conf_path(), st.period_ns,
                st.sync0_cycle_ns, st.sync0_shift_ns, st.wait_ms,
                st.use_dc);
    }

    if (!fw) {
        fprintf(stderr, "--per-joint needs --fw <ver>\n");
        return 1;
    }
    if (!sync0_set && !shift_set && !wait_set) {
        if (period_set)
            return patch_period(jsdk_dc_timing_conf_path(), period);
        fprintf(stderr, "--per-joint --fw needs --sync0-cycle / "
                "--sync0-shift / --wait-ms\n");
        return 1;
    }

    return upsert_per_joint_fw(jsdk_dc_timing_conf_path(), fw, period,
            period_set, sync0, shift, wait_ms, use_dc);
}
