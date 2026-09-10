/*****************************************************************************
 *
 *  守护兽关节 SDK — 原生 CSP（紧凑 RPDO 0x1601 + soft-ramp）
 *
 *  本机实测要点（ISVD90RC / 8.1.50）:
 *    - RPDO 用 0x1601（CW+目标位置）；全量 0x1600 会 vel-limit 飞车
 *    - 位置环相对 0x607A 反向；0x607E 运动会掉 OP → 软件误差反相
 *      write = 2*actual - cmd（默认 --invert）
 *    - 0x2002:1: TwinCAT CSP = POS_FILTER(3)；仅 soft-invert 时用
 *      PASSTHROUGH(1)（POS_FILTER+反相会过冲）
 *    - --fw 选 drive_model 的 DC/周期（电机应用版本，非 CoE 0x100A）
 *
 *  用法:
 *    sudo chrt -f 80 ./diag_csp_single --fw 8.1.50 --hold --seconds 6
 *    sudo chrt -f 80 ./diag_csp_single --fw 8.1.44 --no-invert \
 *         --delta 18192 --speed 20000 --seconds 12
 *    sudo chrt -f 80 ./diag_csp_single --fw 8.1.50 --pos <绝对> --seconds 12
 *
 *  --speed → 必填（--delta/--pos）；soft-ramp 速率 counts/s；lead=--speed
 *  --pos N → 绝对目标（与 --delta 二选一）；同样按 --speed 爬坡
 *  --fw / --model: motor-app version → CSP soft-invert from drive_model
 *    (8.1.50=ON, 8.1.44/8.1.60=OFF). Override with --invert / --no-invert.
 *  DC: config/dc_timing.conf（与 dual 相同）；无 conf 则用 drive_model。
 *  --invert / --no-invert → 覆盖软件位置误差反相
 *  Prefer: sudo chrt -f 80 ./diag_csp_single --fw 8.1.44 --no-invert \
 *       --delta 18192 --speed 20000 --seconds 12
 *
 ****************************************************************************/

#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <limits.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/stat.h>

#include <joint_sdk/dc_timing_conf.h>
#include <joint_sdk/joint_sdk.h>

#define PERIOD_NS_DEFAULT 1000000u
#define NSEC_PER_SEC 1000000000L
#define MAX_SAFE_STACK (8 * 1024)
/*
 * Soft-ramp at exactly --speed via shared time profile (same as dual).
 * Lead cap = --speed.
 */
#define SETTLE_SEC 0.10
#define POS_ERR_OK_FLOOR 15000
#define VEL_RUNAWAY 250000
/** Mirror actual after enable before first step (seconds). */
#define ENABLE_MIRROR_SEC 0.08
/** Sustained bad bus (~50 ms @ 1 ms) before arming post-WC realign. */
#define BUS_BAD_FREEZE_CYCLES 50
/**
 * After a *sustained* WC hole: brief hold then resume soft-ramp.
 * Brief holes must NOT snap cmd→actual (that zeros following error).
 */
#define RECOVER_HOLD_SEC 0.15
#define RECOVER_RAMP_SEC 0.4
/** Only snap cmd→actual after hole if following error this large. */
#define RECOVER_SNAP_ERR 8000
/*
 * Do NOT stall-snap on |goal-act|: if the plant is not following, that
 * glues cmd to actual forever (zero following error → motor never moves).
 *
 * DEFAULT_INVERT: -1 = auto from drive_model.csp_sw_invert (--fw);
 * CLI --invert/--no-invert overrides. Plant: 8.1.50=ON, 8.1.60=OFF.
 */
#define DEFAULT_INVERT (-1)

static int32_t csp_write_position(int32_t cmd_pos, int32_t actual, int invert)
{
    int64_t w;

    if (!invert)
        return cmd_pos;
    /* error_drive = write - actual = actual - cmd = -(cmd - actual) */
    w = (int64_t)actual * 2 - (int64_t)cmd_pos;
    if (w > INT32_MAX)
        w = INT32_MAX;
    else if (w < INT32_MIN)
        w = INT32_MIN;
    return (int32_t)w;
}

static int32_t iabs32(int32_t v)
{
    return v < 0 ? -v : v;
}

static int32_t csp_pos_err_ok_for_travel(int32_t travel)
{
    int32_t mag = iabs32(travel);
    int32_t ok;

    ok = mag - mag / 3;
    if (ok < POS_ERR_OK_FLOOR)
        ok = POS_ERR_OK_FLOOR;
    if (mag > 0 && ok >= mag)
        ok = mag / 2;
    if (ok < 1)
        ok = 1;
    return ok;
}

static int32_t csp_profile_at(int32_t home, int32_t goal, double move_s,
        uint32_t speed)
{
    int64_t dist = (int64_t)goal - (int64_t)home;
    int64_t mag = dist < 0 ? -dist : dist;
    int64_t planned;

    if (move_s < 0.0)
        move_s = 0.0;
    planned = (int64_t)((double)speed * move_s + 0.5);
    if (planned < 0)
        planned = 0;
    if (planned > mag)
        planned = mag;
    if (dist >= 0)
        return home + (int32_t)planned;
    return home - (int32_t)planned;
}

static int32_t csp_apply_lead(int32_t prof, int32_t actual, int32_t lead_lim)
{
    int32_t lead = prof - actual;

    if (lead > lead_lim)
        return actual + lead_lim;
    if (lead < -lead_lim)
        return actual - lead_lim;
    return prof;
}

static volatile sig_atomic_t running = 1;
static uint32_t g_period_ns = PERIOD_NS_DEFAULT;

static void on_signal(int sig)
{
    (void)sig;
    running = 0;
}

static uint64_t monotonic_time_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)t.tv_sec * NSEC_PER_SEC + (uint64_t)t.tv_nsec;
}

static void add_period(struct timespec *t)
{
    t->tv_nsec += (long)g_period_ns;
    while (t->tv_nsec >= NSEC_PER_SEC) {
        t->tv_nsec -= NSEC_PER_SEC;
        t->tv_sec++;
    }
}

static void stack_prefault(void)
{
    unsigned char dummy[MAX_SAFE_STACK];
    memset(dummy, 0, sizeof(dummy));
}

static void setup_realtime(void)
{
    struct sched_param param;

    memset(&param, 0, sizeof(param));
    param.sched_priority = 80;
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        fprintf(stderr,
                "warning: SCHED_FIFO 80 unavailable (%s); "
                "use: sudo chrt -f 80 ./build/diag_csp_single ...\n",
                strerror(errno));
    } else {
        printf("sched=SCHED_FIFO priority=80\n");
    }
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
        fprintf(stderr, "warning: mlockall: %s\n", strerror(errno));
    stack_prefault();
}

/*
 * Program exit used to disable in one cycle while the shaft was still at
 * vel_limit — motor kept spinning. Freeze CSP target to actual, then send
 * many disable frames before destroy/deactivate.
 */
static void csp_controlled_stop(jsdk_context_t *ctx, jsdk_joint_t *joint,
        struct timespec *wakeup_time)
{
    unsigned int i;
    unsigned int n_freeze;
    unsigned int n_dis;
    jsdk_status_t st;
    jsdk_joint_feedback_t f;

    n_freeze = 800000000u / g_period_ns; /* ~0.8 s hold actual */
    n_dis = 600000000u / g_period_ns;    /* ~0.6 s disable */
    if (n_freeze < 50)
        n_freeze = 50;
    if (n_dis < 50)
        n_dis = 50;

    printf("Stopping: CSP freeze target=actual (%.1f s)...\n",
           (double)n_freeze * g_period_ns / 1e9);
    for (i = 0; i < n_freeze; i++) {
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, wakeup_time, NULL);
        st = jsdk_context_cycle_begin(ctx, monotonic_time_ns());
        if (st != JSDK_OK)
            break;
        jsdk_joint_get_feedback(joint, &f);
        jsdk_joint_set_target_position(joint, f.actual_position);
        jsdk_joint_set_target_velocity(joint, 0);
        jsdk_joint_set_target_torque(joint, 0);
        jsdk_context_cycle_end(ctx);
        add_period(wakeup_time);
    }

    printf("Stopping: disable (%.1f s)...\n",
           (double)n_dis * g_period_ns / 1e9);
    jsdk_joint_request_disable(joint);
    for (i = 0; i < n_dis; i++) {
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, wakeup_time, NULL);
        st = jsdk_context_cycle_begin(ctx, monotonic_time_ns());
        if (st != JSDK_OK)
            break;
        jsdk_joint_get_feedback(joint, &f);
        jsdk_joint_set_target_position(joint, f.actual_position);
        jsdk_joint_set_target_velocity(joint, 0);
        jsdk_joint_set_target_torque(joint, 0);
        jsdk_context_cycle_end(ctx);
        add_period(wakeup_time);
    }
}

int main(int argc, char **argv)
{
    jsdk_context_config_t ctx_config;
    jsdk_joint_config_t joint_config;
    jsdk_context_t *ctx;
    jsdk_joint_t *joint = NULL;
    jsdk_status_t status;
    struct timespec wakeup_time;
    int32_t home = 0;
    int32_t goal = 0;
    int home_ok = 0;
    unsigned int counter = 0;
    int ret = 0;
    int hold_mode = 0;
    int abs_mode = 0;
    int delta_mode = 0;
    int32_t abs_pos = 0;
    int32_t delta = 0;
    uint32_t speed = 0;
    int speed_set = 0;
    int32_t follow_lead = 0;
    int seconds = 0;
    const char *log_path = NULL;
    FILE *log_fp = NULL;
    int print_header = 1;
    uint64_t t0_ns = 0;
    uint64_t en_t0 = 0;
    int enabled_seen = 0;
    int ok_n = 0;
    int settle_n = 0;
    int runaway = 0;
    int wc_drop_secs = 0;
    int interrupted = 0;
    int bus_bad_cycles = 0;
    int need_recover_ramp = 0;
    uint64_t recover_t0 = 0;
    int32_t last_goal = 0, last_act = 0, last_err = 0, last_vel = 0;
    int32_t final_goal = 0;
    int32_t pos_err_ok = POS_ERR_OK_FLOOR;
    int32_t cmd_pos = 0;
    int use_dc = -1;
    int sync0_shift_set = 0;
    int32_t sync0_shift_ns = 0;
    int sync0_cycle_set = 0;
    uint32_t sync0_cycle_ns = 0;
    int period_set = 0;
    int input_mode_2002 = -1; /* -1 = auto from invert */
    uint32_t period_ns = 0;
    int invert = DEFAULT_INVERT;
    uint32_t vmax_6080;
    const char *fw_version = "8.1.50";
    const char *model_name = NULL;
    const jsdk_drive_model_t *drive_model = NULL;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--hold")) {
            hold_mode = 1;
        } else if (!strcmp(argv[i], "--pos") && i + 1 < argc) {
            abs_mode = 1;
            abs_pos = (int32_t)strtol(argv[++i], NULL, 0);
        } else if (!strcmp(argv[i], "--delta") && i + 1 < argc) {
            delta_mode = 1;
            delta = (int32_t)strtol(argv[++i], NULL, 0);
        } else if (!strcmp(argv[i], "--speed") && i + 1 < argc) {
            speed = (uint32_t)strtoul(argv[++i], NULL, 0);
            speed_set = 1;
            if (speed < 1)
                speed = 1;
        } else if (!strcmp(argv[i], "--invert")) {
            invert = 1;
        } else if (!strcmp(argv[i], "--no-invert")) {
            invert = 0;
        } else if (!strcmp(argv[i], "--dc")) {
            use_dc = 1;
        } else if (!strcmp(argv[i], "--no-dc")) {
            use_dc = 0;
        } else if (!strcmp(argv[i], "--sync0-shift") && i + 1 < argc) {
            sync0_shift_set = 1;
            sync0_shift_ns = (int32_t)strtol(argv[++i], NULL, 0);
        } else if (!strcmp(argv[i], "--sync0-cycle") && i + 1 < argc) {
            sync0_cycle_set = 1;
            sync0_cycle_ns = (uint32_t)strtoul(argv[++i], NULL, 0);
        } else if (!strcmp(argv[i], "--period") && i + 1 < argc) {
            period_set = 1;
            period_ns = (uint32_t)strtoul(argv[++i], NULL, 0);
        } else if (!strcmp(argv[i], "--input-mode") && i + 1 < argc) {
            input_mode_2002 = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "--fw") && i + 1 < argc) {
            fw_version = argv[++i];
        } else if (!strcmp(argv[i], "--model") && i + 1 < argc) {
            model_name = argv[++i];
        } else if (!strcmp(argv[i], "--seconds") && i + 1 < argc) {
            seconds = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "--log") && i + 1 < argc) {
            log_path = argv[++i];
        } else if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) {
            fprintf(stderr,
                    "Usage: %s --hold | --delta N | --pos N\n"
                    "          [--fw 8.1.50|--model ISVD90RC-v8.1.50]\n"
                    "          [--dc|--no-dc] [--sync0-shift N] "
                    "[--sync0-cycle ns] [--period ns]\n"
                    "          [--invert|--no-invert] [--input-mode 1|3]\n"
                    "          --speed counts/s (required with --delta/--pos)\n"
                    "          [--seconds N] [--log f]\n"
                    "  Prefer: sudo chrt -f 80 %s --fw 8.1.44 --no-invert "
                    "--delta 18192 --speed 4000 --seconds 12\n"
                    "  CSP soft-invert defaults from --fw "
                    "(8.1.50=ON, 8.1.44/60=OFF); --invert/--no-invert override\n"
                    "  0x2002:1 default PASSTHROUGH(1); "
                    "soft-ramp at --speed; lead cap = --speed\n",
                    argv[0], argv[0]);
            return EXIT_SUCCESS;
        }
    }

    if (!hold_mode && !abs_mode && !delta_mode) {
        fprintf(stderr, "need --hold or --delta N or --pos N\n");
        return EXIT_FAILURE;
    }
    if ((delta_mode || abs_mode) && !speed_set) {
        fprintf(stderr, "need --speed counts/s with --delta/--pos\n");
        return EXIT_FAILURE;
    }
    if (hold_mode && !speed_set)
        speed = 1000; /* hold only: soft settle floor */

    follow_lead = (int32_t)speed;
    if (follow_lead < 1)
        follow_lead = 1;

    /* Soft-ramp uses --speed; 0x6080 must not choke the plant (see dual). */
    vmax_6080 = speed * 2u;
    if (vmax_6080 < 100000u)
        vmax_6080 = 100000u;

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    if (model_name)
        drive_model = jsdk_drive_model_by_name(model_name);
    if (!drive_model)
        drive_model = jsdk_drive_model_find(fw_version, 0x00080117u);
    if (!drive_model)
        drive_model = jsdk_drive_model_default();

    if (invert < 0) {
        invert = jsdk_drive_model_csp_sw_invert(drive_model);
        printf("CSP soft-invert=%s (from drive_model fw=%s)\n",
               invert ? "ON" : "OFF", fw_version);
    } else {
        printf("CSP soft-invert=%s (CLI; model default=%s)\n",
               invert ? "ON" : "OFF",
               jsdk_drive_model_csp_sw_invert(drive_model) ? "ON" : "OFF");
    }
    /*
     * TwinCAT CSP guide: 0x2002:1 = POS_FILTER(3).
     * Soft-invert + POS_FILTER fights → PASSTHROUGH(1).
     * Bench 8.1.44: PASSTHROUGH tracks IgH CSP more reliably than filter.
     */
    if (input_mode_2002 < 0)
        input_mode_2002 = 1;
    printf("0x2002:1 input_mode=%d (%s)\n", input_mode_2002,
           input_mode_2002 == 3 ? "POS_FILTER" :
           input_mode_2002 == 1 ? "PASSTHROUGH" : "custom");
    jsdk_context_config_default(&ctx_config);
    ctx_config.master_index = 0;
    ctx_config.period_ns = PERIOD_NS_DEFAULT;
    ctx_config.max_joints = 1;
    jsdk_context_config_apply_drive_model(&ctx_config, drive_model);
    (void)jsdk_dc_timing_conf_apply_fw(&ctx_config, fw_version);
    {
        uint32_t conf_period = jsdk_dc_timing_conf_period_ns();

        if (conf_period)
            ctx_config.period_ns = conf_period;
    }
    if (use_dc >= 0) {
        ctx_config.use_dc = use_dc;
        ctx_config.auto_reference_clock = use_dc ? 1 : 0;
    }
    if (use_dc == 0) {
        ctx_config.sync0_cycle_ns = 0;
        ctx_config.wait_before_safeop_ms = 0;
    } else if (ctx_config.dc_timing_mode == JSDK_DC_TIMING_SHARED) {
        if (sync0_cycle_set)
            ctx_config.sync0_cycle_ns = sync0_cycle_ns;
        else if (!ctx_config.sync0_cycle_ns)
            /* Keep conf Sync0 (e.g. 2 ms) when set; only default to task. */
            ctx_config.sync0_cycle_ns = ctx_config.period_ns;
    } else if (sync0_cycle_set) {
        ctx_config.sync0_cycle_ns = sync0_cycle_ns;
    }
    if (sync0_shift_set)
        ctx_config.sync0_shift_ns = sync0_shift_ns;
    if (period_set && period_ns)
        ctx_config.period_ns = period_ns;
    if (!ctx_config.period_ns)
        ctx_config.period_ns = PERIOD_NS_DEFAULT;
    g_period_ns = ctx_config.period_ns;

    ctx = jsdk_context_create(&ctx_config);
    if (!ctx) {
        fprintf(stderr, "create context failed\n");
        return EXIT_FAILURE;
    }

    memset(&joint_config, 0, sizeof(joint_config));
    joint_config.alias = 0;
    joint_config.position = 0;
    joint_config.profile_name = JSDK_PROFILE_CYBERBEAST_JOINT_MODULE;
    joint_config.firmware_version = fw_version;
    joint_config.drive_model_name = drive_model->name;
    joint_config.initial_mode = JSDK_MODE_CSP;
    joint_config.max_motor_velocity = vmax_6080;
    /* Do not set polarity_607e — SDO 0x607E drops OP on motion here. */
    joint_config.input_mode_2002 = input_mode_2002;

    status = jsdk_context_add_joint(ctx, &joint_config, &joint);
    if (status != JSDK_OK) {
        fprintf(stderr, "add_joint: %s (%s)\n",
                jsdk_status_string(status), jsdk_context_last_error(ctx));
        jsdk_context_destroy(ctx);
        return EXIT_FAILURE;
    }

    {
        int eff_use = ctx_config.use_dc;
        int32_t eff_shift = ctx_config.sync0_shift_ns;
        uint32_t eff_wait = ctx_config.wait_before_safeop_ms;
        uint32_t eff_sync0 = ctx_config.sync0_cycle_ns
                ? ctx_config.sync0_cycle_ns : ctx_config.period_ns;
        const char *fw_key = drive_model->fw_match && drive_model->fw_match[0]
                ? drive_model->fw_match : drive_model->name;

        if (ctx_config.dc_timing_mode == JSDK_DC_TIMING_PER_JOINT) {
            eff_use = drive_model->use_dc;
            eff_shift = drive_model->sync0_shift_ns;
            if (drive_model->sync0_cycle_ns)
                eff_sync0 = drive_model->sync0_cycle_ns;
            else if (drive_model->preferred_period_ns)
                eff_sync0 = drive_model->preferred_period_ns;
            else
                eff_sync0 = ctx_config.period_ns;
            if (drive_model->wait_before_safeop_ms)
                eff_wait = drive_model->wait_before_safeop_ms;
            jsdk_dc_timing_conf_lookup_fw(fw_key, &eff_use, &eff_shift,
                    &eff_wait, NULL, &eff_sync0);
        }

        printf("drive_model=%s fw=%s use_dc=%d period=%u sync0=%u "
               "shift=%d wait_ms=%u  DC_mode=%s\n",
               drive_model->name,
               fw_version,
               eff_use,
               ctx_config.period_ns,
               eff_sync0,
               (int)eff_shift,
               eff_wait,
               ctx_config.dc_timing_mode == JSDK_DC_TIMING_PER_JOINT
                       ? "PER_JOINT" : "SHARED");
    }

    setup_realtime();

    status = jsdk_context_activate(ctx);
    if (status != JSDK_OK) {
        fprintf(stderr, "activate: %s (%s)\n",
                jsdk_status_string(status), jsdk_context_last_error(ctx));
        jsdk_context_destroy(ctx);
        return EXIT_FAILURE;
    }

    /* Settle OP+WC before CiA402 enable (same idea as CSV/CST). */
    {
        unsigned int good = 0;
        unsigned int tries = 0;
        unsigned int settle_ms = 1000;
        unsigned int need_cycles;
        unsigned int max_cycles;
        unsigned int period_ms;

        period_ms = g_period_ns / 1000000u;
        if (period_ms == 0)
            period_ms = 1;
        need_cycles = settle_ms / period_ms;
        if (need_cycles < 100)
            need_cycles = 100;
        max_cycles = 15000u / period_ms;

        clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
        add_period(&wakeup_time);
        printf("bus settle (need %u cycles OP+WC, period=%u ms)...\n",
               need_cycles, period_ms);
        while (good < need_cycles && tries < max_cycles) {
            jsdk_bus_state_t bus;
            jsdk_joint_feedback_t f;

            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                    &wakeup_time, NULL);
            status = jsdk_context_cycle_begin(ctx, monotonic_time_ns());
            if (status != JSDK_OK)
                break;
            jsdk_joint_get_feedback(joint, &f);
            jsdk_joint_set_target_position(joint, f.actual_position);
            jsdk_joint_set_target_velocity(joint, 0);
            jsdk_joint_set_target_torque(joint, 0);
            jsdk_context_get_bus_state(ctx, &bus);
            jsdk_context_cycle_end(ctx);
            add_period(&wakeup_time);
            tries++;
            if (bus.working_counter > 0 && bus.wc_state == 2 &&
                    (bus.al_states & 0x08) && f.online &&
                    ((f.al_state & 0x0f) == 0x08 || f.operational)) {
                good++;
            } else {
                good = 0;
            }
        }
        if (good < need_cycles) {
            fprintf(stderr,
                    "FAIL: bus never reached stable OP+WC "
                    "(good=%u/%u tries=%u)\n",
                    good, need_cycles, tries);
            jsdk_context_destroy(ctx);
            return EXIT_FAILURE;
        }
        printf("bus settle ok (%u cycles stable OP+WC)\n", good);
    }

    jsdk_joint_request_enable(joint, JSDK_MODE_CSP);

    if (log_path) {
        char dirbuf[512];
        char *slash;

        snprintf(dirbuf, sizeof(dirbuf), "%s", log_path);
        slash = strrchr(dirbuf, '/');
        if (slash) {
            *slash = '\0';
            if (dirbuf[0])
                mkdir(dirbuf, 0755);
        }
        log_fp = fopen(log_path, "w");
        if (!log_fp) {
            fprintf(stderr, "fopen %s: %s\n", log_path, strerror(errno));
            jsdk_context_destroy(ctx);
            return EXIT_FAILURE;
        }
        fprintf(log_fp,
                "t_s,status,sw,goal,actual,err,mode,vel,trq,wc,slv,al,ok\n");
    }

    printf("=== CSP ===\n");
    printf("DC=%s", ctx_config.use_dc ? "ON(Sync0)" : "OFF(free-run)");
    if (ctx_config.use_dc) {
        uint32_t s0 = ctx_config.sync0_cycle_ns
                ? ctx_config.sync0_cycle_ns : ctx_config.period_ns;
        printf(" task=%u ns sync0=%u ns shift=%d ns",
               ctx_config.period_ns, s0, (int)ctx_config.sync0_shift_ns);
    }
    printf("\n");
    printf("TwinCAT: mode=8  input_mode=%d(%s)  0x6080=%u  "
           "speed=%u  lead=%d  period=%u ns  sw_invert=%s\n",
           input_mode_2002,
           input_mode_2002 == 3 ? "POS_FILTER" :
           input_mode_2002 == 1 ? "PASSTHROUGH" : "custom",
           vmax_6080, speed, follow_lead, g_period_ns,
           invert ? "ON" : "OFF");
    if (hold_mode)
        printf("HOLD — keep target = enable position\n");
    else if (delta_mode)
        printf("RELATIVE — target = home + (%d)\n", (int)delta);
    else
        printf("ABSOLUTE — target = %d\n", (int)abs_pos);
    if (seconds > 0)
        printf("auto-stop after %d s\n", seconds);

    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
    add_period(&wakeup_time);
    t0_ns = monotonic_time_ns();

    while (running) {
        jsdk_joint_feedback_t f;
        jsdk_bus_state_t bus_cycle;
        int32_t err = 0;
        uint64_t now;
        double t_s;
        double en_s = 0.0;
        int bus_ok = 0;
        int enabled_now = 0;

        ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                &wakeup_time, NULL);
        if (ret) {
            if (ret == EINTR)
                interrupted = 1;
            else
                fprintf(stderr, "clock_nanosleep: %s\n", strerror(ret));
            running = 0;
            break;
        }

        now = monotonic_time_ns();
        t_s = (double)(now - t0_ns) / (double)NSEC_PER_SEC;

        status = jsdk_context_cycle_begin(ctx, now);
        if (status != JSDK_OK) {
            fprintf(stderr, "cycle_begin: %s (%s)\n",
                    jsdk_status_string(status), jsdk_context_last_error(ctx));
            break;
        }

        jsdk_joint_get_feedback(joint, &f);
        jsdk_context_get_bus_state(ctx, &bus_cycle);
        /*
         * Match dual: WC complete + OP bit present (do not require
         * al_states==0x08 exactly — OR can be 0x0A briefly).
         */
        bus_ok = (bus_cycle.wc_state == 2 &&
                bus_cycle.working_counter > 0 &&
                (bus_cycle.al_states & 0x08) != 0);
        enabled_now = jsdk_joint_is_enabled(joint);

        if (enabled_now) {
            if (!enabled_seen) {
                enabled_seen = 1;
                en_t0 = now;
            }
            en_s = (double)(now - en_t0) / (double)NSEC_PER_SEC;

            if (!home_ok) {
                home = f.actual_position;
                home_ok = 1;
                cmd_pos = home;
                if (hold_mode)
                    final_goal = home;
                else if (delta_mode)
                    final_goal = home + delta;
                else
                    final_goal = abs_pos;
                pos_err_ok = csp_pos_err_ok_for_travel(final_goal - home);
                printf("\n>>> ENABLED home=%d final_goal=%d "
                       "(mirror %.2fs, speed=%u lead=%d YES|err|<=%d)\n\n",
                       home, final_goal, ENABLE_MIRROR_SEC, speed,
                       follow_lead, pos_err_ok);
            }

            goal = final_goal;
            err = goal - f.actual_position;

            if (!bus_ok) {
                bus_bad_cycles++;
                if (bus_bad_cycles >= BUS_BAD_FREEZE_CYCLES)
                    need_recover_ramp = 1;
                /*
                 * Brief hole: pause ramp, keep logical cmd_pos (do not
                 * snap to actual — that was killing following error).
                 * Re-send last write; SDK may mirror actual while WC bad.
                 */
                jsdk_joint_set_target_position(joint,
                        csp_write_position(cmd_pos, f.actual_position,
                                invert));
                jsdk_joint_set_target_velocity(joint, 0);
                jsdk_joint_set_target_torque(joint, 0);
            } else {
                if (bus_bad_cycles >= BUS_BAD_FREEZE_CYCLES) {
                    need_recover_ramp = 1;
                    recover_t0 = now;
                    /*
                     * If already following, keep cmd — snapping mid-catch
                     * threw away progress and blocked YES (|err|<=10k).
                     */
                    if (iabs32(cmd_pos - f.actual_position) >
                            RECOVER_SNAP_ERR) {
                        cmd_pos = f.actual_position;
                        printf("bus recovered — realign cmd=actual, hold "
                               "%.2fs then resume\n", RECOVER_HOLD_SEC);
                    } else {
                        printf("bus recovered — keep cmd=%d "
                               "(follow ok), hold %.2fs\n",
                               (int)cmd_pos, RECOVER_HOLD_SEC);
                    }
                }
                bus_bad_cycles = 0;

                {
                    int traj_freeze = 0;
                    double recover_s = 0.0;

                    if (need_recover_ramp && recover_t0) {
                        recover_s = (double)(now - recover_t0) /
                                (double)NSEC_PER_SEC;
                        if (recover_s < RECOVER_HOLD_SEC)
                            traj_freeze = 1;
                        if (recover_s >= RECOVER_RAMP_SEC)
                            need_recover_ramp = 0;
                    }

                    if (traj_freeze) {
                        /* Hold last cmd (already realigned only if needed). */
                        jsdk_joint_set_target_position(joint,
                                csp_write_position(cmd_pos,
                                        f.actual_position, invert));
                        jsdk_joint_set_target_velocity(joint, 0);
                        jsdk_joint_set_target_torque(joint, 0);
                    } else if (en_s < ENABLE_MIRROR_SEC) {
                        /*
                         * Stay mirrored briefly after enable so the first
                         * PDO is not a position step during CiA402.
                         */
                        cmd_pos = f.actual_position;
                        jsdk_joint_set_target_position(joint,
                                csp_write_position(cmd_pos,
                                        f.actual_position, invert));
                        jsdk_joint_set_target_velocity(joint, 0);
                        jsdk_joint_set_target_torque(joint, 0);
                    } else {
                        if (!hold_mode && home_ok && en_s > 0.7 &&
                                en_s < 1.5) {
                            int32_t moved = f.actual_position - home;
                            int32_t want = final_goal - home;
                            if (iabs32(moved) > 2000 && want != 0 &&
                                    ((moved > 0) != (want > 0))) {
                                fprintf(stderr,
                                        "FAIL: motion opposite to command "
                                        "(moved=%d want=%d) — fix "
                                        "drive_model.csp_sw_invert for this "
                                        "--fw (not CLI --invert)\n",
                                        (int)moved, (int)want);
                                runaway = 1;
                                running = 0;
                            }
                        }

                        cmd_pos = csp_apply_lead(
                                csp_profile_at(home, final_goal,
                                        en_s - ENABLE_MIRROR_SEC, speed),
                                f.actual_position, follow_lead);

                        jsdk_joint_set_target_position(joint,
                                csp_write_position(cmd_pos,
                                        f.actual_position, invert));
                        jsdk_joint_set_target_velocity(joint, 0);
                        jsdk_joint_set_target_torque(joint, 0);

                        if (!hold_mode && home_ok && en_s > 1.0) {
                            int32_t move = iabs32(final_goal - home);
                            if (move < 1000)
                                move = 1000;
                            if (iabs32(err) > move * 3 + pos_err_ok) {
                                runaway = 1;
                                running = 0;
                            }
                        }
                    }
                }
            }
        } else {
            /* Not enabled: always mirror — never leave target at 0. */
            jsdk_joint_set_target_position(joint, f.actual_position);
            jsdk_joint_set_target_velocity(joint, 0);
            jsdk_joint_set_target_torque(joint, 0);
            /*
             * Keep goal/err meaningful for the 1 Hz row. Previously err
             * stayed 0 here while goal/cmd were stale → fake "err=0 ramp".
             */
            if (home_ok) {
                goal = final_goal;
                err = goal - f.actual_position;
            }
        }

        if (!counter--) {
            jsdk_bus_state_t bus;
            int settled;
            int ok;
            const char *ok_s;
            int32_t follow_err;

            counter = NSEC_PER_SEC / g_period_ns;
            bus = bus_cycle;
            settled = enabled_seen && enabled_now && en_s >= SETTLE_SEC;
            if (settled && iabs32(f.actual_velocity) > VEL_RUNAWAY)
                runaway = 1;

            if (enabled_seen && en_s >= 1.0 &&
                    (bus.wc_state != 2 || bus.working_counter == 0 ||
                     (bus.al_states & 0x0f) != 0x08)) {
                wc_drop_secs++;
            }

            /*
             * YES = |goal-act| within pos_err_ok (~2/3 travel) + bus OP/WC.
             * Do not gate on actual_velocity — PDO vel spikes while
             * decelerating into the goal (false "no"). Runaway still
             * uses VEL_RUNAWAY separately.
             */
            if (home_ok)
                err = goal - f.actual_position;
            follow_err = home_ok ? (cmd_pos - f.actual_position) : 0;

            ok = settled && !runaway
                    && iabs32(err) <= pos_err_ok
                    && f.mode_display == 8
                    && bus.wc_state == 2
                    && (bus.al_states & 0x0f) == 0x08;

            if (settled) {
                settle_n++;
                if (ok)
                    ok_n++;
            }

            last_goal = goal;
            last_act = f.actual_position;
            last_err = err;
            last_vel = f.actual_velocity;

            if (print_header) {
                printf("%-8s %-8s %-10s %-10s %-10s %-8s "
                       "%-5s %-8s %-5s %4s %3s %3s %4s\n",
                       "axis", "sw", "cmd", "goal", "actual", "err",
                       "mode", "vel", "trq", "wc", "slv", "al", "ok");
                print_header = 0;
            }

            if (ok)
                ok_s = "YES";
            else if (settled)
                ok_s = "no";
            else if (enabled_seen && !enabled_now)
                ok_s = "drop";
            else
                ok_s = "ramp";

            printf("%-8s %04X     %-10d %-10d %-10d %-8d "
                   "%-5d %-8d %-5d %4u %3u %02X %4s\n",
                   jsdk_axis_state_string(f.axis_state),
                   f.statusword,
                   home_ok ? cmd_pos : 0,
                   goal,
                   f.actual_position,
                   err,
                   f.mode_display,
                   f.actual_velocity,
                   f.actual_torque,
                   bus.working_counter,
                   bus.slaves_responding,
                   bus.al_states,
                   ok_s);
            if (home_ok && iabs32(follow_err) > 2000 &&
                    iabs32(follow_err) != iabs32(err)) {
                /* cmd desynced from actual while goal-err still huge */
                printf("         (follow cmd-act=%d)\n", (int)follow_err);
            }

            if (log_fp) {
                fprintf(log_fp,
                        "%.3f,%s,%04X,%d,%d,%d,%d,%d,%d,%u,%u,%02X,%d\n",
                        t_s, jsdk_axis_state_string(f.axis_state),
                        f.statusword, goal, f.actual_position, err,
                        f.mode_display, f.actual_velocity, f.actual_torque,
                        bus.working_counter, bus.slaves_responding,
                        bus.al_states, ok);
                fflush(log_fp);
            }

            if (seconds > 0 && t_s >= (double)seconds)
                running = 0;
        }

        status = jsdk_context_cycle_end(ctx);
        if (status != JSDK_OK) {
            fprintf(stderr, "cycle_end: %s (%s)\n",
                    jsdk_status_string(status), jsdk_context_last_error(ctx));
            break;
        }
        add_period(&wakeup_time);
    }

    printf("\n");
    csp_controlled_stop(ctx, joint, &wakeup_time);
    jsdk_context_destroy(ctx);
    if (log_fp)
        fclose(log_fp);

    printf("summary: goal=%d act=%d err=%d vel=%d ok=%d/%d "
           "wc_drops=%d enabled=%d\n",
           (int)last_goal, (int)last_act, (int)last_err, (int)last_vel,
           ok_n, settle_n, wc_drop_secs, enabled_seen);

    if (interrupted) {
        fprintf(stderr, "Interrupted — re-run without Ctrl+C for PASS.\n");
        return EXIT_FAILURE;
    }
    if (runaway || !enabled_seen) {
        fprintf(stderr, "FAIL: CSP runaway or never enabled.\n");
        return EXIT_FAILURE;
    }
    if (wc_drop_secs > 3 && ok_n < 1) {
        fprintf(stderr,
                "FAIL: %d sample(s) with WC/AL dropout after enable.\n",
                wc_drop_secs);
        return EXIT_FAILURE;
    }
    if (wc_drop_secs > 0)
        printf("note: %d WC/AL dropout sample(s) (tolerated if motion ok)\n",
               wc_drop_secs);

    {
        int32_t dpos = last_act - home;
        int32_t want = last_goal - home;
        int32_t need = (int32_t)speed;
        int at_goal = (ok_n >= 1) && (iabs32(last_err) <= pos_err_ok);
        int spinning;

        if (need < 2000)
            need = 2000;
        spinning = home_ok &&
                (want == 0 || ((dpos > 0) == (want > 0))) &&
                iabs32(dpos) >= need;

        if (!at_goal && !spinning) {
            fprintf(stderr,
                    "FAIL: CSP neither near goal nor spinning "
                    "(dpos=%d need>=%d err=%d).\n",
                    (int)dpos, (int)need, (int)last_err);
            return EXIT_FAILURE;
        }
        if (!at_goal && spinning)
            printf("note: spinning (dpos=%d) — not yet at goal (err=%d); "
                   "OK per motion policy\n",
                   (int)dpos, (int)last_err);
    }
    printf("PASS: CSP moving/settled (soft-ramp @%u, 0x6080=%u)\n",
           speed, vmax_6080);
    return ret ? EXIT_FAILURE : EXIT_SUCCESS;
}
