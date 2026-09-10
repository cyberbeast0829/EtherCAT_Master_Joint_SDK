/*****************************************************************************
 *
 *  守护兽关节 SDK — 双轴 CSP 最小诊断（同一主站、同一 TwinCAT DC）
 *
 *  用法示例:
 *    sudo chrt -f 80 ./diag_csp_dual --fw0 8.1.50 --fw1 8.1.44 \
 *         --invert0 --no-invert1 --delta 18192 --speed 4000 --seconds 15
 *    sudo chrt -f 80 ./diag_csp_dual --abs0 <A> --abs1 <B> --speed 4000
 *    sudo chrt -f 80 ./diag_csp_dual --pos <绝对> --speed 4000   # 两轴同一绝对值
 *    sudo chrt -f 80 ./diag_csp_dual --hold --seconds 6
 *
 *  --pos0/--pos1: EtherCAT 站址（默认 0 / 1）；参考时钟 = 第 0 个 add_joint
 *  --fw0/--fw1: 各站 motor-app 版本（DC/型号）；默认 8.1.50 / 8.1.60
 *  --invertN / --no-invertN: 每轴软件位置误差反相（write=2*act-cmd）
 *  --speed: 必填；两轴共用进度 frac（最慢轴 lead 卡住快轴）→ err 对齐、同停
 *  --pos / --abs0/--abs1: 绝对目标（同样 sync-frac）
 *  PASS: 到点或已朝目标转起来
 *  DC 取自 conf / --fw0；[--no-dc] [--twincat-dc] [--sync0-cycle N]
 *
 ****************************************************************************/

#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <limits.h>
#include <sched.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include <joint_sdk/dc_timing_conf.h>
#include <joint_sdk/joint_sdk.h>

#define N_AXES 2
#define NSEC_PER_SEC 1000000000L
#define MAX_SAFE_STACK (8 * 1024)
/*
 * Dual CSP: one shared progress fraction for both axes.
 * Ideal frac from --speed×t; each axis also limits frac by |cmd-act|<=lead.
 * frac = min(ideal, lim0, lim1) → slower plant holds the other back →
 * same err, move/stop together. Invert only affects PDO write.
 */
#define SETTLE_SEC 0.10
/** Floor for YES; large moves use ~2/3 of |travel| so YES comes earlier. */
#define POS_ERR_OK_FLOOR 15000
#define MIN_TRACK_OK 3
#define ENABLE_MIRROR_SEC 0.08
/** Brief WC/AL holes allowed if motion still finishes cleanly. */
#define WC_DROP_TOLERANCE 8
/** Need this many consecutive bad 1 Hz samples before counting a drop. */
#define WC_DROP_DEBOUNCE 3
/** App: freeze trajectory only after sustained bus hole (~50 ms @ 1 ms). */
#define BUS_BAD_FREEZE_CYCLES 50
#define RECOVER_HOLD_SEC 0.20
#define RECOVER_RAMP_SEC 0.35

static volatile sig_atomic_t running = 1;
static uint32_t g_period_ns = 1000000u;

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

static void setup_realtime(void)
{
    struct sched_param param;
    unsigned char dummy[MAX_SAFE_STACK];

    memset(&param, 0, sizeof(param));
    param.sched_priority = 80;
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        fprintf(stderr,
                "warning: SCHED_FIFO 80 unavailable (%s); "
                "use: sudo chrt -f 80 ...\n",
                strerror(errno));
    } else {
        printf("sched=SCHED_FIFO priority=80\n");
    }
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
        fprintf(stderr, "warning: mlockall: %s\n", strerror(errno));
    memset(dummy, 0, sizeof(dummy));
}

static int32_t iabs32(int32_t v)
{
    return v < 0 ? -v : v;
}

static int32_t csp_pos_err_ok_for_travel(int32_t travel)
{
    int32_t mag = iabs32(travel);
    int32_t ok;

    /* YES after ~1/3 of the move (|err| still ~2/3 of travel). */
    ok = mag - mag / 3;
    if (ok < POS_ERR_OK_FLOOR)
        ok = POS_ERR_OK_FLOOR;
    if (mag > 0 && ok >= mag)
        ok = mag / 2;
    if (ok < 1)
        ok = 1;
    return ok;
}

/**
 * Actual progress fraction [0,1] of an axis along home→goal.
 */
static double csp_frac_actual(int32_t home, int32_t travel, int32_t actual)
{
    int64_t t = travel;
    int64_t d = (int64_t)actual - (int64_t)home;
    double f;

    if (t == 0)
        return 1.0;
    f = (double)d / (double)t;
    if (f < 0.0)
        f = 0.0;
    if (f > 1.0)
        f = 1.0;
    return f;
}

/**
 * Ideal progress fraction [0,1] after move_s at --speed over |travel|.
 */
static double csp_frac_ideal(double move_s, uint32_t speed, int32_t travel)
{
    int64_t mag = travel < 0 ? -(int64_t)travel : (int64_t)travel;
    int64_t planned;

    if (mag <= 0)
        return 1.0;
    if (move_s < 0.0)
        move_s = 0.0;
    planned = (int64_t)((double)speed * move_s + 0.5);
    if (planned >= mag)
        return 1.0;
    return (double)planned / (double)mag;
}

static int32_t csp_cmd_from_frac(int32_t home, int32_t goal, double frac)
{
    int64_t travel = (int64_t)goal - (int64_t)home;
    int64_t cmd;

    if (frac < 0.0)
        frac = 0.0;
    if (frac > 1.0)
        frac = 1.0;
    cmd = (int64_t)home + (int64_t)(travel * frac + (travel >= 0 ? 0.5 : -0.5));
    if (cmd > INT32_MAX)
        cmd = INT32_MAX;
    if (cmd < INT32_MIN)
        cmd = INT32_MIN;
    return (int32_t)cmd;
}

/* Effective Sync0/shift/wait: SHARED→context; PER_JOINT→model+conf. */
static void print_joint_dc_line(unsigned int idx, const char *fw,
        const jsdk_context_config_t *cfg)
{
    const jsdk_drive_model_t *m =
            jsdk_drive_model_find(fw, 0x00080117u);
    int use_dc = cfg->use_dc;
    int32_t shift = cfg->sync0_shift_ns;
    uint32_t wait_ms = cfg->wait_before_safeop_ms;
    uint32_t sync0 = cfg->sync0_cycle_ns ? cfg->sync0_cycle_ns
                                         : cfg->period_ns;

    if (!m)
        m = jsdk_drive_model_default();

    if (cfg->dc_timing_mode == JSDK_DC_TIMING_PER_JOINT) {
        const char *key = m->fw_match && m->fw_match[0] ? m->fw_match
                                                        : m->name;

        use_dc = m->use_dc;
        shift = m->sync0_shift_ns;
        if (m->sync0_cycle_ns)
            sync0 = m->sync0_cycle_ns;
        else if (m->preferred_period_ns)
            sync0 = m->preferred_period_ns;
        else
            sync0 = cfg->period_ns;
        if (m->wait_before_safeop_ms)
            wait_ms = m->wait_before_safeop_ms;
        jsdk_dc_timing_conf_lookup_fw(key, &use_dc, &shift, &wait_ms,
                NULL, &sync0);
    }

    printf("  joint%u fw=%s model=%s sync0=%u shift=%d wait=%u "
           "use_dc=%d\n",
           idx, fw, m->name, sync0, (int)shift, wait_ms, use_dc);
}

static int32_t csp_write_position(int32_t cmd_pos, int32_t actual, int invert)
{
    int64_t w;

    if (!invert)
        return cmd_pos;
    w = (int64_t)actual * 2 - (int64_t)cmd_pos;
    if (w > INT32_MAX)
        w = INT32_MAX;
    else if (w < INT32_MIN)
        w = INT32_MIN;
    return (int32_t)w;
}

typedef struct {
    jsdk_joint_t *joint;
    uint16_t position;
    const char *fw;
    int invert;
    int32_t home;
    int32_t final_goal;
    int32_t cmd_pos;
    int32_t pos_err_ok; /* YES when |goal-act| <= this (per travel) */
    int home_ok;
    int reached; /* latched once near goal with good bus */
    int ok_n;
    int settle_n;
    int hold_n; /* samples during WC hold after reach */
    int runaway;
    int32_t last_act;
    int32_t last_err;
    int32_t last_vel;
    double stall_s; /* consecutive stalled seconds while far from goal */
} axis_t;

static void axis_controlled_stop(jsdk_context_t *ctx, axis_t *axes,
        struct timespec *wakeup_time)
{
    unsigned int n_freeze = 800000000u / g_period_ns;
    unsigned int n_dis = 600000000u / g_period_ns;
    unsigned int i, a;

    if (n_freeze < 50)
        n_freeze = 50;
    if (n_dis < 50)
        n_dis = 50;

    printf("Stopping: freeze both axes (%.1f s)...\n",
           (double)n_freeze * g_period_ns / 1e9);
    for (i = 0; i < n_freeze; i++) {
        jsdk_joint_feedback_t f;
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, wakeup_time, NULL);
        if (jsdk_context_cycle_begin(ctx, monotonic_time_ns()) != JSDK_OK)
            break;
        for (a = 0; a < N_AXES; a++) {
            jsdk_joint_get_feedback(axes[a].joint, &f);
            jsdk_joint_set_target_position(axes[a].joint, f.actual_position);
            jsdk_joint_set_target_velocity(axes[a].joint, 0);
            jsdk_joint_set_target_torque(axes[a].joint, 0);
        }
        jsdk_context_cycle_end(ctx);
        add_period(wakeup_time);
    }

    printf("Stopping: disable both (%.1f s)...\n",
           (double)n_dis * g_period_ns / 1e9);
    for (a = 0; a < N_AXES; a++)
        jsdk_joint_request_disable(axes[a].joint);
    for (i = 0; i < n_dis; i++) {
        jsdk_joint_feedback_t f;
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, wakeup_time, NULL);
        if (jsdk_context_cycle_begin(ctx, monotonic_time_ns()) != JSDK_OK)
            break;
        for (a = 0; a < N_AXES; a++) {
            jsdk_joint_get_feedback(axes[a].joint, &f);
            jsdk_joint_set_target_position(axes[a].joint, f.actual_position);
            jsdk_joint_set_target_velocity(axes[a].joint, 0);
            jsdk_joint_set_target_torque(axes[a].joint, 0);
        }
        jsdk_context_cycle_end(ctx);
        add_period(wakeup_time);
    }
}

int main(int argc, char **argv)
{
    jsdk_context_config_t ctx_config;
    jsdk_context_t *ctx;
    jsdk_status_t status;
    struct timespec wakeup_time;
    axis_t axes[N_AXES];
    int hold_mode = 0;
    int delta_mode = 0;
    int abs_mode = 0;
    int32_t delta = 0;
    int32_t abs_pos[N_AXES];
    int abs_set[N_AXES];
    uint32_t speed = 0;
    int speed_set = 0;
    int32_t follow_lead = 0;
    uint32_t vmax_6080;
    int seconds = 0; /* 0 = run until Ctrl+C */
    int wc_drop_secs = 0;
    int wc_bad_streak = 0;
    int interrupted = 0;
    unsigned int counter = 0;
    uint64_t t0_ns = 0;
    uint64_t en_t0 = 0;
    int enabled_seen = 0;
    int print_header = 1;
    int a;
    int use_dc = 1;
    int twincat_dc = 0;
    int sync0_cycle_set = 0;
    uint32_t sync0_cycle_ns = 0;
    int bus_bad_cycles = 0;
    int need_recover_ramp = 0;
    uint64_t recover_t0 = 0;
    const jsdk_drive_model_t *drive_model;

    memset(axes, 0, sizeof(axes));
    memset(abs_pos, 0, sizeof(abs_pos));
    memset(abs_set, 0, sizeof(abs_set));
    axes[0].position = 0;
    axes[1].position = 1;
    axes[0].fw = "8.1.50";
    axes[1].fw = "8.1.60";
    /* CLI polarity: Prefer --no-invert0 --invert1 */
    axes[0].invert = 0;
    axes[1].invert = 1;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--hold")) {
            hold_mode = 1;
        } else if (!strcmp(argv[i], "--delta") && i + 1 < argc) {
            delta_mode = 1;
            delta = (int32_t)strtol(argv[++i], NULL, 0);
        } else if (!strcmp(argv[i], "--pos") && i + 1 < argc) {
            /* Absolute goal for both axes (same value). */
            abs_mode = 1;
            abs_pos[0] = abs_pos[1] = (int32_t)strtol(argv[++i], NULL, 0);
            abs_set[0] = abs_set[1] = 1;
        } else if (!strcmp(argv[i], "--abs0") && i + 1 < argc) {
            abs_mode = 1;
            abs_pos[0] = (int32_t)strtol(argv[++i], NULL, 0);
            abs_set[0] = 1;
        } else if (!strcmp(argv[i], "--abs1") && i + 1 < argc) {
            abs_mode = 1;
            abs_pos[1] = (int32_t)strtol(argv[++i], NULL, 0);
            abs_set[1] = 1;
        } else if (!strcmp(argv[i], "--speed") && i + 1 < argc) {
            speed = (uint32_t)strtoul(argv[++i], NULL, 0);
            speed_set = 1;
            if (speed < 1)
                speed = 1;
        } else if (!strcmp(argv[i], "--seconds") && i + 1 < argc) {
            seconds = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "--fw0") && i + 1 < argc) {
            axes[0].fw = argv[++i];
        } else if (!strcmp(argv[i], "--fw1") && i + 1 < argc) {
            axes[1].fw = argv[++i];
        } else if (!strcmp(argv[i], "--pos0") && i + 1 < argc) {
            axes[0].position = (uint16_t)strtoul(argv[++i], NULL, 0);
        } else if (!strcmp(argv[i], "--pos1") && i + 1 < argc) {
            axes[1].position = (uint16_t)strtoul(argv[++i], NULL, 0);
        } else if (!strcmp(argv[i], "--invert0")) {
            axes[0].invert = 1;
        } else if (!strcmp(argv[i], "--no-invert0")) {
            axes[0].invert = 0;
        } else if (!strcmp(argv[i], "--invert1")) {
            axes[1].invert = 1;
        } else if (!strcmp(argv[i], "--no-invert1")) {
            axes[1].invert = 0;
        } else if (!strcmp(argv[i], "--no-dc")) {
            use_dc = 0;
        } else if (!strcmp(argv[i], "--dc")) {
            use_dc = 1;
        } else if (!strcmp(argv[i], "--twincat-dc")) {
            twincat_dc = 1;
        } else if (!strcmp(argv[i], "--sync0-cycle") && i + 1 < argc) {
            sync0_cycle_set = 1;
            sync0_cycle_ns = (uint32_t)strtoul(argv[++i], NULL, 0);
        } else if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) {
            fprintf(stderr,
                    "Usage: %s --hold | --delta N | --pos N | "
                    "--abs0 A --abs1 B\n"
                    "  [--fw0 8.1.50] [--fw1 8.1.60]\n"
                    "  [--pos0 0] [--pos1 1]  (EtherCAT station address)\n"
                    "  [--invert0|--no-invert0] [--invert1|--no-invert1]\n"
                    "  [--dc|--no-dc] [--twincat-dc] [--sync0-cycle N]\n"
                    "  --speed counts/s (required with --delta/--pos/--abs*)\n"
                    "  [--seconds N]\n"
                    "  Prefer: sudo chrt -f 80 %s --fw0 8.1.50 --fw1 8.1.44 "
                    "--invert0 --no-invert1 --delta 18192 "
                    "--speed 4000 --seconds 15\n"
                    "  soft-ramp: shared time profile @ --speed; "
                    "YES only when BOTH near\n",
                    argv[0], argv[0]);
            return EXIT_SUCCESS;
        }
    }

    if (!hold_mode && !delta_mode && !abs_mode) {
        fprintf(stderr, "need --hold or --delta N or --pos N or "
                "--abs0/--abs1\n");
        return EXIT_FAILURE;
    }
    if (delta_mode && abs_mode) {
        fprintf(stderr, "use only one of --delta / --pos|--abs*\n");
        return EXIT_FAILURE;
    }
    if (abs_mode && (!abs_set[0] || !abs_set[1])) {
        fprintf(stderr, "absolute mode needs both axes: --pos N or "
                "--abs0 A --abs1 B\n");
        return EXIT_FAILURE;
    }
    if ((delta_mode || abs_mode) && !speed_set) {
        fprintf(stderr, "need --speed counts/s with --delta/--pos/--abs*\n");
        return EXIT_FAILURE;
    }
    if (hold_mode && !speed_set)
        speed = 1000; /* hold only: soft settle floor */

    /* Lead = --speed (cmd soft-ramp rate). 0x6080 must stay HIGH —
     * speed*2 alone (e.g. 8000) can choke a soft plant to a crawl. */
    follow_lead = (int32_t)speed;
    if (follow_lead < 1)
        follow_lead = 1;

    vmax_6080 = speed * 2u;
    if (vmax_6080 < 100000u)
        vmax_6080 = 100000u;

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    drive_model = jsdk_drive_model_find(axes[0].fw, 0x00080117u);
    if (!drive_model)
        drive_model = jsdk_drive_model_default();

    jsdk_context_config_default(&ctx_config);
    ctx_config.master_index = 0;
    ctx_config.max_joints = N_AXES;
    jsdk_context_config_apply_drive_model(&ctx_config, drive_model);
    jsdk_dc_timing_conf_apply_fw(&ctx_config, axes[0].fw);
    {
        uint32_t conf_period = jsdk_dc_timing_conf_period_ns();

        if (conf_period)
            ctx_config.period_ns = conf_period;
    }
    if (!ctx_config.period_ns)
        ctx_config.period_ns = 1000000u;
    ctx_config.use_dc = use_dc;
    if (!use_dc) {
        ctx_config.sync0_cycle_ns = 0;
        ctx_config.wait_before_safeop_ms = 0;
    } else if (ctx_config.dc_timing_mode == JSDK_DC_TIMING_SHARED) {
        if (sync0_cycle_set) {
            ctx_config.sync0_cycle_ns = sync0_cycle_ns;
        } else if (!twincat_dc && !ctx_config.sync0_cycle_ns) {
            /* Default Sync0==task only when conf left Sync0 unset. */
            ctx_config.sync0_cycle_ns = ctx_config.period_ns;
        }
    } else if (sync0_cycle_set) {
        /* PER_JOINT: CLI sync0 only seeds context fallback. */
        ctx_config.sync0_cycle_ns = sync0_cycle_ns;
    }
    g_period_ns = ctx_config.period_ns;

    ctx = jsdk_context_create(&ctx_config);
    if (!ctx) {
        fprintf(stderr, "create context failed\n");
        return EXIT_FAILURE;
    }

    for (a = 0; a < N_AXES; a++) {
        jsdk_joint_config_t jc;
        const jsdk_drive_model_t *m;

        memset(&jc, 0, sizeof(jc));
        jc.alias = 0;
        jc.position = axes[a].position;
        jc.profile_name = JSDK_PROFILE_CYBERBEAST_JOINT_MODULE;
        jc.firmware_version = axes[a].fw;
        m = jsdk_drive_model_find(axes[a].fw, 0x00080117u);
        if (m)
            jc.drive_model_name = m->name;
        jc.initial_mode = JSDK_MODE_CSP;
        jc.max_motor_velocity = vmax_6080;
        jc.input_mode_2002 = 1; /* PASSTHROUGH */

        status = jsdk_context_add_joint(ctx, &jc, &axes[a].joint);
        if (status != JSDK_OK) {
            fprintf(stderr, "add_joint pos=%u: %s (%s)\n",
                    axes[a].position,
                    jsdk_status_string(status),
                    jsdk_context_last_error(ctx));
            jsdk_context_destroy(ctx);
            return EXIT_FAILURE;
        }
    }

    printf("dual CSP: ref_clock=joint0(pos=%u fw=%s)  "
           "joint1(pos=%u fw=%s)\n",
           axes[0].position, axes[0].fw,
           axes[1].position, axes[1].fw);
    printf("DC timing mode=%s (%d)\n",
           ctx_config.dc_timing_mode == JSDK_DC_TIMING_PER_JOINT
                   ? "PER_JOINT" : "SHARED",
           ctx_config.dc_timing_mode);
    printf("DC from fw0 model=%s use_dc=%d period=%u sync0=%u shift=%d "
           "wait_ms=%u%s\n",
           drive_model->name,
           ctx_config.use_dc,
           ctx_config.period_ns,
           ctx_config.sync0_cycle_ns ? ctx_config.sync0_cycle_ns
                                     : ctx_config.period_ns,
           (int)ctx_config.sync0_shift_ns,
           ctx_config.wait_before_safeop_ms,
           (!use_dc) ? " (FreeRun)"
                     : (ctx_config.dc_timing_mode ==
                                JSDK_DC_TIMING_PER_JOINT)
                               ? " (per-joint: see lines below)"
                     : (twincat_dc || sync0_cycle_set) ? ""
                                                       : " (sync0=task)");
    for (a = 0; a < N_AXES; a++)
        print_joint_dc_line(a, axes[a].fw, &ctx_config);
    printf("invert0=%s invert1=%s  ramp=%u  lead=%d  0x6080=%u\n",
           axes[0].invert ? "ON" : "OFF",
           axes[1].invert ? "ON" : "OFF",
           speed, follow_lead, vmax_6080);

    setup_realtime();

    status = jsdk_context_activate(ctx);
    if (status != JSDK_OK) {
        fprintf(stderr, "activate: %s (%s)\n",
                jsdk_status_string(status), jsdk_context_last_error(ctx));
        jsdk_context_destroy(ctx);
        return EXIT_FAILURE;
    }

    /* Bus settle: both axes OP+WC. */
    {
        unsigned int good = 0;
        unsigned int tries = 0;
        unsigned int period_ms = g_period_ns / 1000000u;
        unsigned int need_cycles;
        unsigned int max_cycles;

        if (period_ms == 0)
            period_ms = 1;
        need_cycles = 1000u / period_ms;
        if (need_cycles < 100)
            need_cycles = 100;
        max_cycles = 15000u / period_ms;

        clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
        add_period(&wakeup_time);
        printf("bus settle (need %u cycles both OP+WC)...\n", need_cycles);
        while (good < need_cycles && tries < max_cycles) {
            jsdk_bus_state_t bus;
            int all_ok = 1;

            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                    &wakeup_time, NULL);
            if (jsdk_context_cycle_begin(ctx, monotonic_time_ns()) != JSDK_OK)
                break;
            for (a = 0; a < N_AXES; a++) {
                jsdk_joint_feedback_t f;
                jsdk_joint_get_feedback(axes[a].joint, &f);
                jsdk_joint_set_target_position(axes[a].joint,
                        f.actual_position);
                jsdk_joint_set_target_velocity(axes[a].joint, 0);
                jsdk_joint_set_target_torque(axes[a].joint, 0);
                if (!f.online ||
                        ((f.al_state & 0x0f) != 0x08 && !f.operational))
                    all_ok = 0;
            }
            jsdk_context_get_bus_state(ctx, &bus);
            jsdk_context_cycle_end(ctx);
            add_period(&wakeup_time);
            tries++;
            if (all_ok && bus.working_counter > 0 && bus.wc_state == 2 &&
                    (bus.al_states & 0x08)) {
                good++;
            } else {
                good = 0;
            }
        }
        if (good < need_cycles) {
            fprintf(stderr,
                    "FAIL: bus never stable OP+WC (good=%u/%u tries=%u)\n",
                    good, need_cycles, tries);
            jsdk_context_destroy(ctx);
            return EXIT_FAILURE;
        }
        printf("bus settle ok (%u cycles)\n", good);
    }

    for (a = 0; a < N_AXES; a++)
        jsdk_joint_request_enable(axes[a].joint, JSDK_MODE_CSP);

    if (hold_mode)
        printf("HOLD both axes\n");
    else if (delta_mode)
        printf("RELATIVE both: home + (%d)\n", (int)delta);
    else
        printf("ABSOLUTE goals: j0=%d j1=%d\n",
               (int)abs_pos[0], (int)abs_pos[1]);
    if (seconds > 0)
        printf("auto-stop after %d s\n", seconds);

    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
    add_period(&wakeup_time);
    t0_ns = monotonic_time_ns();

    while (running) {
        uint64_t now;
        double t_s;
        double en_s = 0.0;
        int ret;
        jsdk_bus_state_t bus;
        jsdk_joint_feedback_t fb[N_AXES];
        int bus_ok;

        ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                &wakeup_time, NULL);
        if (ret) {
            if (ret == EINTR)
                interrupted = 1;
            running = 0;
            break;
        }

        now = monotonic_time_ns();
        t_s = (double)(now - t0_ns) / (double)NSEC_PER_SEC;

        status = jsdk_context_cycle_begin(ctx, now);
        if (status != JSDK_OK) {
            fprintf(stderr, "cycle_begin: %s\n", jsdk_status_string(status));
            break;
        }

        jsdk_context_get_bus_state(ctx, &bus);
        /*
         * Match bus-settle: require WC complete + OP present.
         * Do NOT require al_states==0x08 exactly — OR of slaves can be
         * 0x0A briefly while WC is still complete; that used to freeze
         * the trajectory forever and spam "ramp".
         */
        bus_ok = (bus.wc_state == 2 && bus.working_counter > 0 &&
                (bus.al_states & 0x08) != 0);

        for (a = 0; a < N_AXES; a++)
            jsdk_joint_get_feedback(axes[a].joint, &fb[a]);

        if (jsdk_joint_is_enabled(axes[0].joint) &&
                jsdk_joint_is_enabled(axes[1].joint)) {
            if (!enabled_seen) {
                enabled_seen = 1;
                en_t0 = now;
            }
            en_s = (double)(now - en_t0) / (double)NSEC_PER_SEC;

            if (!bus_ok) {
                bus_bad_cycles++;
                if (bus_bad_cycles >= BUS_BAD_FREEZE_CYCLES)
                    need_recover_ramp = 1;
                /* Brief hole: pause ramp, keep cmd (do not snap). */
                for (a = 0; a < N_AXES; a++) {
                    jsdk_joint_set_target_position(axes[a].joint,
                            csp_write_position(axes[a].cmd_pos,
                                    fb[a].actual_position,
                                    axes[a].invert));
                    jsdk_joint_set_target_velocity(axes[a].joint, 0);
                    jsdk_joint_set_target_torque(axes[a].joint, 0);
                }
            } else {
                if (bus_bad_cycles >= BUS_BAD_FREEZE_CYCLES) {
                    need_recover_ramp = 1;
                    recover_t0 = now;
                    /*
                     * Always realign both logical cmds. "keep cmd" after a
                     * WC/AL hole left invert j0 stuck (goal−act frozen) while
                     * j1 raced — restart lead-capped ramp from actual.
                     */
                    for (a = 0; a < N_AXES; a++) {
                        axes[a].stall_s = 0.0;
                        axes[a].cmd_pos = fb[a].actual_position;
                        axes[a].reached = 0;
                    }
                    printf("bus recovered — realign both cmd→actual, hold "
                           "%.2fs then resume\n", RECOVER_HOLD_SEC);
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

                    for (a = 0; a < N_AXES; a++) {
                        axis_t *ax = &axes[a];
                        jsdk_joint_feedback_t *f = &fb[a];

                        if (!ax->home_ok) {
                            ax->home = f->actual_position;
                            ax->home_ok = 1;
                            ax->cmd_pos = ax->home;
                            if (hold_mode)
                                ax->final_goal = ax->home;
                            else if (delta_mode)
                                ax->final_goal = ax->home + delta;
                            else
                                ax->final_goal = abs_pos[a];
                            ax->pos_err_ok = csp_pos_err_ok_for_travel(
                                    ax->final_goal - ax->home);
                            printf(">>> axis%u ENABLED home=%d goal=%d "
                                   "invert=%s (pace=slower-of-j0/j1, "
                                   "sync @%u, lead=%d YES|err|<=%d)\n",
                                   a, ax->home, ax->final_goal,
                                   ax->invert ? "ON" : "OFF",
                                   speed, follow_lead, ax->pos_err_ok);
                        }
                    }

                    if (traj_freeze) {
                        for (a = 0; a < N_AXES; a++) {
                            axis_t *ax = &axes[a];
                            jsdk_joint_feedback_t *f = &fb[a];

                            jsdk_joint_set_target_position(ax->joint,
                                    csp_write_position(ax->cmd_pos,
                                            f->actual_position,
                                            ax->invert));
                            jsdk_joint_set_target_velocity(ax->joint, 0);
                            jsdk_joint_set_target_torque(ax->joint, 0);
                        }
                    } else if (en_s < ENABLE_MIRROR_SEC) {
                        for (a = 0; a < N_AXES; a++) {
                            axis_t *ax = &axes[a];
                            jsdk_joint_feedback_t *f = &fb[a];

                            ax->cmd_pos = f->actual_position;
                            jsdk_joint_set_target_position(ax->joint,
                                    csp_write_position(ax->cmd_pos,
                                            f->actual_position,
                                            ax->invert));
                            jsdk_joint_set_target_velocity(ax->joint, 0);
                            jsdk_joint_set_target_torque(ax->joint, 0);
                        }
                    } else if (axes[0].home_ok && axes[1].home_ok) {
                        double move_s = en_s - ENABLE_MIRROR_SEC;
                        double frac_id;
                        double frac;
                        double frac_a0;
                        double frac_a1;
                        double mag;
                        double lead_frac;
                        int32_t travel0 = axes[0].final_goal - axes[0].home;
                        int32_t travel1 = axes[1].final_goal - axes[1].home;
                        int32_t travels = iabs32(travel0) >= iabs32(travel1)
                                ? travel0 : travel1;

                        /*
                         * Shared progress: frac = min(ideal, lim0, lim1).
                         * lim_i = actual_frac_i + lead — whichever plant is
                         * slower caps both cmds so neither races ahead.
                         */
                        mag = (double)iabs32(travels);
                        if (mag < 1.0)
                            mag = 1.0;
                        lead_frac = (double)follow_lead / mag;
                        if (lead_frac > 0.2)
                            lead_frac = 0.2;

                        frac_id = csp_frac_ideal(move_s, speed, travels);
                        frac_a0 = csp_frac_actual(axes[0].home, travel0,
                                fb[0].actual_position);
                        frac_a1 = csp_frac_actual(axes[1].home, travel1,
                                fb[1].actual_position);

                        frac = frac_id;
                        if (frac_a0 + lead_frac < frac)
                            frac = frac_a0 + lead_frac;
                        if (frac_a1 + lead_frac < frac)
                            frac = frac_a1 + lead_frac;

                        axes[0].cmd_pos = csp_cmd_from_frac(axes[0].home,
                                axes[0].final_goal, frac);
                        axes[1].cmd_pos = csp_cmd_from_frac(axes[1].home,
                                axes[1].final_goal, frac);

                        for (a = 0; a < N_AXES; a++) {
                            axis_t *ax = &axes[a];
                            jsdk_joint_feedback_t *f = &fb[a];

                            if (!hold_mode && en_s > 0.7 && en_s < 1.5) {
                                int32_t moved = f->actual_position -
                                        ax->home;
                                int32_t want = ax->final_goal - ax->home;

                                if (iabs32(moved) > 2000 && want != 0 &&
                                        ((moved > 0) != (want > 0))) {
                                    fprintf(stderr,
                                            "FAIL: axis%u motion opposite "
                                            "(moved=%d want=%d) — flip "
                                            "--invert%u / --no-invert%u\n",
                                            a, (int)moved, (int)want,
                                            a, a);
                                    ax->runaway = 1;
                                    running = 0;
                                }
                            }

                            jsdk_joint_set_target_position(ax->joint,
                                    csp_write_position(ax->cmd_pos,
                                            f->actual_position,
                                            ax->invert));
                            jsdk_joint_set_target_velocity(ax->joint, 0);
                            jsdk_joint_set_target_torque(ax->joint, 0);
                        }
                    }
                }
            }
        } else {
            for (a = 0; a < N_AXES; a++) {
                jsdk_joint_set_target_position(axes[a].joint,
                        fb[a].actual_position);
                jsdk_joint_set_target_velocity(axes[a].joint, 0);
                jsdk_joint_set_target_torque(axes[a].joint, 0);
            }
        }

        if (!counter--) {
            counter = NSEC_PER_SEC / g_period_ns;

            if (enabled_seen && en_s >= 1.0 && !bus_ok) {
                wc_bad_streak++;
                if (wc_bad_streak >= WC_DROP_DEBOUNCE)
                    wc_drop_secs++;
            } else {
                wc_bad_streak = 0;
            }

            if (print_header) {
                printf("%-6s %-8s %-10s %-10s %-8s %-5s %-8s "
                       "%4s %3s %3s %4s\n",
                       "axis", "sw", "goal", "actual", "err", "mode",
                       "vel", "wc", "slv", "al", "ok");
                print_header = 0;
            }

            {
                int all_near = 1;
                int any_home = 0;

                for (a = 0; a < N_AXES; a++) {
                    axis_t *ax = &axes[a];
                    jsdk_joint_feedback_t *f = &fb[a];
                    int32_t err = ax->home_ok
                            ? (ax->final_goal - f->actual_position) : 0;
                    int near = ax->home_ok
                            && iabs32(err) <= ax->pos_err_ok
                            && f->mode_display == 8;

                    ax->last_act = f->actual_position;
                    ax->last_err = err;
                    ax->last_vel = f->actual_velocity;
                    if (!ax->home_ok || !near)
                        all_near = 0;
                    if (ax->home_ok)
                        any_home = 1;
                }

                /*
                 * Latch / print YES only when BOTH axes are near — same
                 * sample. Early j0-only YES was the old per-axis latch.
                 */
                if (enabled_seen && en_s >= SETTLE_SEC && all_near &&
                        any_home && bus_ok && !need_recover_ramp) {
                    for (a = 0; a < N_AXES; a++)
                        axes[a].reached = 1;
                }

                for (a = 0; a < N_AXES; a++) {
                    axis_t *ax = &axes[a];
                    jsdk_joint_feedback_t *f = &fb[a];
                    int near = ax->home_ok
                            && iabs32(ax->last_err) <= ax->pos_err_ok
                            && f->mode_display == 8;
                    int bus_hole = !bus_ok &&
                            bus_bad_cycles >= BUS_BAD_FREEZE_CYCLES;
                    const char *tag = "ramp";

                    if (need_recover_ramp && bus_ok) {
                        tag = "rrcv";
                    } else if (bus_hole) {
                        tag = "hold";
                    } else if (!bus_ok) {
                        tag = "gap";
                    } else if (ax->reached && near) {
                        ax->settle_n++;
                        ax->ok_n++;
                        tag = "YES";
                    } else if (near && !all_near) {
                        /* Waiting for the other axis — keep tags aligned. */
                        tag = "wait";
                    } else if (ax->reached && !near) {
                        ax->settle_n++;
                        tag = "no";
                    } else {
                        tag = "ramp";
                    }

                    printf("j%-5u %04X     %-10d %-10d %-8d %-5d %-8d "
                           "%4u %3u %02X %4s\n",
                           a,
                           f->statusword,
                           ax->home_ok ? ax->final_goal : 0,
                           f->actual_position,
                           ax->last_err,
                           f->mode_display,
                           f->actual_velocity,
                           bus.working_counter,
                           bus.slaves_responding,
                           bus.al_states,
                           tag);
                }
            }

            if (seconds > 0 && t_s >= (double)seconds)
                running = 0;
        }

        status = jsdk_context_cycle_end(ctx);
        if (status != JSDK_OK) {
            fprintf(stderr, "cycle_end: %s\n", jsdk_status_string(status));
            break;
        }
        add_period(&wakeup_time);
    }

    printf("\n");
    axis_controlled_stop(ctx, axes, &wakeup_time);
    jsdk_context_destroy(ctx);

    for (a = 0; a < N_AXES; a++) {
        printf("summary j%u: goal=%d act=%d err=%d vel=%d ok=%d/%d "
               "hold=%d reached=%d runaway=%d\n",
               a,
               axes[a].home_ok ? axes[a].final_goal : 0,
               axes[a].last_act, axes[a].last_err, axes[a].last_vel,
               axes[a].ok_n, axes[a].settle_n, axes[a].hold_n,
               axes[a].reached, axes[a].runaway);
    }
    printf("wc_drops=%d enabled=%d\n", wc_drop_secs, enabled_seen);

    if (interrupted) {
        fprintf(stderr, "Interrupted — re-run without Ctrl+C for PASS.\n");
        return EXIT_FAILURE;
    }
    if (!enabled_seen) {
        fprintf(stderr, "FAIL: never enabled both axes.\n");
        return EXIT_FAILURE;
    }
    for (a = 0; a < N_AXES; a++) {
        int32_t dpos;
        int32_t want;
        int32_t need;
        int at_goal;
        int spinning;

        if (axes[a].runaway) {
            fprintf(stderr, "FAIL: axis%u runaway / invert check failed.\n",
                    a);
            return EXIT_FAILURE;
        }
        if (!axes[a].home_ok) {
            fprintf(stderr, "FAIL: axis%u never got home.\n", a);
            return EXIT_FAILURE;
        }

        dpos = axes[a].last_act - axes[a].home;
        want = axes[a].final_goal - axes[a].home;
        need = (int32_t)speed;
        if (need < 2000)
            need = 2000;
        at_goal = axes[a].reached && axes[a].ok_n >= MIN_TRACK_OK &&
                iabs32(axes[a].last_err) <= axes[a].pos_err_ok;
        /* Fast spin: moved toward goal by ~1s of --speed (need not arrive). */
        spinning = (want == 0 || ((dpos > 0) == (want > 0))) &&
                iabs32(dpos) >= need;

        if (at_goal)
            continue;
        if (spinning) {
            printf("note: axis%u spinning (dpos=%d need>=%d) — not yet "
                   "at goal (err=%d); OK per motion policy\n",
                   a, (int)dpos, (int)need, axes[a].last_err);
            continue;
        }
        if (wc_drop_secs > 0 || axes[a].hold_n > 0) {
            fprintf(stderr,
                    "FAIL: axis%u neither at goal nor spinning after "
                    "WC/AL hole (dpos=%d err=%d wc_drops=%d).\n",
                    a, (int)dpos, axes[a].last_err, wc_drop_secs);
        } else {
            fprintf(stderr,
                    "FAIL: axis%u neither near goal nor spinning "
                    "(dpos=%d need>=%d err=%d).\n",
                    a, (int)dpos, (int)need, axes[a].last_err);
        }
        return EXIT_FAILURE;
    }
    if (wc_drop_secs > WC_DROP_TOLERANCE) {
        int end_ok = 1;

        for (a = 0; a < N_AXES; a++) {
            if (axes[a].ok_n * 2 < axes[a].settle_n && !axes[a].reached)
                end_ok = 0;
        }
        if (!end_ok) {
            fprintf(stderr, "FAIL: %d sustained WC/AL dropout sample(s) "
                    "and tracking did not recover (tolerance %d).\n",
                    wc_drop_secs, WC_DROP_TOLERANCE);
            return EXIT_FAILURE;
        }
        printf("note: wc_drops=%d (within recover policy)\n", wc_drop_secs);
    }
    printf("PASS: dual CSP both axes moving/settled (soft-ramp @%u, "
           "0x6080=%u)\n", speed, vmax_6080);
    return EXIT_SUCCESS;
}
