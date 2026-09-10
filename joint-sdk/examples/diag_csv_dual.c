/*****************************************************************************
 *
 *  守护兽关节 SDK — 双轴 CSV 最小诊断（同一主站、同一 DC）
 *
 *  用法示例:
 *    sudo chrt -f 80 ./diag_csv_dual \
 *         --rps 0.5 --seconds 12
 *    sudo chrt -f 80 ./diag_csv_dual --hold --seconds 6
 *    sudo chrt -f 80 ./diag_csv_dual --vel 8192 --seconds 10
 *
 *  --pos0/--pos1: EtherCAT 站址（默认 0 / 1）；参考时钟 = 第 0 个 add_joint
 *  默认 fw0=8.1.60 / fw1=8.1.50；换线序改 --fwN
 *  负 --rps / --vel 即反转，不做软件极性反相
 *  DC 取自 --fw0；双轴默认 Sync0=task（IgH 上 TwinCAT ×3 易 sync 失败）
 *
 ****************************************************************************/

#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <math.h>
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
#define ENCODER_CPR 16384
#define VEL_CMD_TO_ACT 1.0
#define RAMP_SEC 1.5
#define RAMP_SEC_FAST 1.8  /* softer accel for |rps| >= 1.5 (was 2.5) */
#define HOLD_SETTLE_SEC 1.0
#define TRACK_TOL_FRAC 0.35
#define TRACK_TOL_ABS 2500
#define MIN_MOVE_ACT 500
#define WC_DROP_TOLERANCE 8
#define WC_DROP_DEBOUNCE 3
#define MIN_TRACK_OK 3
#define MIN_DPOS 500
/** Soft re-ramp after sustained WC hole (seconds). */
#define RECOVER_RAMP_SEC 0.8
/** App: consecutive bad-bus cycles before forcing vel=0 (~100 ms @ 2 ms). */
#define BUS_BAD_ZERO_CYCLES 50

static volatile sig_atomic_t running = 1;
static uint32_t g_period_ns = 1000000u;
static double g_ramp_sec = RAMP_SEC;

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

static int32_t soft_ramp(int32_t target, double elapsed_s)
{
    double a;

    if (elapsed_s >= g_ramp_sec)
        return target;
    if (elapsed_s <= 0.0)
        return 0;
    a = elapsed_s / g_ramp_sec;
    return (int32_t)lround((double)target * a);
}

static int velocity_tracks(int32_t tgt, int32_t act)
{
    int32_t err = act - tgt;
    int32_t atol;
    int32_t atgt = iabs32(tgt);
    int32_t aact = iabs32(act);

    err = iabs32(err);
    atol = (int32_t)(TRACK_TOL_FRAC * (double)atgt);
    if (atol < TRACK_TOL_ABS)
        atol = TRACK_TOL_ABS;
    if (atgt >= MIN_MOVE_ACT && aact < MIN_MOVE_ACT)
        return 0;
    return err <= atol;
}

typedef struct {
    jsdk_joint_t *joint;
    uint16_t position;
    const char *fw;
    int32_t vel_cmd;
    int32_t last_tgt;
    int32_t last_act;
    int32_t pos_at_enable;
    int32_t last_pos;
    int home_ok;
    int reached;
    int ok_n;
    int settle_n;
    int hold_n;
} axis_t;

static void axis_controlled_stop(jsdk_context_t *ctx, axis_t *axes,
        struct timespec *wakeup_time)
{
    unsigned int n_zero = 600000000u / g_period_ns;
    unsigned int n_dis = 600000000u / g_period_ns;
    unsigned int i, a;

    if (n_zero < 50)
        n_zero = 50;
    if (n_dis < 50)
        n_dis = 50;

    printf("Stopping: zero velocity (%.1f s)...\n",
           (double)n_zero * g_period_ns / 1e9);
    for (i = 0; i < n_zero; i++) {
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, wakeup_time, NULL);
        if (jsdk_context_cycle_begin(ctx, monotonic_time_ns()) != JSDK_OK)
            break;
        for (a = 0; a < N_AXES; a++) {
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
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, wakeup_time, NULL);
        if (jsdk_context_cycle_begin(ctx, monotonic_time_ns()) != JSDK_OK)
            break;
        for (a = 0; a < N_AXES; a++) {
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
    int fixed_vel_mode = 0;
    int32_t fixed_vel = 0;
    int32_t expect_act = 0;
    double rps = 0.0;
    int seconds = 0; /* 0 = run until Ctrl+C */
    int wc_drop_secs = 0;
    int wc_bad_streak = 0;
    int bus_bad_cycles = 0;
    int need_recover_ramp = 0;
    uint64_t recover_t0 = 0;
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
    const jsdk_drive_model_t *drive_model;

    memset(axes, 0, sizeof(axes));
    axes[0].position = 0;
    axes[1].position = 1;
    axes[0].fw = "8.1.60";
    axes[1].fw = "8.1.50";

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--hold")) {
            hold_mode = 1;
        } else if (!strcmp(argv[i], "--vel") && i + 1 < argc) {
            fixed_vel_mode = 1;
            fixed_vel = (int32_t)strtol(argv[++i], NULL, 0);
            expect_act = fixed_vel;
            rps = 0.0;
        } else if (!strcmp(argv[i], "--rps") && i + 1 < argc) {
            fixed_vel_mode = 1;
            rps = strtod(argv[++i], NULL);
            expect_act = (int32_t)lround(rps * (double)ENCODER_CPR);
            fixed_vel = (int32_t)lround((double)expect_act / VEL_CMD_TO_ACT);
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
                    "Usage: %s --hold | --vel N | --rps F\n"
                    "  [--fw0 8.1.60] [--fw1 8.1.50]\n"
                    "  [--pos0 0] [--pos1 1]\n"
                    "  [--dc|--no-dc] [--twincat-dc] [--sync0-cycle N]\n"
                    "  [--seconds N]\n"
                    "  Negative --rps/--vel reverses rotation.\n"
                    "  Prefer: sudo chrt -f 80 %s --rps 0.5 --seconds 12\n",
                    argv[0], argv[0]);
            return EXIT_SUCCESS;
        }
    }

    if (!hold_mode && !fixed_vel_mode) {
        fprintf(stderr, "need --hold or --vel N or --rps F\n");
        return EXIT_FAILURE;
    }

    if (!hold_mode && fabs(rps) >= 1.5)
        g_ramp_sec = RAMP_SEC_FAST;
    else
        g_ramp_sec = RAMP_SEC;

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
            /* Keep conf Sync0 when set; else default Sync0==task. */
            ctx_config.sync0_cycle_ns = ctx_config.period_ns;
        }
    } else if (sync0_cycle_set) {
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
        jc.initial_mode = JSDK_MODE_CSV;
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

    printf("dual CSV: ref_clock=joint0(pos=%u fw=%s)  "
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
        jsdk_joint_request_enable(axes[a].joint, JSDK_MODE_CSV);

    if (hold_mode)
        printf("HOLD both axes (vel=0)\n");
    else if (rps != 0.0)
        printf("BOTH: --rps %.3f → 0x60FF=%d expect_act≈%d  ramp %.1fs\n",
               rps, (int)fixed_vel, (int)expect_act, g_ramp_sec);
    else
        printf("BOTH: --vel %d counts/s  ramp %.1fs\n",
               (int)fixed_vel, g_ramp_sec);
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
        int32_t base_cmd = 0;
        int32_t track_tgt = 0;

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
        bus_ok = (bus.wc_state == 2 && bus.working_counter > 0 &&
                (bus.al_states & 0x0f) == 0x08);

        for (a = 0; a < N_AXES; a++)
            jsdk_joint_get_feedback(axes[a].joint, &fb[a]);

        if (jsdk_joint_is_enabled(axes[0].joint) &&
                jsdk_joint_is_enabled(axes[1].joint)) {
            double recover_scale = 1.0;

            if (!enabled_seen) {
                enabled_seen = 1;
                en_t0 = now;
            }
            en_s = (double)(now - en_t0) / (double)NSEC_PER_SEC;

            if (!bus_ok) {
                bus_bad_cycles++;
                if (bus_bad_cycles >= BUS_BAD_ZERO_CYCLES)
                    need_recover_ramp = 1;
            } else {
                if (bus_bad_cycles >= BUS_BAD_ZERO_CYCLES) {
                    need_recover_ramp = 1;
                    recover_t0 = now;
                    printf("bus recovered — soft-ramp velocity again\n");
                }
                bus_bad_cycles = 0;
            }

            if (need_recover_ramp && bus_ok && recover_t0) {
                double rs = (double)(now - recover_t0) / (double)NSEC_PER_SEC;
                if (rs >= RECOVER_RAMP_SEC) {
                    need_recover_ramp = 0;
                    recover_scale = 1.0;
                } else if (rs <= 0.0) {
                    recover_scale = 0.0;
                } else {
                    recover_scale = rs / RECOVER_RAMP_SEC;
                }
            }

            if (hold_mode)
                base_cmd = 0;
            else
                base_cmd = (int32_t)lround(
                        (double)soft_ramp(fixed_vel, en_s) * recover_scale);
            track_tgt = hold_mode ? 0
                    : (int32_t)lround(
                            (double)soft_ramp(expect_act, en_s) *
                            recover_scale);

            for (a = 0; a < N_AXES; a++) {
                axis_t *ax = &axes[a];
                jsdk_joint_feedback_t *f = &fb[a];
                int32_t cmd;

                if (!ax->home_ok) {
                    ax->home_ok = 1;
                    ax->pos_at_enable = f->actual_position;
                    printf(">>> axis%u ENABLED pos=%d\n",
                           a, f->actual_position);
                }
                ax->last_pos = f->actual_position;

                /*
                 * Brief WC holes: keep commanding last/desired velocity
                 * (SDK also debounces). Only hard-zero after sustained loss.
                 */
                if (!bus_ok && bus_bad_cycles >= BUS_BAD_ZERO_CYCLES) {
                    ax->vel_cmd = 0;
                    jsdk_joint_set_target_velocity(ax->joint, 0);
                    jsdk_joint_set_target_torque(ax->joint, 0);
                    continue;
                }

                cmd = base_cmd;
                ax->vel_cmd = cmd;
                ax->last_tgt = cmd;
                ax->last_act = f->actual_velocity;
                jsdk_joint_set_target_velocity(ax->joint, cmd);
                jsdk_joint_set_target_torque(ax->joint, 0);
            }
        } else {
            for (a = 0; a < N_AXES; a++) {
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
                printf("%-6s %-8s %-8s %-8s %-8s %-8s %-5s "
                       "%4s %3s %3s %4s\n",
                       "axis", "sw", "tgt", "act_v", "err", "pos", "mode",
                       "wc", "slv", "al", "ok");
                print_header = 0;
            }

            for (a = 0; a < N_AXES; a++) {
                axis_t *ax = &axes[a];
                jsdk_joint_feedback_t *f = &fb[a];
                int32_t expect_signed = track_tgt;
                int32_t err = f->actual_velocity - expect_signed;
                /*
                 * Track against the *current* soft-ramp target — do not wait
                 * for full g_ramp_sec before YES (that spammed "ramp").
                 */
                int settled = enabled_seen && !need_recover_ramp;
                int bus_hole = !bus_ok &&
                        bus_bad_cycles >= BUS_BAD_ZERO_CYCLES;
                int track;
                const char *tag = "ramp";

                if (hold_mode)
                    settled = enabled_seen &&
                            en_s >= HOLD_SETTLE_SEC &&
                            !need_recover_ramp;

                if (hold_mode) {
                    track = settled && bus_ok && f->mode_display == 9 &&
                            iabs32(f->actual_velocity) <= TRACK_TOL_ABS;
                } else {
                    track = settled && bus_ok && f->mode_display == 9 &&
                            velocity_tracks(expect_signed,
                                    f->actual_velocity);
                }

                if (settled && track)
                    ax->reached = 1;

                if (!ax->reached) {
                    tag = "ramp";
                } else if (need_recover_ramp) {
                    /* Soft re-accel after sustained hole — not initial ramp. */
                    tag = "rrcv";
                } else if (bus_hole) {
                    ax->settle_n++;
                    ax->hold_n++;
                    tag = "hold";
                } else if (!bus_ok) {
                    /* Brief hole while still commanding. */
                    tag = "gap";
                } else if (track) {
                    ax->settle_n++;
                    ax->ok_n++;
                    tag = "YES";
                } else {
                    ax->settle_n++;
                    tag = "no";
                }

                ax->last_tgt = ax->vel_cmd;
                ax->last_act = f->actual_velocity;
                ax->last_pos = f->actual_position;

                printf("j%-5u %04X     %-8d %-8d %-8d %-8d %-5d "
                       "%4u %3u %02X %4s\n",
                       a,
                       f->statusword,
                       ax->vel_cmd,
                       f->actual_velocity,
                       err,
                       f->actual_position,
                       f->mode_display,
                       bus.working_counter,
                       bus.slaves_responding,
                       bus.al_states,
                       tag);
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
        int32_t dpos = axes[a].last_pos - axes[a].pos_at_enable;
        printf("summary j%u: tgt=%d act_v=%d ok=%d/%d hold=%d "
               "reached=%d dpos=%d\n",
               a, axes[a].last_tgt, axes[a].last_act,
               axes[a].ok_n, axes[a].settle_n, axes[a].hold_n,
               axes[a].reached, (int)dpos);
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
    if (hold_mode) {
        for (a = 0; a < N_AXES; a++) {
            if (!axes[a].reached || axes[a].ok_n < MIN_TRACK_OK) {
                fprintf(stderr, "FAIL: axis%u hold not stable.\n", a);
                return EXIT_FAILURE;
            }
        }
        if (wc_drop_secs > WC_DROP_TOLERANCE) {
            fprintf(stderr, "FAIL: %d sustained WC/AL dropout sample(s) "
                    "(tolerance %d).\n",
                    wc_drop_secs, WC_DROP_TOLERANCE);
            return EXIT_FAILURE;
        }
        printf("PASS: dual CSV hold stable OP+WC\n");
        return EXIT_SUCCESS;
    }
    for (a = 0; a < N_AXES; a++) {
        int32_t dpos = iabs32(axes[a].last_pos - axes[a].pos_at_enable);
        if (!axes[a].reached || axes[a].ok_n < MIN_TRACK_OK) {
            if (wc_drop_secs > 0 || axes[a].hold_n > 0) {
                fprintf(stderr,
                        "FAIL: axis%u lost velocity tracking after WC/AL "
                        "hole (ok=%d/%d hold=%d wc_drops=%d).\n",
                        a, axes[a].ok_n, axes[a].settle_n,
                        axes[a].hold_n, wc_drop_secs);
            } else {
                fprintf(stderr,
                        "FAIL: axis%u velocity not tracking "
                        "(ok=%d/%d).\n",
                        a, axes[a].ok_n, axes[a].settle_n);
            }
            return EXIT_FAILURE;
        }
        if (dpos < MIN_DPOS) {
            fprintf(stderr,
                    "FAIL: axis%u act_pos barely changed (dpos=%d) — "
                    "stuck feedback?\n",
                    a, (int)dpos);
            return EXIT_FAILURE;
        }
    }
    /* High-speed dual: allow brief WC holes if tracking recovered. */
    if (wc_drop_secs > WC_DROP_TOLERANCE) {
        int end_ok = 1;
        for (a = 0; a < N_AXES; a++) {
            if (axes[a].ok_n * 2 < axes[a].settle_n)
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
    printf("PASS: dual CSV both axes velocity tracking\n");
    return EXIT_SUCCESS;
}
