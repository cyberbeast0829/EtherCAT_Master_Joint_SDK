/*****************************************************************************
 *
 *  守护兽关节 SDK CSV 诊断示例
 *
 *  现场结论（ISVD90RC）：
 *    - 0x60FF 与 0x606C 同为「指令单位/s」（≈ encoder counts/s）
 *    - 勿把 50000 当成 turns/s；约 1 rps ≈ 16384
 *    - CSV 下 0x2002:1 用 PASSTHROUGH(1)（SDK 默认）；周期性发速度用不到 VEL_RAMP
 *
 *  用法（DC 建议用 chrt 减掉线）:
 *    sudo chrt -f 80 ./diag_csv_single --fw 8.1.50 --rps 0.5 --seconds 20
 *    ./diag_csv_single --hold
 *    ./diag_csv_single --fw 8.1.50 --rps 0.5 --seconds 20
 *    ./diag_csv_single --vel 8192 --seconds 8
 *
 ****************************************************************************/

#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sched.h>
#include <sys/mman.h>

#include <joint_sdk/dc_timing_conf.h>
#include <joint_sdk/joint_sdk.h>

#define PERIOD_NS_DEFAULT 1000000u
#define NSEC_PER_SEC 1000000000L
#define MAX_SAFE_STACK (8 * 1024)
#define VEL_AMPLITUDE 32768.0
#define VEL_FREQ_HZ 0.25
#define PI 3.14159265358979323846
#define ENCODER_CPR 16384
/*
 * 旧板标定：act ≈ cmd * 20.5。新控制板实测写 799 时 act≈0、trq=0，
 * 先按 counts/s 1:1 发指令；旧公式保留备查。
 */
/* #define VEL_CMD_TO_ACT 20.5 */
#define VEL_CMD_TO_ACT 1.0
#define RAMP_SEC 1.5
#define TRACK_TOL_FRAC 0.35
/* #define TRACK_TOL_ABS  12000 */
#define TRACK_TOL_ABS  2500
#define MIN_MOVE_ACT   500

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
    /* Prefer 80 (same as: sudo chrt -f 80) for Sync0 stability. */
    param.sched_priority = 80;
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        fprintf(stderr,
                "warning: SCHED_FIFO 80 unavailable (%s); "
                "use: sudo chrt -f 80 ./build/diag_csv_single ...\n",
                strerror(errno));
    } else {
        printf("sched=SCHED_FIFO priority=80\n");
    }
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
        fprintf(stderr, "warning: mlockall failed: %s\n", strerror(errno));
    stack_prefault();
}

static int velocity_tracks(int32_t tgt, int32_t act)
{
    int32_t err = act - tgt;
    int32_t atol;
    int32_t atgt = tgt < 0 ? -tgt : tgt;
    int32_t aact = act < 0 ? -act : act;

    if (err < 0)
        err = -err;
    atol = (int32_t)(TRACK_TOL_FRAC * (double)atgt);
    if (atol < TRACK_TOL_ABS)
        atol = TRACK_TOL_ABS;
    if (atgt >= MIN_MOVE_ACT && aact < MIN_MOVE_ACT)
        return 0;
    return err <= atol;
}

static int32_t soft_ramp(int32_t target, double elapsed_s)
{
    double a;

    if (elapsed_s >= RAMP_SEC)
        return target;
    if (elapsed_s <= 0.0)
        return 0;
    a = elapsed_s / RAMP_SEC;
    return (int32_t)lround((double)target * a);
}

int main(int argc, char **argv)
{
    jsdk_context_config_t ctx_config;
    jsdk_joint_config_t joint_config;
    jsdk_context_t *ctx;
    jsdk_joint_t *joint = NULL;
    jsdk_status_t status;
    struct timespec wakeup_time;
    double traj_time = 0.0;
    unsigned int counter = 0;
    int ret = 0;
    int hold_mode = 0;
    int fixed_vel_mode = 0;
    int invert = 0;
    int turns_mode = 0; /* 1 = 0x60FF 直接写 turns/s 整数 */
    int32_t fixed_vel = 0;
    int32_t expect_act = 0; /* 用于判定的期望 act（counts/s） */
    double rps = 0.0;
    int print_header = 1;
    int seconds = 0;
    const char *log_path = NULL;
    FILE *log_fp = NULL;
    uint64_t t0_ns = 0;
    uint64_t enabled_t0_ns = 0;
    int enabled_seen = 0;
    int track_ok_samples = 0;
    int track_samples = 0;
    int32_t last_tgt = 0;
    int32_t last_act = 0;
    int32_t pos_at_enable = 0;
    int32_t last_pos = 0;
    int input_mode_override = -1;
    uint32_t vmax = 0;
    int use_dc = -1; /* -1 = from drive_model */
    int sync0_shift_set = 0;
    int32_t sync0_shift_ns = 0;
    int sync0_cycle_set = 0;
    uint32_t sync0_cycle_ns = 0;
    const char *fw_version = "8.1.50";
    const char *model_name = NULL;
    const jsdk_drive_model_t *drive_model = NULL;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--hold")) {
            hold_mode = 1;
        } else if (!strcmp(argv[i], "--invert")) {
            invert = 1;
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
        } else if (!strcmp(argv[i], "--fw") && i + 1 < argc) {
            fw_version = argv[++i];
        } else if (!strcmp(argv[i], "--model") && i + 1 < argc) {
            model_name = argv[++i];
        } else if (!strcmp(argv[i], "--vel") && i + 1 < argc) {
            fixed_vel_mode = 1;
            turns_mode = 0;
            fixed_vel = (int32_t)strtol(argv[++i], NULL, 0);
            expect_act = fixed_vel;
            rps = 0.0;
        } else if (!strcmp(argv[i], "--rps") && i + 1 < argc) {
            fixed_vel_mode = 1;
            turns_mode = 0;
            rps = atof(argv[++i]);
            expect_act = (int32_t)lround(rps * (double)ENCODER_CPR);
            fixed_vel = (int32_t)lround((double)expect_act / VEL_CMD_TO_ACT);
            if (fixed_vel == 0 && rps != 0.0)
                fixed_vel = (rps > 0.0) ? 1 : -1;
        } else if (!strcmp(argv[i], "--turns") && i + 1 < argc) {
            fixed_vel_mode = 1;
            turns_mode = 1;
            rps = atof(argv[++i]);
            fixed_vel = (int32_t)lround(rps);
            expect_act = (int32_t)lround(rps * (double)ENCODER_CPR);
        } else if (!strcmp(argv[i], "--input-mode") && i + 1 < argc) {
            input_mode_override = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "--vmax") && i + 1 < argc) {
            vmax = (uint32_t)strtoul(argv[++i], NULL, 0);
        } else if (!strcmp(argv[i], "--seconds") && i + 1 < argc) {
            seconds = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "--log") && i + 1 < argc) {
            log_path = argv[++i];
        } else if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) {
            fprintf(stderr,
                    "Usage: %s [--hold] [--vel counts/s] [--rps turns]\n"
                    "          [--fw 8.1.50|--model ISVD90RC-v8.1.50]\n"
                    "          [--dc|--no-dc] [--sync0-shift N]\n"
                    "          [--invert] [--vmax N] [--seconds N] [--log f]\n"
                    "  Default: FW 8.1.50 drive_model DC table.\n",
                    argv[0]);
            return EXIT_SUCCESS;
        }
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    jsdk_context_config_default(&ctx_config);
    ctx_config.master_index = 0;
    ctx_config.period_ns = PERIOD_NS_DEFAULT;
    ctx_config.max_joints = 1;
    /* Load conf before find() so conf-only [fw] becomes a drive_model. */
    (void)jsdk_dc_timing_conf_apply(&ctx_config);

    if (model_name)
        drive_model = jsdk_drive_model_by_name(model_name);
    if (!drive_model)
        drive_model = jsdk_drive_model_find(fw_version, 0x00080117u);
    if (!drive_model)
        drive_model = jsdk_drive_model_default();

    jsdk_context_config_apply_drive_model(&ctx_config, drive_model);
    /* PER_JOINT [fw] Sync0/shift/wait; task period stays conf global. */
    (void)jsdk_dc_timing_conf_apply_fw(&ctx_config, fw_version);
    {
        uint32_t conf_period = jsdk_dc_timing_conf_period_ns();

        if (conf_period)
            ctx_config.period_ns = conf_period;
    }
    /* CLI overrides after model + conf */
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
            ctx_config.sync0_cycle_ns = ctx_config.period_ns;
    } else if (sync0_cycle_set) {
        ctx_config.sync0_cycle_ns = sync0_cycle_ns;
    }
    if (sync0_shift_set)
        ctx_config.sync0_shift_ns = sync0_shift_ns;
    if (!ctx_config.period_ns)
        ctx_config.period_ns = PERIOD_NS_DEFAULT;
    g_period_ns = ctx_config.period_ns;

    ctx = jsdk_context_create(&ctx_config);
    if (!ctx) {
        fprintf(stderr, "failed to create SDK context\n");
        return EXIT_FAILURE;
    }

    memset(&joint_config, 0, sizeof(joint_config));
    joint_config.alias = 0;
    joint_config.position = 0;
    joint_config.profile_name = JSDK_PROFILE_CYBERBEAST_JOINT_MODULE;
    joint_config.initial_mode = JSDK_MODE_CSV;
    joint_config.firmware_version = fw_version;
    /* Prefer --fw resolution in transport; avoid pinning freerun by name. */
    if (drive_model->fw_match && drive_model->fw_match[0])
        joint_config.drive_model_name = drive_model->name;
    if (invert)
        joint_config.polarity_607e = 0x40;
    if (vmax)
        joint_config.max_motor_velocity = vmax;

    status = jsdk_context_add_joint(ctx, &joint_config, &joint);
    if (status != JSDK_OK) {
        fprintf(stderr, "add_joint: %s (%s)\n",
                jsdk_status_string(status), jsdk_context_last_error(ctx));
        jsdk_context_destroy(ctx);
        return EXIT_FAILURE;
    }

    (void)input_mode_override;

    printf("drive_model=%s fw=%s use_dc=%d period=%u sync0=%u "
           "shift=%d wait_ms=%u mode=%s\n",
           drive_model->name,
           fw_version,
           ctx_config.use_dc,
           ctx_config.period_ns,
           ctx_config.sync0_cycle_ns ? ctx_config.sync0_cycle_ns
                                     : ctx_config.period_ns,
           (int)ctx_config.sync0_shift_ns,
           ctx_config.wait_before_safeop_ms,
           ctx_config.dc_timing_mode == JSDK_DC_TIMING_PER_JOINT
                   ? "per_joint" : "shared");

    /* RT before activate so SAFEOP→OP bootstrap is not preempted. */
    setup_realtime();

    status = jsdk_context_activate(ctx);
    if (status != JSDK_OK) {
        fprintf(stderr, "activate: %s (%s)\n",
                jsdk_status_string(status), jsdk_context_last_error(ctx));
        jsdk_context_destroy(ctx);
        return EXIT_FAILURE;
    }

    if (input_mode_override >= 0) {
        fprintf(stderr,
                "note: --input-mode=%d requested; SDK CSV default is "
                "PASSTHROUGH(1). To force another value, run before activate:\n"
                "  ethercat download -p0 0x2002 1 %d\n",
                input_mode_override, input_mode_override);
    }

    /*
     * Exchange PDOs (vel=0) until OP+WC stay good for settle_ms, then enable.
     * Never enable from SAFEOP — that is what caused early wc=0 bursts on
     * motor-app 8.1.50.
     */
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
        max_cycles = 15000u / period_ms; /* up to ~15 s */

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
            jsdk_joint_set_target_velocity(joint, 0);
            jsdk_joint_get_feedback(joint, &f);
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
                    "(good=%u/%u tries=%u). "
                    "Check: sudo dmesg | grep -i 001A\n",
                    good, need_cycles, tries);
            jsdk_context_destroy(ctx);
            return EXIT_FAILURE;
        }
        printf("bus settle ok (%u cycles stable OP+WC)\n", good);
    }

    jsdk_joint_request_enable(joint, JSDK_MODE_CSV);

    if (log_path) {
        log_fp = fopen(log_path, "w");
        if (!log_fp) {
            fprintf(stderr, "fopen %s: %s\n", log_path, strerror(errno));
            jsdk_context_destroy(ctx);
            return EXIT_FAILURE;
        }
        fprintf(log_fp,
                "t_s,status,sw,tgt_vel,act_vel,err,act_pos,mode,trq,"
                "wc,slv,al,track\n");
    }

    printf("=== Diagnostic CSV Example ===\n");
    printf("PREOP: mode=CSV(9) input_mode=PASSTHROUGH(1) polarity=%s "
           "vmax=%s  DC=%s",
           invert ? "0x40" : "unchanged",
           vmax ? "set" : "slave-default",
           ctx_config.use_dc ? "ON(Sync0)" : "OFF(free-run)");
    if (ctx_config.use_dc) {
        uint32_t s0 = ctx_config.sync0_cycle_ns
                ? ctx_config.sync0_cycle_ns : ctx_config.period_ns;
        printf(" task=%u sync0=%u shift=%d",
               ctx_config.period_ns, s0, (int)ctx_config.sync0_shift_ns);
    }
    printf("\n");
    if (fixed_vel_mode) {
        if (turns_mode) {
            printf("period=%u ns  --turns %.3f → 0x60FF=%d  "
                   "expect_act≈%d  soft-ramp %.1fs\n",
                   g_period_ns, rps, (int)fixed_vel, (int)expect_act, RAMP_SEC);
        } else if (rps != 0.0) {
            printf("period=%u ns  --rps %.3f → 0x60FF=%d "
                   "(expect_act≈%d, scale=%.1f)  soft-ramp %.1fs\n",
                   g_period_ns, rps, (int)fixed_vel, (int)expect_act,
                   VEL_CMD_TO_ACT, RAMP_SEC);
        } else {
            printf("period=%u ns  --vel %d counts/s  soft-ramp %.1fs\n",
                   g_period_ns, (int)fixed_vel, RAMP_SEC);
        }
    } else {
        printf("period=%u ns  hold=%s  vel_amplitude=%.0f\n",
                g_period_ns, hold_mode ? "YES" : "NO", VEL_AMPLITUDE);
    }
    if (seconds > 0)
        printf("auto-stop after %d s\n", seconds);

    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
    add_period(&wakeup_time);
    t0_ns = monotonic_time_ns();

    while (running) {
        jsdk_joint_feedback_t f;
        int32_t vel_cmd = 0;
        uint64_t now;
        double t_s;
        double enabled_s = 0.0;

        ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                &wakeup_time, NULL);
        if (ret) {
            fprintf(stderr, "clock_nanosleep: %s\n", strerror(ret));
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

        if (jsdk_joint_is_enabled(joint)) {
            if (!enabled_seen) {
                enabled_seen = 1;
                enabled_t0_ns = now;
                pos_at_enable = f.actual_position;
            }
            enabled_s = (double)(now - enabled_t0_ns) / (double)NSEC_PER_SEC;
            last_pos = f.actual_position;

            if (hold_mode) {
                vel_cmd = 0;
            } else if (fixed_vel_mode) {
                vel_cmd = soft_ramp(fixed_vel, enabled_s);
            } else {
                double dt = (double)g_period_ns / (double)NSEC_PER_SEC;
                traj_time += dt;
                vel_cmd = (int32_t)(VEL_AMPLITUDE *
                        sin(2.0 * PI * VEL_FREQ_HZ * traj_time));
            }
            jsdk_joint_set_target_velocity(joint, vel_cmd);
        } else {
            jsdk_joint_set_target_velocity(joint, 0);
        }

        if (!counter--) {
            jsdk_bus_state_t bus;
            int32_t err;
            int track;
            int settled;

            counter = NSEC_PER_SEC / g_period_ns;
            jsdk_context_get_bus_state(ctx, &bus);
            err = f.actual_velocity - ((rps != 0.0 || turns_mode)
                    ? soft_ramp(expect_act, enabled_s) : vel_cmd);
            settled = enabled_seen && (!fixed_vel_mode ||
                    enabled_s >= RAMP_SEC);
            {
                int32_t track_tgt = (rps != 0.0 || turns_mode)
                        ? soft_ramp(expect_act, enabled_s) : vel_cmd;
                track = settled && velocity_tracks(track_tgt,
                        f.actual_velocity);
            }

            if (settled && !hold_mode) {
                track_samples++;
                if (track)
                    track_ok_samples++;
            }
            last_tgt = vel_cmd;
            last_act = f.actual_velocity;

            if (print_header) {
                printf("%-8s %-8s %-8s %-8s %-8s %-8s "
                       "%-6s %-4s %4s %5s %5s %5s\n",
                       "axis", "status", "tgt_vel", "act_vel",
                       "err", "act_pos", "mode", "trq",
                       "wc", "slv", "al", "ok");
                print_header = 0;
            }

            /*
             * ok column:
             *   YES  — settled + tracking
             *   no   — settled but not tracking
             *   ramp — still in soft-ramp window
             *   drop — was enabled, but this sample lost enable
             *          (AL/OP debounce); not a velocity ramp
             */
            {
                const char *ok_s;

                if (track)
                    ok_s = "YES";
                else if (settled)
                    ok_s = "no";
                else if (enabled_seen &&
                        !jsdk_joint_is_enabled(joint))
                    ok_s = "drop";
                else
                    ok_s = "ramp";

                printf("%-8s %04X     %-8d %-8d %-8d %-8d "
                       "%-6d %-4d %4u %4u %02X %5s\n",
                       jsdk_axis_state_string(f.axis_state),
                       f.statusword,
                       vel_cmd,
                       f.actual_velocity,
                       err,
                       f.actual_position,
                       f.mode_display,
                       f.actual_torque,
                       bus.working_counter,
                       bus.slaves_responding,
                       bus.al_states,
                       ok_s);
            }

            if (log_fp) {
                fprintf(log_fp,
                        "%.3f,%s,%04X,%d,%d,%d,%d,%d,%d,%u,%u,%02X,%d\n",
                        t_s,
                        jsdk_axis_state_string(f.axis_state),
                        f.statusword,
                        vel_cmd,
                        f.actual_velocity,
                        err,
                        f.actual_position,
                        f.mode_display,
                        f.actual_torque,
                        bus.working_counter,
                        bus.slaves_responding,
                        bus.al_states,
                        track);
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

    printf("\nStopping...\n");
    jsdk_joint_request_disable(joint);
    jsdk_context_cycle_begin(ctx, monotonic_time_ns());
    jsdk_context_cycle_end(ctx);
    jsdk_context_destroy(ctx);
    if (log_fp)
        fclose(log_fp);

    printf("summary: tgt=%d act=%d track_ok=%d/%d enabled=%d\n",
           (int)last_tgt, (int)last_act,
           track_ok_samples, track_samples, enabled_seen);

    if (hold_mode)
        return ret ? EXIT_FAILURE : EXIT_SUCCESS;

    if (!enabled_seen || track_samples < 2 ||
            track_ok_samples * 2 < track_samples) {
        fprintf(stderr,
                "FAIL: velocity not tracking. "
                "Try --invert, lower --rps, or check AL drops (al!=08).\n");
        return EXIT_FAILURE;
    }

    /* Reject false PASS when encoder/vel feedback is frozen. */
    {
        int32_t dpos = last_pos - pos_at_enable;
        if (dpos < 0)
            dpos = -dpos;
        if (dpos < 500) {
            fprintf(stderr,
                    "FAIL: act_pos barely changed (%d → %d) while claiming "
                    "velocity track — encoder/estimate stuck?\n",
                    (int)pos_at_enable, (int)last_pos);
            return EXIT_FAILURE;
        }
    }

    printf("PASS: velocity tracking within tolerance\n");
    return ret ? EXIT_FAILURE : EXIT_SUCCESS;
}
