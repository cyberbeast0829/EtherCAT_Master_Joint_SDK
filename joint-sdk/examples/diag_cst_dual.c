/*****************************************************************************
 *
 *  守护兽关节 SDK — 双轴 CST 最小诊断（同一主站、同一 DC）
 *
 *  现场注意：
 *    - CST 恒力矩会持续加速；SDK 对 CST 自动打开 0x200A:12 限速并设 0x6080
 *    - 0x6071：0.1% 额定（额定 10 N·m 时，100 = 1 N·m）
 *    - 0x6077 反馈量纲未必与 0x6071 相同
 *    - 负 --nm 即反转力矩方向（不做软件极性反相）
 *    - 双轴默认 Sync0=task（IgH 上 TwinCAT ×3 易 sync 失败）
 *
 *  用法:
 *    sudo chrt -f 80 ./diag_cst_dual --nm 1 --vmax 50000 --seconds 10
 *    sudo chrt -f 80 ./diag_cst_dual --hold --seconds 6
 *    sudo chrt -f 80 ./diag_cst_dual --nm 1.5 --vmax 50000 --seconds 10
 *
 *  默认 fw0=8.1.50 / fw1=8.1.44；换线序用 --fw0/--fw1（或两次 --fw）。
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
#define RAMP_SEC 2.0
#define HOLD_SETTLE_SEC 2.0
#define VEL_RUNAWAY 200000
#define HOLD_VEL_OK 2000
#define MIN_MOVE_VEL 2000
#define MIN_DPOS 500
#define NM_PER_TRQ_UNIT 0.01
#define CST_DEFAULT_VMAX 50000u
#define WC_DROP_TOLERANCE 8
#define WC_DROP_DEBOUNCE 3
/** Soft re-ramp after sustained WC hole (seconds). */
#define RECOVER_RAMP_SEC 0.8
/** App: consecutive bad-bus cycles before forcing trq=0 (~100 ms @ 2 ms). */
#define BUS_BAD_ZERO_CYCLES 50
#define MIN_TRACK_OK 3

static volatile sig_atomic_t running = 1;
static uint32_t g_period_ns = 1000000u;
static uint32_t g_vmax = CST_DEFAULT_VMAX;

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

static int16_t soft_ramp_i16(int16_t target, double elapsed_s, double ramp_s)
{
    double a;

    if (ramp_s < 0.5)
        ramp_s = 0.5;
    if (elapsed_s >= ramp_s)
        return target;
    if (elapsed_s <= 0.0)
        return 0;
    a = elapsed_s / ramp_s;
    return (int16_t)lround((double)target * a);
}

/*
 * Near vmax, ease torque so Sync0 is not slammed into 0x6080.
 */
static int16_t speed_limited_trq(int16_t cmd, int32_t act_vel,
        double *scale_io)
{
    int32_t abs_vel = act_vel < 0 ? -act_vel : act_vel;
    int32_t soft = (int32_t)(g_vmax * 90u / 100u);
    int32_t hard = (int32_t)(g_vmax * 98u / 100u);
    double instant;
    double scale = *scale_io;

    if (hard <= soft)
        instant = 1.0;
    else if (abs_vel <= soft)
        instant = 1.0;
    else if (abs_vel >= hard)
        instant = 0.0;
    else
        instant = (double)(hard - abs_vel) / (double)(hard - soft);

    if (instant < scale)
        scale = instant;
    else if (abs_vel < soft)
        scale = scale + 0.02;
    if (scale > 1.0)
        scale = 1.0;
    if (scale < 0.0)
        scale = 0.0;
    *scale_io = scale;
    return (int16_t)lround((double)cmd * scale);
}

typedef struct {
    jsdk_joint_t *joint;
    uint16_t position;
    const char *fw;
    int16_t trq_cmd;
    int16_t last_tgt;
    int16_t last_act;
    int32_t last_vel;
    int32_t pos_at_enable;
    int32_t last_pos;
    int32_t peak_abs_vel;
    double trq_scale;
    int home_ok;
    int reached;
    int ok_n;
    int settle_n;
    int hold_n;
    int runaway;
} axis_t;

/*
 * CST must not cut the bus while still applying torque.
 * 1) torque=0 while still CST  2) CSV vel=0 brake  3) disable
 */
static void axis_controlled_stop(jsdk_context_t *ctx, axis_t *axes,
        struct timespec *wakeup_time, int hold_mode)
{
    unsigned int n_zero = 600000000u / g_period_ns;
    unsigned int n_brake = 1500000000u / g_period_ns;
    unsigned int n_dis = 400000000u / g_period_ns;
    unsigned int i, a;

    if (n_zero < 50)
        n_zero = 50;
    if (n_brake < 100)
        n_brake = 100;
    if (n_dis < 50)
        n_dis = 50;

    printf("Stopping: CST torque→0 (%.1f s)...\n",
           (double)n_zero * g_period_ns / 1e9);
    for (i = 0; i < n_zero; i++) {
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, wakeup_time, NULL);
        if (jsdk_context_cycle_begin(ctx, monotonic_time_ns()) != JSDK_OK)
            break;
        for (a = 0; a < N_AXES; a++) {
            jsdk_joint_set_target_torque(axes[a].joint, 0);
            jsdk_joint_set_target_velocity(axes[a].joint, 0);
        }
        jsdk_context_cycle_end(ctx);
        add_period(wakeup_time);
    }

    if (!hold_mode) {
        printf("Stopping: CSV vel=0 brake (%.1f s)...\n",
               (double)n_brake * g_period_ns / 1e9);
        for (a = 0; a < N_AXES; a++)
            jsdk_joint_request_enable(axes[a].joint, JSDK_MODE_CSV);
        for (i = 0; i < n_brake; i++) {
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                    wakeup_time, NULL);
            if (jsdk_context_cycle_begin(ctx, monotonic_time_ns()) !=
                    JSDK_OK)
                break;
            for (a = 0; a < N_AXES; a++) {
                jsdk_joint_set_target_velocity(axes[a].joint, 0);
                jsdk_joint_set_target_torque(axes[a].joint, 0);
            }
            jsdk_context_cycle_end(ctx);
            add_period(wakeup_time);
        }
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
    int fixed_nm_mode = 0;
    double target_nm = 0.0;
    int16_t trq = 0;
    double ramp_s = RAMP_SEC;
    int seconds = 0; /* 0 = run until Ctrl+C */
    int wc_drop_secs = 0;
    int wc_bad_streak = 0;
    int interrupted = 0;
    unsigned int counter = 0;
    uint64_t t0_ns = 0;
    uint64_t en_t0 = 0;
    uint64_t first_enable_ns = 0;
    int enabled_seen = 0;
    int print_header = 1;
    int a;
    int use_dc = 1;
    int twincat_dc = 0;
    int sync0_cycle_set = 0;
    uint32_t sync0_cycle_ns = 0;
    uint32_t vmax = 0;
    int fw_cli_n = 0;
    int bus_bad_cycles = 0;
    int need_recover_ramp = 0;
    uint64_t recover_t0 = 0;
    const jsdk_drive_model_t *drive_model;

    memset(axes, 0, sizeof(axes));
    axes[0].position = 0;
    axes[1].position = 1;
    axes[0].fw = "8.1.50";
    axes[1].fw = "8.1.44";
    for (a = 0; a < N_AXES; a++)
        axes[a].trq_scale = 1.0;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--hold")) {
            hold_mode = 1;
        } else if (!strcmp(argv[i], "--nm") && i + 1 < argc) {
            fixed_nm_mode = 1;
            target_nm = atof(argv[++i]);
            trq = (int16_t)lround(target_nm / NM_PER_TRQ_UNIT);
            ramp_s = 0.5 + 0.15 * fabs(target_nm);
            if (ramp_s > 2.0)
                ramp_s = 2.0;
        } else if (!strcmp(argv[i], "--vmax") && i + 1 < argc) {
            vmax = (uint32_t)strtoul(argv[++i], NULL, 0);
        } else if (!strcmp(argv[i], "--seconds") && i + 1 < argc) {
            seconds = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "--fw0") && i + 1 < argc) {
            axes[0].fw = argv[++i];
        } else if (!strcmp(argv[i], "--fw1") && i + 1 < argc) {
            axes[1].fw = argv[++i];
        } else if (!strcmp(argv[i], "--fw") && i + 1 < argc) {
            /* Same as csv_dual intent: 1st→fw0, 2nd→fw1. */
            if (!fw_cli_n)
                axes[0].fw = argv[++i];
            else if (fw_cli_n == 1)
                axes[1].fw = argv[++i];
            else {
                fprintf(stderr, "too many --fw (use --fw0/--fw1)\n");
                return EXIT_FAILURE;
            }
            fw_cli_n++;
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
                    "Usage: %s --hold | --nm N\n"
                    "  [--fw0 8.1.50] [--fw1 8.1.44]\n"
                    "  [--fw VER]     可写两次：第1次=fw0，第2次=fw1\n"
                    "  [--pos0 0] [--pos1 1]\n"
                    "  [--vmax N]  写 0x6080，可任意设（省略则默认 %u）\n"
                    "              例: --vmax 30000 / --vmax 50000 / --vmax 80000\n"
                    "  [--dc|--no-dc] [--twincat-dc] [--sync0-cycle N]\n"
                    "  [--seconds N]\n"
                    "  Negative --nm reverses torque direction.\n"
                    "  Prefer: sudo chrt -f 80 %s --fw0 8.1.50 --fw1 8.1.44 "
                    "--nm 1 --vmax %u --seconds 10\n",
                    argv[0], CST_DEFAULT_VMAX, argv[0], CST_DEFAULT_VMAX);
            return EXIT_SUCCESS;
        } else {
            fprintf(stderr, "unknown arg: %s (try --help)\n", argv[i]);
            return EXIT_FAILURE;
        }
    }

    if (!hold_mode && !fixed_nm_mode) {
        fprintf(stderr, "need --hold or --nm N\n");
        return EXIT_FAILURE;
    }

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

    if (!hold_mode && !vmax)
        vmax = CST_DEFAULT_VMAX;
    g_vmax = vmax ? vmax : CST_DEFAULT_VMAX;

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
        /* --hold: CST+0 coasts; use CSV vel=0 brake. */
        if (hold_mode)
            jc.initial_mode = JSDK_MODE_CSV;
        else
            jc.initial_mode = JSDK_MODE_CST;
        jc.input_mode_2002 = 1; /* PASSTHROUGH */
        /* Always write 0x6080 for CST (and hold CSV brake uses same). */
        jc.max_motor_velocity = g_vmax;

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

    printf("dual CST: ref_clock=joint0(pos=%u fw=%s)  "
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
    printf("0x6080/vmax=%u%s\n",
           g_vmax,
           hold_mode ? " (hold=CSV)" : "");

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
            jsdk_joint_feedback_t fb[N_AXES];
            int both_ok = 1;

            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                    &wakeup_time, NULL);
            status = jsdk_context_cycle_begin(ctx, monotonic_time_ns());
            if (status != JSDK_OK)
                break;
            for (a = 0; a < N_AXES; a++) {
                jsdk_joint_set_target_torque(axes[a].joint, 0);
                jsdk_joint_set_target_velocity(axes[a].joint, 0);
                jsdk_joint_get_feedback(axes[a].joint, &fb[a]);
            }
            jsdk_context_get_bus_state(ctx, &bus);
            jsdk_context_cycle_end(ctx);
            add_period(&wakeup_time);
            tries++;

            if (!(bus.working_counter > 0 && bus.wc_state == 2 &&
                        (bus.al_states & 0x0f) == 0x08))
                both_ok = 0;
            for (a = 0; a < N_AXES; a++) {
                if (!(fb[a].online &&
                            ((fb[a].al_state & 0x0f) == 0x08 ||
                             fb[a].operational)))
                    both_ok = 0;
            }
            if (both_ok)
                good++;
            else
                good = 0;
        }
        if (good < need_cycles) {
            fprintf(stderr,
                    "FAIL: bus never reached stable OP+WC "
                    "(good=%u/%u tries=%u)\n",
                    good, need_cycles, tries);
            jsdk_context_destroy(ctx);
            return EXIT_FAILURE;
        }
        printf("bus settle ok (%u cycles)\n", good);
    }

    for (a = 0; a < N_AXES; a++) {
        if (hold_mode)
            jsdk_joint_request_enable(axes[a].joint, JSDK_MODE_CSV);
        else
            jsdk_joint_request_enable(axes[a].joint, JSDK_MODE_CST);
    }

    if (hold_mode)
        printf("HOLD both axes (CSV vel=0)\n");
    else
        printf("BOTH: --nm %.3f → trq(0x6071)=%d  ramp %.1fs  "
               "mode=CST(10) --vmax %u (0x6080)\n",
               target_nm, (int)trq, ramp_s, g_vmax);
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
        for (a = 0; a < N_AXES; a++)
            jsdk_joint_get_feedback(axes[a].joint, &fb[a]);
        bus_ok = (bus.wc_state == 2 && bus.working_counter > 0);
        for (a = 0; a < N_AXES; a++) {
            if (!(((fb[a].al_state & 0x0f) == 0x08) || fb[a].operational))
                bus_ok = 0;
        }

        if (jsdk_joint_is_enabled(axes[0].joint) &&
                jsdk_joint_is_enabled(axes[1].joint)) {
            double recover_scale = 1.0;

            if (!enabled_seen) {
                enabled_seen = 1;
                en_t0 = now;
                first_enable_ns = now;
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
                    for (a = 0; a < N_AXES; a++)
                        axes[a].trq_scale = 1.0;
                    printf("bus recovered — soft-ramp torque again\n");
                }
                bus_bad_cycles = 0;
            }

            if (need_recover_ramp && bus_ok && recover_t0) {
                double rs = (double)(now - recover_t0) /
                        (double)NSEC_PER_SEC;
                if (rs >= RECOVER_RAMP_SEC) {
                    need_recover_ramp = 0;
                    recover_scale = 1.0;
                } else if (rs <= 0.0) {
                    recover_scale = 0.0;
                } else {
                    recover_scale = rs / RECOVER_RAMP_SEC;
                }
            }

            for (a = 0; a < N_AXES; a++) {
                axis_t *ax = &axes[a];
                jsdk_joint_feedback_t *f = &fb[a];
                int16_t cmd;
                int32_t abs_v;

                if (!ax->home_ok) {
                    ax->home_ok = 1;
                    ax->pos_at_enable = f->actual_position;
                    printf(">>> axis%u ENABLED pos=%d\n",
                           a, f->actual_position);
                }
                ax->last_pos = f->actual_position;
                abs_v = iabs32(f->actual_velocity);
                if (abs_v > ax->peak_abs_vel)
                    ax->peak_abs_vel = abs_v;

                /*
                 * Brief WC holes: keep last/desired torque (SDK also
                 * debounces). Sustained loss → zero then soft re-ramp.
                 */
                if (!bus_ok && bus_bad_cycles >= BUS_BAD_ZERO_CYCLES) {
                    ax->trq_cmd = 0;
                    jsdk_joint_set_target_torque(ax->joint, 0);
                    jsdk_joint_set_target_velocity(ax->joint, 0);
                    continue;
                }

                if (hold_mode) {
                    cmd = 0;
                    jsdk_joint_set_target_velocity(ax->joint, 0);
                    jsdk_joint_set_target_torque(ax->joint, 0);
                } else {
                    cmd = soft_ramp_i16(trq, en_s, ramp_s);
                    cmd = (int16_t)lround((double)cmd * recover_scale);
                    cmd = speed_limited_trq(cmd, f->actual_velocity,
                            &ax->trq_scale);
                    jsdk_joint_set_target_torque(ax->joint, cmd);
                    jsdk_joint_set_target_velocity(ax->joint, 0);
                }
                ax->trq_cmd = cmd;
                ax->last_tgt = cmd;
                ax->last_act = f->actual_torque;
                ax->last_vel = f->actual_velocity;
            }
        } else {
            for (a = 0; a < N_AXES; a++) {
                jsdk_joint_set_target_torque(axes[a].joint, 0);
                if (hold_mode)
                    jsdk_joint_set_target_velocity(axes[a].joint, 0);
            }
        }

        if (!counter--) {
            double settle_need = hold_mode ? HOLD_SETTLE_SEC : ramp_s;
            int settled;

            counter = NSEC_PER_SEC / g_period_ns;
            en_s = enabled_seen
                    ? (double)(now - en_t0) / (double)NSEC_PER_SEC
                    : 0.0;
            settled = enabled_seen && en_s >= settle_need
                    && !need_recover_ramp;

            if (enabled_seen && first_enable_ns &&
                    (double)(now - first_enable_ns) /
                            (double)NSEC_PER_SEC >= 1.0 &&
                    !bus_ok) {
                wc_bad_streak++;
                if (wc_bad_streak >= WC_DROP_DEBOUNCE)
                    wc_drop_secs++;
            } else {
                wc_bad_streak = 0;
            }

            if (print_header) {
                printf("%-6s %-8s %-8s %-8s %-8s %-5s "
                       "%4s %3s %3s %4s\n",
                       "axis", "sw", "tgt_trq", "act_trq", "act_vel",
                       "mode", "wc", "slv", "al", "ok");
                print_header = 0;
            }

            for (a = 0; a < N_AXES; a++) {
                axis_t *ax = &axes[a];
                jsdk_joint_feedback_t *f = &fb[a];
                int32_t abs_vel = iabs32(f->actual_velocity);
                int track;
                const char *tag = "ramp";
                int bus_hole = !bus_ok &&
                        bus_bad_cycles >= BUS_BAD_ZERO_CYCLES;

                if (settled && !hold_mode && abs_vel > VEL_RUNAWAY)
                    ax->runaway = 1;

                if (hold_mode) {
                    track = settled && bus_ok && f->mode_display == 9 &&
                            abs_vel <= HOLD_VEL_OK;
                } else {
                    track = settled && bus_ok && !ax->runaway &&
                            f->mode_display == 10 &&
                            abs_vel > MIN_MOVE_VEL;
                }

                if (settled && track)
                    ax->reached = 1;

                if (!ax->reached || need_recover_ramp) {
                    tag = "ramp";
                } else if (bus_hole) {
                    ax->settle_n++;
                    ax->hold_n++;
                    tag = "hold";
                } else if (track) {
                    ax->settle_n++;
                    ax->ok_n++;
                    tag = "YES";
                } else if (!bus_ok) {
                    /* Brief hole while still commanding — don't spam hold. */
                    tag = "gap";
                } else {
                    ax->settle_n++;
                    tag = "no";
                }

                ax->last_tgt = ax->trq_cmd;
                ax->last_act = f->actual_torque;
                ax->last_vel = f->actual_velocity;
                ax->last_pos = f->actual_position;

                printf("j%-5u %04X     %-8d %-8d %-8d %-5d "
                       "%4u %3u %02X %4s\n",
                       a,
                       f->statusword,
                       ax->trq_cmd,
                       f->actual_torque,
                       f->actual_velocity,
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
    axis_controlled_stop(ctx, axes, &wakeup_time, hold_mode);
    jsdk_context_destroy(ctx);

    for (a = 0; a < N_AXES; a++) {
        int32_t dpos = axes[a].last_pos - axes[a].pos_at_enable;
        printf("summary j%u: tgt=%d act_trq=%d act_v=%d ok=%d/%d "
               "hold=%d reached=%d dpos=%d peak_v=%d runaway=%d\n",
               a, axes[a].last_tgt, axes[a].last_act, axes[a].last_vel,
               axes[a].ok_n, axes[a].settle_n, axes[a].hold_n,
               axes[a].reached, (int)dpos, axes[a].peak_abs_vel,
               axes[a].runaway);
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
        if (axes[a].runaway) {
            fprintf(stderr,
                    "FAIL: axis%u velocity runaway (|v|>%d). "
                    "Check 0x200A:12 and 0x6080.\n",
                    a, VEL_RUNAWAY);
            return EXIT_FAILURE;
        }
    }
    if (hold_mode) {
        for (a = 0; a < N_AXES; a++) {
            if (!axes[a].reached || axes[a].ok_n < MIN_TRACK_OK) {
                fprintf(stderr, "FAIL: axis%u hold not stable.\n", a);
                return EXIT_FAILURE;
            }
        }
        if (wc_drop_secs > WC_DROP_TOLERANCE) {
            fprintf(stderr, "FAIL: %d sustained WC/AL dropout sample(s).\n",
                    wc_drop_secs);
            return EXIT_FAILURE;
        }
        printf("PASS: dual CST hold (CSV vel=0) stable OP+WC\n");
        return EXIT_SUCCESS;
    }
    for (a = 0; a < N_AXES; a++) {
        int32_t dpos = iabs32(axes[a].last_pos - axes[a].pos_at_enable);
        int moved = (dpos >= MIN_DPOS) ||
                (axes[a].peak_abs_vel >= MIN_MOVE_VEL);

        if (!axes[a].reached || axes[a].ok_n < MIN_TRACK_OK) {
            if (wc_drop_secs > 0 || axes[a].hold_n > 0) {
                fprintf(stderr,
                        "FAIL: axis%u CST lost tracking after WC/AL hole "
                        "(ok=%d/%d hold=%d wc_drops=%d).\n",
                        a, axes[a].ok_n, axes[a].settle_n,
                        axes[a].hold_n, wc_drop_secs);
            } else if (!moved) {
                fprintf(stderr,
                        "FAIL: axis%u barely moved "
                        "(dpos/peak_v too small).\n",
                        a);
            } else {
                fprintf(stderr,
                        "FAIL: axis%u CST not stable "
                        "(ok=%d/%d peak_v=%d).\n",
                        a, axes[a].ok_n, axes[a].settle_n,
                        axes[a].peak_abs_vel);
            }
            return EXIT_FAILURE;
        }
        if (!moved) {
            fprintf(stderr,
                    "FAIL: axis%u barely moved "
                    "(dpos=%d peak_v=%d).\n",
                    a, (int)dpos, axes[a].peak_abs_vel);
            return EXIT_FAILURE;
        }
    }
    if (wc_drop_secs > WC_DROP_TOLERANCE) {
        int end_ok = 1;

        for (a = 0; a < N_AXES; a++) {
            if (axes[a].ok_n * 2 < axes[a].settle_n)
                end_ok = 0;
        }
        if (!end_ok) {
            fprintf(stderr, "FAIL: WC drops and tracking did not recover.\n");
            return EXIT_FAILURE;
        }
        printf("note: wc_drops=%d (within recover policy)\n", wc_drop_secs);
    }
    printf("PASS: dual CST both axes continuous torque + motion\n");
    return EXIT_SUCCESS;
}
