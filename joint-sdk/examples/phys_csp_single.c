/*****************************************************************************
 *
 *  守护兽关节 SDK 物理单位示例
 *
 *  演示使用 rad / rad·s⁻¹ / N·m 进行位置控制，
 *  内部自动完成指令单位换算和 16→32bit 位置展开。
 *
 *  用法:
 *    ./phys_csp_single              # 正弦轨迹 (rad)
 *    ./phys_csp_single --hold       # 静止
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

#define PERIOD_NS 1000000u
#define NSEC_PER_SEC 1000000000L
#define MAX_SAFE_STACK (8 * 1024)
#define AMPLITUDE_RAD 0.3      /* ±0.3 rad 摆动幅度 */
#define FREQ_HZ 0.25
#define PI 3.14159265358979323846

static volatile sig_atomic_t running = 1;
static void on_signal(int sig) { (void)sig; running = 0; }

static uint64_t monotonic_time_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)t.tv_sec * NSEC_PER_SEC + (uint64_t)t.tv_nsec;
}

static void add_period(struct timespec *t)
{
    t->tv_nsec += PERIOD_NS;
    while (t->tv_nsec >= NSEC_PER_SEC) {
        t->tv_nsec -= NSEC_PER_SEC;
        t->tv_sec++;
    }
}

static void stack_prefault(void)
{
    unsigned char d[MAX_SAFE_STACK];
    memset(d, 0, sizeof(d));
}

static void setup_realtime(void)
{
    struct sched_param p;
    memset(&p, 0, sizeof(p));
    p.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(0, SCHED_FIFO, &p) == -1)
        perror("sched_setscheduler");
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
        fprintf(stderr, "warning: mlockall: %s\n", strerror(errno));
    stack_prefault();
}

int main(int argc, char **argv)
{
    jsdk_context_config_t ctx_config;
    jsdk_joint_config_t joint_config;
    jsdk_context_t *ctx;
    jsdk_joint_t *joint = NULL;
    jsdk_status_t status;
    struct timespec wakeup_time;
    double home_rad = 0.0;
    int home_captured = 0;
    double traj_time = 0.0;
    unsigned int counter = 0;
    int ret = 0;
    int hold_mode = 0;
    int print_header = 1;
    jsdk_unit_scale_t scale;

    for (int i = 1; i < argc; i++)
        if (!strcmp(argv[i], "--hold")) hold_mode = 1;

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    jsdk_context_config_default(&ctx_config);
    ctx_config.master_index = 0;
    ctx_config.period_ns = PERIOD_NS;
    ctx_config.max_joints = 1;
    jsdk_dc_timing_conf_apply(&ctx_config);

    ctx = jsdk_context_create(&ctx_config);
    if (!ctx) { fprintf(stderr, "create ctx failed\n"); return -1; }

    memset(&joint_config, 0, sizeof(joint_config));
    joint_config.alias = 0;
    joint_config.position = 0;
    joint_config.profile_name = JSDK_PROFILE_CYBERBEAST_JOINT_MODULE;

    status = jsdk_context_add_joint(ctx, &joint_config, &joint);
    if (status != JSDK_OK) {
        fprintf(stderr, "add_joint: %s (%s)\n",
                jsdk_status_string(status), jsdk_context_last_error(ctx));
        jsdk_context_destroy(ctx); return -1;
    }

    /* 设置换算系数。若已知实际 0x608F/0x6091/0x6076 值则用 calc 精确计算：
       jsdk_unit_scale_calc(&scale, 16384, 8, 1, 10000); */
    jsdk_unit_scale_default(&scale, 10000);
    jsdk_joint_set_scale(joint, &scale);
    printf("Scale: %.6f rad/count, trq_to_Nm=%.6f\n",
            scale.pos_counts_to_rad, scale.trq_to_Nm);

    status = jsdk_context_activate(ctx);
    if (status != JSDK_OK) {
        fprintf(stderr, "activate: %s (%s)\n",
                jsdk_status_string(status), jsdk_context_last_error(ctx));
        jsdk_context_destroy(ctx); return -1;
    }

    jsdk_joint_request_enable(joint, JSDK_MODE_CSP);
    setup_realtime();

    printf("=== Physical-Unit CSP Example ===\n");
    printf("period=%u ns  hold=%s  amplitude=%.2f rad\n",
            PERIOD_NS, hold_mode ? "YES" : "NO", AMPLITUDE_RAD);

    /* Do not idle 1s after activate — DC needs application_time immediately. */
    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
    add_period(&wakeup_time);

    while (running) {
        jsdk_joint_feedback_t f;
        double target_rad;

        ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                &wakeup_time, NULL);
        if (ret) { fprintf(stderr, "nanosleep: %s\n", strerror(ret)); break; }

        status = jsdk_context_cycle_begin(ctx, monotonic_time_ns());
        if (status != JSDK_OK) break;

        jsdk_joint_get_feedback(joint, &f);

        if (jsdk_joint_is_enabled(joint)) {
            if (!home_captured) {
                home_rad = jsdk_joint_actual_position_rad(joint);
                home_captured = 1;
                traj_time = 0.0;
                printf("\n>>> ENABLED (home=%.4f rad)\n\n", home_rad);
            }

            if (hold_mode) {
                target_rad = home_rad;
            } else {
                double dt = (double)PERIOD_NS / NSEC_PER_SEC;
                traj_time += dt;
                target_rad = home_rad +
                    AMPLITUDE_RAD * sin(2.0 * PI * FREQ_HZ * traj_time);
            }
            jsdk_joint_set_target_position_rad(joint, target_rad);
        } else {
            home_captured = 0;
        }

        if (!counter--) {
            jsdk_bus_state_t bus;
            counter = NSEC_PER_SEC / PERIOD_NS;
            jsdk_context_get_bus_state(ctx, &bus);

            double actual_rad = jsdk_joint_actual_position_rad(joint);
            double actual_vel = jsdk_joint_actual_velocity_rad_s(joint);

            if (print_header) {
                printf("%-8s %-8s %-8s %-8s "
                       "%-6s %-6s %4s %5s %5s\n",
                       "axis", "tgt_rad", "act_rad", "act_vel",
                       "mode", "trq_Nm", "wc", "slv", "al");
                print_header = 0;
            }

            printf("%-8s %-8.3f %-8.3f %-8.3f "
                   "%-6d %-6.2f %4u %4u %02X\n",
                   jsdk_axis_state_string(f.axis_state),
                   target_rad,
                   actual_rad,
                   actual_vel,
                   f.mode_display,
                   jsdk_joint_actual_torque_Nm(joint),
                   bus.working_counter,
                   bus.slaves_responding,
                   bus.al_states);
        }

        status = jsdk_context_cycle_end(ctx);
        if (status != JSDK_OK) break;

        add_period(&wakeup_time);
    }

    printf("\nStopping...\n");
    jsdk_joint_request_disable(joint);
    jsdk_context_cycle_begin(ctx, monotonic_time_ns());
    jsdk_context_cycle_end(ctx);
    jsdk_context_destroy(ctx);
    return ret ? EXIT_FAILURE : EXIT_SUCCESS;
}
