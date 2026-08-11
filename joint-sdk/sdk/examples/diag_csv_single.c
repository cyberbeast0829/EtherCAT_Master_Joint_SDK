/*****************************************************************************
 *
 *  守护兽关节 SDK CSV 诊断示例
 *
 *  用法:
 *    ./diag_csv_single              # 默认正弦速度
 *    ./diag_csv_single --hold       # 速度固定为 0
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

#include <joint_sdk/joint_sdk.h>

#define PERIOD_NS 1000000u
#define NSEC_PER_SEC 1000000000L
#define MAX_SAFE_STACK (8 * 1024)
#define VEL_AMPLITUDE 100000.0     /* 速度幅值 (指令单位/s)。首次测试用较小值 */
#define VEL_FREQ_HZ 0.5
#define PI 3.14159265358979323846

static volatile sig_atomic_t running = 1;

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
    t->tv_nsec += PERIOD_NS;
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
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1)
        perror("sched_setscheduler");
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
        fprintf(stderr, "warning: mlockall failed: %s\n", strerror(errno));
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
    double traj_time = 0.0;
    unsigned int counter = 0;
    int ret = 0;
    int hold_mode = 0;
    int print_header = 1;

    for (int i = 1; i < argc; i++)
        if (!strcmp(argv[i], "--hold")) hold_mode = 1;

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    jsdk_context_config_default(&ctx_config);
    ctx_config.master_index = 0;
    ctx_config.period_ns = PERIOD_NS;
    ctx_config.max_joints = 1;

    ctx = jsdk_context_create(&ctx_config);
    if (!ctx) {
        fprintf(stderr, "failed to create SDK context\n");
        return EXIT_FAILURE;
    }

    memset(&joint_config, 0, sizeof(joint_config));
    joint_config.alias = 0;
    joint_config.position = 0;
    joint_config.profile_name = "./ECAT_CIA402.xml";

    status = jsdk_context_add_joint(ctx, &joint_config, &joint);
    if (status != JSDK_OK) {
        fprintf(stderr, "add_joint: %s (%s)\n",
                jsdk_status_string(status), jsdk_context_last_error(ctx));
        jsdk_context_destroy(ctx);
        return EXIT_FAILURE;
    }

    status = jsdk_context_activate(ctx);
    if (status != JSDK_OK) {
        fprintf(stderr, "activate: %s (%s)\n",
                jsdk_status_string(status), jsdk_context_last_error(ctx));
        jsdk_context_destroy(ctx);
        return EXIT_FAILURE;
    }

    jsdk_joint_request_enable(joint, JSDK_MODE_CSV);
    setup_realtime();

    printf("=== Diagnostic CSV Example ===\n");
    printf("period=%u ns  hold=%s  vel_amplitude=%.0f\n",
            PERIOD_NS, hold_mode ? "YES" : "NO", VEL_AMPLITUDE);

    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
    wakeup_time.tv_sec += 1;
    wakeup_time.tv_nsec = 0;

    while (running) {
        jsdk_joint_feedback_t f;
        int32_t vel_cmd = 0;

        ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                &wakeup_time, NULL);
        if (ret) {
            fprintf(stderr, "clock_nanosleep: %s\n", strerror(ret));
            break;
        }

        status = jsdk_context_cycle_begin(ctx, monotonic_time_ns());
        if (status != JSDK_OK) {
            fprintf(stderr, "cycle_begin: %s (%s)\n",
                    jsdk_status_string(status), jsdk_context_last_error(ctx));
            break;
        }

        jsdk_joint_get_feedback(joint, &f);

        if (jsdk_joint_is_enabled(joint)) {
            if (hold_mode) {
                vel_cmd = 0;
            } else {
                double dt = (double)PERIOD_NS / (double)NSEC_PER_SEC;
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
            counter = NSEC_PER_SEC / PERIOD_NS;
            jsdk_context_get_bus_state(ctx, &bus);

            if (print_header) {
                printf("%-8s %-8s %-8s %-8s %-8s "
                       "%-6s %-4s %4s %5s %5s\n",
                       "axis", "status", "tgt_vel", "act_vel",
                       "act_pos", "mode", "trq",
                       "wc", "slv", "al");
                print_header = 0;
            }

            printf("%-8s %04X     %-8d %-8d %-8d "
                   "%-6d %-4d %4u %4u %02X\n",
                   jsdk_axis_state_string(f.axis_state),
                   f.statusword,
                   vel_cmd,
                   f.actual_velocity,
                   f.actual_position,
                   f.mode_display,
                   f.actual_torque,
                   bus.working_counter,
                   bus.slaves_responding,
                   bus.al_states);
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
    return ret ? EXIT_FAILURE : EXIT_SUCCESS;
}
