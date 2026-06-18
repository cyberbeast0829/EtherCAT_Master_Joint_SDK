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
#define TRAJ_AMPLITUDE 16384.0
#define TRAJ_FREQ_HZ 0.25
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

static void add_period(struct timespec *time)
{
    time->tv_nsec += PERIOD_NS;
    while (time->tv_nsec >= NSEC_PER_SEC) {
        time->tv_nsec -= NSEC_PER_SEC;
        time->tv_sec++;
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
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("sched_setscheduler failed");
    }

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        fprintf(stderr, "warning: mlockall failed: %s\n", strerror(errno));
    }
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
    int32_t home_position = 0;
    int home_captured = 0;
    double traj_time = 0.0;
    unsigned int counter = 0;
    int ret = 0;

    (void)argc;
    (void)argv;

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
    joint_config.profile_name = JSDK_PROFILE_CYBERBEAST_JOINT_MODULE;

    status = jsdk_context_add_joint(ctx, &joint_config, &joint);
    if (status != JSDK_OK) {
        fprintf(stderr, "add_joint failed: %s (%s)\n",
                jsdk_status_string(status), jsdk_context_last_error(ctx));
        jsdk_context_destroy(ctx);
        return EXIT_FAILURE;
    }

    status = jsdk_context_activate(ctx);
    if (status != JSDK_OK) {
        fprintf(stderr, "activate failed: %s (%s)\n",
                jsdk_status_string(status), jsdk_context_last_error(ctx));
        jsdk_context_destroy(ctx);
        return EXIT_FAILURE;
    }

    jsdk_joint_request_enable(joint, JSDK_MODE_CSP);
    setup_realtime();

    printf("Starting SDK CSP example, period=%u ns.\n", PERIOD_NS);
    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
    wakeup_time.tv_sec += 1;
    wakeup_time.tv_nsec = 0;

    while (running) {
        jsdk_joint_feedback_t feedback;

        ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                &wakeup_time, NULL);
        if (ret) {
            fprintf(stderr, "clock_nanosleep: %s\n", strerror(ret));
            break;
        }

        status = jsdk_context_cycle_begin(ctx, monotonic_time_ns());
        if (status != JSDK_OK) {
            fprintf(stderr, "cycle_begin failed: %s (%s)\n",
                    jsdk_status_string(status), jsdk_context_last_error(ctx));
            break;
        }

        jsdk_joint_get_feedback(joint, &feedback);
        if (jsdk_joint_is_enabled(joint)) {
            double dt = (double)PERIOD_NS / (double)NSEC_PER_SEC;
            int32_t target;

            if (!home_captured) {
                home_position = feedback.actual_position;
                home_captured = 1;
                traj_time = 0.0;
                printf("Enabled. Home position=%d\n", home_position);
            }

            traj_time += dt;
            target = home_position + (int32_t)(TRAJ_AMPLITUDE *
                    sin(2.0 * PI * TRAJ_FREQ_HZ * traj_time));
            jsdk_joint_set_target_position(joint, target);
        } else {
            home_captured = 0;
            jsdk_joint_set_target_position(joint, feedback.actual_position);
        }

        if (!counter--) {
            jsdk_bus_state_t bus;

            counter = NSEC_PER_SEC / PERIOD_NS;
            jsdk_context_get_bus_state(ctx, &bus);
            printf("axis=%s status=0x%04X mode=%d pos=%d "
                   "wc=%u slaves=%u al=0x%02X\n",
                    jsdk_axis_state_string(feedback.axis_state),
                    feedback.statusword, feedback.mode_display,
                    feedback.actual_position, bus.working_counter,
                    bus.slaves_responding, bus.al_states);
        }

        status = jsdk_context_cycle_end(ctx);
        if (status != JSDK_OK) {
            fprintf(stderr, "cycle_end failed: %s (%s)\n",
                    jsdk_status_string(status), jsdk_context_last_error(ctx));
            break;
        }

        add_period(&wakeup_time);
    }

    printf("Stopping...\n");
    jsdk_joint_request_disable(joint);
    jsdk_context_cycle_begin(ctx, monotonic_time_ns());
    jsdk_context_cycle_end(ctx);
    jsdk_context_destroy(ctx);
    return ret ? EXIT_FAILURE : EXIT_SUCCESS;
}
