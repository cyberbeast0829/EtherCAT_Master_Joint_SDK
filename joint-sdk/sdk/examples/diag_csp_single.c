/*****************************************************************************
 *
 *  守护兽关节 SDK 诊断增强示例 — 基于 CSP
 *
 *  相比 csp_single_sdk，新增以下诊断输出：
 *    - 使能前逐周期打印 status/mode/al_state 直到进入 OP
 *    - 使能后同时打印 target / actual / error / velocity / torque
 *    - 提供"静止测试模式"(--hold)，目标固定不运动，仅监测反馈
 *    - 提供"PDO 偏移打印"(--offsets)，运行前打印各 PDO 偏移
 *    - 可选"直流模式"(--nosine)，禁用正弦, target=home
 *
 *  用法:
 *    ./diag_csp_single              # 默认正弦轨迹
 *    ./diag_csp_single --hold       # 目标固定不动
 *    ./diag_csp_single --offsets    # 激活前打印 PDO 偏移
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
    int32_t home_position = 0;
    int home_captured = 0;
    double traj_time = 0.0;
    unsigned int counter = 0;
    int ret = 0;
    int show_offsets = 0;
    int hold_mode = 0;
    int no_sine = 0;
    int print_header = 1;

    /* 解析命令行选项 */
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--hold"))    hold_mode = 1;
        if (!strcmp(argv[i], "--nosine"))  no_sine = 1;
        if (!strcmp(argv[i], "--offsets")) show_offsets = 1;
    }

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
    /* 默认用 ESI 文件动态加载（不依赖编译期 profile）
       若仅验证链路可用内置 profile:
         joint_config.profile_name = JSDK_PROFILE_CYBERBEAST_JOINT_MODULE;
    */
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

    /* 需要 PDO 偏移时，在激活后无法直接读取；当前设计未暴露该接口。
     * 此处仅打印说明，后续可在 internal.h 增加 getter 或 profile 打印函数。 */
    if (show_offsets) {
        printf("[diagnostic] PDO offsets are internal; "
               "use --hold + manual SDO upload to cross-check.\n");
    }

    jsdk_joint_request_enable(joint, JSDK_MODE_CSP);
    setup_realtime();

    printf("=== Diagnostic CSP Example ===\n");
    printf("period=%u ns  mode=%s  hold=%s  sine=%s\n",
            PERIOD_NS,
            hold_mode ? "stationary" : (no_sine ? "dc" : "sine"),
            hold_mode ? "YES" : "NO",
            (hold_mode || no_sine) ? "disabled" : "enabled");

    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
    wakeup_time.tv_sec += 1;
    wakeup_time.tv_nsec = 0;

    while (running) {
        jsdk_joint_feedback_t f;
        int32_t target = 0;
        int32_t error = 0;

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
            if (!home_captured) {
                home_position = f.actual_position;
                home_captured = 1;
                traj_time = 0.0;
                printf("\n>>> ENABLED (home=%d)\n", home_position);
                printf("    Now printing per-second diagnostics...\n\n");
            }

            if (hold_mode || no_sine) {
                /* 静止/直流模式：目标固定 */
                target = home_position;
            } else {
                /* 正弦模式 */
                double dt = (double)PERIOD_NS / (double)NSEC_PER_SEC;
                traj_time += dt;
                target = home_position +
                    (int32_t)(TRAJ_AMPLITUDE *
                            sin(2.0 * PI * TRAJ_FREQ_HZ * traj_time));
            }
            error = target - f.actual_position;
            jsdk_joint_set_target_position(joint, target);
        } else {
            home_captured = 0;
            jsdk_joint_set_target_position(joint, f.actual_position);
        }

        /* 每秒打印增强诊断 */
        if (!counter--) {
            jsdk_bus_state_t bus;
            counter = NSEC_PER_SEC / PERIOD_NS;
            jsdk_context_get_bus_state(ctx, &bus);

            if (print_header) {
                printf("%-8s %-8s %-8s %-8s %-8s "
                       "%-6s %-6s %-4s %4s %5s %5s\n",
                       "axis", "status", "target", "actual",
                       "error", "mode", "vel", "trq",
                       "wc", "slv", "al");
                print_header = 0;
            }

            printf("%-8s %04X     %-8d %-8d %-8d "
                   "%-6d %-6d %-4d %4u %4u %02X\n",
                   jsdk_axis_state_string(f.axis_state),
                   f.statusword,
                   target,
                   f.actual_position,
                   error,
                   f.mode_display,
                   f.actual_velocity,
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
