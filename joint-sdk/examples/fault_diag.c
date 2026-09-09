/*****************************************************************************
 *
 *  守护兽关节 SDK 故障诊断示例
 *
 *  演示：
 *    - SDK 内置故障自动检测：Statusword bit3=1 时自动通过 SDO
 *      读取 0x603F / 0x1001 / 0x203F / 0x203E
 *    - 故障回调注册：实时打印故障详情
 *    - 故障信息同步查询：反馈中查看故障快照
 *    - fault_reset() 复位后观察回调触发和清除
 *
 *  用法:
 *    ./fault_diag         # 正常运行，Ctrl-C 停机
 *    若想主动触发故障（如手动断开编码器线缆、拔网线），
 *    终端输出会显示自动捕获的故障码。
 *
 ****************************************************************************/

#define _POSIX_C_SOURCE 200809L

#include <errno.h>
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

static volatile sig_atomic_t running = 1;

static void on_signal(int sig) { (void)sig; running = 0; }

/* 故障回调 —— 在 RT 周期上下文中调用，只能做轻量操作 */
static void on_fault(jsdk_joint_t *joint, const jsdk_fault_info_t *info,
        void *user_data)
{
    (void)joint;
    (void)user_data;

    fprintf(stderr,
            "\n!!! FAULT DETECTED !!!\n"
            "    0x603F (CiA402):  0x%04X\n"
            "    0x1001 (register):  0x%02X\n"
            "    0x203F (vendor lo): 0x%08X\n"
            "    0x203E (vendor hi): 0x%08X\n",
            info->code_603f,
            info->error_register,
            info->vendor_lo,
            info->vendor_hi);
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
    unsigned int counter = 0;
    int ret = 0;
    int print_header = 1;
    int last_fault_state = 0;

    (void)argc; (void)argv;
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

    /* 注册故障回调 */
    jsdk_context_set_fault_callback(ctx, on_fault, NULL);

    status = jsdk_context_activate(ctx);
    if (status != JSDK_OK) {
        fprintf(stderr, "activate: %s (%s)\n",
                jsdk_status_string(status), jsdk_context_last_error(ctx));
        jsdk_context_destroy(ctx); return -1;
    }

    jsdk_joint_request_enable(joint, JSDK_MODE_CSP);
    setup_realtime();

    printf("=== Fault Diagnostic Example ===\n");
    printf("Fault callback registered. "
           "Disconnect encoder or pull network cable to trigger.\n");

    /* Do not idle 1s after activate — DC needs application_time immediately. */
    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
    add_period(&wakeup_time);

    while (running) {
        jsdk_joint_feedback_t f;
        jsdk_fault_info_t fi;

        ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                &wakeup_time, NULL);
        if (ret) { fprintf(stderr, "nanosleep: %s\n", strerror(ret)); break; }

        status = jsdk_context_cycle_begin(ctx, monotonic_time_ns());
        if (status != JSDK_OK) break;

        jsdk_joint_get_feedback(joint, &f);
        if (jsdk_joint_is_enabled(joint))
            jsdk_joint_set_target_position(joint, f.actual_position);
        else
            jsdk_joint_set_target_position(joint, 0);

        /* 每秒打印：fault 状态 + 故障信息快照 */
        if (!counter--) {
            jsdk_bus_state_t bus;
            counter = NSEC_PER_SEC / PERIOD_NS;
            jsdk_context_get_bus_state(ctx, &bus);
            jsdk_joint_get_fault_info(joint, &fi);

            if (print_header) {
                printf("%-8s %5s %-8s %-8s %4s %5s %5s  %s\n",
                       "axis", "fault", "603F", "1001",
                       "wc", "slv", "al", "vendor_lo/vendor_hi");
                print_header = 0;
            }

            printf("%-8s %5s %04X     %02X       "
                   "%4u %4u %02X  %08X/%08X\n",
                   jsdk_axis_state_string(f.axis_state),
                   f.statusword & 0x0008 ? "YES" : "no",
                   fi.code_603f, fi.error_register,
                   bus.working_counter, bus.slaves_responding,
                   bus.al_states,
                   fi.vendor_lo, fi.vendor_hi);

            /* 故障清除检测 */
            if (last_fault_state && !(f.statusword & 0x0008))
                printf(">>> Fault cleared.\n");
            last_fault_state = !!(f.statusword & 0x0008);
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
