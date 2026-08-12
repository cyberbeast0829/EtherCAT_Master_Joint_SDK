/*****************************************************************************
 *
 *  守护兽关节 SDK 多轴 CSP 同步示例
 *
 *  配置 N 个关节到同一 domain，验证：
 *    - 多轴 PDO 偏移正确不重叠
 *    - DC 参考时钟自动选择
 *    - WKC 按从站数增长
 *    - 单轴故障不影响其他轴（需在另一终端手动触发）
 *
 *  用法:
 *    ./multi_axis_csp               # 默认 2 轴
 *    ./multi_axis_csp 3             # 3 轴
 *
 *  注意：物理总线上必须有 ≥N 个守护兽关节连接。
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
#define MAX_AXES 10
#define AMPLITUDE 5000.0     /* 小幅值，各轴错开相位 */
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
    jsdk_context_t *ctx;
    jsdk_joint_t *joints[MAX_AXES];
    jsdk_status_t status;
    struct timespec wakeup_time;
    int home_captured[MAX_AXES] = {0};
    int32_t home_pos[MAX_AXES] = {0};
    double traj_time = 0.0;
    unsigned int counter = 0;
    int ret = 0;
    int num_axes = 2;
    int print_header = 1;
    unsigned int i;

    if (argc >= 2) {
        num_axes = atoi(argv[1]);
        if (num_axes < 1) num_axes = 1;
        if (num_axes > MAX_AXES) num_axes = MAX_AXES;
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    jsdk_context_config_default(&ctx_config);
    ctx_config.master_index = 0;
    ctx_config.period_ns = PERIOD_NS;
    ctx_config.max_joints = (unsigned int)num_axes;

    ctx = jsdk_context_create(&ctx_config);
    if (!ctx) {
        fprintf(stderr, "create ctx failed\n");
        return EXIT_FAILURE;
    }

    for (i = 0; i < (unsigned int)num_axes; i++) {
        jsdk_joint_config_t jcfg;

        memset(&jcfg, 0, sizeof(jcfg));
        jcfg.alias = 0;
        jcfg.position = (uint16_t)i;  /* 环上第 0,1,2,... 个从站 */
        jcfg.profile_name = "./ECAT_CIA402.xml";

        status = jsdk_context_add_joint(ctx, &jcfg, &joints[i]);
        if (status != JSDK_OK) {
            fprintf(stderr, "add_joint[%u] at position %u: %s (%s)\n",
                    i, (unsigned int)jcfg.position,
                    jsdk_status_string(status),
                    jsdk_context_last_error(ctx));
            jsdk_context_destroy(ctx);
            return -1;
        }
    }

    status = jsdk_context_activate(ctx);
    if (status != JSDK_OK) {
        fprintf(stderr, "activate: %s (%s)\n",
                jsdk_status_string(status), jsdk_context_last_error(ctx));
        jsdk_context_destroy(ctx);
        return -1;
    }

    for (i = 0; i < (unsigned int)num_axes; i++)
        jsdk_joint_request_enable(joints[i], JSDK_MODE_CSP);

    setup_realtime();

    printf("=== Multi-Axis CSP Example ===\n");
    printf("axes=%d  period=%u ns\n", num_axes, PERIOD_NS);

    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
    wakeup_time.tv_sec += 1;
    wakeup_time.tv_nsec = 0;

    while (running) {
        ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                &wakeup_time, NULL);
        if (ret) { fprintf(stderr, "nanosleep: %s\n", strerror(ret)); break; }

        status = jsdk_context_cycle_begin(ctx, monotonic_time_ns());
        if (status != JSDK_OK) break;

        for (i = 0; i < (unsigned int)num_axes; i++) {
            jsdk_joint_feedback_t f;
            jsdk_joint_get_feedback(joints[i], &f);

            if (jsdk_joint_is_enabled(joints[i])) {
                int32_t target;

                if (!home_captured[i]) {
                    home_pos[i] = f.actual_position;
                    home_captured[i] = 1;
                    printf("Axis %u enabled, home=%d\n", i, home_pos[i]);
                }

                /* 各轴错开相位避免同频共振 */
                double phase = i * PI / (double)num_axes;
                double dt = PERIOD_NS / (double)NSEC_PER_SEC;
                traj_time += dt;
                target = home_pos[i] + (int32_t)(AMPLITUDE *
                        sin(2.0 * PI * FREQ_HZ * traj_time + phase));
                jsdk_joint_set_target_position(joints[i], target);
            } else {
                home_captured[i] = 0;
            }
        }
        traj_time -= (num_axes > 0)
            ? (num_axes - 1) * PERIOD_NS / (double)NSEC_PER_SEC : 0.0;

        if (!counter--) {
            jsdk_bus_state_t bus;
            counter = NSEC_PER_SEC / PERIOD_NS;
            jsdk_context_get_bus_state(ctx, &bus);

            if (print_header) {
                printf(" WC=%u  slaves=%u  al=0x%02X  link=%s\n",
                       bus.working_counter, bus.slaves_responding,
                       bus.al_states,
                       bus.link_up ? "up" : "DOWN");
                print_header = 0;
            }

            for (i = 0; i < (unsigned int)num_axes; i++) {
                jsdk_joint_feedback_t f;
                jsdk_joint_get_feedback(joints[i], &f);
                printf("  axis%u: %s  status=0x%04X  mode=%d  pos=%d"
                       "  online=%d\n",
                       i, jsdk_axis_state_string(f.axis_state),
                       f.statusword, f.mode_display,
                       f.actual_position, f.online);
            }
        }

        status = jsdk_context_cycle_end(ctx);
        if (status != JSDK_OK) break;

        add_period(&wakeup_time);
    }

    printf("\nStopping...\n");
    for (i = 0; i < (unsigned int)num_axes; i++)
        jsdk_joint_request_disable(joints[i]);
    jsdk_context_cycle_begin(ctx, monotonic_time_ns());
    jsdk_context_cycle_end(ctx);
    jsdk_context_destroy(ctx);
    return ret ? EXIT_FAILURE : EXIT_SUCCESS;
}
