/*****************************************************************************
 *
 *  守护兽关节 SDK 异步 SDO 诊断示例
 *
 *  验证运行期通过异步 SDO 读写关键对象，不中断周期。
 *  包括：读取实际位置/速度、故障码、温度，以及调增益。
 *
 *  用法:
 *    ./sdo_diag                      # 读取诊断对象 + CSP 运行
 *    ./sdo_diag --tune               # 额外演示写增益（位置环 Kp * 0.9）
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

/* ---- SDO 轮询：一次只发一个请求，读完再发下一个 ---- */

typedef enum { S_IDLE, S_BUSY, S_DONE } sdo_seq_state_t;

/* 下面两个大开关循环：enable 前轮询状态机，enable 后轮询 SDO */
int main(int argc, char **argv)
{
    jsdk_context_config_t ctx_config;
    jsdk_joint_config_t joint_config;
    jsdk_context_t *ctx;
    jsdk_joint_t *joint = NULL;
    jsdk_status_t status;
    struct timespec wakeup_time;
    unsigned int slow_counter = 0;
    int ret = 0;
    int tune_mode = 0;
    int tune_done = 0;

    /* SDO 句柄 */
    jsdk_sdo_handle_t h_actual_pos;
    jsdk_sdo_handle_t h_actual_vel;
    jsdk_sdo_handle_t h_error_code;
    jsdk_sdo_handle_t h_mosfet_temp;
    jsdk_sdo_handle_t h_motor_temp;
    jsdk_sdo_handle_t h_pos_kp;

    /* SDO 轮询控制 */
    int sdo_seq = 0;          /* 0=读 pos, 1=读 vel, 2=读 error, 3=temperature, 4=pos_kp */
    sdo_seq_state_t sdo_state = S_IDLE;

    for (int i = 1; i < argc; i++)
        if (!strcmp(argv[i], "--tune")) tune_mode = 1;

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

    /* 激活前创建所有 SDO 请求 */
    h_actual_pos  = jsdk_joint_sdo_create(joint, 0x6064, 0, 4);
    h_actual_vel  = jsdk_joint_sdo_create(joint, 0x606C, 0, 4);
    h_error_code  = jsdk_joint_sdo_create(joint, 0x603F, 0, 2);
    h_mosfet_temp = jsdk_joint_sdo_create(joint, 0x200B, 1, 2);
    h_motor_temp  = jsdk_joint_sdo_create(joint, 0x200B, 2, 2);
    h_pos_kp      = jsdk_joint_sdo_create(joint, 0x2008, 4, 2);

    if (h_actual_pos < 0 || h_actual_vel < 0 || h_error_code < 0) {
        fprintf(stderr, "SDO create failed\n");
        jsdk_context_destroy(ctx); return -1;
    }
    printf("SDO requests created: pos=%d vel=%d err=%d "
           "mosfet_temp=%d motor_temp=%d pos_kp=%d\n",
           h_actual_pos, h_actual_vel, h_error_code,
           h_mosfet_temp, h_motor_temp, h_pos_kp);

    status = jsdk_context_activate(ctx);
    if (status != JSDK_OK) {
        fprintf(stderr, "activate: %s (%s)\n",
                jsdk_status_string(status), jsdk_context_last_error(ctx));
        jsdk_context_destroy(ctx); return -1;
    }

    jsdk_joint_request_enable(joint, JSDK_MODE_CSP);
    setup_realtime();

    printf("=== SDO Diagnostic Example ===\n");
    printf("period=%u ns  tune=%s\n", PERIOD_NS,
            tune_mode ? "YES" : "NO");

    /* Do not idle 1s after activate — DC needs application_time immediately. */
    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
    add_period(&wakeup_time);

    while (running) {
        jsdk_joint_feedback_t f;

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
            jsdk_joint_set_target_position(joint, f.actual_position);

            /* ---- SDO 轮询：一次只发一个请求 ---- */
            switch (sdo_state) {
            case S_IDLE: {
                jsdk_sdo_handle_t h = -1;
                switch (sdo_seq) {
                case 0: h = h_actual_pos;  break;
                case 1: h = h_actual_vel;  break;
                case 2: h = h_error_code;  break;
                case 3: h = h_mosfet_temp; break;
                case 4: h = h_motor_temp;  break;
                case 5: h = h_pos_kp;      break;
                }
                if (h >= 0) jsdk_joint_sdo_read(joint, h);
                sdo_state = S_BUSY;
                break;
            }
            case S_BUSY: {
                jsdk_sdo_state_t st;
                jsdk_sdo_handle_t h = -1;
                switch (sdo_seq) {
                case 0: h = h_actual_pos;  break;
                case 1: h = h_actual_vel;  break;
                case 2: h = h_error_code;  break;
                case 3: h = h_mosfet_temp; break;
                case 4: h = h_motor_temp;  break;
                case 5: h = h_pos_kp;      break;
                }
                st = jsdk_joint_sdo_state(joint, h);
                if (st == JSDK_SDO_SUCCESS || st == JSDK_SDO_ERROR) {
                    /* 读 pos_kp 后尝试写 */
                    if (sdo_seq == 5 && tune_mode && !tune_done) {
                        uint16_t *data = (uint16_t *)
                            jsdk_joint_sdo_data(joint, h_pos_kp);
                        if (data && st == JSDK_SDO_SUCCESS) {
                            /* 读取成功：降低 10% */
                            uint16_t new_kp = (uint16_t)
                                ((*data) * 9 / 10);
                            *(uint16_t *)jsdk_joint_sdo_data(
                                    joint, h_pos_kp) = new_kp;
                            jsdk_joint_sdo_write(joint, h_pos_kp);
                            printf("[tune] pos_kp: %u -> %u\n",
                                    *data, new_kp);
                            tune_done = 1;
                        }
                        sdo_state = S_BUSY;
                    } else {
                        sdo_seq = (sdo_seq + 1) % 6;
                        sdo_state = S_IDLE;
                    }
                }
                break;
            }
            default: break;
            }
            /* ---- SDO 轮询结束 ---- */

        } else {
            jsdk_joint_set_target_position(joint, f.actual_position);
        }

        /* 慢速打印 SDO 读取结果 (每 2 秒) */
        if (!slow_counter--) {
            slow_counter = NSEC_PER_SEC * 2 / PERIOD_NS;

            uint16_t err_code = 0;
            uint16_t mosfet_t = 0, motor_t = 0;
            uint16_t pos_kp = 0;
            int32_t sdo_pos = 0, sdo_vel = 0;

            if (jsdk_joint_sdo_state(joint, h_error_code)
                    == JSDK_SDO_SUCCESS) {
                uint16_t *d = (uint16_t *)
                    jsdk_joint_sdo_data(joint, h_error_code);
                if (d) err_code = *d;
            }
            if (jsdk_joint_sdo_state(joint, h_actual_pos)
                    == JSDK_SDO_SUCCESS) {
                int32_t *d = (int32_t *)
                    jsdk_joint_sdo_data(joint, h_actual_pos);
                if (d) sdo_pos = *d;
            }
            if (jsdk_joint_sdo_state(joint, h_actual_vel)
                    == JSDK_SDO_SUCCESS) {
                int32_t *d = (int32_t *)
                    jsdk_joint_sdo_data(joint, h_actual_vel);
                if (d) sdo_vel = *d;
            }
            if (jsdk_joint_sdo_state(joint, h_mosfet_temp)
                    == JSDK_SDO_SUCCESS) {
                uint16_t *d = (uint16_t *)
                    jsdk_joint_sdo_data(joint, h_mosfet_temp);
                if (d) mosfet_t = *d;
            }
            if (jsdk_joint_sdo_state(joint, h_motor_temp)
                    == JSDK_SDO_SUCCESS) {
                uint16_t *d = (uint16_t *)
                    jsdk_joint_sdo_data(joint, h_motor_temp);
                if (d) motor_t = *d;
            }
            if (jsdk_joint_sdo_state(joint, h_pos_kp)
                    == JSDK_SDO_SUCCESS) {
                uint16_t *d = (uint16_t *)
                    jsdk_joint_sdo_data(joint, h_pos_kp);
                if (d) pos_kp = *d;
            }

            printf("SDO: err=0x%04X  pos=%d  vel=%d  "
                   "mosfet_t=%.1fC  motor_t=%.1fC  pos_kp=0x%04X  "
                   "PDO_pos=%d\n",
                   err_code, sdo_pos, sdo_vel,
                   mosfet_t * 0.1, motor_t * 0.1,
                   pos_kp, f.actual_position);
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
