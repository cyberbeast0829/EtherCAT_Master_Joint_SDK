/*****************************************************************************
 *
 *  守护兽关节 SDK CST 诊断示例
 *
 *  现场注意：
 *    - CST 恒力矩会持续加速；SDK 对 CST 自动打开 0x200A:12 限速并设 0x6080
 *    - 0x6071 目标：0.1% 额定（额定 10 N·m 时，100 = 1 N·m）
 *    - 0x6077 反馈可能是 Iq·Kt·gear，与 0x6071 数值不一定同量纲
 *    - --fw 选择 drive_model 的 DC/周期（电机应用版本，非 CoE 0x100A）
 *
 *  用法（固件与 vmax 由命令行指定，无代码默认）:
 *    sudo chrt -f 80 ./build/diag_cst_single --fw0 8.1.50 --nm 1 \
 *         --vmax 50000 --seconds 10
 *    sudo chrt -f 80 ./build/diag_cst_single --fw0 8.1.50 --hold \
 *         --vmax 50000 --seconds 8
 *
 *  --nm N：目标力矩 N·m（1→1 N·m，2→2 N·m）。内部再换成 0x6071 的
 *  0.1% 额定单位（额定按 10 N·m：trq = nm / 0.01）。
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
 #include <sys/stat.h>
 
 #include <joint_sdk/dc_timing_conf.h>
 #include <joint_sdk/joint_sdk.h>
 
 #define PERIOD_NS_DEFAULT 1000000u
 #define NSEC_PER_SEC 1000000000L
 #define MAX_SAFE_STACK (8 * 1024)
 #define TRQ_AMPLITUDE 50
 #define TRQ_FREQ_HZ 0.25
 #define PI 3.14159265358979323846
 #define RAMP_SEC 2.0
 #define HOLD_SETTLE_SEC 2.0
 /* |act_vel| above this after settle → treat as runaway FAIL (CST) */
 #define VEL_RUNAWAY 200000
 #define HOLD_VEL_OK 2000
 #define TRACK_TOL_ABS 80
 /* 额定力矩 10 N·m 时，0x6071 每 1 个单位 = 0.1% 额定 = 0.01 N·m */
 #define RATED_NM 10.0
 #define NM_PER_TRQ_UNIT 0.01
 
 static volatile sig_atomic_t running = 1;
 static uint32_t g_period_ns = PERIOD_NS_DEFAULT;
 static uint32_t g_vmax = 0;
 
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
     param.sched_priority = 80;
     if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
         fprintf(stderr,
                 "warning: SCHED_FIFO 80 unavailable (%s); "
                 "use: sudo chrt -f 80 ./build/diag_cst_single ...\n",
                 strerror(errno));
     } else {
         printf("sched=SCHED_FIFO priority=80\n");
     }
     if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
         fprintf(stderr, "warning: mlockall failed: %s\n", strerror(errno));
     stack_prefault();
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
  * Soft-ramp to --nm, then hold for the whole run. Near vmax, ease torque
  * so Sync0 is not slammed into 0x6080; when speed falls, restore slowly
  * so the shaft keeps turning (not a one-shot pulse).
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
         scale = scale + 0.02; /* restore quickly below soft band */
     if (scale > 1.0)
         scale = 1.0;
     if (scale < 0.0)
         scale = 0.0;
     *scale_io = scale;
     return (int16_t)lround((double)cmd * scale);
 }
 
 /*
  * CST must not cut the bus while still applying torque — coast forever.
  * 1) torque=0 while still CST  2) CSV vel=0 brake  3) disable ladder
  * (destroy/deactivate still sends more zero frames).
  */
 static void cst_controlled_stop(jsdk_context_t *ctx, jsdk_joint_t *joint,
         struct timespec *wakeup_time)
 {
     unsigned int i;
     unsigned int n_zero;
     unsigned int n_brake;
     unsigned int n_dis;
     jsdk_status_t st;
 
     /* Durations in ns so 2 ms periods do not truncate to 0 cycles. */
     n_zero = 600000000u / g_period_ns;    /* ~0.6 s */
     n_brake = 1500000000u / g_period_ns;  /* ~1.5 s */
     n_dis = 400000000u / g_period_ns;     /* ~0.4 s */
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
         st = jsdk_context_cycle_begin(ctx, monotonic_time_ns());
         if (st != JSDK_OK)
             break;
         jsdk_joint_set_target_torque(joint, 0);
         jsdk_joint_set_target_velocity(joint, 0);
         jsdk_context_cycle_end(ctx);
         add_period(wakeup_time);
     }
 
     printf("Stopping: CSV vel=0 brake (%.1f s)...\n",
            (double)n_brake * g_period_ns / 1e9);
     jsdk_joint_request_enable(joint, JSDK_MODE_CSV);
     for (i = 0; i < n_brake; i++) {
         clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, wakeup_time, NULL);
         st = jsdk_context_cycle_begin(ctx, monotonic_time_ns());
         if (st != JSDK_OK)
             break;
         jsdk_joint_set_target_velocity(joint, 0);
         jsdk_joint_set_target_torque(joint, 0);
         jsdk_context_cycle_end(ctx);
         add_period(wakeup_time);
     }
 
     printf("Stopping: disable...\n");
     jsdk_joint_request_disable(joint);
     for (i = 0; i < n_dis; i++) {
         clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, wakeup_time, NULL);
         st = jsdk_context_cycle_begin(ctx, monotonic_time_ns());
         if (st != JSDK_OK)
             break;
         jsdk_joint_set_target_velocity(joint, 0);
         jsdk_joint_set_target_torque(joint, 0);
         jsdk_context_cycle_end(ctx);
         add_period(wakeup_time);
     }
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
     int fixed_nm_mode = 0;
     double target_nm = 0.0;
     int16_t trq = 0; /* 内部：0x6071 单位（0.1% 额定），由 --nm 换算 */
     int print_header = 1;
     int seconds = 0;
     const char *log_path = NULL;
     FILE *log_fp = NULL;
     uint64_t t0_ns = 0;
     uint64_t enabled_t0_ns = 0;
     uint64_t first_enable_ns = 0; /* never reset on bus recovery */
     int enabled_seen = 0;
     int ok_samples = 0;
     int settled_samples = 0;
     int runaway = 0;
     int interrupted = 0;
     int bus_ok_streak = 0;
     int need_reramp = 0;
     int wc_drop_secs = 0;
     int16_t peak_tgt = 0;
     int32_t peak_abs_vel = 0;
     double trq_scale = 1.0;
     unsigned int bus_recover_cycles;
     int16_t last_tgt = 0;
     int16_t last_act = 0;
     int32_t last_vel = 0;
     uint32_t vmax = 0;
     int vmax_set = 0;
     double ramp_s = RAMP_SEC;
     int use_dc = -1;
     int sync0_shift_set = 0;
     int32_t sync0_shift_ns = 0;
     int sync0_cycle_set = 0;
     uint32_t sync0_cycle_ns = 0;
     int period_set = 0;
     uint32_t period_ns = 0;
     const char *fw_version = NULL; /* required via --fw / --fw0 */
     const char *model_name = NULL;
     const jsdk_drive_model_t *drive_model = NULL;
 
     for (int i = 1; i < argc; i++) {
         if (!strcmp(argv[i], "--hold")) {
             hold_mode = 1;
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
         } else if (!strcmp(argv[i], "--period") && i + 1 < argc) {
             period_set = 1;
             period_ns = (uint32_t)strtoul(argv[++i], NULL, 0);
         } else if ((!strcmp(argv[i], "--fw") || !strcmp(argv[i], "--fw0"))
                 && i + 1 < argc) {
             fw_version = argv[++i];
         } else if (!strcmp(argv[i], "--model") && i + 1 < argc) {
             model_name = argv[++i];
         } else if (!strcmp(argv[i], "--nm") && i + 1 < argc) {
             fixed_nm_mode = 1;
             target_nm = atof(argv[++i]);
             /* --nm 1 → 1 N·m → trq=100 @ 10 N·m rated */
             trq = (int16_t)lround(target_nm / NM_PER_TRQ_UNIT);
             /* Soft start; then hold --nm until controlled stop. */
             ramp_s = 0.5 + 0.15 * fabs(target_nm);
             if (ramp_s > 2.0)
                 ramp_s = 2.0;
         } else if (!strcmp(argv[i], "--vmax") && i + 1 < argc) {
             vmax_set = 1;
             vmax = (uint32_t)strtoul(argv[++i], NULL, 0);
         } else if (!strcmp(argv[i], "--seconds") && i + 1 < argc) {
             seconds = atoi(argv[++i]);
         } else if (!strcmp(argv[i], "--log") && i + 1 < argc) {
             log_path = argv[++i];
         } else if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) {
             fprintf(stderr,
                     "Usage: %s --fw0 VER --vmax N [--hold|--nm N]\n"
                     "          [--fw VER|--model NAME] "
                     "[--dc|--no-dc] [--sync0-shift N]\n"
                     "          [--sync0-cycle ns] [--period ns]\n"
                     "          [--seconds N] [--log f]\n"
                     "  Prefer: sudo chrt -f 80 %s --fw0 8.1.50 "
                     "--nm 1 --vmax 50000 --seconds 10\n"
                     "  --fw/--fw0 and --vmax required "
                     "(no built-in defaults).\n",
                     argv[0], argv[0]);
             return EXIT_SUCCESS;
         }
     }
 
     if (!fw_version && !model_name) {
         fprintf(stderr,
                 "need --fw / --fw0 <ver> or --model <name>\n"
                 "  e.g. sudo chrt -f 80 %s --fw0 8.1.50 "
                 "--nm 1 --vmax 50000 --seconds 10\n",
                 argv[0]);
         return EXIT_FAILURE;
     }
     if (!vmax_set || vmax == 0) {
         fprintf(stderr,
                 "need --vmax <counts/s> (0x6080 max motor speed)\n"
                 "  e.g. sudo chrt -f 80 %s --fw0 8.1.50 "
                 "--nm 1 --vmax 50000 --seconds 10\n",
                 argv[0]);
         return EXIT_FAILURE;
     }
 
     signal(SIGINT, on_signal);
     signal(SIGTERM, on_signal);
 
     if (model_name)
         drive_model = jsdk_drive_model_by_name(model_name);
     if (!drive_model && fw_version)
         drive_model = jsdk_drive_model_find(fw_version, 0x00080117u);
     if (!drive_model) {
         fprintf(stderr, "unknown drive model (fw=%s model=%s)\n",
                 fw_version ? fw_version : "-",
                 model_name ? model_name : "-");
         return EXIT_FAILURE;
     }
     if (!fw_version)
         fw_version = drive_model->fw_match;
 
     jsdk_context_config_default(&ctx_config);
     ctx_config.master_index = 0;
     ctx_config.period_ns = PERIOD_NS_DEFAULT;
     ctx_config.max_joints = 1;
     jsdk_context_config_apply_drive_model(&ctx_config, drive_model);
     (void)jsdk_dc_timing_conf_apply_fw(&ctx_config, fw_version);
     {
         uint32_t conf_period = jsdk_dc_timing_conf_period_ns();

         if (conf_period)
             ctx_config.period_ns = conf_period;
     }
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
     if (period_set && period_ns)
         ctx_config.period_ns = period_ns;
     if (!ctx_config.period_ns)
         ctx_config.period_ns = PERIOD_NS_DEFAULT;
     /* DC period/shift: drive_model + dc_timing.conf, optional CLI override. */
     g_period_ns = ctx_config.period_ns;
 
     bus_recover_cycles = 500000000u / g_period_ns; /* ~500 ms good WC */
     if (bus_recover_cycles < 80)
         bus_recover_cycles = 80;
 
     ctx = jsdk_context_create(&ctx_config);
     if (!ctx) {
         fprintf(stderr, "failed to create SDK context\n");
         return EXIT_FAILURE;
     }
 
     memset(&joint_config, 0, sizeof(joint_config));
     joint_config.alias = 0;
     joint_config.position = 0;
     joint_config.profile_name = JSDK_PROFILE_CYBERBEAST_JOINT_MODULE;
     joint_config.firmware_version = fw_version;
     joint_config.drive_model_name = drive_model->name;
     /*
      * --hold: CST+0 力矩会放飞；用 CSV 目标速度=0 主动制动。
      */
     if (hold_mode)
         joint_config.initial_mode = JSDK_MODE_CSV;
     else
         joint_config.initial_mode = JSDK_MODE_CST;
     joint_config.max_motor_velocity = vmax;
     g_vmax = vmax;
 
     status = jsdk_context_add_joint(ctx, &joint_config, &joint);
     if (status != JSDK_OK) {
         fprintf(stderr, "add_joint: %s (%s)\n",
                 jsdk_status_string(status), jsdk_context_last_error(ctx));
         jsdk_context_destroy(ctx);
         return EXIT_FAILURE;
     }
 
     printf("drive_model=%s fw=%s use_dc=%d period=%u sync0=%u "
            "shift=%d wait_ms=%u\n",
            drive_model->name,
            fw_version,
            ctx_config.use_dc,
            ctx_config.period_ns,
            ctx_config.sync0_cycle_ns ? ctx_config.sync0_cycle_ns
                                      : ctx_config.period_ns,
            (int)ctx_config.sync0_shift_ns,
            ctx_config.wait_before_safeop_ms);
 
     setup_realtime();
 
     status = jsdk_context_activate(ctx);
     if (status != JSDK_OK) {
         fprintf(stderr, "activate: %s (%s)\n",
                 jsdk_status_string(status), jsdk_context_last_error(ctx));
         jsdk_context_destroy(ctx);
         return EXIT_FAILURE;
     }
 
     /* Settle OP+WC before CiA402 enable (same as CSV diag). */
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
         max_cycles = 15000u / period_ms;
 
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
             jsdk_joint_set_target_torque(joint, 0);
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
                     "(good=%u/%u tries=%u)\n",
                     good, need_cycles, tries);
             jsdk_context_destroy(ctx);
             return EXIT_FAILURE;
         }
         printf("bus settle ok (%u cycles stable OP+WC)\n", good);
     }
 
     if (hold_mode)
         jsdk_joint_request_enable(joint, JSDK_MODE_CSV);
     else
         jsdk_joint_request_enable(joint, JSDK_MODE_CST);
 
     if (log_path) {
         char dirbuf[512];
         char *slash;
 
         snprintf(dirbuf, sizeof(dirbuf), "%s", log_path);
         slash = strrchr(dirbuf, '/');
         if (slash) {
             *slash = '\0';
             if (dirbuf[0])
                 mkdir(dirbuf, 0755);
         }
         log_fp = fopen(log_path, "w");
         if (!log_fp) {
             fprintf(stderr, "fopen %s: %s\n", log_path, strerror(errno));
             jsdk_context_destroy(ctx);
             return EXIT_FAILURE;
         }
         fprintf(log_fp,
                 "t_s,status,sw,tgt_trq,act_trq,act_vel,mode,act_pos,"
                 "wc,slv,al,ok\n");
     }
 
     printf("=== Diagnostic CST Example ===\n");
     printf("DC=%s", ctx_config.use_dc ? "ON(Sync0)" : "OFF(free-run)");
     if (ctx_config.use_dc) {
         uint32_t s0 = ctx_config.sync0_cycle_ns
                 ? ctx_config.sync0_cycle_ns : ctx_config.period_ns;
         printf(" task=%u ns sync0=%u ns shift=%d ns",
                ctx_config.period_ns, s0, (int)ctx_config.sync0_shift_ns);
     }
     printf("\n");
     if (hold_mode) {
         printf("PREOP: mode=CSV(9)  (zero-velocity hold / brake)\n");
         printf("period=%u ns  CSV target velocity = 0\n", g_period_ns);
     } else {
         printf("PREOP: mode=CST(10) input_mode=PASSTHROUGH(1) "
                "torque_vel_limit=ON vmax=%u\n",
                vmax);
         if (fixed_nm_mode) {
             printf("period=%u ns  --nm %.3f → %.3f N·m → trq(0x6071)=%d  "
                    "soft-ramp %.1fs then hold until stop\n",
                    g_period_ns, target_nm, target_nm, (int)trq, ramp_s);
             printf("note: act_trq(0x6077) may use different units than tgt\n");
         } else {
             printf("period=%u ns  sine amplitude=%d (0.1%% rated)\n",
                    g_period_ns, TRQ_AMPLITUDE);
         }
     }
     if (seconds > 0)
         printf("auto-stop after %d s\n", seconds);
 
     clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
     add_period(&wakeup_time);
     t0_ns = monotonic_time_ns();
 
     while (running) {
         jsdk_joint_feedback_t f;
         int16_t trq_cmd = 0;
         uint64_t now;
         double t_s;
         double enabled_s = 0.0;
 
         ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                 &wakeup_time, NULL);
         if (ret) {
             if (ret == EINTR)
                 interrupted = 1;
             else
                 fprintf(stderr, "clock_nanosleep: %s\n", strerror(ret));
             running = 0;
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
 
         {
             jsdk_bus_state_t bus_now;
             int bus_ok;
 
             jsdk_context_get_bus_state(ctx, &bus_now);
             bus_ok = (bus_now.wc_state == 2) && (bus_now.al_states & 0x08);
 
             /*
              * Bad WC/AL: torque must stay 0. After recovery wait
              * bus_recover_cycles then soft-ramp again (no step to full Nm).
              */
             if (!bus_ok) {
                 bus_ok_streak = 0;
                 need_reramp = 1;
                 jsdk_joint_set_target_torque(joint, 0);
                 jsdk_joint_set_target_velocity(joint, 0);
                 trq_cmd = 0;
             } else if (jsdk_joint_is_enabled(joint)) {
                 if (!enabled_seen) {
                     enabled_seen = 1;
                     enabled_t0_ns = now;
                     first_enable_ns = now;
                 }
 
                 if (bus_ok_streak < (int)bus_recover_cycles) {
                     bus_ok_streak++;
                     jsdk_joint_set_target_torque(joint, 0);
                     jsdk_joint_set_target_velocity(joint, 0);
                     trq_cmd = 0;
                     if (bus_ok_streak >= (int)bus_recover_cycles) {
                         enabled_t0_ns = now;
                         trq_scale = 1.0;
                         if (need_reramp)
                             printf("bus recovered — soft-ramp torque again\n");
                         need_reramp = 0;
                     }
                 } else {
                     int32_t abs_v;
 
                     enabled_s = (double)(now - enabled_t0_ns)
                             / (double)NSEC_PER_SEC;
                     abs_v = f.actual_velocity < 0
                             ? -f.actual_velocity : f.actual_velocity;
                     if (abs_v > peak_abs_vel)
                         peak_abs_vel = abs_v;
 
                     if (hold_mode) {
                         jsdk_joint_set_target_velocity(joint, 0);
                         jsdk_joint_set_target_torque(joint, 0);
                         trq_cmd = 0;
                     } else if (fixed_nm_mode) {
                         trq_cmd = soft_ramp_i16(trq, enabled_s, ramp_s);
                         trq_cmd = speed_limited_trq(trq_cmd,
                                 f.actual_velocity, &trq_scale);
                         jsdk_joint_set_target_torque(joint, trq_cmd);
                     } else {
                         double dt = (double)g_period_ns
                                 / (double)NSEC_PER_SEC;
                         traj_time += dt;
                         trq_cmd = (int16_t)(TRQ_AMPLITUDE *
                                 sin(2.0 * PI * TRQ_FREQ_HZ * traj_time));
                         trq_cmd = speed_limited_trq(trq_cmd,
                                 f.actual_velocity, &trq_scale);
                         jsdk_joint_set_target_torque(joint, trq_cmd);
                     }
                     if (trq_cmd > peak_tgt)
                         peak_tgt = trq_cmd;
                     else if (-trq_cmd > peak_tgt)
                         peak_tgt = (int16_t)(-trq_cmd);
                 }
             } else {
                 jsdk_joint_set_target_torque(joint, 0);
                 if (hold_mode)
                     jsdk_joint_set_target_velocity(joint, 0);
             }
         }
 
         if (!counter--) {
             jsdk_bus_state_t bus;
             int settled;
             int ok;
             int32_t abs_vel;
             double settle_need;
 
             counter = NSEC_PER_SEC / g_period_ns;
             jsdk_context_get_bus_state(ctx, &bus);
             settle_need = hold_mode ? HOLD_SETTLE_SEC : ramp_s;
             settled = enabled_seen && enabled_s >= settle_need
                     && bus_ok_streak >= (int)bus_recover_cycles;
             abs_vel = f.actual_velocity < 0
                     ? -f.actual_velocity : f.actual_velocity;
             if (settled && !hold_mode && abs_vel > VEL_RUNAWAY)
                 runaway = 1;
 
             if (enabled_seen && first_enable_ns &&
                     (double)(now - first_enable_ns) /
                             (double)NSEC_PER_SEC >= 1.0 &&
                     (bus.wc_state != 2 || bus.working_counter == 0 ||
                      (bus.al_states & 0x0f) != 0x08)) {
                 wc_drop_secs++;
             }
 
             if (hold_mode) {
                 ok = settled && abs_vel <= HOLD_VEL_OK
                         && f.mode_display == 9
                         && bus.wc_state == 2
                         && (bus.al_states & 0x0f) == 0x08;
             } else {
                 /* Must be moving under CST, not just bus-alive at standstill. */
                 ok = settled && !runaway && f.mode_display == 10
                         && abs_vel > 2000
                         && bus.wc_state == 2
                         && (bus.al_states & 0x0f) == 0x08;
             }
 
             if (settled) {
                 settled_samples++;
                 if (ok)
                     ok_samples++;
             }
 
             last_tgt = trq_cmd;
             last_act = f.actual_torque;
             last_vel = f.actual_velocity;
 
             if (print_header) {
                 printf("%-8s %-8s %-8s %-8s %-8s "
                        "%-6s %-10s %4s %5s %5s %5s\n",
                        "axis", "status", "tgt_trq", "act_trq",
                        "act_vel", "mode", "act_pos",
                        "wc", "slv", "al", "ok");
                 print_header = 0;
             }
 
             printf("%-8s %04X     %-8d %-8d %-8d "
                    "%-6d %-10d %4u %4u %02X %5s\n",
                    jsdk_axis_state_string(f.axis_state),
                    f.statusword,
                    trq_cmd,
                    f.actual_torque,
                    f.actual_velocity,
                    f.mode_display,
                    f.actual_position,
                    bus.working_counter,
                    bus.slaves_responding,
                    bus.al_states,
                    ok ? "YES" : (settled ? "no" : "ramp"));
 
             if (log_fp) {
                 fprintf(log_fp,
                         "%.3f,%s,%04X,%d,%d,%d,%d,%d,%u,%u,%02X,%d\n",
                         t_s,
                         jsdk_axis_state_string(f.axis_state),
                         f.statusword,
                         trq_cmd,
                         f.actual_torque,
                         f.actual_velocity,
                         f.mode_display,
                         f.actual_position,
                         bus.working_counter,
                         bus.slaves_responding,
                         bus.al_states,
                         ok);
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
 
     printf("\n");
     if (hold_mode) {
         printf("Stopping...\n");
         jsdk_joint_request_disable(joint);
         jsdk_context_cycle_begin(ctx, monotonic_time_ns());
         jsdk_context_cycle_end(ctx);
     } else {
         cst_controlled_stop(ctx, joint, &wakeup_time);
     }
     jsdk_context_destroy(ctx);
     if (log_fp)
         fclose(log_fp);
 
     printf("summary: tgt=%d act_trq=%d act_vel=%d ok=%d/%d "
            "runaway=%d wc_drops=%d enabled=%d peak_tgt=%d peak_vel=%d\n",
            (int)last_tgt, (int)last_act, (int)last_vel,
            ok_samples, settled_samples, runaway, wc_drop_secs, enabled_seen,
            (int)peak_tgt, (int)peak_abs_vel);
 
     if (hold_mode) {
         int32_t abs_last = last_vel < 0 ? -last_vel : last_vel;
         if (ret || !enabled_seen || ok_samples < 1 ||
                 abs_last > HOLD_VEL_OK) {
             fprintf(stderr,
                     "FAIL: hold not stationary (|act_vel| should be <= %d). "
                     "Last act_vel=%d ok=%d/%d\n",
                     HOLD_VEL_OK, (int)last_vel, ok_samples, settled_samples);
             return EXIT_FAILURE;
         }
         printf("PASS: hold stationary (CSV vel=0, |act_vel|<=%d)\n",
                HOLD_VEL_OK);
         return EXIT_SUCCESS;
     }
 
     if (runaway) {
         fprintf(stderr,
                 "FAIL: velocity runaway (|act_vel|>%d). "
                 "Check 0x200A:12 and 0x6080.\n", VEL_RUNAWAY);
         return EXIT_FAILURE;
     }
     if (interrupted) {
         fprintf(stderr,
                 "Interrupted (Ctrl+C) — stop sequence done. "
                 "Re-run without Ctrl+C for a full PASS.\n");
         return EXIT_FAILURE;
     }
     if (wc_drop_secs > 0) {
         fprintf(stderr,
                 "FAIL: %d sample(s) with WC/AL dropout after enable. "
                 "CST Sync0 not stable under continuous torque.\n",
                 wc_drop_secs);
         return EXIT_FAILURE;
     }
     if (peak_abs_vel < 5000) {
         fprintf(stderr,
                 "FAIL: motor did not spin (peak_vel=%d). "
                 "Check CST enable / 0x6071 / brake.\n",
                 (int)peak_abs_vel);
         return EXIT_FAILURE;
     }
     if (!enabled_seen || settled_samples < 2 ||
             ok_samples * 2 < settled_samples) {
         fprintf(stderr, "FAIL: CST not stable (mode/AL/enable/WC/motion).\n");
         return EXIT_FAILURE;
     }
 
     printf("PASS: CST continuous torque, motor spun, no WC drops "
            "(act_trq unit may differ from tgt — see handbook 0x6077)\n");
     (void)TRACK_TOL_ABS;
     return EXIT_SUCCESS;
 }
 