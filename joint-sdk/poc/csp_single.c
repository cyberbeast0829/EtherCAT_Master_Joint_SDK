/*****************************************************************************
 *
 *  守护兽关节 (CyberBeast Joint Module) — 单轴 CSP PoC
 *
 *  基于 IgH EtherCAT Master 用户空间 ecrt_* API。
 *  目标：验证与本公司机器人关节的 CiA402 周期同步位置 (CSP) 交互链路。
 *
 *  本文件参考 examples/user/main.c 的生命周期写法，新增：
 *    - 按 ESI 确认的 vendor/product 校验从站
 *    - 0x1600 / 0x1A00 全功能 PDO 映射 + 0x1C12/0x1C13 分配
 *    - DC 配置 (AssignActivate=0x300, SYNC0=周期, shift=周期/4)
 *    - CiA402 DS402 状态机自动使能
 *    - 周期写目标位置 0x607A、读实际位置 0x6064
 *
 *  仅用于 PoC / 联调，非生产代码。
 *
 ****************************************************************************/

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>      /* clock_gettime() */
#include <sys/mman.h>  /* mlockall() */
#include <sched.h>     /* sched_setscheduler() */
#include <math.h>      /* sin() */

/****************************************************************************/

#include "ecrt.h"

/****************************************************************************/

/* 周期 (ns)。1 ms = 1 kHz。首版用 1 ms，远高于设备最小 DC 周期 (单轴≈41 µs)，
   给联调留足裕量；稳定后可缩短。 */
#define PERIOD_NS       (1000000U)

/* SYNC0 shift = 周期 / 4 (手册推荐，保证 SYNC0 在 SM2 事件之后)。 */
#define SYNC0_SHIFT_NS  (PERIOD_NS / 4)

#define MAX_SAFE_STACK  (8 * 1024) /* 预触发的安全栈大小 */

#define NSEC_PER_SEC    (1000000000L)
#define FREQUENCY       (NSEC_PER_SEC / PERIOD_NS)

/****************************************************************************/

/* 从站身份 (来自 ESI: ECAT_CIA402.xml)。 */
#define JOINT_ALIAS         0
#define JOINT_POSITION      0
#define CYBERBEAST_VENDOR   0x000C0B00u
#define CYBERBEAST_JOINT_PRODUCT 0x00080153u

/* DC: ESI 的 Device->Dc->OpMode "DC" AssignActivate。 */
#define DC_ASSIGN_ACTIVATE  0x0300u

/* CiA402 对象索引。 */
#define OD_CONTROLWORD      0x6040 /* U16  RxPDO */
#define OD_TARGET_POSITION  0x607A /* I32  RxPDO */
#define OD_TARGET_VELOCITY  0x60FF /* I32  RxPDO */
#define OD_TARGET_TORQUE    0x6071 /* I16  RxPDO */
#define OD_MODE_OF_OP       0x6060 /* I8   RxPDO */
#define OD_STATUSWORD       0x6041 /* U16  TxPDO */
#define OD_ACTUAL_POSITION  0x6064 /* I32  TxPDO */
#define OD_ACTUAL_VELOCITY  0x606C /* I32  TxPDO */
#define OD_ACTUAL_TORQUE    0x6077 /* I16  TxPDO */
#define OD_MODE_DISPLAY     0x6061 /* I8   TxPDO */

/* 运行模式编码 (CiA402 标准；⚠ PoC 需以实物确认本设备是否一致)。 */
#define MODE_CSP            8
#define MODE_CSV            9
#define MODE_CST            10

/* DS402 控制字命令。 */
#define CW_SHUTDOWN         0x0006
#define CW_SWITCH_ON        0x0007
#define CW_ENABLE_OP        0x000F
#define CW_FAULT_RESET      0x0080

/* DS402 状态字位掩码 (低 4 位 + bit5 + bit6) 用于状态判定。 */
#define SW_MASK             0x006F
#define SW_NOT_READY        0x0000
#define SW_SWITCH_ON_DISABLED 0x0040
#define SW_READY_TO_SWITCH_ON 0x0021
#define SW_SWITCHED_ON      0x0023
#define SW_OPERATION_ENABLED 0x0027
#define SW_FAULT            0x0008 /* bit3 故障，用 (sw & 0x0008) 判定 */

/****************************************************************************/

/* EtherCAT 句柄。 */
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {0};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {0};

static ec_slave_config_t *sc_joint = NULL;
static ec_slave_config_state_t sc_joint_state = {0};

/* 过程数据指针与各 PDO 条目偏移。 */
static uint8_t *domain1_pd = NULL;

static unsigned int off_ctrlword;
static unsigned int off_target_pos;
static unsigned int off_target_vel;
static unsigned int off_target_trq;
static unsigned int off_mode;
static unsigned int off_statusword;
static unsigned int off_actual_pos;
static unsigned int off_actual_vel;
static unsigned int off_actual_trq;
static unsigned int off_mode_disp;

/* 域条目登记 (顺序须与 SM/PDO 映射一致)。 */
static const ec_pdo_entry_reg_t domain1_regs[] = {
    {JOINT_ALIAS, JOINT_POSITION, CYBERBEAST_VENDOR, CYBERBEAST_JOINT_PRODUCT,
        OD_CONTROLWORD,     0, &off_ctrlword},
    {JOINT_ALIAS, JOINT_POSITION, CYBERBEAST_VENDOR, CYBERBEAST_JOINT_PRODUCT,
        OD_TARGET_POSITION, 0, &off_target_pos},
    {JOINT_ALIAS, JOINT_POSITION, CYBERBEAST_VENDOR, CYBERBEAST_JOINT_PRODUCT,
        OD_TARGET_VELOCITY, 0, &off_target_vel},
    {JOINT_ALIAS, JOINT_POSITION, CYBERBEAST_VENDOR, CYBERBEAST_JOINT_PRODUCT,
        OD_TARGET_TORQUE,   0, &off_target_trq},
    {JOINT_ALIAS, JOINT_POSITION, CYBERBEAST_VENDOR, CYBERBEAST_JOINT_PRODUCT,
        OD_MODE_OF_OP,      0, &off_mode},
    {JOINT_ALIAS, JOINT_POSITION, CYBERBEAST_VENDOR, CYBERBEAST_JOINT_PRODUCT,
        OD_STATUSWORD,      0, &off_statusword},
    {JOINT_ALIAS, JOINT_POSITION, CYBERBEAST_VENDOR, CYBERBEAST_JOINT_PRODUCT,
        OD_ACTUAL_POSITION, 0, &off_actual_pos},
    {JOINT_ALIAS, JOINT_POSITION, CYBERBEAST_VENDOR, CYBERBEAST_JOINT_PRODUCT,
        OD_ACTUAL_VELOCITY, 0, &off_actual_vel},
    {JOINT_ALIAS, JOINT_POSITION, CYBERBEAST_VENDOR, CYBERBEAST_JOINT_PRODUCT,
        OD_ACTUAL_TORQUE,   0, &off_actual_trq},
    {JOINT_ALIAS, JOINT_POSITION, CYBERBEAST_VENDOR, CYBERBEAST_JOINT_PRODUCT,
        OD_MODE_DISPLAY,    0, &off_mode_disp},
    {0}
};

/****************************************************************************/

/* RxPDO 0x1600 @ SM2 (输出)。顺序 + 8 位填充须与 ESI 一致。 */
static const ec_pdo_entry_info_t joint_rx_entries[] = {
    {OD_CONTROLWORD,     0, 16},
    {OD_TARGET_POSITION, 0, 32},
    {OD_TARGET_VELOCITY, 0, 32},
    {OD_TARGET_TORQUE,   0, 16},
    {OD_MODE_OF_OP,      0,  8},
    {0x0000,             0,  8}, /* ESI 中的对齐填充 */
};

/* TxPDO 0x1A00 @ SM3 (输入)。 */
static const ec_pdo_entry_info_t joint_tx_entries[] = {
    {OD_STATUSWORD,      0, 16},
    {OD_ACTUAL_POSITION, 0, 32},
    {OD_ACTUAL_VELOCITY, 0, 32},
    {OD_ACTUAL_TORQUE,   0, 16},
    {OD_MODE_DISPLAY,    0,  8},
    {0x0000,             0,  8}, /* ESI 中的对齐填充 */
};

static const ec_pdo_info_t joint_rx_pdos[] = {
    {0x1600, 6, joint_rx_entries},
};

static const ec_pdo_info_t joint_tx_pdos[] = {
    {0x1A00, 6, joint_tx_entries},
};

/* SM2=RxPDO(输出), SM3=TxPDO(输入)，与 ESI 的 SM 布局一致。 */
static const ec_sync_info_t joint_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT,  0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, joint_rx_pdos, EC_WD_ENABLE},
    {3, EC_DIR_INPUT,  1, joint_tx_pdos, EC_WD_DISABLE},
    {0xff}
};

/****************************************************************************/

/* 运行参数。 */
static volatile sig_atomic_t g_running = 1;
static unsigned int counter = 0;

/* 简单正弦轨迹参数 (指令单位，联调时按实际换算因子调整)。 */
static int32_t home_position = 0;     /* 使能瞬间锁定的起点 */
static int home_captured = 0;
static double  traj_time = 0.0;
#define TRAJ_AMPLITUDE  16384.0       /* 约 1 圈 (默认编码器分辨率 16384)，按需改 */
#define TRAJ_FREQ_HZ    0.25          /* 正弦轨迹频率 */

/****************************************************************************/

static void signal_handler(int sig)
{
    (void)sig;
    g_running = 0;
}

/****************************************************************************/

static void check_domain_state(void)
{
    ec_domain_state_t ds;
    ecrt_domain_state(domain1, &ds);

    if (ds.working_counter != domain1_state.working_counter) {
        printf("Domain: WC %u.\n", ds.working_counter);
    }
    if (ds.wc_state != domain1_state.wc_state) {
        printf("Domain: WC state %u.\n", ds.wc_state);
    }
    domain1_state = ds;
}

/****************************************************************************/

static void check_master_state(void)
{
    ec_master_state_t ms;
    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding) {
        printf("%u slave(s) responding.\n", ms.slaves_responding);
    }
    if (ms.al_states != master_state.al_states) {
        printf("AL states: 0x%02X.\n", ms.al_states);
    }
    if (ms.link_up != master_state.link_up) {
        printf("Link is %s.\n", ms.link_up ? "up" : "down");
    }
    master_state = ms;
}

/****************************************************************************/

static void check_slave_state(void)
{
    ec_slave_config_state_t s;
    ecrt_slave_config_state(sc_joint, &s);

    if (s.al_state != sc_joint_state.al_state) {
        printf("Joint: AL state 0x%02X.\n", s.al_state);
    }
    if (s.online != sc_joint_state.online) {
        printf("Joint: %s.\n", s.online ? "online" : "offline");
    }
    if (s.operational != sc_joint_state.operational) {
        printf("Joint: %soperational.\n", s.operational ? "" : "not ");
    }
    sc_joint_state = s;
}

/****************************************************************************/

/** DS402 状态机：每周期驱动一次，自动从任意状态引导到 Operation Enabled。
 *  返回 1 表示已使能 (可下发运动指令)，否则 0。 */
static int ds402_engage(uint16_t status)
{
    uint16_t cw;

    /* 故障：先复位。 */
    if (status & SW_FAULT) {
        EC_WRITE_U16(domain1_pd + off_ctrlword, CW_FAULT_RESET);
        return 0;
    }

    switch (status & SW_MASK) {
    case SW_SWITCH_ON_DISABLED:
        cw = CW_SHUTDOWN;   /* -> Ready to switch on */
        break;
    case SW_READY_TO_SWITCH_ON:
        cw = CW_SWITCH_ON;  /* -> Switched on */
        break;
    case SW_SWITCHED_ON:
        cw = CW_ENABLE_OP;  /* -> Operation enabled */
        break;
    case SW_OPERATION_ENABLED:
        EC_WRITE_U16(domain1_pd + off_ctrlword, CW_ENABLE_OP);
        return 1;
    default:
        cw = CW_SHUTDOWN;
        break;
    }

    EC_WRITE_U16(domain1_pd + off_ctrlword, cw);
    return 0;
}

/****************************************************************************/

static void cyclic_task(struct timespec *wakeup_time)
{
    uint16_t status;
    int32_t actual_pos;
    int enabled;

    /* 接收过程数据。 */
    ecrt_master_receive(master);
    ecrt_domain_process(domain1);

    check_domain_state();

    status = EC_READ_U16(domain1_pd + off_statusword);
    actual_pos = EC_READ_S32(domain1_pd + off_actual_pos);

    /* 始终保持 CSP 模式。 */
    EC_WRITE_S8(domain1_pd + off_mode, MODE_CSP);

    /* 引导 DS402 状态机。 */
    enabled = ds402_engage(status);

    if (enabled) {
        if (!home_captured) {
            home_position = actual_pos;
            home_captured = 1;
            traj_time = 0.0;
            printf("Enabled. Home position = %d\n", home_position);
        }

        /* 正弦轨迹 (CSP: 周期下发目标位置)。 */
        double dt = (double)PERIOD_NS / (double)NSEC_PER_SEC;
        traj_time += dt;
        double offset = TRAJ_AMPLITUDE *
                sin(2.0 * M_PI * TRAJ_FREQ_HZ * traj_time);
        int32_t target = home_position + (int32_t)offset;
        EC_WRITE_S32(domain1_pd + off_target_pos, target);
    } else {
        /* 未使能时目标位置跟随实际位置，避免使能瞬间跳变。 */
        home_captured = 0;
        EC_WRITE_S32(domain1_pd + off_target_pos, actual_pos);
    }

    /* 1 Hz 打印诊断。 */
    if (counter) {
        counter--;
    } else {
        counter = FREQUENCY;
        check_master_state();
        check_slave_state();
        printf("status=0x%04X mode_disp=%d actual_pos=%d\n",
                status, EC_READ_S8(domain1_pd + off_mode_disp), actual_pos);
    }

    /* DC: 提供应用时间并同步时钟。须在每周期固定点调用。 */
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    ecrt_master_application_time(master,
            (uint64_t)t.tv_sec * NSEC_PER_SEC + t.tv_nsec);
    ecrt_master_sync_reference_clock(master);
    ecrt_master_sync_slave_clocks(master);

    (void)wakeup_time;

    /* 发送过程数据。 */
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);
}

/****************************************************************************/

static void stack_prefault(void)
{
    unsigned char dummy[MAX_SAFE_STACK];
    memset(dummy, 0, MAX_SAFE_STACK);
}

/****************************************************************************/

int main(int argc, char **argv)
{
    struct timespec wakeup_time;
    int ret = 0;

    (void)argc;
    (void)argv;

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    master = ecrt_request_master(0);
    if (!master) {
        fprintf(stderr, "Failed to request master.\n");
        return EXIT_FAILURE;
    }

    domain1 = ecrt_master_create_domain(master);
    if (!domain1) {
        fprintf(stderr, "Failed to create domain.\n");
        return EXIT_FAILURE;
    }

    printf("Getting slave configuration "
           "(vendor=0x%08X product=0x%08X)...\n",
           CYBERBEAST_VENDOR, CYBERBEAST_JOINT_PRODUCT);
    sc_joint = ecrt_master_slave_config(master, JOINT_ALIAS, JOINT_POSITION,
            CYBERBEAST_VENDOR, CYBERBEAST_JOINT_PRODUCT);
    if (!sc_joint) {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return EXIT_FAILURE;
    }

    printf("Configuring PDOs...\n");
    if (ecrt_slave_config_pdos(sc_joint, EC_END, joint_syncs)) {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return EXIT_FAILURE;
    }

    /* 默认模式预置为 CSP (配置期 SDO，激活时下发)。 */
    if (ecrt_slave_config_sdo8(sc_joint, OD_MODE_OF_OP, 0, MODE_CSP)) {
        fprintf(stderr, "Failed to configure default mode SDO.\n");
        return EXIT_FAILURE;
    }

    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) {
        fprintf(stderr, "PDO entry registration failed.\n");
        return EXIT_FAILURE;
    }

    /* 配置 DC：AssignActivate=0x300，SYNC0=周期，shift=周期/4。 */
    printf("Configuring DC (assign=0x%04X cycle=%u ns shift=%u ns)...\n",
            DC_ASSIGN_ACTIVATE, PERIOD_NS, SYNC0_SHIFT_NS);
    ecrt_slave_config_dc(sc_joint, DC_ASSIGN_ACTIVATE,
            PERIOD_NS, SYNC0_SHIFT_NS, 0, 0);

    printf("Activating master...\n");
    if (ecrt_master_activate(master)) {
        fprintf(stderr, "Failed to activate master.\n");
        return EXIT_FAILURE;
    }

    if (!(domain1_pd = ecrt_domain_data(domain1))) {
        fprintf(stderr, "Failed to get domain data pointer.\n");
        return EXIT_FAILURE;
    }

    /* 实时优先级。 */
    struct sched_param param = {0};
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    printf("Using priority %i.\n", param.sched_priority);
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("sched_setscheduler failed");
    }

    /* 锁定内存。 */
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        fprintf(stderr, "Warning: failed to lock memory: %s\n",
                strerror(errno));
    }
    stack_prefault();

    printf("Starting RT task with dt=%u ns.\n", PERIOD_NS);

    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
    wakeup_time.tv_sec += 1; /* start in future */
    wakeup_time.tv_nsec = 0;

    while (g_running) {
        ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                &wakeup_time, NULL);
        if (ret) {
            fprintf(stderr, "clock_nanosleep(): %s\n", strerror(ret));
            break;
        }

        cyclic_task(&wakeup_time);

        wakeup_time.tv_nsec += PERIOD_NS;
        while (wakeup_time.tv_nsec >= NSEC_PER_SEC) {
            wakeup_time.tv_nsec -= NSEC_PER_SEC;
            wakeup_time.tv_sec++;
        }
    }

    /* 退出前尝试停机 (清控制字)。 */
    printf("\nStopping...\n");
    EC_WRITE_U16(domain1_pd + off_ctrlword, CW_SHUTDOWN);
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);

    ecrt_release_master(master);
    printf("Done.\n");
    return ret;
}

/****************************************************************************/
