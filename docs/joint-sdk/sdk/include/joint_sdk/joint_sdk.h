#ifndef JSDK_JOINT_SDK_H
#define JSDK_JOINT_SDK_H

#include <stdint.h>
#include <stddef.h>

#include <joint_sdk/cia402.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct jsdk_context jsdk_context_t;
typedef struct jsdk_joint jsdk_joint_t;

typedef enum {
    JSDK_OK = 0,
    JSDK_ERR_INVALID_ARG = -1,
    JSDK_ERR_NO_MEMORY = -2,
    JSDK_ERR_NOT_FOUND = -3,
    JSDK_ERR_BAD_STATE = -4,
    JSDK_ERR_ECRT = -5,
    JSDK_ERR_UNSUPPORTED = -6
} jsdk_status_t;

typedef enum {
    JSDK_MODE_CSP = 8,
    JSDK_MODE_CSV = 9,
    JSDK_MODE_CST = 10
} jsdk_mode_t;

typedef enum {
    JSDK_AXIS_UNKNOWN = 0,
    JSDK_AXIS_SWITCH_ON_DISABLED,
    JSDK_AXIS_READY_TO_SWITCH_ON,
    JSDK_AXIS_SWITCHED_ON,
    JSDK_AXIS_OPERATION_ENABLED,
    JSDK_AXIS_FAULT
} jsdk_axis_state_t;

typedef struct {
    unsigned int master_index;
    uint32_t period_ns;
    int32_t sync0_shift_ns;
    unsigned int max_joints;
    int auto_reference_clock;
} jsdk_context_config_t;

typedef struct {
    uint16_t alias;
    uint16_t position;
    const char *profile_name;
} jsdk_joint_config_t;

typedef struct {
    uint16_t statusword;
    int8_t mode_display;
    int32_t actual_position;
    int32_t actual_velocity;
    int16_t actual_torque;
    jsdk_axis_state_t axis_state;
    int online;
    int operational;
    uint8_t al_state;
} jsdk_joint_feedback_t;

typedef struct {
    unsigned int working_counter;
    unsigned int wc_state;
    unsigned int slaves_responding;
    unsigned int al_states;
    int link_up;
} jsdk_bus_state_t;

#define JSDK_DEFAULT_PERIOD_NS 1000000u
#define JSDK_DEFAULT_MAX_JOINTS 16u

#define JSDK_PROFILE_CYBERBEAST_JOINT_MODULE "CyberBeast Joint Module"
#define JSDK_PROFILE_CYBERBEAST_JOINT_MODULE_ID "cyberbeast_joint_module"

void jsdk_context_config_default(jsdk_context_config_t *config);

jsdk_context_t *jsdk_context_create(const jsdk_context_config_t *config);
void jsdk_context_destroy(jsdk_context_t *ctx);

jsdk_status_t jsdk_context_add_joint(jsdk_context_t *ctx,
        const jsdk_joint_config_t *config, jsdk_joint_t **joint);

jsdk_status_t jsdk_context_configure(jsdk_context_t *ctx);
jsdk_status_t jsdk_context_activate(jsdk_context_t *ctx);
void jsdk_context_deactivate(jsdk_context_t *ctx);

jsdk_status_t jsdk_context_cycle_begin(jsdk_context_t *ctx,
        uint64_t app_time_ns);
jsdk_status_t jsdk_context_cycle_end(jsdk_context_t *ctx);

jsdk_status_t jsdk_context_get_bus_state(jsdk_context_t *ctx,
        jsdk_bus_state_t *state);
const char *jsdk_context_last_error(jsdk_context_t *ctx);

void jsdk_joint_request_enable(jsdk_joint_t *joint, jsdk_mode_t mode);
void jsdk_joint_request_disable(jsdk_joint_t *joint);
void jsdk_joint_request_fault_reset(jsdk_joint_t *joint);

void jsdk_joint_set_mode(jsdk_joint_t *joint, jsdk_mode_t mode);
void jsdk_joint_set_target_position(jsdk_joint_t *joint, int32_t value);
void jsdk_joint_set_target_velocity(jsdk_joint_t *joint, int32_t value);
void jsdk_joint_set_target_torque(jsdk_joint_t *joint, int16_t value);

jsdk_status_t jsdk_joint_get_feedback(jsdk_joint_t *joint,
        jsdk_joint_feedback_t *feedback);
int jsdk_joint_is_enabled(jsdk_joint_t *joint);
int jsdk_joint_is_fault(jsdk_joint_t *joint);

const char *jsdk_status_string(jsdk_status_t status);
const char *jsdk_axis_state_string(jsdk_axis_state_t state);

/* === 单位换算（指令单位 ↔ 物理量） === */

typedef struct {
    double pos_counts_to_rad;    /* 1 指令单位 = N rad (输出轴) */
    double vel_counts_to_rad_s;  /* 1 指令单位/s = N rad/s (输出轴) */
    double trq_to_Nm;            /* 1 转矩单位 = N N·m */
    int valid;                   /* 1 = 系数已计算 */
} jsdk_unit_scale_t;

/* 用默认参数初始化（encoder=16384, gear=8:1, rated_trq=10 N·m） */
void jsdk_unit_scale_default(jsdk_unit_scale_t *scale, uint32_t rated_trq);

/* 根据 ESI 参数精确计算换算系数 */
void jsdk_unit_scale_calc(jsdk_unit_scale_t *scale,
        uint32_t encoder_resolution,   /* 0x608F:1 */
        uint32_t motor_rev,            /* 0x6091:1 */
        uint32_t shaft_rev,            /* 0x6091:2 */
        uint32_t rated_torque);        /* 0x6076 */

/* 设置/获取 joint 的换算系数。激活前或激活后均可调用。 */
void jsdk_joint_set_scale(jsdk_joint_t *joint,
        const jsdk_unit_scale_t *scale);
void jsdk_joint_get_scale(jsdk_joint_t *joint,
        jsdk_unit_scale_t *scale);

/* 物理量接口（rt_safe，内部自动完成单位换算和位置展开）。
 * 可与指令单位接口混用，在同一 cycle 内先后调用任意一种均可。 */
void jsdk_joint_set_target_position_rad(jsdk_joint_t *joint, double rad);
void jsdk_joint_set_target_velocity_rad_s(jsdk_joint_t *joint, double rad_s);
void jsdk_joint_set_target_torque_Nm(jsdk_joint_t *joint, double Nm);

double jsdk_joint_actual_position_rad(jsdk_joint_t *joint);
double jsdk_joint_actual_velocity_rad_s(jsdk_joint_t *joint);
double jsdk_joint_actual_torque_Nm(jsdk_joint_t *joint);

/* === 故障诊断 === */

typedef struct {
    int valid;               /* 1 = 已读取到有效故障信息 */
    uint16_t code_603f;      /* CiA402 标准故障码 (0x603F) */
    uint8_t  error_register; /* 错误寄存器 (0x1001) */
    uint32_t vendor_lo;      /* 厂商故障码低 32 位 (0x203F) */
    uint32_t vendor_hi;      /* 厂商故障码高 32 位 (0x203E) */
} jsdk_fault_info_t;

/* 故障回调类型。在 rt_safe 周期上下文中调用，禁止 sleep/malloc/加锁。 */
typedef void (*jsdk_fault_callback_t)(jsdk_joint_t *joint,
        const jsdk_fault_info_t *info, void *user_data);

/* 注册故障回调。激活前或激活后均可调用。 */
void jsdk_context_set_fault_callback(jsdk_context_t *ctx,
        jsdk_fault_callback_t cb, void *user_data);

/* 同步获取最近一次故障信息（rt_safe）。
 * 返回 0 无故障，返回 1 有故障 (valid=1)。
 * 无故障时 info 各字段均为 0。 */
int jsdk_joint_get_fault_info(jsdk_joint_t *joint,
        jsdk_fault_info_t *info);

/* === 异步 SDO 读写（运行期，rt_safe，不阻塞周期） === */

typedef int jsdk_sdo_handle_t;

typedef enum {
    JSDK_SDO_IDLE = 0,     /* 空闲，可发起新请求 */
    JSDK_SDO_BUSY,          /* 正在处理中 */
    JSDK_SDO_SUCCESS,       /* 上次请求成功 */
    JSDK_SDO_ERROR          /* 上次请求失败 */
} jsdk_sdo_state_t;

/* 创建异步 SDO 请求句柄（激活前调用，idle/blocking）。
 *   index/subindex  目标 CoE 对象
 *   size            预留数据缓冲区大小（需 ≥ 对象的实际字节数）
 * 返回 >=0 的句柄，失败返回 -1。 */
jsdk_sdo_handle_t jsdk_joint_sdo_create(jsdk_joint_t *joint,
        uint16_t index, uint8_t subindex, size_t size);

/* 查询 SDO 请求当前状态（rt_safe）。 */
jsdk_sdo_state_t jsdk_joint_sdo_state(jsdk_joint_t *joint,
        jsdk_sdo_handle_t handle);

/* 获取 SDO 数据缓冲区指针（rt_safe）。
 * 写入前：在此缓冲区填充数据，再调用 jsdk_joint_sdo_write()。
 * 读取后：状态为 SUCCESS 时从此缓冲区取数据。 */
uint8_t *jsdk_joint_sdo_data(jsdk_joint_t *joint,
        jsdk_sdo_handle_t handle);

/* 获取上次读取的实际数据大小（rt_safe）。 */
size_t jsdk_joint_sdo_data_size(jsdk_joint_t *joint,
        jsdk_sdo_handle_t handle);

/* 调度异步 SDO 读/写操作（rt_safe）。仅在状态非 BUSY 时可调用。
 * 返回 0 成功，<0 失败（如缓冲区不足）。 */
int jsdk_joint_sdo_read(jsdk_joint_t *joint, jsdk_sdo_handle_t handle);
int jsdk_joint_sdo_write(jsdk_joint_t *joint, jsdk_sdo_handle_t handle);

/* === ESI XML 动态加载 === */

/* 从 ESI XML 文件加载关节 profile。
 * 若 vendor_id 或 product_code 为 0 则不校验，取文件中第一个匹配项。
 * 返回的 profile 可通过 jsdk_profile_destroy() 释放。
 * 也可直接将返回的 profile 名字传给 jsdk_joint_config_t::profile_name。
 * 失败返回 NULL。 */
const struct jsdk_joint_profile *jsdk_profile_load_from_esi(
        const char *esi_path,
        uint32_t vendor_id,
        uint32_t product_code);

/* 释放 jsdk_profile_load_from_esi() 返回的 profile。 */
void jsdk_profile_destroy(const struct jsdk_joint_profile *profile);

/* 获取 profile 的名称字符串，可用于 jsdk_joint_config_t::profile_name。 */
const char *jsdk_profile_get_name(const struct jsdk_joint_profile *profile);

#ifdef __cplusplus
}
#endif

#endif
