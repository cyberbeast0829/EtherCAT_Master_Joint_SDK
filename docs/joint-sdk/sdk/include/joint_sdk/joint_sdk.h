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
