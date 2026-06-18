#ifndef JSDK_INTERNAL_H
#define JSDK_INTERNAL_H

#include <stdarg.h>
#include <stdint.h>
#include <stddef.h>

#include "ecrt.h"
#include <joint_sdk/joint_sdk.h>

typedef struct {
    unsigned int controlword;
    unsigned int target_position;
    unsigned int target_velocity;
    unsigned int target_torque;
    unsigned int mode_of_operation;
    unsigned int statusword;
    unsigned int actual_position;
    unsigned int actual_velocity;
    unsigned int actual_torque;
    unsigned int mode_display;
} jsdk_pdo_offsets_t;

typedef struct {
    uint16_t index;
    uint8_t subindex;
    uint8_t width_bits;
} jsdk_pdo_object_t;

typedef struct jsdk_joint_profile {
    const char *name;
    uint32_t vendor_id;
    uint32_t product_code;
    uint32_t revision_no;
    uint16_t dc_assign_activate;
    uint32_t min_cycle_ns;
    int8_t mode_csp;
    int8_t mode_csv;
    int8_t mode_cst;
    const ec_sync_info_t *syncs;
} jsdk_joint_profile_t;

typedef struct {
    int enable_requested;
    int fault_reset_requested;
    int8_t requested_mode;
    uint16_t controlword;
    jsdk_axis_state_t axis_state;
    int operation_enabled;
} jsdk_cia402_axis_t;

typedef struct {
    int32_t target_position;
    int32_t target_velocity;
    int16_t target_torque;
} jsdk_joint_command_t;

struct jsdk_joint {
    struct jsdk_context *ctx;
    const jsdk_joint_profile_t *profile;
    ec_slave_config_t *sc;
    uint16_t alias;
    uint16_t position;
    jsdk_pdo_offsets_t offsets;
    jsdk_cia402_axis_t cia402;
    jsdk_joint_command_t command;
    jsdk_joint_feedback_t feedback;
};

struct jsdk_context {
    ec_master_t *master;
    ec_domain_t *domain;
    uint8_t *domain_pd;
    jsdk_context_config_t config;
    jsdk_joint_t **joints;
    unsigned int joint_count;
    int configured;
    int activated;
    char last_error[256];
};

const jsdk_joint_profile_t *jsdk_profile_find(const char *name);
const jsdk_joint_profile_t *jsdk_profile_cyberbeast_joint_module(void);

void jsdk_cia402_init(jsdk_cia402_axis_t *axis, int8_t default_mode);
void jsdk_cia402_request_enable(jsdk_cia402_axis_t *axis, int8_t mode);
void jsdk_cia402_request_disable(jsdk_cia402_axis_t *axis);
void jsdk_cia402_request_fault_reset(jsdk_cia402_axis_t *axis);
void jsdk_cia402_update(jsdk_cia402_axis_t *axis, uint16_t statusword);

void jsdk_set_error(jsdk_context_t *ctx, const char *fmt, ...);

#endif
