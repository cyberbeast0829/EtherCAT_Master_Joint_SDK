#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "internal.h"
#include "esi_parser.h"

void jsdk_set_error(jsdk_context_t *ctx, const char *fmt, ...)
{
    va_list ap;

    if (!ctx || !fmt) {
        return;
    }

    va_start(ap, fmt);
    vsnprintf(ctx->last_error, sizeof(ctx->last_error), fmt, ap);
    va_end(ap);
}

static int valid_mode(const jsdk_joint_t *joint, jsdk_mode_t mode)
{
    int8_t value = (int8_t)mode;

    return value == joint->profile->mode_csp ||
        value == joint->profile->mode_csv ||
        value == joint->profile->mode_cst;
}

/** 把 CoE 对象索引映射到 jsdk_pdo_offsets_t 中的对应字段。 */
static unsigned int *offset_for_object(jsdk_joint_t *joint,
        uint16_t index, uint8_t subindex)
{
    (void)subindex;
    switch (index) {
    case JSDK_CIA402_CONTROLWORD:      return &joint->offsets.controlword;
    case JSDK_CIA402_TARGET_POSITION:  return &joint->offsets.target_position;
    case JSDK_CIA402_TARGET_VELOCITY:  return &joint->offsets.target_velocity;
    case JSDK_CIA402_TARGET_TORQUE:    return &joint->offsets.target_torque;
    case JSDK_CIA402_MODE_OF_OPERATION:return &joint->offsets.mode_of_operation;
    case JSDK_CIA402_STATUSWORD:       return &joint->offsets.statusword;
    case JSDK_CIA402_ACTUAL_POSITION:  return &joint->offsets.actual_position;
    case JSDK_CIA402_ACTUAL_VELOCITY:  return &joint->offsets.actual_velocity;
    case JSDK_CIA402_ACTUAL_TORQUE:    return &joint->offsets.actual_torque;
    case JSDK_CIA402_MODE_DISPLAY:     return &joint->offsets.mode_display;
    default: return NULL;
    }
}

static jsdk_status_t register_joint_pdos(jsdk_joint_t *joint)
{
    const jsdk_joint_profile_t *p = joint->profile;
    unsigned int i;

    /* 注册所有 PDO 条目到域。
     * 【关键】padding(index==0x0000) 和 SDK 暂不识别的对象也必须调用
     * ecrt_slave_config_reg_pdo_entry，否则后续条目的字节偏移会全体错位。 */
    for (i = 0; i < p->rx_entry_count; i++) {
        const ec_pdo_entry_info_t *e = &p->rx_entries[i];
        unsigned int bit_position = 0;
        int ret = ecrt_slave_config_reg_pdo_entry(joint->sc,
                e->index, e->subindex, joint->ctx->domain,
                &bit_position);

        if (ret < 0) {
            jsdk_set_error(joint->ctx,
                    "failed to register RxPDO entry 0x%04X:%u: %d",
                    e->index, e->subindex, ret);
            return JSDK_ERR_ECRT;
        }
        if (bit_position) {
            jsdk_set_error(joint->ctx,
                    "RxPDO entry 0x%04X:%u not byte-aligned (bit %u)",
                    e->index, e->subindex, bit_position);
            return JSDK_ERR_UNSUPPORTED;
        }

        /* padding(0x0000) 和不识别的对象：只占域位置，不存偏移 */
        if (e->index != 0x0000) {
            unsigned int *off = offset_for_object(joint, e->index,
                    e->subindex);
            if (off) {
                *off = (unsigned int)ret;
            }
            /* 不认识的对象静默忽略；未来扩展对象词典后在此补映射 */
        }
    }

    for (i = 0; i < p->tx_entry_count; i++) {
        const ec_pdo_entry_info_t *e = &p->tx_entries[i];
        unsigned int bit_position = 0;
        int ret = ecrt_slave_config_reg_pdo_entry(joint->sc,
                e->index, e->subindex, joint->ctx->domain,
                &bit_position);

        if (ret < 0) {
            jsdk_set_error(joint->ctx,
                    "failed to register TxPDO entry 0x%04X:%u: %d",
                    e->index, e->subindex, ret);
            return JSDK_ERR_ECRT;
        }
        if (bit_position) {
            jsdk_set_error(joint->ctx,
                    "TxPDO entry 0x%04X:%u not byte-aligned (bit %u)",
                    e->index, e->subindex, bit_position);
            return JSDK_ERR_UNSUPPORTED;
        }

        if (e->index != 0x0000) {
            unsigned int *off = offset_for_object(joint, e->index,
                    e->subindex);
            if (off) {
                *off = (unsigned int)ret;
            }
        }
    }

    return JSDK_OK;
}

static void read_joint_feedback(jsdk_joint_t *joint)
{
    uint8_t *pd = joint->ctx->domain_pd;
    ec_slave_config_state_t slave_state;

    joint->feedback.statusword = EC_READ_U16(pd + joint->offsets.statusword);
    joint->feedback.mode_display = EC_READ_S8(pd + joint->offsets.mode_display);
    joint->feedback.actual_position =
        EC_READ_S32(pd + joint->offsets.actual_position);
    joint->feedback.actual_velocity =
        EC_READ_S32(pd + joint->offsets.actual_velocity);
    joint->feedback.actual_torque =
        EC_READ_S16(pd + joint->offsets.actual_torque);

    jsdk_cia402_update(&joint->cia402, joint->feedback.statusword);
    joint->feedback.axis_state = joint->cia402.axis_state;

    if (ecrt_slave_config_state(joint->sc, &slave_state) == 0) {
        joint->feedback.online = slave_state.online;
        joint->feedback.operational = slave_state.operational;
        joint->feedback.al_state = slave_state.al_state;
    }

    if (!joint->cia402.operation_enabled) {
        joint->command.target_position = joint->feedback.actual_position;
        joint->command.target_velocity = 0;
        joint->command.target_torque = 0;
    }
}

static void write_joint_command(jsdk_joint_t *joint)
{
    uint8_t *pd = joint->ctx->domain_pd;

    EC_WRITE_U16(pd + joint->offsets.controlword,
            joint->cia402.controlword);
    EC_WRITE_S8(pd + joint->offsets.mode_of_operation,
            joint->cia402.requested_mode);
    EC_WRITE_S32(pd + joint->offsets.target_position,
            joint->command.target_position);
    EC_WRITE_S32(pd + joint->offsets.target_velocity,
            joint->command.target_velocity);
    EC_WRITE_S16(pd + joint->offsets.target_torque,
            joint->command.target_torque);
}

const jsdk_joint_profile_t *jsdk_profile_load_from_esi(
        const char *esi_path,
        uint32_t vendor_id,
        uint32_t product_code)
{
    return esi_profile_load(esi_path, vendor_id, product_code);
}

const char *jsdk_profile_get_name(const struct jsdk_joint_profile *profile)
{
    return profile ? profile->name : NULL;
}

void jsdk_context_config_default(jsdk_context_config_t *config)
{
    if (!config) {
        return;
    }

    memset(config, 0, sizeof(*config));
    config->master_index = 0;
    config->period_ns = JSDK_DEFAULT_PERIOD_NS;
    config->sync0_shift_ns = -1;
    config->max_joints = JSDK_DEFAULT_MAX_JOINTS;
    config->auto_reference_clock = 1;
}

jsdk_context_t *jsdk_context_create(const jsdk_context_config_t *config)
{
    jsdk_context_t *ctx;
    jsdk_context_config_t effective;

    if (config) {
        effective = *config;
    } else {
        jsdk_context_config_default(&effective);
    }

    if (!effective.period_ns) {
        effective.period_ns = JSDK_DEFAULT_PERIOD_NS;
    }
    if (!effective.max_joints) {
        effective.max_joints = JSDK_DEFAULT_MAX_JOINTS;
    }

    ctx = calloc(1, sizeof(*ctx));
    if (!ctx) {
        return NULL;
    }

    ctx->joints = calloc(effective.max_joints, sizeof(ctx->joints[0]));
    if (!ctx->joints) {
        free(ctx);
        return NULL;
    }

    ctx->config = effective;
    return ctx;
}

void jsdk_context_destroy(jsdk_context_t *ctx)
{
    unsigned int i;

    if (!ctx) {
        return;
    }

    if (ctx->master) {
        ecrt_release_master(ctx->master);
    }

    for (i = 0; i < ctx->joint_count; i++) {
        free(ctx->joints[i]);
    }
    for (i = 0; i < ctx->loaded_profile_count; i++) {
        jsdk_profile_destroy(ctx->loaded_profiles[i]);
    }
    free(ctx->loaded_profiles);
    free(ctx->joints);
    free(ctx);
}

jsdk_status_t jsdk_context_add_joint(jsdk_context_t *ctx,
        const jsdk_joint_config_t *config, jsdk_joint_t **joint_out)
{
    jsdk_joint_t *joint;
    const jsdk_joint_profile_t *profile;

    if (!ctx || !config) {
        return JSDK_ERR_INVALID_ARG;
    }
    if (ctx->configured || ctx->activated) {
        jsdk_set_error(ctx, "cannot add joints after configuration");
        return JSDK_ERR_BAD_STATE;
    }
    if (ctx->joint_count >= ctx->config.max_joints) {
        jsdk_set_error(ctx, "joint capacity exceeded (%u)",
                ctx->config.max_joints);
        return JSDK_ERR_BAD_STATE;
    }

    /* 如果 profile_name 看起来像文件路径（含 .xml 或 /），从 ESI 加载 */
    if (config->profile_name &&
            (strstr(config->profile_name, ".xml") ||
             strchr(config->profile_name, '/'))) {
        /* Load and store in context for cleanup */
        if (ctx->loaded_profile_count >= ctx->loaded_profile_capacity) {
            unsigned int new_cap = ctx->loaded_profile_capacity
                ? ctx->loaded_profile_capacity * 2 : 4;
            const jsdk_joint_profile_t **tmp = realloc(
                    ctx->loaded_profiles,
                    new_cap * sizeof(ctx->loaded_profiles[0]));
            if (!tmp) return JSDK_ERR_NO_MEMORY;
            ctx->loaded_profiles = tmp;
            ctx->loaded_profile_capacity = new_cap;
        }

        profile = esi_profile_load(config->profile_name, 0, 0);
        if (profile) {
            ctx->loaded_profiles[ctx->loaded_profile_count++] = profile;
        }
    }
    if (!profile) {
        profile = jsdk_profile_find(config->profile_name);
    }
    if (!profile) {
        jsdk_set_error(ctx, "unsupported joint profile '%s' "
            "(supported: %s, %s, default; legacy alias: FL90BLW14)",
            config->profile_name ? config->profile_name : "(null)",
            JSDK_PROFILE_CYBERBEAST_JOINT_MODULE,
            JSDK_PROFILE_CYBERBEAST_JOINT_MODULE_ID);
        return JSDK_ERR_NOT_FOUND;
    }

    if (ctx->config.period_ns < profile->min_cycle_ns) {
        jsdk_set_error(ctx, "period %u ns is below profile minimum %u ns",
                ctx->config.period_ns, profile->min_cycle_ns);
        return JSDK_ERR_UNSUPPORTED;
    }

    joint = calloc(1, sizeof(*joint));
    if (!joint) {
        return JSDK_ERR_NO_MEMORY;
    }

    joint->ctx = ctx;
    joint->profile = profile;
    joint->alias = config->alias;
    joint->position = config->position;
    jsdk_cia402_init(&joint->cia402, profile->mode_csp);

    ctx->joints[ctx->joint_count++] = joint;
    if (joint_out) {
        *joint_out = joint;
    }
    return JSDK_OK;
}

jsdk_status_t jsdk_context_configure(jsdk_context_t *ctx)
{
    unsigned int i;
    int32_t shift;

    if (!ctx) {
        return JSDK_ERR_INVALID_ARG;
    }
    if (ctx->configured) {
        return JSDK_OK;
    }
    if (!ctx->joint_count) {
        jsdk_set_error(ctx, "no joints configured");
        return JSDK_ERR_BAD_STATE;
    }

    if (!ctx->master) {
        ctx->master = ecrt_request_master(ctx->config.master_index);
        if (!ctx->master) {
            jsdk_set_error(ctx, "failed to request master %u",
                    ctx->config.master_index);
            return JSDK_ERR_ECRT;
        }
    }

    ctx->domain = ecrt_master_create_domain(ctx->master);
    if (!ctx->domain) {
        jsdk_set_error(ctx, "failed to create process data domain");
        return JSDK_ERR_ECRT;
    }

    shift = ctx->config.sync0_shift_ns;
    if (shift < 0) {
        shift = (int32_t)(ctx->config.period_ns / 4u);
    }

    for (i = 0; i < ctx->joint_count; i++) {
        jsdk_joint_t *joint = ctx->joints[i];
        int ret;

        joint->sc = ecrt_master_slave_config(ctx->master, joint->alias,
                joint->position, joint->profile->vendor_id,
                joint->profile->product_code);
        if (!joint->sc) {
            jsdk_set_error(ctx,
                    "failed to get slave config for position %u",
                    joint->position);
            return JSDK_ERR_ECRT;
        }

        ret = ecrt_slave_config_pdos(joint->sc, EC_END,
                joint->profile->syncs);
        if (ret) {
            jsdk_set_error(ctx,
                    "failed to configure PDOs for position %u: %d",
                    joint->position, ret);
            return JSDK_ERR_ECRT;
        }

        ret = ecrt_slave_config_sdo8(joint->sc,
                JSDK_CIA402_MODE_OF_OPERATION, 0,
                (uint8_t)joint->profile->mode_csp);
        if (ret) {
            jsdk_set_error(ctx,
                    "failed to configure mode SDO for position %u: %d",
                    joint->position, ret);
            return JSDK_ERR_ECRT;
        }

        ret = ecrt_slave_config_dc(joint->sc,
                joint->profile->dc_assign_activate, ctx->config.period_ns,
                shift, 0, 0);
        if (ret) {
            jsdk_set_error(ctx,
                    "failed to configure DC for position %u: %d",
                    joint->position, ret);
            return JSDK_ERR_ECRT;
        }

        ret = register_joint_pdos(joint);
        if (ret != JSDK_OK) {
            return ret;
        }

        if (ctx->config.auto_reference_clock && i == 0) {
            ret = ecrt_master_select_reference_clock(ctx->master, joint->sc);
            if (ret) {
                jsdk_set_error(ctx,
                        "failed to select reference clock at position %u: %d",
                        joint->position, ret);
                return JSDK_ERR_ECRT;
            }
        }
    }

    ctx->configured = 1;
    return JSDK_OK;
}

jsdk_status_t jsdk_context_activate(jsdk_context_t *ctx)
{
    if (!ctx) {
        return JSDK_ERR_INVALID_ARG;
    }
    if (!ctx->configured) {
        jsdk_status_t ret = jsdk_context_configure(ctx);
        if (ret != JSDK_OK) {
            return ret;
        }
    }
    if (ctx->activated) {
        return JSDK_OK;
    }

    if (ecrt_master_activate(ctx->master)) {
        jsdk_set_error(ctx, "failed to activate master");
        return JSDK_ERR_ECRT;
    }

    ctx->domain_pd = ecrt_domain_data(ctx->domain);
    if (!ctx->domain_pd) {
        jsdk_set_error(ctx, "failed to get domain data pointer");
        return JSDK_ERR_ECRT;
    }

    ctx->activated = 1;
    return JSDK_OK;
}

void jsdk_context_deactivate(jsdk_context_t *ctx)
{
    unsigned int i;

    if (!ctx || !ctx->master || !ctx->activated) {
        return;
    }

    ecrt_master_deactivate(ctx->master);
    ctx->activated = 0;
    ctx->configured = 0;
    ctx->domain_pd = NULL;
    ctx->domain = NULL;

    for (i = 0; i < ctx->joint_count; i++) {
        ctx->joints[i]->sc = NULL;
    }
}

jsdk_status_t jsdk_context_cycle_begin(jsdk_context_t *ctx,
        uint64_t app_time_ns)
{
    unsigned int i;

    if (!ctx || !ctx->activated) {
        return JSDK_ERR_BAD_STATE;
    }

    ecrt_master_application_time(ctx->master, app_time_ns);
    ecrt_master_receive(ctx->master);
    ecrt_domain_process(ctx->domain);

    for (i = 0; i < ctx->joint_count; i++) {
        read_joint_feedback(ctx->joints[i]);
    }

    return JSDK_OK;
}

jsdk_status_t jsdk_context_cycle_end(jsdk_context_t *ctx)
{
    unsigned int i;

    if (!ctx || !ctx->activated) {
        return JSDK_ERR_BAD_STATE;
    }

    for (i = 0; i < ctx->joint_count; i++) {
        write_joint_command(ctx->joints[i]);
    }

    ecrt_master_sync_reference_clock(ctx->master);
    ecrt_master_sync_slave_clocks(ctx->master);
    ecrt_domain_queue(ctx->domain);
    ecrt_master_send(ctx->master);
    return JSDK_OK;
}

jsdk_status_t jsdk_context_get_bus_state(jsdk_context_t *ctx,
        jsdk_bus_state_t *state)
{
    ec_domain_state_t domain_state;
    ec_master_state_t master_state;

    if (!ctx || !state || !ctx->domain || !ctx->master) {
        return JSDK_ERR_INVALID_ARG;
    }

    memset(state, 0, sizeof(*state));
    ecrt_domain_state(ctx->domain, &domain_state);
    ecrt_master_state(ctx->master, &master_state);

    state->working_counter = domain_state.working_counter;
    state->wc_state = domain_state.wc_state;
    state->slaves_responding = master_state.slaves_responding;
    state->al_states = master_state.al_states;
    state->link_up = master_state.link_up;
    return JSDK_OK;
}

const char *jsdk_context_last_error(jsdk_context_t *ctx)
{
    if (!ctx) {
        return "invalid context";
    }
    return ctx->last_error[0] ? ctx->last_error : "no error";
}

void jsdk_joint_request_enable(jsdk_joint_t *joint, jsdk_mode_t mode)
{
    if (!joint || !valid_mode(joint, mode)) {
        return;
    }
    jsdk_cia402_request_enable(&joint->cia402, (int8_t)mode);
}

void jsdk_joint_request_disable(jsdk_joint_t *joint)
{
    if (!joint) {
        return;
    }
    jsdk_cia402_request_disable(&joint->cia402);
}

void jsdk_joint_request_fault_reset(jsdk_joint_t *joint)
{
    if (!joint) {
        return;
    }
    jsdk_cia402_request_fault_reset(&joint->cia402);
}

void jsdk_joint_set_mode(jsdk_joint_t *joint, jsdk_mode_t mode)
{
    if (!joint || !valid_mode(joint, mode)) {
        return;
    }
    joint->cia402.requested_mode = (int8_t)mode;
}

void jsdk_joint_set_target_position(jsdk_joint_t *joint, int32_t value)
{
    if (joint) {
        joint->command.target_position = value;
    }
}

void jsdk_joint_set_target_velocity(jsdk_joint_t *joint, int32_t value)
{
    if (joint) {
        joint->command.target_velocity = value;
    }
}

void jsdk_joint_set_target_torque(jsdk_joint_t *joint, int16_t value)
{
    if (joint) {
        joint->command.target_torque = value;
    }
}

jsdk_status_t jsdk_joint_get_feedback(jsdk_joint_t *joint,
        jsdk_joint_feedback_t *feedback)
{
    if (!joint || !feedback) {
        return JSDK_ERR_INVALID_ARG;
    }
    *feedback = joint->feedback;
    return JSDK_OK;
}

int jsdk_joint_is_enabled(jsdk_joint_t *joint)
{
    return joint ? joint->cia402.operation_enabled : 0;
}

int jsdk_joint_is_fault(jsdk_joint_t *joint)
{
    return joint && joint->cia402.axis_state == JSDK_AXIS_FAULT;
}

const char *jsdk_status_string(jsdk_status_t status)
{
    switch (status) {
    case JSDK_OK:
        return "ok";
    case JSDK_ERR_INVALID_ARG:
        return "invalid_arg";
    case JSDK_ERR_NO_MEMORY:
        return "no_memory";
    case JSDK_ERR_NOT_FOUND:
        return "not_found";
    case JSDK_ERR_BAD_STATE:
        return "bad_state";
    case JSDK_ERR_ECRT:
        return "ecrt_error";
    case JSDK_ERR_UNSUPPORTED:
        return "unsupported";
    default:
        return "unknown";
    }
}
