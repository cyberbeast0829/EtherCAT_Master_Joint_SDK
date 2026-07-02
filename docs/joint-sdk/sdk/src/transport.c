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

    /* ---- 故障诊断自动 SDO 读取 ---- */
    if (joint->feedback.statusword & JSDK_CIA402_SW_FAULT) {
        /* 故障态：启动或继续 SDO 轮流读取 */
        if (joint->fault_seq == 0) {
            joint->fault_seq = 1;  /* 开始读 0x603F */
            joint->fault_notified = 0;
            memset(&joint->fault_info, 0, sizeof(joint->fault_info));
        }

        if (joint->fault_seq >= 1 && joint->fault_seq <= 4) {
            ec_sdo_request_t *req = joint->sdo_reqs[joint->fault_seq - 1];
            if (req) {
                ec_request_state_t st = ecrt_sdo_request_state(req);
                if (st == EC_REQUEST_UNUSED) {
                    ecrt_sdo_request_read(req);
                } else if (st == EC_REQUEST_SUCCESS || st == EC_REQUEST_ERROR) {
                    /* 读取结果 */
                    if (joint->fault_seq == 1 && st == EC_REQUEST_SUCCESS) {
                        uint8_t *d = ecrt_sdo_request_data(req);
                        joint->fault_info.code_603f =
                            (uint16_t)(d[0] | (d[1] << 8));
                    } else if (joint->fault_seq == 2 && st == EC_REQUEST_SUCCESS) {
                        joint->fault_info.error_register =
                            *ecrt_sdo_request_data(req);
                    } else if (joint->fault_seq == 3 && st == EC_REQUEST_SUCCESS) {
                        uint8_t *d = ecrt_sdo_request_data(req);
                        joint->fault_info.vendor_lo =
                            (uint32_t)(d[0] | (d[1] << 8) |
                                    (d[2] << 16) | (d[3] << 24));
                    } else if (joint->fault_seq == 4 && st == EC_REQUEST_SUCCESS) {
                        uint8_t *d = ecrt_sdo_request_data(req);
                        joint->fault_info.vendor_hi =
                            (uint32_t)(d[0] | (d[1] << 8) |
                                    (d[2] << 16) | (d[3] << 24));
                    }
                    joint->fault_seq++;

                    if (joint->fault_seq > 4) {
                        joint->fault_info.valid = 1;
                        /* 触发回调 */
                        if (joint->ctx->fault_cb && !joint->fault_notified) {
                            joint->ctx->fault_cb(joint,
                                    &joint->fault_info,
                                    joint->ctx->fault_cb_data);
                            joint->fault_notified = 1;
                        }
                    }
                }
            }
        }
    } else {
        /* 无故障：复位序列 */
        joint->fault_seq = 0;
        joint->fault_notified = 0;
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
    jsdk_unit_scale_default(&joint->scale, 10000);

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

        /* 预创建故障诊断 SDO (handle 0-3) */
        joint->sdo_reqs[0] = ecrt_slave_config_create_sdo_request(
                joint->sc, 0x603F, 0, 2);
        joint->sdo_reqs[1] = ecrt_slave_config_create_sdo_request(
                joint->sc, 0x1001, 0, 1);
        joint->sdo_reqs[2] = ecrt_slave_config_create_sdo_request(
                joint->sc, 0x203F, 0, 4);
        joint->sdo_reqs[3] = ecrt_slave_config_create_sdo_request(
                joint->sc, 0x203E, 0, 4);
        joint->sdo_req_count = 4;
        joint->fault_seq = 0;
        joint->fault_notified = 0;
        memset(&joint->fault_info, 0, sizeof(joint->fault_info));

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

/* ================================================================
 * 异步 SDO 读写接口
 * ================================================================ */

jsdk_sdo_handle_t jsdk_joint_sdo_create(jsdk_joint_t *joint,
        uint16_t index, uint8_t subindex, size_t size)
{
    ec_sdo_request_t *req;

    if (!joint || !joint->sc || !size) return -1;
    if (joint->sdo_req_count >= JSDK_MAX_SDO_REQUESTS) return -1;

    req = ecrt_slave_config_create_sdo_request(joint->sc, index,
            subindex, size);
    if (!req) return -1;

    joint->sdo_reqs[joint->sdo_req_count] = req;
    return (jsdk_sdo_handle_t)(joint->sdo_req_count++);
}

static ec_sdo_request_t *sdo_get_req(jsdk_joint_t *joint,
        jsdk_sdo_handle_t handle)
{
    if (!joint || handle < 0 ||
            (unsigned int)handle >= joint->sdo_req_count)
        return NULL;
    return joint->sdo_reqs[handle];
}

jsdk_sdo_state_t jsdk_joint_sdo_state(jsdk_joint_t *joint,
        jsdk_sdo_handle_t handle)
{
    ec_sdo_request_t *req = sdo_get_req(joint, handle);
    if (!req) return JSDK_SDO_ERROR;

    switch (ecrt_sdo_request_state(req)) {
    case EC_REQUEST_UNUSED:   return JSDK_SDO_IDLE;
    case EC_REQUEST_BUSY:     return JSDK_SDO_BUSY;
    case EC_REQUEST_SUCCESS:  return JSDK_SDO_SUCCESS;
    case EC_REQUEST_ERROR:    return JSDK_SDO_ERROR;
    default:                  return JSDK_SDO_ERROR;
    }
}

uint8_t *jsdk_joint_sdo_data(jsdk_joint_t *joint,
        jsdk_sdo_handle_t handle)
{
    ec_sdo_request_t *req = sdo_get_req(joint, handle);
    return req ? ecrt_sdo_request_data(req) : NULL;
}

size_t jsdk_joint_sdo_data_size(jsdk_joint_t *joint,
        jsdk_sdo_handle_t handle)
{
    ec_sdo_request_t *req = sdo_get_req(joint, handle);
    return req ? ecrt_sdo_request_data_size(req) : 0;
}

int jsdk_joint_sdo_read(jsdk_joint_t *joint, jsdk_sdo_handle_t handle)
{
    ec_sdo_request_t *req = sdo_get_req(joint, handle);
    if (!req) return -1;
    return ecrt_sdo_request_read(req);
}

int jsdk_joint_sdo_write(jsdk_joint_t *joint, jsdk_sdo_handle_t handle)
{
    ec_sdo_request_t *req = sdo_get_req(joint, handle);
    if (!req) return -1;
    return ecrt_sdo_request_write(req);
}

/* ================================================================
 * 故障诊断
 * ================================================================ */

void jsdk_context_set_fault_callback(jsdk_context_t *ctx,
        jsdk_fault_callback_t cb, void *user_data)
{
    if (!ctx) return;
    ctx->fault_cb = cb;
    ctx->fault_cb_data = user_data;
}

int jsdk_joint_get_fault_info(jsdk_joint_t *joint,
        jsdk_fault_info_t *info)
{
    if (!joint || !info) return 0;
    *info = joint->fault_info;
    return joint->fault_info.valid;
}

/* ================================================================
 * 单位换算
 * ================================================================ */

#define JSDK_PI 3.14159265358979323846

void jsdk_unit_scale_default(jsdk_unit_scale_t *scale, uint32_t rated_trq)
{
    if (!scale) return;
    /* 默认参数：编码器 16384 counts/rev，齿轮比 8:1，输出轴 131072 counts/rev */
    jsdk_unit_scale_calc(scale, 16384, 8, 1, rated_trq ? rated_trq : 10000);
}

void jsdk_unit_scale_calc(jsdk_unit_scale_t *scale,
        uint32_t encoder_resolution,
        uint32_t motor_rev, uint32_t shaft_rev,
        uint32_t rated_torque)
{
    if (!scale) return;
    if (!encoder_resolution || !motor_rev) {
        memset(scale, 0, sizeof(*scale));
        return;
    }

    /* 输出轴 counts/rev = encoder_resolution * motor_rev / shaft_rev */
    double shaft_counts_per_rev = (double)encoder_resolution
            * (double)motor_rev / (double)(shaft_rev ? shaft_rev : 1);

    scale->pos_counts_to_rad   = 2.0 * JSDK_PI / shaft_counts_per_rev;
    scale->vel_counts_to_rad_s = 2.0 * JSDK_PI / shaft_counts_per_rev;

    /* 转矩：1 单位 = 0.1% 额定转矩 → N·m = unit * rated_torque / 1000000 */
    scale->trq_to_Nm = rated_torque ? ((double)rated_torque / 1000000.0) : 0.01;
    scale->valid = 1;
}

void jsdk_joint_set_scale(jsdk_joint_t *joint,
        const jsdk_unit_scale_t *scale)
{
    if (joint && scale) joint->scale = *scale;
}

void jsdk_joint_get_scale(jsdk_joint_t *joint,
        jsdk_unit_scale_t *scale)
{
    if (joint && scale) *scale = joint->scale;
}

/* --- 位置累计展开（16→32 bit 回绕修正） --- */

static void pos_unwrap(jsdk_joint_t *joint, int32_t raw)
{
    if (!joint->pos_accum_valid) {
        joint->pos_accumulator = (int64_t)raw;
        joint->last_raw_position = raw;
        joint->pos_accum_valid = 1;
        return;
    }

    {
        uint16_t cur  = (uint16_t)(raw & 0xFFFF);
        uint16_t prev = (uint16_t)(joint->last_raw_position & 0xFFFF);
        int16_t  diff = (int16_t)(cur - prev);
        joint->pos_accumulator += diff;
        joint->last_raw_position = raw;
    }
}

/* --- 物理量 getter（rt_safe） --- */

double jsdk_joint_actual_position_rad(jsdk_joint_t *joint)
{
    if (!joint || !joint->scale.valid) return 0.0;
    pos_unwrap(joint, joint->feedback.actual_position);
    return (double)joint->pos_accumulator * joint->scale.pos_counts_to_rad;
}

double jsdk_joint_actual_velocity_rad_s(jsdk_joint_t *joint)
{
    if (!joint || !joint->scale.valid) return 0.0;
    return (double)joint->feedback.actual_velocity
            * joint->scale.vel_counts_to_rad_s;
}

double jsdk_joint_actual_torque_Nm(jsdk_joint_t *joint)
{
    if (!joint || !joint->scale.valid) return 0.0;
    return (double)joint->feedback.actual_torque * joint->scale.trq_to_Nm;
}

/* --- 物理量 setter（rt_safe） --- */

void jsdk_joint_set_target_position_rad(jsdk_joint_t *joint, double rad)
{
    if (!joint || !joint->scale.valid) return;
    int32_t counts = (int32_t)(rad / joint->scale.pos_counts_to_rad);
    jsdk_joint_set_target_position(joint, counts);
}

void jsdk_joint_set_target_velocity_rad_s(jsdk_joint_t *joint, double rad_s)
{
    if (!joint || !joint->scale.valid) return;
    int32_t counts = (int32_t)(rad_s / joint->scale.vel_counts_to_rad_s);
    jsdk_joint_set_target_velocity(joint, counts);
}

void jsdk_joint_set_target_torque_Nm(jsdk_joint_t *joint, double Nm)
{
    if (!joint || !joint->scale.valid) return;
    int16_t units = (int16_t)(Nm / joint->scale.trq_to_Nm);
    jsdk_joint_set_target_torque(joint, units);
}
