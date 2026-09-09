#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "internal.h"
#include "esi_parser.h"
#include <joint_sdk/dc_timing_conf.h>

/** Default wait after DC AssignActivate before PREOP→SAFEOP (ms).
 * Raised from 100 → 500: new boards often need longer settle or they
 * hit AL 0x001A Synchronization error right after OP. */
#define JSDK_WAIT_BEFORE_SAFEOP_MS 500

/** How long activate() may cycle-wait for SAFEOP/OP (ms). */
#define JSDK_ACTIVATE_WAIT_OP_MS 8000
/** Extra headroom when WaitBeforeSAFEOP is long or multi-joint. */
#define JSDK_ACTIVATE_WAIT_OP_MIN_MS 15000

/** Extra consecutive good OP+WC cycles after first OP (ms). */
#define JSDK_ACTIVATE_SETTLE_MS 500

/** Hold CiA402 enable across brief AL/WC glitches (cycles @ period). */
#define JSDK_OP_LOSS_DEBOUNCE 80
/**
 * CSP-only: after WC recovers, keep target=actual this many cycles.
 * Was 100 (~100 ms @ 1 ms) — glued 0x607A to actual so following error
 * never built and the motor appeared stuck. Keep a short latch only.
 */
#define JSDK_CSP_HOLD_AFTER_WC_CYCLES 8
/** CSP-only: reject TxPDO position jumps after a WC hole (counts). */
#define JSDK_CSP_POS_JUMP_REJECT 30000
/**
 * CSV/CST: keep last vel/torque through this many incomplete-WC cycles
 * (~100 ms @ 2 ms period) before forcing zero — avoids 0↔full steps that
 * amplify dual-axis sync loss at high speed.
 */
#define JSDK_CSV_WC_HOLD_CYCLES 50

static int joint_is_csp(const jsdk_joint_t *joint)
{
    return joint && (joint->csp_compact_rx ||
            joint->cia402.requested_mode == (int8_t)JSDK_MODE_CSP);
}

/**
 * CyberBeast / ODrive input_mode (0x2002:1):
 *   CSV/CST → PASSTHROUGH(1)  (cyclic — verified, do not change)
 *   CSP → POS_FILTER(3)       (TwinCAT CSP guide: 滤波位置)
 */
static uint16_t cyberbeast_input_mode(int8_t cia_mode)
{
    if (cia_mode == (int8_t)JSDK_MODE_CSV)
        return 1;
    if (cia_mode == (int8_t)JSDK_MODE_CST)
        return 1;
    return 3;
}

/**
 * TwinCAT "csp - axis" / fixed RPDO 0x1601 on these drives:
 *   0x6040:00/16 + 0x607A:00/32 + 0x0000:00/16
 * Slaves reject PDO remap ("does not support changing the PDO mapping").
 * Do NOT insert 0x6060 here — mode stays PREOP SDO only.
 * Omit 0x60FF/0x6071 (full 0x1600) — they interfere with native CSP.
 * Keep full TPDO 0x1A00 for vel/torque/mode diagnostics.
 */
static const ec_pdo_entry_info_t jsdk_csp_rx_entries[] = {
    {JSDK_CIA402_CONTROLWORD,     0, 16},
    {JSDK_CIA402_TARGET_POSITION, 0, 32},
    {0x0000,                      0, 16}, /* fixed pad — do not remap */
};

static const ec_pdo_info_t jsdk_csp_rx_pdos[] = {
    {0x1601, 3, jsdk_csp_rx_entries},
};

static const ec_pdo_entry_info_t jsdk_csp_tx_entries[] = {
    {JSDK_CIA402_STATUSWORD,      0, 16},
    {JSDK_CIA402_ACTUAL_POSITION, 0, 32},
    {JSDK_CIA402_ACTUAL_VELOCITY, 0, 32},
    {JSDK_CIA402_ACTUAL_TORQUE,   0, 16},
    {JSDK_CIA402_MODE_DISPLAY,    0,  8},
    {0x0000,                      0,  8},
};

static const ec_pdo_info_t jsdk_csp_tx_pdos[] = {
    {0x1A00, 6, jsdk_csp_tx_entries},
};

static const ec_sync_info_t jsdk_csp_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT,  0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, jsdk_csp_rx_pdos, EC_WD_ENABLE},
    {3, EC_DIR_INPUT,  1, jsdk_csp_tx_pdos, EC_WD_DISABLE},
    {0xff}
};

/** Safe 0x6080 when CST enables torque-mode velocity limiting. */
#define JSDK_CST_DEFAULT_VEL_LIMIT 80000u
/** Default 0x6080 for CSP so step moves cannot freewheel at open-loop max. */
#define JSDK_CSP_DEFAULT_VEL_LIMIT 80000u

static uint64_t monotonic_time_ns(void)
{
    struct timespec t;

    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)t.tv_sec * 1000000000ULL + (uint64_t)t.tv_nsec;
}

/**
 * One non-realtime exchange so the master FSM sees application_time and
 * process-data datagrams before / while configuring DC → SAFEOP.
 */
static void bootstrap_master_cycle(jsdk_context_t *ctx, uint64_t app_time_ns)
{
    ecrt_master_application_time(ctx->master, app_time_ns);
    ecrt_master_receive(ctx->master);
    ecrt_domain_process(ctx->domain);
    ecrt_master_sync_reference_clock(ctx->master);
    ecrt_master_sync_slave_clocks(ctx->master);
    ecrt_domain_queue(ctx->domain);
    ecrt_master_send(ctx->master);
}

static void idle_sleep_ms(unsigned int ms)
{
    struct timespec ts;

    ts.tv_sec = (time_t)(ms / 1000u);
    ts.tv_nsec = (long)(ms % 1000u) * 1000000L;
    nanosleep(&ts, NULL);
}

/** True if any configured joint's ring slave has IgH error_flag set. */
static int joint_slaves_error_flag(jsdk_context_t *ctx)
{
    unsigned int i;

    if (!ctx || !ctx->master)
        return 0;

    for (i = 0; i < ctx->joint_count; i++) {
        ec_slave_info_t info;

        memset(&info, 0, sizeof(info));
        if (ecrt_master_get_slave(ctx->master, ctx->joints[i]->position,
                &info))
            continue;
        if (info.error_flag)
            return 1;
    }
    return 0;
}

/**
 * Idle-master recovery: ecrt_master_reset() clears SAFEOP+ERROR left by
 * abrupt deactivate / DC sync loss so the next activate does not need
 * ethercatctl restart.
 */
jsdk_status_t jsdk_context_recover_bus(jsdk_context_t *ctx)
{
    unsigned int pass;

    if (!ctx)
        return JSDK_ERR_INVALID_ARG;
    if (ctx->activated) {
        jsdk_set_error(ctx, "recover_bus requires deactivated master");
        return JSDK_ERR_BAD_STATE;
    }
    if (!ctx->master) {
        ctx->master = ecrt_request_master(ctx->config.master_index);
        if (!ctx->master) {
            jsdk_set_error(ctx, "failed to request master %u for recover",
                    ctx->config.master_index);
            return JSDK_ERR_ECRT;
        }
    }

    /*
     * Always reset once (teardown often leaves 0x001A without a sticky
     * userspace error_flag yet). Extra passes if error_flag remains.
     */
    for (pass = 0; pass < 3u; pass++) {
        ecrt_master_reset(ctx->master);
        idle_sleep_ms(pass == 0 ? 400u : 300u);
        if (!joint_slaves_error_flag(ctx))
            break;
    }

    return JSDK_OK;
}

static void release_activation(jsdk_context_t *ctx)
{
    unsigned int i;

    if (!ctx || !ctx->master)
        return;

    ecrt_master_deactivate(ctx->master);
    ctx->activated = 0;
    ctx->sync_dc = 0;
    ctx->domain_pd = NULL;
    ctx->domain = NULL;
    ctx->configured = 0;
    for (i = 0; i < ctx->joint_count; i++)
        ctx->joints[i]->sc = NULL;
}

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
    unsigned int i;
    const ec_pdo_entry_info_t *rx = joint->rx_entries;
    const ec_pdo_entry_info_t *tx = joint->tx_entries;
    unsigned int rx_n = joint->rx_entry_count;
    unsigned int tx_n = joint->tx_entry_count;

    if (!rx || !tx || !rx_n || !tx_n) {
        rx = joint->profile->rx_entries;
        tx = joint->profile->tx_entries;
        rx_n = joint->profile->rx_entry_count;
        tx_n = joint->profile->tx_entry_count;
    }

    /* 注册所有 PDO 条目到域。
     * 【关键】padding(index==0x0000) 和 SDK 暂不识别的对象也必须调用
     * ecrt_slave_config_reg_pdo_entry，否则后续条目的字节偏移会全体错位。 */
    for (i = 0; i < rx_n; i++) {
        const ec_pdo_entry_info_t *e = &rx[i];
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

    for (i = 0; i < tx_n; i++) {
        const ec_pdo_entry_info_t *e = &tx[i];
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
    int allow_cia402_enable;
    int wc_ok = joint->ctx->domain_wc_ok;
    int is_csp = joint_is_csp(joint);
    int32_t new_pos;
    int32_t new_vel;
    int16_t new_trq;
    int8_t new_mode;
    uint16_t new_sw;

    /*
     * Incomplete WC: process image is stale/garbage. Keep last feedback.
     * Mode-specific setpoint policy:
     *   CSP  — freeze target position + latch hold (pos_hold_cycles)
     *   CSV/CST — debounce: hold last vel/torque briefly, then zero
     */
    if (!wc_ok) {
        if (joint->cia402.enable_requested)
            joint->op_loss_cycles++;
        if (is_csp) {
            joint->wc_bad_streak++;
            joint->pos_hold_cycles = JSDK_CSP_HOLD_AFTER_WC_CYCLES;
            joint->command.target_torque = 0;
            joint->command.target_velocity = 0;
            /* Keep last target_position — snapping here zeroed FE forever. */
        } else {
            joint->wc_miss_cycles++;
            if (joint->wc_miss_cycles >= JSDK_CSV_WC_HOLD_CYCLES) {
                joint->command.target_torque = 0;
                joint->command.target_velocity = 0;
            }
            /* else: keep last CSV/CST command through a short hole */
        }
        joint->feedback.axis_state = joint->cia402.axis_state;
        return;
    }

    joint->wc_miss_cycles = 0;

    new_sw = EC_READ_U16(pd + joint->offsets.statusword);
    new_mode = EC_READ_S8(pd + joint->offsets.mode_display);
    new_pos = EC_READ_S32(pd + joint->offsets.actual_position);
    new_vel = EC_READ_S32(pd + joint->offsets.actual_velocity);
    new_trq = EC_READ_S16(pd + joint->offsets.actual_torque);

    /*
     * CSP-only: after a WC hole, reject implausible position jumps.
     * CSV/CST always accept the new sample (no pos jump filter).
     */
    if (is_csp && joint->wc_bad_streak > 0 && joint->feedback_pos_valid) {
        int32_t jump = new_pos - joint->feedback.actual_position;

        if (jump < 0)
            jump = -jump;
        if (jump > JSDK_CSP_POS_JUMP_REJECT) {
            joint->pos_hold_cycles = JSDK_CSP_HOLD_AFTER_WC_CYCLES;
            joint->command.target_velocity = 0;
            joint->command.target_torque = 0;
            joint->feedback.statusword = new_sw;
            joint->feedback.mode_display = new_mode;
            joint->feedback.actual_velocity = 0;
            /* keep actual_position + app target (soft-invert safe) */
        } else {
            joint->wc_bad_streak = 0;
            joint->feedback.actual_position = new_pos;
            joint->feedback.actual_velocity = new_vel;
            joint->feedback.actual_torque = new_trq;
            joint->feedback.statusword = new_sw;
            joint->feedback.mode_display = new_mode;
            /*
             * Do NOT snap command.target_position to raw actual here.
             * Soft-invert apps store wire=2*act-cmd in target_position;
             * clobbering to actual then releasing pos_hold left invert
             * axes (e.g. dual j0 8.1.50) stuck at FE=0 while logical
             * cmd stayed far ahead. Wire still mirrors via pos_hold.
             */
            joint->pos_hold_cycles = JSDK_CSP_HOLD_AFTER_WC_CYCLES;
            joint->feedback_pos_valid = 1;
        }
    } else {
        if (is_csp)
            joint->wc_bad_streak = 0;
        joint->feedback.statusword = new_sw;
        joint->feedback.mode_display = new_mode;
        joint->feedback.actual_position = new_pos;
        joint->feedback.actual_velocity = new_vel;
        joint->feedback.actual_torque = new_trq;
        joint->feedback_pos_valid = 1;
    }

    joint->feedback.online = 0;
    joint->feedback.operational = 0;
    joint->feedback.al_state = 0;
    if (!ecrt_slave_config_state(joint->sc, &slave_state)) {
        joint->feedback.online = slave_state.online;
        joint->feedback.operational = slave_state.operational;
        joint->feedback.al_state = slave_state.al_state;
    }

    /*
     * EtherCAT AL state ≠ CiA402 servo state.
     *
     * Hold enable across short SAFEOP dips (debounce). Only force the
     * disable ladder after JSDK_OP_LOSS_DEBOUNCE consecutive bad cycles.
     */
    allow_cia402_enable = joint->cia402.enable_requested
            && joint->feedback.online
            && ((joint->feedback.al_state & 0x0f) == EC_AL_STATE_OP);

    if (allow_cia402_enable) {
        joint->op_loss_cycles = 0;
        jsdk_cia402_update(&joint->cia402, joint->feedback.statusword);
    } else if (joint->cia402.enable_requested &&
            joint->op_loss_cycles < JSDK_OP_LOSS_DEBOUNCE) {
        joint->op_loss_cycles++;
        /*
         * Brief AL dip: CSP freezes position. CSV/CST keep last vel/torque
         * so a short SAFEOP glitch does not slam the velocity loop to 0.
         */
        if (is_csp) {
            joint->command.target_torque = 0;
            joint->command.target_velocity = 0;
            /* Mirror on the wire via pos_hold; keep app target (invert). */
            joint->pos_hold_cycles = JSDK_CSP_HOLD_AFTER_WC_CYCLES;
        }
        jsdk_cia402_update(&joint->cia402, joint->feedback.statusword);
    } else {
        int saved = joint->cia402.enable_requested;
        joint->cia402.enable_requested = 0;
        jsdk_cia402_update(&joint->cia402, joint->feedback.statusword);
        joint->cia402.enable_requested = saved;
    }
    joint->feedback.axis_state = joint->cia402.axis_state;

    if (!joint->cia402.operation_enabled) {
        joint->command.target_position = joint->feedback.actual_position;
        joint->command.target_velocity = 0;
        joint->command.target_torque = 0;
    }

    /*
     * Do not overwrite command.target_position while pos_hold>0 — the app
     * keeps ramping the logical setpoint; write_joint_command() still
     * mirrors actual on the wire for the short latch window.
     */

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

    /* TwinCAT CSP RPDO 0x1601: only CW + target position.
     * While not yet Operation Enabled, always mirror actual — never
     * stream a default 0 setpoint (that is a multi-million-count step
     * at the instant the drive enables and causes vmax runaway).
     * During/after WC holes, force target=actual even if the app wrote
     * a ramped command this cycle. */
    if (joint->csp_compact_rx) {
        int32_t tp = joint->command.target_position;

        if (!joint->cia402.operation_enabled ||
                joint->pos_hold_cycles > 0 ||
                !joint->ctx->domain_wc_ok) {
            tp = joint->feedback.actual_position;
        }
        EC_WRITE_S32(pd + joint->offsets.target_position, tp);
        if (joint->pos_hold_cycles > 0 && joint->ctx->domain_wc_ok)
            joint->pos_hold_cycles--;
        return;
    }

    EC_WRITE_S8(pd + joint->offsets.mode_of_operation,
            joint->cia402.requested_mode);

    /* While not Operation Enabled, mirror actual / zero vel&torque so the
     * first enabled frame is not a huge step from an unset command (0). */
    if (!joint->cia402.operation_enabled) {
        EC_WRITE_S32(pd + joint->offsets.target_position,
                joint->feedback.actual_position);
        EC_WRITE_S32(pd + joint->offsets.target_velocity, 0);
        EC_WRITE_S16(pd + joint->offsets.target_torque, 0);
        return;
    }

    /*
     * Full 0x1600 always carries 0x607A+0x60FF+0x6071. For CSV/CST only
     * vel/torque is the setpoint — keep 0x607A mirrored to actual so a
     * frozen enable-time target_position cannot fight the velocity loop.
     *
     * Original:
     * EC_WRITE_S32(pd + joint->offsets.target_position,
     *         joint->command.target_position);
     */
    if (joint->cia402.requested_mode == (int8_t)JSDK_MODE_CSV ||
            joint->cia402.requested_mode == (int8_t)JSDK_MODE_CST) {
        EC_WRITE_S32(pd + joint->offsets.target_position,
                joint->feedback.actual_position);
    } else {
        EC_WRITE_S32(pd + joint->offsets.target_position,
                joint->command.target_position);
    }
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
    config->use_dc = 1;
    config->wait_before_safeop_ms = 0;
    config->dc_timing_mode = JSDK_DC_TIMING_MODE_DEFAULT;
}

void jsdk_context_config_apply_drive_model(jsdk_context_config_t *config,
        const jsdk_drive_model_t *model)
{
    if (!config || !model)
        return;

    config->use_dc = model->use_dc;
    config->auto_reference_clock = model->use_dc ? 1 : 0;
    config->sync0_shift_ns = model->sync0_shift_ns;
    if (model->wait_before_safeop_ms)
        config->wait_before_safeop_ms = model->wait_before_safeop_ms;
    /* preferred_period_ns is Sync0 fallback only — not task period. */
    config->sync0_cycle_ns = model->sync0_cycle_ns;
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
    /*
     * Do NOT call jsdk_dc_timing_conf_apply() here. Callers apply conf
     * then CLI overrides (diag_*); re-applying would clobber --sync0-cycle
     * etc. while the example still prints the pre-create config (lie).
     * Examples that skip conf_apply rely on drive_model / defaults only.
     */

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

    /* Always stop axes before releasing the master. */
    jsdk_context_deactivate(ctx);

    if (ctx->master) {
        ecrt_release_master(ctx->master);
        ctx->master = NULL;
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
    const jsdk_joint_profile_t *profile = NULL;

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
    joint->attach_vendor_id = config->vendor_id
            ? config->vendor_id : profile->vendor_id;
    joint->attach_product_code = config->product_code
            ? config->product_code : profile->product_code;
    if (config->initial_mode == JSDK_MODE_CSP ||
            config->initial_mode == JSDK_MODE_CSV ||
            config->initial_mode == JSDK_MODE_CST) {
        joint->preop_mode = (int8_t)config->initial_mode;
    } else {
        joint->preop_mode = profile->mode_csp;
    }
    joint->polarity_607e = config->polarity_607e
            ? config->polarity_607e : -1;
    joint->input_mode_2002 = config->input_mode_2002 > 0
            ? config->input_mode_2002 : -1;
    /* Distinguish "unset (0)" from explicit 0: treat 0 as default max. */
    joint->max_motor_velocity = config->max_motor_velocity;

    /* Resolve drive-model: --fw / firmware_version wins over name
     * (name may be stale freerun from an old diag binary path). */
    joint->drive_model = NULL;
    if (config->firmware_version && config->firmware_version[0]) {
        joint->drive_model = jsdk_drive_model_find(
                config->firmware_version, joint->attach_product_code);
    }
    if ((!joint->drive_model ||
            !joint->drive_model->fw_match ||
            !joint->drive_model->fw_match[0]) &&
            config->drive_model_name && config->drive_model_name[0]) {
        const jsdk_drive_model_t *by_name =
                jsdk_drive_model_by_name(config->drive_model_name);

        if (by_name && by_name->fw_match && by_name->fw_match[0])
            joint->drive_model = by_name;
        else if (!joint->drive_model)
            joint->drive_model = by_name;
    }
    if (joint->drive_model) {
        joint->dc_assign_activate =
                joint->drive_model->dc_assign_activate;
    } else {
        joint->dc_assign_activate = profile->dc_assign_activate;
    }
    /*
     * TwinCAT CSP: use compact RPDO 0x1601 (CW + target pos only).
     * Full 0x1600 vel/torque channels interfere with native CSP on this
     * drive (continuous vel-limit runaway away from goal).
     */
    if (joint->preop_mode == (int8_t)JSDK_MODE_CSP) {
        joint->csp_compact_rx = 1;
        joint->active_syncs = jsdk_csp_syncs;
        joint->rx_entries = jsdk_csp_rx_entries;
        joint->tx_entries = jsdk_csp_tx_entries;
        joint->rx_entry_count = 3;
        joint->tx_entry_count = 6;
    } else {
        joint->csp_compact_rx = 0;
        joint->active_syncs = profile->syncs;
        joint->rx_entries = profile->rx_entries;
        joint->tx_entries = profile->tx_entries;
        joint->rx_entry_count = profile->rx_entry_count;
        joint->tx_entry_count = profile->tx_entry_count;
    }
    jsdk_cia402_init(&joint->cia402, joint->preop_mode);
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
        /* Legacy default: ~period/4 when model did not set an explicit shift. */
        shift = (int32_t)(ctx->config.period_ns / 4u);
    }

    for (i = 0; i < ctx->joint_count; i++) {
        jsdk_joint_t *joint = ctx->joints[i];
        int ret;
        int joint_use_dc;
        int32_t joint_shift;
        uint32_t sync0_cycle;
        unsigned int wait_ms;
        uint16_t aa;

        joint->sc = ecrt_master_slave_config(ctx->master, joint->alias,
                joint->position, joint->attach_vendor_id,
                joint->attach_product_code);
        if (!joint->sc) {
            jsdk_set_error(ctx,
                    "failed to get slave config for position %u "
                    "(vendor=0x%08X product=0x%08X)",
                    joint->position, joint->attach_vendor_id,
                    joint->attach_product_code);
            return JSDK_ERR_ECRT;
        }

        ret = ecrt_slave_config_pdos(joint->sc, EC_END,
                joint->active_syncs ? joint->active_syncs
                : joint->profile->syncs);
        if (ret) {
            jsdk_set_error(ctx,
                    "failed to configure PDOs for position %u: %d",
                    joint->position, ret);
            return JSDK_ERR_ECRT;
        }

        ret = ecrt_slave_config_sdo8(joint->sc,
                JSDK_CIA402_MODE_OF_OPERATION, 0,
                (uint8_t)joint->preop_mode);
        if (ret) {
            jsdk_set_error(ctx,
                    "failed to configure mode SDO for position %u: %d",
                    joint->position, ret);
            return JSDK_ERR_ECRT;
        }

        /*
         * CyberBeast default 0x6072 (Max torque) is 0 — clamps all CST
         * commands to zero so the motor never moves. Raise to 100% rated
         * (0.1% units) before SAFEOP.
         */
        ret = ecrt_slave_config_sdo16(joint->sc, 0x6072, 0, 10000);
        if (ret) {
            jsdk_set_error(ctx,
                    "failed to configure max torque 0x6072 for position %u: %d",
                    joint->position, ret);
            return JSDK_ERR_ECRT;
        }

        /*
         * 0x2002:1: CSP→POS_FILTER(3), CSV/CST→PASSTHROUGH(1),
         * unless joint_config.input_mode_2002 overrides.
         */
        {
            uint16_t imode = joint->input_mode_2002 > 0
                    ? (uint16_t)joint->input_mode_2002
                    : cyberbeast_input_mode(joint->preop_mode);

            ret = ecrt_slave_config_sdo16(joint->sc, 0x2002, 1, imode);
            if (ret) {
                jsdk_set_error(ctx,
                        "failed to configure input mode 0x2002:1 for "
                        "position %u: %d",
                        joint->position, ret);
                return JSDK_ERR_ECRT;
            }
        }

        /*
         * CST without torque-mode vel limit freewheels to vel_limit under
         * any non-zero torque (seen as act_vel≈±3e5 and wild act_trq).
         * Enable 0x200A:12 and ensure 0x6080 is non-zero.
         *
         * CSP: set a default 0x6080 so large 0x607A steps cannot run away
         * at encoder max; do NOT enable 0x200A:12 (CSV/CSP leave it off).
         *
         * CSV: leave 0x6080 to caller / drive default; 0x200A:12 off.
         */
        if (joint->preop_mode == (int8_t)JSDK_MODE_CST) {
            uint32_t vmax = joint->max_motor_velocity
                    ? joint->max_motor_velocity
                    : JSDK_CST_DEFAULT_VEL_LIMIT;

            ret = ecrt_slave_config_sdo32(joint->sc, 0x6080, 0, vmax);
            if (ret) {
                jsdk_set_error(ctx,
                        "failed to configure max velocity 0x6080 for "
                        "position %u: %d",
                        joint->position, ret);
                return JSDK_ERR_ECRT;
            }

            ret = ecrt_slave_config_sdo8(joint->sc, 0x200A, 11, 1);
            if (ret) {
                jsdk_set_error(ctx,
                        "failed to configure 0x200A:11 for position %u: %d",
                        joint->position, ret);
                return JSDK_ERR_ECRT;
            }

            ret = ecrt_slave_config_sdo8(joint->sc, 0x200A, 12, 1);
            if (ret) {
                jsdk_set_error(ctx,
                        "failed to configure 0x200A:12 for position %u: %d",
                        joint->position, ret);
                return JSDK_ERR_ECRT;
            }
        } else if (joint->preop_mode == (int8_t)JSDK_MODE_CSP) {
            uint32_t vmax = joint->max_motor_velocity
                    ? joint->max_motor_velocity
                    : JSDK_CSP_DEFAULT_VEL_LIMIT;

            ret = ecrt_slave_config_sdo32(joint->sc, 0x6080, 0, vmax);
            if (ret) {
                jsdk_set_error(ctx,
                        "failed to configure max velocity 0x6080 for "
                        "position %u: %d",
                        joint->position, ret);
                return JSDK_ERR_ECRT;
            }

            ret = ecrt_slave_config_sdo8(joint->sc, 0x200A, 11, 1);
            if (ret) {
                jsdk_set_error(ctx,
                        "failed to configure 0x200A:11 for position %u: %d",
                        joint->position, ret);
                return JSDK_ERR_ECRT;
            }

            ret = ecrt_slave_config_sdo8(joint->sc, 0x200A, 12, 0);
            if (ret) {
                jsdk_set_error(ctx,
                        "failed to configure 0x200A:12 for position %u: %d",
                        joint->position, ret);
                return JSDK_ERR_ECRT;
            }

            /* Ensure position loop gains are non-zero (handbook defaults). */
            ret = ecrt_slave_config_sdo16(joint->sc, 0x2008, 4, 2000);
            if (ret) {
                jsdk_set_error(ctx,
                        "failed to configure pos kp 0x2008:4 for "
                        "position %u: %d",
                        joint->position, ret);
                return JSDK_ERR_ECRT;
            }
            ret = ecrt_slave_config_sdo16(joint->sc, 0x2008, 1, 10);
            if (ret) {
                jsdk_set_error(ctx,
                        "failed to configure vel kp 0x2008:1 for "
                        "position %u: %d",
                        joint->position, ret);
                return JSDK_ERR_ECRT;
            }
            ret = ecrt_slave_config_sdo16(joint->sc, 0x2008, 2, 100);
            if (ret) {
                jsdk_set_error(ctx,
                        "failed to configure vel ki 0x2008:2 for "
                        "position %u: %d",
                        joint->position, ret);
                return JSDK_ERR_ECRT;
            }
        } else {
            /* CSV: always set a usable 0x6080 (idle boards often have
             * vel_limit odd; 0 blocks motion). Original: only if caller set.
             *
             * if (joint->max_motor_velocity) { ... }
             */
            uint32_t vmax = joint->max_motor_velocity
                    ? joint->max_motor_velocity
                    : 200000u;

            ret = ecrt_slave_config_sdo32(joint->sc, 0x6080, 0, vmax);
            if (ret) {
                jsdk_set_error(ctx,
                        "failed to configure max velocity 0x6080 for "
                        "position %u: %d",
                        joint->position, ret);
                return JSDK_ERR_ECRT;
            }

            ret = ecrt_slave_config_sdo8(joint->sc, 0x200A, 11, 1);
            if (ret) {
                jsdk_set_error(ctx,
                        "failed to configure 0x200A:11 for position %u: %d",
                        joint->position, ret);
                return JSDK_ERR_ECRT;
            }

            ret = ecrt_slave_config_sdo8(joint->sc, 0x200A, 12, 0);
            if (ret) {
                jsdk_set_error(ctx,
                        "failed to configure 0x200A:12 for position %u: %d",
                        joint->position, ret);
                return JSDK_ERR_ECRT;
            }
        }

        if (joint->polarity_607e > 0) {
            ret = ecrt_slave_config_sdo8(joint->sc, 0x607E, 0,
                    (uint8_t)joint->polarity_607e);
            if (ret) {
                jsdk_set_error(ctx,
                        "failed to configure polarity 0x607E for "
                        "position %u: %d",
                        joint->position, ret);
                return JSDK_ERR_ECRT;
            }
        }

        /*
         * DC Sync0:
         *   SHARED — context use_dc / sync0_cycle / shift / wait for all.
         *   PER_JOINT — each joint->drive_model (same rows ⇒ same as SHARED).
         * Task period stays context-global either way.
         */
        joint_use_dc = ctx->config.use_dc;
        joint_shift = shift;
        sync0_cycle = ctx->config.sync0_cycle_ns
                ? ctx->config.sync0_cycle_ns
                : ctx->config.period_ns;
        wait_ms = ctx->config.wait_before_safeop_ms
                ? ctx->config.wait_before_safeop_ms
                : JSDK_WAIT_BEFORE_SAFEOP_MS;
        aa = joint->dc_assign_activate
                ? joint->dc_assign_activate
                : joint->profile->dc_assign_activate;

        if (ctx->config.dc_timing_mode == JSDK_DC_TIMING_PER_JOINT &&
                joint->drive_model) {
            const jsdk_drive_model_t *m = joint->drive_model;
            const char *fw = m->fw_match ? m->fw_match : m->name;
            int o_use;
            int32_t o_shift;
            uint32_t o_wait;
            uint32_t o_period;
            uint32_t o_sync0;

            joint_use_dc = m->use_dc;
            joint_shift = m->sync0_shift_ns;
            if (joint_shift < 0)
                joint_shift = (int32_t)(ctx->config.period_ns / 4u);
            if (m->sync0_cycle_ns)
                sync0_cycle = m->sync0_cycle_ns;
            else if (m->preferred_period_ns)
                sync0_cycle = m->preferred_period_ns;
            else
                sync0_cycle = ctx->config.period_ns;
            if (m->wait_before_safeop_ms)
                wait_ms = m->wait_before_safeop_ms;
            if (m->dc_assign_activate)
                aa = m->dc_assign_activate;

            o_use = joint_use_dc;
            o_shift = joint_shift;
            o_wait = wait_ms;
            o_period = 0;
            o_sync0 = sync0_cycle;
            if (jsdk_dc_timing_conf_lookup_fw(fw, &o_use, &o_shift, &o_wait,
                    &o_period, &o_sync0)) {
                joint_use_dc = o_use;
                joint_shift = o_shift;
                wait_ms = o_wait;
                if (o_sync0)
                    sync0_cycle = o_sync0;
            }
        }

        if (joint_use_dc) {
            ret = ecrt_slave_config_dc(joint->sc, aa,
                    sync0_cycle, joint_shift, 0, 0);
        } else {
            ret = ecrt_slave_config_dc(joint->sc, 0, 0, 0, 0, 0);
        }
        if (ret) {
            jsdk_set_error(ctx,
                    "failed to configure DC for position %u: %d",
                    joint->position, ret);
            return JSDK_ERR_ECRT;
        }

#ifdef EC_HAVE_FLAGS
        if (joint_use_dc) {
            ret = ecrt_slave_config_flag(joint->sc, "WaitBeforeSAFEOPms",
                    (int)wait_ms);
            if (ret) {
                jsdk_set_error(ctx,
                        "failed to set WaitBeforeSAFEOPms for position %u: %d",
                        joint->position, ret);
                return JSDK_ERR_ECRT;
            }
        }
#endif

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

        if (joint_use_dc && ctx->config.auto_reference_clock
                && i == 0) {
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
    int attempt;

    if (!ctx) {
        return JSDK_ERR_INVALID_ARG;
    }
    if (ctx->activated) {
        return JSDK_OK;
    }

    for (attempt = 0; attempt < 2; attempt++) {
        unsigned int waited_ms = 0;
        unsigned int settle_ms = 0;
        unsigned int period_us;
        unsigned int i;
        unsigned int activate_wait_ms = JSDK_ACTIVATE_WAIT_OP_MS;
        int reached_safeop = 0;
        int reached_op_settle = 0;
        jsdk_status_t rec;

        if (!ctx->configured) {
            jsdk_status_t ret = jsdk_context_configure(ctx);
            if (ret != JSDK_OK) {
                return ret;
            }
        }

        /* Clear SAFEOP+ERROR from prior abrupt exit before (re)activate. */
        rec = jsdk_context_recover_bus(ctx);
        if (rec != JSDK_OK) {
            return rec;
        }

        if (ecrt_master_activate(ctx->master)) {
            jsdk_set_error(ctx, "failed to activate master");
            return JSDK_ERR_ECRT;
        }

        ctx->domain_pd = ecrt_domain_data(ctx->domain);
        if (!ctx->domain_pd) {
            jsdk_set_error(ctx, "failed to get domain data pointer");
            release_activation(ctx);
            return JSDK_ERR_ECRT;
        }

        /*
         * Seed application_time immediately. If the app delays the first
         * cyclic call, IgH programs DC StartTime with app_time==0 and
         * PREOP→SAFEOP hangs.
         *
         * 1) Must reach SAFEOP.
         * 2) Prefer OP+WC settle; if OP late, still OK after SAFEOP.
         */
        period_us = ctx->config.period_ns / 1000u;
        if (period_us == 0) {
            period_us = 1000;
        }

        if (ctx->config.wait_before_safeop_ms + 8000u > activate_wait_ms)
            activate_wait_ms = ctx->config.wait_before_safeop_ms + 8000u;
        if (ctx->joint_count > 1 &&
                activate_wait_ms < JSDK_ACTIVATE_WAIT_OP_MIN_MS)
            activate_wait_ms = JSDK_ACTIVATE_WAIT_OP_MIN_MS;

        bootstrap_master_cycle(ctx, monotonic_time_ns());

        {
            struct timespec wakeup;

            clock_gettime(CLOCK_MONOTONIC, &wakeup);
            wakeup.tv_nsec += (long)period_us * 1000L;
            while (wakeup.tv_nsec >= 1000000000L) {
                wakeup.tv_nsec -= 1000000000L;
                wakeup.tv_sec++;
            }

            while (waited_ms < activate_wait_ms) {
                int all_safeop = ctx->joint_count > 0;
                int all_op = ctx->joint_count > 0;
                ec_domain_state_t ds;

                clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                        &wakeup, NULL);
                wakeup.tv_nsec += (long)period_us * 1000L;
                while (wakeup.tv_nsec >= 1000000000L) {
                    wakeup.tv_nsec -= 1000000000L;
                    wakeup.tv_sec++;
                }
                waited_ms += period_us / 1000u;
                if (period_us / 1000u == 0) {
                    waited_ms += 1;
                }

                bootstrap_master_cycle(ctx, monotonic_time_ns());

                for (i = 0; i < ctx->joint_count; i++) {
                    ec_slave_config_state_t st;

                    memset(&st, 0, sizeof(st));
                    ecrt_slave_config_state(ctx->joints[i]->sc, &st);
                    if (!st.online ||
                            (st.al_state & 0x0f) < EC_AL_STATE_SAFEOP) {
                        all_safeop = 0;
                        all_op = 0;
                        break;
                    }
                    if (!st.operational ||
                            (st.al_state & 0x0f) < EC_AL_STATE_OP) {
                        all_op = 0;
                    }
                }

                if (all_safeop)
                    reached_safeop = 1;

                memset(&ds, 0, sizeof(ds));
                ecrt_domain_state(ctx->domain, &ds);
                if (all_op && ds.wc_state == EC_WC_COMPLETE) {
                    settle_ms += period_us / 1000u;
                    if (period_us / 1000u == 0)
                        settle_ms += 1;
                    if (settle_ms >= JSDK_ACTIVATE_SETTLE_MS) {
                        reached_op_settle = 1;
                        break;
                    }
                } else {
                    settle_ms = 0;
                }

                if (reached_safeop &&
                        waited_ms >= (ctx->config.wait_before_safeop_ms
                                ? ctx->config.wait_before_safeop_ms
                                : JSDK_WAIT_BEFORE_SAFEOP_MS) + 5000u &&
                        settle_ms == 0 && !all_op) {
                    break;
                }
            }
        }

        (void)reached_op_settle;

        if (reached_safeop) {
            ctx->activated = 1;
            ctx->sync_dc = 0;
            if (ctx->config.dc_timing_mode == JSDK_DC_TIMING_PER_JOINT) {
                for (i = 0; i < ctx->joint_count; i++) {
                    const jsdk_drive_model_t *m =
                            ctx->joints[i]->drive_model;
                    if (m ? m->use_dc : ctx->config.use_dc) {
                        ctx->sync_dc = 1;
                        break;
                    }
                }
            } else if (ctx->config.use_dc) {
                ctx->sync_dc = 1;
            }
            return JSDK_OK;
        }

        release_activation(ctx);
        jsdk_context_recover_bus(ctx);

        if (attempt == 0) {
            /* One automatic retry — product path without ethercatctl. */
            continue;
        }

        jsdk_set_error(ctx,
                "slaves did not reach SAFEOP within %u ms after activate "
                "(retried after bus recover; check: sudo dmesg | "
                "grep -i ethercat)",
                activate_wait_ms);
        return JSDK_ERR_ECRT;
    }

    return JSDK_ERR_ECRT;
}

void jsdk_context_deactivate(jsdk_context_t *ctx)
{
    unsigned int i;
    unsigned int n;
    unsigned int n_sync_off;
    unsigned int period_us;
    struct timespec ts;

    if (!ctx || !ctx->master || !ctx->activated) {
        return;
    }

    /* Request CiA402 disable on every joint and keep exchanging process data
     * long enough for Disable Op → Shutdown → Disable Voltage to complete.
     * Cutting the bus immediately left many drives enabled / freewheeling. */
    for (i = 0; i < ctx->joint_count; i++) {
        jsdk_joint_request_disable(ctx->joints[i]);
        ctx->joints[i]->command.target_velocity = 0;
        ctx->joints[i]->command.target_torque = 0;
    }

    period_us = ctx->config.period_ns / 1000u;
    if (period_us == 0) {
        period_us = 1000;
    }
    /*
     * CST coasts if disable frames are too short / bus already bad.
     * Send ~800 ms of zero-torque + disable ladder before releasing master.
     */
    n = 800000u / period_us;
    if (n < 100) {
        n = 100;
    }
    if (n > 4000) {
        n = 4000;
    }
    /* Stop DC clock sync for the last ~200 ms to reduce 0x001A on release. */
    n_sync_off = 200000u / period_us;
    if (n_sync_off < 20)
        n_sync_off = 20;
    if (n_sync_off > n)
        n_sync_off = n / 2u;

    ts.tv_sec = 0;
    ts.tv_nsec = (long)period_us * 1000L;

    for (i = 0; i < n; i++) {
        uint64_t now;
        struct timespec now_ts;
        unsigned int j;

        if (i + n_sync_off >= n)
            ctx->sync_dc = 0;

        clock_gettime(CLOCK_MONOTONIC, &now_ts);
        now = (uint64_t)now_ts.tv_sec * 1000000000ULL
                + (uint64_t)now_ts.tv_nsec;

        for (j = 0; j < ctx->joint_count; j++) {
            ctx->joints[j]->command.target_velocity = 0;
            ctx->joints[j]->command.target_torque = 0;
        }

        if (jsdk_context_cycle_begin(ctx, now) != JSDK_OK) {
            break;
        }
        if (jsdk_context_cycle_end(ctx) != JSDK_OK) {
            break;
        }
        nanosleep(&ts, NULL);
    }

    release_activation(ctx);
    /* Leave bus without sticky SAFEOP+ERROR for the next process start. */
    jsdk_context_recover_bus(ctx);
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

    {
        ec_domain_state_t ds;

        memset(&ds, 0, sizeof(ds));
        ecrt_domain_state(ctx->domain, &ds);
        ctx->domain_wc_ok = (ds.wc_state == EC_WC_COMPLETE);
    }

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

    if (ctx->sync_dc) {
        ecrt_master_sync_reference_clock(ctx->master);
        ecrt_master_sync_slave_clocks(ctx->master);
    }
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

void jsdk_joint_request_quick_stop(jsdk_joint_t *joint)
{
    if (!joint) {
        return;
    }
    joint->command.target_velocity = 0;
    joint->command.target_torque = 0;
    jsdk_cia402_request_quick_stop(&joint->cia402);
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
    if (!joint)
        return;
    /*
     * During an incomplete WC frame the domain image is garbage — keep
     * last command. Do NOT reject app setpoints for the whole pos_hold
     * window: that left printed cmd racing while PDO stayed at actual.
     * write_joint_command() still mirrors actual while pos_hold>0.
     */
    if (joint->ctx && !joint->ctx->domain_wc_ok)
        return;
    joint->command.target_position = value;
}

void jsdk_joint_set_target_velocity(jsdk_joint_t *joint, int32_t value)
{
    if (!joint)
        return;
    /*
     * CSV/CST: only force-zero after sustained WC loss. Brief holes keep
     * streaming the app command so recovery is not a 0→full step.
     */
    if (joint->ctx && !joint->ctx->domain_wc_ok &&
            joint->wc_miss_cycles >= JSDK_CSV_WC_HOLD_CYCLES) {
        joint->command.target_velocity = 0;
        return;
    }
    joint->command.target_velocity = value;
}

void jsdk_joint_set_target_torque(jsdk_joint_t *joint, int16_t value)
{
    if (!joint)
        return;
    if (joint->ctx && !joint->ctx->domain_wc_ok &&
            joint->wc_miss_cycles >= JSDK_CSV_WC_HOLD_CYCLES) {
        joint->command.target_torque = 0;
        return;
    }
    joint->command.target_torque = value;
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
