#include "internal.h"

static jsdk_axis_state_t decode_state(uint16_t statusword)
{
    if (statusword & JSDK_CIA402_SW_FAULT) {
        return JSDK_AXIS_FAULT;
    }

    switch (statusword & JSDK_CIA402_SW_MASK) {
    case JSDK_CIA402_SW_SWITCH_ON_DISABLED:
        return JSDK_AXIS_SWITCH_ON_DISABLED;
    case JSDK_CIA402_SW_READY_TO_SWITCH_ON:
        return JSDK_AXIS_READY_TO_SWITCH_ON;
    case JSDK_CIA402_SW_SWITCHED_ON:
        return JSDK_AXIS_SWITCHED_ON;
    case JSDK_CIA402_SW_OPERATION_ENABLED:
        return JSDK_AXIS_OPERATION_ENABLED;
    default:
        return JSDK_AXIS_UNKNOWN;
    }
}

void jsdk_cia402_init(jsdk_cia402_axis_t *axis, int8_t default_mode)
{
    if (!axis) {
        return;
    }

    axis->enable_requested = 0;
    axis->fault_reset_requested = 0;
    axis->quick_stop_requested = 0;
    axis->requested_mode = default_mode;
    axis->controlword = JSDK_CIA402_CW_SHUTDOWN;
    axis->axis_state = JSDK_AXIS_UNKNOWN;
    axis->operation_enabled = 0;
}

void jsdk_cia402_request_enable(jsdk_cia402_axis_t *axis, int8_t mode)
{
    if (!axis) {
        return;
    }

    axis->requested_mode = mode;
    axis->enable_requested = 1;
}

void jsdk_cia402_request_disable(jsdk_cia402_axis_t *axis)
{
    if (!axis) {
        return;
    }

    axis->enable_requested = 0;
    axis->quick_stop_requested = 0;
    axis->operation_enabled = 0;
    /* Immediate command; update() will step the full shutdown ladder. */
    axis->controlword = JSDK_CIA402_CW_DISABLE_OP;
}

void jsdk_cia402_request_quick_stop(jsdk_cia402_axis_t *axis)
{
    if (!axis) {
        return;
    }

    axis->enable_requested = 0;
    axis->operation_enabled = 0;
    axis->quick_stop_requested = 1;
    axis->controlword = JSDK_CIA402_CW_QUICK_STOP;
}

void jsdk_cia402_request_fault_reset(jsdk_cia402_axis_t *axis)
{
    if (!axis) {
        return;
    }

    axis->fault_reset_requested = 1;
}

void jsdk_cia402_update(jsdk_cia402_axis_t *axis, uint16_t statusword)
{
    if (!axis) {
        return;
    }

    axis->axis_state = decode_state(statusword);
    axis->operation_enabled = 0;

    if (axis->axis_state == JSDK_AXIS_FAULT) {
        if (axis->fault_reset_requested || axis->enable_requested) {
            axis->controlword = JSDK_CIA402_CW_FAULT_RESET;
            axis->fault_reset_requested = 0;
        } else {
            axis->controlword = JSDK_CIA402_CW_DISABLE_VOLTAGE;
        }
        return;
    }

    axis->fault_reset_requested = 0;

    /* Quick Stop (0x0002): brake ramp, then fall through to disable ladder. */
    if (axis->quick_stop_requested) {
        if (axis->axis_state == JSDK_AXIS_OPERATION_ENABLED ||
                axis->axis_state == JSDK_AXIS_SWITCHED_ON) {
            axis->controlword = JSDK_CIA402_CW_QUICK_STOP;
            return;
        }
        axis->quick_stop_requested = 0;
        axis->controlword = JSDK_CIA402_CW_DISABLE_VOLTAGE;
        return;
    }

    /*
     * Controlled stop ladder (CiA402):
     *   Operation enabled → Disable operation (0x0007)
     *   Switched on       → Shutdown (0x0006)
     *   Ready to switch on / Switch on disabled → Disable voltage (0x0000)
     *
     * Previously we always wrote DISABLE_OP and then cut the bus after one
     * cycle, so the drive often stayed enabled / freewheeled (esp. in CST).
     */
    if (!axis->enable_requested) {
        switch (axis->axis_state) {
        case JSDK_AXIS_OPERATION_ENABLED:
            axis->controlword = JSDK_CIA402_CW_DISABLE_OP;
            break;
        case JSDK_AXIS_SWITCHED_ON:
            axis->controlword = JSDK_CIA402_CW_SHUTDOWN;
            break;
        case JSDK_AXIS_READY_TO_SWITCH_ON:
        case JSDK_AXIS_SWITCH_ON_DISABLED:
        default:
            axis->controlword = JSDK_CIA402_CW_DISABLE_VOLTAGE;
            break;
        }
        return;
    }

    switch (axis->axis_state) {
    case JSDK_AXIS_SWITCH_ON_DISABLED:
        axis->controlword = JSDK_CIA402_CW_SHUTDOWN;
        break;
    case JSDK_AXIS_READY_TO_SWITCH_ON:
        axis->controlword = JSDK_CIA402_CW_SWITCH_ON;
        break;
    case JSDK_AXIS_SWITCHED_ON:
        axis->controlword = JSDK_CIA402_CW_ENABLE_OP;
        break;
    case JSDK_AXIS_OPERATION_ENABLED:
        axis->controlword = JSDK_CIA402_CW_ENABLE_OP;
        axis->operation_enabled = 1;
        break;
    default:
        axis->controlword = JSDK_CIA402_CW_SHUTDOWN;
        break;
    }
}

const char *jsdk_axis_state_string(jsdk_axis_state_t state)
{
    switch (state) {
    case JSDK_AXIS_UNKNOWN:
        return "unknown";
    case JSDK_AXIS_SWITCH_ON_DISABLED:
        return "switch_on_disabled";
    case JSDK_AXIS_READY_TO_SWITCH_ON:
        return "ready_to_switch_on";
    case JSDK_AXIS_SWITCHED_ON:
        return "switched_on";
    case JSDK_AXIS_OPERATION_ENABLED:
        return "operation_enabled";
    case JSDK_AXIS_FAULT:
        return "fault";
    default:
        return "invalid";
    }
}
