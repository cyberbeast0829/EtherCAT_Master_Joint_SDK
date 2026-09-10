#include <string.h>

#include "internal.h"

#define CYBERBEAST_VENDOR_ID 0x000C0B00u
/* Bus device ISVD90RC-300B-100-70 reports 0x00080117 (not ESI FL90BLW14 0x80153). */
#define CYBERBEAST_JOINT_PRODUCT_CODE 0x00080117u
#define CYBERBEAST_JOINT_REVISION_NO 0x00000001u
#define CYBERBEAST_JOINT_DC_ASSIGN_ACTIVATE 0x0300u
#define CYBERBEAST_JOINT_MIN_CYCLE_NS 50000u

static const ec_pdo_entry_info_t cb_joint_rx_entries[] = {
    {JSDK_CIA402_CONTROLWORD,       0, 16},
    {JSDK_CIA402_TARGET_POSITION,   0, 32},
    {JSDK_CIA402_TARGET_VELOCITY,   0, 32},
    {JSDK_CIA402_TARGET_TORQUE,     0, 16},
    {JSDK_CIA402_MODE_OF_OPERATION, 0,  8},
    {0x0000,                        0,  8},
};

static const ec_pdo_entry_info_t cb_joint_tx_entries[] = {
    {JSDK_CIA402_STATUSWORD,        0, 16},
    {JSDK_CIA402_ACTUAL_POSITION,   0, 32},
    {JSDK_CIA402_ACTUAL_VELOCITY,   0, 32},
    {JSDK_CIA402_ACTUAL_TORQUE,     0, 16},
    {JSDK_CIA402_MODE_DISPLAY,      0,  8},
    {0x0000,                        0,  8},
};

static const ec_pdo_info_t cb_joint_rx_pdos[] = {
    {0x1600, 6, cb_joint_rx_entries},
};

static const ec_pdo_info_t cb_joint_tx_pdos[] = {
    {0x1A00, 6, cb_joint_tx_entries},
};

static const ec_sync_info_t cb_joint_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT,  0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, cb_joint_rx_pdos, EC_WD_ENABLE},
    {3, EC_DIR_INPUT,  1, cb_joint_tx_pdos, EC_WD_DISABLE},
    {0xff}
};

static const jsdk_joint_profile_t cb_joint_profile = {
    JSDK_PROFILE_CYBERBEAST_JOINT_MODULE,
    CYBERBEAST_VENDOR_ID,
    CYBERBEAST_JOINT_PRODUCT_CODE,
    CYBERBEAST_JOINT_REVISION_NO,
    CYBERBEAST_JOINT_DC_ASSIGN_ACTIVATE,
    CYBERBEAST_JOINT_MIN_CYCLE_NS,
    JSDK_MODE_CSP,
    JSDK_MODE_CSV,
    JSDK_MODE_CST,
    cb_joint_syncs,
    /* PDO entry arrays (same order as ec_pdo_entry_info_t above) */
    6, 6,
    cb_joint_rx_entries,
    cb_joint_tx_entries,
    NULL  /* heap_block: static profile, no heap */
};

const jsdk_joint_profile_t *jsdk_profile_cyberbeast_joint_module(void)
{
    return &cb_joint_profile;
}

const jsdk_joint_profile_t *jsdk_profile_find(const char *name)
{
    if (!name || !name[0] ||
            !strcmp(name, JSDK_PROFILE_CYBERBEAST_JOINT_MODULE) ||
            !strcmp(name, JSDK_PROFILE_CYBERBEAST_JOINT_MODULE_ID) ||
            !strcmp(name, "CyberBeastJointModule") ||
            !strcmp(name, "cyberbeast-joint-module") ||
            !strcmp(name, "CyberBeast") ||
            !strcmp(name, "FL90BLW14") ||
            !strcmp(name, "cyberbeast_fl90blw14") ||
            !strcmp(name, "ISVD90RC-300B-100-70") ||
            !strcmp(name, "ISVD90RC") ||
            !strcmp(name, "default")) {
        return &cb_joint_profile;
    }

    return NULL;
}
