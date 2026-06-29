#ifndef JSDK_ESI_PARSER_H
#define JSDK_ESI_PARSER_H

#include "internal.h"

/* 从 ESI XML 文件加载关节 profile。
 *
 *   esi_path     ESI XML 文件路径 (如 "./ECAT_CIA402.xml")
 *   vendor_id    期望的 vendor_id (0 = 不校验, 取文件中第一个)
 *   product_code 期望的 product_code (0 = 不校验, 取文件中第一个)
 *
 *   返回 heap-allocated jsdk_joint_profile_t*, 调用者最终须用
 *   jsdk_profile_destroy() 释放。
 *   失败返回 NULL。
 */
const jsdk_joint_profile_t *esi_profile_load(
        const char *esi_path,
        uint32_t vendor_id,
        uint32_t product_code);

/* 释放 esi_profile_load() 返回的 profile。对编译期静态 profile 无操作。 */
void jsdk_profile_destroy(const jsdk_joint_profile_t *profile);

#endif
