# 机器人关节 EtherCAT SDK

守护兽（CyberBeast）机器人关节的 EtherCAT 集成 SDK。基于 IgH EtherCAT Master，封装 CiA402 协议，让客户几行代码即可驱动关节。

## 特性

- **三模式**：CSP（周期同步位置）/ CSV（周期同步速度）/ CST（周期同步力矩）
- **多型号免重编译**：ESI XML 动态加载，30+ 型号只需一份 XML 文件
- **异步 SDO**：运行期非阻塞读写参数（PID 增益、限值、软限位）
- **故障诊断闭环**：自动读取 0x603F/0x1001/0x203F/0x203E + 回调通知
- **单位换算**：指令单位 ↔ rad/rad·s⁻¹/N·m + 位置回绕修正
- **C ABI + C++ 外观层**：稳定 C 接口 + header-only C++ 封装（RAII、内置 RT 线程）
- **多轴**：单 domain 容纳 1~10+ 轴

## 快速开始

```bash
# 1. 编译
cd joint-sdk
make ETHERLAB_DIR=/opt/etherlab

# 2. 运行单轴 CSP 示例
sudo ./build/csp_single_sdk

# 3. 运行 C++ 外观层示例（内置 RT 线程）
sudo ./build/cpp_group_csp
```

## 最小示例（C，形态 B：嵌入客户 RT 循环）

```c
#include <joint_sdk/joint_sdk.h>

int main(void)
{
    jsdk_context_t *ctx;
    jsdk_joint_t *joint;
    jsdk_context_config_t cfg;
    jsdk_joint_config_t jcfg;

    jsdk_context_config_default(&cfg);
    cfg.period_ns = 1000000;   /* 1 ms */
    ctx = jsdk_context_create(&cfg);

    jcfg.alias = 0;
    jcfg.position = 0;
    jcfg.profile_name = "./ECAT_CIA402.xml";   /* ESI 动态加载 */
    jsdk_context_add_joint(ctx, &jcfg, &joint);
    jsdk_context_activate(ctx);

    jsdk_joint_request_enable(joint, JSDK_MODE_CSP);

    while (running) {
        jsdk_joint_feedback_t fb;
        uint64_t now = app_time_ns();

        jsdk_context_cycle_begin(ctx, now);
        jsdk_joint_get_feedback(joint, &fb);
        jsdk_joint_set_target_position(joint, fb.actual_position);
        jsdk_context_cycle_end(ctx);
    }

    jsdk_context_destroy(ctx);
    return 0;
}
```

## 最小示例（C++，形态 A：内置 RT 线程）

```cpp
#include <joint_sdk/joint_group.hpp>

int main()
{
    jsdk::JointGroup group(0, 1000000, 1);   // master0, 1ms, 1轴
    jsdk::Joint &j = group.addJoint(0, 0, "./ECAT_CIA402.xml");
    group.activate();
    j.enable(JSDK_MODE_CSP);

    group.start([&](jsdk::JointGroup &g) {
        jsdk::Joint &axis = g.joint(0);
        axis.setTargetPositionRad(0.5);       // 目标 0.5 rad
    });

    /* 主线程等待 Ctrl-C */
    group.stop();
    return 0;
}
```

## 三模式切换

```c
/* CSP：周期下发目标位置 */
jsdk_joint_request_enable(joint, JSDK_MODE_CSP);
jsdk_joint_set_target_position(joint, 16384);

/* CSV：周期下发目标速度 */
jsdk_joint_request_enable(joint, JSDK_MODE_CSV);
jsdk_joint_set_target_velocity(joint, 1000);

/* CST：周期下发目标力矩（0.1% 额定转矩） */
jsdk_joint_request_enable(joint, JSDK_MODE_CST);
jsdk_joint_set_target_torque(joint, 100);
```

## 物理量接口

```c
/* 设置换算系数（默认 encoder=16384, gear=8:1, rated_trq=10 N·m） */
jsdk_unit_scale_t scale;
jsdk_unit_scale_default(&scale, 10000);
jsdk_joint_set_scale(joint, &scale);

/* 直接用物理量 */
jsdk_joint_set_target_position_rad(joint, 0.5);
double q = jsdk_joint_actual_position_rad(joint);       /* rad */
double w = jsdk_joint_actual_velocity_rad_s(joint);     /* rad/s */
double t = jsdk_joint_actual_torque_Nm(joint);          /* N·m */
```

## 异步 SDO 读写

```c
/* 激活前创建句柄 */
jsdk_sdo_handle_t h = jsdk_joint_sdo_create(joint, 0x2008, 4, 2); /* 位置环 Kp */

/* 周期内读 */
if (jsdk_joint_sdo_state(joint, h) != JSDK_SDO_BUSY)
    jsdk_joint_sdo_read(joint, h);
if (jsdk_joint_sdo_state(joint, h) == JSDK_SDO_SUCCESS) {
    uint16_t kp = *(uint16_t *)jsdk_joint_sdo_data(joint, h);
}

/* 周期内写 */
if (jsdk_joint_sdo_state(joint, h) != JSDK_SDO_BUSY) {
    *(uint16_t *)jsdk_joint_sdo_data(joint, h) = 1800;
    jsdk_joint_sdo_write(joint, h);
}
```

## 故障诊断

```c
/* 注册回调（激活前） */
static void on_fault(jsdk_joint_t *j, const jsdk_fault_info_t *info, void *ud)
{
    printf("FAULT: 603F=0x%04X vendor=0x%08X%08X\n",
           info->code_603f, info->vendor_hi, info->vendor_lo);
}
jsdk_context_set_fault_callback(ctx, on_fault, NULL);

/* 同步查询（周期内） */
jsdk_fault_info_t fi;
if (jsdk_joint_get_fault_info(joint, &fi)) {
    /* 有故障，处理 */
}
```

## 文档

- [INSTALL.md](INSTALL.md) — 安装与依赖
- [ARCHITECTURE.zh-CN.md](ARCHITECTURE.zh-CN.md) — 架构设计
- [守护兽驱动ETHERCAT协议手册.md](守护兽驱动ETHERCAT协议手册.md) — 关节协议参考

## 许可

内部使用，随公司产品分发。详见 [../COPYING](../COPYING)。
