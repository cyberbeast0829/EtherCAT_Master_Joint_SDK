/*****************************************************************************
 *
 *  守护兽关节 SDK C++ 外观层示例（形态 A：内置 RT 线程）
 *
 *  演示使用 JointGroup / Joint 类：
 *    - RAII 生命周期
 *    - 语义化 API（enable / setTargetPosition / actualPosition）
 *    - 内置 RT 线程 + lambda 周期回调
 *
 *  用法:
 *    ./cpp_group_csp             # 单轴正弦
 *    ./cpp_group_csp --hold      # 静止
 *
 ****************************************************************************/

#define _POSIX_C_SOURCE 200809L

#include <cmath>
#include <cstdio>
#include <cstring>
#include <csignal>

#include <joint_sdk/joint_group.hpp>

#define PI 3.14159265358979323846
#define AMPLITUDE 5000.0
#define FREQ_HZ 0.25

static volatile sig_atomic_t g_running = 1;

static void on_signal(int sig) { (void)sig; g_running = 0; }

int main(int argc, char **argv)
{
    bool hold = false;
    for (int i = 1; i < argc; i++)
        if (!strcmp(argv[i], "--hold")) hold = true;

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    using namespace jsdk;

    /* RAII：构造即 create，析构即 destroy */
    JointGroup group(0, 1000000, 1);
    if (!group.valid()) {
        std::fprintf(stderr, "failed to create joint group\n");
        return 1;
    }

    /* 添加关节（可链式） */
    Joint &j = group.addJoint(0, 0, "./ECAT_CIA402.xml");
    if (!j.raw()) {
        std::fprintf(stderr, "add joint failed: %s\n", group.lastError());
        return 1;
    }

    /* 激活主站 */
    if (!group.activate()) {
        std::fprintf(stderr, "activate failed: %s\n", group.lastError());
        return 1;
    }

    /* 使能 CSP */
    j.enable(JSDK_MODE_CSP);

    std::printf("=== C++ JointGroup CSP Example ===\n");
    std::printf("hold=%s\n", hold ? "YES" : "NO");

    /* 形态 A：内置 RT 线程 + lambda 回调 */
    double traj_time = 0.0;
    int32_t home = 0;
    int home_captured = 0;
    unsigned int slow = 0;

    bool started = group.start([&](JointGroup &g) {
        Joint &axis = g.joint(0);
        auto fb = axis.feedback();

        if (axis.isEnabled()) {
            if (!home_captured) {
                home = fb.actual_position;
                home_captured = 1;
                traj_time = 0.0;
                std::printf("\n>>> ENABLED (home=%d)\n\n", home);
            }

            if (hold) {
                axis.setTargetPosition(home);
            } else {
                traj_time += 1e-6; /* 1 ms 周期 */
                int32_t target = home + (int32_t)(AMPLITUDE *
                        std::sin(2.0 * PI * FREQ_HZ * traj_time));
                axis.setTargetPosition(target);
            }
        } else {
            home_captured = 0;
        }

        if (!slow--) {
            slow = 1000;
            auto bus = g.busState();
            std::printf("axis=%s status=0x%04X mode=%d pos=%d "
                        "wc=%u slaves=%u al=0x%02X\n",
                        jsdk_axis_state_string(fb.axis_state),
                        fb.statusword, fb.mode_display,
                        fb.actual_position, bus.working_counter,
                        bus.slaves_responding, bus.al_states);
        }
    });

    if (!started) {
        std::fprintf(stderr, "failed to start RT thread\n");
        return 1;
    }

    /* 主线程等待 Ctrl-C */
    while (g_running) {
        timespec ts{0, 100000000}; /* 100 ms */
        nanosleep(&ts, nullptr);
    }

    std::printf("\nStopping...\n");
    j.disable();
    group.stop();   /* 停止 RT 线程 */

    return 0;
}
