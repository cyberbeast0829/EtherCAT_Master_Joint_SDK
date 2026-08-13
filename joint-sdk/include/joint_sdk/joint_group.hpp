#ifndef JSDK_JOINT_GROUP_HPP
#define JSDK_JOINT_GROUP_HPP
/*
 *  L4 C++ 外观层：JointGroup / Joint
 *
 *  header-only，仅依赖 C ABI <joint_sdk/joint_sdk.h>。
 *  提供：
 *    - RAII 生命周期：JointGroup 构造 = create + add_joints，析构 = destroy
 *    - 语义化 API：joint.enable() / joint.setTargetPosition() / ...
 *    - 形态 A：内置 RT 线程 + std::function 周期回调
 *    - 形态 B：手动调用 group.cycle(callback) 嵌入客户已有循环
 */

#include <joint_sdk/joint_sdk.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <functional>
#include <thread>
#include <vector>

namespace jsdk {

using CycleCallback = std::function<void(class JointGroup &)>;

class Joint {
public:
    explicit Joint(jsdk_joint_t *handle) : handle_(handle) {}

    jsdk_joint_t *raw() { return handle_; }
    const jsdk_joint_t *raw() const { return handle_; }

    /* 状态机 */
    void enable(jsdk_mode_t mode = JSDK_MODE_CSP) {
        jsdk_joint_request_enable(handle_, mode);
    }
    void disable() { jsdk_joint_request_disable(handle_); }
    void faultReset() { jsdk_joint_request_fault_reset(handle_); }
    void setMode(jsdk_mode_t mode) { jsdk_joint_set_mode(handle_, mode); }

    /* 指令单位 */
    void setTargetPosition(int32_t v) {
        jsdk_joint_set_target_position(handle_, v);
    }
    void setTargetVelocity(int32_t v) {
        jsdk_joint_set_target_velocity(handle_, v);
    }
    void setTargetTorque(int16_t v) {
        jsdk_joint_set_target_torque(handle_, v);
    }

    /* 物理量 */
    void setTargetPositionRad(double rad) {
        jsdk_joint_set_target_position_rad(handle_, rad);
    }
    void setTargetVelocityRadS(double rad_s) {
        jsdk_joint_set_target_velocity_rad_s(handle_, rad_s);
    }
    void setTargetTorqueNm(double Nm) {
        jsdk_joint_set_target_torque_Nm(handle_, Nm);
    }

    double actualPositionRad() {
        return jsdk_joint_actual_position_rad(handle_);
    }
    double actualVelocityRadS() {
        return jsdk_joint_actual_velocity_rad_s(handle_);
    }
    double actualTorqueNm() {
        return jsdk_joint_actual_torque_Nm(handle_);
    }

    jsdk_joint_feedback_t feedback() const {
        jsdk_joint_feedback_t f;
        memset(&f, 0, sizeof(f));
        jsdk_joint_get_feedback(handle_, &f);
        return f;
    }

    bool isEnabled() const { return jsdk_joint_is_enabled(handle_) != 0; }
    bool isFault() const { return jsdk_joint_is_fault(handle_) != 0; }

    /* 换算系数 */
    void setScale(const jsdk_unit_scale_t &s) {
        jsdk_joint_set_scale(handle_, &s);
    }
    void getScale(jsdk_unit_scale_t &s) const {
        jsdk_joint_get_scale(handle_, &s);
    }

private:
    jsdk_joint_t *handle_;
};

class JointGroup {
public:
    JointGroup(unsigned int master_index = 0,
               uint32_t period_ns = JSDK_DEFAULT_PERIOD_NS,
               unsigned int max_joints = JSDK_DEFAULT_MAX_JOINTS) {
        jsdk_context_config_t cfg;
        jsdk_context_config_default(&cfg);
        cfg.master_index = master_index;
        cfg.period_ns = period_ns;
        cfg.max_joints = max_joints;
        ctx_ = jsdk_context_create(&cfg);
        period_ns_ = period_ns;
    }

    ~JointGroup() {
        stop();
        if (ctx_) {
            jsdk_context_destroy(ctx_);
            ctx_ = nullptr;
        }
    }

    /* 禁用拷贝，允许移动 */
    JointGroup(const JointGroup &) = delete;
    JointGroup &operator=(const JointGroup &) = delete;

    bool valid() const { return ctx_ != nullptr; }

    /* 添加关节（激活前调用），返回引用方便链式配置 */
    Joint &addJoint(uint16_t alias, uint16_t position,
                    const char *profile_name) {
        jsdk_joint_config_t cfg;
        memset(&cfg, 0, sizeof(cfg));
        cfg.alias = alias;
        cfg.position = position;
        cfg.profile_name = profile_name;

        jsdk_joint_t *h = nullptr;
        jsdk_status_t st = jsdk_context_add_joint(ctx_, &cfg, &h);
        joints_.emplace_back(h ? h : nullptr);
        return joints_.back();
    }

    /* 激活主站（等价于 configure + activate） */
    bool activate() {
        return jsdk_context_activate(ctx_) == JSDK_OK;
    }

    void deactivate() { jsdk_context_deactivate(ctx_); }

    /* 形态 B：手动单周期。在客户已有 RT 循环中调用。 */
    bool cycle(CycleCallback cb, uint64_t app_time_ns = 0) {
        if (!app_time_ns) {
            app_time_ns = monotonicNs();
        }
        jsdk_status_t st = jsdk_context_cycle_begin(ctx_, app_time_ns);
        if (st != JSDK_OK) return false;
        if (cb) cb(*this);
        return jsdk_context_cycle_end(ctx_) == JSDK_OK;
    }

    /* 形态 A：内置 RT 线程。周期由构造时 period_ns 决定。 */
    bool start(CycleCallback cb) {
        if (running_.load()) return false;
        callback_ = std::move(cb);
        running_.store(true);
        rt_thread_ = std::thread(&JointGroup::rtLoop, this);
        return true;
    }

    void stop() {
        running_.store(false);
        if (rt_thread_.joinable()) {
            rt_thread_.join();
        }
    }

    bool isRunning() const { return running_.load(); }

    Joint &joint(size_t index) { return joints_.at(index); }
    const Joint &joint(size_t index) const { return joints_.at(index); }
    size_t size() const { return joints_.size(); }

    jsdk_bus_state_t busState() const {
        jsdk_bus_state_t s;
        memset(&s, 0, sizeof(s));
        jsdk_context_get_bus_state(ctx_, &s);
        return s;
    }

    const char *lastError() const {
        return jsdk_context_last_error(ctx_);
    }

    uint32_t periodNs() const { return period_ns_; }

    jsdk_context_t *raw() { return ctx_; }

private:
    static uint64_t monotonicNs() {
        using namespace std::chrono;
        return (uint64_t)duration_cast<nanoseconds>(
                steady_clock::now().time_since_epoch()).count();
    }

    void rtLoop() {
        const long NSEC = 1000000000L;
        uint64_t period = period_ns_;
        timespec wakeup{};
        clock_gettime(CLOCK_MONOTONIC, &wakeup);
        wakeup.tv_sec += 1;  /* 延迟 1 秒启动，留出初始化时间 */
        wakeup.tv_nsec = 0;

        while (running_.load()) {
            int ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                                      &wakeup, nullptr);
            if (ret) {
                if (ret != EINTR) break;
                continue;
            }

            uint64_t now = (uint64_t)wakeup.tv_sec * NSEC
                           + (uint64_t)wakeup.tv_nsec;
            jsdk_status_t st = jsdk_context_cycle_begin(ctx_, now);
            if (st != JSDK_OK) break;

            if (callback_) callback_(*this);

            st = jsdk_context_cycle_end(ctx_);
            if (st != JSDK_OK) break;

            wakeup.tv_nsec += (long)period;
            while (wakeup.tv_nsec >= NSEC) {
                wakeup.tv_nsec -= NSEC;
                wakeup.tv_sec++;
            }
        }
        running_.store(false);
    }

    jsdk_context_t *ctx_ = nullptr;
    uint32_t period_ns_ = JSDK_DEFAULT_PERIOD_NS;
    std::vector<Joint> joints_;
    std::thread rt_thread_;
    std::atomic<bool> running_{false};
    CycleCallback callback_;
};

} // namespace jsdk

#endif // JSDK_JOINT_GROUP_HPP
