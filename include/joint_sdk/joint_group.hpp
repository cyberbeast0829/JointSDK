/**
 * @file    joint_group.hpp
 * @brief   `jsdk_can` 的 C++ 包装（header-only，RAII，C++14）
 *
 * 设计取舍（都不是随便定的）：
 *
 * 1. **只依赖 `joint_sdk.h`**：不包含任何内置后端。C++ 客户既可以用
 *    `jsdk_hal_virtual_open()` 之类的工厂（那就自己 include `jsdk_hal_builtin.h`），
 *    也可以自己实现 `jsdk_can_hal_t`（MCU 常见），包装层不掺和。
 * 2. **不拥有传输层**：`Group` 不管 `jsdk_hal_handle_t` 的生死 ——
 *    句柄的关闭时机属于客户（可能是全局单例）。用 `jsdk_hal_builtin.h` 的工厂时，
 *    请自己 `jsdk_hal_close()`（示例见 `examples/08_cpp_wrapper.cpp`）。
 * 3. **不可拷贝、不可移动**：上下文会把 `&cfg.desc.arena_used` 存下来以便回写，
 *    而 `cfg` / arena 都是本对象的成员 —— 一旦被移动，那个指针就指向"被搬空的
 *    那个对象"了。与其提供一个会静默出错的移动构造，不如直接禁止（写错了编译不过）。
 * 4. **失败抛异常**：`Error` 携带 `jsdk_status_t` + 可读文本（含 `last_error()`）。
 *    控制回路里每周期抛异常代价高、也不该发生，所以**只有配置阶段**的 API 抛；
 *    周期内的 `cycle_begin/end` 返回状态码，由调用者决定（与 C API 一致）。
 * 5. **零拷贝的反馈**：`Joint::feedback()` 直接返回 C 结构体副本（POD，几百字节）。
 *
 * 典型用法：
 * @code
 *   jsdk_can_hal_t hal; jsdk_hal_handle_t* hh = nullptr;
 *   jsdk_hal_virtual_open(&hal, &hh, "0:id=1,gear=16.5,timeout=30000,fd");
 *   jsdk_hal_virtual_set_autotick(hh, 1);          // 仿真：让阻塞式配置 API 能跑完
 *
 *   jsdk::Group ctx(hal);                           // 默认配置 + 自动 arena
 *   jsdk::Joint& j = ctx.add_joint(1);
 *   ctx.configure();
 *   ctx.activate();
 *   ctx.run(100, [&](unsigned k) { j.set_mit(0.1 * k / 100.0, 0.0, 2.0, 0.2, 0.0); });
 *   ctx.deactivate();
 *   jsdk_hal_close(hh);
 * @endcode
 */

#ifndef JSDK_JOINT_GROUP_HPP
#define JSDK_JOINT_GROUP_HPP

#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include "joint_sdk.h"

namespace jsdk {

/* ==========================================================================
 * 端点条目（`endpoints()` 的返回值）
 * ======================================================================== */

struct Endpoint {
    std::string    path;
    uint16_t       id;
    jsdk_ep_type_t type;
    uint8_t        access;
};

/* ==========================================================================
 * 异常
 * ======================================================================== */

class Error : public std::runtime_error {
public:
    Error(jsdk_status_t code, const std::string &what)
        : std::runtime_error(what), code_(code) {}

    /** 原始状态码（可与 `JSDK_ERR_*` 比对；注意它不是 errno）。 */
    jsdk_status_t code() const noexcept { return code_; }

private:
    jsdk_status_t code_;
};

/** 统一的"检查 + 抛"。`ctx` 非空时把 `last_error()` 也带上（现场诊断全靠它）。 */
inline void check(jsdk_status_t st, const char *what,
                  jsdk_context_t *ctx = nullptr)
{
    if (st == JSDK_OK) return;

    std::string msg = what ? what : "jsdk";
    msg += ": ";
    msg += jsdk_status_string(st);

    if (ctx) {
        const char *detail = jsdk_context_last_error(ctx);
        if (detail && *detail) {
            msg += " (";
            msg += detail;
            msg += ")";
        }
    }
    throw Error(st, msg);
}

/* ==========================================================================
 * 关节
 * ======================================================================== */

class Group;

class Joint {
public:
    Joint() : j_(nullptr), owner_(nullptr) {}

    /** 是否已使能（**只有使能序列含安全首帧走完才为真**）。 */
    bool enabled() const noexcept { return jsdk_joint_is_enabled(j_) != 0; }

    /** 是否在线（收到过有效帧）。 */
    bool online() const noexcept
    {
        jsdk_joint_feedback_t fb;
        return jsdk_joint_get_feedback(j_, &fb) == JSDK_OK && fb.online != 0;
    }

    jsdk_joint_feedback_t feedback() const
    {
        jsdk_joint_feedback_t fb;
        check(jsdk_joint_get_feedback(j_, &fb), "get_feedback", ctx_());
        return fb;
    }

    /* --- 控制（周期内调用；不抛异常，错了记在 status_flags 里） --- */

    /** MIT：kp/kd 是**线上值** → 输出端等效刚度 = kp × gear/(2π)，见 set_mit_stiffness()。 */
    void set_mit(double pos_rad, double vel_rad_s, double kp, double kd, double tau_Nm) noexcept
    {
        jsdk_joint_set_mit(j_, pos_rad, vel_rad_s, kp, kd, tau_Nm);
    }

    /** 以**真实输出端刚度/阻尼**为输入（内部换算 kp = stiffness × 2π / gear）。 */
    void set_mit_stiffness(double pos_rad, double vel_rad_s,
                           double stiffness_Nm_per_rad, double damping_Nm_per_rad_s,
                           double tau_Nm) noexcept
    {
        jsdk_joint_set_mit_stiffness(j_, pos_rad, vel_rad_s, stiffness_Nm_per_rad,
                                     damping_Nm_per_rad_s, tau_Nm);
    }

    void set_target_position_rad(double rad) noexcept { jsdk_joint_set_target_position_rad(j_, rad); }
    void set_target_velocity_rad_s(double r) noexcept { jsdk_joint_set_target_velocity_rad_s(j_, r); }
    void set_target_torque_Nm(double nm) noexcept { jsdk_joint_set_target_torque_Nm(j_, nm); }

    /* --- 生命周期（非阻塞请求：要跑若干周期才生效） --- */

    void request_enable(jsdk_mode_t mode = JSDK_MODE_MIT) noexcept { jsdk_joint_request_enable(j_, mode); }
    void request_disable() noexcept { jsdk_joint_request_disable(j_); }
    void set_mode(jsdk_mode_t mode) noexcept { jsdk_joint_set_mode(j_, mode); }

    /* --- 参数（按路径；抛异常） --- */

    /** 支持的 T：`float` / `uint32_t` / `int32_t` / `bool`（其它类型编译期报错）。 */
    template <class T> T param(const char *path) const
    {
        (void)path;
        static_assert(sizeof(T) == 0,
                      "Joint::param<T>: 只支持 float / uint32_t / int32_t / bool");
        return T();
    }

    /** 支持的 T：`float` / `uint32_t`（i32/bool 请用 C API 的 jsdk_joint_param_set）。 */
    template <class T> void set_param(const char *path, T v)
    {
        (void)path; (void)v;
        static_assert(sizeof(T) == 0,
                      "Joint::set_param<T>: 只支持 float / uint32_t");
    }

    /** 逃生通道：拿原始 C 句柄。 */
    jsdk_joint_t *raw() noexcept { return j_; }

private:
    friend class Group;
    Joint(jsdk_joint_t *j, Group *owner) : j_(j), owner_(owner) {}
    jsdk_context_t *ctx_() const noexcept;

    jsdk_joint_t *j_;
    Group        *owner_;
};

template <> inline float Joint::param<float>(const char *path) const
{
    float v = 0.0f;
    check(jsdk_joint_param_get_f32(j_, path, &v), path, ctx_());
    return v;
}
template <> inline uint32_t Joint::param<uint32_t>(const char *path) const
{
    uint32_t v = 0u;
    check(jsdk_joint_param_get_u32(j_, path, &v), path, ctx_());
    return v;
}
template <> inline int32_t Joint::param<int32_t>(const char *path) const
{
    int32_t v = 0;
    check(jsdk_joint_param_get_i32(j_, path, &v), path, ctx_());
    return v;
}
template <> inline bool Joint::param<bool>(const char *path) const
{
    int v = 0;
    check(jsdk_joint_param_get_bool(j_, path, &v), path, ctx_());
    return v != 0;
}

template <> inline void Joint::set_param<float>(const char *path, float v)
{
    check(jsdk_joint_param_set_f32(j_, path, v), path, ctx_());
}
template <> inline void Joint::set_param<uint32_t>(const char *path, uint32_t v)
{
    check(jsdk_joint_param_set_u32(j_, path, v), path, ctx_());
}
/* 注：C API 目前只提供 f32/u32 两个类型化 setter；i32/bool 走
   `jsdk_joint_param_set(j, path, &value)`。包装层不自己造第三套语义。 */

/* ==========================================================================
 * 上下文（RAII）
 * ======================================================================== */

class Group {
public:
    /** 用默认配置构造（FD、100 Hz、RETAIN_ALL、自动 arena）。 */
    explicit Group(const jsdk_can_hal_t &hal) { init(hal, default_config()); }

    /** 用调用者给的配置构造；`cfg.desc.arena` 会被本对象**覆盖**成自管 arena。 */
    Group(const jsdk_can_hal_t &hal, const jsdk_context_config_t &cfg) { init(hal, cfg); }

    ~Group()
    {
        if (inited_) {
            jsdk_context_deactivate(ctx());   /* 发安全帧 + 等 2 周期 + STOP_MOTOR */
        }
    }

    Group(const Group &) = delete;
    Group &operator=(const Group &) = delete;
    Group(Group &&) = delete;
    Group &operator=(Group &&) = delete;

    /* --- 配置阶段（阻塞；失败抛异常） --- */

    /** 加一个关节（`node_id` 必须与总线上设备的实际节点号一致）。 */
    Joint &add_joint(uint8_t node_id, jsdk_mode_t mode = JSDK_MODE_MIT)
    {
        if (nj_ >= JSDK_MAX_JOINTS_STATIC) {
            throw Error(JSDK_ERR_NO_MEMORY, "add_joint: 超过 JSDK_MAX_JOINTS_STATIC");
        }
        jsdk_joint_config_t jc;
        std::memset(&jc, 0, sizeof jc);
        jc.node_id      = node_id;
        jc.initial_mode = mode;

        jsdk_joint_t *raw = nullptr;
        check(jsdk_context_add_joint(ctx(), &jc, &raw), "add_joint", ctx());
        joints_[nj_] = Joint(raw, this);
        return joints_[nj_++];
    }

    /** 下载描述符（41 KB / 数秒）—— 只有需要才调（可从缓存导入代替）。 */
    void desc_fetch() { check(jsdk_context_desc_fetch(ctx()), "desc_fetch", ctx()); }

    /** 读量程 + 校验控制周期能否喂得动设备的 break_timeout。 */
    void configure() { check(jsdk_context_configure(ctx()), "configure", ctx()); }

    /** 使能全部关节（阻塞直到序列走完；失败抛异常）。 */
    void activate() { check(jsdk_context_activate(ctx()), "activate", ctx()); }

    /** 失能全部关节（发安全帧 → 等 2 周期 → STOP_MOTOR）。 */
    void deactivate() noexcept { jsdk_context_deactivate(ctx()); }

    /** 广播急停（最高仲裁优先级；不需要任何前置条件）。 */
    void estop() noexcept { jsdk_context_estop(ctx()); }

    /* --- 控制循环 --- */

    jsdk_status_t cycle_begin(uint64_t app_time_ns) noexcept
    {
        return jsdk_context_cycle_begin(ctx(), app_time_ns);
    }

    jsdk_status_t cycle_end() noexcept { return jsdk_context_cycle_end(ctx()); }

    /**
     * 跑 `n` 个周期，每周期调用 `step(k)` 设置目标。
     *
     * 这就是"Form A 线程"（调用者拥有循环）的 C++ 版本：SDK 不建线程、不睡眠 ——
     * 真机上把 `std::this_thread::sleep_until` 加进 `step` 里即可。
     * 任一周期的 begin/end 返回非 OK 就提前停止并返回那个状态码。
     */
    template <class F> jsdk_status_t run(unsigned n, F &&step)
    {
        for (unsigned k = 0u; k < n; ++k) {
            jsdk_status_t st = cycle_begin((uint64_t)k * period_ns());
            if (st != JSDK_OK) return st;
            step(k);
            st = cycle_end();
            if (st != JSDK_OK) return st;
        }
        return JSDK_OK;
    }

    /* --- 查询 --- */

    unsigned joint_count() const noexcept { return nj_; }
    Joint &joint(unsigned i) { return joints_[i]; }
    const Joint &joint(unsigned i) const { return joints_[i]; }

    jsdk_bus_state_t bus_state() const
    {
        jsdk_bus_state_t bs;
        std::memset(&bs, 0, sizeof bs);
        check(jsdk_context_get_bus_state(ctx(), &bs), "get_bus_state", ctx());
        return bs;
    }

    jsdk_desc_info_t desc_info() const
    {
        jsdk_desc_info_t info;
        std::memset(&info, 0, sizeof info);
        check(jsdk_context_get_desc_info(ctx(), &info), "get_desc_info", ctx());
        return info;
    }

    /** 枚举已保留端点（`access` 见 `JSDK_ACCESS_*`）。 */
    std::vector<Endpoint> endpoints()
    {
        std::vector<Endpoint> out;
        jsdk_status_t st = jsdk_endpoint_enumerate(ctx(), &Group::visit_, &out);
        check(st, "endpoint_enumerate", ctx());
        return out;
    }

    /** 描述符原始字节的流出回调（路线 B 的 Flash 缓存用），透传给 C API。 */
    void set_desc_raw_sink(jsdk_desc_raw_sink_fn fn, void *user) noexcept
    {
        jsdk_context_set_desc_raw_sink(ctx(), fn, user);
    }

    size_t arena_size() const noexcept { return arena_.size(); }
    size_t arena_used() const noexcept { return cfg_.desc.arena_used; }

    jsdk_context_t *raw() const noexcept { return ctx(); }
    const char *last_error() const noexcept { return jsdk_context_last_error(ctx()); }

private:
    static int visit_(void *user, const char *path, uint16_t id,
                      jsdk_ep_type_t type, uint8_t access)
    {
        std::vector<Endpoint> *v = static_cast<std::vector<Endpoint> *>(user);
        Endpoint ep;
        ep.path   = path ? path : "";
        ep.id     = id;
        ep.type   = type;
        ep.access = access;
        v->push_back(ep);
        return 0;                 /* 返回非 0 = 停止遍历 */
    }

    static jsdk_context_config_t default_config()
    {
        jsdk_context_config_t cfg;
        jsdk_context_config_default(&cfg);
        cfg.is_fd           = 1u;
        cfg.period_ns       = 10000000u;   /* 100 Hz */
        cfg.desc.mode       = JSDK_DESC_DYNAMIC;
        cfg.desc.retain     = JSDK_DESC_RETAIN_ALL;
        cfg.desc.timeout_ms = 5000u;
        return cfg;
    }

    /* `storage_` 是裸字节缓冲，C API 要的是非 const 指针 ——
       这里 const_cast 是安全的：写回的是 `arena_used`（描述符解析的输出），
       并不改变逻辑上的"本对象是否可变"。 */
    jsdk_context_t *ctx() const noexcept
    {
        return reinterpret_cast<jsdk_context_t *>(
            const_cast<jsdk_context_storage_t *>(&storage_));
    }

    uint32_t period_ns() const noexcept { return cfg_.period_ns; }

    void init(const jsdk_can_hal_t &hal, const jsdk_context_config_t &cfg)
    {
        cfg_     = cfg;
        cfg_.hal = hal;                       /* C API 会复制 vtable，但 cfg 本身要活到结束 */

        /* arena 自管：大小按描述符配置算，**之后再也不能 resize**（ctx 指进去过） */
        arena_.assign(jsdk_desc_arena_size(&cfg_.desc), 0u);
        cfg_.desc.arena      = arena_.empty() ? nullptr : arena_.data();
        cfg_.desc.arena_size = arena_.size();

        check(jsdk_context_init(ctx(), &cfg_), "jsdk_context_init");
        inited_ = true;
    }

    jsdk_context_config_t  cfg_{};
    jsdk_context_storage_t storage_{};
    std::vector<uint8_t>   arena_;
    Joint                  joints_[JSDK_MAX_JOINTS_STATIC];
    unsigned               nj_ = 0u;
    bool                   inited_ = false;
};

inline jsdk_context_t *Joint::ctx_() const noexcept
{
    return owner_ ? owner_->raw() : nullptr;
}

}  /* namespace jsdk */

#endif /* JSDK_JOINT_GROUP_HPP */
