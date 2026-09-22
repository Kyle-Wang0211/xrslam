#include <chrono>
#include <iostream>
#include "../utility/pw_trace.h"
#include <xrslam/common.h>
#include <xrslam/core/detail.h>
#include <xrslam/core/feature_tracker.h>
#include <xrslam/core/frontend_worker.h>
#include <xrslam/core/initializer.h>
#include <xrslam/core/sliding_window_tracker.h>
#include <xrslam/geometry/stereo.h>
#include <xrslam/localizer/localizer.h>
#include <xrslam/map/frame.h>
#include <xrslam/map/map.h>
#include <xrslam/map/track.h>

// [PW 2026-09-16] 后端记账。XRSLAM 自带的计时通道只有 feature_tracker_time
// (feature_tracker.cpp:25),包住整个前端;后端一处都没有。而 1920×1440 的整帧
// 预算 51.67 ms 里前端只占 20.22 ms(39%),剩下 61% 究竟是后端滑窗优化还是台架
// 回放 I/O,没有这组计数就分不开——而那正是决定下一刀砍哪里的数。
// 主机侧计时:这几段都是粗粒度调用(每帧一次),不是常量偏移紧循环,所以不适用
// 「加法式探针会被 CSE 吃掉或被去优化」那条教训。
extern "C" {
double pw_bk_work_ms = 0.0;
double pw_bk_init_ms = 0.0;
double pw_bk_mirror_ms = 0.0;
double pw_bk_track_ms = 0.0;
unsigned long long pw_bk_work_n = 0;
unsigned long long pw_bk_init_n = 0;
unsigned long long pw_bk_track_n = 0;
}

namespace {
inline double pw_now_ms() {
    return std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}
// RAII 而不是在函数尾部加一行:work() 里有多条分支,只有作用域退出才保证每个
// 出口都记上。
struct PwScope {
    double t0;
    double *acc;
    unsigned long long *n;
    explicit PwScope(double *a, unsigned long long *c = nullptr)
        : t0(pw_now_ms()), acc(a), n(c) {}
    ~PwScope() {
        *acc += pw_now_ms() - t0;
        if (n) ++*n;
    }
};
} // namespace

namespace xrslam {

FrontendWorker::FrontendWorker(XRSLAM::Detail *detail,
                               std::shared_ptr<Config> config)
    : detail(detail), config(config) {
    initializer = std::make_unique<Initializer>(config);

    latest_state = {{}, nil(), {}, {}};
}

FrontendWorker::~FrontendWorker() = default;

bool FrontendWorker::empty() const { return pending_frame_ids.empty(); }

void FrontendWorker::work(std::unique_lock<std::mutex> &l) {
    PW_ZONE("backend.work");
    PwScope pw_work(&pw_bk_work_ms, &pw_bk_work_n);
    if (initializer) {
        PW_ZONE("backend.initializer");
        PwScope pw_init(&pw_bk_init_ms, &pw_bk_init_n);
        size_t pending_frame_id = pending_frame_ids.front();
        pending_frame_ids.clear();
        notify_space();
        l.unlock();
        synchronized(detail->feature_tracker->map) {
            initializer->mirror_keyframe_map(detail->feature_tracker->map.get(),
                                             pending_frame_id);
        }
        if ((sliding_window_tracker = initializer->initialize())) {
#if defined(XRSLAM_IOS)
            synchronized(detail->feature_tracker->keymap) {
                detail->feature_tracker->synchronize_keymap(
                    sliding_window_tracker->map.get());
            }
#endif
            if (config->visual_localization_enable() &&
                global_localization_state()) {
                localizer = std::make_unique<Localizer>(config);
                sliding_window_tracker->map->create_virtual_object_manager(localizer.get());
            } else {
                sliding_window_tracker->map->create_virtual_object_manager();
            }
            sliding_window_tracker->feature_tracking_map = detail->feature_tracker->map;
            sliding_window_tracker->set_detail(detail);
            std::unique_lock lk(latest_state_mutex);
            auto [t, pose, motion] = sliding_window_tracker->get_latest_state();
            latest_state = {t, pending_frame_id, pose, motion};
            lk.unlock();
            initializer.reset();
        }
    } else if (sliding_window_tracker) {
        size_t pending_frame_id = pending_frame_ids.front();
        pending_frame_ids.pop_front();
        pending_count_.store(pending_frame_ids.size(), std::memory_order_relaxed);
        notify_space();
        l.unlock();
        {
            PW_ZONE("backend.mirror_frame");
            PwScope pw_mirror(&pw_bk_mirror_ms);
            synchronized(detail->feature_tracker->map) {
                sliding_window_tracker->mirror_frame(
                    detail->feature_tracker->map.get(), pending_frame_id);
            }
        }
        bool pw_tracked;
        {
            PW_ZONE("backend.sliding_window_track");
            PwScope pw_track(&pw_bk_track_ms, &pw_bk_track_n);
            pw_tracked = sliding_window_tracker->track();
        }
        if (pw_tracked) {
#if defined(XRSLAM_IOS)
            synchronized(detail->feature_tracker->keymap) {
                detail->feature_tracker->synchronize_keymap(
                    sliding_window_tracker->map.get());
            }
#endif
            std::unique_lock lk(latest_state_mutex);
            auto [t, pose, motion] = sliding_window_tracker->get_latest_state();
            latest_state = {t, pending_frame_id, pose, motion};
            lk.unlock();
        } else {
            std::unique_lock lk(latest_state_mutex);
            latest_state = {{}, nil(), {}, {}};
            lk.unlock();
            initializer = std::make_unique<Initializer>(config);
            sliding_window_tracker.reset();
        }
    }
}

void FrontendWorker::issue_frame(Frame *frame) {
    auto l = lock();
    // 调用点 feature_tracker.cpp:153 在 synchronized(map) 块之外且 l 已解开,
    // 不持任何锁,可安全阻塞。
    await_capacity(l);
    pending_frame_ids.push_back(frame->id());
    pending_count_.store(pending_frame_ids.size(), std::memory_order_relaxed);
    resume(l);
}

std::tuple<double, size_t, PoseState, MotionState>
FrontendWorker::get_latest_state() const {
    std::unique_lock lk(latest_state_mutex);
    return latest_state;
}

size_t FrontendWorker::create_virtual_object() {
    auto l = lock();
    if (sliding_window_tracker) {
        return sliding_window_tracker->map->create_virtual_object();
    } else {
        return nil();
    }
    l.unlock();
}

OutputObject FrontendWorker::get_virtual_object_pose_by_id(size_t id) {
    auto l = lock();
    if (sliding_window_tracker) {
        return sliding_window_tracker->map->get_virtual_object_pose_by_id(id);
    } else {
        return {{0.0, 0.0, 0.0, 1.0}, {1000.0, 1000.0, 1000.0}, 1};
    }
    l.unlock();
}

SysState FrontendWorker::get_system_state() const {
    if (initializer) {
        return SysState::SYS_INITIALIZING;
    } else if (sliding_window_tracker) {
        return SysState::SYS_TRACKING;
    }
    return SysState::SYS_UNKNOWN;
}

void FrontendWorker::query_frame() {
    if (localizer)
        localizer->query_frame();
}

bool FrontendWorker::global_localization_state() const {
    return global_localization_flag;
}

void FrontendWorker::set_global_localization_state(bool state) {
    global_localization_flag = state;
}

} // namespace xrslam
