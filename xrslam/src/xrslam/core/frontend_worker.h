#ifndef XRSLAM_FRONTEND_WORKER_H
#define XRSLAM_FRONTEND_WORKER_H
#include <atomic>
#include <xrslam/common.h>
#include <xrslam/estimation/state.h>
#include <xrslam/utility/worker.h>

namespace xrslam {

class Config;
class Frame;
class Initializer;
class SlidingWindowTracker;
class Localizer;

class FrontendWorker : public Worker {
  public:
    FrontendWorker(XRSLAM::Detail *detail, std::shared_ptr<Config> config);
    ~FrontendWorker();

    bool empty() const override;

    // [pw 2026-09-02] 元素只是 size_t,但每个 id 都钉住 map 里一帧不让释放,
    // 所以同样要封顶。可用 -DXRSLAM_FRONTEND_QUEUE_CAPACITY 覆盖。
#ifndef XRSLAM_FRONTEND_QUEUE_CAPACITY
#define XRSLAM_FRONTEND_QUEUE_CAPACITY 4
#endif
    size_t capacity() const override { return XRSLAM_FRONTEND_QUEUE_CAPACITY; }
    size_t pending_locked() const override { return pending_frame_ids.size(); }

    /// Bisect variant A.
    size_t pending_frame_count() const {
        return pending_count_.load(std::memory_order_relaxed);
    }
    void work(std::unique_lock<std::mutex> &l) override;

    void issue_frame(Frame *frame);

    size_t create_virtual_object();
    OutputObject get_virtual_object_pose_by_id(size_t id);

    std::tuple<double, size_t, PoseState, MotionState> get_latest_state() const;
    SysState get_system_state() const;

    bool global_localization_state() const;
    void set_global_localization_state(bool state);
    void query_frame();

    std::unique_ptr<Localizer> localizer;

  private:
    std::deque<size_t> pending_frame_ids;
    std::atomic<size_t> pending_count_{0};

    XRSLAM::Detail *detail;
    std::shared_ptr<Config> config;
    std::unique_ptr<Initializer> initializer;
    std::unique_ptr<SlidingWindowTracker> sliding_window_tracker;

    std::tuple<double, size_t, PoseState, MotionState> latest_state;
    mutable std::mutex latest_state_mutex;

    bool global_localization_flag = false;
};

} // namespace xrslam

#endif // XRSLAM_FRONTEND_WORKER_H
