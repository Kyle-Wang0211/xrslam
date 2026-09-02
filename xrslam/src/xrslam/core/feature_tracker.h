#ifndef XRSLAM_FEATURE_TRACKER_H
#define XRSLAM_FEATURE_TRACKER_H

#include <atomic>
#include <xrslam/common.h>
#include <xrslam/estimation/state.h>
#include <xrslam/utility/worker.h>

namespace xrslam {

class FeatureTracker : public Worker {
  public:
    FeatureTracker(XRSLAM::Detail *detail, std::shared_ptr<Config> config);
    ~FeatureTracker();

    bool empty() const override { return frames.empty(); }

    // [pw 2026-09-02] 入口队列上界。每个元素是一整帧图像(1920x1440 灰度约
    // 2.7MB),这里是 threading 开启后无界增长的主体。2 = 一帧在处理、一帧在等,
    // 既保住异步重叠又把驻留封顶。可用 -DXRSLAM_FEATURE_TRACKER_QUEUE_CAPACITY 覆盖。
#ifndef XRSLAM_FEATURE_TRACKER_QUEUE_CAPACITY
#define XRSLAM_FEATURE_TRACKER_QUEUE_CAPACITY 2
#endif
    size_t capacity() const override {
        return XRSLAM_FEATURE_TRACKER_QUEUE_CAPACITY;
    }
    size_t pending_locked() const override { return frames.size(); }

    /// Bisect variant A: lock-free backlog size only.
    size_t pending_frame_count() const {
        return pending_count_.load(std::memory_order_relaxed);
    }

    void work(std::unique_lock<std::mutex> &l) override;

    void track_frame(std::unique_ptr<Frame> frame);

    void mirror_map(Map *sliding_window_tracker_map);
    void synchronize_keymap(Map *sliding_window_tracker_map);
    void mirror_keyframe(Map *sliding_window_tracker_map, size_t frame_id);
    void mirror_lastframe(Map *sliding_window_tracker_map);
    void attach_latest_frame(Frame *frame);
    void manage_keymap();
    void solve_pnp();

    std::optional<std::tuple<double, PoseState, MotionState>>
    get_latest_state() const;

    std::shared_ptr<Map> map;
    std::unique_ptr<Map> keymap;

  private:
    XRSLAM::Detail *detail;
    std::deque<std::unique_ptr<Frame>> frames;
    std::atomic<size_t> pending_count_{0};
    std::shared_ptr<Config> config;
    std::optional<std::tuple<double, PoseState, MotionState>> latest_state;
    mutable std::mutex latest_pose_mutex;
};

} // namespace xrslam

#endif // XRSLAM_FEATURE_TRACKER_H
