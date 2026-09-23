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

    // [pw 2026-09-23] Tracking-loss recovery (sliding_window_tracker.h, "reference image").
    // work() releases every frame's pixel buffers as soon as it has tracked the next frame
    // (see the release_image_buffer() call in feature_tracker.cpp). The recovery search needs
    // the pixels of the last well-tracked frame for up to lost_timeout seconds, so the backend
    // may exempt exactly one image from that release. Both calls must be made while holding
    // this->map's lock (the same lock work() releases under); nothing else is needed for
    // thread safety. Unused (always null) unless tracking_recovery_enable() is on, in which
    // case work() behaves exactly as before.
    void pw_pin_reference_image(std::shared_ptr<Image> image);
    void pw_unpin_reference_image();
    // Drops the pose this tracker hands to Detail::predict_pose, as the existing
    // "SWT cannot catch up" path does; used when recovery gives up and the map is reset, so
    // that no dead-reckoned pose is reported while the system re-initialises.
    void pw_reset_latest_state();

  private:
    std::shared_ptr<Image> pw_pinned_image;
    XRSLAM::Detail *detail;
    std::deque<std::unique_ptr<Frame>> frames;
    std::atomic<size_t> pending_count_{0};
    std::shared_ptr<Config> config;
    std::optional<std::tuple<double, PoseState, MotionState>> latest_state;
    mutable std::mutex latest_pose_mutex;
};

} // namespace xrslam

#endif // XRSLAM_FEATURE_TRACKER_H
