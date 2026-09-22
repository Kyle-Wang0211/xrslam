#include <xrslam/common.h>
#include <xrslam/core/detail.h>
#include <xrslam/core/feature_tracker.h>
#include <xrslam/core/frontend_worker.h>
#include <xrslam/estimation/solver.h>
#include <xrslam/geometry/stereo.h>
#include <xrslam/inspection.h>
#if XRSLAM_ENABLE_VISUAL_LOCALIZATION
#include <xrslam/localizer/localizer.h>
#endif
#include <xrslam/map/frame.h>
#include <xrslam/map/map.h>
#include <xrslam/map/track.h>
#include <xrslam/utility/runtime_budget.h>
#include <xrslam/utility/unique_timer.h>
namespace xrslam {

FeatureTracker::FeatureTracker(XRSLAM::Detail *detail,
                               std::shared_ptr<Config> config)
    : detail(detail), config(config) {
    map = std::make_unique<Map>();
    keymap = std::make_unique<Map>();
    cap_pending_frames_ = config->runtime_max_pending_camera_frames();
    cap_tracking_map_frames_ = config->runtime_max_tracking_map_frames();
    // [pw] 条目 17:这条上限是**兜底**,不许悄悄取代既定策略。
    //   下面那个 while 用的是 max_frames / max_init_frames(基类默认 200 / 60)。
    //   如果兜底值比它们还小,跟踪图就会被兜底值而不是被设计好的滑窗长度决定 ——
    //   那是一次静默的行为改变,正是「fail-safe 只许推迟不许丢数据」要挡的东西。
    //   所以这里把它夹到工作集之上,并且**说出来**,而不是默默照做。
    const size_t working_set =
        std::max(config->feature_tracker_max_frames(),
                 config->feature_tracker_max_init_frames());
    if (cap_tracking_map_frames_ > 0 &&
        cap_tracking_map_frames_ <= working_set) {
        log_warning("[pw][budget] runtime.max_tracking_map_frames=%zu 不大于"
                    "跟踪图的正常工作集 %zu(max_frames / max_init_frames),"
                    "已上调为 %zu。兜底上限不得取代既定滑窗策略。",
                    cap_tracking_map_frames_, working_set, working_set + 8);
        cap_tracking_map_frames_ = working_set + 8;
    }
}

FeatureTracker::~FeatureTracker() = default;

void FeatureTracker::work(std::unique_lock<std::mutex> &l) {
    auto ft_timer = make_timer([](double t) {
        inspect(feature_tracker_time, time) {
            static double avg_time = 0;
            static double avg_count = 0;
            avg_time = (avg_time * avg_count + t) / (avg_count + 1);
            avg_count += 1.0;
            time = avg_time;
        }
    });

    std::unique_ptr<Frame> frame = std::move(frames.front());
    frames.pop_front();
    {
        auto &c = runtime::counters();
        runtime::observe_depth(c.depth_tracker_frame_queue,
                               c.hw_tracker_frame_queue, frames.size());
    }
    // [pw] 任务 3:**必须在这里**取真实 IMU 样本数。再往下 40 行有一段
    //      `frame->preintegration.data.insert(begin(), imu)` 的补桩逻辑:当本帧
    //      一个真实样本都没有时,它会塞进一个"上一帧最后一个样本 + 改时间戳"的
    //      合成样本 ⇒ 之后再数就永远数不到 0,「静默退化成纯单目」这个信号会被
    //      补桩自己抹掉。raw = 补桩前的真实观测数。
    const size_t n_imu_raw = frame->preintegration.data.size();
    l.unlock();

    frame->image->preprocess(config->feature_tracker_clahe_clip_limit(),
                             config->feature_tracker_clahe_width(),
                             config->feature_tracker_clahe_height());

    auto [latest_optimized_time, latest_optimized_frame_id,
          latest_optimized_pose, latest_optimized_motion] =
        detail->frontend->get_latest_state();
    bool is_initialized = latest_optimized_frame_id != nil();
    bool slidind_window_frame_tag =
        !is_initialized ||
        frame->id() % config->sliding_window_tracker_frequent() == 0;
    synchronized(map) {
        if (map->frame_num() > 0) {
            if (is_initialized) {
                size_t latest_optimized_frame_index =
                    map->frame_index_by_id(latest_optimized_frame_id);
                if (latest_optimized_frame_index != nil()) {
                    Frame *latest_optimized_frame =
                        map->get_frame(latest_optimized_frame_index);
                    latest_optimized_frame->pose = latest_optimized_pose;
                    latest_optimized_frame->motion = latest_optimized_motion;
                    for (size_t j = latest_optimized_frame_index + 1;
                         j < map->frame_num(); ++j) {
                        Frame *frame_i = map->get_frame(j - 1);
                        Frame *frame_j = map->get_frame(j);
                        frame_j->preintegration.integrate(
                            frame_j->image->t, frame_i->motion.bg,
                            frame_i->motion.ba, false, false);
                        frame_j->preintegration.predict(frame_i, frame_j);
                    }
                } else {
                    // TODO: unfortunately the frame has slided out, which means
                    // we are lost...
                    log_warning("SWT cannot catch up.");
                    std::unique_lock lk(latest_pose_mutex);
                    latest_state.reset();
                }
            }
            Frame *last_frame = map->get_frame(map->frame_num() - 1);
            if (!last_frame->preintegration.data.empty()) {
                if (frame->preintegration.data.empty() ||
                    (frame->preintegration.data.front().t -
                         last_frame->image->t >
                     1.0e-5)) {
                    ImuData imu = last_frame->preintegration.data.back();
                    imu.t = last_frame->image->t;
                    frame->preintegration.data.insert(
                        frame->preintegration.data.begin(), imu);
                }
            }
            frame->preintegration.integrate(
                frame->image->t, last_frame->motion.bg, last_frame->motion.ba,
                false, false);
            last_frame->track_keypoints(frame.get(), config.get());
            if (is_initialized) {
                frame->preintegration.predict(last_frame, frame.get());
// [pw] 原为 #if defined(XRSLAM_IOS)。这条是「每帧 keymap PnP + 更新 latest_state」的
//      低延迟位姿链,是产品行为选择,不是平台判定。以前只有 iOS 编得进来,Android 走
//      下面的 #else 分支 ⇒ 两端位姿输出率与精度来源完全不同。改挂 XRSLAM_LOWLATENCY_POSE。
#if defined(XRSLAM_LOWLATENCY_POSE)
                synchronized(keymap) {
                    attach_latest_frame(frame.get());
                    solve_pnp();
                    Frame *latest_frame =
                        keymap->get_frame(keymap->frame_num() - 1);
                    std::unique_lock lk(latest_pose_mutex);
                    latest_state = {latest_frame->image->t, latest_frame->pose,
                                    latest_frame->motion};
                    lk.unlock();
                    keymap->erase_frame(keymap->frame_num() - 1);
#if XRSLAM_ENABLE_VISUAL_LOCALIZATION
                    if (config->visual_localization_enable() &&
                        detail->frontend->global_localization_state()) {
                        detail->frontend->localizer->query_localization(
                            latest_frame->image, latest_frame->pose);
                        // detail->frontend->localizer->send_pose_message(frame->image->t);
                    }
#endif
                }
#else
                std::unique_lock lk(latest_pose_mutex);
                latest_state = {frame->image->t, frame->pose, frame->motion};
#if XRSLAM_ENABLE_VISUAL_LOCALIZATION
                if (config->visual_localization_enable() &&
                    detail->frontend->global_localization_state()) {
                    detail->frontend->localizer->query_localization(
                        frame->image, frame->pose);
                    // detail->frontend->localizer->send_pose_message(frame->image->t);
                }
#endif
                lk.unlock();
#endif
            }
            last_frame->image->release_image_buffer();
        }

        if (slidind_window_frame_tag)
            frame->detect_keypoints(config.get());
        map->attach_frame(std::move(frame));

        {
            // [pw] 任务 3:发布「这一帧到底有没有用上 IMU」。
            //      imu_samples == 0 就是静默退化成纯单目的信号 —— 这一帧的
            //      预积分里一个惯性样本都没有。
            Frame *attached = map->get_frame(map->frame_num() - 1);
            auto &h = runtime::frame_health_atomics();
            const int n_imu_integrated =
                (int)attached->preintegration.data.size();
            h.frame_t.store(attached->image->t, std::memory_order_relaxed);
            h.imu_samples.store((int)n_imu_raw, std::memory_order_relaxed);
            h.imu_samples_integrated.store(n_imu_integrated,
                                           std::memory_order_relaxed);
            h.detected.store((int)attached->keypoint_num(),
                             std::memory_order_relaxed);
            if (n_imu_raw == 0) {
                unsigned long long starved = h.imu_starved_frames.fetch_add(
                                                 1, std::memory_order_relaxed) +
                                             1;
                if (runtime::should_log_at(starved)) {
                    log_warning("[pw][health] 本帧(t=%.6f)没有任何真实 IMU "
                                "样本参与预积分(积分用样本数=%d,其中 %d 个是"
                                "补桩合成的)⇒ 已静默退化成纯单目,累计 %llu 帧",
                                attached->image->t, n_imu_integrated,
                                n_imu_integrated, starved);
                }
            }
            h.seq.fetch_add(1, std::memory_order_relaxed);
        }

        size_t max_frame_num = is_initialized? config->feature_tracker_max_frames(): config->feature_tracker_max_init_frames();
        while (map->frame_num() > max_frame_num && map->get_frame(0)->id() < latest_optimized_frame_id) {
            map->erase_frame(0);
        }

        // [pw] 条目 17:上面那个 while 的第二个条件依赖后端在推进
        //      latest_optimized_frame_id。后端一旦卡住(热降频)或丢跟踪,
        //      这张跟踪图就**一帧都不裁**了 —— 每帧带着关键点和 track,
        //      是 native 匿名内存里增长最快的一块。这里加一条不依赖后端的
        //      绝对上限,策略同样是丢最老。裁掉的只是跟踪器的内部工作集,
        //      交付数据(采集到的图像本身)不在这条链上。
        if (cap_tracking_map_frames_ > 0) {
            size_t dropped = 0;
            while (map->frame_num() > cap_tracking_map_frames_) {
                map->erase_frame(0);
                ++dropped;
            }
            if (dropped > 0) {
                auto &c = runtime::counters();
                unsigned long long total =
                    c.dropped_tracking_map_frames.fetch_add(
                        dropped, std::memory_order_relaxed) +
                    dropped;
                if (runtime::should_log_at(total)) {
                    log_warning("[pw][budget] 跟踪图超过绝对上限 %zu 帧"
                                "(后端未推进 latest_optimized_frame_id),"
                                "丢弃最老 %zu 帧,累计 %llu",
                                cap_tracking_map_frames_, dropped, total);
                }
            }
        }
        {
            auto &c = runtime::counters();
            runtime::observe_depth(c.depth_tracking_map_frames,
                                   c.hw_tracking_map_frames, map->frame_num());
            c.depth_tracking_map_tracks.store(map->track_num(),
                                              std::memory_order_relaxed);
        }

        inspect_debug(feature_tracker_painter, p) {
            if (p.has_value()) {
                auto painter = std::any_cast<InspectPainter *>(p);
                auto frame = map->get_frame(map->frame_num() - 1);
                painter->set_image(frame->image.get());
                for (size_t i = 0; i < frame->keypoint_num(); ++i) {
                    if (Track *track = frame->get_track(i)) {
                        color3b c = {255, 255, 0};
                        painter->point(apply_k(frame->get_keypoint(i), frame->K).cast<int>(),c, 2, 1);
                    }
                }
            }
        }
    }
    if (slidind_window_frame_tag)
        detail->frontend->issue_frame(map->get_frame(map->frame_num() - 1));
}

void FeatureTracker::track_frame(std::unique_ptr<Frame> frame) {
    auto l = lock();
    frames.emplace_back(std::move(frame));
    {
        // [pw] 条目 17:跟踪器的输入队列。热降频时跟踪线程跟不上就会在这里堆
        //      整张图 —— 与 detail.frames 同一类交付敏感对象,所以复用同一个
        //      上限旋钮,默认 0(不设限,行为与改动前一致),只观测 + 告警。
        auto &c = runtime::counters();
        runtime::drop_oldest_over(frames, cap_pending_frames_,
                                  c.dropped_tracker_frame_queue,
                                  "feature_tracker.frames(跟踪输入队列)");
        runtime::observe_depth(c.depth_tracker_frame_queue,
                               c.hw_tracker_frame_queue, frames.size());
    }
    resume(l);
}

std::optional<std::tuple<double, PoseState, MotionState>>
FeatureTracker::get_latest_state() const {
    std::unique_lock lk(latest_pose_mutex);
    return latest_state;
}

void FeatureTracker::synchronize_keymap(Map *sliding_window_tracker_map) {

    // clean keymap
    while (keymap->frame_num()) {
        keymap->erase_frame(0);
    }

    // mirror the latest SWT map to keymap
    mirror_map(sliding_window_tracker_map);

    // add last frame (include subframe) in swt map to keymap for track
    // associating
    mirror_lastframe(sliding_window_tracker_map);
}

void FeatureTracker::mirror_map(Map *sliding_window_tracker_map) {

    for (size_t index = 0; index < sliding_window_tracker_map->frame_num();
         ++index) {
        keymap->attach_frame(
            sliding_window_tracker_map->get_frame(index)->clone());
    }

    for (size_t j = 1; j < keymap->frame_num(); ++j) {
        Frame *old_frame_i = sliding_window_tracker_map->get_frame(j - 1);
        Frame *old_frame_j = sliding_window_tracker_map->get_frame(j);
        Frame *new_frame_i = keymap->get_frame(j - 1);
        Frame *new_frame_j = keymap->get_frame(j);
        for (size_t ki = 0; ki < old_frame_i->keypoint_num(); ++ki) {
            if (Track *track = old_frame_i->get_track(ki)) {
                if (size_t kj = track->get_keypoint_index(old_frame_j);
                    kj != nil()) {
                    Track *new_track = new_frame_i->get_track(ki, keymap.get());
                    new_track->add_keypoint(new_frame_j, kj);
                    new_track->landmark = track->landmark;
                    new_track->tag(TT_VALID) = track->tag(TT_VALID);
                    new_track->tag(TT_TRIANGULATED) =
                        track->tag(TT_TRIANGULATED);
                    new_track->tag(TT_FIX_INVD) = true;
                }
            }
        }
    }

    for (size_t index = 0; index < keymap->frame_num(); ++index) {
        Frame *keyframe = keymap->get_frame(index);
        keyframe->tag(FT_KEYFRAME) = true;
        keyframe->tag(FT_FIX_POSE) = true;
        keyframe->tag(FT_FIX_MOTION) = true;
    }
}

void FeatureTracker::mirror_lastframe(Map *sliding_window_tracker_map) {

    Frame *last_keyframe_i = keymap->get_frame(keymap->frame_num() - 1);
    Frame *last_keyframe_j = sliding_window_tracker_map->get_frame(
        sliding_window_tracker_map->frame_num() - 1);

    if (last_keyframe_j->subframes.empty()) // no subframes means this keyframe
                                            // has been existed in FT map
        return;

    Frame *last_subframe = last_keyframe_j->subframes.back().get();

    keymap->attach_frame(last_subframe->clone());

    Frame *new_keyframe = keymap->get_frame(keymap->frame_num() - 1);

    for (size_t ki = 0; ki < last_keyframe_j->keypoint_num(); ++ki) {
        if (Track *track = last_keyframe_j->get_track(ki)) {
            if (size_t kj = track->get_keypoint_index(last_subframe);
                kj != nil()) {
                last_keyframe_i->get_track(ki, keymap.get())
                    ->add_keypoint(new_keyframe, kj);
            }
        }
    }

    new_keyframe->tag(FT_KEYFRAME) = false;
    new_keyframe->tag(FT_FIX_POSE) = false;
    new_keyframe->tag(FT_FIX_MOTION) = false;
}

void FeatureTracker::attach_latest_frame(Frame *frame) {

    Frame *new_last_frame_i = keymap->get_frame(keymap->frame_num() - 1);
    size_t last_frame_index = map->frame_index_by_id(new_last_frame_i->id());

    size_t frame_index_i = map->frame_index_by_id(
        keymap->get_frame(keymap->frame_num() - 1)->id());
    size_t frame_index_j = map->frame_num() - 1;

    keymap->attach_frame(frame->clone());
    Frame *new_last_frame_j = keymap->get_frame(keymap->frame_num() - 1);

    if (last_frame_index != nil()) {
        Frame *old_last_frame_i = map->get_frame(last_frame_index);
        Frame *old_last_frame_j = frame;
        for (size_t ki = 0; ki < old_last_frame_i->keypoint_num(); ++ki) {
            if (Track *track = old_last_frame_i->get_track(ki)) {
                if (size_t kj = track->get_keypoint_index(old_last_frame_j);
                    kj != nil()) {
                    Track *track =
                        new_last_frame_i->get_track(ki, keymap.get());
                    track->add_keypoint(new_last_frame_j, kj);
                }
            }
        }
        new_last_frame_j->tag(FT_KEYFRAME) = false;
        new_last_frame_j->tag(FT_FIX_POSE) = false;
    } else {
        std::cout << "error: cannot find last frame id in FT map" << std::endl;
    }
}

void FeatureTracker::solve_pnp() {

    Frame *latest_frame = keymap->get_frame(keymap->frame_num() - 1);

    auto solver = Solver::create();

    // Only the latest frame's pose is free; the mirrored keyframes and their
    // landmarks are fixed (see mirror_map). This is a pure PnP, so no motion state.
    solver->add_frame_states(latest_frame, false);

    size_t factor_count = 0;
    for (size_t j = 0; j < latest_frame->keypoint_num(); ++j) {
        if (Track *track = latest_frame->get_track(j)) {
            if (track->all_tagged(TT_VALID, TT_TRIANGULATED)) {
                solver->put_factor(Solver::create_reprojection_prior_factor(
                    latest_frame, track));
                ++factor_count;
            }
        }
    }

    // Visually refine the live output pose against the (LiDAR-depth-anchored) map
    // landmarks every frame. Without this solve the per-frame pose is pure IMU
    // dead-reckoning forward from the last backend keyframe; when the device is
    // still the backend stops emitting keyframes, so that dead-reckoning leg grows
    // and its noise shows up as the tracked features floating. Known landmark depth
    // makes translation observable even at zero parallax, so the PnP pins it.
    // Skip when too few constraints, where a degenerate PnP would be worse than the
    // IMU prediction we already have.
    if (factor_count >= 6) {
        solver->solve();
    }
}

// ---------------------------------------------------------------------------
// [pw] 任务 3:配合 C API 层健康状态机的只读出口。核内定义,消费方前向声明。
// ---------------------------------------------------------------------------
void get_frame_health(FrameHealth &out) {
    auto &h = runtime::frame_health_atomics();
    const std::memory_order r = std::memory_order_relaxed;
    out.frame_seq = h.seq.load(r);
    out.frame_t = h.frame_t.load(r);
    out.imu_samples = h.imu_samples.load(r);
    out.imu_samples_integrated = h.imu_samples_integrated.load(r);
    out.detected_keypoints = h.detected.load(r);
    out.tracked_keypoints = h.tracked.load(r);
    out.inlier_keypoints = h.inliers.load(r);
    out.mapped_landmarks = h.mapped_landmarks.load(r);
    out.imu_starved_frames = h.imu_starved_frames.load(r);
}

void get_frame_health_counters(int &imu_samples, int &tracked_keypoints,
                               int &inlier_keypoints,
                               int &detected_keypoints) {
    auto &h = runtime::frame_health_atomics();
    const std::memory_order r = std::memory_order_relaxed;
    imu_samples = h.imu_samples.load(r);
    tracked_keypoints = h.tracked.load(r);
    inlier_keypoints = h.inliers.load(r);
    detected_keypoints = h.detected.load(r);
}

} // namespace xrslam
