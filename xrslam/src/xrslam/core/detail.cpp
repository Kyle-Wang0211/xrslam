#include <xrslam/common.h>
#include <xrslam/core/detail.h>
#include <xrslam/core/feature_tracker.h>
#include <xrslam/core/frontend_worker.h>
#include <xrslam/estimation/solver.h>
#include <xrslam/geometry/lie_algebra.h>
#include <xrslam/geometry/stereo.h>
#include <xrslam/inspection.h>
#if XRSLAM_ENABLE_VISUAL_LOCALIZATION
#include <xrslam/localizer/localizer.h>
#endif
#include <xrslam/map/frame.h>
#include <xrslam/map/map.h>

namespace xrslam {

static void propagate_state(double &state_time, PoseState &state_pose,
                            MotionState &state_motion, double t,
                            const vector<3> &w, const vector<3> &a) {
    static const vector<3> gravity = {0, 0, -XRSLAM_GRAVITY_NOMINAL};
    double dt = t - state_time;
    state_pose.p =
        state_pose.p + dt * state_motion.v +
        0.5 * dt * dt * (gravity + state_pose.q * (a - state_motion.ba));
    state_motion.v =
        state_motion.v + dt * (gravity + state_pose.q * (a - state_motion.ba));
    state_pose.q =
        (state_pose.q * expmap((w - state_motion.bg) * dt)).normalized();
    state_time = t;
}

XRSLAM::Detail::Detail(std::shared_ptr<Config> config) : config(config) {
    Solver::init(config.get());

    // [pw] 条目 08:dt 一直是实测的(见 preintegrator.cpp 的 integrate()),
    //      这里装的是"实测值到底长什么样"的监视器 —— 尤其是批内等距伪造的检测。
    imu_timing_.configure(config->imu_timing_warmup_samples(),
                          config->imu_timing_window_samples(),
                          config->imu_timing_batch_gap_ratio());
    // [pw] 条目 17:缓存内部工作集的硬上限。
    cap_raw_imu_queue_ = config->runtime_max_raw_imu_queue();
    cap_pending_imu_ = config->runtime_max_pending_imu();
    cap_frontal_imu_ = config->runtime_max_frontal_imu();
    cap_pending_camera_frames_ = config->runtime_max_pending_camera_frames();

    frontend = std::make_unique<FrontendWorker>(this, config);
    feature_tracker = std::make_unique<FeatureTracker>(this, config);

    frontend->start();
    feature_tracker->start();
}

XRSLAM::Detail::~Detail() {
    feature_tracker->stop();
    frontend->stop();
}

const Config *XRSLAM::Detail::configurations() const { return config.get(); }

Pose XRSLAM::Detail::track_gyroscope(const double &t, const double &x,
                                     const double &y, const double &z) {
    if (accelerometers.size() > 0) {
        if (t < accelerometers.front().t) {
            // [pw] 任务 1.4 —— 这一行原来是**静默**整桶清空。
            //   属实的部分:陀螺与加速度成批交替到达时,新到的陀螺批整体早于
            //   已积压的加速度队列,于是每来一个陀螺样本就清一次桶,一批陀螺
            //   会被逐个清掉、只留最后一个。行为上确实是"成批静默清空"。
            //   不属实的部分:被清掉的样本严格早于本次新样本 t,而 t 又早于
            //   accelerometers.front().t,所以它们不可能是任何待插值加速度的
            //   下界 bracket(新样本是更紧的下界)。⇒ 不丢加速度、不丢 IMU 融合
            //   样本,是可观测性缺口,不是数据丢失。
            // 现在:计数 + 限流告警,不再静默。
            size_t discarded = gyroscopes.size();
            gyroscopes.clear();
            auto &c = runtime::counters();
            unsigned long long n =
                c.gyro_buffer_clears.fetch_add(1, std::memory_order_relaxed) +
                1;
            c.gyro_samples_dropped_by_clear.fetch_add(
                discarded, std::memory_order_relaxed);
            if (runtime::should_log_at(n)) {
                log_warning("[pw][imu] 陀螺缓冲被重同步清空(丢弃 %zu 个早于"
                            "待插值加速度队列的陀螺样本),累计 %llu 次。"
                            "持续出现 = 两路传感器成批交替到达。",
                            discarded, n);
            }
        } else {
            while (accelerometers.size() > 0 && t >= accelerometers.front().t) {
                const auto &acc = accelerometers.front();
                double lambda =
                    (acc.t - gyroscopes[0].t) / (t - gyroscopes[0].t);
                vector<3> w = gyroscopes[0].w +
                              lambda * (vector<3>{x, y, z} - gyroscopes[0].w);
                track_imu({acc.t, w, acc.a});
                accelerometers.pop_front();
            }
            if (accelerometers.size() > 0) {
                while (gyroscopes.size() > 0 && gyroscopes.front().t < t) {
                    gyroscopes.pop_front();
                }
            }
        }
    }
    gyroscopes.emplace_back(GyroscopeData{t, {x, y, z}});
    // [pw] 条目 17:原始陀螺队列的硬上限(丢最老)。稳态深度是个位数;
    //      只有两路彻底失同步时才会碰到,那时最老的样本已经没有配对对象了。
    auto &c = runtime::counters();
    runtime::drop_oldest_over(gyroscopes, cap_raw_imu_queue_, c.dropped_raw_gyro,
                              "gyroscopes");
    runtime::observe_depth(c.depth_raw_gyro, c.hw_raw_gyro, gyroscopes.size());
    runtime::observe_depth(c.depth_raw_accel, c.hw_raw_accel,
                           accelerometers.size());
    return predict_pose(t);
}

Pose XRSLAM::Detail::track_accelerometer(const double &t, const double &x,
                                         const double &y, const double &z) {
    if (gyroscopes.size() > 0 && t >= gyroscopes.front().t) {
        if (t > gyroscopes.back().t) {
            while (gyroscopes.size() > 1) {
                gyroscopes.pop_front();
            }
            // t > gyroscopes[0].t
            accelerometers.emplace_back(AccelerometerData{t, {x, y, z}});
        } else if (t == gyroscopes.back().t) {
            while (gyroscopes.size() > 1) {
                gyroscopes.pop_front();
            }
            track_imu({t, gyroscopes.front().w, {x, y, z}});
        } else {
            // pre-condition: gyroscopes.front().t <= t < gyroscopes.back().t
            // ==>  gyroscopes.size() >= 2
            while (t >= gyroscopes[1].t) {
                gyroscopes.pop_front();
            }
            // post-condition: t < gyroscopes[1].t
            double lambda =
                (t - gyroscopes[0].t) / (gyroscopes[1].t - gyroscopes[0].t);
            vector<3> w =
                gyroscopes[0].w + lambda * (gyroscopes[1].w - gyroscopes[0].w);
            track_imu({t, w, {x, y, z}});
        }
    } else {
        // [pw] 任务 1.4 的**镜像缺口**(上游同样静默,而且这一条是真丢数据):
        //   条件为假 = 陀螺缓冲为空,或本加速度样本早于最老的陀螺样本。
        //   两种情况下该加速度样本既**不入 accelerometers 队列**(emplace_back
        //   只在 `t > gyroscopes.back().t` 分支里),也**不进 track_imu** ⇒
        //   被整个丢弃。开机头几个样本必然命中(陀螺还没来),属正常;
        //   持续增长 = 两路时间戳基准不同或陀螺路断流。
        auto &cc = runtime::counters();
        unsigned long long n = cc.accel_dropped_no_bracket.fetch_add(
                                   1, std::memory_order_relaxed) +
                               1;
        if (runtime::should_log_at(n)) {
            log_warning("[pw][imu] 加速度样本(t=%.6f)找不到可插值的陀螺下界,"
                        "被丢弃(陀螺缓冲 %zu 个),累计 %llu 个。"
                        "开机初期正常;持续增长 = 两路 IMU 失同步。",
                        t, gyroscopes.size(), n);
        }
    }
    // [pw] 条目 17:原始加速度队列的硬上限(丢最老)。
    auto &c = runtime::counters();
    runtime::drop_oldest_over(accelerometers, cap_raw_imu_queue_,
                              c.dropped_raw_accel, "accelerometers");
    runtime::observe_depth(c.depth_raw_accel, c.hw_raw_accel,
                           accelerometers.size());
    runtime::observe_depth(c.depth_raw_gyro, c.hw_raw_gyro, gyroscopes.size());
    return predict_pose(t);
}

Pose XRSLAM::Detail::track_camera(std::shared_ptr<Image> image) {
    std::unique_ptr<Frame> frame = std::make_unique<Frame>();
    frame->K = config->camera_intrinsic();
    frame->image = image;
    frame->depth = image ? image->depth : nullptr;
    frame->sqrt_inv_cov = frame->K.block<2, 2>(0, 0);
    frame->sqrt_inv_cov(0, 0) /= ::sqrt(config->keypoint_noise_cov()(0, 0));
    frame->sqrt_inv_cov(1, 1) /= ::sqrt(config->keypoint_noise_cov()(1, 1));
    frame->camera.q_cs = config->camera_to_body_rotation();
    frame->camera.p_cs = config->camera_to_body_translation();
    frame->imu.q_cs = config->imu_to_body_rotation();
    frame->imu.p_cs = config->imu_to_body_translation();
    frame->preintegration.cov_a = config->accelerometer_noise_cov();
    frame->preintegration.cov_w = config->gyroscope_noise_cov();
    frame->preintegration.cov_ba = config->accelerometer_bias_noise_cov();
    frame->preintegration.cov_bg = config->gyroscope_bias_noise_cov();

    frames.emplace_back(std::move(frame));
    {
        // [pw] 条目 17:待融合相机帧队列。只有 IMU 停了才会增长,而每个元素
        //      持有一整张图 ⇒ 是最快的 OOM 路径。但裁它会在 VIO 轨迹上留下
        //      永久空洞,违反「fail-safe 只许推迟不许丢数据」,所以默认
        //      cap = 0(不设限,行为与改动前逐字节一致),只观测 + 告警。
        auto &c = runtime::counters();
        runtime::drop_oldest_over(frames, cap_pending_camera_frames_,
                                  c.dropped_pending_camera_frames,
                                  "detail.frames(待融合相机帧)");
        runtime::observe_depth(c.depth_pending_camera_frames,
                               c.hw_pending_camera_frames, frames.size());
    }

    Pose outpose = predict_pose(image->t);
    std::unique_lock<std::mutex> lk(latest_mutex_);
    if (image->t > latest_timestamp_) {
        latest_pose_ = outpose;
        latest_timestamp_ = image->t;
    }
    return outpose;
}

void XRSLAM::Detail::track_imu(const ImuData &imu) {
    // [pw] 条目 08:所有被融合的 IMU 样本都从这里过一次 —— 唯一的漏斗。
    imu_timing_.add_sample(imu.t);

    frontal_imus.emplace_back(imu);
    imus.emplace_back(imu);
    while (imus.size() > 0 && frames.size() > 0) {
        if (imus.front().t <= frames.front()->image->t) {
            frames.front()->preintegration.data.push_back(imus.front());
            imus.pop_front();
        } else {
            feature_tracker->track_frame(std::move(frames.front()));
            frames.pop_front();
        }
    }

    // [pw] 条目 17:两条只在**退化状态**下才增长的队列,必须有硬上限。
    //   imus         —— 只在 frames 为空(相机停了)时增长。丢最老会让下一帧的
    //                   预积分起点后移,但 feature_tracker 本来就会在缺口处补一个
    //                   外推样本(见 feature_tracker.cpp 里 "insert(begin(), imu)"),
    //                   所以是可降级的,不是硬损坏。上限给得很宽(默认约 10s@400Hz)。
    //   frontal_imus —— 只在 predict_pose 拿得到 latest_state 时才被清理;
    //                   初始化失败或丢跟踪期间**一次都不清**,是真正无界的那一条。
    //                   它纯粹是前向传播的临时缓冲,丢最老不影响任何交付数据。
    auto &c = runtime::counters();
    runtime::drop_oldest_over(imus, cap_pending_imu_, c.dropped_pending_imu,
                              "detail.imus(待挂帧 IMU)");
    runtime::drop_oldest_over(frontal_imus, cap_frontal_imu_,
                              c.dropped_frontal_imu,
                              "detail.frontal_imus(前向传播 IMU)");
    runtime::observe_depth(c.depth_pending_imu, c.hw_pending_imu, imus.size());
    runtime::observe_depth(c.depth_frontal_imu, c.hw_frontal_imu,
                           frontal_imus.size());
    runtime::observe_depth(c.depth_pending_camera_frames,
                           c.hw_pending_camera_frames, frames.size());
}

Pose XRSLAM::Detail::predict_pose(const double &t) {
    Pose output_pose;
    if (auto maybe_state = feature_tracker->get_latest_state()) {
        auto [state_time, state_pose, state_motion] = maybe_state.value();
        inspect_debug(input_output_lag, lag) {
            lag = std::min(t - state_time, 5.0);
        }
        // std::cout << "delay: " << t - state_time << std::endl;

        while (!frontal_imus.empty() && frontal_imus.front().t <= state_time) {
            frontal_imus.pop_front();
        }
        for (const auto &imu : frontal_imus) {
            if (imu.t <= t) {
                propagate_state(state_time, state_pose, state_motion, imu.t,
                                imu.w, imu.a);
            }
        }
        output_pose.q = state_pose.q * config->output_to_body_rotation();
        output_pose.p =
            state_pose.p + state_pose.q * config->output_to_body_translation();
    } else {
        output_pose.q.coeffs().setZero();
        output_pose.p.setZero();
    }

#if XRSLAM_ENABLE_VISUAL_LOCALIZATION
    if (config->visual_localization_enable() &&
        frontend->global_localization_state()) {
        if (frontend->localizer.get()) {
            return frontend->localizer->transform(output_pose);
        }
    }
#endif
    return output_pose;
}

size_t XRSLAM::Detail::create_virtual_object() {
    return frontend->create_virtual_object();
}

OutputObject XRSLAM::Detail::get_virtual_object_pose_by_id(size_t id) {

    OutputObject object = frontend->get_virtual_object_pose_by_id(id);
    return object;
}

SysState XRSLAM::Detail::get_system_state() const {
    return frontend->get_system_state();
}

void XRSLAM::Detail::enable_global_localization() {
    //   std::cout << "VLoc. Mode" << std::endl;
    frontend->set_global_localization_state(true);
}

void XRSLAM::Detail::disable_global_localization() {
    //   std::cout << "SLAM Mode" << std::endl;
    frontend->set_global_localization_state(false);
}

void XRSLAM::Detail::query_frame() { frontend->query_frame(); }

bool XRSLAM::Detail::global_localization_initialized() {
#if XRSLAM_ENABLE_VISUAL_LOCALIZATION
    return frontend->localizer ? frontend->localizer->is_initialized() : false;
#else
    return false;
#endif
}

// for AR
std::tuple<double, Pose> XRSLAM::Detail::get_latest_state() const {
    Pose output_pose;
    double timestamp;
    if (auto maybe_state = feature_tracker->get_latest_state()) {
        auto [state_time, state_pose, state_motion] = maybe_state.value();
        output_pose.q = state_pose.q * config->output_to_body_rotation();
        output_pose.p =
            state_pose.p + state_pose.q * config->output_to_body_translation();
        timestamp = state_time;
    } else {
        output_pose.q.coeffs().setZero();
        output_pose.p.setZero();
        timestamp = 0.0;
    }

#if XRSLAM_ENABLE_VISUAL_LOCALIZATION
    if (config->visual_localization_enable() &&
        frontend->global_localization_state()) {
        if (frontend->localizer.get()) {
            output_pose = frontend->localizer->transform(output_pose);
        }
    }
#endif

    return {timestamp, output_pose};
}

std::tuple<double, Pose> XRSLAM::Detail::get_latest_pose() {
    std::unique_lock<std::mutex> lk(latest_mutex_);
    return {latest_timestamp_, latest_pose_};
}

// ---------------------------------------------------------------------------
// [pw] 条目 08 / 17 / 配合 B 的只读遥测出口。
//      与 get_depth_fusion_stats 同款:核内定义,消费方(xrslam-interface)
//      直接前向声明即可,不需要 C API 改动。
// ---------------------------------------------------------------------------

void get_imu_timing(ImuTiming &out) {
    auto &a = runtime::imu_timing_atomics();
    auto &c = runtime::counters();
    out.samples = a.samples.load(std::memory_order_relaxed);
    out.warmed_up = a.warmed.load(std::memory_order_relaxed) != 0;
    out.measured_dt_median = a.dt_median.load(std::memory_order_relaxed);
    out.measured_rate_hz =
        out.measured_dt_median > 0.0 ? 1.0 / out.measured_dt_median : 0.0;
    out.jitter_cv = a.jitter_cv.load(std::memory_order_relaxed);
    out.batched = a.batched.load(std::memory_order_relaxed) != 0;
    out.batch_period = a.batch_period.load(std::memory_order_relaxed);
    out.batch_size = a.batch_size.load(std::memory_order_relaxed);
    out.fabricated_uniform =
        a.fabricated_uniform.load(std::memory_order_relaxed) != 0;
    out.nonmonotonic_samples =
        c.nonmonotonic_imu_dt.load(std::memory_order_relaxed);
    out.arrival_realtime =
        a.arrival_realtime.load(std::memory_order_relaxed) != 0;
    out.arrival_batched =
        a.arrival_batched.load(std::memory_order_relaxed) != 0;
    out.arrival_burst_ratio =
        a.arrival_burst_ratio.load(std::memory_order_relaxed);
    out.arrival_batch_size =
        a.arrival_batch_size.load(std::memory_order_relaxed);
    out.arrival_batch_period =
        a.arrival_batch_period.load(std::memory_order_relaxed);
}

void get_memory_budget_stats(MemoryBudgetStats &out) {
    auto &c = runtime::counters();
    const std::memory_order r = std::memory_order_relaxed;
    out.raw_gyro_queue = c.depth_raw_gyro.load(r);
    out.raw_accel_queue = c.depth_raw_accel.load(r);
    out.pending_imu_queue = c.depth_pending_imu.load(r);
    out.frontal_imu_queue = c.depth_frontal_imu.load(r);
    out.pending_camera_frames = c.depth_pending_camera_frames.load(r);
    out.tracker_frame_queue = c.depth_tracker_frame_queue.load(r);
    out.tracking_map_frames = c.depth_tracking_map_frames.load(r);
    out.tracking_map_tracks = c.depth_tracking_map_tracks.load(r);
    out.pending_frame_ids = c.depth_pending_frame_ids.load(r);
    out.sliding_window_frames = c.depth_sliding_window_frames.load(r);
    out.sliding_window_tracks = c.depth_sliding_window_tracks.load(r);
    out.hw_raw_gyro = c.hw_raw_gyro.load(r);
    out.hw_raw_accel = c.hw_raw_accel.load(r);
    out.hw_pending_imu = c.hw_pending_imu.load(r);
    out.hw_frontal_imu = c.hw_frontal_imu.load(r);
    out.hw_pending_camera_frames = c.hw_pending_camera_frames.load(r);
    out.hw_tracker_frame_queue = c.hw_tracker_frame_queue.load(r);
    out.hw_tracking_map_frames = c.hw_tracking_map_frames.load(r);
    out.dropped_raw_gyro = c.dropped_raw_gyro.load(r);
    out.dropped_raw_accel = c.dropped_raw_accel.load(r);
    out.dropped_pending_imu = c.dropped_pending_imu.load(r);
    out.dropped_frontal_imu = c.dropped_frontal_imu.load(r);
    out.dropped_pending_camera_frames = c.dropped_pending_camera_frames.load(r);
    out.dropped_tracker_frame_queue = c.dropped_tracker_frame_queue.load(r);
    out.dropped_tracking_map_frames = c.dropped_tracking_map_frames.load(r);
    out.dropped_pending_frame_ids = c.dropped_pending_frame_ids.load(r);
    out.gyro_buffer_clears = c.gyro_buffer_clears.load(r);
    out.gyro_samples_dropped_by_clear = c.gyro_samples_dropped_by_clear.load(r);
    out.accel_dropped_no_bracket = c.accel_dropped_no_bracket.load(r);
    out.nonmonotonic_imu_dt = c.nonmonotonic_imu_dt.load(r);
    out.zero_imu_dt = c.zero_imu_dt.load(r);
}

} // namespace xrslam
