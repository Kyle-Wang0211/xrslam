#ifndef XRSLAM_SLAMMANAGER_H
#define XRSLAM_SLAMMANAGER_H

#include <chrono>
#include <deque>
#include <iostream>
#include <mutex>
#include <opencv2/opencv.hpp>

#include "xrslam/inspection.h"
#include "xrslam/core/detail.h"
#include <xrslam/utility/logger.h>
#include "xrslam/extra/opencv_image.h"
#include "xrslam/extra/yaml_config.h"
#include "XRSLAM.h"

namespace xrslam {

class Config;
class Image;

class XRSLAMManager {
  public:
    static XRSLAMManager &Instance();
    ~XRSLAMManager();

    void Init(std::shared_ptr<Config> config);
    int CheckLicense(const char *license_path, const char *product_name);

    int  PushImage(XRSLAMImage *image);   // [pw] 返回 XRSLAM_OK / XRSLAM_ERR_*
    void PushDepth(XRSLAMDepthImage *depth);
    // [pw] 原为 void。加时间戳自检闸之后必须有错误通道,否则被拒的样本又是静默丢弃。
    int  PushAcceleration(XRSLAMAcceleration *acc);
    int  PushGyroscope(XRSLAMGyroscope *gyro);

    void RunOneFrame();

    void Destroy();

    // [pw] detail_/config_ 只在 Init 里创建、Destroy 里销毁。所有 getter 与
    // push 路径在解引用前必须先过这道闸,否则 Create 之前 / Destroy 之后调用
    // 就是空指针解引用 —— 段错误不是 C++ 异常,extern "C" 上的 catch(...) 拦不住。
    bool ready() const { return detail_ != nullptr && config_ != nullptr; }

    // [pw] 类型化 getter。返回 XRSLAM_OK / XRSLAM_ERR_*,出参无条件先清零。
    int GetCameraPose(XRSLAMPose *pose) const;
    int GetBodyPose(XRSLAMPose *pose) const;
    int GetState(XRSLAMState *state) const;
    int GetBias(XRSLAMIMUBias *bias) const;
    int GetIntrinsics(XRSLAMIntrinsics *intrinsics) const;
    int GetVersion(char *out_buf, int32_t *io_len) const;
    int GetLandmarks(double *out_xyz, int32_t *io_point_count) const;
    int GetLandmarksEx(double *out_xyz, unsigned char *out_flags,
                       int32_t *io_point_count,
                       XRSLAMLandmarkStats *out_stats) const;
    int GetHealth(XRSLAMHealth *out) const;
    int SetHealthThresholds(double max_abs_cam_imu_delta_sec,
                            double max_baseline_drift_sec,
                            int32_t min_landmarks_for_healthy);
    int TryGetLatestPose(double *out_pose7, double *out_timestamp);

    // [pw] 已删除 GetResultLandmarks(XRSLAMLandmarks*):library-allocates + 无 Free
    //      = 每帧泄漏。改用上面的 GetLandmarks(caller-allocates)。
    // [pw] 已删除 GetResultVersion(XRSLAMStringOutput*):同款 new char[] 无人 delete。
    //      改用上面的 GetVersion(caller-allocates)。
    void GetResultFeatures(XRSLAMFeatures *features) const; // NOT IMPLEMENTED,只置空
    void GetDepthFusionStats(int *seeded, int *total) const;
  private:
    XRSLAMManager();

    // [pw] ---- landmark 取数的公共实现(条目 05)----
    // require_triangulated=true  -> XRSLAMGetLandmarks 的严格口径
    // require_triangulated=false -> XRSLAMGetLandmarksEx 的宽口径(只滤非有限值)
    int GetLandmarksImpl(double *out_xyz, unsigned char *out_flags,
                         int32_t *io_point_count, XRSLAMLandmarkStats *out_stats,
                         bool require_triangulated) const;

    // [pw] ---- 时间戳自检闸(Blocker 01)----
    // 三个 note_* 都要求**已持有 health_mutex_**。
    // 返回 XRSLAM_OK 表示样本可用;否则调用方应丢弃该样本。
    // 返回 XRSLAM_OK 表示时间戳合法;t 非有限、或不严格大于 *last_t,则计数并拒。
    int  gate_monotonic_locked(double t, double *last_t);
    void note_cross_channel_locked(bool is_camera, double t);
    void shadow_drain_locked();
    void reset_health_locked();

  private:
    std::shared_ptr<Config> config_;
    std::unique_ptr<XRSLAM::Detail> detail_;
    std::mutex input_mutex_;
    std::shared_ptr<xrslam::Image> cur_image_;
    std::shared_ptr<xrslam::DepthMap> cur_depth_;

    // [pw] ---- 健康计数(条目 16)。锁序铁律:input_mutex_ 与 health_mutex_
    //      **绝不嵌套**。PushImage 先取 health_mutex_ 做闸、放掉,最后才取
    //      input_mutex_;RunOneFrame 先在 input_mutex_ 作用域里跑完 track_camera,
    //      出了作用域再取 health_mutex_ 记账。 ----
    mutable std::mutex health_mutex_;

    double h_last_image_t_       = 0.0;
    double h_last_imu_t_         = 0.0;   // acc/gyro 取较新者
    double h_last_accel_t_       = 0.0;   // 各路各自的单调性基准
    double h_last_gyro_t_        = 0.0;
    double h_last_cam_imu_delta_ = 0.0;
    double h_baseline_delta_     = 0.0;
    double h_max_abs_delta_      = 0.0;
    double h_max_abs_drift_      = 0.0;
    double h_last_frame_ms_      = 0.0;
    bool   h_baseline_set_       = false;
    bool   h_domain_mismatch_    = false;

    // 阈值:Create/Destroy 不重置,由 SetHealthThresholds 设置。
    double  h_thr_delta_    = 1.0;
    double  h_thr_drift_    = 0.5;
    int32_t h_thr_min_lms_  = 20;

    int64_t h_image_accepted_ = 0, h_image_rejected_ = 0;
    int64_t h_accel_accepted_ = 0, h_accel_rejected_ = 0;
    int64_t h_gyro_accepted_  = 0, h_gyro_rejected_  = 0;
    int64_t h_reject_non_finite_ = 0, h_reject_non_monotonic_ = 0;
    int64_t h_reject_bad_arg_    = 0;
    int64_t h_domain_events_     = 0, h_drift_events_ = 0;
    int64_t h_frames_run_        = 0, h_frames_zero_imu_ = 0;
    int64_t h_shadow_overflow_   = 0;
    int32_t h_imu_last_frame_    = -1;   // -1 = 还没有任何一帧被派发过
    int32_t h_ts_convention_     = 0;

    // [pw] 影子队列:复刻 detail.cpp XRSLAM::Detail::track_imu 的归并谓词,
    //      用来算出"这一帧到底有几个 IMU 样本进了 preintegration"。
    //      核内没有任何现成信号能读到 Frame::preintegration.data.size()。
    static const size_t kShadowCap = 4096;
    std::deque<double> shadow_imu_ts_;
    std::deque<double> shadow_frame_ts_;
    int32_t shadow_pending_imu_ = 0;

    // [pw] TryGetLatestPose 的"上次交出去的时间戳",独立小锁,不与上面两把嵌套。
    mutable std::mutex pose_serve_mutex_;
    double  last_served_pose_t_ = 0.0;
};
} // namespace xrslam
#endif
