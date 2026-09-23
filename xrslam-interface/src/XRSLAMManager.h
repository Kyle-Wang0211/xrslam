#ifndef XRSLAM_SLAMMANAGER_H
#define XRSLAM_SLAMMANAGER_H

#include <iostream>
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

/// [bench 2026-09-17] Replicated verbatim from Monado's `enum xrt_space_relation_flags`
/// (src/xrt/include/xrt/xrt_defines.h, SPDX-License-Identifier: BSL-1.0, Copyright 2019-2024
/// Collabora Ltd. / 2024-2026 NVIDIA CORPORATION), which is itself the reference implementation of
/// the OpenXR 1.1 `XrSpaceLocationFlags` contract. Bit values and ordering are Monado's; the
/// velocity bits are kept so the mask stays byte-compatible with the source even though this
/// interface does not yet publish velocity.
///
/// Why this contract and not a single "is it good" boolean: OpenXR separates *may I read this*
/// from *was this observed*. The spec (spaces.adoc) says a runtime losing tracking "should continue
/// to provide valid but untracked position values that are inferred or last-known, e.g. based on
/// neck model updates, **inertial dead reckoning**, or a last-known position ... clearing
/// XR_SPACE_LOCATION_POSITION_TRACKED_BIT until positional tracking is recovered", while an
/// application "must not read the pose field's position if this flag is unset".
///
/// That is exactly the distinction this interface was missing. `GetResultPropagatedPose` returns an
/// all-zero quaternion before the first propagated pose exists, and a caller had no way to tell that
/// apart from a pose -- which is why a live run that had merely not initialised yet was scored as a
/// tracking failure. Under this contract "not initialised" is BITMASK_NONE, a dead-reckoned pose is
/// VALID without TRACKED, and only an optimised pose carries both.
enum XRSLAMSpaceRelationFlags {
    XRSLAM_SPACE_RELATION_ORIENTATION_VALID_BIT = (1u << 0u),
    XRSLAM_SPACE_RELATION_POSITION_VALID_BIT = (1u << 1u),
    XRSLAM_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT = (1u << 2u),
    XRSLAM_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT = (1u << 3u),
    XRSLAM_SPACE_RELATION_ORIENTATION_TRACKED_BIT = (1u << 4u),
    XRSLAM_SPACE_RELATION_POSITION_TRACKED_BIT = (1u << 5u),
    XRSLAM_SPACE_RELATION_BITMASK_ALL =
        (unsigned int)XRSLAM_SPACE_RELATION_ORIENTATION_VALID_BIT |
        (unsigned int)XRSLAM_SPACE_RELATION_POSITION_VALID_BIT |
        (unsigned int)XRSLAM_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT |
        (unsigned int)XRSLAM_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT |
        (unsigned int)XRSLAM_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
        (unsigned int)XRSLAM_SPACE_RELATION_POSITION_TRACKED_BIT,
    /// Monado's note on its XRT_SPACE_RELATION_ZERO, replicated because the convention is the
    /// whole point: "Despite this initializing all members (to zero or identity), this sets the
    /// relation_flags to XRT_SPACE_RELATION_BITMASK_NONE - so this is safe to assign before an
    /// error return, etc."
    XRSLAM_SPACE_RELATION_BITMASK_NONE = 0,
};

class XRSLAMManager {
  public:
    static XRSLAMManager &Instance();

    /// Bisect variant A.
    int PendingWorkerFrames() const;
    ~XRSLAMManager();

    void Init(std::shared_ptr<Config> config);
    int CheckLicense(const char *license_path, const char *product_name);

    void PushImage(XRSLAMImage *image);
    void PushAcceleration(XRSLAMAcceleration *acc);
    void PushGyroscope(XRSLAMGyroscope *gyro);

    void RunOneFrame();

    void Destroy();

    void GetResultCameraPose(XRSLAMPose *pose) const;
    void GetResultBodyPose(XRSLAMPose *pose) const;
    /// [bench 2026-09-09] The IMU-propagated pose, at the timestamp of the newest IMU sample.
    ///
    /// `Detail::track_gyroscope` and `track_accelerometer` each already return `predict_pose(t)` --
    /// the last optimised state propagated through the IMU that arrived after it -- and this
    /// interface discarded that return value, so `GetResultBodyPose` could only ever advance once
    /// per image (`latest_pose_` is written in `track_camera`). Nothing new is computed here: the
    /// value upstream hands back is kept instead of dropped. Returns a zero quaternion before the
    /// first propagated pose exists.
    void GetResultPropagatedPose(XRSLAMPose *pose) const;
    /// Pose plus the flags describing which of its components may be read and which were actually
    /// observed. Filled under one lock so the flags always describe the pose handed back with them
    /// -- the reason the source keeps them in a single struct rather than two accessors.
    void GetPropagatedPoseRelation(XRSLAMPose *pose, unsigned int *flags) const;
    void GetBodyPoseRelation(XRSLAMPose *pose, unsigned int *flags) const;
    void KeepPropagatedPose(double t, const Pose &pose);
    void GetResultState(XRSLAMState *state) const;
    void GetResultLandmarks(XRSLAMLandmarks *landmarks) const;
    void GetResultFeatures(XRSLAMFeatures *features) const;
    void GetResultBias(XRSLAMIMUBias *bias) const;
    void GetResultVersion(XRSLAMStringOutput *output) const;
    void GetInfoIntrinsics(XRSLAMIntrinsics *intrinsics) const;
  private:
    XRSLAMManager();

  private:
    std::shared_ptr<Config> config_;
    std::unique_ptr<XRSLAM::Detail> detail_;
    std::mutex input_mutex_;
    std::shared_ptr<xrslam::Image> cur_image_;
    // The pose Detail::track_gyroscope/track_accelerometer returns, kept instead of discarded.
    mutable std::mutex imu_pose_mutex_;
    Pose imu_pose_;
    double imu_pose_timestamp_ = 0.0;
    bool imu_pose_valid_ = false;
    // [pw 2026-09-22 逐帧内参] 最近一帧由 ext 送进来的 K,供 GetInfoIntrinsics 报出。
    mutable std::mutex intrinsics_mutex_;
    XRSLAMIntrinsics latest_intrinsics_{};
    bool has_latest_intrinsics_ = false;
};
} // namespace xrslam
#endif
