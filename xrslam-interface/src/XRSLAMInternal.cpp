#include <atomic>
#include "XRSLAMManager.h"
#include "xrslam/xrslam.h"

int XRSLAMCreate(
    const char *slam_config_path,   // slam configuration file path
    const char *device_config_path, // device configuration file path
    const char *license_path, const char *product_name, void **config) {
    if (xrslam::XRSLAMManager::Instance().CheckLicense(license_path,
                                                       product_name) == 0)
        return 0;

    std::shared_ptr<xrslam::extra::YamlConfig> yaml_config =
        std::make_shared<xrslam::extra::YamlConfig>(slam_config_path,
                                                    device_config_path);
    xrslam::XRSLAMManager::Instance().Init(yaml_config);
    *config = static_cast<void *>(yaml_config.get());
    return 1;
}

void XRSLAMPushSensorData(XRSLAMSensorType sensor_type, // sensor type
                          void *sensor_data             // sensor data
) {
    switch (sensor_type) {
    case XRSLAM_SENSOR_CAMERA:
        xrslam::XRSLAMManager::Instance().PushImage(
            static_cast<XRSLAMImage *>(sensor_data));
        break;
    case XRSLAM_SENSOR_ACCELERATION:
        xrslam::XRSLAMManager::Instance().PushAcceleration(
            static_cast<XRSLAMAcceleration *>(sensor_data));
        break;
    case XRSLAM_SENSOR_GYROSCOPE:
        xrslam::XRSLAMManager::Instance().PushGyroscope(
            static_cast<XRSLAMGyroscope *>(sensor_data));
        break;
    case XRSLAM_SENSOR_DEPTH_CAMERA:
    case XRSLAM_SENSOR_GRAVITY:
    case XRSLAM_SENSOR_ROTATION_VECTOR:
    case XRSLAM_SENSOR_UNKNOWN:
    default:
        break;
    }
}

void XRSLAMRunOneFrame() { xrslam::XRSLAMManager::Instance().RunOneFrame(); }

// [bench 2026-09-09] Additive: the IMU-propagated pose, without touching XRSLAMGetResult's
// contract. See XRSLAMManager::GetResultPropagatedPose.
void XRSLAMGetPropagatedPose(XRSLAMPose *pose) {
    xrslam::XRSLAMManager::Instance().GetResultPropagatedPose(pose);
}

void XRSLAMGetResult(XRSLAMResultType result_type, // result type
                     void *result_data             // result data
) {
    switch (result_type) {
    case XRSLAM_RESULT_BODY_POSE:
        xrslam::XRSLAMManager::Instance().GetResultBodyPose(
            static_cast<XRSLAMPose *>(result_data));
        break;
    case XRSLAM_RESULT_CAMERA_POSE:
        xrslam::XRSLAMManager::Instance().GetResultCameraPose(
            static_cast<XRSLAMPose *>(result_data));
        break;
    case XRSLAM_RESULT_STATE:
        xrslam::XRSLAMManager::Instance().GetResultState(
            static_cast<XRSLAMState *>(result_data));
        break;
    case XRSLAM_RESULT_LANDMARKS:
        xrslam::XRSLAMManager::Instance().GetResultLandmarks(
            static_cast<XRSLAMLandmarks *>(result_data));
        break;
    case XRSLAM_RESULT_FEATURES:
        xrslam::XRSLAMManager::Instance().GetResultFeatures(
            static_cast<XRSLAMFeatures *>(result_data));
        break;
    case XRSLAM_RESULT_BIAS:
        xrslam::XRSLAMManager::Instance().GetResultBias(
            static_cast<XRSLAMIMUBias *>(result_data));
        break;
    case XRSLAM_RESULT_VERSION:
        xrslam::XRSLAMManager::Instance().GetResultVersion(
            static_cast<XRSLAMStringOutput *>(result_data));
        break;
    case XRSLAM_INFO_INTRINSICS:
        xrslam::XRSLAMManager::Instance().GetInfoIntrinsics(
            static_cast<XRSLAMIntrinsics *>(result_data));
        break;
    case XRSLAM_RESULT_DEBUG_LOGS:
    case XRSLAM_RESULT_UNKNOWN:
    default:
        break;
    }
}

void XRSLAMDestroy() { xrslam::XRSLAMManager::Instance().Destroy(); }

extern "C" int XRSLAMGetPendingWorkerFrames(void) {
    return xrslam::XRSLAMManager::Instance().PendingWorkerFrames();
}

namespace xrslam {
extern std::atomic<uint64_t> pw_init_counters[8];
extern std::atomic<uint64_t> pw_init_mirror_us;
extern std::atomic<uint64_t> pw_solver_counters[];
}

// [bench 2026-09-09] Read-only: why initialisation has not finished yet, in the order
// too_few_frames, attempts, fail_matches, fail_parallax, fail_rotation, fail_triangulation,
// fail_imu, success. Declared in the bench bridge rather than XRSLAM.h so the public ABI header
// stays byte-identical to upstream's, the same way the backlog accessor is.
extern "C" void XRSLAMGetInitCounters(unsigned long long *out, int count) {
    if (out == nullptr)
        return;
    for (int i = 0; i < count && i < 8; ++i)
        out[i] = xrslam::pw_init_counters[i].load(std::memory_order_relaxed);
    if (count > 8)
        out[8] = xrslam::pw_init_mirror_us.load(std::memory_order_relaxed);
}

// [bench 2026-09-17] Read-only: the two "should this frame be trusted" quantities the sliding
// window already computes and then drops -- Ceres' own IsSolutionUsable() verdict, and the
// per-track mean reprojection error behind the TT_VALID gate. Order matches PwSolverCounter in
// sliding_window_tracker.cpp:
//   0 solve_calls, 1 solve_unusable, 2 track_evaluated, 3 track_reject_depth, 4 track_reject_rpe,
//   5 track_rpe_samples, 6 track_rpe_millipx, 7 frames_rpe_calls, 8 frames_rpe_reject,
//   9 frames_rpe_samples, 10 frames_rpe_millipx
// Declared here rather than in XRSLAM.h so the public ABI header stays byte-identical to
// upstream's, the same way XRSLAMGetInitCounters is.
extern "C" void XRSLAMGetSolverCounters(unsigned long long *out, int count) {
    if (out == nullptr)
        return;
    for (int i = 0; i < count && i < 11; ++i)
        out[i] = xrslam::pw_solver_counters[i].load(std::memory_order_relaxed);
}

// [bench 2026-09-17] Pose + OpenXR/Monado relation flags, together. See
// XRSLAMSpaceRelationFlags in XRSLAMManager.h for the contract and its source. Declared here rather
// than in XRSLAM.h so the public ABI header stays byte-identical to upstream's, the same way
// XRSLAMGetInitCounters and XRSLAMGetSolverCounters are.
//
// `flags == 0` (BITMASK_NONE) means no component of the pose may be read -- this is what "still
// initialising" looks like, and it is deliberately the same value the accessors assign before any
// early return, so a caller that ignores the return path still sees "do not read".
extern "C" void XRSLAMGetPropagatedPoseRelation(XRSLAMPose *pose, unsigned int *flags) {
    xrslam::XRSLAMManager::Instance().GetPropagatedPoseRelation(pose, flags);
}

extern "C" void XRSLAMGetBodyPoseRelation(XRSLAMPose *pose, unsigned int *flags) {
    xrslam::XRSLAMManager::Instance().GetBodyPoseRelation(pose, flags);
}
