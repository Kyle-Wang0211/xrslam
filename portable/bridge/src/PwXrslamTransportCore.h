#ifndef POCKETWORLD_XRSLAM_TRANSPORT_CORE_H_
#define POCKETWORLD_XRSLAM_TRANSPORT_CORE_H_

#include <stdint.h>
#include "PwXrslamParsedCameraConfig.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct PWXrslamRawPose {
  double timestamp;
  // Frozen upstream XRSLAM camera-pose ABI order is x, y, z, w.
  double quaternion[4];
  double translation[3];
} PWXrslamRawPose;

typedef enum PWXrslamTransportStatus {
  PW_XRSLAM_OK = 0,
  PW_XRSLAM_ERR_INVALID_ARGUMENT = -1,
  PW_XRSLAM_ERR_NOT_RUNNING = -2,
  PW_XRSLAM_ERR_NON_MONOTONIC = -3,
  PW_XRSLAM_ERR_INVALID_POSE = -4
} PWXrslamTransportStatus;

typedef enum PWXrslamStream {
  PW_XRSLAM_STREAM_CAMERA = 0,
  PW_XRSLAM_STREAM_ACCELERATION = 1,
  PW_XRSLAM_STREAM_GYROSCOPE = 2
} PWXrslamStream;

typedef enum PWXrslamTransformSemantics {
  PW_XRSLAM_T_WORLD_CAMERA = 1
} PWXrslamTransformSemantics;

typedef enum PWXrslamQuaternionOrder {
  PW_XRSLAM_QUATERNION_XYZW = 1
} PWXrslamQuaternionOrder;

// Diagnostic receipt for the most recent attempted sample on one stream.
// A submitted_sequence increment proves only that the transport called the
// frozen official void push ABI; it is deliberately not named "accepted".
typedef struct PWXrslamTimestampTrace {
  int32_t stream;
  int32_t status;
  double raw_timestamp;
  double applied_offset;
  double effective_timestamp;
  uint64_t submitted_sequence;
} PWXrslamTimestampTrace;

typedef struct PWXrslamPoseMetadata {
  int32_t transform_semantics;
  int32_t quaternion_order;
} PWXrslamPoseMetadata;

typedef struct PWXrslamDestroyReceipt {
  uint64_t lifecycle_generation;
  uint64_t camera_submitted;
  uint64_t camera_run_calls;
  uint64_t acceleration_submitted;
  uint64_t gyroscope_submitted;
  uint64_t rejected_invalid_argument;
  uint64_t rejected_non_monotonic;
  uint64_t rejected_not_running;
  int32_t destroy_acknowledged;
} PWXrslamDestroyReceipt;

// Live transport receipt. XRSLAM's frozen void sensor ABI does not expose an
// internal acceptance counter, so these values mean the wrapper successfully
// called that exact ABI. They are read from the same C++ ledger used by the
// terminal Destroy receipt; Swift must not synthesize them.
typedef struct PWXrslamTransportCounters {
  uint64_t lifecycle_generation;
  uint64_t camera_submitted;
  uint64_t camera_run_calls;
  uint64_t acceleration_submitted;
  uint64_t gyroscope_submitted;
  uint64_t rejected_invalid_argument;
  uint64_t rejected_non_monotonic;
  uint64_t rejected_not_running;
  int32_t running;
} PWXrslamTransportCounters;

// This describes the selected linked archive's XRSLAMCreate ABI, not the caller
// input: the public Create entry points continue to receive file paths.
typedef enum PWXrslamArtifactConfigInputMode {
  PW_XRSLAM_ARTIFACT_FILE_PATHS = 1,
  PW_XRSLAM_ARTIFACT_YAML_TEXT = 2
} PWXrslamArtifactConfigInputMode;

int32_t PWXrslamTransportGetArtifactConfigInputMode(void);

int32_t PWXrslamTransportCreate(const char *slam_config_path,
                                const char *device_config_path);

// Applies camera_time_offset_seconds once, at the shared transport boundary,
// before the camera sample reaches XRSLAM. IMU timestamps are unchanged.
// The return value preserves the frozen XRSLAMCreate convention: 1 success,
// 0 failure.
int32_t
PWXrslamTransportCreateWithCameraTimeOffset(const char *slam_config_path,
                                            const char *device_config_path,
                                            double camera_time_offset_seconds);

void PWXrslamTransportDestroy(void);
int32_t PWXrslamTransportDestroyWithReceipt(PWXrslamDestroyReceipt *receipt);

// Common PC frontend: every valid camera is pushed. Run/result queries begin
// only after both IMU streams have been submitted in the current generation;
// CAMERA pose is queried only after TRACKING_SUCCESS. Pre-IMU output is an
// initialized raw state and zero pose, without delaying or dropping the image.
// PW_XRSLAM_OK confirms the camera Push call; it does not imply a Run call.
int32_t PWXrslamTransportPushCameraAndRunRaw(uint8_t *data, double timestamp,
                                             int32_t stride, int32_t camera_id,
                                             int32_t channel,
                                             int32_t *raw_state,
                                             PWXrslamRawPose *raw_pose);

int32_t PWXrslamTransportPushAccelerationRaw(double timestamp, double x,
                                             double y, double z);
int32_t PWXrslamTransportPushGyroscopeRaw(double timestamp, double x, double y,
                                          double z);

int32_t PWXrslamTransportGetLastTimestampTrace(int32_t stream,
                                               PWXrslamTimestampTrace *trace);

int32_t PWXrslamTransportGetPoseMetadata(PWXrslamPoseMetadata *metadata);
int32_t PWXrslamTransportGetCounters(PWXrslamTransportCounters *counters);

// Cross-platform transport preparation for a caller-owned preallocated gray
// frame slot. It preserves the versioned box-NxN half-up kernel already used
// by PocketWorld and performs no XRSLAM state transition or policy decision.
int32_t PWXrslamTransportPrepareGrayBoxNxN(
    const uint8_t *source, int32_t source_width, int32_t source_height,
    int32_t source_stride, int32_t factor, uint8_t *destination,
    int32_t destination_capacity, int32_t *destination_width,
    int32_t *destination_height);

// Converts the frozen upstream camera result (T_world_camera, quaternion xyzw)
// to a row-major 4x4 homogeneous diagnostic matrix using Eigen Quaterniond
// toRotationMatrix without normalization, after the existing validity checks.
// PC viewer uses the same Eigen API with float precision. Raw pose is unchanged.
int32_t
PWXrslamTransportPoseToWorldFromCameraMatrix(const PWXrslamRawPose *pose,
                                             double matrix_row_major_4x4[16]);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // POCKETWORLD_XRSLAM_TRANSPORT_CORE_H_
