#ifndef XRSLAM_VIO_BENCH_NATIVE_H
#define XRSLAM_VIO_BENCH_NATIVE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Stable C boundary around the complete pinned OpenXRLab stock XRSLAM core at
 * 4beb1a942f33da9afbfae2d70e2c641cfc2bb675. This adapter does not use ARKit,
 * RealityKit or any alternate/synthetic estimator. The linked stock library
 * defines XRSLAM_IOS, while this wrapper uses its five official C APIs.
 *
 * Calls which submit sensors and advance frames must be serialized by the
 * caller. IMU events are forwarded synchronously in accepted callback order,
 * matching the official iOS sample. One create-time preallocated grayscale
 * slot holds the image only between push_image and run_one_frame; it cannot
 * grow. Monotonicity is checked per sensor stream. The only timestamp
 * conversion is ns -> seconds at the exact upstream C ABI boundary; no
 * camera/IMU offset is applied here.
 */
typedef struct xrslam_bench xrslam_bench_t;

typedef enum xrslam_bench_status {
  XRSLAM_BENCH_OK = 0,
  XRSLAM_BENCH_INVALID_ARGUMENT = 1,
  XRSLAM_BENCH_BACKEND_UNAVAILABLE = 2,
  XRSLAM_BENCH_CREATE_FAILED = 3,
  XRSLAM_BENCH_INVALID_STATE = 4,
  XRSLAM_BENCH_TIMESTAMP_REGRESSION = 5,
  XRSLAM_BENCH_INPUT_SEALED = 6,
  XRSLAM_BENCH_NO_FRAME = 7,
  XRSLAM_BENCH_NO_OUTPUT = 8,
  XRSLAM_BENCH_END_OF_STREAM = 9,
  XRSLAM_BENCH_NONFINITE_OUTPUT = 10,
  XRSLAM_BENCH_INTERNAL_ERROR = 11,
  XRSLAM_BENCH_QUEUE_FULL = 12
} xrslam_bench_status_t;

typedef enum xrslam_bench_phase {
  XRSLAM_BENCH_PHASE_RUNNING = 0,
  XRSLAM_BENCH_PHASE_SEALED = 1,
  XRSLAM_BENCH_PHASE_DRAINED = 2,
  XRSLAM_BENCH_PHASE_STOPPED = 3,
  XRSLAM_BENCH_PHASE_FAILED = 4
} xrslam_bench_phase_t;

typedef struct xrslam_bench_create_options {
  size_t struct_size;
  /* Filesystem paths at the unchanged bench ABI. This stock adapter reads
     both files and passes their text to the official XRSLAM_IOS YAML::Load. */
  const char *slam_config_path;
  const char *device_config_path;
  uint32_t image_width;
  uint32_t image_height;
} xrslam_bench_create_options_t;

typedef struct xrslam_bench_image {
  const uint8_t *gray_pixels;
  uint32_t width;
  uint32_t height;
  uint32_t bytes_per_row;
} xrslam_bench_image_t;

typedef struct xrslam_bench_frame_result {
  /// Stock uses -1 for unavailable divergence metadata; this is no health verdict.
  int32_t divergence_flags;
  int64_t input_timestamp_ns;
  int64_t pose_timestamp_ns;
  double pose_timestamp_seconds;
  /* Exact official BODY_POSE fields: quaternion is x,y,z,w. */
  double quaternion_xyzw[4];
  double translation_xyz[3];
  int32_t state;
} xrslam_bench_frame_result_t;

/* Stock lacks extended estimator telemetry. Its init/solver/relation/divergence
   counters use UINT64_MAX to mean unavailable; transport counters remain real. */
typedef struct xrslam_bench_counters {
  /// Poses read with xrslam_bench_query_pose (IMU-propagated, no frame consumed).
  uint64_t poses_queried;
  /// Why initialisation has not finished. Two of the engine's four SfM exits are silent, so without
  /// these a run can only say "still initialising"; with them, time-to-first-pose is attributable.
  uint64_t init_too_few_frames;   ///< history shorter than keyframe_gap * (keyframe_num - 1)
  uint64_t init_attempts;
  uint64_t init_fail_matches;     ///< common tracks < initializer.min_matches
  uint64_t init_fail_parallax;    ///< mean parallax < initializer.min_parallax
  uint64_t init_fail_rotation;    ///< homography says pure rotation
  uint64_t init_fail_triangulation;
  uint64_t init_fail_imu;
  uint64_t init_success;
  /// [2026-09-17] "Should this frame be trusted" -- two quantities the sliding window already
  /// computes and then drops. `Solver::solve()` returns Ceres' own IsSolutionUsable() and all four
  /// call sites ignored it; the per-track mean reprojection error behind the TT_VALID gate was
  /// compared against a bare 3.0 px and discarded. Read-only: the engine-side counters sit beside
  /// existing statements and the EuRoC replay is byte-identical with them compiled in.
  uint64_t solve_calls;
  uint64_t solve_unusable;        ///< solve() returned !IsSolutionUsable()
  uint64_t track_evaluated;       ///< triangulated tracks passing the TT_VALID gate
  uint64_t track_reject_depth;    ///< rejected by the y.z() <= 1e-3 || > 50 window
  uint64_t track_reject_rpe;      ///< rejected by mean rpe >= 3.0 px
  uint64_t track_rpe_samples;
  uint64_t track_rpe_millipx;     ///< sum of per-track mean rpe, in milli-pixels
  uint64_t frames_rpe_calls;
  uint64_t frames_rpe_reject;
  uint64_t frames_rpe_samples;
  uint64_t frames_rpe_millipx;
  /// [2026-09-17] OpenXR/Monado relation-flag tallies for the pose handed to the consumer.
  /// The contract separates "may I read this" (VALID) from "was this observed" (TRACKED); before it
  /// existed, a frame that had merely not initialised yet was indistinguishable from a failure.
  uint64_t relation_samples;
  uint64_t relation_none;               ///< BITMASK_NONE: no component may be read
  uint64_t relation_orientation_unreadable; ///< ORIENTATION_VALID clear (the zero-norm quaternion)
  uint64_t relation_position_unreadable;    ///< POSITION_VALID clear
  uint64_t relation_valid_untracked;    ///< readable but dead-reckoned (VALID set, TRACKED clear)
  uint64_t relation_tracked;            ///< at least one TRACKED bit set
  uint64_t events_offered;
  uint64_t events_accepted;
  uint64_t events_processed;
  uint64_t events_rejected_timestamp;
  uint64_t events_rejected_sealed;
  uint64_t events_rejected_queue_full;
  uint64_t accel_offered;
  uint64_t accel_accepted;
  uint64_t accel_processed;
  uint64_t accel_rejected_timestamp;
  uint64_t accel_rejected_sealed;
  uint64_t accel_rejected_queue_full;
  uint64_t gyro_offered;
  uint64_t gyro_accepted;
  uint64_t gyro_processed;
  uint64_t gyro_rejected_timestamp;
  uint64_t gyro_rejected_sealed;
  uint64_t gyro_rejected_queue_full;
  uint64_t images_offered;
  uint64_t images_accepted;
  uint64_t images_processed;
  uint64_t images_rejected_timestamp;
  uint64_t images_rejected_sealed;
  uint64_t images_rejected_queue_full;
  uint64_t image_bytes_copied;
  uint64_t replay_images_decoded;
  uint64_t replay_images_undistorted;
  uint64_t replay_image_decode_failures;
  uint64_t replay_image_preprocess_failures;
  uint64_t frames_run;
  uint64_t results_produced;
  uint64_t results_polled;
  uint64_t results_rejected_queue_full;
  uint64_t nonfinite_results_rejected;
  /// [2026-09-16] `nonfinite_results_rejected` folds three unrelated causes into
  /// one number, and the live validity gate fails a run on it. Every live run to
  /// date has been marked invalid by exactly one of these, and the count is 1 in
  /// every run that produced poses and 0 in every run that produced none -- which
  /// says it is a once-per-initialization event, not a numerical failure. These
  /// three separate the causes so the gate can be argued about on evidence.
  uint64_t result_state_out_of_range;
  uint64_t result_pose_nonfinite;
  uint64_t result_timestamp_unconvertible;
  uint64_t divergence_frames;
  uint64_t divergence_flags_seen;
  /// Longest unbroken stretch the divergence condition held, and how many
  /// frames it held past ORB-SLAM3's inertial time_recently_lost of 5 s.
  uint64_t divergence_longest_ms;
  uint64_t divergence_sustained_frames;
  uint64_t upstream_destroy_calls;
} xrslam_bench_counters_t;

typedef struct xrslam_bench_snapshot {
  xrslam_bench_phase_t phase;
  int64_t last_event_timestamp_ns;
  int64_t last_accel_timestamp_ns;
  int64_t last_gyro_timestamp_ns;
  int64_t last_image_timestamp_ns;
  uint64_t pending_event_count;
  uint64_t pending_result_count;
  uint64_t event_queue_capacity;
  uint64_t event_queue_peak;
  /// UINT64_MAX in stock: there is no official worker-backlog C API.
  uint64_t engine_backlog_peak;
  /// What actually reached the estimator, as opposed to what the bridge
  /// accepted. Upstream pairs the accelerometer and gyroscope streams against
  /// each other and drops unpaired samples without a trace.
  uint64_t estimator_imu_ingested;
  uint64_t estimator_camera_ingested;
  /// Initialisation exit counts, mirrored from the counters block: too_few_frames, attempts, the
  /// four SfM exits, the IMU exit, success. Two of the SfM exits are silent in the engine's logging,
  /// which is why time-to-first-pose could not be attributed before these existed.
  uint64_t init_too_few_frames;
  uint64_t init_attempts;
  uint64_t init_fail_matches;
  uint64_t init_fail_parallax;
  uint64_t init_fail_rotation;
  uint64_t init_fail_triangulation;
  uint64_t init_fail_imu;
  uint64_t init_success;
  /// [2026-09-17] "Should this frame be trusted" -- two quantities the sliding window already
  /// computes and then drops. `Solver::solve()` returns Ceres' own IsSolutionUsable() and all four
  /// call sites ignored it; the per-track mean reprojection error behind the TT_VALID gate was
  /// compared against a bare 3.0 px and discarded. Read-only: the engine-side counters sit beside
  /// existing statements and the EuRoC replay is byte-identical with them compiled in.
  uint64_t solve_calls;
  uint64_t solve_unusable;        ///< solve() returned !IsSolutionUsable()
  uint64_t track_evaluated;       ///< triangulated tracks passing the TT_VALID gate
  uint64_t track_reject_depth;    ///< rejected by the y.z() <= 1e-3 || > 50 window
  uint64_t track_reject_rpe;      ///< rejected by mean rpe >= 3.0 px
  uint64_t track_rpe_samples;
  uint64_t track_rpe_millipx;     ///< sum of per-track mean rpe, in milli-pixels
  uint64_t frames_rpe_calls;
  uint64_t frames_rpe_reject;
  uint64_t frames_rpe_samples;
  uint64_t frames_rpe_millipx;
  /// [2026-09-17] OpenXR/Monado relation-flag tallies for the pose handed to the consumer.
  /// The contract separates "may I read this" (VALID) from "was this observed" (TRACKED); before it
  /// existed, a frame that had merely not initialised yet was indistinguishable from a failure.
  uint64_t relation_samples;
  uint64_t relation_none;               ///< BITMASK_NONE: no component may be read
  uint64_t relation_orientation_unreadable; ///< ORIENTATION_VALID clear (the zero-norm quaternion)
  uint64_t relation_position_unreadable;    ///< POSITION_VALID clear
  uint64_t relation_valid_untracked;    ///< readable but dead-reckoned (VALID set, TRACKED clear)
  uint64_t relation_tracked;            ///< at least one TRACKED bit set
  /// [2026-09-16] The three causes folded into `nonfinite_results_rejected`,
  /// mirrored out so the live validity gate can be argued about on evidence.
  uint64_t result_state_out_of_range;
  uint64_t result_pose_nonfinite;
  uint64_t result_timestamp_unconvertible;
  /// Microseconds spent cloning keyframes for initialisation attempts.
  uint64_t init_mirror_us;
  /// Frames whose estimator met one of VINS-Mono's published divergence
  /// criteria, and the union of which criteria ever fired.
  uint64_t divergence_frames;
  uint64_t divergence_flags_seen;
  /// Longest unbroken stretch the divergence condition held, and how many
  /// frames it held past ORB-SLAM3's inertial time_recently_lost of 5 s.
  uint64_t divergence_longest_ms;
  uint64_t divergence_sustained_frames;
  uint64_t result_queue_capacity;
  uint64_t result_queue_peak;
  xrslam_bench_counters_t counters;
} xrslam_bench_snapshot_t;

int xrslam_bench_backend_available(void);

xrslam_bench_status_t
xrslam_bench_create(const xrslam_bench_create_options_t *options,
                    xrslam_bench_t **out_bench);

/* Live Core Motion acceleration in G. The adapter explicitly multiplies each
   axis by the pinned upstream factor -9.80665 before calling XRSLAM. */
xrslam_bench_status_t xrslam_bench_push_accelerometer_g(xrslam_bench_t *bench,
                                                        int64_t timestamp_ns,
                                                        double x_g, double y_g,
                                                        double z_g);

/* EuRoC replay acceleration is already in m/s^2 and passes through unchanged.
 */
xrslam_bench_status_t xrslam_bench_push_acceleration_mps2(xrslam_bench_t *bench,
                                                          int64_t timestamp_ns,
                                                          double x, double y,
                                                          double z);

/* Gyroscope radians/second pass through unchanged for live and replay. */
xrslam_bench_status_t xrslam_bench_push_gyroscope(xrslam_bench_t *bench,
                                                  int64_t timestamp_ns,
                                                  double x, double y, double z);

/* Only the create-time configured 8-bit grayscale geometry is accepted.
   Pixels are copied into the single preallocated slot before this returns.
   The caller must immediately call run_one_frame before another sensor push. */
xrslam_bench_status_t
xrslam_bench_push_image(xrslam_bench_t *bench, int64_t timestamp_ns,
                        const xrslam_bench_image_t *image);

/* Upstream's own live conversion, transcribed from
   xrslam-ios/visualizer/src/XRSLAM_iOS.mm processBuffer:

       cv::Mat raw_image = cv::Mat(h, w, CV_8UC4, baseAddress, pixelPerRow);
       cv::cvtColor(raw_image, cvimage, cv::COLOR_BGRA2GRAY);

   The official iOS app captures 32BGRA and reaches gray this way. This bench's
   live transport captures 420f and takes the ISP's luma plane directly, which
   is a different grayscale: one is OpenCV's BT.601 weights over the ISP's RGB,
   the other is the ISP's own Y. Wiring the official path needs upstream's two
   lines rather than a second implementation of them, so they live here.

   `gray_out` must hold height*gray_bytes_per_row bytes. Returns
   XRSLAM_BENCH_INVALID_ARGUMENT on a null pointer or non-positive geometry. */
xrslam_bench_status_t xrslam_bench_bgra_to_gray(
    const unsigned char *bgra, int width, int height, int bgra_bytes_per_row,
    unsigned char *gray_out, int gray_bytes_per_row);

/* Exact pinned official EuRoC PC-reader image path. OpenCV 4.0.1 decodes the
   source PNG with IMREAD_UNCHANGED, applies cv::undistort using the frozen
   MH_01 cam0 float K/D values, and converts to gray only if still multichannel.
   Live iPhone frames must never enter this function. */
xrslam_bench_status_t
xrslam_bench_push_euroc_image_file(xrslam_bench_t *bench,
                                   int64_t timestamp_ns,
                                   const char *image_path);

/* Advances the frozen core through the pending image, then executes the
   official PC-evaluation XRSLAMRunOneFrame/GetResult(BODY_POSE,STATE)
   sequence. BODY_POSE matches Basalt's IMU state and EuRoC ground truth. */
xrslam_bench_status_t xrslam_bench_run_one_frame(xrslam_bench_t *bench);

xrslam_bench_status_t
xrslam_bench_poll_result(xrslam_bench_t *bench,
                         xrslam_bench_frame_result_t *out_result);

/* Existing ABI symbol retained. The complete stock C API provides no separate
   propagated-pose query, so this adapter returns BACKEND_UNAVAILABLE instead of
   synthesizing a pose or using an unofficial accessor. Use official positive-t
   TRACKING_SUCCESS BODY results through poll_result. */
xrslam_bench_status_t
xrslam_bench_query_pose(xrslam_bench_t *bench,
                        xrslam_bench_frame_result_t *out_result);

xrslam_bench_status_t xrslam_bench_seal_inputs(xrslam_bench_t *bench);
xrslam_bench_status_t xrslam_bench_drain(xrslam_bench_t *bench);

/* Idempotent official shutdown order: seal ingress, run the accepted pending
   image if present, then call XRSLAMDestroy exactly once. */
xrslam_bench_status_t xrslam_bench_stop(xrslam_bench_t *bench);

xrslam_bench_status_t
xrslam_bench_get_snapshot(xrslam_bench_t *bench,
                          xrslam_bench_snapshot_t *out_snapshot);

/* Safe on null. A live handle follows the same seal/drain/stop path first. */
void xrslam_bench_destroy(xrslam_bench_t *bench);

/* Additive original factory/default-Async component route. The caller runs this
   blocking call on its existing owner thread. Output must not already exist.
   Status/cancel are observer-only and never call the estimator. Destroy during
   replay is rejected (handle retained); call destroy after replay returns. */
typedef struct xrslam_bench_original_io_snapshot {
  size_t struct_size;
  int32_t active, cancel_requested, natural_eof, incomplete, failed;
  uint64_t camera, gyroscope, accelerometer, frames, poses, again;
} xrslam_bench_original_io_snapshot_t;
xrslam_bench_status_t xrslam_bench_replay_original_io(
    xrslam_bench_t *bench, const char *absolute_mav0, const char *fresh_output_tum);
xrslam_bench_status_t xrslam_bench_cancel_original_io(xrslam_bench_t *bench);
xrslam_bench_status_t xrslam_bench_original_io_status(
    xrslam_bench_t *bench, xrslam_bench_original_io_snapshot_t *out);

const char *xrslam_bench_status_name(xrslam_bench_status_t status);

#ifdef __cplusplus
}
#endif

#endif
