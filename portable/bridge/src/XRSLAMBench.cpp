#include <cstdio>
// The per-frame cost at the production resolution is dominated by two OpenCV
// calls inside upstream's OpenCvImage::preprocess -- CLAHE over the whole frame
// and buildOpticalFlowPyramid with derivatives. Both go through
// cv::parallel_for_, so how many threads OpenCV believes it has decides whether
// that work uses one core or all of them. Nothing in this tree ever set or
// reported it.
#include <opencv2/core.hpp>
#include <string>
#include <sstream>
#include <cstdlib>
#include <fstream>
#include "XRSLAMBench.h"
#include "CommonCoreOwner.h"
#include <vector> // Required by the exact official XRSLAM.h declarations.
#include "XRSLAMBenchTesting.hpp"
#include "XRSLAMOriginalIOReplay.h"
#include "HeadlessEOFWait.h"

#include <algorithm>
#include <array>
#include <deque>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <mutex>
#include <new>
#include <utility>
#include <vector>

#if defined(PW_XRSLAM_CORE_LINKED) && PW_XRSLAM_CORE_LINKED
#include "XRSLAMManager.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#endif

/* Frozen/XRSLAM.h is the exact pinned official header. These asserts pin
 * the unchanged arm64 ABI consumed by the shared PC-profile archive. */
static_assert(sizeof(XRSLAMSensorType) == 4);
static_assert(sizeof(XRSLAMResultType) == 4);
static_assert(sizeof(XRSLAMState) == 4);
static_assert(sizeof(XRSLAMImageExtension) == 32);
static_assert(sizeof(XRSLAMImage) == 40);
static_assert(offsetof(XRSLAMImage, data) == 0);
static_assert(offsetof(XRSLAMImage, timeStamp) == 8);
static_assert(offsetof(XRSLAMImage, stride) == 16);
static_assert(offsetof(XRSLAMImage, camera_id) == 20);
static_assert(offsetof(XRSLAMImage, channel) == 24);
static_assert(offsetof(XRSLAMImage, ext) == 32);
static_assert(sizeof(XRSLAMDepthImage) == 24);
static_assert(sizeof(XRSLAMAcceleration) == 32);
static_assert(offsetof(XRSLAMAcceleration, timestamp) == 24);
static_assert(sizeof(XRSLAMGyroscope) == 32);
static_assert(offsetof(XRSLAMGyroscope, timestamp) == 24);
static_assert(sizeof(XRSLAMGravity) == 32);
static_assert(sizeof(XRSLAMRotationVector) == 40);
static_assert(sizeof(XRSLAMPose) == 64);
static_assert(offsetof(XRSLAMPose, quaternion) == 0);
static_assert(offsetof(XRSLAMPose, translation) == 32);
static_assert(offsetof(XRSLAMPose, timestamp) == 56);
static_assert(sizeof(XRSLAMIntrinsics) == 32);
static_assert(sizeof(XRSLAMBias) == 24);
static_assert(sizeof(XRSLAMIMUBias) == 48);
static_assert(XRSLAM_SENSOR_CAMERA == 0);

extern "C" xrslam_bench_status_t xrslam_bench_bgra_to_gray(
    const unsigned char *bgra, int width, int height, int bgra_bytes_per_row,
    unsigned char *gray_out, int gray_bytes_per_row) {
  if (bgra == nullptr || gray_out == nullptr || width <= 0 || height <= 0 ||
      bgra_bytes_per_row < width * 4 || gray_bytes_per_row < width) {
    return XRSLAM_BENCH_INVALID_ARGUMENT;
  }
  /* Upstream, XRSLAM_iOS.mm processBuffer, verbatim apart from writing into a
     caller-owned destination instead of a member Mat. */
  cv::Mat raw_image(height, width, CV_8UC4, const_cast<unsigned char *>(bgra),
                    static_cast<size_t>(bgra_bytes_per_row));
  cv::Mat cvimage(height, width, CV_8UC1, gray_out,
                  static_cast<size_t>(gray_bytes_per_row));
  cv::cvtColor(raw_image, cvimage, cv::COLOR_BGRA2GRAY);
  /* cvtColor reallocates when the destination does not already match; if it
     did, the caller's buffer was not written and saying OK would be a lie. */
  if (cvimage.data != gray_out) return XRSLAM_BENCH_INTERNAL_ERROR;
  return XRSLAM_BENCH_OK;
}

/* Reads a whole config file. XRSLAMCreate takes the text, not the path. */
static std::string read_file(const char *path) {
  if (path == nullptr || path[0] == '\0') return {};
  std::ifstream stream(path, std::ios::in | std::ios::binary);
  if (!stream) return {};
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}
static_assert(XRSLAM_SENSOR_ACCELERATION == 2);
static_assert(XRSLAM_SENSOR_GYROSCOPE == 3);
static_assert(XRSLAM_RESULT_BODY_POSE == 0);
static_assert(XRSLAM_RESULT_STATE == 2);
static_assert(XRSLAM_STATE_INITIALIZING == 0);
static_assert(XRSLAM_STATE_TRACKING_SUCCESS == 1);
static_assert(XRSLAM_STATE_TRACKING_FAIL == 2);

/* Stock has exactly five implemented XRSLAM C entry points. No optional
 * fork accessor, weak algorithm fallback, or derived health rule is used. */

namespace {

constexpr double kSecondsPerNanosecond = 1e-9;
constexpr double kCoreMotionGravityScale = -9.80665;

enum class EventKind { Acceleration, Gyroscope, Image };

constexpr uint64_t kImageSlotCapacity = 1;
/* Only the wrapper's single image handoff slot is observable. Official worker
 * completion/backlog is unavailable; do not infer it from BODY timestamps. */
constexpr size_t kResultQueueCapacity = 16;

bool finite3(double x, double y, double z) {
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

bool pose_is_finite(const XRSLAMPose &pose) {
  if (!std::isfinite(pose.timestamp))
    return false;
  for (double value : pose.quaternion) {
    if (!std::isfinite(value))
      return false;
  }
  for (double value : pose.translation) {
    if (!std::isfinite(value))
      return false;
  }
  return true;
}

bool seconds_to_nanoseconds(double seconds, int64_t *out) {
  if (out == nullptr || !std::isfinite(seconds))
    return false;
  const long double nanoseconds =
      static_cast<long double>(seconds) * 1000000000.0L;
  if (nanoseconds <
          static_cast<long double>(std::numeric_limits<int64_t>::min()) ||
      nanoseconds >
          static_cast<long double>(std::numeric_limits<int64_t>::max())) {
    return false;
  }
  *out = static_cast<int64_t>(std::llround(nanoseconds));
  return true;
}

bool api_is_complete(const xrslam_bench_upstream_api *api) {
  return api != nullptr && api->create != nullptr &&
         api->push_sensor_data != nullptr && api->run_one_frame != nullptr &&
         api->get_result != nullptr && api->destroy != nullptr;
}

#if defined(PW_XRSLAM_CORE_LINKED) && PW_XRSLAM_CORE_LINKED
const xrslam_bench_upstream_api kProductionAPI = {
    XRSLAMCreate, XRSLAMPushSensorData, XRSLAMRunOneFrame, XRSLAMGetResult,
    XRSLAMDestroy};
#endif

std::mutex g_lifecycle_mutex;
struct xrslam_bench *g_active_bench = nullptr;

} // namespace

struct xrslam_bench {
  std::mutex mutex;
  OriginalIOState original_io;
  bool original_io_destroying = false;
  xrslam_bench_upstream_api api{};
  void *upstream_config = nullptr;
  xrslam_bench_phase_t phase = XRSLAM_BENCH_PHASE_RUNNING;
  bool upstream_destroyed = false;
  pw_common_core::Token owner_token = 0;
  bool has_last_event = false;
  bool has_last_accel = false;
  bool has_last_gyro = false;
  bool has_last_image = false;
  int64_t last_event_timestamp_ns = std::numeric_limits<int64_t>::min();
  int64_t last_accel_timestamp_ns = std::numeric_limits<int64_t>::min();
  int64_t last_gyro_timestamp_ns = std::numeric_limits<int64_t>::min();
  int64_t last_image_timestamp_ns = std::numeric_limits<int64_t>::min();
  uint32_t image_width = 0;
  uint32_t image_height = 0;
  std::vector<uint8_t> image_storage;
  bool has_pending_image = false;
  int64_t pending_image_timestamp_ns = 0;
  uint64_t image_slot_peak = 0;
  std::array<xrslam_bench_frame_result_t, kResultQueueCapacity> results{};
  size_t result_head = 0;
  size_t result_count = 0;
  uint64_t result_queue_peak = 0;
  uint64_t engine_backlog_peak = 0;
  xrslam_bench_counters_t counters{};
};

namespace {

void record_offered_locked(xrslam_bench *bench, EventKind kind) {
  ++bench->counters.events_offered;
  switch (kind) {
  case EventKind::Acceleration:
    ++bench->counters.accel_offered;
    break;
  case EventKind::Gyroscope:
    ++bench->counters.gyro_offered;
    break;
  case EventKind::Image:
    ++bench->counters.images_offered;
    break;
  }
}

void record_rejected_sealed_locked(xrslam_bench *bench, EventKind kind) {
  ++bench->counters.events_rejected_sealed;
  switch (kind) {
  case EventKind::Acceleration:
    ++bench->counters.accel_rejected_sealed;
    break;
  case EventKind::Gyroscope:
    ++bench->counters.gyro_rejected_sealed;
    break;
  case EventKind::Image:
    ++bench->counters.images_rejected_sealed;
    break;
  }
}

void record_rejected_timestamp_locked(xrslam_bench *bench, EventKind kind) {
  ++bench->counters.events_rejected_timestamp;
  switch (kind) {
  case EventKind::Acceleration:
    ++bench->counters.accel_rejected_timestamp;
    break;
  case EventKind::Gyroscope:
    ++bench->counters.gyro_rejected_timestamp;
    break;
  case EventKind::Image:
    ++bench->counters.images_rejected_timestamp;
    break;
  }
}

void record_rejected_queue_full_locked(xrslam_bench *bench, EventKind kind) {
  ++bench->counters.events_rejected_queue_full;
  switch (kind) {
  case EventKind::Acceleration:
    ++bench->counters.accel_rejected_queue_full;
    break;
  case EventKind::Gyroscope:
    ++bench->counters.gyro_rejected_queue_full;
    break;
  case EventKind::Image:
    ++bench->counters.images_rejected_queue_full;
    break;
  }
}

void release_singleton_ownership(xrslam_bench *bench) {
  std::lock_guard<std::mutex> lifecycle_lock(g_lifecycle_mutex);
  if (g_active_bench == bench &&
      pw_common_core::Release(pw_common_core::Mode::Bench, bench->owner_token)) {
    bench->owner_token = 0;
    g_active_bench = nullptr;
  }
}

xrslam_bench_status_t validate_timestamp_locked(xrslam_bench *bench,
                                                EventKind kind,
                                                int64_t timestamp_ns) {
  bool has_stream_timestamp = false;
  int64_t stream_timestamp = std::numeric_limits<int64_t>::min();
  switch (kind) {
  case EventKind::Acceleration:
    has_stream_timestamp = bench->has_last_accel;
    stream_timestamp = bench->last_accel_timestamp_ns;
    break;
  case EventKind::Gyroscope:
    has_stream_timestamp = bench->has_last_gyro;
    stream_timestamp = bench->last_gyro_timestamp_ns;
    break;
  case EventKind::Image:
    has_stream_timestamp = bench->has_last_image;
    stream_timestamp = bench->last_image_timestamp_ns;
    break;
  }
  if (timestamp_ns < 0 ||
      (has_stream_timestamp && timestamp_ns <= stream_timestamp)) {
    record_rejected_timestamp_locked(bench, kind);
    return XRSLAM_BENCH_TIMESTAMP_REGRESSION;
  }
  return XRSLAM_BENCH_OK;
}

void record_timestamp_locked(xrslam_bench *bench, EventKind kind,
                             int64_t timestamp_ns) {
  bench->has_last_event = true;
  bench->last_event_timestamp_ns = timestamp_ns;
  switch (kind) {
  case EventKind::Acceleration:
    bench->has_last_accel = true;
    bench->last_accel_timestamp_ns = timestamp_ns;
    ++bench->counters.accel_accepted;
    break;
  case EventKind::Gyroscope:
    bench->has_last_gyro = true;
    bench->last_gyro_timestamp_ns = timestamp_ns;
    ++bench->counters.gyro_accepted;
    break;
  case EventKind::Image:
    bench->has_last_image = true;
    bench->last_image_timestamp_ns = timestamp_ns;
    ++bench->counters.images_accepted;
    break;
  }
  ++bench->counters.events_accepted;
}

xrslam_bench_status_t enqueue_vector(xrslam_bench *bench, EventKind kind,
                                     int64_t timestamp_ns, double x, double y,
                                     double z) {
  if (bench == nullptr || !finite3(x, y, z)) {
    return XRSLAM_BENCH_INVALID_ARGUMENT;
  }
  std::lock_guard<std::mutex> lock(bench->mutex);
  if (bench->original_io.active.load()) return XRSLAM_BENCH_INVALID_STATE;
  record_offered_locked(bench, kind);
  if (bench->phase != XRSLAM_BENCH_PHASE_RUNNING) {
    record_rejected_sealed_locked(bench, kind);
    return XRSLAM_BENCH_INPUT_SEALED;
  }
  /* The public contract requires push_image -> run_one_frame to be atomic on
     the caller's serial queue. Refuse interleaving instead of silently
     reordering an IMU callback ahead of an already accepted camera event. */
  if (bench->has_pending_image) {
    record_rejected_queue_full_locked(bench, kind);
    return XRSLAM_BENCH_QUEUE_FULL;
  }
  const xrslam_bench_status_t timestamp_status =
      validate_timestamp_locked(bench, kind, timestamp_ns);
  if (timestamp_status != XRSLAM_BENCH_OK)
    return timestamp_status;
  try {
    const double timestamp_seconds =
        static_cast<double>(timestamp_ns) * kSecondsPerNanosecond;
    if (kind == EventKind::Acceleration) {
      XRSLAMAcceleration acceleration{{x, y, z}, timestamp_seconds};
      bench->api.push_sensor_data(XRSLAM_SENSOR_ACCELERATION, &acceleration);
      ++bench->counters.accel_processed;
    } else if (kind == EventKind::Gyroscope) {
      XRSLAMGyroscope gyroscope{{x, y, z}, timestamp_seconds};
      bench->api.push_sensor_data(XRSLAM_SENSOR_GYROSCOPE, &gyroscope);
      ++bench->counters.gyro_processed;
    } else {
      return XRSLAM_BENCH_INTERNAL_ERROR;
    }
  } catch (...) {
    bench->phase = XRSLAM_BENCH_PHASE_FAILED;
    return XRSLAM_BENCH_INTERNAL_ERROR;
  }
  record_timestamp_locked(bench, kind, timestamp_ns);
  ++bench->counters.events_processed;
  return XRSLAM_BENCH_OK;
}

xrslam_bench_status_t run_next_frame_locked(xrslam_bench *bench) {
  if (!bench->has_pending_image)
    return XRSLAM_BENCH_NO_FRAME;
  if (bench->result_count == kResultQueueCapacity) {
    ++bench->counters.results_rejected_queue_full;
    return XRSLAM_BENCH_QUEUE_FULL;
  }
  try {
    XRSLAMImage image{};
    image.data = bench->image_storage.data();
    image.timeStamp = static_cast<double>(bench->pending_image_timestamp_ns) *
                      kSecondsPerNanosecond;
    image.stride = static_cast<int>(bench->image_width);
    image.camera_id = 0;
    image.channel = 1;
    image.ext = nullptr;
    bench->api.push_sensor_data(XRSLAM_SENSOR_CAMERA, &image);
    const int64_t input_timestamp_ns = bench->pending_image_timestamp_ns;
    bench->has_pending_image = false;
    ++bench->counters.events_processed;
    ++bench->counters.images_processed;

    /* Pinned official xrslam-pc/player/src/main.cpp:147-158:
       push every camera; Run only after both IMU types; STATE first; read BODY
       only for TRACKING_SUCCESS; output only when pose.timestamp > 0. */
    if (!bench->has_last_accel || !bench->has_last_gyro)
      return XRSLAM_BENCH_OK;
    bench->api.run_one_frame();
    ++bench->counters.frames_run;
    XRSLAMState state = XRSLAM_STATE_INITIALIZING;
    bench->api.get_result(XRSLAM_RESULT_STATE, &state);
    if (state != XRSLAM_STATE_TRACKING_SUCCESS)
      return XRSLAM_BENCH_OK;
    XRSLAMPose pose{};
    bench->api.get_result(XRSLAM_RESULT_BODY_POSE, &pose);
    if (!(pose.timestamp > 0))
      return XRSLAM_BENCH_OK;

    xrslam_bench_frame_result_t result{};
    /* -1 means the stock API has no divergence metadata. It is not a health
       verdict; stock consumers must treat this field as unavailable. */
    result.divergence_flags = -1;
    result.input_timestamp_ns = input_timestamp_ns;
    result.pose_timestamp_seconds = pose.timestamp;
    if (!seconds_to_nanoseconds(pose.timestamp, &result.pose_timestamp_ns))
      ++bench->counters.result_timestamp_unconvertible;
    if (!pose_is_finite(pose))
      ++bench->counters.result_pose_nonfinite;
    /* Diagnostic counts never filter or alter the official positive-t pose.
       Raw double t, xyzw and xyz remain exactly the returned fields. */
    std::copy(std::begin(pose.quaternion), std::end(pose.quaternion),
              result.quaternion_xyzw);
    std::copy(std::begin(pose.translation), std::end(pose.translation),
              result.translation_xyz);
    result.state = static_cast<int32_t>(state);
    const size_t result_tail =
        (bench->result_head + bench->result_count) % kResultQueueCapacity;
    bench->results[result_tail] = result;
    ++bench->result_count;
    bench->result_queue_peak =
        std::max<uint64_t>(bench->result_queue_peak, bench->result_count);
    ++bench->counters.results_produced;
    return XRSLAM_BENCH_OK;
  } catch (...) {
    bench->phase = XRSLAM_BENCH_PHASE_FAILED;
    return XRSLAM_BENCH_INTERNAL_ERROR;
  }
}

xrslam_bench_status_t seal_locked(xrslam_bench *bench) {
  switch (bench->phase) {
  case XRSLAM_BENCH_PHASE_RUNNING:
    bench->phase = XRSLAM_BENCH_PHASE_SEALED;
    return XRSLAM_BENCH_OK;
  case XRSLAM_BENCH_PHASE_SEALED:
  case XRSLAM_BENCH_PHASE_DRAINED:
  case XRSLAM_BENCH_PHASE_STOPPED:
    return XRSLAM_BENCH_OK;
  case XRSLAM_BENCH_PHASE_FAILED:
    return XRSLAM_BENCH_INVALID_STATE;
  }
  return XRSLAM_BENCH_INTERNAL_ERROR;
}

xrslam_bench_status_t drain_locked(xrslam_bench *bench) {
  if (bench->phase == XRSLAM_BENCH_PHASE_STOPPED ||
      bench->phase == XRSLAM_BENCH_PHASE_DRAINED) {
    return XRSLAM_BENCH_OK;
  }
  if (bench->phase == XRSLAM_BENCH_PHASE_FAILED) {
    return XRSLAM_BENCH_INVALID_STATE;
  }
  const xrslam_bench_status_t seal_status = seal_locked(bench);
  if (seal_status != XRSLAM_BENCH_OK)
    return seal_status;

  while (bench->has_pending_image) {
    const xrslam_bench_status_t frame_status = run_next_frame_locked(bench);
    if (frame_status != XRSLAM_BENCH_OK)
      return frame_status;
  }
  bench->phase = XRSLAM_BENCH_PHASE_DRAINED;
  return XRSLAM_BENCH_OK;
}

xrslam_bench_status_t
create_with_api(const xrslam_bench_create_options_t *options,
                const xrslam_bench_upstream_api *api,
                xrslam_bench_t **out_bench) {
  if (out_bench == nullptr)
    return XRSLAM_BENCH_INVALID_ARGUMENT;
  *out_bench = nullptr;
  if (options == nullptr ||
      options->struct_size != sizeof(xrslam_bench_create_options_t) ||
      options->slam_config_path == nullptr ||
      options->device_config_path == nullptr ||
      options->slam_config_path[0] == '\0' ||
      options->device_config_path[0] == '\0' || options->image_width == 0 ||
      options->image_height == 0 || !api_is_complete(api)) {
    return XRSLAM_BENCH_INVALID_ARGUMENT;
  }

  std::lock_guard<std::mutex> lifecycle_lock(g_lifecycle_mutex);
  if (g_active_bench != nullptr)
    return XRSLAM_BENCH_INVALID_STATE;

  pw_common_core::Reservation reservation(pw_common_core::Mode::Bench);
  if (!reservation.token()) return XRSLAM_BENCH_INVALID_STATE;

  xrslam_bench *bench = new (std::nothrow) xrslam_bench();
  if (bench == nullptr)
    return XRSLAM_BENCH_INTERNAL_ERROR;
  bench->api = *api;
  bench->image_width = options->image_width;
  bench->image_height = options->image_height;
  bool upstream_create_started = false;
  try {
    const size_t image_size = static_cast<size_t>(bench->image_width) *
                              static_cast<size_t>(bench->image_height);
    if (image_size / bench->image_width != bench->image_height) {
      delete bench;
      return XRSLAM_BENCH_INVALID_ARGUMENT;
    }
    bench->image_storage.resize(image_size);
    /* This separate adapter links the exact shared PC-profile archive.
       Its explicit common text-transport YamlConfig constructor uses YAML::Load, so filesystem
       arguments at the existing bench ABI are read and passed as YAML text.
       No environment switch or estimator configuration rewrite is applied. */
    const bool config_as_text = true;
    const std::string slam_text =
        config_as_text ? read_file(options->slam_config_path) : std::string();
    const std::string device_text =
        config_as_text ? read_file(options->device_config_path) : std::string();
    fprintf(stderr, "[xrslam-trace] create() mode=%s slam=%s device=%s\n",
            config_as_text ? "contents" : "paths",
            options->slam_config_path, options->device_config_path);
    fflush(stderr);
    fprintf(stderr, "[xrslam-common-core] profile=COMMON_CORE_ONLY_shared_original_PC revision=4beb1a942f33da9afbfae2d70e2c641cfc2bb675 canonical_archive_sha256=7b2ca572941d2895a123d6b23b18b9397fe7cd1cb3278716350b4489d2eab521 core_source_revision=R06 official_PARSAC_sha256=0d63d9381542194008455a3983fbb2f11e5c9382dc960b9b5da58f8ef1a43804 GetResultState_source_sha256=0423b8383ef9bc41ef347609f006bca1375f4b092511a9efde6a61089d24d282 headless_eof_adapter=01 core_XRSLAM_IOS=false core_threading=ON state_publication_adapter=atomic lifecycle_adapter=owned_worker_queue_active_observer config_transport=explicit_common_yaml_text cached_output_state_guard=none_official_STATE_mapping source_classification=original_component_headless_not_GUI_or_paper_parity whole_engine_pristine=false locator_enabled=false cross_platform_runtime_verified=false extended-diagnostics=unavailable\n");
    upstream_create_started = true;
    const int created = bench->api.create(
        config_as_text ? slam_text.c_str() : options->slam_config_path,
        config_as_text ? device_text.c_str() : options->device_config_path, "",
        "xrslam-vio-bench", &bench->upstream_config);
    fprintf(stderr, "[xrslam-trace] create() -> %d config=%p\n",
            created, bench->upstream_config);
    fflush(stderr);
    if (created != 1 || bench->upstream_config == nullptr) {
      if (created == 1) {
        try {
          fprintf(stderr, "[xrslam-trace] destroy() called\n"); fflush(stderr);
        bench->api.destroy();
        } catch (...) {
          // Creation succeeded but teardown did not: poison the reservation.
          (void)reservation.Detach();
        }
      }
      delete bench;
      return XRSLAM_BENCH_CREATE_FAILED;
    }
  } catch (...) {
    // An exception from official Create has unknown singleton ownership.
    if (upstream_create_started) (void)reservation.Detach();
    delete bench;
    return XRSLAM_BENCH_CREATE_FAILED;
  }

  bench->owner_token = reservation.Detach();
  g_active_bench = bench;
  *out_bench = bench;
  return XRSLAM_BENCH_OK;
}

} // namespace

extern "C" int xrslam_bench_backend_available(void) {
#if defined(PW_XRSLAM_CORE_LINKED) && PW_XRSLAM_CORE_LINKED
  return 1;
#else
  return 0;
#endif
}

extern "C" xrslam_bench_status_t
xrslam_bench_create(const xrslam_bench_create_options_t *options,
                    xrslam_bench_t **out_bench) {
#if defined(PW_XRSLAM_CORE_LINKED) && PW_XRSLAM_CORE_LINKED
  return create_with_api(options, &kProductionAPI, out_bench);
#else
  if (out_bench == nullptr)
    return XRSLAM_BENCH_INVALID_ARGUMENT;
  *out_bench = nullptr;
  (void)options;
  return XRSLAM_BENCH_BACKEND_UNAVAILABLE;
#endif
}

xrslam_bench_status_t xrslam_bench_create_with_api_for_testing(
    const xrslam_bench_create_options_t *options,
    const xrslam_bench_upstream_api *api, xrslam_bench_t **out_bench) {
  return create_with_api(options, api, out_bench);
}

extern "C" xrslam_bench_status_t
xrslam_bench_push_accelerometer_g(xrslam_bench_t *bench, int64_t timestamp_ns,
                                  double x_g, double y_g, double z_g) {
  return enqueue_vector(bench, EventKind::Acceleration, timestamp_ns,
                        x_g * kCoreMotionGravityScale,
                        y_g * kCoreMotionGravityScale,
                        z_g * kCoreMotionGravityScale);
}

extern "C" xrslam_bench_status_t
xrslam_bench_push_acceleration_mps2(xrslam_bench_t *bench, int64_t timestamp_ns,
                                    double x, double y, double z) {
  return enqueue_vector(bench, EventKind::Acceleration, timestamp_ns, x, y, z);
}

extern "C" xrslam_bench_status_t
xrslam_bench_push_gyroscope(xrslam_bench_t *bench, int64_t timestamp_ns,
                            double x, double y, double z) {
  return enqueue_vector(bench, EventKind::Gyroscope, timestamp_ns, x, y, z);
}

extern "C" xrslam_bench_status_t
xrslam_bench_push_image(xrslam_bench_t *bench, int64_t timestamp_ns,
                        const xrslam_bench_image_t *image) {
  if (bench == nullptr || image == nullptr || image->gray_pixels == nullptr ||
      image->width != bench->image_width ||
      image->height != bench->image_height ||
      image->bytes_per_row < bench->image_width) {
    return XRSLAM_BENCH_INVALID_ARGUMENT;
  }

  std::lock_guard<std::mutex> lock(bench->mutex);
  if (bench->original_io.active.load()) return XRSLAM_BENCH_INVALID_STATE;
  record_offered_locked(bench, EventKind::Image);
  if (bench->phase != XRSLAM_BENCH_PHASE_RUNNING) {
    record_rejected_sealed_locked(bench, EventKind::Image);
    return XRSLAM_BENCH_INPUT_SEALED;
  }
  if (bench->has_pending_image) {
    record_rejected_queue_full_locked(bench, EventKind::Image);
    return XRSLAM_BENCH_QUEUE_FULL;
  }
  const xrslam_bench_status_t timestamp_status =
      validate_timestamp_locked(bench, EventKind::Image, timestamp_ns);
  if (timestamp_status != XRSLAM_BENCH_OK)
    return timestamp_status;

  for (uint32_t row = 0; row < bench->image_height; ++row) {
    std::memcpy(bench->image_storage.data() +
                    static_cast<size_t>(row) * bench->image_width,
                image->gray_pixels +
                    static_cast<size_t>(row) * image->bytes_per_row,
                bench->image_width);
  }
  bench->pending_image_timestamp_ns = timestamp_ns;
  bench->has_pending_image = true;
  bench->image_slot_peak = 1;
  record_timestamp_locked(bench, EventKind::Image, timestamp_ns);
  bench->counters.image_bytes_copied +=
      static_cast<uint64_t>(bench->image_width) * bench->image_height;
  return XRSLAM_BENCH_OK;
}

extern "C" xrslam_bench_status_t
xrslam_bench_push_euroc_image_file(xrslam_bench_t *bench,
                                   int64_t timestamp_ns,
                                   const char *image_path) {
  if (bench == nullptr || image_path == nullptr || image_path[0] == '\0')
    return XRSLAM_BENCH_INVALID_ARGUMENT;

#if defined(PW_XRSLAM_CORE_LINKED) && PW_XRSLAM_CORE_LINKED
  try {
    cv::Mat img_distorted = cv::imread(image_path, cv::IMREAD_UNCHANGED);
    if (img_distorted.empty()) {
      std::lock_guard<std::mutex> lock(bench->mutex);
  if (bench->original_io.active.load()) return XRSLAM_BENCH_INVALID_STATE;
      ++bench->counters.replay_image_decode_failures;
      return XRSLAM_BENCH_INVALID_ARGUMENT;
    }
    {
      std::lock_guard<std::mutex> lock(bench->mutex);
  if (bench->original_io.active.load()) return XRSLAM_BENCH_INVALID_STATE;
      ++bench->counters.replay_images_decoded;
    }

    // Byte-for-byte parameter and float-matrix construction semantics from
    // pinned upstream 4beb1a9 EurocDatasetReader::read_image().
    cv::Mat distortion =
        (cv::Mat_<float>(4, 1) << -0.28340811f, 0.07395907f,
         0.00019359f, 1.76187114e-05f);
    cv::Mat intrinsic =
        (cv::Mat_<float>(3, 3) << 458.654f, 0.0f, 367.215f, 0.0f,
         457.296f, 248.375f, 0.0f, 0.0f, 1.0f);
    cv::Mat img;
    cv::undistort(img_distorted, img, intrinsic, distortion);
    if (img.channels() != 1)
      cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    if (img.empty() || !img.isContinuous() || img.elemSize() != 1) {
      std::lock_guard<std::mutex> lock(bench->mutex);
  if (bench->original_io.active.load()) return XRSLAM_BENCH_INVALID_STATE;
      ++bench->counters.replay_image_preprocess_failures;
      return XRSLAM_BENCH_INTERNAL_ERROR;
    }

    xrslam_bench_image_t image{};
    image.gray_pixels = img.ptr<uint8_t>();
    image.width = static_cast<uint32_t>(img.cols);
    image.height = static_cast<uint32_t>(img.rows);
    image.bytes_per_row = static_cast<uint32_t>(img.step[0]);
    const xrslam_bench_status_t status =
        xrslam_bench_push_image(bench, timestamp_ns, &image);
    if (status == XRSLAM_BENCH_OK) {
      std::lock_guard<std::mutex> lock(bench->mutex);
  if (bench->original_io.active.load()) return XRSLAM_BENCH_INVALID_STATE;
      ++bench->counters.replay_images_undistorted;
    }
    return status;
  } catch (...) {
    std::lock_guard<std::mutex> lock(bench->mutex);
  if (bench->original_io.active.load()) return XRSLAM_BENCH_INVALID_STATE;
    ++bench->counters.replay_image_preprocess_failures;
    return XRSLAM_BENCH_INTERNAL_ERROR;
  }
#else
  (void)timestamp_ns;
  return XRSLAM_BENCH_BACKEND_UNAVAILABLE;
#endif
}

extern "C" xrslam_bench_status_t
xrslam_bench_run_one_frame(xrslam_bench_t *bench) {
  if (bench == nullptr)
    return XRSLAM_BENCH_INVALID_ARGUMENT;
  std::lock_guard<std::mutex> lock(bench->mutex);
  if (bench->original_io.active.load()) return XRSLAM_BENCH_INVALID_STATE;
  if (bench->phase == XRSLAM_BENCH_PHASE_DRAINED ||
      bench->phase == XRSLAM_BENCH_PHASE_STOPPED ||
      bench->phase == XRSLAM_BENCH_PHASE_FAILED) {
    return XRSLAM_BENCH_INVALID_STATE;
  }
  return run_next_frame_locked(bench);
}

extern "C" xrslam_bench_status_t
xrslam_bench_poll_result(xrslam_bench_t *bench,
                         xrslam_bench_frame_result_t *out_result) {
  if (bench == nullptr || out_result == nullptr) {
    return XRSLAM_BENCH_INVALID_ARGUMENT;
  }
  *out_result = {};
  std::lock_guard<std::mutex> lock(bench->mutex);
  if (bench->original_io.active.load()) return XRSLAM_BENCH_INVALID_STATE;
  if (bench->result_count == 0) {
    return bench->phase == XRSLAM_BENCH_PHASE_DRAINED ||
                   bench->phase == XRSLAM_BENCH_PHASE_STOPPED
               ? XRSLAM_BENCH_END_OF_STREAM
               : XRSLAM_BENCH_NO_OUTPUT;
  }
  *out_result = bench->results[bench->result_head];
  bench->result_head = (bench->result_head + 1) % kResultQueueCapacity;
  --bench->result_count;
  ++bench->counters.results_polled;
  return XRSLAM_BENCH_OK;
}

extern "C" xrslam_bench_status_t
xrslam_bench_query_pose(xrslam_bench_t *bench,
                        xrslam_bench_frame_result_t *out_result) {
  if (bench == nullptr || out_result == nullptr)
    return XRSLAM_BENCH_INVALID_ARGUMENT;
  *out_result = {};
  /* This existing ABI symbol is retained, but stock supplies no separate
     propagated-pose C API. Never synthesize a pose or call a fork accessor. */
  return XRSLAM_BENCH_BACKEND_UNAVAILABLE;
}

extern "C" xrslam_bench_status_t
xrslam_bench_seal_inputs(xrslam_bench_t *bench) {
  if (bench == nullptr)
    return XRSLAM_BENCH_INVALID_ARGUMENT;
  std::lock_guard<std::mutex> lock(bench->mutex);
  if (bench->original_io.active.load()) return XRSLAM_BENCH_INVALID_STATE;
  return seal_locked(bench);
}

extern "C" xrslam_bench_status_t xrslam_bench_drain(xrslam_bench_t *bench) {
  if (bench == nullptr)
    return XRSLAM_BENCH_INVALID_ARGUMENT;
  std::lock_guard<std::mutex> lock(bench->mutex);
  if (bench->original_io.active.load()) return XRSLAM_BENCH_INVALID_STATE;
  return drain_locked(bench);
}

extern "C" xrslam_bench_status_t xrslam_bench_stop(xrslam_bench_t *bench) {
  if (bench == nullptr)
    return XRSLAM_BENCH_INVALID_ARGUMENT;

  xrslam_bench_status_t drain_status = XRSLAM_BENCH_OK;
  xrslam_bench_status_t destroy_status = XRSLAM_BENCH_OK;
  {
    std::lock_guard<std::mutex> lock(bench->mutex);
  if (bench->original_io.active.load()) return XRSLAM_BENCH_INVALID_STATE;
    if (bench->phase == XRSLAM_BENCH_PHASE_STOPPED) {
      return XRSLAM_BENCH_OK;
    }
    if (bench->phase != XRSLAM_BENCH_PHASE_FAILED) {
      drain_status = drain_locked(bench);
    } else {
      drain_status = XRSLAM_BENCH_INVALID_STATE;
    }
    if (!bench->upstream_destroyed) {
      if (!pw_common_core::Owns(pw_common_core::Mode::Bench, bench->owner_token))
        return XRSLAM_BENCH_INVALID_STATE;
      try {
        fprintf(stderr, "[xrslam-trace] destroy() called\n"); fflush(stderr);
        bench->api.destroy();
        ++bench->counters.upstream_destroy_calls;
      } catch (...) {
        // Failed teardown must retain exclusion; another mode cannot enter
        // a core whose worker/reader quiescence is unproven.
        return XRSLAM_BENCH_INTERNAL_ERROR;
      }
      bench->upstream_destroyed = true;
    }
    bench->phase = XRSLAM_BENCH_PHASE_STOPPED;
  }
  release_singleton_ownership(bench);
  if (drain_status != XRSLAM_BENCH_OK)
    return drain_status;
  return destroy_status;
}

extern "C" xrslam_bench_status_t
xrslam_bench_get_snapshot(xrslam_bench_t *bench,
                          xrslam_bench_snapshot_t *out_snapshot) {
  if (bench == nullptr || out_snapshot == nullptr) {
    return XRSLAM_BENCH_INVALID_ARGUMENT;
  }
  *out_snapshot = {};
  std::lock_guard<std::mutex> lock(bench->mutex);

  out_snapshot->phase = bench->phase;
  out_snapshot->last_event_timestamp_ns = bench->last_event_timestamp_ns;
  out_snapshot->last_accel_timestamp_ns = bench->last_accel_timestamp_ns;
  out_snapshot->last_gyro_timestamp_ns = bench->last_gyro_timestamp_ns;
  out_snapshot->last_image_timestamp_ns = bench->last_image_timestamp_ns;
  // UINT64_MAX is transport metadata for unavailable stock observability.
  constexpr uint64_t unavailable = std::numeric_limits<uint64_t>::max();
  out_snapshot->engine_backlog_peak = unavailable;
  bench->counters.init_too_few_frames = unavailable;
  out_snapshot->init_too_few_frames = unavailable;
  bench->counters.init_attempts = unavailable;
  out_snapshot->init_attempts = unavailable;
  bench->counters.init_fail_matches = unavailable;
  out_snapshot->init_fail_matches = unavailable;
  bench->counters.init_fail_parallax = unavailable;
  out_snapshot->init_fail_parallax = unavailable;
  bench->counters.init_fail_rotation = unavailable;
  out_snapshot->init_fail_rotation = unavailable;
  bench->counters.init_fail_triangulation = unavailable;
  out_snapshot->init_fail_triangulation = unavailable;
  bench->counters.init_fail_imu = unavailable;
  out_snapshot->init_fail_imu = unavailable;
  bench->counters.init_success = unavailable;
  out_snapshot->init_success = unavailable;
  bench->counters.solve_calls = unavailable;
  out_snapshot->solve_calls = unavailable;
  bench->counters.solve_unusable = unavailable;
  out_snapshot->solve_unusable = unavailable;
  bench->counters.track_evaluated = unavailable;
  out_snapshot->track_evaluated = unavailable;
  bench->counters.track_reject_depth = unavailable;
  out_snapshot->track_reject_depth = unavailable;
  bench->counters.track_reject_rpe = unavailable;
  out_snapshot->track_reject_rpe = unavailable;
  bench->counters.track_rpe_samples = unavailable;
  out_snapshot->track_rpe_samples = unavailable;
  bench->counters.track_rpe_millipx = unavailable;
  out_snapshot->track_rpe_millipx = unavailable;
  bench->counters.frames_rpe_calls = unavailable;
  out_snapshot->frames_rpe_calls = unavailable;
  bench->counters.frames_rpe_reject = unavailable;
  out_snapshot->frames_rpe_reject = unavailable;
  bench->counters.frames_rpe_samples = unavailable;
  out_snapshot->frames_rpe_samples = unavailable;
  bench->counters.frames_rpe_millipx = unavailable;
  out_snapshot->frames_rpe_millipx = unavailable;
  bench->counters.relation_samples = unavailable;
  out_snapshot->relation_samples = unavailable;
  bench->counters.relation_none = unavailable;
  out_snapshot->relation_none = unavailable;
  bench->counters.relation_orientation_unreadable = unavailable;
  out_snapshot->relation_orientation_unreadable = unavailable;
  bench->counters.relation_position_unreadable = unavailable;
  out_snapshot->relation_position_unreadable = unavailable;
  bench->counters.relation_valid_untracked = unavailable;
  out_snapshot->relation_valid_untracked = unavailable;
  bench->counters.relation_tracked = unavailable;
  out_snapshot->relation_tracked = unavailable;
  bench->counters.divergence_frames = unavailable;
  out_snapshot->divergence_frames = unavailable;
  bench->counters.divergence_flags_seen = unavailable;
  out_snapshot->divergence_flags_seen = unavailable;
  bench->counters.divergence_longest_ms = unavailable;
  out_snapshot->divergence_longest_ms = unavailable;
  bench->counters.divergence_sustained_frames = unavailable;
  out_snapshot->divergence_sustained_frames = unavailable;
  out_snapshot->init_mirror_us = unavailable;
  out_snapshot->result_state_out_of_range = bench->counters.result_state_out_of_range;
  out_snapshot->result_pose_nonfinite = bench->counters.result_pose_nonfinite;
  out_snapshot->result_timestamp_unconvertible = bench->counters.result_timestamp_unconvertible;
  out_snapshot->pending_event_count = bench->has_pending_image ? 1 : 0;
  out_snapshot->pending_result_count = bench->result_count;
  out_snapshot->event_queue_capacity = kImageSlotCapacity;
  out_snapshot->estimator_imu_ingested = unavailable;
  out_snapshot->estimator_camera_ingested = unavailable;
  out_snapshot->event_queue_peak =
      bench->image_slot_peak;
  out_snapshot->result_queue_capacity = kResultQueueCapacity;
  out_snapshot->result_queue_peak = bench->result_queue_peak;
  out_snapshot->counters = bench->counters;
  return XRSLAM_BENCH_OK;
}

extern "C" void xrslam_bench_destroy(xrslam_bench_t *bench) {
  if (bench == nullptr)
    return;
  {
    std::lock_guard<std::mutex> lock(bench->mutex);
    if (bench->original_io.active.load()) return;
    bench->original_io_destroying = true;
  }
  (void)xrslam_bench_stop(bench);
  // Failed official teardown retains both the handle and the shared owner.
  // Caller must quiesce status/cancel observers before destroying this handle,
  // as required by the original IO ABI; active replay remains rejected above.
  if (!bench->upstream_destroyed) return;
  delete bench;
}

extern "C" xrslam_bench_status_t xrslam_bench_replay_original_io(
    xrslam_bench_t *bench, const char *mav0, const char *output) {
  if (!bench || !mav0 || !output || mav0[0]!='/' || output[0]!='/')
    return XRSLAM_BENCH_INVALID_ARGUMENT;
  {
    std::lock_guard<std::mutex> lock(bench->mutex);
    if (bench->original_io.active.load() || bench->original_io_destroying ||
        bench->phase != XRSLAM_BENCH_PHASE_RUNNING ||
        bench->counters.events_accepted || bench->has_pending_image)
      return XRSLAM_BENCH_INVALID_STATE;
    bench->original_io.reset();
    bench->original_io.active.store(true);
  }
  auto result = replay_original_io(bench->api, bench->upstream_config,
                                         mav0, output, bench->original_io);
  // replay_original_io has returned: its original writer is closed. The
  // active lease seals all legacy ingress and prevents Stop/Destroy until this
  // read-only terminal observation ends. No Run/STATE/BODY/writer calls follow.
  bool quiescent = false;
  if (bench->original_io.natural_eof.load() &&
      !bench->original_io.failed.load()) {
    try {
      const auto outcome = pw_headless_eof::wait(
          [bench] {
            if (!pw_common_core::Owns(pw_common_core::Mode::Bench,
                                       bench->owner_token))
              return pw_headless_eof::Observation::Unavailable;
#if defined(PW_XRSLAM_CORE_LINKED) && PW_XRSLAM_CORE_LINKED
            // A fake testing backend must not observe an unrelated singleton.
            const auto &api = bench->api;
            if (api.create != kProductionAPI.create ||
                api.push_sensor_data != kProductionAPI.push_sensor_data ||
                api.run_one_frame != kProductionAPI.run_one_frame ||
                api.get_result != kProductionAPI.get_result ||
                api.destroy != kProductionAPI.destroy)
              return pw_headless_eof::Observation::Unavailable;
            return pw_headless_eof::classify(
                xrslam::XRSLAMManager::Instance().ObserveProcessing());
#else
            return pw_headless_eof::Observation::Unavailable;
#endif
          },
          [bench] { return bench->original_io.cancel.load(); },
          [bench] { bench->original_io.wait_terminal_poll(); });
      quiescent = outcome == pw_headless_eof::Outcome::Quiescent;
      if (!quiescent) {
        bench->original_io.failed = true;
        result = outcome == pw_headless_eof::Outcome::Cancelled
                     ? XRSLAM_BENCH_INPUT_SEALED : XRSLAM_BENCH_INTERNAL_ERROR;
      }
    } catch (...) {
      bench->original_io.failed = true;
      result = XRSLAM_BENCH_INTERNAL_ERROR;
    }
  }
  {
    std::lock_guard<std::mutex> lock(bench->mutex);
    // Linearize terminal acceptance against cancel under the existing lease
    // lock. The generation is retained through later official Destroy/join.
    if (bench->original_io.cancel.load() ||
        !pw_common_core::Owns(pw_common_core::Mode::Bench, bench->owner_token)) {
      quiescent = false;
      result = bench->original_io.cancel.load() ? XRSLAM_BENCH_INPUT_SEALED
                                               : XRSLAM_BENCH_INTERNAL_ERROR;
    }
    if (!quiescent) bench->original_io.failed = true;
    bench->original_io.incomplete = !quiescent || bench->original_io.poses.load()==0;
    bench->phase = quiescent ? XRSLAM_BENCH_PHASE_DRAINED
                             : XRSLAM_BENCH_PHASE_FAILED;
    bench->original_io.active.store(false);
  }
  return result;
}
extern "C" xrslam_bench_status_t xrslam_bench_cancel_original_io(xrslam_bench_t *bench) {
  if (!bench) return XRSLAM_BENCH_INVALID_ARGUMENT;
  // Narrow lease lock prevents a cancel from being lost during reset/start.
  std::lock_guard<std::mutex> lock(bench->mutex);
  if (!bench->original_io.active.load()) return XRSLAM_BENCH_INVALID_STATE;
  bench->original_io.request_cancel();
  return XRSLAM_BENCH_OK;
}
extern "C" xrslam_bench_status_t xrslam_bench_original_io_status(
    xrslam_bench_t *bench, xrslam_bench_original_io_snapshot_t *out) {
  if (!bench || !out || out->struct_size != sizeof(*out))
    return XRSLAM_BENCH_INVALID_ARGUMENT;
  bench->original_io.snapshot(*out);
  return XRSLAM_BENCH_OK;
}

extern "C" const char *xrslam_bench_status_name(xrslam_bench_status_t status) {
  switch (status) {
  case XRSLAM_BENCH_OK:
    return "ok";
  case XRSLAM_BENCH_INVALID_ARGUMENT:
    return "invalid_argument";
  case XRSLAM_BENCH_BACKEND_UNAVAILABLE:
    return "backend_unavailable";
  case XRSLAM_BENCH_CREATE_FAILED:
    return "create_failed";
  case XRSLAM_BENCH_INVALID_STATE:
    return "invalid_state";
  case XRSLAM_BENCH_TIMESTAMP_REGRESSION:
    return "timestamp_regression";
  case XRSLAM_BENCH_INPUT_SEALED:
    return "input_sealed";
  case XRSLAM_BENCH_NO_FRAME:
    return "no_frame";
  case XRSLAM_BENCH_NO_OUTPUT:
    return "no_output";
  case XRSLAM_BENCH_END_OF_STREAM:
    return "end_of_stream";
  case XRSLAM_BENCH_NONFINITE_OUTPUT:
    return "nonfinite_output";
  case XRSLAM_BENCH_INTERNAL_ERROR:
    return "internal_error";
  case XRSLAM_BENCH_QUEUE_FULL:
    return "queue_full";
  }
  return "unknown";
}
