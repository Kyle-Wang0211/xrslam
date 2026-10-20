#include "PwXrslamTransportCore.h"
#include "CommonCoreOwner.h"
#include "PwXrslamPhaseTiming.h"
#include "PhaseAccumulator.hpp"
#include <chrono>
#include <limits>
#include <cstring>
#include <type_traits>

#include <cmath>
#include <cstddef>
#include <mutex>
#include <fstream>
#include <iterator>
#include <string>

// Artifact bindings must choose explicitly. Inferring from platform or a
// filename permits file paths to be interpreted as YAML by the shared core.
#ifndef PW_XRSLAM_ARTIFACT_CONFIG_INPUT_MODE
#error "Select the pinned XRSLAM archive config-input ABI explicitly"
#endif
#if PW_XRSLAM_ARTIFACT_CONFIG_INPUT_MODE != 1 && PW_XRSLAM_ARTIFACT_CONFIG_INPUT_MODE != 2
#error "Unsupported XRSLAM archive config-input ABI"
#endif

#include <vector> // pinned official XRSLAM.h declares std::vector debug data
#include "XRSLAM.h"
#include <xrslam/extra/yaml_config.h>
#include <Eigen/Geometry>

namespace {

std::mutex g_core_mutex;
std::mutex g_phase_mutex;
PWXrslamPhaseTimingSnapshot g_phase_timing{};

uint64_t PhaseNowNs() {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}
void RecordPhase(int phase, uint64_t start, uint64_t end, uint64_t generation) {
  std::lock_guard<std::mutex> lock(g_phase_mutex);
  if (generation != g_phase_timing.lifecycle_generation) return;
  pw_common_core::AccumulatePhase(g_phase_timing.phases[phase], start, end);
}
uint64_t PhaseGeneration() {
  std::lock_guard<std::mutex> lock(g_phase_mutex);
  return g_phase_timing.lifecycle_generation;
}
class PhaseTimer {
 public:
  explicit PhaseTimer(int phase) : phase_(phase), generation_(PhaseGeneration()), start_(PhaseNowNs()) {}
  ~PhaseTimer() { RecordPhase(phase_, start_, PhaseNowNs(), generation_); }
 private:
  int phase_;
  uint64_t generation_, start_;
};
void ResetPhaseTiming(uint64_t generation) {
  std::lock_guard<std::mutex> lock(g_phase_mutex);
  g_phase_timing = {};
  g_phase_timing.struct_size = sizeof(g_phase_timing);
  g_phase_timing.abi_version = 1;
  g_phase_timing.phase_count = PW_XRSLAM_PHASE_COUNT;
  g_phase_timing.lifecycle_generation = generation;
}


struct StreamState {
  bool has_submitted_timestamp = false;
  double last_submitted_timestamp = 0.0;
  PWXrslamTimestampTrace trace{};
};

struct SessionCounters {
  uint64_t camera_submitted = 0;
  uint64_t camera_run_calls = 0;
  uint64_t acceleration_submitted = 0;
  uint64_t gyroscope_submitted = 0;
  uint64_t rejected_invalid_argument = 0;
  uint64_t rejected_non_monotonic = 0;
  uint64_t rejected_not_running = 0;
};

bool g_running = false;
bool g_ingress_poisoned = false;
pw_common_core::Token g_owner_token = 0;
double g_camera_time_offset_seconds = 0.0;
uint64_t g_lifecycle_generation = 0;
StreamState g_streams[3];
SessionCounters g_counters;

// Capture once during successful Create while the real returned YamlConfig
// is alive. Only POD copies are retained; the config pointer is never stored.
PWXrslamParsedCameraConfig g_parsed_camera_config{};
pw_common_core::Token g_parsed_camera_owner_token = 0;
bool g_parsed_camera_config_valid = false;

void InvalidateParsedCameraConfigLocked() {
  g_parsed_camera_config = {};
  g_parsed_camera_owner_token = 0;
  g_parsed_camera_config_valid = false;
}

void CopyParsedCameraConfigLocked(void *config) noexcept {
  InvalidateParsedCameraConfigLocked();
  if (config == nullptr)
    return;
  try {
    // Pinned XRSLAMInternal.cpp: XRSLAMCreate assigns
    // static_cast<void *>(yaml_config.get()) for shared_ptr<extra::YamlConfig>.
    // Include the real official class, not a guessed interface/layout mirror.
    const auto *yaml_config =
        static_cast<const xrslam::extra::YamlConfig *>(config);
    const auto intrinsic = yaml_config->camera_intrinsic();
    const auto resolution = yaml_config->camera_resolution();
    static_assert(std::is_same<typename decltype(intrinsic)::Scalar, double>::value,
                  "official camera_intrinsic must remain double");
    static_assert(std::is_same<typename decltype(resolution)::Scalar, double>::value,
                  "official camera_resolution must remain double");
    static_assert(std::numeric_limits<double>::is_iec559,
                  "parsed config ABI requires IEEE-754 double");
    static_assert(decltype(intrinsic)::RowsAtCompileTime == 3 &&
                      decltype(intrinsic)::ColsAtCompileTime == 3,
                  "official intrinsic shape must remain 3x3");
    static_assert(decltype(resolution)::RowsAtCompileTime == 2 &&
                      decltype(resolution)::ColsAtCompileTime == 1,
                  "official resolution shape must remain 2x1");
    PWXrslamParsedCameraConfig snapshot{};
    snapshot.abi_version = PW_XRSLAM_PARSED_CAMERA_CONFIG_ABI_VERSION;
    snapshot.struct_size = sizeof(snapshot);
    snapshot.lifecycle_generation = g_lifecycle_generation;
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        std::memcpy(&snapshot.camera_intrinsic_row_major[row * 3 + column],
                    &intrinsic(row, column), sizeof(double));
      }
    }
    for (int index = 0; index < 2; ++index) {
      std::memcpy(&snapshot.camera_resolution[index], &resolution(index),
                  sizeof(double));
    }
    std::memcpy(&g_parsed_camera_config, &snapshot, sizeof(snapshot));
    g_parsed_camera_owner_token = g_owner_token;
    g_parsed_camera_config_valid = true;
  } catch (...) {
    // Readback is observational. Do not change Create's official return or
    // fill a guessed/fallback K when the accessor copy cannot be completed.
    InvalidateParsedCameraConfigLocked();
  }
}


bool IsFinite3(double x, double y, double z) {
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

bool IsValidStream(int32_t stream) {
  return stream >= PW_XRSLAM_STREAM_CAMERA &&
         stream <= PW_XRSLAM_STREAM_GYROSCOPE;
}

void ResetSessionState() {
  for (StreamState &stream : g_streams)
    stream = {};
  g_counters = {};
}

void FillDestroyReceipt(bool acknowledged, PWXrslamDestroyReceipt *receipt) {
  if (receipt == nullptr)
    return;
  *receipt = {};
  receipt->lifecycle_generation = g_lifecycle_generation;
  receipt->camera_submitted = g_counters.camera_submitted;
  receipt->camera_run_calls = g_counters.camera_run_calls;
  receipt->acceleration_submitted = g_counters.acceleration_submitted;
  receipt->gyroscope_submitted = g_counters.gyroscope_submitted;
  receipt->rejected_invalid_argument = g_counters.rejected_invalid_argument;
  receipt->rejected_non_monotonic = g_counters.rejected_non_monotonic;
  receipt->rejected_not_running = g_counters.rejected_not_running;
  receipt->destroy_acknowledged = acknowledged ? 1 : 0;
}

void FillCounters(PWXrslamTransportCounters *counters) {
  if (counters == nullptr)
    return;
  *counters = {};
  counters->lifecycle_generation = g_lifecycle_generation;
  counters->camera_submitted = g_counters.camera_submitted;
  counters->camera_run_calls = g_counters.camera_run_calls;
  counters->acceleration_submitted = g_counters.acceleration_submitted;
  counters->gyroscope_submitted = g_counters.gyroscope_submitted;
  counters->rejected_invalid_argument = g_counters.rejected_invalid_argument;
  counters->rejected_non_monotonic = g_counters.rejected_non_monotonic;
  counters->rejected_not_running = g_counters.rejected_not_running;
  counters->running = g_running && !g_ingress_poisoned ? 1 : 0;
}

void RecordTrace(int32_t stream, int32_t status, double raw_timestamp,
                 double offset, double effective_timestamp) {
  StreamState &state = g_streams[stream];
  state.trace.stream = stream;
  state.trace.status = status;
  state.trace.raw_timestamp = raw_timestamp;
  state.trace.applied_offset = offset;
  state.trace.effective_timestamp = effective_timestamp;
}

int32_t ValidateTimestampLocked(int32_t stream, double raw_timestamp,
                                double offset, double *effective_timestamp) {
  if (!g_running || g_ingress_poisoned) {
    ++g_counters.rejected_not_running;
    if (std::isfinite(raw_timestamp) && std::isfinite(offset)) {
      RecordTrace(stream, PW_XRSLAM_ERR_NOT_RUNNING, raw_timestamp, offset,
                  raw_timestamp + offset);
    }
    return PW_XRSLAM_ERR_NOT_RUNNING;
  }
  if (!std::isfinite(raw_timestamp) || !std::isfinite(offset)) {
    ++g_counters.rejected_invalid_argument;
    RecordTrace(stream, PW_XRSLAM_ERR_INVALID_ARGUMENT, raw_timestamp, offset,
                raw_timestamp);
    return PW_XRSLAM_ERR_INVALID_ARGUMENT;
  }
  const double effective = raw_timestamp + offset;
  if (!std::isfinite(effective)) {
    ++g_counters.rejected_invalid_argument;
    RecordTrace(stream, PW_XRSLAM_ERR_INVALID_ARGUMENT, raw_timestamp, offset,
                effective);
    return PW_XRSLAM_ERR_INVALID_ARGUMENT;
  }
  StreamState &state = g_streams[stream];
  if (state.has_submitted_timestamp &&
      effective <= state.last_submitted_timestamp) {
    ++g_counters.rejected_non_monotonic;
    RecordTrace(stream, PW_XRSLAM_ERR_NON_MONOTONIC, raw_timestamp, offset,
                effective);
    return PW_XRSLAM_ERR_NON_MONOTONIC;
  }
  *effective_timestamp = effective;
  return PW_XRSLAM_OK;
}

void MarkSubmittedLocked(int32_t stream, double raw_timestamp, double offset,
                         double effective_timestamp) {
  StreamState &state = g_streams[stream];
  state.has_submitted_timestamp = true;
  state.last_submitted_timestamp = effective_timestamp;
  ++state.trace.submitted_sequence;
  RecordTrace(stream, PW_XRSLAM_OK, raw_timestamp, offset, effective_timestamp);
}

#if PW_XRSLAM_ARTIFACT_CONFIG_INPUT_MODE == 2
bool ReadConfigText(const char *path, std::string *text) {
  try {
    std::ifstream input(path, std::ios::binary);
    if (!input)
      return false;
    text->assign(std::istreambuf_iterator<char>(input),
                 std::istreambuf_iterator<char>());
    return !input.bad() && !text->empty() &&
           text->find('\0') == std::string::npos;
  } catch (const std::ios_base::failure &) {
    return false;
  }
}
#endif

void CopyRawPose(const XRSLAMPose &source, PWXrslamRawPose *destination) {
  if (destination == nullptr)
    return;
  destination->timestamp = source.timestamp;
  for (int i = 0; i < 4; ++i) {
    destination->quaternion[i] = source.quaternion[i];
  }
  for (int i = 0; i < 3; ++i) {
    destination->translation[i] = source.translation[i];
  }
}

} // namespace

extern "C" int32_t PWXrslamTransportGetArtifactConfigInputMode(void) {
  return PW_XRSLAM_ARTIFACT_CONFIG_INPUT_MODE;
}

extern "C" int32_t PWXrslamTransportCreate(const char *slam_config_path,
                                           const char *device_config_path) {
  return PWXrslamTransportCreateWithCameraTimeOffset(slam_config_path,
                                                     device_config_path, 0.0);
}

extern "C" int32_t
PWXrslamTransportCreateWithCameraTimeOffset(const char *slam_config_path,
                                            const char *device_config_path,
                                            double camera_time_offset_seconds) {
  if (slam_config_path == nullptr || device_config_path == nullptr ||
      !std::isfinite(camera_time_offset_seconds)) {
    return 0;
  }
  std::lock_guard<std::mutex> lock(g_core_mutex);
  if (g_running)
    return 0;
  const char *slam_config_input = slam_config_path;
  const char *device_config_input = device_config_path;
#if PW_XRSLAM_ARTIFACT_CONFIG_INPUT_MODE == 2
  std::string slam_config_text;
  std::string device_config_text;
  if (!ReadConfigText(slam_config_path, &slam_config_text) ||
      !ReadConfigText(device_config_path, &device_config_text)) {
    return 0;
  }
  slam_config_input = slam_config_text.c_str();
  device_config_input = device_config_text.c_str();
#endif
  pw_common_core::Reservation reservation(pw_common_core::Mode::Live);
  if (!reservation.token()) return 0;
  InvalidateParsedCameraConfigLocked();
  void *config = nullptr;
  int32_t result = 0;
  try {
    result = XRSLAMCreate(slam_config_input, device_config_input, "",
                                      "pocketworld", &config);
  } catch (...) {
    // Upstream ownership after an exception is unknown: retain exclusion.
    g_owner_token = reservation.Detach();
    return 0;
  }
  if (result != 1)
    return 0;
  g_owner_token = reservation.Detach();
  ++g_lifecycle_generation;
  ResetPhaseTiming(g_lifecycle_generation);
  ResetSessionState();
  g_camera_time_offset_seconds = camera_time_offset_seconds;
  g_ingress_poisoned = false;
  g_running = true;
  CopyParsedCameraConfigLocked(config);
  return 1;
}

extern "C" void PWXrslamTransportDestroy(void) {
  (void)PWXrslamTransportDestroyWithReceipt(nullptr);
}

extern "C" int32_t
PWXrslamTransportDestroyWithReceipt(PWXrslamDestroyReceipt *receipt) {
  std::lock_guard<std::mutex> lock(g_core_mutex);
  InvalidateParsedCameraConfigLocked();
  if (!g_running) {
    ++g_counters.rejected_not_running;
    FillDestroyReceipt(false, receipt);
    return PW_XRSLAM_ERR_NOT_RUNNING;
  }
  if (!pw_common_core::Owns(pw_common_core::Mode::Live, g_owner_token)) {
    FillDestroyReceipt(false, receipt);
    return PW_XRSLAM_ERR_NOT_RUNNING;
  }
  // The transport mutex has quiesced all live calls. Do not release the owner
  // until official teardown returns, including its own worker quiescence.
  try { XRSLAMDestroy(); } catch (...) {
    // Retain the owner for explicit teardown retry, but never feed a core
    // whose partial destruction/worker state is unknown.
    g_ingress_poisoned = true;
    FillDestroyReceipt(false, receipt);
    return PW_XRSLAM_ERR_NOT_RUNNING;
  }
  g_running = false;
  pw_common_core::Release(pw_common_core::Mode::Live, g_owner_token);
  g_owner_token = 0;
  FillDestroyReceipt(true, receipt);
  return PW_XRSLAM_OK;
}

extern "C" int32_t PWXrslamTransportPushCameraAndRunRaw(
    uint8_t *data, double timestamp, int32_t stride, int32_t camera_id,
    int32_t channel, int32_t *raw_state, PWXrslamRawPose *raw_pose) {
  const uint64_t wait_start = PhaseNowNs();
  std::lock_guard<std::mutex> lock(g_core_mutex);
  const uint64_t wait_end = PhaseNowNs();
  RecordPhase(PW_XRSLAM_PHASE_CAMERA_MUTEX_WAIT, wait_start, wait_end, PhaseGeneration());
  if (data == nullptr || stride <= 0 || channel <= 0 || raw_state == nullptr ||
      raw_pose == nullptr) {
    ++g_counters.rejected_invalid_argument;
    return PW_XRSLAM_ERR_INVALID_ARGUMENT;
  }
  double effective_timestamp = 0.0;
  const int32_t validation = ValidateTimestampLocked(
      PW_XRSLAM_STREAM_CAMERA, timestamp, g_camera_time_offset_seconds,
      &effective_timestamp);
  if (validation != PW_XRSLAM_OK)
    return validation;

  XRSLAMImage image{};
  image.data = data;
  image.timeStamp = effective_timestamp;
  image.stride = stride;
  image.camera_id = camera_id;
  image.channel = channel;
  image.ext = nullptr;

  { PhaseTimer timer(PW_XRSLAM_PHASE_CAMERA_PUSH); XRSLAMPushSensorData(XRSLAM_SENSOR_CAMERA, &image); }
  MarkSubmittedLocked(PW_XRSLAM_STREAM_CAMERA, timestamp,
                      g_camera_time_offset_seconds, effective_timestamp);
  ++g_counters.camera_submitted;
  *raw_state = static_cast<int32_t>(XRSLAM_STATE_INITIALIZING);
  *raw_pose = {};
  // Frozen PC player main.cpp: every image reaches PushSensorData, while
  // RunOneFrame/result queries begin only after both IMU streams have been
  // submitted in this generation. Rejected wrapper inputs never increment
  // these counters; ResetSessionState clears them after successful Create.
  if (g_counters.acceleration_submitted == 0 ||
      g_counters.gyroscope_submitted == 0) {
    return PW_XRSLAM_OK;
  }
  { PhaseTimer timer(PW_XRSLAM_PHASE_RUN); XRSLAMRunOneFrame(); }
  ++g_counters.camera_run_calls;

  XRSLAMState state = XRSLAM_STATE_INITIALIZING;
  { PhaseTimer timer(PW_XRSLAM_PHASE_RESULT_STATE); XRSLAMGetResult(XRSLAM_RESULT_STATE, &state); }
  *raw_state = static_cast<int32_t>(state);
  if (state == XRSLAM_STATE_TRACKING_SUCCESS) {
    // Product consumers require upstream CAMERA pose. The PC benchmark's
    // separate BODY-pose output must not relabel this product frame.
    XRSLAMPose pose{};
    { PhaseTimer timer(PW_XRSLAM_PHASE_RESULT_CAMERA_POSE); XRSLAMGetResult(XRSLAM_RESULT_CAMERA_POSE, &pose); }
    CopyRawPose(pose, raw_pose);
  }
  return PW_XRSLAM_OK;
}

extern "C" int32_t PWXrslamTransportPushAccelerationRaw(double timestamp,
                                                        double x, double y,
                                                        double z) {
  const uint64_t wait_start = PhaseNowNs();
  std::lock_guard<std::mutex> lock(g_core_mutex);
  const uint64_t wait_end = PhaseNowNs();
  RecordPhase(PW_XRSLAM_PHASE_ACCELERATION_MUTEX_WAIT, wait_start, wait_end, PhaseGeneration());
  if (!IsFinite3(x, y, z)) {
    ++g_counters.rejected_invalid_argument;
    return PW_XRSLAM_ERR_INVALID_ARGUMENT;
  }
  double effective_timestamp = 0.0;
  const int32_t validation = ValidateTimestampLocked(
      PW_XRSLAM_STREAM_ACCELERATION, timestamp, 0.0, &effective_timestamp);
  if (validation != PW_XRSLAM_OK)
    return validation;
  XRSLAMAcceleration sample{{x, y, z}, effective_timestamp};
  { PhaseTimer timer(PW_XRSLAM_PHASE_ACCELERATION_PUSH); XRSLAMPushSensorData(XRSLAM_SENSOR_ACCELERATION, &sample); }
  MarkSubmittedLocked(PW_XRSLAM_STREAM_ACCELERATION, timestamp, 0.0,
                      effective_timestamp);
  ++g_counters.acceleration_submitted;
  return PW_XRSLAM_OK;
}

extern "C" int32_t PWXrslamTransportPushGyroscopeRaw(double timestamp, double x,
                                                     double y, double z) {
  const uint64_t wait_start = PhaseNowNs();
  std::lock_guard<std::mutex> lock(g_core_mutex);
  const uint64_t wait_end = PhaseNowNs();
  RecordPhase(PW_XRSLAM_PHASE_GYROSCOPE_MUTEX_WAIT, wait_start, wait_end, PhaseGeneration());
  if (!IsFinite3(x, y, z)) {
    ++g_counters.rejected_invalid_argument;
    return PW_XRSLAM_ERR_INVALID_ARGUMENT;
  }
  double effective_timestamp = 0.0;
  const int32_t validation = ValidateTimestampLocked(
      PW_XRSLAM_STREAM_GYROSCOPE, timestamp, 0.0, &effective_timestamp);
  if (validation != PW_XRSLAM_OK)
    return validation;
  XRSLAMGyroscope sample{{x, y, z}, effective_timestamp};
  { PhaseTimer timer(PW_XRSLAM_PHASE_GYROSCOPE_PUSH); XRSLAMPushSensorData(XRSLAM_SENSOR_GYROSCOPE, &sample); }
  MarkSubmittedLocked(PW_XRSLAM_STREAM_GYROSCOPE, timestamp, 0.0,
                      effective_timestamp);
  ++g_counters.gyroscope_submitted;
  return PW_XRSLAM_OK;
}

extern "C" int32_t
PWXrslamTransportGetLastTimestampTrace(int32_t stream,
                                       PWXrslamTimestampTrace *trace) {
  if (!IsValidStream(stream) || trace == nullptr) {
    return PW_XRSLAM_ERR_INVALID_ARGUMENT;
  }
  std::lock_guard<std::mutex> lock(g_core_mutex);
  *trace = g_streams[stream].trace;
  trace->stream = stream;
  return PW_XRSLAM_OK;
}

extern "C" int32_t
PWXrslamTransportGetPoseMetadata(PWXrslamPoseMetadata *metadata) {
  if (metadata == nullptr)
    return PW_XRSLAM_ERR_INVALID_ARGUMENT;
  metadata->transform_semantics = PW_XRSLAM_T_WORLD_CAMERA;
  metadata->quaternion_order = PW_XRSLAM_QUATERNION_XYZW;
  return PW_XRSLAM_OK;
}

extern "C" int32_t
PWXrslamTransportGetCounters(PWXrslamTransportCounters *counters) {
  if (counters == nullptr)
    return PW_XRSLAM_ERR_INVALID_ARGUMENT;
  std::lock_guard<std::mutex> lock(g_core_mutex);
  FillCounters(counters);
  return PW_XRSLAM_OK;
}

extern "C" int32_t PWXrslamTransportPrepareGrayBoxNxN(
    const uint8_t *source, int32_t source_width, int32_t source_height,
    int32_t source_stride, int32_t factor, uint8_t *destination,
    int32_t destination_capacity, int32_t *destination_width,
    int32_t *destination_height) {
  PhaseTimer gray_timer(PW_XRSLAM_PHASE_GRAY_PREPARE);
  if (source == nullptr || destination == nullptr ||
      destination_width == nullptr || destination_height == nullptr ||
      source_width <= 0 || source_height <= 0 ||
      source_stride < source_width || factor <= 0 ||
      source_width % factor != 0 || source_height % factor != 0) {
    return PW_XRSLAM_ERR_INVALID_ARGUMENT;
  }
  const int32_t output_width = source_width / factor;
  const int32_t output_height = source_height / factor;
  const int64_t output_size =
      static_cast<int64_t>(output_width) * output_height;
  if (output_size <= 0 || output_size > destination_capacity) {
    return PW_XRSLAM_ERR_INVALID_ARGUMENT;
  }
  const int32_t area = factor * factor;
  const int32_t half = area / 2;
  for (int32_t y = 0; y < output_height; ++y) {
    uint8_t *output_row = destination + y * output_width;
    for (int32_t x = 0; x < output_width; ++x) {
      int32_t sum = 0;
      for (int32_t dy = 0; dy < factor; ++dy) {
        const uint8_t *input = source +
            (y * factor + dy) * source_stride + x * factor;
        for (int32_t dx = 0; dx < factor; ++dx)
          sum += input[dx];
      }
      output_row[x] = static_cast<uint8_t>((sum + half) / area);
    }
  }
  *destination_width = output_width;
  *destination_height = output_height;
  return PW_XRSLAM_OK;
}

extern "C" int32_t
PWXrslamTransportPoseToWorldFromCameraMatrix(const PWXrslamRawPose *pose,
                                             double matrix[16]) {
  if (pose == nullptr || matrix == nullptr ||
      !IsFinite3(pose->translation[0], pose->translation[1],
                 pose->translation[2]) ||
      !std::isfinite(pose->timestamp)) {
    return PW_XRSLAM_ERR_INVALID_ARGUMENT;
  }
  const double x = pose->quaternion[0];
  const double y = pose->quaternion[1];
  const double z = pose->quaternion[2];
  const double w = pose->quaternion[3];
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
      !std::isfinite(w)) {
    return PW_XRSLAM_ERR_INVALID_POSE;
  }
  const double norm_squared = x * x + y * y + z * z + w * w;
  if (!std::isfinite(norm_squared) || norm_squared <= 1e-24) {
    return PW_XRSLAM_ERR_INVALID_POSE;
  }
  // Match the pinned PC viewer's Eigen quaternion-to-matrix API without
  // normalization. Double precision preserves this diagnostic helper's ABI;
  // the original viewer uses Quaternionf/Matrix4f. Raw pose is never changed.
  const Eigen::Matrix3d rotation =
      Eigen::Quaterniond(w, x, y, z).toRotationMatrix();
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      matrix[row * 4 + column] = rotation(row, column);
    }
    matrix[row * 4 + 3] = pose->translation[row];
  }
  matrix[12] = 0.0;
  matrix[13] = 0.0;
  matrix[14] = 0.0;
  matrix[15] = 1.0;
  return PW_XRSLAM_OK;
}

extern "C" int32_t PWXrslamTransportGetPhaseTiming(PWXrslamPhaseTimingSnapshot *snapshot) {
  if (!snapshot || snapshot->struct_size != sizeof(*snapshot))
    return PW_XRSLAM_ERR_INVALID_ARGUMENT;
  std::lock_guard<std::mutex> lock(g_phase_mutex);
  *snapshot = g_phase_timing;
  snapshot->struct_size = sizeof(*snapshot);
  snapshot->abi_version = 1;
  snapshot->phase_count = PW_XRSLAM_PHASE_COUNT;
  return PW_XRSLAM_OK;
}


extern "C" int32_t PWXrslamTransportGetParsedCameraConfig(
    uint64_t expected_lifecycle_generation,
    PWXrslamParsedCameraConfig *snapshot) {
  if (snapshot == nullptr ||
      snapshot->abi_version != PW_XRSLAM_PARSED_CAMERA_CONFIG_ABI_VERSION ||
      snapshot->struct_size != sizeof(*snapshot)) {
    return PW_XRSLAM_ERR_INVALID_ARGUMENT;
  }
  std::lock_guard<std::mutex> lock(g_core_mutex);
  *snapshot = {};
  if (expected_lifecycle_generation == 0)
    return PW_XRSLAM_ERR_INVALID_ARGUMENT;
  if (!g_running || g_ingress_poisoned ||
      !pw_common_core::Owns(pw_common_core::Mode::Live, g_owner_token) ||
      !g_parsed_camera_config_valid ||
      g_parsed_camera_owner_token != g_owner_token ||
      expected_lifecycle_generation != g_lifecycle_generation ||
      g_parsed_camera_config.lifecycle_generation != g_lifecycle_generation) {
    return PW_XRSLAM_ERR_NOT_RUNNING;
  }
  std::memcpy(snapshot, &g_parsed_camera_config, sizeof(*snapshot));
  return PW_XRSLAM_OK;
}
