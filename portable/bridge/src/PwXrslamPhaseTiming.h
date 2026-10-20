#ifndef PW_XRSLAM_PHASE_TIMING_H
#define PW_XRSLAM_PHASE_TIMING_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
// Transport observations only; CAMERA pose frame remains unchanged.
enum PWXrslamPhaseTimingPhase {
 PW_XRSLAM_PHASE_CAMERA_MUTEX_WAIT = 0,
 PW_XRSLAM_PHASE_ACCELERATION_MUTEX_WAIT = 1,
 PW_XRSLAM_PHASE_GYROSCOPE_MUTEX_WAIT = 2,
 PW_XRSLAM_PHASE_GRAY_PREPARE = 3,
 PW_XRSLAM_PHASE_CAMERA_PUSH = 4,
 PW_XRSLAM_PHASE_RUN = 5,
 PW_XRSLAM_PHASE_RESULT_STATE = 6,
 PW_XRSLAM_PHASE_RESULT_CAMERA_POSE = 7,
 PW_XRSLAM_PHASE_ACCELERATION_PUSH = 8,
 PW_XRSLAM_PHASE_GYROSCOPE_PUSH = 9,
 PW_XRSLAM_PHASE_COUNT = 10
};
// flags: bit0 counter/total saturation; bit1 clock anomaly.
typedef struct PWXrslamPhaseTimingCounter {
 uint64_t count, total_ns, max_ns;
 uint32_t overflow_flags, reserved;
} PWXrslamPhaseTimingCounter;
typedef struct PWXrslamPhaseTimingSnapshot {
 uint32_t struct_size, abi_version, phase_count, reserved;
 uint64_t lifecycle_generation;
 PWXrslamPhaseTimingCounter phases[PW_XRSLAM_PHASE_COUNT];
} PWXrslamPhaseTimingSnapshot;
// Caller initializes struct_size=sizeof(snapshot). Returns existing transport
// OK/INVALID_ARGUMENT. Snapshot does not instrument itself or call XRSLAM.
// Fixed counters reset only on successful live Create and persist through Stop.
int32_t PWXrslamTransportGetPhaseTiming(PWXrslamPhaseTimingSnapshot *snapshot);
#ifdef __cplusplus
}
static_assert(sizeof(PWXrslamPhaseTimingCounter)==32, "phase ABI");
static_assert(sizeof(PWXrslamPhaseTimingSnapshot)==344, "snapshot ABI");
#endif
#endif
