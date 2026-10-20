#ifndef POCKETWORLD_XRSLAM_PARSED_CAMERA_CONFIG_H_
#define POCKETWORLD_XRSLAM_PARSED_CAMERA_CONFIG_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PW_XRSLAM_PARSED_CAMERA_CONFIG_ABI_VERSION 1u

// Exact copies of official YamlConfig accessor return values, NOT a separate
// YAML parser, physical calibration claim, normalization, or image transform.
// Fixed-width C fields only; double binary64. Row-major storage here is an
// ABI layout choice: each element is copied from intrinsic(row,column).
// camera_resolution retains both original doubles, including fractions.
typedef struct PWXrslamParsedCameraConfig {
  uint32_t abi_version;
  uint32_t struct_size;
  uint64_t lifecycle_generation;
  double camera_intrinsic_row_major[9];
  double camera_resolution[2];
} PWXrslamParsedCameraConfig;

// Additive 35th combined-engine export; keep all existing 34 exports.
// Caller initializes abi_version=1 and struct_size=sizeof(*snapshot), and
// supplies the successful session's nonzero generation from GetCounters or
// its existing start receipt. A stale generation is rejected, never rebound.
// Returns existing PW_XRSLAM_OK / ERR_INVALID_ARGUMENT / ERR_NOT_RUNNING.
// An exact-size/version output is zeroed before any unavailable/stale return;
// invalid/null/unknown-size/version outputs are not written. Serialized under
// the same core mutex as Create/Destroy; no retained/destroyed config is read.
// Unavailable parsed config leaves Create's original 1/0 semantics unchanged.
int32_t PWXrslamTransportGetParsedCameraConfig(
    uint64_t expected_lifecycle_generation,
    PWXrslamParsedCameraConfig *snapshot);

#ifdef __cplusplus
} // extern "C"
#define PW_XRSLAM_PARSED_CONFIG_ASSERT static_assert
#elif defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
#define PW_XRSLAM_PARSED_CONFIG_ASSERT _Static_assert
#endif

#ifdef PW_XRSLAM_PARSED_CONFIG_ASSERT
PW_XRSLAM_PARSED_CONFIG_ASSERT(sizeof(double) == 8, "parsed config requires binary64 size");
PW_XRSLAM_PARSED_CONFIG_ASSERT(sizeof(PWXrslamParsedCameraConfig) == 104, "parsed config ABI size");
PW_XRSLAM_PARSED_CONFIG_ASSERT(offsetof(PWXrslamParsedCameraConfig, abi_version) == 0, "ABI version offset");
PW_XRSLAM_PARSED_CONFIG_ASSERT(offsetof(PWXrslamParsedCameraConfig, struct_size) == 4, "ABI size offset");
PW_XRSLAM_PARSED_CONFIG_ASSERT(offsetof(PWXrslamParsedCameraConfig, lifecycle_generation) == 8, "generation offset");
PW_XRSLAM_PARSED_CONFIG_ASSERT(offsetof(PWXrslamParsedCameraConfig, camera_intrinsic_row_major) == 16, "intrinsic offset");
PW_XRSLAM_PARSED_CONFIG_ASSERT(offsetof(PWXrslamParsedCameraConfig, camera_resolution) == 88, "resolution offset");
#undef PW_XRSLAM_PARSED_CONFIG_ASSERT
#endif

#endif // POCKETWORLD_XRSLAM_PARSED_CAMERA_CONFIG_H_
