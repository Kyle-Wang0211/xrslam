#ifndef XRSLAM_VERSION_H
#define XRSLAM_VERSION_H

// clang-format off

#define XRSLAM_MAJOR_VERSION 0
#define XRSLAM_MINOR_VERSION 5
#define XRSLAM_PATCH_VERSION 0

#define XRSLAM_VERSION_STRING "xrslam v0.5.0"

#define xrslam xrslam_0_5_0

#define XRSLAM_DEBUG
#define XRSLAM_ENABLE_DEBUG_INSPECTION
#define XRSLAM_ENABLE_THREADING
/* #undef XRSLAM_PC_HEADLESS_ONLY */
#if defined(XRSLAM_IOS)
#error "XRSLAM_IOS is a CMake packaging option, not a shared-core compiler define"
#endif
// All targets use the original PC/shared-core algorithm branches.
/* #undef XRSLAM_TEST */
// clang-format on

#endif // XRSLAM_VERSION_H
