cmake_minimum_required(VERSION 3.13 FATAL_ERROR)
set(CMAKE_BUILD_TYPE Release)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
project(xrslam-superbuild)

include(CMakeDependentOption)
include(SuperBuildDepends)
superbuild_option(cpp17)
superbuild_option(modern-cpp)

option(XRSLAM_DEBUG "..." ON)
option(XRSLAM_ENABLE_DEBUG_INSPECTION "..." ON)
option(XRSLAM_ENABLE_THREADING "..." OFF)
option(XRSLAM_PC_HEADLESS_ONLY "..." OFF)
option(XRSLAM_IOS "..." OFF)
option(XRSLAM_TEST "..." OFF)

# [bench 2026-08-31] 与主树同样的解耦:上游把线程化写死在 if(IOS) 里,导致
# Android 拿 OFF、iOS 拿 ON,两端是不同的执行架构而非性能差异。这里只解耦,
# 不改默认值——两者都由命令行 -D 决定,便于做单变量对比。
if(IOS AND NOT DEFINED XRSLAM_IOS_OVERRIDE)
  set(XRSLAM_IOS ON)
endif()
if(IOS AND NOT DEFINED XRSLAM_THREADING_OVERRIDE)
  set(XRSLAM_ENABLE_THREADING ON)
  #set(XRSLAM_ENABLE_DEBUG_INSPECTION OFF CACHE BOOL "..." FORCE)
endif()


if(NOT IOS)
  set(XRSLAM_PC ON)
endif()

# [PW 2026-09-16] Tracy (BSD-3, https://github.com/wolfpld/tracy, pinned v0.14.1).
# Replaces hand-rolled host-side timers: those were written twice and read wrong
# twice (a composite counter that folded three causes, and an empty-submit probe
# read as an additive per-frame cost). Tracy is the same instrumentation on every
# end we ship to -- its own capability table lists CPU zones on Windows/Linux/
# Android/OSX/iOS, and GPU zones over WebGPU on all of them, which is our Dawn
# stack. Default OFF so a normal build is unchanged.
option(XRSLAM_TRACY "Instrument with the Tracy profiler (BSD-3)" OFF)
set(TRACY_ROOT "" CACHE PATH "Tracy checkout containing public/TracyClient.cpp")
add_library(xrslam-tracy INTERFACE)
if(XRSLAM_TRACY)
  if(NOT EXISTS "${TRACY_ROOT}/public/TracyClient.cpp")
    message(FATAL_ERROR "XRSLAM_TRACY=ON but TRACY_ROOT/public/TracyClient.cpp not found: ${TRACY_ROOT}")
  endif()
  target_include_directories(xrslam-tracy INTERFACE "${TRACY_ROOT}/public")
  # TRACY_DELAYED_INIT is required on Apple platforms (Tracy manual: no linker
  # support for init-time profiling on OSX/iOS); TRACY_ON_DEMAND keeps the client
  # quiet until a profiler actually connects, so an uninstrumented run costs
  # nothing and a scripted capture can attach mid-run.
  target_compile_definitions(xrslam-tracy INTERFACE TRACY_ENABLE TRACY_ON_DEMAND)
  if(APPLE)
    # TRACY_DELAYED_INIT only: the manual says OSX/iOS have no linker support for
    # init-time profiling, so the profiler initialises on first use. Do NOT add
    # TRACY_MANUAL_LIFETIME here -- under it "the profiler does not exist until
    # StartupProfiler() is called", so nothing listens and a capture just hangs
    # on "Connecting to ...".
    target_compile_definitions(xrslam-tracy INTERFACE TRACY_DELAYED_INIT)
  endif()
endif()

add_subdirectory(xrslam)
add_subdirectory(xrslam-extra)
add_subdirectory(xrslam-interface)
add_subdirectory(xrslam-localization)

if(XRSLAM_PC)
  add_subdirectory(xrslam-pc)
endif()
# [bench 2026-09-17] The XRSLAM_IOS macro and upstream's iOS demo app arrive
# together: `#cmakedefine XRSLAM_IOS` in xrslam/cmake/version.h.in turns on the
# keyframe rule at sliding_window_tracker.cpp:211, and the same variable pulls in
# xrslam-ios/, a Swift app CMake cannot configure for a cross-compiled iOS target
# (its try-compile probe targets arm64-apple-macosx against the iPhoneOS sysroot
# and fails to load the standard library). We only ever consume the static
# libraries, so the app is separable. Default ON keeps upstream's behaviour for
# anyone who does want it.
option(XRSLAM_IOS_APP "Also build upstream's iOS demo app when XRSLAM_IOS is on" ON)
if(XRSLAM_IOS AND XRSLAM_IOS_APP)
  add_subdirectory(xrslam-ios)
endif()
if(XRSLAM_TEST)
  add_subdirectory(xrslam-test)
endif()
