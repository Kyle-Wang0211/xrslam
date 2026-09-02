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

add_subdirectory(xrslam)
add_subdirectory(xrslam-extra)
add_subdirectory(xrslam-interface)
add_subdirectory(xrslam-localization)

if(XRSLAM_PC)
  add_subdirectory(xrslam-pc)
endif()
if(XRSLAM_IOS)
  add_subdirectory(xrslam-ios)
endif()
if(XRSLAM_TEST)
  add_subdirectory(xrslam-test)
endif()
