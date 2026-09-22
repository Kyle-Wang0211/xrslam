cmake_minimum_required(VERSION 3.13 FATAL_ERROR)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
project(xrslam-library VERSION 0.5.0 LANGUAGES CXX)

include(SuperBuildDepends)
include(XRSlamHardening)
superbuild_depend(eigen)
superbuild_depend(ceres-solver)
superbuild_depend(spdlog)

configure_file(cmake/version.h.in include/xrslam/version.h)

set(XRSLAM_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(XRSLAM_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(XRSLAM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(XRSLAM_BINARY_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/include)
set(XRSLAM_TEST_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/test)

set(PUBLIC_HEADERS
  ${XRSLAM_INCLUDE_DIR}/xrslam/inspection.h
  ${XRSLAM_INCLUDE_DIR}/xrslam/xrslam.h
  ${XRSLAM_BINARY_INCLUDE_DIR}/xrslam/version.h
)

set(PUBLIC_SOURCES
)

set(PRIVATE_HEADERS
  ${XRSLAM_SOURCE_DIR}/xrslam/common.h
  ${XRSLAM_SOURCE_DIR}/xrslam/core/detail.h
  ${XRSLAM_SOURCE_DIR}/xrslam/core/feature_tracker.h
  ${XRSLAM_SOURCE_DIR}/xrslam/core/frontend_worker.h
  ${XRSLAM_SOURCE_DIR}/xrslam/core/initializer.h
  ${XRSLAM_SOURCE_DIR}/xrslam/core/sliding_window_tracker.h
  ${XRSLAM_SOURCE_DIR}/xrslam/estimation/ceres/cost_function_validator.h
  ${XRSLAM_SOURCE_DIR}/xrslam/estimation/ceres/marginalization_factor.h
  ${XRSLAM_SOURCE_DIR}/xrslam/estimation/ceres/preintegration_factor.h
  ${XRSLAM_SOURCE_DIR}/xrslam/estimation/ceres/quaternion_parameterization.h
  ${XRSLAM_SOURCE_DIR}/xrslam/estimation/ceres/reprojection_factor.h
  ${XRSLAM_SOURCE_DIR}/xrslam/estimation/ceres/rotation_factor.h
  ${XRSLAM_SOURCE_DIR}/xrslam/estimation/marginalization_factor.h
  ${XRSLAM_SOURCE_DIR}/xrslam/estimation/preintegration_factor.h
  ${XRSLAM_SOURCE_DIR}/xrslam/estimation/preintegrator.h
  ${XRSLAM_SOURCE_DIR}/xrslam/estimation/reprojection_factor.h
  ${XRSLAM_SOURCE_DIR}/xrslam/estimation/rotation_factor.h
  ${XRSLAM_SOURCE_DIR}/xrslam/estimation/solver.h
  ${XRSLAM_SOURCE_DIR}/xrslam/estimation/state.h
  ${XRSLAM_SOURCE_DIR}/xrslam/geometry/essential.h
  ${XRSLAM_SOURCE_DIR}/xrslam/geometry/homography.h
  ${XRSLAM_SOURCE_DIR}/xrslam/geometry/lie_algebra.h
  ${XRSLAM_SOURCE_DIR}/xrslam/geometry/stereo.h
  ${XRSLAM_SOURCE_DIR}/xrslam/geometry/wahba.h
  ${XRSLAM_SOURCE_DIR}/xrslam/geometry/pnp.h
  ${XRSLAM_SOURCE_DIR}/xrslam/map/frame.h
  ${XRSLAM_SOURCE_DIR}/xrslam/map/map.h
  ${XRSLAM_SOURCE_DIR}/xrslam/map/track.h
  ${XRSLAM_SOURCE_DIR}/xrslam/utility/debug.h
  ${XRSLAM_SOURCE_DIR}/xrslam/utility/identifiable.h
  ${XRSLAM_SOURCE_DIR}/xrslam/utility/poisson_disk_filter.h
  ${XRSLAM_SOURCE_DIR}/xrslam/utility/random.h
  ${XRSLAM_SOURCE_DIR}/xrslam/utility/ransac.h
  ${XRSLAM_SOURCE_DIR}/xrslam/utility/tag.h
  ${XRSLAM_SOURCE_DIR}/xrslam/utility/unique_timer.h
  ${XRSLAM_SOURCE_DIR}/xrslam/utility/worker.h
  ${XRSLAM_SOURCE_DIR}/xrslam/utility/logger.h
  ${XRSLAM_SOURCE_DIR}/xrslam/utility/parsac.h
  ${XRSLAM_SOURCE_DIR}/xrslam/utility/imu_parsac.h
  ${XRSLAM_SOURCE_DIR}/xrslam/ar/virtual_object_manager.h
)

set(PRIVATE_SOURCES
  ${XRSLAM_SOURCE_DIR}/xrslam/config.cpp
  ${XRSLAM_SOURCE_DIR}/xrslam/core/detail.cpp
  ${XRSLAM_SOURCE_DIR}/xrslam/core/feature_tracker.cpp
  ${XRSLAM_SOURCE_DIR}/xrslam/core/frontend_worker.cpp
  ${XRSLAM_SOURCE_DIR}/xrslam/core/initializer.cpp
  ${XRSLAM_SOURCE_DIR}/xrslam/core/sliding_window_tracker.cpp
  ${XRSLAM_SOURCE_DIR}/xrslam/estimation/preintegrator.cpp
  ${XRSLAM_SOURCE_DIR}/xrslam/estimation/solver.cpp
  ${XRSLAM_SOURCE_DIR}/xrslam/geometry/essential.cpp
  ${XRSLAM_SOURCE_DIR}/xrslam/geometry/homography.cpp
  ${XRSLAM_SOURCE_DIR}/xrslam/geometry/lie_algebra.cpp
  ${XRSLAM_SOURCE_DIR}/xrslam/geometry/stereo.cpp
  ${XRSLAM_SOURCE_DIR}/xrslam/inspection.cpp
  ${XRSLAM_SOURCE_DIR}/xrslam/map/frame.cpp
  ${XRSLAM_SOURCE_DIR}/xrslam/map/map.cpp
  ${XRSLAM_SOURCE_DIR}/xrslam/map/track.cpp
  ${XRSLAM_SOURCE_DIR}/xrslam/xrslam.cpp
  ${XRSLAM_SOURCE_DIR}/xrslam/utility/debug.cpp
  ${XRSLAM_SOURCE_DIR}/xrslam/utility/worker.cpp
  ${XRSLAM_SOURCE_DIR}/xrslam/ar/virtual_object_manager.cpp
)

if(XRSLAM_ENABLE_VISUAL_LOCALIZATION)
  list(APPEND PRIVATE_HEADERS
    ${XRSLAM_SOURCE_DIR}/xrslam/localizer/base64.h
    ${XRSLAM_SOURCE_DIR}/xrslam/localizer/base64_convert.h
    ${XRSLAM_SOURCE_DIR}/xrslam/localizer/localizer.h
    ${XRSLAM_SOURCE_DIR}/xrslam/localizer/httplib.h
    ${XRSLAM_SOURCE_DIR}/xrslam/localizer/json.h
  )
  list(APPEND PRIVATE_SOURCES
    ${XRSLAM_SOURCE_DIR}/xrslam/localizer/localizer.cpp
  )
endif()

add_compile_options(-fPIC)

add_library(xrslam-core
  ${PUBLIC_HEADERS}
  ${PUBLIC_SOURCES}
  ${PRIVATE_HEADERS}
  ${PRIVATE_SOURCES}
)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(xrslam-core
    PRIVATE
      -Qunused-arguments
      -mllvm
      -inline-threshold=5000
  )
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
  target_compile_definitions(xrslam-core
    PUBLIC
      _ENABLE_EXTENDED_ALIGNED_STORAGE
    PRIVATE
      _USE_MATH_DEFINES
      _CRT_SECURE_NO_WARNINGS
  )
endif()

target_include_directories(xrslam-core
  PUBLIC
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/include>
    $<BUILD_INTERFACE:${XRSLAM_INCLUDE_DIR}>
    ${XRSLAM_BINARY_INCLUDE_DIR}
    ${XRSLAM_SOURCE_DIR}
  PRIVATE
    ${XRSLAM_SOURCE_DIR}
)

target_compile_definitions(xrslam-core
  PUBLIC
    XRSLAM_ENABLE_VISUAL_LOCALIZATION=$<BOOL:${XRSLAM_ENABLE_VISUAL_LOCALIZATION}>
)

# [pw] 此处曾加过 target_compile_definitions(... XRSLAM_ENABLE_THREADING),已撤销。
#      前提是错的:该宏一直经 cmake/version.h.in 的 #cmakedefine 下发,
#      并由 common.h:38 -> xrslam/xrslam.h -> xrslam/version.h 传到每个 TU。
#      判据"ninja -t commands | grep = 0"查的是命令行,查错了地方。
#      同时下发会造成宏重定义(-D 定义为 1,#cmakedefine 定义为空),实测 1 条告警。

target_link_libraries(xrslam-core
PUBLIC
  options::modern-cpp
  depends::eigen
  depends::spdlog
  depends::ceres-solver
  depends::opencv
)

# [pw] Blocker 02 第 1 层:内部静态库编译期隐藏。
#      xrslam-core 里没有任何 C API,整库都该是 hidden。这一层对 iOS 尤其重要:
#      iOS 走静态库 + libtool 合并,**没有我们控制的链接步骤**,链接期导出裁剪
#      够不着,只有编译期可见性能把我们自己的符号挡在 App 的导出面之外。
#      注意用 target 属性而不是 add_compile_options —— 后者是目录属性,会顺着
#      上面 superbuild_depend() 里的 add_subdirectory 泼到 ceres/spdlog 上。
xrslam_hide_internal_symbols(xrslam-core)

set(XRSLAM_LIBRARY_DIR ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "")
