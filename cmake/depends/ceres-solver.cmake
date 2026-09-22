if(NOT TARGET depends::ceres-solver)
  if(NOT TARGET depends::eigen)
    message(FATAL_ERROR "depends::ceres-solver expects depends::eigen")
  endif()
  FetchContent_Declare(
    depends-ceres-solver
    GIT_REPOSITORY https://github.com/ceres-solver/ceres-solver.git
    GIT_TAG        2.2.0
  )
  FetchContent_GetProperties(depends-ceres-solver)
  if(NOT depends-ceres-solver_POPULATED)
    message(STATUS "Fetching ceres-solver sources")
    FetchContent_Populate(depends-ceres-solver)
    message(STATUS "Fetching ceres-solver sources - done")
  endif()
  # TODO: clean up parameters and avoid pollution.
  set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
  set(BUILD_BENCHMARKS OFF CACHE BOOL "" FORCE)
  set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
  set(MINIGLOG ON CACHE BOOL "" FORCE)
  set(GFLAGS OFF CACHE BOOL "" FORCE)
  set(SUITESPARSE OFF CACHE BOOL "" FORCE)
  set(CXSPARSE OFF CACHE BOOL "" FORCE)
  set(TBB OFF CACHE BOOL "" FORCE)
  set(OPENMP OFF CACHE BOOL "" FORCE)
  # [pw] LAPACK 在移动端一律关闭,且不依赖 Ceres 内部保护:
  #  - iOS: Ceres 2.2.0 CMakeLists:173 的 if(IOS) 会 update_cache_variable(LAPACK OFF),
  #    但那依赖 ios.toolchain.cmake 定义的 IOS 变量。若改用 CMake 3.14+ 原生的
  #    CMAKE_SYSTEM_NAME=iOS 交叉编译,该保护静默失效,而链入 BLAS 的 dsyrk_ 符号
  #    有 App Store 审核风险(Ceres 上游注释即为此)。
  #  - Android/鸿蒙: find_package(LAPACK) 通常找不到,Ceres 会在 :278 自动降级;
  #    但若构建机恰好装了某个 BLAS 实现,会被静默链入(许可与 ABI 均不可控)。
  #  桌面保留 ON,便于开发期做性能对照。
  if(IOS OR ANDROID OR OHOS
     OR CMAKE_SYSTEM_NAME MATCHES "iOS|watchOS|tvOS|visionOS|Android|OHOS")
    set(LAPACK OFF CACHE BOOL "" FORCE)
  else()
    set(LAPACK ON CACHE BOOL "" FORCE)
  endif()

  # [pw] 跨端数值对称:Ceres 在 Apple 平台会**自动探测并打开** ACCELERATESPARSE,
  #   而 Android 上没有 Accelerate ⇒ 两端走不同的稀疏求解器 ⇒ 同一份源码得到
  #   不同的数值结果。这与 XRSLAM_IOS 一个宏控三件事是同一类病,只是藏在依赖的
  #   自动探测里,我们那条 LAPACK 守卫覆盖不到。
  #   原则同 OpenCV 侧的 KleidiCV 取舍:**对称且慢,好过不对称且快**。
  #   留成 cache 变量以便将来做 A/B,但默认必须是 OFF。
  set(XRSLAM_CERES_ACCELERATE_SPARSE OFF CACHE BOOL
      "Allow Ceres to use Apple Accelerate sparse solvers (breaks iOS/Android numerical symmetry)")
  if(NOT XRSLAM_CERES_ACCELERATE_SPARSE)
    set(ACCELERATESPARSE OFF CACHE BOOL "" FORCE)
  endif()
  set(MINIGLOG_MAX_LOG_LEVEL 0 CACHE STRING "" FORCE)
  set(EIGEN_PREFER_EXPORTED_EIGEN_CMAKE_CONFIGURATION OFF CACHE BOOL "" FORCE)
  set(EIGEN_INCLUDE_DIR ${depends-eigen-source-dir} CACHE PATH "" FORCE)
  set(EIGEN3_INCLUDE_DIR ${depends-eigen-source-dir} CACHE PATH "" FORCE)

  set(depends-eigen3-config-dir ${depends-ceres-solver_BINARY_DIR}/eigen3-config)
  file(MAKE_DIRECTORY ${depends-eigen3-config-dir})
  file(WRITE ${depends-eigen3-config-dir}/Eigen3Config.cmake
"set(Eigen3_FOUND TRUE)
set(Eigen3_VERSION 5.0.1)
if(NOT TARGET Eigen3::Eigen)
  add_library(Eigen3::Eigen INTERFACE IMPORTED)
  set_target_properties(Eigen3::Eigen PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES \"${depends-eigen-source-dir}\")
endif()
")
  file(WRITE ${depends-eigen3-config-dir}/Eigen3ConfigVersion.cmake
"set(PACKAGE_VERSION 5.0.1)
set(PACKAGE_VERSION_COMPATIBLE TRUE)
if(PACKAGE_FIND_VERSION VERSION_EQUAL PACKAGE_VERSION)
  set(PACKAGE_VERSION_EXACT TRUE)
endif()
")
  set(Eigen3_DIR ${depends-eigen3-config-dir} CACHE PATH "" FORCE)
  add_subdirectory(${depends-ceres-solver_SOURCE_DIR} ${depends-ceres-solver_BINARY_DIR})
  add_library(depends::ceres-solver INTERFACE IMPORTED GLOBAL)
  target_include_directories(depends::ceres-solver
    INTERFACE
      ${depends-ceres-solver_BINARY_DIR}/include
      ${depends-ceres-solver_SOURCE_DIR}/internal/ceres/miniglog
      ${depends-ceres-solver_SOURCE_DIR}/include
  )
  target_link_libraries(depends::ceres-solver INTERFACE ceres depends::eigen)
  if(IOS)
    target_link_libraries(depends::ceres-solver INTERFACE "-framework Accelerate")
  endif()
  set(depends-ceres-solver-source-dir ${depends-ceres-solver_SOURCE_DIR} CACHE INTERNAL "" FORCE)
  set(depends-ceres-solver-binary-dir ${depends-ceres-solver_BINARY_DIR} CACHE INTERNAL "" FORCE)
  mark_as_advanced(depends-ceres-solver-source-dir)
  mark_as_advanced(depends-ceres-solver-binary-dir)
endif()
