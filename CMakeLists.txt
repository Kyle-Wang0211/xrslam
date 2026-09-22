# [pw] 3.13 -> 3.17:SuperBuildDepends 改用 CMAKE_CURRENT_FUNCTION_LIST_DIR(3.17 引入)。
#      Android Gradle 自带的 CMake 是 3.22.1,iOS/桌面用宿主 cmake,都满足。
cmake_minimum_required(VERSION 3.17 FATAL_ERROR)

# [pw] 原来是无条件 set(CMAKE_BUILD_TYPE Release):它会盖掉命令行 -DCMAKE_BUILD_TYPE,
#      也会盖掉 Android Gradle 传进来的 debug variant ⇒ debug 构建静默变 Release。
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
project(xrslam-superbuild)

# [pw] 仓库根的唯一权威锚点。不能用 CMAKE_SOURCE_DIR(被 add_subdirectory/FetchContent
#      嵌进 Flutter 插件时会指向插件根),也不能用 PROJECT_SOURCE_DIR(本仓每个子目录
#      都各自调了 project(),PROJECT_SOURCE_DIR 就是那个子目录本身)。
set(XRSLAM_ROOT_DIR "${CMAKE_CURRENT_LIST_DIR}" CACHE INTERNAL "xrslam repository root")

include(CMakeDependentOption)
include(SuperBuildDepends)
include(XRSlamHardening)
superbuild_option(cpp17)
superbuild_option(modern-cpp)

option(XRSLAM_DEBUG "..." ON)
option(XRSLAM_ENABLE_DEBUG_INSPECTION "..." ON)
option(XRSLAM_ENABLE_THREADING "..." OFF)
# [pw] XRSLAM_PC 原来不是 option,而是下面一句普通 `set(XRSLAM_PC ON)`。普通变量会
#      遮蔽命令行 -DXRSLAM_PC=OFF 写进 cache 的值(实测 cache=OFF 而目录作用域取值 ON),
#      于是 xrslam-pc → liteviz → imgui/glfw/glad 这条桌面 GUI 链**无法用任何方式关闭**
#      —— Android configure 会一头撞进去并硬失败在 depends-glfw-src。
#      改成真 option 后 -DXRSLAM_PC=OFF 生效,五个桌面依赖整条掉出构建图
#      (glad 的零许可声明问题随之消失)。默认值保持上游行为。
option(XRSLAM_PC "Build the desktop player (pulls in liteviz/imgui/GLFW/glad + OpenGL)" ON)
# [pw] 已删除 option(XRSLAM_PC_HEADLESS_ONLY ...) —— 全仓零消费,只生成到 version.h
#      的 #cmakedefine,名字还会误导成"存在无头 PC 构建"这条路。
# [pw] XRSLAM_IOS 现在只表示「Apple 平台构建」:静态库 + libtool 合并 + lib/iOS 输出
#      + add_subdirectory(xrslam-ios)。下面两件事以前也挂在它身上,但和平台无关,拆出来。
option(XRSLAM_IOS "Apple build: static lib + libtool merge + lib/iOS output" OFF)
option(XRSLAM_TEST "..." OFF)

# [pw] 拆分 1:XRSLAMCreate 的 slam_config_path/device_config_path 到底是什么。
#      ON  = 参数是 YAML 正文(YAML::Load)  —— 以前 iOS 走这条
#      OFF = 参数是文件路径(YAML::LoadFile) —— 以前 Android/PC 走这条
#      同一个 C ABI 却有两种契约,Dart 侧被迫分平台写。移动端统一取 ON。
option(XRSLAM_CONFIG_FROM_STRING "XRSLAMCreate takes YAML text instead of file paths" OFF)

# [pw] 拆分 2:低延迟位姿链 —— 每帧在 feature_tracker 线程上对 keymap 做 PnP 并更新
#      latest_state,外加 synchronize_keymap 和一条不同的关键帧判定规则。
#      以前只有 iOS 编得进去 ⇒ 两端位姿输出率和关键帧策略根本不同。移动端统一取 ON。
option(XRSLAM_LOWLATENCY_POSE "Per-frame keymap PnP for low-latency pose output" OFF)

if(IOS OR ANDROID)
  set(XRSLAM_CONFIG_FROM_STRING ON)
  set(XRSLAM_LOWLATENCY_POSE ON)
endif()
option(XRSLAM_ENABLE_VISUAL_LOCALIZATION "Enable visual localization support" OFF)

if(IOS)
  set(XRSLAM_IOS ON)
  #set(XRSLAM_ENABLE_DEBUG_INSPECTION OFF CACHE BOOL "..." FORCE)
endif()

# [pw] 线程化必须与 iOS 解耦。原来 set(XRSLAM_ENABLE_THREADING ON) 写在 if(IOS) 里,
#      Android(IOS 为假)拿到的是 OFF ⇒ xrslam/src/xrslam/utility/worker.h:35-42 的
#      resume() 走 #else 分支直接内联 worker_loop(),FrontendWorker 的 BA 就跑在
#      调用 XRSLAMPushSensorData 的那条线程上(相机回调线程 / Dart isolate)。
#      iOS 上则是 detail.cpp:37-38 起的两条真后台线程。这不是性能差异,是架构差异。
#      ⚠ 这是 Android 第一次真正开线程,落地后必须重新跑一遍稳定性验证。
if(IOS OR ANDROID)
  set(XRSLAM_ENABLE_THREADING ON)
endif()

# [pw] 新增 Android 分支,与 iOS 对称。已实测交叉编译出 arm64-v8a 的
#      libxrslam.so(16.8MB stripped,ELF AArch64,DT_NEEDED 仅 bionic 系统库)。
if(ANDROID)
  set(XRSLAM_ANDROID ON)
  set(XRSLAM_ENABLE_THREADING ON)
endif()

# [pw] 移动端一律不建桌面 player。原为 `if(NOT IOS) set(XRSLAM_PC ON)`,见上方 option 注释。
if(IOS OR ANDROID)
  set(XRSLAM_PC OFF)
endif()

# [pw] 浮点策略必须在任何 add_subdirectory 之前落地,否则 FetchContent 拉进来的
#      ceres / spdlog / yaml-cpp 继承不到(目录属性只对**之后**添加的子目录生效)。
#      xrslam_forbid_fast_math() 是 configure 期硬失败,见 XRSlamHardening.cmake 的说明。
xrslam_forbid_fast_math()
xrslam_apply_fp_policy()
# [pw] 条目 18 第二层:16KB 页。判据是"没有被显式关掉",不是"有没有打开" ——
#      NDK r29 不设即为 16KB,只有显式 =OFF 才会追加 -Wl,-z,max-page-size=4096。
xrslam_assert_flexible_page_sizes()

add_subdirectory(xrslam)
add_subdirectory(xrslam-extra)
add_subdirectory(xrslam-interface)
if(XRSLAM_ENABLE_VISUAL_LOCALIZATION)
  add_subdirectory(xrslam-localization)
endif()

if(XRSLAM_PC)
  add_subdirectory(xrslam-pc)
endif()
if(XRSLAM_IOS)
  add_subdirectory(xrslam-ios)
endif()
if(XRSLAM_TEST)
  add_subdirectory(xrslam-test)
endif()
