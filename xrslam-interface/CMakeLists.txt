cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
project(xrslam-interface)

include(SuperBuildDepends)
include(XRSlamHardening)

set(XRSLAM_INTERFACE_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(XRSLAM_INTERFACE_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

IF (XRSLAM_IOS)
    message(STATUS "xrslam will build as static library")
    add_library(xrslam STATIC
        ${XRSLAM_INTERFACE_INCLUDE_DIR}/XRSLAM.h
        ${XRSLAM_INTERFACE_SOURCE_DIR}/XRSLAMInternal.cpp
        ${XRSLAM_INTERFACE_SOURCE_DIR}/XRSLAMManager.cpp)
ELSE()
    message(STATUS "xrslam will build as shared library")
    add_library(xrslam SHARED
        ${XRSLAM_INTERFACE_INCLUDE_DIR}/XRSLAM.h
        ${XRSLAM_INTERFACE_SOURCE_DIR}/XRSLAMInternal.cpp
        ${XRSLAM_INTERFACE_SOURCE_DIR}/XRSLAMManager.cpp)
ENDIF()

target_link_libraries(xrslam
  PUBLIC
    xrslam-extra-opencv-image
    xrslam-extra-yaml-config
)
# [pw] Blocker 02 第 2 层:最终共享库的链接期导出裁剪。
#      这一层才是把 .dynsym 的 6558 个 DEFINED 压下去的东西 —— 泄漏的符号绝大多数
#      来自静态链进来的 libc++ / opencv / tbb / ceres / spdlog / yaml-cpp,
#      编译期可见性开关对它们无效,只有链接期管得到。
#      ELF 走 --version-script + --exclude-libs,ALL;Apple ld64 不认这两个,走
#      -exported_symbols_list。分叉在 xrslam_restrict_exports() 里。
#      iOS(XRSLAM_IOS=ON)是 STATIC,没有链接步骤,该函数会自己 return。
xrslam_restrict_exports(xrslam)

# [pw] 条目 18 第二层的第二条线:不依赖 NDK 默认值,显式要 16KB 页对齐。
#      NDK r26 及更早没有 ANDROID_SUPPORT_FLEXIBLE_PAGE_SIZES 变量、默认 4KB,
#      上面那条 configure 期检查对它是失明的。非 Android 平台该函数自己 return。
xrslam_align_16k(xrslam)

# [pw] 第 3 层:把 xrslam-interface 自己的 TU 也编成 hidden。
#  ✅ 2026-08-23 已打开 —— 前置条件已满足:XRSLAM.h 里 18 个 C API 全部带
#     XRSLAM_API = __attribute__((visibility("default"))) __attribute__((used))。
#     顺序不能反:没有那个宏就打开这条,STV_HIDDEN 链接期无法再被 version script
#     拉回来,实测产物 DEFINED=0、一个 XRSLAM* 都不导出。
#  这一条是 **iOS 静态库路径唯一有效的收口手段** —— iOS 走 libtool 合并,
#  没有我们控制的链接步骤,--version-script / -exported_symbols_list 都够不着。
option(XRSLAM_HIDE_INTERFACE_TU
       "Compile xrslam-interface TUs with -fvisibility=hidden. REQUIRES XRSLAM.h to carry XRSLAM_API=__attribute__((visibility(\"default\"),used)) on every exported function, otherwise NOTHING is exported."
       ON)
if(XRSLAM_HIDE_INTERFACE_TU)
  xrslam_hide_internal_symbols(xrslam)
endif()

target_include_directories(xrslam
  PUBLIC
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/include>
    $<BUILD_INTERFACE:${XRSLAM_INTERFACE_INCLUDE_DIR}>
    ${XRSLAM_INTERFACE_INCLUDE_DIR}
  PRIVATE
    ${XRSLAM_INTERFACE_SOURCE_DIR}
)

if(XRSLAM_IOS)
    # for PC static_lib merge
    # add_custom_command(TARGET xrslam POST_BUILD
    #                         COMMAND ar x $<TARGET_FILE:xrslam>
    #                         COMMAND ar x $<TARGET_FILE:xrslam-extra-opencv-image>
    #                         COMMAND ar x $<TARGET_FILE:xrslam-extra-yaml-config>
    #                         COMMAND ar x $<TARGET_FILE:xrslam-core>
    #                         COMMAND ar x $<TARGET_LINKER_FILE:yaml-cpp>
    #                         COMMAND ar cr $<TARGET_FILE:xrslam> *.o
    #                         COMMAND ranlib $<TARGET_FILE:xrslam>
    #                         COMMAND rm *.o
    #                         )
    # for IOS static_lib merge
    add_custom_command(TARGET xrslam POST_BUILD
                       COMMAND libtool -static -o $<TARGET_FILE:xrslam> $<TARGET_FILE:xrslam> $<TARGET_FILE:xrslam-extra-opencv-image> $<TARGET_FILE:xrslam-extra-yaml-config> $<TARGET_FILE:xrslam-core> $<TARGET_LINKER_FILE:yaml-cpp> $<TARGET_LINKER_FILE:spdlog>
                      )
endif()

# output libs
# [pw] 删除了 file(COPY xrslam-interface/include/XRSLAM.h -> ${CMAKE_SOURCE_DIR}/include)。
#      它在 **configure 期** 执行,不重跑 cmake 就永远陈旧,而 xrslam-ios/visualizer
#      恰恰 include 的是那份副本 ⇒ iOS target 与静态库按两份不同的头编译(实测
#      XRSLAMDepthImage 24 vs 32 字节)。唯一权威头是 xrslam-interface/include/XRSLAM.h,
#      ffigen 的 headers.entry-points 也必须钉死这一份的绝对路径。
# [pw] 删除了 SET(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib[/iOS])(原 3 行,首行本身
#      就是被紧跟的 if/else 无条件覆盖的死代码)。三个理由:
#      1) CMAKE_SOURCE_DIR 在被嵌进 Flutter 插件工程时指向插件根,产物写到别人家去;
#      2) Android Gradle 并行构建 arm64-v8a / armeabi-v7a 两个 ABI 会争抢同一个输出文件;
#      3) 构建产物落进源码树(靠 .gitignore 的 lib/** 遮着)。
#      产物现在留在各自 build 目录的默认位置,由 podspec / Gradle externalNativeBuild 去取。
#      ⚠ 落地时顺手删掉遗留的 <repo>/lib/libxrslam.dylib 和 <repo>/lib/iOS/,
#        否则旧产物会被误当成新的。任何硬编码 <repo>/lib 路径的脚本也要跟着改。

set(XRSLAM_INTERFACE_LIBRARY_DIR ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "")
