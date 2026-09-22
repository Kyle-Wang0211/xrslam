# [pw] 两件与「一套核跨 iOS/Android」直接相关的加固,集中在这里,避免散落在四个
#      CMakeLists 里各写一份而漂移。
#
#   A. 符号可见性(Blocker 02)
#   B. 浮点收缩语义 + -ffast-math 渗入防线(Blocker 03 的一部分)
#
# 该文件被 cmake/Modules 加入 CMAKE_MODULE_PATH 后 include(XRSlamHardening) 使用。

include_guard(GLOBAL)

# [pw] 路径不能在 include 期算进普通变量。本文件被 4 个 CMakeLists 各 include 一次,
#      配上 include_guard(GLOBAL) 后**只有第一个 include 的目录**拿得到那些变量;
#      如果第一个是 xrslam/(比如 build-android-probe 那种自定义 driver),
#      兄弟目录 xrslam-interface/ 就是空值 —— 实测因此生成过
#      `-Xlinker --version-script -Xlinker --exclude-libs`(路径被吞),
#      lld 报 "cannot find version script --exclude-libs"。
#      所以在**函数体内**用 CMAKE_CURRENT_FUNCTION_LIST_DIR(= 本文件所在目录,
#      CMake ≥3.17)现算,不依赖任何 include 顺序或作用域继承。
macro(_xrslam_export_paths)
  get_filename_component(_xrslam_export_dir
    "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../export" ABSOLUTE)
  set(XRSLAM_VERSION_SCRIPT "${_xrslam_export_dir}/xrslam.map")
  set(XRSLAM_EXPORTED_SYMBOLS_LIST "${_xrslam_export_dir}/xrslam.exported_symbols")
endmacro()

# ---------------------------------------------------------------------------
# A1. 内部静态库:编译期隐藏
# ---------------------------------------------------------------------------
# 只加在**我们自己的** target 上,用 target 属性而不是 add_compile_options ——
# 后者是目录属性,会顺着 add_subdirectory 泼到 FetchContent 拉进来的 ceres /
# spdlog / yaml-cpp 上去。
#
# ⚠ 绝不要把这个用在 xrslam-interface 的 target 上:XRSLAM.h 里的 C API 声明
#   目前没有 __attribute__((visibility("default"))),编译期隐藏会把它们变成
#   STV_HIDDEN,而 STV_HIDDEN 的符号**链接期无法再被 version script 拉回来**
#   —— 结果是 .so 一个符号都不导出。见 XRSLAM_HIDE_INTERFACE_TU 的说明。
function(xrslam_hide_internal_symbols target)
  if(MSVC)
    return()
  endif()
  set_target_properties(${target} PROPERTIES
    C_VISIBILITY_PRESET hidden
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
  )
endfunction()

# ---------------------------------------------------------------------------
# A2. 最终共享库:链接期只放行 XRSLAM* / xrslam_*
# ---------------------------------------------------------------------------
# 这一层才是真正把 6558 个 DEFINED 压下去的东西:泄漏的符号绝大多数来自静态库
# (libc++、opencv、tbb、ceres、spdlog、yaml-cpp),编译期开关管不到它们,
# 只有链接期的导出裁剪管得到。
#
# 平台分叉是硬的:--version-script 是 GNU ld / lld 的东西,Apple ld64 不认;
# ld64 用 -exported_symbols_list。两条路都铺。
function(xrslam_restrict_exports target)
  get_target_property(_xrslam_type ${target} TYPE)
  if(NOT _xrslam_type STREQUAL "SHARED_LIBRARY" AND NOT _xrslam_type STREQUAL "MODULE_LIBRARY")
    # 静态库没有链接步骤,导出裁剪由最终消费者(Xcode 的 framework/app 链接)负责。
    return()
  endif()
  if(MSVC)
    return()
  endif()

  _xrslam_export_paths()

  # [pw] 不用 CMake 的 "LINKER:a,b" 语法。实测在 Android NDK 工具链下它会展开成
  #      `-Xlinker --version-script -Xlinker --exclude-libs -Xlinker ALL` —— 脚本路径
  #      被整个吞掉,链接器报 "cannot find version script --exclude-libs"。
  #      直接写 -Wl,xxx=path 单 token 形式,行为确定。
  #      同时做存在性检查:路径拼错时必须炸在 configure,而不是变成一条没人看的链接参数。
  if(NOT EXISTS "${XRSLAM_VERSION_SCRIPT}")
    message(FATAL_ERROR "missing version script: '${XRSLAM_VERSION_SCRIPT}'")
  endif()
  if(NOT EXISTS "${XRSLAM_EXPORTED_SYMBOLS_LIST}")
    message(FATAL_ERROR "missing exported symbols list: '${XRSLAM_EXPORTED_SYMBOLS_LIST}'")
  endif()

  if(APPLE)
    target_link_options(${target} PRIVATE
      "-Wl,-exported_symbols_list,${XRSLAM_EXPORTED_SYMBOLS_LIST}")
    set_property(TARGET ${target} APPEND PROPERTY
      LINK_DEPENDS "${XRSLAM_EXPORTED_SYMBOLS_LIST}")
  else()
    target_link_options(${target} PRIVATE
      "-Wl,--version-script=${XRSLAM_VERSION_SCRIPT}"
      # 静态库带进来的符号一律不进动态导出表。与 version script 有重叠,
      # 但两者失效模式不同(前者按输入档案,后者按符号名),留作双保险。
      "-Wl,--exclude-libs,ALL")
    set_property(TARGET ${target} APPEND PROPERTY
      LINK_DEPENDS "${XRSLAM_VERSION_SCRIPT}")
  endif()
endfunction()

# ---------------------------------------------------------------------------
# B. 浮点:显式统一收缩语义,并挡住 -ffast-math
# ---------------------------------------------------------------------------
# 实测(2026-08-23 复跑,本机,证据都是反汇编计数不是读 cmake 文件):
#
#  (1) -ffp-contract 确实是有效旋钮。`double f(a,b,c){return a*b+c;}`,aarch64 -O3:
#        off=0 条 FMA / on=1 条 / fast=1 条。
#
#  (2) **但 off 并不等于"零 FMA"**,这一条上一轮写漏了、必须记下来:
#      Eigen 的 pmadd 在 NEON 上走的是**显式 intrinsic** vfmaq_f64/vfmaq_f32
#      (PacketMath.h:1135/5110,由 EIGEN_VECTORIZE_FMA 打开)。intrinsic 是源码
#      写死的函数调用,-ffp-contract 管不着它。实测 8x8 double 矩阵乘:
#        -ffp-contract=off               → 795 条 FMA
#        -ffp-contract=off + EIGEN_DONT_VECTORIZE → 0 条
#      也就是说 off 之后剩下的 FMA **全部**来自 Eigen 的向量化路径。
#
#  (3) 而这恰恰是选 off 的理由,不是反对它的理由:
#        同一份 eig.cc,-ffp-contract=off
#          NDK clang 21 (aarch64-linux-android24) → 795
#          Apple clang 17 (-arch arm64)           → 795   ← 一致
#      intrinsic 那部分由**源码**决定(同一份 Eigen、同一个 EIGEN_VECTORIZE_FMA
#      门),两端天然一致;真正会两端漂的是**优化器自行决定**要不要融合的那部分,
#      而 off 把这部分整个拿掉。所以 off = 把不确定性从"优化器版本"移回"源码",
#      这是可复现、可 review 的。
#      ⚠ 不要把这条写成"实测 on/fast 会两端不一致":本次那个小样本上
#        on(6 vs 6)和 fast(10 vs 10)两端也相同。off 的保证是**构造性**的
#        (优化器不再有自由度),不是"测出来 on 会炸"。别把没测出来的当成测出来了。
#
#  (4) 代价是真的:ARM64 上凡是**不走 Eigen 向量化**的标量热点(pnp / parsac /
#      preintegrator 里的手写标量循环)拿不到 FMA。留成 cache 变量,性能对照时
#      可以 -DXRSLAM_FP_CONTRACT=fast 做 A/B,但默认必须是 off。
#
#  (5) ⚠ 只覆盖**我们编译的**代码。预编译的 OpenCV(iOS 官方 4.11.0 epnp.o
#      FMA 0 条 / Android 官方 5.0.0 epnp.cpp.o FMA 253 条)不在覆盖范围内,
#      那是"两端 OpenCV 版本都不同(4.11 vs 5.0)"这条更大的战线,不是本项能解的。
set(XRSLAM_FP_CONTRACT "off" CACHE STRING
    "-ffp-contract= value applied to every TU we compile (off|on|fast)")
set_property(CACHE XRSLAM_FP_CONTRACT PROPERTY STRINGS off on fast)

function(xrslam_apply_fp_policy)
  if(MSVC)
    return()
  endif()
  if(NOT XRSLAM_FP_CONTRACT MATCHES "^(off|on|fast|fast-honor-pragmas)$")
    message(FATAL_ERROR
      "XRSLAM_FP_CONTRACT must be one of off/on/fast/fast-honor-pragmas, got '${XRSLAM_FP_CONTRACT}'")
  endif()
  # 目录级:必须在 add_subdirectory 之前调用,这样 FetchContent 拉进来的
  # ceres / spdlog / yaml-cpp 也一起继承(Eigen 是 header-only,本来就在我们的 TU 里编)。
  add_compile_options(-ffp-contract=${XRSLAM_FP_CONTRACT})
endfunction()

# -ffast-math 渗入检查。
# 为什么是 FATAL 而不是"顺手去掉":-ffast-math 蕴含 -ffinite-math-only,
# clang 会把 std::isfinite/isnan 常量折叠成 true/false,而 **Ceres 的
# IsArrayValid() 正是靠它拒绝发散的步长**;一旦折叠,Ceres 会接受 NaN 的
# trust-region 步长,VIO 会以"没有报错"的方式漂掉。这种东西必须炸在 configure,
# 不能靠人去 review 编译命令行。
#
# 已知来源:ceres-solver 自带的 cmake/iOS.cmake:263
#   set(CMAKE_CXX_FLAGS_RELEASE "-DNDEBUG -O3 -fomit-frame-pointer -ffast-math ...")
# 该文件是 ceres 自己的 iOS **toolchain 文件**。实测我们从来没把它传给
# -DCMAKE_TOOLCHAIN_FILE(build-ios-a1 用的是 CMake 原生 CMAKE_SYSTEM_NAME=iOS,
# 根本没有 toolchain file),所以当前**未生效**;但只要哪天有人图省事用了它,
# 或者上游 Flutter 插件模板带进来,就会静默生效。这个 guard 就是为那天准备的。
function(xrslam_forbid_fast_math)
  set(_vars
    CMAKE_C_FLAGS CMAKE_CXX_FLAGS CMAKE_OBJCXX_FLAGS
    CMAKE_C_FLAGS_DEBUG CMAKE_CXX_FLAGS_DEBUG
    CMAKE_C_FLAGS_RELEASE CMAKE_CXX_FLAGS_RELEASE
    CMAKE_C_FLAGS_RELWITHDEBINFO CMAKE_CXX_FLAGS_RELWITHDEBINFO
    CMAKE_C_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_MINSIZEREL)
  foreach(_v IN LISTS _vars)
    if("${${_v}}" MATCHES "(-ffast-math|-Ofast|-ffinite-math-only|-funsafe-math-optimizations|-fno-honor-nans|-fno-honor-infinities)")
      message(FATAL_ERROR
        "${_v}='${${_v}}' 含 fast-math 类开关。-ffast-math 蕴含 -ffinite-math-only,"
        "会让 clang 把 std::isfinite 折叠成 true,Ceres 的 IsArrayValid() 就再也拒绝不了"
        "发散步长。已知来源:ceres-solver 的 cmake/iOS.cmake:263。请改用别的 toolchain。")
    endif()
  endforeach()
  # 目录属性里也可能被塞进来(比如上游 add_compile_options)。
  get_directory_property(_dir_opts COMPILE_OPTIONS)
  if("${_dir_opts}" MATCHES "(-ffast-math|-Ofast|-ffinite-math-only|-funsafe-math-optimizations)")
    message(FATAL_ERROR
      "目录级 COMPILE_OPTIONS='${_dir_opts}' 含 fast-math 类开关,理由同上。")
  endif()
endfunction()

# ---------------------------------------------------------------------------
# C. 16KB 页(条目 18 的第二层)
# ---------------------------------------------------------------------------
# 第一层(ELF PT_LOAD p_align ≥ 0x4000)已实测通过:NDK r29 交叉编出的
# libxrslam.so 三个 PT_LOAD 全是 0x4000。但那是**工具链默认值**,不是我们要来的
# —— 默认值随 NDK 版本变,而 Flutter 的 AGP 可以把 NDK 钉到任意版本。
# 这里把它变成我们自己的、可复现的保证,两条独立的线:
#
#  C1. 挡住唯一能把它降回 4KB 的开关。
#      实测 NDK r29 的 build/cmake/flags.cmake:35-44 是:
#        if(DEFINED ANDROID_SUPPORT_FLEXIBLE_PAGE_SIZES)
#          if(NOT ANDROID_SUPPORT_FLEXIBLE_PAGE_SIZES)
#            <加 -D__BIONIC_DEPRECATED_PAGE_SIZE_MACRO>
#            <arm64-v8a/x86_64 追加 -Wl,-z,max-page-size=4096>
#      也就是说:**不设 = 16KB(好);显式设成 OFF = 降回 4KB(坏)**。
#      所以判据不是"有没有设成 ON",而是"有没有被设成 OFF"。
#      写成 configure 期硬失败,因为它一旦发生只会表现为线上某些 16KB 页机型
#      (Pixel 8+/Android 15+)装不上,本地永远看不见。
function(xrslam_assert_flexible_page_sizes)
  if(NOT ANDROID)
    return()
  endif()
  if(DEFINED ANDROID_SUPPORT_FLEXIBLE_PAGE_SIZES
     AND NOT ANDROID_SUPPORT_FLEXIBLE_PAGE_SIZES)
    message(FATAL_ERROR
      "ANDROID_SUPPORT_FLEXIBLE_PAGE_SIZES=OFF 会让 NDK 追加 "
      "-Wl,-z,max-page-size=4096,产物在 16KB 页设备(Android 15+ / Pixel 8 起)"
      "无法加载。本仓不允许关闭它 —— 不设即为 16KB。")
  endif()
endfunction()

#  C2. 不依赖工具链默认值,自己显式要 16KB。
#      NDK r26 及更早根本没有 ANDROID_SUPPORT_FLEXIBLE_PAGE_SIZES 这个变量,
#      默认就是 4KB,C1 的检查对它完全无效(变量 NOT DEFINED ⇒ 直接放行)。
#      显式写死 max-page-size=16384 才是跨 NDK 版本可复现的那条线。
#      放在 target_link_options 上;若 NDK 自己也追加了同名参数,lld 取最后一个,
#      而 target 的 PRIVATE 链接参数排在工具链 init flags 之后。
function(xrslam_align_16k target)
  if(NOT ANDROID)
    return()
  endif()
  get_target_property(_xrslam_type ${target} TYPE)
  if(NOT _xrslam_type STREQUAL "SHARED_LIBRARY" AND NOT _xrslam_type STREQUAL "MODULE_LIBRARY")
    return()
  endif()
  target_link_options(${target} PRIVATE "-Wl,-z,max-page-size=16384")
endfunction()
