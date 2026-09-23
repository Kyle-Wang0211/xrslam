#!/bin/bash
# ============================================================================
# 在 macOS 上把本树(4beb1a9 谱系,ceres 1.14 pinned / eigen 3.3.7)编成无头回放
# 用的 libxrslam.dylib,并编出 pw_euroc_runner。口径尽量对齐 iOS gpufe_nothread 臂:
#   · XRSLAM_ENABLE_THREADING=OFF(nothread 臂)
#   · CXXFLAGS 与 build_gpufe_nothread.sh 逐字相同(-ffp-contract=off -fno-fast-math
#     -fchar8_t -Dceres=pw_xrslam_ceres_1_14)
#   · eigen/ceres/spdlog/yaml-cpp 与 iOS 同一份本地 tarball
#   · ceres 开关抄 build-gpufe-nothread/CMakeCache.txt:SUITESPARSE=OFF CXSPARSE=OFF EIGENSPARSE=ON
#     (宿主机会摸到 brew 的 SuiteSparse 然后死在 ceres 1.14 的老 FindTBB;iOS 交叉编译摸不到)
#   🔴 唯一对不齐:OpenCV(iOS 4.0.1 framework vs brew 5.0.0),
#      见记忆 reference_xrslam_pc_build_recipe_20260921。
# 顶层 CMake 在非 iOS 下强制 XRSLAM_PC=ON ⇒ 会为 xrslam-pc/player 拉 argparse/liteviz
# (git,本机网络会挂死)。这里用 FETCHCONTENT_SOURCE_DIR_* 指向两个空 stub,
# 目标能被定义但我们从不构建它。
#
#   build_pc_headless.sh <src_tree> <build_dir> [runner_src.cpp]
# ============================================================================
set -euo pipefail
X="$1"; B="$2"; RUNNER="${3:-$X/pw_tools/regression/euroc_runner.cpp}"
D="${XRSLAM_DEPS:-$HOME/Developer/xrslam-deps-tarballs}"
STUBS="${PW_CMAKE_STUBS:?set PW_CMAKE_STUBS to the dir holding argparse/ and liteviz/ stub CMake projects}"
OCV="${OpenCV_DIR:-/opt/homebrew/lib/cmake/opencv5}"
OCV_INC="${OpenCV_INC:-/opt/homebrew/opt/opencv/include/opencv5}"
OCV_LIB="${OpenCV_LIB:-/opt/homebrew/opt/opencv/lib}"
CXXFLAGS="-ffp-contract=off -fno-fast-math -fchar8_t -Dceres=pw_xrslam_ceres_1_14"

# brew OpenCV 5 删掉了 4.x 的 C 头 <opencv2/calib3d/calib3d_c.h>;本树只用它的 CV_EPNP
# 一个常量(pnp.h:25,64;== cv::SOLVEPNP_EPNP == 1,同一算法)。不改源码,给编译器
# 一个 include shim(主树 d052dc3 是把 pnp.h 改成 <opencv2/geometry/3d.hpp>,那属于源码迁移)。
SHIM="$B.ocv_shim"; mkdir -p "$SHIM/opencv2/calib3d"
cat > "$SHIM/opencv2/calib3d/calib3d_c.h" <<'H'
/* shim: OpenCV 5 has no calib3d_c.h; the tree only needs CV_EPNP (OpenCV 4.0.1: CV_EPNP=1 == cv::SOLVEPNP_EPNP). */
#ifndef PW_CALIB3D_C_SHIM_H
#define PW_CALIB3D_C_SHIM_H
#include <opencv2/calib3d.hpp>
#ifndef CV_EPNP
#define CV_EPNP cv::SOLVEPNP_EPNP
#endif
#endif
H
CXXFLAGS="$CXXFLAGS -I$SHIM"

cmake -S "$X" -B "$B" -G Ninja \
  -D CMAKE_BUILD_TYPE=Release -D CMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -D XRSLAM_ENABLE_THREADING=OFF \
  -D SUITESPARSE=OFF -D CXSPARSE=OFF -D EIGENSPARSE=ON \
  -D CMAKE_CXX_FLAGS="$CXXFLAGS" \
  -D OpenCV_DIR="$OCV" \
  -D FETCHCONTENT_SOURCE_DIR_DEPENDS-EIGEN="$D/eigen-3.3.7" \
  -D FETCHCONTENT_SOURCE_DIR_DEPENDS-CERES-SOLVER="$D/ceres_pinned" \
  -D FETCHCONTENT_SOURCE_DIR_DEPENDS-SPDLOG="$D/spdlog" \
  -D FETCHCONTENT_SOURCE_DIR_DEPENDS-YAML-CPP="$D/yamlcpp" \
  -D FETCHCONTENT_SOURCE_DIR_DEPENDS-ARGPARSE="$STUBS/argparse" \
  -D FETCHCONTENT_SOURCE_DIR_DEPENDS-LITEVIZ="$STUBS/liteviz" \
  -D FETCHCONTENT_FULLY_DISCONNECTED=ON > "$B.configure.log" 2>&1 \
  || { echo "configure 失败,见 $B.configure.log"; tail -30 "$B.configure.log"; exit 1; }
ninja -C "$B" -j "${JOBS:-8}" xrslam > "$B.build.log" 2>&1 \
  || { echo "build 失败,见 $B.build.log"; tail -30 "$B.build.log"; exit 1; }

# runner 没有 CMake 目标(同主树 pw_tools 的做法),手编。
[ -f "$RUNNER" ] || { echo "LIB_OK (runner 源不存在,跳过) $B"; exit 0; }
IO="$X/xrslam-pc/player/src/IO"
c++ -std=c++17 -O2 $CXXFLAGS -o "$B/pw_euroc_runner" \
  "$RUNNER" "$IO/dataset_reader.cpp" "$IO/euroc_dataset_reader.cpp" \
  "$IO/async_dataset_reader.cpp" "$IO/tum_dataset_reader.cpp" \
  -I"$X/xrslam-interface/include" -I"$IO" -I"$X/xrslam-pc/player/src" \
  -I"$X/xrslam-extra/include" -I"$X/xrslam/include" -I"$B/xrslam/include" \
  -I"$D/eigen-3.3.7" -I"$D/yamlcpp/include" -I"$OCV_INC" \
  -L"$X/lib" -lxrslam \
  -L"$OCV_LIB" -lopencv_core -lopencv_imgcodecs -lopencv_imgproc \
  -Wl,-rpath,"$X/lib" -Wl,-rpath,"$OCV_LIB"
# 上游 xrslam-interface/CMakeLists.txt 把 dylib 放到 <src>/lib/(LIBRARY_OUTPUT_PATH),不在 build 目录里。
echo "BUILD_OK $B/pw_euroc_runner"
shasum -a 256 "$X/lib/libxrslam.dylib" "$B/pw_euroc_runner"
