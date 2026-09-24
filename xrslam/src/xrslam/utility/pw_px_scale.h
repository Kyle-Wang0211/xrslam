#pragma once
// [xrhires 2026-09-24, experiment only, not for commit] Resolution scaling of the front end's
// hard-coded PIXEL constants, read once from the environment. Unset => every factor is exactly
// 1.0 / +0 levels / blockSize 3, and x*1.0 is exact in IEEE-754, so the official behaviour is
// reproduced bit for bit. Sites (all upstream literals): opencv_image.cpp border 20 (detect+track),
// forward-backward 0.5 px, GFTT minDistance 20 / blockSize 3, LK maxLevel 3 (opencv_image.h);
// initializer.cpp RANSAC 0.7 px (already /fx, i.e. 0.7 px at whatever resolution is fed).
#include <cmath>
#include <cstdlib>
namespace xrslam {
inline double pw_px_scale() {
    static const double s = [] {
        const char *e = std::getenv("PW_PX_SCALE");
        double v = e ? std::atof(e) : 1.0;
        return (v > 0.0) ? v : 1.0;
    }();
    return s;
}
inline int pw_lk_extra_levels() {
    static const int n = [] {
        const char *e = std::getenv("PW_LK_EXTRA_LEVELS");
        return e ? std::atoi(e) : 0;
    }();
    return n;
}
inline int pw_gftt_block() {
    static const int b = [] {
        const char *e = std::getenv("PW_GFTT_BLOCK");
        int v = e ? std::atoi(e) : 3;
        return v >= 3 ? v : 3;
    }();
    return b;
}
} // namespace xrslam
