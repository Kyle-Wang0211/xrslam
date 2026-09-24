#pragma once

// [PW 2026-09-16] One include for profiler zones, so the instrumentation is the
// same source on every end this engine ships to.
//
// Tracy (BSD-3, https://github.com/wolfpld/tracy, pinned v0.14.1) is switched on
// by the CMake option XRSLAM_TRACY. When it is off these macros expand to
// nothing and the Tracy headers need not be present, so an uninstrumented build
// is unchanged.
//
// Why a profiler instead of more hand-rolled timers: the hand-rolled ones were
// written twice and misread twice -- a counter that folded three unrelated
// causes into one number, and an "empty submit" probe treated as an additive
// per-frame cost when it measures a synchronised wake-up. Tracy computes the
// interval statistics itself, and its own capability table lists CPU zones on
// Windows/Linux/Android/OSX/iOS plus GPU zones over WebGPU on all of them, which
// is the Dawn stack this engine already uses.

#if defined(TRACY_ENABLE)
#include <tracy/Tracy.hpp>
#define PW_ZONE(name) ZoneScopedN(name)
#elif defined(PW_ZONE_STATS)
// [xrhires 2026-09-24, measurement only, not for commit] Mac replay timing without Tracy:
// every PW_ZONE site keeps its own list of wall-clock durations (ms); pw_zone_stats_dump()
// (feature_tracker.cpp) writes n/mean/p50/p90/p99/max per zone. Engine logic is untouched:
// the macro only adds a function-local static and an RAII timer at the existing zone sites.
#include <chrono>
#include <mutex>
#include <vector>
namespace pw_zs {
struct Site {
    const char *name;
    std::mutex m;
    std::vector<float> ms;
    Site *next = nullptr;
    explicit Site(const char *n);
};
inline Site *&head() {
    static Site *h = nullptr;
    return h;
}
inline std::mutex &reg_mutex() {
    static std::mutex m;
    return m;
}
inline Site::Site(const char *n) : name(n) {
    std::lock_guard<std::mutex> g(reg_mutex());
    next = head();
    head() = this;
}
struct Timer {
    Site &s;
    std::chrono::steady_clock::time_point t0;
    explicit Timer(Site &site) : s(site), t0(std::chrono::steady_clock::now()) {}
    ~Timer() {
        float d = std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - t0).count();
        std::lock_guard<std::mutex> g(s.m);
        s.ms.push_back(d);
    }
};
} // namespace pw_zs
#define PW_ZS_CAT2(a, b) a##b
#define PW_ZS_CAT(a, b) PW_ZS_CAT2(a, b)
#define PW_ZONE(name)                                                  \
    static pw_zs::Site PW_ZS_CAT(pw_zs_site_, __LINE__)(name);         \
    pw_zs::Timer PW_ZS_CAT(pw_zs_timer_, __LINE__)(PW_ZS_CAT(pw_zs_site_, __LINE__))
#else
#define PW_ZONE(name) \
    do {              \
    } while (0)
#endif
