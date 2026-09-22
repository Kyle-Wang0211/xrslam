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
#else
#define PW_ZONE(name) \
    do {              \
    } while (0)
#endif
