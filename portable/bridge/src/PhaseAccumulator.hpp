#pragma once
#include "PwXrslamPhaseTiming.h"
#include <limits>
namespace pw_common_core {
// Fixed-size diagnostic arithmetic only, shared by production and host fixture.
inline void AccumulatePhase(PWXrslamPhaseTimingCounter &counter,
                            uint64_t start, uint64_t end) {
 const uint64_t max = std::numeric_limits<uint64_t>::max();
 if (counter.count == max) counter.overflow_flags |= 1;
 else ++counter.count;
 if (end < start) { counter.overflow_flags |= 2; return; }
 const uint64_t duration = end - start;
 if (duration > max - counter.total_ns) {
  counter.total_ns = max;
  counter.overflow_flags |= 1;
 } else counter.total_ns += duration;
 if (duration > counter.max_ns) counter.max_ns = duration;
}
}
