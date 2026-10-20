#pragma once
#include <xrslam/utility/processing_status.h>
namespace pw_headless_eof {
enum class Observation { Busy, Working, Quiescent, PendingCamera, Unavailable };
enum class Outcome { Quiescent, Cancelled, PendingCamera, Unavailable };
inline Observation classify(const xrslam::ProcessingSnapshot &s) {
    if (!s.core_present) return Observation::Unavailable;
    if (!s.observed) return Observation::Busy;
    if (!s.feature_tracker.running || !s.frontend.running)
        return Observation::Unavailable;
    // Input is already sealed: these frames cannot route without new IMU.
    if (s.pending_camera) return Observation::PendingCamera;
    return s.quiescent() ? Observation::Quiescent : Observation::Working;
}
// Observe is read-only; Park is only the polling interval, never completion proof.
// The existing outer deadline invokes Cancel, exactly like explicit user cancel.
template<class Observe, class Cancel, class Park>
Outcome wait(Observe observe, Cancel cancelled, Park park) {
    for (;;) {
        if (cancelled()) return Outcome::Cancelled;
        const auto status = observe();
        if (cancelled()) return Outcome::Cancelled;
        switch (status) {
        case Observation::Quiescent: return Outcome::Quiescent;
        case Observation::PendingCamera: return Outcome::PendingCamera;
        case Observation::Unavailable: return Outcome::Unavailable;
        case Observation::Busy:
        case Observation::Working: park(); break;
        }
    }
}
} // namespace pw_headless_eof
