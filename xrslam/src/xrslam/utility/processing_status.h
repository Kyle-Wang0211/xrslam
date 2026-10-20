#ifndef XRSLAM_PROCESSING_STATUS_H
#define XRSLAM_PROCESSING_STATUS_H
#include <xrslam/version.h>
#include <cstddef>
namespace xrslam {
// Common-platform lifecycle observation adapter; not an upstream algorithm API.
struct WorkerProcessingStatus {
    bool running = false;
    bool queue_empty = false;
    bool active = false;
    bool idle() const { return running && queue_empty && !active; }
};
struct ProcessingSnapshot {
    bool core_present = false;
    bool observed = false;
    std::size_t pending_camera = 0;
    WorkerProcessingStatus feature_tracker;
    WorkerProcessingStatus frontend;
    bool quiescent() const {
        return core_present && observed && pending_camera == 0 &&
               feature_tracker.idle() && frontend.idle();
    }
};
} // namespace xrslam
#endif
