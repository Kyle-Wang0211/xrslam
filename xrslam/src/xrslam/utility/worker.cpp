#include <xrslam/utility/worker.h>

namespace xrslam {

void Worker::worker_loop() {
    while (worker_running) {
        auto l = lock();
#if defined(XRSLAM_ENABLE_THREADING)
        if (worker_running && empty()) {
            worker_cv.wait(l, [this] { return !worker_running || !empty(); });
        }
#else
        if (empty())
            break;
#endif
        if (!worker_running)
            break;
        worker_active.store(true, std::memory_order_release);
        work(l);
        worker_active.store(false, std::memory_order_release);
    }
}

} // namespace xrslam
