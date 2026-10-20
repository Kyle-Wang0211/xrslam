#ifndef XRSLAM_WORKER_H
#define XRSLAM_WORKER_H

#include <xrslam/common.h>
#include <xrslam/utility/processing_status.h>

namespace xrslam {

class Worker {
  public:
    Worker() : worker_running(false) {}

    virtual ~Worker() { stop(); }

    void start() {
        auto l = lock();
        worker_running = true;
#if defined(XRSLAM_ENABLE_THREADING)
        worker_thread = std::thread(&Worker::worker_loop, this);
#endif
    }

    void stop() {
#if defined(XRSLAM_ENABLE_THREADING)
        std::thread thread_to_join;
#endif
        {
            auto l = lock();
            worker_running = false;
#if defined(XRSLAM_ENABLE_THREADING)
            // A worker may request its own stop; only its external owner joins.
            if (worker_thread.joinable() &&
                worker_thread.get_id() != std::this_thread::get_id()) {
                thread_to_join = std::move(worker_thread);
            }
#endif
        }
#if defined(XRSLAM_ENABLE_THREADING)
        worker_cv.notify_all();
        if (thread_to_join.joinable())
            thread_to_join.join();
#endif
    }

    std::unique_lock<std::mutex> lock() const {
        return std::unique_lock(worker_mutex);
    }

    void resume(std::unique_lock<std::mutex> &l) {
        l.unlock();
#if defined(XRSLAM_ENABLE_THREADING)
        worker_cv.notify_all();
#else
        worker_loop();
#endif
    }

    // Both queue mutexes are held at once. Try-locks never wait behind work,
    // so the headless waiter can always observe cancellation/deadlines.
    static bool ObserveProcessingPair(const Worker &first, const Worker &second,
                                      WorkerProcessingStatus &first_status,
                                      WorkerProcessingStatus &second_status) {
        if (&first == &second) return false;
        std::unique_lock<std::mutex> a(first.worker_mutex, std::try_to_lock);
        if (!a.owns_lock()) return false;
        std::unique_lock<std::mutex> b(second.worker_mutex, std::try_to_lock);
        if (!b.owns_lock()) return false;
        first_status = {first.worker_running.load(), first.empty(),
                        first.worker_active.load(std::memory_order_acquire)};
        second_status = {second.worker_running.load(), second.empty(),
                         second.worker_active.load(std::memory_order_acquire)};
        return true;
    }

    virtual bool empty() const = 0;
    virtual void work(std::unique_lock<std::mutex> &l) = 0;

  protected:
    std::atomic<bool> worker_running;

  private:
    void worker_loop();
    // True before work can remove a queue item; false only after work returns.
    // A false observation under worker_mutex cannot race a new work start.
    std::atomic<bool> worker_active{false};

#if defined(XRSLAM_ENABLE_THREADING)
    std::thread worker_thread;
    std::condition_variable worker_cv;
#endif
    mutable std::mutex worker_mutex;
};

} // namespace xrslam

#endif // XRSLAM_WORKER_H
