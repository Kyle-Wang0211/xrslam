#ifndef XRSLAM_WORKER_H
#define XRSLAM_WORKER_H

#include <xrslam/common.h>

namespace xrslam {

class Worker {
  public:
    Worker() {}

    virtual ~Worker() { stop(); }

    void start() {
        worker_running = true;
#if defined(XRSLAM_ENABLE_THREADING)
        worker_thread = std::thread(&Worker::worker_loop, this);
#endif
    }

    void stop() {
        if (worker_running) {
            worker_running = false;
#if defined(XRSLAM_ENABLE_THREADING)
            worker_cv.notify_all();
            worker_thread.join();
#endif
        }
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

    virtual bool empty() const = 0;
    virtual void work(std::unique_lock<std::mutex> &l) = 0;

  protected:
    // [pw] 原为 std::atomic<bool> worker_running;(默认初始化)。C++17 下 std::atomic
    //      的默认构造**不做值初始化**,值不确定;而 Worker(){} 不赋值、~Worker() 无条件
    //      调 stop() 读它。若构造完成到 start() 之间抛异常,析构就会拿着垃圾值进 stop(),
    //      在 XRSLAM_ENABLE_THREADING 打开时对未启动的 std::thread 调 join() ⇒ terminate。
    std::atomic<bool> worker_running{false};

  private:
    void worker_loop();

#if defined(XRSLAM_ENABLE_THREADING)
    std::thread worker_thread;
    std::condition_variable worker_cv;
#endif
    mutable std::mutex worker_mutex;
};

} // namespace xrslam

#endif // XRSLAM_WORKER_H
