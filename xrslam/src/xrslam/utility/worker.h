#ifndef XRSLAM_WORKER_H
#define XRSLAM_WORKER_H

#include <xrslam/common.h>

namespace xrslam {

class Worker {
  public:
    // [pw 2026-09-02] 上游是 `Worker() {}`,而 std::atomic<bool> 的默认构造在
    // C++17 下不做值初始化(C++20 才改)。本工程按 -std=gnu++17 编译,于是
    // worker_running 在 start() 之前是不确定值,~Worker()->stop() 会据此决定
    // 要不要 join 一条从未启动的线程。现有调用点在构造后立刻 start()(见
    // detail.cpp:35-36),所以只有构造中途抛异常才踩得到——顺手补上。
    Worker() : worker_running(false) {}

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
            // 停机时必须同时叫醒卡在闸口上的生产者,否则 join() 永远等不到。
            space_cv.notify_all();
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

    // [pw 2026-09-02] 生产者侧闸口。
    //
    // threading 关时,resume() 直接在生产者线程上跑完 worker_loop(),这是整个
    // 设计里唯一的流控,队列深度恒 ≤1。threading 开之后那道流控消失,而
    // 队列没有任何上界(全树无 capacity/drop 概念),相机 30fps 推、后端一旦
    // 慢过 33ms/帧就无界增长,每个元素还拖着一整帧图像。
    //
    // 这里补回流控,但**只阻塞不丢帧**:队列满时生产者等消费者取走。最坏情况
    // 就是退化成 threading 关时的同步行为,不会比现役更差,也永远不会缺帧。
    //
    // 调用约定:必须持有本 Worker 的 worker_mutex(即 lock() 返回的那把)调用,
    // 且调用点不得持有任何其它 SLAM 锁——否则会与消费者抢同一把锁而死锁。
    // 现有两个调用点均满足:FeatureTracker::track_frame 由 Detail::track_imu
    // 调用(不持锁),FrontendWorker::issue_frame 在 synchronized(map) 块之外
    // 被调用(feature_tracker.cpp:153,彼时 l 已在第 38 行解开)。
    void await_capacity(std::unique_lock<std::mutex> &l) {
#if defined(XRSLAM_ENABLE_THREADING)
        const size_t cap = capacity();
        if (cap == 0)
            return; // 0 = 不设上界,保持上游行为
        space_cv.wait(l, [this, cap] {
            return !worker_running || pending_locked() < cap;
        });
#else
        (void)l; // threading 关时 worker_loop 同步跑完,深度恒 ≤1,无需闸口
#endif
    }

    // 消费者取走一个元素后调用,唤醒可能卡在闸口上的生产者。
    void notify_space() {
#if defined(XRSLAM_ENABLE_THREADING)
        space_cv.notify_all();
#endif
    }

    virtual bool empty() const = 0;
    virtual void work(std::unique_lock<std::mutex> &l) = 0;

    // 队列上界;0 表示不设上界(默认,与上游行为一致)。
    virtual size_t capacity() const { return 0; }
    // 当前排队深度。capacity() 返回非 0 的派生类必须重写。
    // 调用时保证已持有 worker_mutex。
    virtual size_t pending_locked() const { return 0; }

  protected:
    std::atomic<bool> worker_running;

  private:
    void worker_loop();

#if defined(XRSLAM_ENABLE_THREADING)
    std::thread worker_thread;
    std::condition_variable worker_cv;
    std::condition_variable space_cv;
#endif
    mutable std::mutex worker_mutex;
};

} // namespace xrslam

#endif // XRSLAM_WORKER_H
