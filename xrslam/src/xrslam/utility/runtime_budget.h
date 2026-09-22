#ifndef XRSLAM_RUNTIME_BUDGET_H
#define XRSLAM_RUNTIME_BUDGET_H

#include <xrslam/common.h>

// [pw] 条目 08 + 条目 17 的核内实现。
//
// 条目 08(IMU 时基):dt **本来就**是实测的(preintegrator.cpp 里
// `data[i+1].t - data[i].t`),yaml 只提供**连续时间**噪声密度,离散化在
// increment() 里用实测 dt 完成。所以"改成用实测 dt"这一步不适用。
// 真正剩下的风险是任务描述里的第 3 条:**批内时间戳被等距伪造**。伪造的等距
// 让每个 dt 看上去完美,离散共分散因此系统性乐观 —— 这是逐样本 dt 无法自证的
// 盲区。ImuTimingMonitor 就是为了把这个盲区变成一个可读的判据。
//
// 条目 17(内存硬上限):Android 17 会静默 kill 吃匿名内存的进程且不留堆栈。
// 这里裁的**只是内部工作集**(还没被融合的原始样本队列、跟踪图的旧帧),
// 裁剪策略一律是"丢最老"(FIFO),并且每一次裁剪都计数 + 限流告警。
// 相机帧队列(= 交付敏感)的上限默认是 0 = 不设限,保持改动前行为。

namespace xrslam {
namespace runtime {

// ---------------------------------------------------------------- 限流日志
// 只在 n = 1,2,5,10,20,50,100,200,500,... 时返回 true。
inline bool should_log_at(unsigned long long n) {
    if (n == 0)
        return false;
    unsigned long long m = n;
    while (m >= 10 && (m % 10) == 0)
        m /= 10;
    return m == 1 || m == 2 || m == 5;
}

// ------------------------------------------------------------ 计数与水位
struct BudgetCounters {
    // 当前深度(每次 push/pop 后由生产者线程写入)
    std::atomic<unsigned long long> depth_raw_gyro{0};
    std::atomic<unsigned long long> depth_raw_accel{0};
    std::atomic<unsigned long long> depth_pending_imu{0};
    std::atomic<unsigned long long> depth_frontal_imu{0};
    std::atomic<unsigned long long> depth_pending_camera_frames{0};
    std::atomic<unsigned long long> depth_tracker_frame_queue{0};
    std::atomic<unsigned long long> depth_tracking_map_frames{0};
    std::atomic<unsigned long long> depth_tracking_map_tracks{0};
    std::atomic<unsigned long long> depth_pending_frame_ids{0};
    std::atomic<unsigned long long> depth_sliding_window_frames{0};
    std::atomic<unsigned long long> depth_sliding_window_tracks{0};

    // 历史高水位(只增)
    std::atomic<unsigned long long> hw_raw_gyro{0};
    std::atomic<unsigned long long> hw_raw_accel{0};
    std::atomic<unsigned long long> hw_pending_imu{0};
    std::atomic<unsigned long long> hw_frontal_imu{0};
    std::atomic<unsigned long long> hw_pending_camera_frames{0};
    std::atomic<unsigned long long> hw_tracker_frame_queue{0};
    std::atomic<unsigned long long> hw_tracking_map_frames{0};

    // 累计裁剪数(全部是"丢最老")
    std::atomic<unsigned long long> dropped_raw_gyro{0};
    std::atomic<unsigned long long> dropped_raw_accel{0};
    std::atomic<unsigned long long> dropped_pending_imu{0};
    std::atomic<unsigned long long> dropped_frontal_imu{0};
    std::atomic<unsigned long long> dropped_pending_camera_frames{0};
    std::atomic<unsigned long long> dropped_tracker_frame_queue{0};
    std::atomic<unsigned long long> dropped_tracking_map_frames{0};
    std::atomic<unsigned long long> dropped_pending_frame_ids{0};

    // detail.cpp 里 `gyroscopes.clear()` 的可观测性(任务 1.4)
    std::atomic<unsigned long long> gyro_buffer_clears{0};
    std::atomic<unsigned long long> gyro_samples_dropped_by_clear{0};
    // [pw] track_accelerometer 的镜像缺口:没有陀螺可做插值下界时,加速度样本
    //      被**整个丢掉**(既不入队也不融合)。上游同样是静默的。
    std::atomic<unsigned long long> accel_dropped_no_bracket{0};

    // 预积分里的时间戳异常
    std::atomic<unsigned long long> nonmonotonic_imu_dt{0};
    std::atomic<unsigned long long> zero_imu_dt{0};
};

inline BudgetCounters &counters() {
    static BudgetCounters c;
    return c;
}

inline void bump_high_water(std::atomic<unsigned long long> &hw, size_t v) {
    unsigned long long want = (unsigned long long)v;
    unsigned long long cur = hw.load(std::memory_order_relaxed);
    while (want > cur &&
           !hw.compare_exchange_weak(cur, want, std::memory_order_relaxed,
                                     std::memory_order_relaxed)) {
    }
}

// 记录一个容器的当前深度并更新高水位。
inline void observe_depth(std::atomic<unsigned long long> &depth,
                          std::atomic<unsigned long long> &hw, size_t v) {
    depth.store((unsigned long long)v, std::memory_order_relaxed);
    bump_high_water(hw, v);
}

// 丢最老直到 size() <= cap。cap == 0 表示不设上限(保持改动前行为)。
// 返回被丢弃的元素数。
template <typename Deque>
size_t drop_oldest_over(Deque &d, size_t cap,
                        std::atomic<unsigned long long> &counter,
                        const char *what) {
    if (cap == 0)
        return 0;
    if (d.size() <= cap)
        return 0;
    size_t n = d.size() - cap;
    for (size_t i = 0; i < n; ++i) {
        d.pop_front();
    }
    unsigned long long total =
        counter.fetch_add(n, std::memory_order_relaxed) + n;
    if (should_log_at(total)) {
        log_warning("[pw][budget] %s 超过硬上限 %zu,丢弃最老的 %zu 个"
                    "(累计 %llu)",
                    what, cap, n, total);
    }
    return n;
}

// ------------------------------------------------- IMU 时基监视器(条目 08)
//
// 逐样本喂 add_sample(t)。预热窗口内只累计不发布;预热结束后发布实测的
// 中位 Δ / 速率 / 抖动,并做**成簇上报检测**:
//   把最近 window 个 Δ 排序,以 gap_ratio × 中位数为界切成"小簇"和"大簇"。
//   两簇同时存在 ⇒ 样本是成批(batch)到达的,大簇的中位数就是批周期。
//   若此时小簇的变异系数 CV < 1e-3(比任何真实 MEMS 的抖动都小一个量级),
//   判定为**批内时间戳等距伪造**:此时逐样本 dt 完美但没有信息,
//   离散共分散会系统性乐观、bias 估计被带偏。
class ImuTimingMonitor {
  public:
    void configure(size_t warmup, size_t window, double gap_ratio) {
        warmup_ = warmup > 0 ? warmup : 1;
        window_ = window >= 8 ? window : 8;
        gap_ratio_ = gap_ratio > 1.0 ? gap_ratio : 3.0;
    }

    void add_sample(double t);

  private:
    void recompute();

    size_t warmup_ = 200;
    size_t window_ = 512;
    double gap_ratio_ = 3.0;

    bool has_last_ = false;
    double last_t_ = 0.0;
    unsigned long long total_ = 0;
    size_t since_recompute_ = 0;
    bool warmed_ = false;
    bool batched_seen_ = false;
    bool arrival_batched_seen_ = false;
    bool fabricated_seen_ = false;

    std::deque<double> dts_;

    // [pw] 宿主到达时钟。传感器时间戳和"样本什么时候真的到手"是两件事,
    //      而"成簇上报"只在后者里留痕 —— 见 recompute() 里的说明。
    bool has_last_host_ = false;
    double last_host_ = 0.0;
    std::deque<double> host_dts_;

    static double host_now() {
        return std::chrono::duration<double>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }
};

// 监视器发布出来的快照(供 get_imu_timing 读)。
struct ImuTimingAtomics {
    std::atomic<unsigned long long> samples{0};
    std::atomic<int> warmed{0};
    std::atomic<double> dt_median{0.0};
    std::atomic<double> jitter_cv{0.0};
    std::atomic<int> batched{0};
    std::atomic<double> batch_period{0.0};
    std::atomic<double> batch_size{0.0};
    std::atomic<int> fabricated_uniform{0};
    // [pw] 到达时钟侧(真正能抓到"成簇上报"的那一路)
    std::atomic<int> arrival_realtime{1};
    std::atomic<int> arrival_batched{0};
    std::atomic<double> arrival_burst_ratio{0.0};
    std::atomic<double> arrival_batch_size{0.0};
    std::atomic<double> arrival_batch_period{0.0};
};

inline ImuTimingAtomics &imu_timing_atomics() {
    static ImuTimingAtomics a;
    return a;
}

inline void ImuTimingMonitor::add_sample(double t) {
    ++total_;
    imu_timing_atomics().samples.store(total_, std::memory_order_relaxed);
    if (!has_last_) {
        has_last_ = true;
        last_t_ = t;
        return;
    }
    double dt = t - last_t_;
    last_t_ = t;
    if (!(dt > 0.0)) {
        // 时间戳回退或重复。**不能**进直方图,否则中位数被污染。
        unsigned long long n = counters().nonmonotonic_imu_dt.fetch_add(
                                   1, std::memory_order_relaxed) +
                               1;
        if (should_log_at(n)) {
            log_warning("[pw][imu] 相邻 IMU 时间戳非递增(Δ=%.9fs),"
                        "已跳过该样本的时基统计(累计 %llu)",
                        dt, n);
        }
        return;
    }
    dts_.push_back(dt);
    while (dts_.size() > window_) {
        dts_.pop_front();
    }
    // 与 dts_ 严格并行地记一条到达时钟 Δ(同一个样本才推一次)。
    const double hnow = host_now();
    if (has_last_host_) {
        double hdt = hnow - last_host_;
        host_dts_.push_back(hdt > 0.0 ? hdt : 0.0);
        while (host_dts_.size() > window_) {
            host_dts_.pop_front();
        }
    }
    has_last_host_ = true;
    last_host_ = hnow;
    if (total_ < warmup_) {
        return; // 预热期:不发布任何实测值
    }
    if (warmed_ && ++since_recompute_ < 64) {
        return; // 预热后每 64 个样本重算一次,避免每样本排序
    }
    since_recompute_ = 0;
    recompute();
}

inline void ImuTimingMonitor::recompute() {
    if (dts_.size() < 8)
        return;
    std::vector<double> s(dts_.begin(), dts_.end());
    std::sort(s.begin(), s.end());

    const double median = s[s.size() / 2];
    const double gap_threshold = gap_ratio_ * median;

    // 小簇 = [0, n_small),大簇 = [n_small, end)
    const size_t n_small = (size_t)(
        std::lower_bound(s.begin(), s.end(), gap_threshold) - s.begin());
    const size_t n_large = s.size() - n_small;

    double small_median = median;
    double mean = 0.0, cv = 0.0;
    if (n_small > 0) {
        small_median = s[n_small / 2];
        for (size_t i = 0; i < n_small; ++i)
            mean += s[i];
        mean /= (double)n_small;
        double var = 0.0;
        for (size_t i = 0; i < n_small; ++i) {
            double d = s[i] - mean;
            var += d * d;
        }
        var /= (double)n_small;
        if (mean > 0.0)
            cv = std::sqrt(var) / mean;
    }

    // 双峰判据:两簇都要有足够样本,且小簇必须是多数
    // (否则"大簇"其实才是主周期,谈不上成批)。
    const bool batched =
        (n_large >= 2) && (n_small >= 4) && (n_large * 2 <= s.size());
    double batch_period = 0.0, batch_size = 0.0;
    if (batched) {
        batch_period = s[n_small + n_large / 2];
        batch_size = (double)n_small / (double)n_large + 1.0;
    }
    // ----------------------------------------------------------------- 到达时钟
    // [pw] 上面那套只看**传感器时间戳**的双峰判据,抓的是"传感器真的停采了"
    //      (占空比调度)。但任务描述里那台手机不是停采:它 200Hz 连续采样、
    //      只是每 20ms 才把一批交给你 —— 时间戳序列**连续无缺口**,直方图
    //      根本不双峰。⇒ 只看时间戳对"成簇上报"结构性失明。
    //      唯一留痕的地方是**样本什么时候到手**:一批 N 个样本几乎同时到达,
    //      于是 N-1 个到达 Δ ≈ 0,只有批与批之间有一个大的。
    double burst_ratio = 0.0, arrival_gap_median = 0.0;
    bool arrival_realtime = true, arrival_batched = false;
    double arrival_batch_size = 0.0;
    if (host_dts_.size() >= 8 && small_median > 0.0) {
        std::vector<double> hs(host_dts_.begin(), host_dts_.end());
        const double co_thresh = 0.25 * small_median;
        std::vector<double> gaps;
        size_t co = 0;
        for (size_t i = 0; i < hs.size(); ++i) {
            if (hs[i] < co_thresh)
                ++co;
            else
                gaps.push_back(hs[i]);
        }
        burst_ratio = (double)co / (double)hs.size();
        if (!gaps.empty()) {
            std::sort(gaps.begin(), gaps.end());
            arrival_gap_median = gaps[gaps.size() / 2];
        }
        // burst_ratio 逼近 1 = 样本几乎全部瞬间到达 ⇒ 这不是"批量上报",
        // 而是**离线回放**(xrslam-pc player / 录制数据集)。此时到达时钟
        // 不携带任何设备信息,必须显式判为不适用,否则会给 B 一个假阳性。
        if (burst_ratio > 0.95) {
            arrival_realtime = false;
        } else if (burst_ratio >= 0.4) {
            arrival_batched = true;
            arrival_batch_size = 1.0 / (1.0 - burst_ratio);
        }
    }

    // 真实 MEMS 的批内抖动 CV 通常在 1e-2 量级;1e-3 以下只可能是被写死的。
    // 判据用"到达是成簇的 ∧ 时间戳完美等距" —— 这正是任务里那台合规手机的
    // 指纹:等距是伪造的,因此离散共分散系统性乐观、bias 估计被带偏。
    const bool fabricated =
        ((arrival_realtime && arrival_batched) || batched) && (cv < 1.0e-3);

    auto &a = imu_timing_atomics();
    a.dt_median.store(small_median, std::memory_order_relaxed);
    a.jitter_cv.store(cv, std::memory_order_relaxed);
    a.batched.store(batched ? 1 : 0, std::memory_order_relaxed);
    a.batch_period.store(batch_period, std::memory_order_relaxed);
    a.batch_size.store(batch_size, std::memory_order_relaxed);
    a.fabricated_uniform.store(fabricated ? 1 : 0, std::memory_order_relaxed);
    a.arrival_realtime.store(arrival_realtime ? 1 : 0,
                             std::memory_order_relaxed);
    a.arrival_batched.store(arrival_batched ? 1 : 0, std::memory_order_relaxed);
    a.arrival_burst_ratio.store(burst_ratio, std::memory_order_relaxed);
    a.arrival_batch_size.store(arrival_batch_size, std::memory_order_relaxed);
    a.arrival_batch_period.store(arrival_gap_median,
                                 std::memory_order_relaxed);

    if (!warmed_) {
        warmed_ = true;
        a.warmed.store(1, std::memory_order_relaxed);
        log_info("[pw][imu] 时基预热结束(%llu 个样本):实测中位 Δ=%.6fs "
                 "(%.1f Hz),批内抖动 CV=%.4f,成簇上报=%s。"
                 "在此之前离散共分散用的是逐样本实测 Δ,不是 yaml 频率。",
                 total_, small_median,
                 small_median > 0.0 ? 1.0 / small_median : 0.0, cv,
                 batched ? "是" : "否");
    }
    if (arrival_realtime && arrival_batched && !arrival_batched_seen_) {
        arrival_batched_seen_ = true;
        log_warning("[pw][imu] 检测到**成簇(批量)上报**(到达时钟):约 %.1f 个"
                    "样本同批到达,批间隔中位 %.6fs,而传感器时间戳中位 Δ=%.6fs"
                    "(时间戳序列连续、直方图不双峰 ⇒ 只看时间戳看不出来)。"
                    "视觉/惯性权重比会随机型漂移,请勿假设 100Hz。",
                    arrival_batch_size, arrival_gap_median, small_median);
    }
    if (batched && !batched_seen_) {
        batched_seen_ = true;
        log_warning("[pw][imu] 传感器时间戳直方图双峰(= 传感器**停采**,"
                    "占空比调度,不同于成簇上报):批周期 %.6fs,"
                    "每批约 %.1f 个样本,批内中位 Δ=%.6fs。"
                    "视觉/惯性权重比会随机型漂移,请勿假设 100Hz。",
                    batch_period, batch_size, small_median);
    }
    if (fabricated && !fabricated_seen_) {
        fabricated_seen_ = true;
        log_warning("[pw][imu] 批内时间戳**疑似等距伪造**(批内 Δ 的 CV=%.2e "
                    "< 1e-3)。伪造的等距会让离散噪声模型系统性乐观、"
                    "bias 估计被带偏 —— 这是逐样本 dt 无法自证的盲区。",
                    cv);
    }
}

// --------------------------------------------- 每帧健康计数(配合 B / 任务 3)
struct FrameHealthAtomics {
    std::atomic<unsigned long long> seq{0};
    std::atomic<double> frame_t{0.0};
    std::atomic<int> imu_samples{-1};
    std::atomic<int> imu_samples_integrated{-1};
    std::atomic<int> detected{-1};
    std::atomic<int> tracked{-1};
    std::atomic<int> inliers{-1};
    std::atomic<int> mapped_landmarks{-1};
    std::atomic<unsigned long long> imu_starved_frames{0};
};

inline FrameHealthAtomics &frame_health_atomics() {
    static FrameHealthAtomics a;
    return a;
}

} // namespace runtime
} // namespace xrslam

#endif // XRSLAM_RUNTIME_BUDGET_H
