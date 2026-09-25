#ifndef XRSLAM_DETAIL_H
#define XRSLAM_DETAIL_H

#include <xrslam/common.h>
#include <xrslam/xrslam.h>
#include <mutex>
namespace xrslam {

class Config;
class FeatureTracker;
class Frame;
class FrontendWorker;
class Image;
class Map;
class Synchronizer;
struct MotionState;

struct XRSLAM::Detail {
    struct GyroscopeData {
        double t;
        vector<3> w;
    };
    struct AccelerometerData {
        double t;
        vector<3> a;
    };

  public:
    Detail(std::shared_ptr<Config> config);
    virtual ~Detail();

    const Config *configurations() const;

    Pose track_gyroscope(const double &t, const double &x, const double &y,
                         const double &z);
    Pose track_accelerometer(const double &t, const double &x, const double &y,
                             const double &z);
    Pose track_camera(std::shared_ptr<Image> image);

    std::tuple<double, Pose> get_latest_state() const;
    std::tuple<double, Pose> get_latest_pose();

    std::unique_ptr<FeatureTracker> feature_tracker;
    std::unique_ptr<FrontendWorker> frontend;

    SysState get_system_state() const;

    size_t create_virtual_object();
    OutputObject get_virtual_object_pose_by_id(size_t id);

    void enable_global_localization();
    void disable_global_localization();
    void query_frame();
    bool global_localization_initialized();

    // [xr-recon-chain 2026-09-25] 把给定状态(时刻 state_time、body 位姿、速度与零偏)沿引擎
    // 缓存的 IMU(imu_history_)外推到 t。外推本身只调用 propagate_state_okvis2(与 predict_pose
    // 同一个函数、同一条「积到 min(t, 最新样本)」规则);返回码与出参见 detail.cpp。
    int propagate_state_with_history(double &state_time, Pose &state_pose,
                                     MotionState &state_motion, double t,
                                     size_t *imu_samples, double *imu_first_t,
                                     double *imu_last_t);

    // imu_history_ 的样本上限(条)。100 Hz 约 164 s、200 Hz 约 82 s;超出丢最旧。
    static constexpr size_t kImuHistoryCapacity = 16384;

  private:
    void track_imu(const ImuData &imu);
    Pose predict_pose(const double &t);

    // [xr-recon-chain 2026-09-25] track_imu 收到的每个样本按到达顺序(= 时间顺序,
    // 同步器只按时间递增合成样本)再存一份。predict_pose 用的 frontal_imus 会随最新
    // 状态前移而弹掉旧样本,而后端帧定稿比最新状态晚约 0.5–1 s,外推它要的那段 IMU
    // 早已不在 frontal_imus 里 —— 所以另存。只存不算;独立一把锁(写在喂 IMU 的线程,
    // 读在调用外推的线程),不碰引擎其它锁。
    std::mutex imu_history_mutex_;
    std::deque<ImuData> imu_history_;

    std::mutex latest_mutex_;
    double latest_timestamp_ = 0.0;
    Pose latest_pose_;

    std::deque<GyroscopeData> gyroscopes;
    std::deque<AccelerometerData> accelerometers;

    std::deque<ImuData> imus;
    std::deque<std::unique_ptr<Frame>> frames;
    std::deque<ImuData> frontal_imus;

    std::shared_ptr<Config> config;
};

} // namespace xrslam

#endif // XRSLAM_DETAIL_H
