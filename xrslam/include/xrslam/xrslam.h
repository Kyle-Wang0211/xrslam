#ifndef XRSLAM_XRSLAM_H
#define XRSLAM_XRSLAM_H

#include <Eigen/Eigen>
#include <cmath>
#include <memory>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <xrslam/version.h>
#include <chrono>
#include <thread>

namespace xrslam {

template <int Rows = Eigen::Dynamic, int Cols = Rows, bool UseRowMajor = false, typename T = double>
using matrix = typename std::conditional<
    Rows != 1 && Cols != 1,
    Eigen::Matrix<T, Rows, Cols, UseRowMajor ? Eigen::RowMajor : Eigen::ColMajor>,
    Eigen::Matrix<T, Rows, Cols>>::type;

template <int Dimension = Eigen::Dynamic, bool RowVector = false, typename T = double>
using vector =
    typename std::conditional<RowVector, matrix<1, Dimension, false, T>,
                              matrix<Dimension, 1, false, T>>::type;

using quaternion = Eigen::Quaternion<double>;

using matrix6 = matrix<6, 6>;
using matrix3 = matrix<3, 3>;
using matrix2 = matrix<2, 2>;
using matrix1 = matrix<1, 1>;
using matrix6x3 = matrix<6, 3>;
using matrix6x1 = matrix<6, 1>;
using matrixX = matrix<-1, -1>;

using vector6 = vector<6>;
using vector5 = vector<5>;
using vector4 = vector<4>;
using vector3 = vector<3>;
using vector2 = vector<2>;
using vector1 = vector<1>;
using vectorX = vector<-1>;

struct Pose {
    Pose() {
        q.setIdentity();
        p.setZero();
    }
    quaternion q;
    vector<3> p;
};

using OutputPose = Pose;
using uchar = unsigned char;

struct OutputState {
    double t;
    quaternion q;
    vector<3> p;
    vector<3> v;
    vector<3> bg;
    vector<3> ba;
};

struct OutputObject {
    quaternion q;
    vector<3> p;
    int isolated;
};

// Metric depth map (e.g. from LiDAR), registered to the color/SLAM frame's FOV but
// typically at a lower resolution. Used to seed/constrain landmark inverse depth.
struct DepthMap {
    std::vector<float> data;   // depth in metres, row-major; <= 0 means invalid
    int width = 0;             // depth-map resolution
    int height = 0;
    int color_width = 0;       // color/SLAM frame resolution this depth registers to
    int color_height = 0;
    double t = 0;

    // Sample depth (metres) at a pixel of the color/SLAM frame; 0 if invalid/out of range.
    double depth_at(double u, double v) const {
        if (width <= 0 || height <= 0 || color_width <= 0 || color_height <= 0)
            return 0.0;
        long du = std::lround(u * (double)width / (double)color_width);
        long dv = std::lround(v * (double)height / (double)color_height);
        if (du < 0 || du >= width || dv < 0 || dv >= height)
            return 0.0;
        float d = data[(size_t)dv * (size_t)width + (size_t)du];
        return d > 0.0f ? (double)d : 0.0;
    }
};

class Config {
  public:
    virtual ~Config();

    virtual vector<2> camera_resolution() const = 0;
    virtual matrix<3> camera_intrinsic() const = 0;
    virtual vector<4> camera_distortion() const = 0;
    virtual quaternion camera_to_body_rotation() const = 0;
    virtual vector<3> camera_to_body_translation() const = 0;
    virtual size_t camera_distortion_flag() const = 0;
    virtual double camera_time_offset() const = 0;
    virtual quaternion imu_to_body_rotation() const = 0;
    virtual vector<3> imu_to_body_translation() const = 0;

    virtual matrix<2> keypoint_noise_cov() const = 0;
    virtual matrix<3> gyroscope_noise_cov() const = 0;
    virtual matrix<3> accelerometer_noise_cov() const = 0;
    virtual matrix<3> gyroscope_bias_noise_cov() const = 0;
    virtual matrix<3> accelerometer_bias_noise_cov() const = 0;

    virtual quaternion output_to_body_rotation() const;
    virtual vector<3> output_to_body_translation() const;

    virtual size_t sliding_window_size() const;
    virtual size_t sliding_window_subframe_size() const;
    virtual size_t sliding_window_force_keyframe_landmarks() const;
    virtual size_t sliding_window_tracker_frequent() const;

    virtual double feature_tracker_min_keypoint_distance() const;
    virtual size_t feature_tracker_max_keypoint_detection() const;
    virtual size_t feature_tracker_max_init_frames() const;
    virtual size_t feature_tracker_max_frames() const;
    virtual double feature_tracker_clahe_clip_limit() const;
    virtual size_t feature_tracker_clahe_width() const;
    virtual size_t feature_tracker_clahe_height() const;
    virtual bool feature_tracker_predict_keypoints() const;

    virtual size_t initializer_keyframe_num() const;
    virtual size_t initializer_keyframe_gap() const;
    virtual size_t initializer_min_matches() const;
    virtual double initializer_min_parallax() const;
    virtual size_t initializer_min_triangulation() const;
    virtual size_t initializer_min_landmarks() const;
    virtual bool initializer_refine_imu() const;

#if XRSLAM_ENABLE_VISUAL_LOCALIZATION
    virtual bool visual_localization_enable() const;
    virtual std::string visual_localization_config_ip() const;
    virtual size_t visual_localization_config_port() const;
#endif

    virtual size_t solver_iteration_limit() const;
    virtual double solver_time_limit() const;

    virtual double rotation_misalignment_threshold() const;
    virtual double rotation_ransac_threshold() const;

    virtual int random() const;

    virtual bool parsac_flag() const;
    virtual double parsac_dynamic_probability() const;
    virtual double parsac_threshold() const;
    virtual double parsac_norm_scale() const;
    virtual size_t parsac_keyframe_check_size() const;
    virtual bool parsac_fast_imu_pnp() const;

    // LiDAR/depth fusion. When disabled the engine behaves exactly as monocular VIO.
    virtual bool depth_fusion_enabled() const;
    virtual double depth_prior_weight() const;

    // [pw] 条目 17:内部工作集的内存硬上限。全部以"元素个数"计,0 = 不设上限。
    //      裁剪策略一律是丢最老(FIFO),每次裁剪都计数(见 get_memory_budget_stats)。
    //      ⚠ runtime_max_pending_camera_frames 默认 0(= 保持改动前行为):
    //        它是唯一会在 VIO 轨迹上留下永久空洞的上限,需要产品侧签决后再开。
    virtual size_t runtime_max_raw_imu_queue() const;
    virtual size_t runtime_max_pending_imu() const;
    virtual size_t runtime_max_frontal_imu() const;
    virtual size_t runtime_max_pending_camera_frames() const;
    virtual size_t runtime_max_tracking_map_frames() const;
    virtual size_t runtime_max_pending_frame_ids() const;

    // [pw] 条目 08:IMU 时基实测窗口 + 成簇上报检测。
    virtual size_t imu_timing_warmup_samples() const;
    virtual size_t imu_timing_window_samples() const;
    virtual double imu_timing_batch_gap_ratio() const;

    void log_config() const;
};

// ---------------------------------------------------------------------------
// [pw] 核内只读遥测。三个 get_* 都是线程安全的(逐字段 relaxed 原子读),
//      但**不是**跨字段的一致快照:同一次调用里不同字段可能来自相邻两帧。
//      为 C API 健康状态机准备,核内不改 C API。
// ---------------------------------------------------------------------------

// 「这一帧到底有没有用上 IMU」
struct FrameHealth {
    unsigned long long frame_seq;   // 每帧 +1;两次读到同一个值 = 期间没有新帧
    double frame_t;                 // 该帧的图像时间戳 [s]
    int imu_samples;                // 该帧收到的**真实** IMU 样本数(补桩之前)。
                                    // **0 = 该帧完全没有惯性观测**,静默退化成纯单目。
                                    // -1 = 还没有任何帧到达跟踪器。
    int imu_samples_integrated;     // 实际进入预积分的样本数。⚠ 与上一个字段的差
                                    // 就是 feature_tracker 补进去的**合成**样本
                                    // (上一帧最后一个样本改时间戳)。
                                    // imu_samples==0 而本字段==1 = 纯外推,不是观测。
    int detected_keypoints;         // 该帧检出后的关键点总数(含跟踪继承来的)
    int tracked_keypoints;          // 从上一帧成功传递过来的关键点数
                                    // (LK 之后,再经本质矩阵 + 泊松盘筛完的存活数)
    int inlier_keypoints;           // LK 存活里通过本质矩阵几何校验的内点数
    int mapped_landmarks;           // 滑窗最新帧看到的 VALID&TRIANGULATED&STATIC 轨迹数;
                                    // -1 = 后端还没跑起来
    unsigned long long imu_starved_frames; // 累计出现 imu_samples==0 的帧数
};
void get_frame_health(FrameHealth &out);

// 不想引入结构体的调用方可以直接前向声明这一个(与 get_depth_fusion_stats 同款)。
void get_frame_health_counters(int &imu_samples, int &tracked_keypoints,
                               int &inlier_keypoints, int &detected_keypoints);

// IMU 时基的实测结果(条目 08)
struct ImuTiming {
    unsigned long long samples;   // 已进入核心的 IMU 样本数
    bool warmed_up;               // false 时下面的实测值全部无效
    double measured_dt_median;    // 实测相邻 Δ 的中位数(成簇时取批内那一簇)[s]
    double measured_rate_hz;      // 1 / measured_dt_median
    double jitter_cv;             // 批内 Δ 的变异系数
    bool batched;                 // Δ 直方图双峰 ⇒ 成簇(批量)上报
    double batch_period;          // 批与批之间的中位间隔 [s](非成簇为 0)
    double batch_size;            // 估计的每批样本数(非成簇为 0)
    bool fabricated_uniform;      // 到达成簇(或时间戳双峰)且 Δ 的 CV < 1e-3
                                  // ⇒ 时间戳疑似等距伪造
    unsigned long long nonmonotonic_samples; // Δ<=0 的样本数(回退/重复时间戳)

    // [pw] 到达时钟侧。**这一路才是"成簇上报"的真判据**:一台 200Hz 连续采样、
    //      每 20ms 交付一批的手机,时间戳序列是连续无缺口的,batched(上面那个,
    //      看时间戳直方图)恒为 false —— 只有样本"什么时候到手"里才有痕迹。
    bool arrival_realtime;        // false = 样本几乎全部瞬间到达 ⇒ 离线回放,
                                  //         下面两个字段无意义,别用来判设备行为
    bool arrival_batched;         // arrival_realtime 且到达 Δ 呈"多个≈0 + 一个大"
    double arrival_burst_ratio;   // 到达 Δ < 0.25×标称间隔的比例
    double arrival_batch_size;    // 估计的每批样本数 = 1/(1-burst_ratio)
    double arrival_batch_period;  // 批与批之间到达间隔的中位数 [s]
};
void get_imu_timing(ImuTiming &out);

// 内部工作集的深度 / 高水位 / 裁剪计数(条目 17)
struct MemoryBudgetStats {
    // 当前深度
    unsigned long long raw_gyro_queue;
    unsigned long long raw_accel_queue;
    unsigned long long pending_imu_queue;
    unsigned long long frontal_imu_queue;
    unsigned long long pending_camera_frames;
    unsigned long long tracker_frame_queue;
    unsigned long long tracking_map_frames;
    unsigned long long tracking_map_tracks;
    unsigned long long pending_frame_ids;
    unsigned long long sliding_window_frames;
    unsigned long long sliding_window_tracks;
    // 历史高水位
    unsigned long long hw_raw_gyro;
    unsigned long long hw_raw_accel;
    unsigned long long hw_pending_imu;
    unsigned long long hw_frontal_imu;
    unsigned long long hw_pending_camera_frames;
    unsigned long long hw_tracker_frame_queue;
    unsigned long long hw_tracking_map_frames;
    // 累计裁剪(全部是丢最老)
    unsigned long long dropped_raw_gyro;
    unsigned long long dropped_raw_accel;
    unsigned long long dropped_pending_imu;
    unsigned long long dropped_frontal_imu;
    unsigned long long dropped_pending_camera_frames;
    unsigned long long dropped_tracker_frame_queue;
    unsigned long long dropped_tracking_map_frames;
    unsigned long long dropped_pending_frame_ids;
    // detail.cpp 里 gyroscopes.clear() 的可观测性
    unsigned long long gyro_buffer_clears;
    unsigned long long gyro_samples_dropped_by_clear;
    // track_accelerometer 的镜像缺口:没有陀螺下界时加速度样本被整个丢掉
    unsigned long long accel_dropped_no_bracket;
    // 预积分时间戳异常
    unsigned long long nonmonotonic_imu_dt;
    unsigned long long zero_imu_dt;
};
void get_memory_budget_stats(MemoryBudgetStats &out);

class Image {
  public:
    double t;

    // Optional depth map registered to this frame (null when depth fusion is off).
    std::shared_ptr<DepthMap> depth;

    virtual uchar *get_rawdata() const = 0;
    virtual size_t width() const = 0;
    virtual size_t height() const = 0;

    virtual size_t level_num() const { return 0; }

    virtual double evaluate(const vector<2> &u, int level = 0) const = 0;
    virtual double evaluate(const vector<2> &u, vector<2> &ddu,
                            int level = 0) const = 0;

    virtual ~Image() = default;
    virtual void preprocess(double clipLimit, int width, int height) {}
    virtual void release_image_buffer() = 0;
    virtual void detect_keypoints(std::vector<vector<2>> &keypoints,
                                  size_t max_points = 0,
                                  double keypoint_distance = 0.5) const = 0;
    virtual void track_keypoints(const Image *next_image,
                                 const std::vector<vector<2>> &curr_keypoints,
                                 std::vector<vector<2>> &next_keypoints,
                                 std::vector<char> &result_status) const = 0;
};

enum SysState { SYS_INITIALIZING = 0, SYS_TRACKING, SYS_CRASH, SYS_UNKNOWN };

// The following interfaces will be deprecated
class XRSLAM {
  public:
    struct Detail;

    XRSLAM(std::shared_ptr<Config> config);
    ~XRSLAM();

    Pose track_gyroscope(const double &t, const double &x, const double &y,
                         const double &z);
    Pose track_accelerometer(const double &t, const double &x, const double &y,
                             const double &z);
    Pose track_camera(std::shared_ptr<Image> image);
    std::tuple<double, Pose> get_latest_camera_state() const;
    SysState get_system_state() const;
    size_t create_virtual_object();
    OutputObject get_virtual_object_pose_by_id(size_t id);
    void enable_global_localization();
    void disable_global_localization();
    void query_frame();
    bool global_localization_initialized();
    std::vector<std::string> get_logger_message();

  private:
    std::unique_ptr<Detail> detail;
};

struct Timer{
    Timer(){reset();}

    void reset(){
        t1 = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    }

    double get_time(){
        double t2 = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        return t2 - t1;
    }

    void sleep(double seconds){
        std::this_thread::sleep_for(std::chrono::milliseconds((int)(seconds * 1000)));
    }

    double t1;
};

} // namespace xrslam

#endif // XRSLAM_XRSLAM_H
