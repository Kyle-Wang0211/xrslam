#ifndef XRSLAM_XRSLAM_H
#define XRSLAM_XRSLAM_H

#include <Eigen/Eigen>
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
    /// [bench 2026-09-17] The mean-reprojection gate on TT_VALID, and the focal length the
    /// authored value belongs to.
    ///
    /// The gate is `rpe_mean < sliding_window_rpe_threshold_px()`, where rpe is in REAL pixels
    /// (sliding_window_tracker.cpp applies K before taking the norm). A bare pixel number is only
    /// meaningful next to a focal length: VINS states the same gate as `ave_err * 460 > 3`, i.e.
    /// 3 px measured on a virtual f=460 camera, and its first author spells out the consequence
    /// (VINS-Mono issue #48, 2017-07-14): "we tolerate 3-pixel noise under 460 focal lengths. If
    /// you change to 920, the tolerate pixel will be 6 pixels, since you project the point to a
    /// further plane."
    ///
    /// `sliding_window_rpe_reference_focal() <= 0` keeps the bare threshold, which is what upstream
    /// does and what every run before this change did. Set it to the focal the number was authored
    /// against and the gate becomes `threshold_px * fx / reference_focal`, i.e. a fixed angle --
    /// the convention this codebase already uses two files away (`initializer.cpp:203` passes
    /// `0.7 / K(0,0)` to the homography RANSAC).
    virtual double sliding_window_rpe_threshold_px() const;
    virtual double sliding_window_rpe_reference_focal() const;
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

    virtual bool visual_localization_enable() const;
    virtual std::string visual_localization_config_ip() const;
    virtual size_t visual_localization_config_port() const;

    virtual size_t solver_iteration_limit() const;
    virtual double solver_time_limit() const;
    /// [pw 2026-09-23] Per-frame optimisation budget, replicated from OKVIS
    /// (BSD-3; see estimation/ceres/okvis_iteration_callback.h and Solver::solve()).
    ///
    /// `solver_frame_time_budget()` is OKVIS's `ceres_options.timeLimit`
    /// (config_fpga_p2_euroc.yaml:62 ships 0.035 s): the wall-clock budget, in
    /// seconds, for the sliding-window estimation of ONE frame. Every solve inside
    /// SlidingWindowTracker::track() gets `budget - (time already spent on this
    /// frame)`, clamped at 0 (ThreadedKFVio.cpp:527-530), and stops at the first
    /// iteration boundary where OKVIS's rule predicts an overrun.
    /// A NEGATIVE value (the default) registers no callback at all, i.e. the solver
    /// is byte-for-byte the pre-change one; OKVIS uses the same convention
    /// ("negative values will set an unlimited time limit", yaml:62;
    /// VioParametersReader.cpp:127 defaults to -1.0).
    ///
    /// `solver_min_iterations()` is OKVIS's `ceres_options.minIterations`
    /// (yaml:60 ships 3): iterations always performed regardless of the budget.
    /// Ceres only checks at iteration boundaries, so this bounds the tail
    /// statistically, not as a hard real-time guarantee.
    /// `solver_time_limit()` / `solver_iteration_limit()` above keep applying on top.
    virtual double solver_frame_time_budget() const;
    virtual size_t solver_min_iterations() const;

    virtual double rotation_misalignment_threshold() const;
    virtual double rotation_ransac_threshold() const;

    virtual int random() const;

    virtual bool parsac_flag() const;
    virtual double parsac_dynamic_probability() const;
    virtual double parsac_threshold() const;
    virtual double parsac_norm_scale() const;
    virtual size_t parsac_keyframe_check_size() const;

    void log_config() const;
};

class Image {
  public:
    double t;

    // [pw 2026-09-22 逐帧内参] 调研判决书 §3.7 第 2 条:平台每帧给的针孔内参。
    // has_K == false(默认)⇒ Detail::track_camera 仍走 config->camera_intrinsic(),
    // 即上游行为,逐位不变。核心其余读 K 的地方本来就读 frame->K(判决书 §3.2),
    // 相邻两帧 K 不同已被结构支持,不需要再动。
    matrix<3> K = matrix<3>::Identity();
    bool has_K = false;

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
