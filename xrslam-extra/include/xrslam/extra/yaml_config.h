#ifndef XRSLAM_EXTRA_YAML_CONFIG_H
#define XRSLAM_EXTRA_YAML_CONFIG_H

#include <stdexcept>
#include <xrslam/xrslam.h>

namespace xrslam::extra {

class YamlConfig : public Config {
  public:
    struct Exception : public std::runtime_error {
        Exception(const std::string &what) : std::runtime_error(what) {}
    };
    struct LoadException : public Exception {
        LoadException(const std::string &filename)
            : Exception("cannot load config " + filename) {}
    };
    struct ParseException : public Exception {
        ParseException(const std::string &message) : Exception(message) {}
    };
    struct ConfigMissingException : public Exception {
        ConfigMissingException(const std::string &config_path)
            : Exception("config \"" + config_path + "\" is mandatory") {}
    };
    struct TypeErrorException : public Exception {
        TypeErrorException(const std::string &config_path)
            : Exception("config \"" + config_path + "\" has wrong type") {}
    };

    YamlConfig(const std::string &slam_config_filename,
               const std::string &device_config_filename);
    ~YamlConfig();

    vector<2> camera_resolution() const override;
    matrix<3> camera_intrinsic() const override;
    vector<4> camera_distortion() const override;
    size_t camera_distortion_flag() const override;
    double camera_time_offset() const override;
    quaternion camera_to_body_rotation() const override;
    vector<3> camera_to_body_translation() const override;
    quaternion imu_to_body_rotation() const override;
    vector<3> imu_to_body_translation() const override;

    matrix<2> keypoint_noise_cov() const override;
    matrix<3> gyroscope_noise_cov() const override;
    matrix<3> accelerometer_noise_cov() const override;
    matrix<3> gyroscope_bias_noise_cov() const override;
    matrix<3> accelerometer_bias_noise_cov() const override;

    quaternion output_to_body_rotation() const override;
    vector<3> output_to_body_translation() const override;

    size_t sliding_window_size() const override;
    size_t sliding_window_subframe_size() const override;
    size_t sliding_window_tracker_frequent() const override;
    size_t sliding_window_force_keyframe_landmarks() const override;

    double feature_tracker_min_keypoint_distance() const override;
    size_t feature_tracker_max_keypoint_detection() const override;
    size_t feature_tracker_max_init_frames() const override;
    size_t feature_tracker_max_frames() const override;
    double feature_tracker_clahe_clip_limit() const override;
    size_t feature_tracker_clahe_width() const override;
    size_t feature_tracker_clahe_height() const override;
    bool feature_tracker_predict_keypoints() const override;

    size_t initializer_keyframe_num() const override;
    size_t initializer_keyframe_gap() const override;
    size_t initializer_min_matches() const override;
    double initializer_min_parallax() const override;
    size_t initializer_min_triangulation() const override;
    size_t initializer_min_landmarks() const override;
    bool initializer_refine_imu() const override;

#if XRSLAM_ENABLE_VISUAL_LOCALIZATION
    bool visual_localization_enable() const override;
    std::string visual_localization_config_ip() const override;
    size_t visual_localization_config_port() const override;
#endif

    size_t solver_iteration_limit() const override;
    double solver_time_limit() const override;

    bool parsac_flag() const override;
    double parsac_dynamic_probability() const override;
    double parsac_threshold() const override;
    double parsac_norm_scale() const override;
    size_t parsac_keyframe_check_size() const override;
    bool parsac_fast_imu_pnp() const override;

    bool depth_fusion_enabled() const override;
    double depth_prior_weight() const override;

    double rotation_misalignment_threshold() const override;
    double rotation_ransac_threshold() const override;

    // [pw] 条目 17 / 条目 08:内部工作集硬上限 + IMU 时基监视。
    //      yaml 键全部可选;不写就用 Config 基类的默认值。
    size_t runtime_max_raw_imu_queue() const override;
    size_t runtime_max_pending_imu() const override;
    size_t runtime_max_frontal_imu() const override;
    size_t runtime_max_pending_camera_frames() const override;
    size_t runtime_max_tracking_map_frames() const override;
    size_t runtime_max_pending_frame_ids() const override;
    size_t imu_timing_warmup_samples() const override;
    size_t imu_timing_window_samples() const override;
    double imu_timing_batch_gap_ratio() const override;

  private:
    vector<2> m_camera_resolution;
    matrix<3> m_camera_intrinsic;
    vector<4> m_camera_distortion;
    size_t m_camera_distortion_flag;
    double m_camera_time_offset;
    quaternion m_camera_to_body_rotation;
    vector<3> m_camera_to_body_translation;
    quaternion m_imu_to_body_rotation;
    vector<3> m_imu_to_body_translation;
    matrix<2> m_keypoint_noise_cov;
    matrix<3> m_gyroscope_noise_cov;
    matrix<3> m_accelerometer_noise_cov;
    matrix<3> m_gyroscope_bias_noise_cov;
    matrix<3> m_accelerometer_bias_noise_cov;

    quaternion m_output_to_body_rotation;
    vector<3> m_output_to_body_translation;

    size_t m_sliding_window_size;
    size_t m_sliding_window_subframe_size;
    size_t m_sliding_window_tracker_frequent;
    size_t m_sliding_window_force_keyframe_landmarks;

    double m_feature_tracker_min_keypoint_distance;
    size_t m_feature_tracker_max_keypoint_detection;
    size_t m_feature_tracker_max_init_frames;
    size_t m_feature_tracker_max_frames;
    double m_feature_tracker_clahe_clip_limit;
    size_t m_feature_tracker_clahe_width;
    size_t m_feature_tracker_clahe_height;
    bool m_feature_tracker_predict_keypoints;

    size_t m_initializer_keyframe_num;
    size_t m_initializer_keyframe_gap;
    size_t m_initializer_min_matches;
    double m_initializer_min_parallax;
    size_t m_initializer_min_triangulation;
    size_t m_initializer_min_landmarks;
    bool m_initializer_refine_imu;

#if XRSLAM_ENABLE_VISUAL_LOCALIZATION
    bool m_visual_localization_enable;
    std::string m_visual_localization_ip;
    size_t m_visual_localization_port;
#endif

    size_t m_solver_iteration_limit;
    double m_solver_time_limit;

    bool m_parsac_flag;
    double m_parsac_dynamic_probability;
    double m_parsac_threshold;
    double m_parsac_norm_scale;
    size_t m_parsac_keyframe_check_size;
    bool m_parsac_fast_imu_pnp;

    bool m_depth_fusion_enabled;
    double m_depth_prior_weight;

    double m_rotation_misalignment_threshold;
    double m_rotation_ransac_threshold;

    size_t m_runtime_max_raw_imu_queue;
    size_t m_runtime_max_pending_imu;
    size_t m_runtime_max_frontal_imu;
    size_t m_runtime_max_pending_camera_frames;
    size_t m_runtime_max_tracking_map_frames;
    size_t m_runtime_max_pending_frame_ids;
    size_t m_imu_timing_warmup_samples;
    size_t m_imu_timing_window_samples;
    double m_imu_timing_batch_gap_ratio;
};

} // namespace xrslam::extra

#endif // XRSLAM_EXTRA_YAML_CONFIG_H
