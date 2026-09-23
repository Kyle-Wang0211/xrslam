#include <xrslam/common.h>

namespace xrslam {

Config::~Config() = default;

vector<4> Config::camera_distortion() const { return vector<4>::Zero(); }

quaternion Config::output_to_body_rotation() const {
    return quaternion::Identity();
}
vector<3> Config::output_to_body_translation() const {
    return vector<3>::Zero();
}

size_t Config::sliding_window_size() const { return 10; }

size_t Config::sliding_window_subframe_size() const { return 3; }

size_t Config::sliding_window_force_keyframe_landmarks() const { return 35; }
double Config::sliding_window_rpe_threshold_px() const { return 3.0; }
// 0 = keep the bare pixel threshold, i.e. upstream's behaviour byte for byte.
double Config::sliding_window_rpe_reference_focal() const { return 0.0; }

double Config::feature_tracker_min_keypoint_distance() const { return 20.0; }

size_t Config::feature_tracker_max_keypoint_detection() const { return 150; }

size_t Config::feature_tracker_max_init_frames() const { return 60; }

size_t Config::feature_tracker_max_frames() const { return 200; }

double Config::feature_tracker_clahe_clip_limit() const { return 6.0; }

size_t Config::feature_tracker_clahe_width() const { return 8; }

size_t Config::feature_tracker_clahe_height() const { return 8; }

bool Config::feature_tracker_predict_keypoints() const { return true; }

size_t Config::initializer_keyframe_num() const { return 8; }

size_t Config::initializer_keyframe_gap() const { return 5; }

size_t Config::initializer_min_matches() const { return 50; }

double Config::initializer_min_parallax() const { return 10; }

size_t Config::initializer_min_triangulation() const { return 50; }

size_t Config::initializer_min_landmarks() const { return 30; }

bool Config::initializer_refine_imu() const { return true; }

bool Config::visual_localization_enable() const { return false; }

std::string Config::visual_localization_config_ip() const { return "0.0.0.0"; }

size_t Config::visual_localization_config_port() const { return 0; }

size_t Config::solver_iteration_limit() const { return 10; }

double Config::solver_time_limit() const { return 1.0e6; }

double Config::rotation_misalignment_threshold() const { return 0.1; }

double Config::rotation_ransac_threshold() const { return 10; }

int Config::random() const { return 648; }

bool Config::parsac_flag() const { return false; }

double Config::parsac_dynamic_probability() const { return 0.0; }

double Config::parsac_threshold() const { return 3.0; }

double Config::parsac_norm_scale() const { return 1.0; }

size_t Config::parsac_keyframe_check_size() const { return 3; }

// [pw 2026-09-23] Tracking-loss recovery defaults. Sources are cited in sliding_window_tracker.h.
bool Config::tracking_recovery_enable() const { return false; }

// Paper stages [P3]/[P4] (new map after 5 s lost / map discarded when lost < 15 s after IMU
// init). OFF by default: with it off, recovery never resets the map (see the .h).
bool Config::tracking_recovery_long_term_reset() const { return false; }

// ORB-SLAM3 Sec. V-D p.8: "enters into visually lost state when less than 15 point maps are tracked"
size_t Config::tracking_recovery_min_tracked_landmarks() const { return 15; }

// ORB-SLAM3 source (read only), Tracking.cc:3033: while RECENTLY_LOST, TrackLocalMap succeeds
// with more than 10 inliers (the paper gives only the loss threshold).
size_t Config::tracking_recovery_recover_tracked_landmarks_above() const { return 10; }

// ORB-SLAM3 Sec. V-D p.8: "Otherwise, after 5 seconds, we pass to the next stage."
double Config::tracking_recovery_lost_timeout() const { return 5.0; }

// ORB-SLAM3 Sec. V-D p.9: "If the system gets lost within 15 seconds after IMU initialization,
// the map is discarded."
double Config::tracking_recovery_min_map_age() const { return 15.0; }

// ORB-SLAM3 source (read only), Tracking.cc:3410-3411 th = 15 while (RECENTLY_)LOST, times
// ORBmatcher.cc:215-220 RadiusByViewingCos = 4.0 px (the non-frontal case) = 60 px at octave 0.
double Config::tracking_recovery_search_radius_px() const { return 60.0; }

// <= 0 keeps the radius in bare pixels (same convention as sliding_window_rpe_reference_focal).
double Config::tracking_recovery_search_reference_focal() const { return 0.0; }

size_t Config::sliding_window_tracker_frequent() const { return 1; }

void Config::log_config() const {
    std::stringstream ss;
    ss << std::scientific << std::boolalpha << std::setprecision(5);

    ss << "Config::camera_intrinsic:\n"
       << camera_intrinsic() << "\n"
       << std::endl;

    ss << "Config::camera_distortion_flag:\n"
       << camera_distortion_flag() << "\n"
       << std::endl;

    ss << "Config::camera_distortion:\n"
       << camera_distortion().transpose() << "\n"
       << std::endl;

    ss << "Config::camera_time_offset:\n"
       << camera_time_offset() << "\n"
       << std::endl;

    ss << "Config::camera_to_body_rotation:\n"
       << camera_to_body_rotation().coeffs().transpose() << "\n"
       << std::endl;

    ss << "Config::camera_to_body_translation:\n"
       << camera_to_body_translation().transpose() << "\n"
       << std::endl;

    ss << "Config::imu_to_body_rotation:\n"
       << imu_to_body_rotation().coeffs().transpose() << "\n"
       << std::endl;

    ss << "Config::imu_to_body_translation:\n"
       << imu_to_body_translation().transpose() << "\n"
       << std::endl;

    ss << "Config::keypoint_noise_cov:\n"
       << keypoint_noise_cov() << "\n"
       << std::endl;

    ss << "Config::gyroscope_noise_cov:\n"
       << gyroscope_noise_cov() << "\n"
       << std::endl;

    ss << "Config::accelerometer_noise_cov:\n"
       << accelerometer_noise_cov() << "\n"
       << std::endl;

    ss << "Config::gyroscope_bias_noise_cov:\n"
       << gyroscope_bias_noise_cov() << "\n"
       << std::endl;

    ss << "Config::accelerometer_bias_noise_cov:\n"
       << accelerometer_bias_noise_cov() << "\n"
       << std::endl;

    ss << "Config::sliding_window_size: " << sliding_window_size() << std::endl;

    ss << "Config::sliding_window_subframe_size: "
       << sliding_window_subframe_size() << std::endl;

    ss << "Config::sliding_window_force_keyframe_landmarks: "
       << sliding_window_force_keyframe_landmarks() << std::endl;
    ss << "Config::sliding_window_rpe_threshold_px: "
       << sliding_window_rpe_threshold_px() << std::endl;
    ss << "Config::sliding_window_rpe_reference_focal: "
       << sliding_window_rpe_reference_focal() << std::endl;

    ss << "Config::sliding_window_tracker_frequent: "
       << sliding_window_tracker_frequent() << std::endl;

    ss << "Config::feature_tracker_min_keypoint_distance: "
       << feature_tracker_min_keypoint_distance() << std::endl;

    ss << "Config::feature_tracker_max_keypoint_detection: "
       << feature_tracker_max_keypoint_detection() << std::endl;

    ss << "Config::feature_tracker_max_init_frames: "
       << feature_tracker_max_init_frames() << std::endl;

    ss << "Config::feature_tracker_max_frames: " << feature_tracker_max_frames()
       << std::endl;

    ss << "Config::feature_tracker_predict_keypoints: "
       << feature_tracker_predict_keypoints() << std::endl;

    ss << "Config::feature_tracker_clahe_clip_limit: "
       << feature_tracker_clahe_clip_limit() << std::endl;

    ss << "Config::feature_tracker_clahe_width: "
       << feature_tracker_clahe_width() << std::endl;

    ss << "Config::feature_tracker_clahe_height: "
       << feature_tracker_clahe_height() << std::endl;

    ss << "Config::initializer_keyframe_gap: " << initializer_keyframe_gap()
       << std::endl;

    ss << "Config::initializer_min_matches: " << initializer_min_matches()
       << std::endl;

    ss << "Config::initializer_min_parallax: " << initializer_min_parallax()
       << std::endl;

    ss << "Config::initializer_min_triangulation: "
       << initializer_min_triangulation() << std::endl;

    ss << "Config::initializer_min_landmarks: " << initializer_min_landmarks()
       << std::endl;

    ss << "Config::initializer_refine_imu: " << initializer_refine_imu()
       << std::endl;

    ss << "Config::visual_localization_enable: " << visual_localization_enable()
       << std::endl;

    ss << "Config::visual_localization_config_ip: "
       << visual_localization_config_ip() << std::endl;

    ss << "Config::visual_localization_config_port: "
       << visual_localization_config_port() << std::endl;

    ss << "Config::solver_iteration_limit: " << solver_iteration_limit()
       << std::endl;

    ss << "Config::solver_time_limit: " << solver_time_limit() << std::endl;

    ss << "Config::parsac_flag: " << parsac_flag() << std::endl;

    ss << "Config::parsac_dynamic_probability: " << parsac_dynamic_probability()
       << std::endl;

    ss << "Config::parsac_threshold: " << parsac_threshold() << std::endl;

    ss << "Config::parsac_norm_scale: " << parsac_norm_scale() << std::endl;

    ss << "Config::tracking_recovery_enable: " << tracking_recovery_enable()
       << std::endl;
    ss << "Config::tracking_recovery_long_term_reset: "
       << tracking_recovery_long_term_reset() << std::endl;
    ss << "Config::tracking_recovery_min_tracked_landmarks: "
       << tracking_recovery_min_tracked_landmarks() << std::endl;
    ss << "Config::tracking_recovery_recover_tracked_landmarks_above: "
       << tracking_recovery_recover_tracked_landmarks_above() << std::endl;
    ss << "Config::tracking_recovery_lost_timeout: "
       << tracking_recovery_lost_timeout() << std::endl;
    ss << "Config::tracking_recovery_min_map_age: "
       << tracking_recovery_min_map_age() << std::endl;
    ss << "Config::tracking_recovery_search_radius_px: "
       << tracking_recovery_search_radius_px() << std::endl;
    ss << "Config::tracking_recovery_search_reference_focal: "
       << tracking_recovery_search_reference_focal() << std::endl;

    ss << "Config::rotation_misalignment_threshold: "
       << rotation_misalignment_threshold() << std::endl;

    ss << "Config::rotation_ransac_threshold: " << rotation_ransac_threshold()
       << std::endl;

#if defined(XRSLAM_ENABLE_THREADING)
    ss << std::endl;
    ss << "THREADING ENABLE" << std::endl;
#else
    ss << std::endl;
    ss << "THREADING DISABLE" << std::endl;
#endif

    log_info("Configurations: \n%s", ss.str().c_str());
}

} // namespace xrslam
