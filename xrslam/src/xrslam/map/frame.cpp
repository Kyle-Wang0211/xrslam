#include <xrslam/estimation/reprojection_factor.h>
#include <xrslam/geometry/stereo.h>
#include <xrslam/inspection.h>
#include <xrslam/map/frame.h>
#include <xrslam/map/map.h>
#include <xrslam/map/track.h>
#include <xrslam/utility/poisson_disk_filter.h>
#include <xrslam/utility/runtime_budget.h>

namespace xrslam {

struct Frame::construct_by_frame_t {};

Frame::Frame() : map(nullptr) {}

Frame::Frame(const Frame &frame, const construct_by_frame_t &construct_by_frame)
    : Tagged(frame), Identifiable(frame), map(nullptr) {}

Frame::~Frame() = default;

std::unique_ptr<Frame> Frame::clone() const {
    std::unique_ptr<Frame> frame =
        std::make_unique<Frame>(*this, construct_by_frame_t());
    frame->K = K;
    frame->sqrt_inv_cov = sqrt_inv_cov;
    frame->image = image;
    frame->depth = depth;
    frame->pose = pose;
    frame->motion = motion;
    frame->camera = camera;
    frame->imu = imu;
    frame->preintegration = preintegration;
    frame->bearings = bearings;
    frame->tracks = std::vector<Track *>(bearings.size(), nullptr);
    frame->reprojection_error_factors = std::vector<std::unique_ptr<ReprojectionErrorFactor>>(bearings.size());
    frame->map = nullptr;
    return frame;
}

void Frame::append_keypoint(const vector<3> &keypoint) {
    bearings.emplace_back(keypoint);
    tracks.emplace_back(nullptr);
    reprojection_error_factors.emplace_back(nullptr);
}

Track *Frame::get_track(size_t keypoint_index, Map *allocation_map) {
    if (!allocation_map) {
        allocation_map = map;
    }
    if (tracks[keypoint_index] == nullptr) {
        Track *track = allocation_map->create_track();
        track->add_keypoint(this, keypoint_index);
    }
    return tracks[keypoint_index];
}

void Frame::detect_keypoints(Config *config) {
    std::vector<vector<2>> pkeypoints(bearings.size());
    for (size_t i = 0; i < bearings.size(); ++i) {
        pkeypoints[i] = apply_k(bearings[i], K);
    }

    image->detect_keypoints(pkeypoints,
                            config->feature_tracker_max_keypoint_detection(),
                            config->feature_tracker_min_keypoint_distance());

    size_t old_keypoint_num = bearings.size();
    bearings.resize(pkeypoints.size());
    tracks.resize(pkeypoints.size(), nullptr);
    reprojection_error_factors.resize(pkeypoints.size());
    for (size_t i = old_keypoint_num; i < pkeypoints.size(); ++i) {
        bearings[i] = remove_k(pkeypoints[i], K);
    }
}

void Frame::track_keypoints(Frame *next_frame, Config *config) {
    std::vector<vector<2>> curr_keypoints(bearings.size());
    std::vector<vector<2>> next_keypoints;

    for (size_t i = 0; i < bearings.size(); ++i) {
        curr_keypoints[i] = apply_k(bearings[i], K);
    }

    if (config->feature_tracker_predict_keypoints()) {
        quaternion delta_key_q =
            (camera.q_cs.conjugate() * imu.q_cs *
             next_frame->preintegration.delta.q *
             next_frame->imu.q_cs.conjugate() * next_frame->camera.q_cs)
                .conjugate();
        next_keypoints.resize(curr_keypoints.size());
        for (size_t i = 0; i < bearings.size(); ++i) {
            next_keypoints[i] =
                apply_k(delta_key_q * bearings[i], next_frame->K);
        }
    }

    std::vector<char> status, mask;
    image->track_keypoints(next_frame->image.get(), curr_keypoints,
                           next_keypoints, status);

    std::vector<vector<2>> curr_keypoints_h, next_keypoints_h;
    std::vector<vector<3>> next_bearings;
    for (size_t i = 0; i < curr_keypoints.size(); ++i) {
        curr_keypoints_h.push_back(bearings[i].hnormalized());
        vector<3> next_keypoint = remove_k(next_keypoints[i], next_frame->K);
        next_keypoints_h.push_back(next_keypoint.hnormalized());
        next_bearings.push_back(next_keypoint);
    }

    matrix<3> E =
        find_essential_matrix(curr_keypoints_h, next_keypoints_h, mask, 1.0);
    // [pw] 任务 3:必须**在这里**数本质矩阵的内点。下面 find_rotation_matrix
    //      拿的是同一个 mask 的非 const 引用,会把它整个覆盖成**旋转模型**的
    //      内点掩码 -- 在那之后再数就不是"通过几何校验的观测数"了。
    int n_essential_inliers = 0;
    for (size_t im = 0; im < mask.size(); ++im) {
        if (mask[im])
            ++n_essential_inliers;
    }
    for (size_t i = 0; i < status.size(); ++i) {
        if (!mask[i]) {
            status[i] = 0;
        }
    }
    matrix<3> R = find_rotation_matrix(bearings, next_bearings, mask,
                                       (M_PI / 180.0) *
                                           config->rotation_ransac_threshold());

    std::vector<double> angles;
    for (size_t i = 0; i < mask.size(); ++i) {
        if (mask[i]) {
            double angle = acos((R * bearings[i]).dot(next_bearings[i]));
            angles.emplace_back(angle * 180 / M_PI);
        }
    }
    std::sort(angles.begin(), angles.end());
    double misalignment =
        angles.size() > 0 ? angles[angles.size() * 7 / 10] : 0;
    inspect_debug(feature_tracker_angle_misalignment, angle_misalignment) {
        angle_misalignment = misalignment;
    }
    if (misalignment < config->rotation_misalignment_threshold()) {
        next_frame->tag(FT_NO_TRANSLATION) = true;
    }

    // filter keypoints based on track length
    std::vector<std::pair<size_t, size_t>> keypoint_index_track_length;
    keypoint_index_track_length.reserve(curr_keypoints.size());
    for (size_t i = 0; i < curr_keypoints.size(); ++i) {
        if (status[i] == 0)
            continue;
        Track *track = get_track(i);
        if (track == nullptr)
            continue;
        keypoint_index_track_length.emplace_back(i, track->keypoint_num());
    }

    std::sort(keypoint_index_track_length.begin(),
              keypoint_index_track_length.end(),
              [](const auto &a, const auto &b) { return a.second > b.second; });

    PoissonDiskFilter<2> filter(
        config->feature_tracker_min_keypoint_distance());
    for (auto &[keypoint_index, track_length] : keypoint_index_track_length) {
        vector<2> pt = next_keypoints[keypoint_index];
        Track *track = this->get_track(keypoint_index);
        if (filter.permit_point(pt) &&
            (!track || (track && !track->tag(TT_TRASH)))) {
            filter.preset_point(pt);
        } else {
            status[keypoint_index] = 0;
        }
    }

    // [pw] 任务 3:发布本帧的 tracked / inlier 计数。
    //   inlier  = **本质矩阵** RANSAC 的内点(在 mask 被旋转模型覆盖之前就已
    //             数好,见上面 n_essential_inliers),纯几何校验结果;
    //   tracked = 最终真正传递到下一帧的关键点数(LK 存活 ∩ 本质矩阵内点
    //             ∩ 泊松盘保留 ∩ 非 TT_TRASH),下一帧实际能用的观测数。
    int n_tracked = 0;

    for (size_t curr_keypoint_index = 0;
         curr_keypoint_index < curr_keypoints.size(); ++curr_keypoint_index) {
        if (status[curr_keypoint_index]) {
            size_t next_keypoint_index = next_frame->keypoint_num();
            next_frame->append_keypoint(next_bearings[curr_keypoint_index]);
            get_track(curr_keypoint_index, nullptr)
                ->add_keypoint(next_frame, next_keypoint_index);
            ++n_tracked;
        }
    }

    auto &h = runtime::frame_health_atomics();
    h.tracked.store(n_tracked, std::memory_order_relaxed);
    h.inliers.store(n_essential_inliers, std::memory_order_relaxed);
}

PoseState Frame::get_pose(const ExtrinsicParams &sensor) const {
    PoseState result;
    result.q = pose.q * sensor.q_cs;
    result.p = pose.p + pose.q * sensor.p_cs;
    return result;
}

void Frame::set_pose(const ExtrinsicParams &sensor, const PoseState &pose) {
    this->pose.q = pose.q * sensor.q_cs.conjugate();
    this->pose.p = pose.p - this->pose.q * sensor.p_cs;
}

std::unique_lock<std::mutex> Frame::lock() const {
    if (map) {
        return map->lock();
    } else {
        return {};
    }
}

} // namespace xrslam
