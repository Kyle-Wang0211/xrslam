#ifdef XRSLAM_GPU_FRONTEND
#include <xrslam/extra/gpu_image.h>
#endif
#include <cmath>
#include "XRSLAMManager.h"
#include "XRSLAMImageExt.h"   // [pw 2026-09-22] 逐帧内参消费规则 + ABI 静态断言
#include "xrslam/core/feature_tracker.h"
#include "xrslam/core/frontend_worker.h"

#define XRSLAM_VERSION "0.1.0"

namespace xrslam {
XRSLAMManager &XRSLAMManager::Instance() {
    static XRSLAMManager SLAMManagerInstance;
    return SLAMManagerInstance;
}

int XRSLAMManager::PendingWorkerFrames() const {
    if (!detail_)
        return 0;
    size_t pending = 0;
    if (detail_->feature_tracker)
        pending += detail_->feature_tracker->pending_frame_count();
    if (detail_->frontend)
        pending += detail_->frontend->pending_frame_count();
    return static_cast<int>(pending);
}

XRSLAMManager::XRSLAMManager() {}
XRSLAMManager::~XRSLAMManager() {}

static const unsigned char logo_ascii[] = {
    0x0A, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x95, 0x97, 0x20, 0x20,
    0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x95, 0x97, 0xE2, 0x96, 0x88,
    0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88,
    0xE2, 0x96, 0x88, 0xE2, 0x95, 0x97, 0x20, 0xE2, 0x96, 0x88, 0xE2, 0x96,
    0x88, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x96,
    0x88, 0xE2, 0x96, 0x88, 0xE2, 0x95, 0x97, 0xE2, 0x96, 0x88, 0xE2, 0x96,
    0x88, 0xE2, 0x95, 0x97, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0xE2, 0x96,
    0x88, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x96,
    0x88, 0xE2, 0x95, 0x97, 0x20, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2,
    0x96, 0x88, 0xE2, 0x95, 0x97, 0x20, 0x20, 0x20, 0xE2, 0x96, 0x88, 0xE2,
    0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x95, 0x97, 0x0A, 0xE2, 0x95, 0x9A,
    0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x95, 0x97, 0xE2, 0x96, 0x88,
    0xE2, 0x96, 0x88, 0xE2, 0x95, 0x94, 0xE2, 0x95, 0x9D, 0xE2, 0x96, 0x88,
    0xE2, 0x96, 0x88, 0xE2, 0x95, 0x94, 0xE2, 0x95, 0x90, 0xE2, 0x95, 0x90,
    0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x95, 0x97, 0xE2, 0x96, 0x88,
    0xE2, 0x96, 0x88, 0xE2, 0x95, 0x94, 0xE2, 0x95, 0x90, 0xE2, 0x95, 0x90,
    0xE2, 0x95, 0x90, 0xE2, 0x95, 0x90, 0xE2, 0x95, 0x9D, 0xE2, 0x96, 0x88,
    0xE2, 0x96, 0x88, 0xE2, 0x95, 0x91, 0x20, 0x20, 0x20, 0x20, 0x20, 0xE2,
    0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x95, 0x94, 0xE2, 0x95, 0x90, 0xE2,
    0x95, 0x90, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x95, 0x97, 0xE2,
    0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2,
    0x95, 0x97, 0x20, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88,
    0xE2, 0x96, 0x88, 0xE2, 0x95, 0x91, 0x0A, 0x20, 0xE2, 0x95, 0x9A, 0xE2,
    0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x95, 0x94, 0xE2,
    0x95, 0x9D, 0x20, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88,
    0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x95, 0x94,
    0xE2, 0x95, 0x9D, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88,
    0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88,
    0xE2, 0x95, 0x97, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x95, 0x91,
    0x20, 0x20, 0x20, 0x20, 0x20, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2,
    0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2,
    0x96, 0x88, 0xE2, 0x95, 0x91, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2,
    0x95, 0x94, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2,
    0x96, 0x88, 0xE2, 0x95, 0x94, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2,
    0x95, 0x91, 0x0A, 0x20, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x95,
    0x94, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x95, 0x97, 0x20, 0xE2,
    0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x95, 0x94, 0xE2, 0x95, 0x90, 0xE2,
    0x95, 0x90, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x95, 0x97, 0xE2,
    0x95, 0x9A, 0xE2, 0x95, 0x90, 0xE2, 0x95, 0x90, 0xE2, 0x95, 0x90, 0xE2,
    0x95, 0x90, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x95, 0x91, 0xE2,
    0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x95, 0x91, 0x20, 0x20, 0x20, 0x20,
    0x20, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x95, 0x94, 0xE2, 0x95,
    0x90, 0xE2, 0x95, 0x90, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x95,
    0x91, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x95, 0x91, 0xE2, 0x95,
    0x9A, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x95, 0x94, 0xE2, 0x95,
    0x9D, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x95, 0x91, 0x0A, 0xE2,
    0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x95, 0x94, 0xE2, 0x95, 0x9D, 0x20,
    0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x95, 0x97, 0xE2, 0x96, 0x88,
    0xE2, 0x96, 0x88, 0xE2, 0x95, 0x91, 0x20, 0x20, 0xE2, 0x96, 0x88, 0xE2,
    0x96, 0x88, 0xE2, 0x95, 0x91, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2,
    0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2,
    0x96, 0x88, 0xE2, 0x95, 0x91, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2,
    0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2,
    0x96, 0x88, 0xE2, 0x95, 0x97, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2,
    0x95, 0x91, 0x20, 0x20, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x95,
    0x91, 0xE2, 0x96, 0x88, 0xE2, 0x96, 0x88, 0xE2, 0x95, 0x91, 0x20, 0xE2,
    0x95, 0x9A, 0xE2, 0x95, 0x90, 0xE2, 0x95, 0x9D, 0x20, 0xE2, 0x96, 0x88,
    0xE2, 0x96, 0x88, 0xE2, 0x95, 0x91, 0x0A, 0xE2, 0x95, 0x9A, 0xE2, 0x95,
    0x90, 0xE2, 0x95, 0x9D, 0x20, 0x20, 0xE2, 0x95, 0x9A, 0xE2, 0x95, 0x90,
    0xE2, 0x95, 0x9D, 0xE2, 0x95, 0x9A, 0xE2, 0x95, 0x90, 0xE2, 0x95, 0x9D,
    0x20, 0x20, 0xE2, 0x95, 0x9A, 0xE2, 0x95, 0x90, 0xE2, 0x95, 0x9D, 0xE2,
    0x95, 0x9A, 0xE2, 0x95, 0x90, 0xE2, 0x95, 0x90, 0xE2, 0x95, 0x90, 0xE2,
    0x95, 0x90, 0xE2, 0x95, 0x90, 0xE2, 0x95, 0x90, 0xE2, 0x95, 0x9D, 0xE2,
    0x95, 0x9A, 0xE2, 0x95, 0x90, 0xE2, 0x95, 0x90, 0xE2, 0x95, 0x90, 0xE2,
    0x95, 0x90, 0xE2, 0x95, 0x90, 0xE2, 0x95, 0x90, 0xE2, 0x95, 0x9D, 0xE2,
    0x95, 0x9A, 0xE2, 0x95, 0x90, 0xE2, 0x95, 0x9D, 0x20, 0x20, 0xE2, 0x95,
    0x9A, 0xE2, 0x95, 0x90, 0xE2, 0x95, 0x9D, 0xE2, 0x95, 0x9A, 0xE2, 0x95,
    0x90, 0xE2, 0x95, 0x9D, 0x20, 0x20, 0x20, 0x20, 0x20, 0xE2, 0x95, 0x9A,
    0xE2, 0x95, 0x90, 0xE2, 0x95, 0x9D};

void XRSLAMManager::Init(std::shared_ptr<Config> config) {
    detail_ = std::make_unique<XRSLAM::Detail>(config);
    config_ = config;
    log_message(XRSLAM_LOG_INFO, (char *)logo_ascii, XRSLAM_VERSION_STRING);
    config_->log_config();
    std::cout << "-----------------Create XRSLAM v" << XRSLAM_VERSION
              << " successfully-----------" << std::endl;
}

void XRSLAMManager::Destroy() {
    {
        std::lock_guard<std::mutex> lck(input_mutex_);
        cur_image_.reset();
    }
    {
        std::lock_guard<std::mutex> lk(intrinsics_mutex_);
        has_latest_intrinsics_ = false;   // [pw 2026-09-22] 不让上一会话的逐帧 K 漏到下一会话
    }
    // XRSLAM::Detail::~Detail() is the upstream owner of worker stop/join.
    detail_.reset();
    config_.reset();
    std::cout << "-----------------Destroy XRSLAM v" << XRSLAM_VERSION
              << " successfully-----------" << std::endl;
}

int XRSLAMManager::CheckLicense(const char *license_path,
                                const char *product_name) {
    return 1;
}

void XRSLAMManager::PushImage(XRSLAMImage *image) {
    // left img
    if (image->camera_id == 0) {
#ifdef XRSLAM_GPU_FRONTEND
        std::shared_ptr<xrslam::extra::OpenCvImage> opencv_image =
            xrslam::extra::GpuImage::create_image();   // GpuImage iff PW_XRSLAM_GPU_FRONTEND=1 and the GPU front end initialised
#else
        std::shared_ptr<xrslam::extra::OpenCvImage> opencv_image =
            std::make_shared<xrslam::extra::OpenCvImage>();
#endif
        int cols = config_->camera_resolution()[0];
        int rows = config_->camera_resolution()[1];
        opencv_image->t = image->timeStamp;
        // [pw 2026-09-22 逐帧内参] 判决书 §3.7 第 3 条:把 ext 里的当帧 K 拷进
        // xrslam::Image,再由 detail.cpp track_camera 注入 frame->K。规则见 XRSLAMImageExt.h;
        // 拿不到(旧调用者 / ext 为空 / 标志未置)就保持 has_K=false ⇒ 上游行为。
        {
            double k4[4];
            if (xrslam::pw::take_frame_intrinsics(image, k4)) {
                // 与 yaml_config.cpp:157-161 同一构造:单位阵 + fx fy cx cy,零 skew
                //(Apple 的 3x3 亦零 skew,判决书 §3.6)。
                opencv_image->K.setIdentity();
                opencv_image->K(0, 0) = k4[0];
                opencv_image->K(1, 1) = k4[1];
                opencv_image->K(0, 2) = k4[2];
                opencv_image->K(1, 2) = k4[3];
                opencv_image->has_K = true;
                std::lock_guard<std::mutex> lk(intrinsics_mutex_);
                latest_intrinsics_ = {k4[0], k4[1], k4[2], k4[3]};
                has_latest_intrinsics_ = true;
            }
        }

        cv::Mat img;
        if(image->channel == 1){
            img = cv::Mat(rows, cols, CV_8UC1, image->data, image->stride);
            opencv_image->image = img.clone();
        }
        else if(image->channel == 3){
            img = cv::Mat(rows, cols, CV_8UC3, image->data, image->stride);
            cv::cvtColor(img, opencv_image->image, cv::COLOR_BGR2GRAY);
        }
        else if(image->channel == 4){
            img = cv::Mat(rows, cols, CV_8UC4, image->data, image->stride);
            cv::cvtColor(img, opencv_image->image, cv::COLOR_BGRA2GRAY);
        }
        else{
            std::cerr << "Image channel is not supported!" << std::endl;
            exit(-1);
        }
            
        opencv_image->raw = img.clone();
#ifdef XRSLAM_GPU_FRONTEND
        if (auto *g = dynamic_cast<xrslam::extra::GpuImage *>(opencv_image.get()))   // pipeline: start the GPU pyramid now, on the capture thread
            g->prefetch(config_->feature_tracker_clahe_clip_limit(), config_->feature_tracker_clahe_width(), config_->feature_tracker_clahe_height());
#endif

        std::lock_guard<std::mutex> lck(input_mutex_);
        cur_image_ = std::shared_ptr<xrslam::Image>(opencv_image);
    }
}

void XRSLAMManager::PushAcceleration(XRSLAMAcceleration *acc) {
    Pose p = detail_->track_accelerometer(acc->timestamp, acc->data[0],
                                          acc->data[1], acc->data[2]);
    KeepPropagatedPose(acc->timestamp, p);
}

void XRSLAMManager::PushGyroscope(XRSLAMGyroscope *gyro) {
    Pose p = detail_->track_gyroscope(gyro->timestamp, gyro->data[0],
                                      gyro->data[1], gyro->data[2]);
    KeepPropagatedPose(gyro->timestamp, p);
}

// Both track_* entry points return Detail::predict_pose(t). Upstream's C++ API hands that pose to
// its caller; this C interface used to drop it, which left XRSLAM_RESULT_BODY_POSE advancing only
// once per image. Storing it costs one mutex per IMU sample and changes no estimate.
void XRSLAMManager::KeepPropagatedPose(double t, const Pose &pose) {
    if (!std::isfinite(t) || pose.q.coeffs().isZero())
        return;
    std::lock_guard<std::mutex> lck(imu_pose_mutex_);
    if (t < imu_pose_timestamp_)
        return;
    imu_pose_ = pose;
    imu_pose_timestamp_ = t;
    imu_pose_valid_ = true;
}

void XRSLAMManager::GetResultPropagatedPose(XRSLAMPose *pose) const {
    std::lock_guard<std::mutex> lck(imu_pose_mutex_);
    if (!imu_pose_valid_) {
        pose->timestamp = 0.0;
        for (int i = 0; i < 4; ++i) pose->quaternion[i] = 0.0;
        for (int i = 0; i < 3; ++i) pose->translation[i] = 0.0;
        return;
    }
    pose->timestamp = imu_pose_timestamp_;
    pose->quaternion[0] = imu_pose_.q.x();
    pose->quaternion[1] = imu_pose_.q.y();
    pose->quaternion[2] = imu_pose_.q.z();
    pose->quaternion[3] = imu_pose_.q.w();
    pose->translation[0] = imu_pose_.p.x();
    pose->translation[1] = imu_pose_.p.y();
    pose->translation[2] = imu_pose_.p.z();
}

// [bench 2026-09-17] The two accessors that carry the OpenXR/Monado relation flags alongside the
// pose. Nothing new is estimated here: every bit is decided from state this interface already had
// (imu_pose_valid_, the quaternion it was about to hand back, and the system state), it just stops
// throwing that state away.
namespace {
// A zero-norm quaternion is not an orientation. XRSLAM emits one on the first TRACKING_SUCCESS and
// whenever no propagated pose exists yet; under the source contract that is ORIENTATION_VALID unset
// ("applications must not read the pose field's orientation if this flag is unset"), not a failure.
inline bool pw_quaternion_readable(const XRSLAMPose *pose) {
    const double n2 = pose->quaternion[0] * pose->quaternion[0] +
                      pose->quaternion[1] * pose->quaternion[1] +
                      pose->quaternion[2] * pose->quaternion[2] +
                      pose->quaternion[3] * pose->quaternion[3];
    return std::isfinite(n2) && n2 > 1.0e-12;
}
inline bool pw_translation_readable(const XRSLAMPose *pose) {
    return std::isfinite(pose->translation[0]) && std::isfinite(pose->translation[1]) &&
           std::isfinite(pose->translation[2]);
}
} // namespace

void XRSLAMManager::GetPropagatedPoseRelation(XRSLAMPose *pose, unsigned int *flags) const {
    // Assign the none-mask before anything else, the way the source does before an error return.
    if (flags != nullptr)
        *flags = XRSLAM_SPACE_RELATION_BITMASK_NONE;
    if (pose == nullptr)
        return;
    GetResultPropagatedPose(pose);
    if (flags == nullptr)
        return;
    unsigned int f = XRSLAM_SPACE_RELATION_BITMASK_NONE;
    if (pose->timestamp > 0.0) {
        // A propagated pose is dead reckoning: the spec names "inertial dead reckoning" as exactly
        // the case that stays VALID with TRACKED cleared, so this path never sets a TRACKED bit.
        if (pw_quaternion_readable(pose))
            f |= XRSLAM_SPACE_RELATION_ORIENTATION_VALID_BIT;
        if (pw_translation_readable(pose))
            f |= XRSLAM_SPACE_RELATION_POSITION_VALID_BIT;
    }
    *flags = f;
}

void XRSLAMManager::GetBodyPoseRelation(XRSLAMPose *pose, unsigned int *flags) const {
    if (flags != nullptr)
        *flags = XRSLAM_SPACE_RELATION_BITMASK_NONE;
    if (pose == nullptr)
        return;
    GetResultBodyPose(pose);
    if (flags == nullptr)
        return;
    unsigned int f = XRSLAM_SPACE_RELATION_BITMASK_NONE;
    if (pose->timestamp <= 0.0) {
        *flags = f;
        return;
    }
    const bool q_ok = pw_quaternion_readable(pose);
    const bool p_ok = pw_translation_readable(pose);
    if (q_ok)
        f |= XRSLAM_SPACE_RELATION_ORIENTATION_VALID_BIT;
    if (p_ok)
        f |= XRSLAM_SPACE_RELATION_POSITION_VALID_BIT;
    // TRACKED means actively observed. The only signal this interface has for that is the system
    // state: SYS_TRACKING is the sliding window running on images. It is a weak signal -- upstream
    // sets it from a pointer being non-null (frontend_worker.cpp:180-187) -- so it is used only to
    // *withhold* the TRACKED bits, never to assert VALID.
    if (detail_ != nullptr && detail_->get_system_state() == SysState::SYS_TRACKING) {
        if (q_ok)
            f |= XRSLAM_SPACE_RELATION_ORIENTATION_TRACKED_BIT;
        if (p_ok)
            f |= XRSLAM_SPACE_RELATION_POSITION_TRACKED_BIT;
    }
    *flags = f;
}

void XRSLAMManager::RunOneFrame() {
    std::lock_guard<std::mutex> lck(input_mutex_);
    detail_->track_camera(cur_image_);
}

void XRSLAMManager::GetResultBodyPose(XRSLAMPose *pose) const {
    std::tuple<double, Pose> latest_state = detail_->get_latest_pose();
    pose->timestamp = std::get<0>(latest_state);
    Pose latest_pose = std::get<1>(latest_state);
    Pose body_pose;
    body_pose.q = latest_pose.q * config_->imu_to_body_rotation();
    body_pose.p = latest_pose.p + latest_pose.q * config_->imu_to_body_translation();
    for (int i = 0; i < 3; i++)
        pose->translation[i] = body_pose.p(i);
    pose->quaternion[0] = body_pose.q.x();
    pose->quaternion[1] = body_pose.q.y();
    pose->quaternion[2] = body_pose.q.z();
    pose->quaternion[3] = body_pose.q.w();
}

void XRSLAMManager::GetResultCameraPose(XRSLAMPose *pose) const {
    std::tuple<double, Pose> latest_state = detail_->get_latest_pose();
    pose->timestamp = std::get<0>(latest_state);
    Pose latest_pose = std::get<1>(latest_state);
    Pose camera_pose;
    camera_pose.q = latest_pose.q * config_->camera_to_body_rotation();
    camera_pose.p = latest_pose.p + latest_pose.q * config_->camera_to_body_translation();
    for (int i = 0; i < 3; i++)
        pose->translation[i] = camera_pose.p(i);
    pose->quaternion[0] = camera_pose.q.x();
    pose->quaternion[1] = camera_pose.q.y();
    pose->quaternion[2] = camera_pose.q.z();
    pose->quaternion[3] = camera_pose.q.w();
}

void XRSLAMManager::GetInfoIntrinsics(XRSLAMIntrinsics *intrinsics) const {
    // [pw 2026-09-22 逐帧内参] 判决书 §3.7 第 5 条(可选项):有逐帧 K 时报最近一帧的,
    // 否则仍报 Config 的常量(上游行为)。
    {
        std::lock_guard<std::mutex> lk(intrinsics_mutex_);
        if (has_latest_intrinsics_) {
            *intrinsics = latest_intrinsics_;
            return;
        }
    }
    intrinsics->fx = config_->camera_intrinsic()(0, 0);
    intrinsics->fy = config_->camera_intrinsic()(1, 1);
    intrinsics->cx = config_->camera_intrinsic()(0, 2);
    intrinsics->cy = config_->camera_intrinsic()(1, 2);
}


void XRSLAMManager::GetResultState(XRSLAMState *state) const {
    SysState cur_state = detail_->get_system_state();
    if (cur_state == SYS_INITIALIZING) {
        *state = XRSLAM_STATE_INITIALIZING;
    } else if (cur_state == SYS_TRACKING) {
        *state = XRSLAM_STATE_TRACKING_SUCCESS;
    } else if (cur_state == SYS_CRASH) {
        *state = XRSLAM_STATE_TRACKING_FAIL;
    } else if (cur_state == SYS_UNKNOWN) {
        *state = XRSLAM_STATE_TRACKING_FAIL;
    }
}
void XRSLAMManager::GetResultLandmarks(XRSLAMLandmarks *landmarks) const {
    inspect_debug(sliding_window_landmarks, swlandmarks) {
        auto pts = std::any_cast<std::vector<xrslam::Landmark>>(swlandmarks);
        landmarks->num_landmarks = pts.size();
        landmarks->landmarks = new XRSLAMLandmark[pts.size()];
        for (int i = 0; i < pts.size(); i++) {
            xrslam::vector<3> cur_p = pts[i].p;
            landmarks->landmarks[i].x = cur_p(0);
            landmarks->landmarks[i].y = cur_p(1);
            landmarks->landmarks[i].z = cur_p(2);
        }
    }
}

void XRSLAMManager::GetResultFeatures(XRSLAMFeatures *features) const {}

void XRSLAMManager::GetResultBias(XRSLAMIMUBias *bias) const {
    inspect_debug(sliding_window_current_bg, bg) {
        if (bg.has_value()) {
            xrslam::vector<3> gyr_bias = std::any_cast<xrslam::vector<3>>(bg);
            bias->gyr_bias.data[0] = gyr_bias(0);
            bias->gyr_bias.data[1] = gyr_bias(1);
            bias->gyr_bias.data[2] = gyr_bias(2);
        }
    }
    inspect_debug(sliding_window_current_ba, ba) {
        if (ba.has_value()) {
            xrslam::vector<3> acc_bias = std::any_cast<xrslam::vector<3>>(ba);
            bias->acc_bias.data[0] = acc_bias(0);
            bias->acc_bias.data[1] = acc_bias(1);
            bias->acc_bias.data[2] = acc_bias(2);
        }
    }
}

void XRSLAMManager::GetResultVersion(XRSLAMStringOutput *output) const {
    output->str_length = strlen(XRSLAM_VERSION);
    output->data = new char[output->str_length + 5];
    strcpy(output->data, XRSLAM_VERSION);
}

} // namespace xrslam
