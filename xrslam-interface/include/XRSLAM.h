/**
 * @file XRSLAM.h
 * @brief XRSlam API
 */

#ifndef _XRSLAM_H_
#define _XRSLAM_H_
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************************
 *                                 XRSLAM sensor data
 ******************************************************************************************/
/**
 * @brief input sensor data type
 */
typedef enum XRSLAMSensorType {
    XRSLAM_SENSOR_CAMERA = 0,      /*!< gray image. */
    XRSLAM_SENSOR_DEPTH_CAMERA,    /*!< depth image. */
    XRSLAM_SENSOR_ACCELERATION,    /*!< acceleration. */
    XRSLAM_SENSOR_GYROSCOPE,       /*!< gyroscope. */
    XRSLAM_SENSOR_GRAVITY,         /*!< gravity. */
    XRSLAM_SENSOR_ROTATION_VECTOR, /*!< rotation vector. */
    XRSLAM_SENSOR_UNKNOWN
} XRSLAMSensorType;

/**
 * @brief Image extension info
 *
 * [pw 2026-09-22 逐帧内参] 前四个 double 是上游原样预留、整仓零消费的字段
 * (调研判决书 §3.4);本次**只在尾部追加**,前 32 字节的布局与语义一字不动。
 * 旧调用者(按 32 字节的旧结构编译)传进来的 ext,引擎不读尾部——
 * 判据是 XRSLAMImage::ext_size(见下),不是指针本身。
 */
typedef struct XRSLAMImageExtension {
    double exposure_time;          /*!< image exposure time. */
    double default_focus_distance; /*!< default focus info. */
    double focal_length;           /*!< current focal length. */
    double focus_distance;         /*!< current focus distance. */
    /* ---- 以下为 2026-09-22 追加(判决书 §3.7 第 1 条) ---- */
    double intrinsics_fxfycxcy[4]; /*!< 当帧针孔内参(像素):fx, fy, cx, cy。
                                        iOS 来源 kCMSampleBufferAttachmentKey_CameraIntrinsicMatrix。 */
    int has_intrinsics;            /*!< == 1 ⇒ intrinsics_fxfycxcy 有效;其它值一律按无效。 */
    int reserved_pad;              /*!< 对齐填充,调用者置 0。 */
} XRSLAMImageExtension;

/** 上游 32 字节旧结构的大小;旧调用者的 ext 至多只有这么大,引擎绝不读越过它。 */
#define XRSLAM_IMAGE_EXTENSION_LEGACY_SIZE 32u

/**
 * @brief input gray image data
 */
typedef struct XRSLAMImage {
    unsigned char
        *data;        /*!< contains the intensity value for each image pixel. */
    double timeStamp; /*!< timestamp in second. */
    int stride;       /*!< image stride, number of bytes per row. */
    int camera_id;    /*!< camera id. */
    int channel;      /*!< image channel. */
    /* [pw 2026-09-22] 版本/尺寸字段,落在上游布局里 channel 与 ext 之间**原有的
       4 字节对齐填充**上:结构体仍是 40 字节,data/timeStamp/stride/camera_id/
       channel/ext 六个成员的偏移一个都没动(XRSLAMManager.cpp 里有 static_assert)。
       语义抄 Win32 的 cbSize 约定:调用者填 sizeof(XRSLAMImageExtension)——按
       **调用者自己编译时**的头文件。旧调用者要么零初始化得到 0(出货传输层
       PwXrslamTransportCore.cpp:211 `XRSLAMImage image{}`),要么 ext 本身就是
       nullptr(上游 XRSLAM_iOS.mm:156 / main.cpp:143 / xrslam_node.cpp:91),
       两种情况引擎都按旧行为走,不读 ext 的尾部。 */
    unsigned int ext_size;     /*!< sizeof(*ext) as compiled by the caller; 0 = legacy. */
    XRSLAMImageExtension *ext; /*!< ext info of image. */
} XRSLAMImage;

/**
 * @brief input depth image data
 */
typedef struct XRSLAMDepthImage {
    uint16_t *data;       /*!< the warped depth data. */
    uint16_t *confidence; /*!< the warped confidence. */
    double timeStamp;     /*!<  timestamp in second. */
} XRSLAMDepthImage;

/**
 * @brief input acceleration data
 */
typedef struct XRSLAMAcceleration {
    double data[3];   /*!< acceleration raw data. */
    double timestamp; /*!< timestamp in second. */
} XRSLAMAcceleration;

/**
 * @brief input gyroscope data
 */
typedef struct XRSLAMGyroscope {
    double data[3];   /*!< gyroscope raw data. */
    double timestamp; /*!< timestamp in second. */
} XRSLAMGyroscope;

/**
 * @brief input gravity direction data
 */
typedef struct XRSLAMGravity {
    double data[3];   /*!< gravity direction. */
    double timestamp; /*!< timestamp in second. */
} XRSLAMGravity;

/**
 * @brief attitude of the device
 */
typedef struct XRSLAMRotationVector {
    double data[4];   /*!< attitude of the device. */
    double timestamp; /*!< timestamp in second. */
} XRSLAMRotationVector;

/******************************************************************************************
 *                                   XRSLAM tracking result
 ******************************************************************************************/
/**
 * @brief slam result data type
 */
typedef enum XRSLAMResultType {
    XRSLAM_RESULT_BODY_POSE = 0, /*!< body pose. */
    XRSLAM_RESULT_CAMERA_POSE,   /*!< camera pose. */
    XRSLAM_RESULT_STATE,         /*!< system state. */
    XRSLAM_RESULT_LANDMARKS,     /*!< 3D landmarks. */
    XRSLAM_RESULT_FEATURES,      /*!< 2D features. */
    XRSLAM_RESULT_BIAS,          /*!< imu bias. */
    XRSLAM_RESULT_DEBUG_LOGS,    /*!< debug logs. */
    XRSLAM_RESULT_VERSION,       /*!< version. */
    XRSLAM_RESULT_UNKNOWN,
    XRSLAM_INFO_INTRINSICS
} XRSLAMResultType;

/**
 * @brief    slam pose.
 * @details  For a 3D point in world coordinate \f$ X_w \f$, its 3D
 *           coordinate in the camera frame \f$ X_c = R * X_w + T \f$
 */
typedef struct XRSLAMPose {
    double quaternion[4];  /*!< quaternion of rotation: format [x, y, z, w]. */
    double translation[3]; /*!< translation vector T. */
    double timestamp;      /*!< timestamp. */
} XRSLAMPose;

typedef struct XRSLAMIntrinsics {
    double fx;
    double fy;
    double cx;
    double cy;
} XRSLAMIntrinsics;


/**
 * @brief    slam state.
 * @details  SLAM state to show the system status.
 */
typedef enum XRSLAMState {
    XRSLAM_STATE_INITIALIZING,     /*!< SLAM is in initialize state. */
    XRSLAM_STATE_TRACKING_SUCCESS, /*!< SLAM is in tracking state and track
                                      success. */
    XRSLAM_STATE_TRACKING_FAIL /*!< SLAM is in tracking state and track fail. */
} XRSLAMState;

/**
 * @brief  landmark 3d position.
 * @details x, y, z is world position of the point.
 */
typedef struct XRSLAMLandmark {
    double x, y, z;
} XRSLAMLandmark;
typedef struct XRSLAMLandmarks {
    XRSLAMLandmark *landmarks;
    int num_landmarks;
} XRSLAMLandmarks;

/**
 * @brief  2d corner in image coordinate.
 */
typedef struct XRSLAMFeature {
    double x, y;
} XRSLAMFeature;
typedef struct XRSLAMFeatures {
    struct Point{
        double x; 
        double y;
    };
    std::vector<Point> pos;
} XRSLAMFeatures;

/**
 * @brief IMU bias.
 */
typedef struct XRSLAMBias {
    double data[3];
} XRSLAMBias;
typedef struct XRSLAMIMUBias {
    XRSLAMBias acc_bias;
    XRSLAMBias gyr_bias;
} XRSLAMIMUBias;
/**
 * @brief  debug logs.
 */
typedef struct XRSLAMStringOutput {
    int str_length;
    char *data; /*!< slam information. */
} XRSLAMStringOutput;

/******************************************************************************************
 *                                   XRSLAM function interface
 ******************************************************************************************/

/**
 * @brief create SLAM system with configuration files.
 * @param[in] slam_config_path slam configuration file path.
 * @param[in] device_config_path  device configuration file path, include camera
 * intrinsics, extrinsics and so on.
 * @param[in] license_path  license path
 * @param[in] product_name  product name which user can define
 * @param[out] config configuration of slam and device
 * @return 1 success, otherwise 0
 */
int XRSLAMCreate(const char *slam_config_path, const char *device_config_path,
                 const char *license_path, const char *product_name,
                 void **config);

/**
 * @brief push sensor data to SLAM system
 * @param[in] sensor_type sensor type.
 * @param[in] sensor_data sensor data.
 */
void XRSLAMPushSensorData(XRSLAMSensorType sensor_type, void *sensor_data);

/**
 * @brief end one frame input and run slam
 */
void XRSLAMRunOneFrame();

void XRSLAMSetViewer(void *viewer);

/**
 * @brief get SLAM results
 * @param[in]  result_type slam result type.
 * @param[out] result_data result data.
 */
/*!
 * \brief Get the IMU-propagated pose (upstream predict_pose at the newest IMU timestamp).
 * Additive to XRSLAMGetResult, whose behaviour is unchanged.
 */
void XRSLAMGetPropagatedPose(XRSLAMPose *pose);

void XRSLAMGetResult(XRSLAMResultType result_type, void *result_data);

/**
 * @brief destroy SLAM system
 */
void XRSLAMDestroy();

#ifdef __cplusplus
}
#endif

#endif // _XRSLAM_H_
