#include "XRSLAMManager.h"
#include <cstdint>
#include <iostream>
#include "xrslam/xrslam.h"

// [pw] ⚠ 这两个参数是 YAML **正文** 还是 **文件路径**,由编译期开关
//      XRSLAM_CONFIG_FROM_STRING 决定(yaml_config.cpp:153/165):
//        定义了  -> YAML::Load(...)      入参是正文字符串
//        没定义  -> YAML::LoadFile(...)  入参是路径
//      移动端口径(-DXRSLAM_CONFIG_FROM_STRING=ON)走的是**正文**。
//      同一个 C ABI 在两种构建下语义不同,Dart 侧必须按构建口径传对东西。
// [pw] 整体 try/catch:YamlConfig 的构造函数会**抛**(ParseException /
//      LoadException / YAML::BadSubscript ...)。实测:把文件路径喂给
//      CONFIG_FROM_STRING 构建,YAML::Load 把它当成一个标量,随后
//      find_node(device_config, "cam0.intrinsics") 抛 YAML::BadSubscript,
//      异常穿过 extern "C" 边界 -> libc++abi terminate -> **整个进程死**。
//      在 Flutter 里这会连 Dart VM 一起带走,而且 Dart 侧捕获不到。
//      CERT ERR59-CPP:异常不得穿过 extern "C"。
int XRSLAMCreate(
    const char *slam_config_path,   // slam configuration (YAML text or path)
    const char *device_config_path, // device configuration (YAML text or path)
    const char *license_path, const char *product_name, void **config) {
    // [pw] 出参先置空:失败路径上调用方也必须拿到已定义的值。
    //      注意 XRSLAMCreate 用的是**上游遗留**约定 1=成功 / 0=失败。
    if (config) *config = nullptr;
    if (!slam_config_path || !device_config_path || !config) return 0;
    try {
        if (xrslam::XRSLAMManager::Instance().CheckLicense(license_path,
                                                          product_name) == 0)
            return 0;

        std::shared_ptr<xrslam::extra::YamlConfig> yaml_config =
            std::make_shared<xrslam::extra::YamlConfig>(slam_config_path,
                                                        device_config_path);
        xrslam::XRSLAMManager::Instance().Init(yaml_config);
        *config = static_cast<void *>(yaml_config.get());
        return 1;
    } catch (const std::exception &e) {
        std::cerr << "XRSLAM: XRSLAMCreate failed: " << e.what()
                  << " (config args are YAML "
#if defined(XRSLAM_CONFIG_FROM_STRING)
                     "TEXT"
#else
                     "FILE PATHS"
#endif
                     " in this build)" << std::endl;
        return 0;
    } catch (...) {
        std::cerr << "XRSLAM: XRSLAMCreate failed with an unknown exception"
                  << std::endl;
        return 0;
    }
}

// [pw] 带错误通道的入口。extern "C" 边界上异常逃逸是 UB(CERT ERR59-CPP),
// 所以整体包 try/catch,catch(...) 是必须的兜底。
int XRSLAMPushSensorDataChecked(XRSLAMSensorType sensor_type, void *sensor_data) {
    if (!sensor_data) return XRSLAM_ERR_BAD_ARG;
    try {
        switch (sensor_type) {
        case XRSLAM_SENSOR_CAMERA:
            return xrslam::XRSLAMManager::Instance().PushImage(
                static_cast<XRSLAMImage *>(sensor_data));
        case XRSLAM_SENSOR_ACCELERATION:
            // [pw] 原来这里无条件 return XRSLAM_OK,把 Push* 的返回值丢了。
            //      加了时间戳自检闸之后必须透传,否则被拒的样本就是静默丢弃。
            return xrslam::XRSLAMManager::Instance().PushAcceleration(
                static_cast<XRSLAMAcceleration *>(sensor_data));
        case XRSLAM_SENSOR_GYROSCOPE:
            return xrslam::XRSLAMManager::Instance().PushGyroscope(
                static_cast<XRSLAMGyroscope *>(sensor_data));
        case XRSLAM_SENSOR_DEPTH_CAMERA:
            xrslam::XRSLAMManager::Instance().PushDepth(
                static_cast<XRSLAMDepthImage *>(sensor_data));
            return XRSLAM_OK;
        default:
            return XRSLAM_ERR_BAD_ARG;
        }
    } catch (const std::exception &e) {
        std::cerr << "XRSLAM: exception in PushSensorData: " << e.what() << std::endl;
        return XRSLAM_ERR_INTERNAL;
    } catch (...) {
        std::cerr << "XRSLAM: unknown exception in PushSensorData" << std::endl;
        return XRSLAM_ERR_INTERNAL;
    }
}

void XRSLAMPushSensorData(XRSLAMSensorType sensor_type, // sensor type
                          void *sensor_data             // sensor data
) {
    // [pw] thin wrapper,保留旧 ABI。新代码应该用 *Checked 并对返回码打点。
    (void)XRSLAMPushSensorDataChecked(sensor_type, sensor_data);
}


// [pw] RunOneFrame 里跑的是整条前端(OpenCV / Ceres / Eigen),全都会抛。
//      原来这里裸调 —— 任何一次抛出都会穿过 extern "C" 直接 terminate。
void XRSLAMRunOneFrame() {
    try {
        xrslam::XRSLAMManager::Instance().RunOneFrame();
    } catch (const std::exception &e) {
        std::cerr << "XRSLAM: exception in XRSLAMRunOneFrame: " << e.what()
                  << std::endl;
    } catch (...) {
        std::cerr << "XRSLAM: unknown exception in XRSLAMRunOneFrame" << std::endl;
    }
}

void XRSLAMGetResult(XRSLAMResultType result_type, // result type
                     void *result_data             // result data
) {
    // [pw] 遗留入口,仅供仓库内 C++ 使用;ffigen 必须 exclude(裸 void* 无类型保护)。
    //      内部一律转调下面那组类型化 getter,所以出参清零/空指针/异常兜底
    //      对这条路径同样生效。
    switch (result_type) {
    case XRSLAM_RESULT_BODY_POSE:
        (void)XRSLAMGetBodyPose(static_cast<XRSLAMPose *>(result_data));
        break;
    case XRSLAM_RESULT_CAMERA_POSE:
        (void)XRSLAMGetCameraPose(static_cast<XRSLAMPose *>(result_data));
        break;
    case XRSLAM_RESULT_STATE:
        (void)XRSLAMGetState(static_cast<XRSLAMState *>(result_data));
        break;
    case XRSLAM_RESULT_BIAS:
        (void)XRSLAMGetBias(static_cast<XRSLAMIMUBias *>(result_data));
        break;
    case XRSLAM_INFO_INTRINSICS:
        (void)XRSLAMGetIntrinsics(static_cast<XRSLAMIntrinsics *>(result_data));
        break;
    case XRSLAM_RESULT_FEATURES:
        try {
            xrslam::XRSLAMManager::Instance().GetResultFeatures(
                static_cast<XRSLAMFeatures *>(result_data)); // NOT IMPLEMENTED,只置空
        } catch (...) {
        }
        break;
    // [pw] XRSLAM_RESULT_LANDMARKS 分支已删除 -> 用 XRSLAMGetLandmarks。
    //      XRSLAM_RESULT_VERSION  分支已删除 -> 用 XRSLAMGetVersion。
    //      两者都是 library-allocates 且没有配套 Free,每次调用净泄漏。
    case XRSLAM_RESULT_LANDMARKS:
    case XRSLAM_RESULT_VERSION:
    case XRSLAM_RESULT_DEBUG_LOGS: // NOT IMPLEMENTED(从来没有过实现)
    case XRSLAM_RESULT_UNKNOWN:
    default:
        break;
    }
}

/******************************************************************************************
 *  [pw] 类型化 getter 的 C 导出层。
 *  每个 thunk 都必须包 try/catch(...) —— extern "C" 边界上异常逃逸是 UB
 *  (CERT ERR59-CPP),在 dart:ffi 下表现为进程 terminate 且 Dart 侧无法捕获。
 *  注意 catch 里不能再碰出参:出参已在 XRSLAMManager 的 getter 入口无条件清零。
 ******************************************************************************************/
#define XRSLAM_C_THUNK(fn, type, method)                                       \
    int fn(type *out) {                                                        \
        try {                                                                  \
            return xrslam::XRSLAMManager::Instance().method(out);               \
        } catch (const std::exception &e) {                                    \
            std::cerr << "XRSLAM: exception in " #fn ": " << e.what()          \
                      << std::endl;                                            \
            return XRSLAM_ERR_INTERNAL;                                        \
        } catch (...) {                                                        \
            std::cerr << "XRSLAM: unknown exception in " #fn << std::endl;     \
            return XRSLAM_ERR_INTERNAL;                                        \
        }                                                                      \
    }

XRSLAM_C_THUNK(XRSLAMGetBodyPose,   XRSLAMPose,       GetBodyPose)
XRSLAM_C_THUNK(XRSLAMGetCameraPose, XRSLAMPose,       GetCameraPose)
XRSLAM_C_THUNK(XRSLAMGetState,      XRSLAMState,      GetState)
XRSLAM_C_THUNK(XRSLAMGetBias,       XRSLAMIMUBias,    GetBias)
XRSLAM_C_THUNK(XRSLAMGetIntrinsics, XRSLAMIntrinsics, GetIntrinsics)
#undef XRSLAM_C_THUNK

int XRSLAMGetVersion(char *out_buf, int32_t *io_len) {
    try {
        return xrslam::XRSLAMManager::Instance().GetVersion(out_buf, io_len);
    } catch (...) {
        if (io_len) *io_len = 0;
        return XRSLAM_ERR_INTERNAL;
    }
}

int XRSLAMGetLandmarks(double *out_xyz, int32_t *io_point_count) {
    try {
        return xrslam::XRSLAMManager::Instance().GetLandmarks(out_xyz,
                                                             io_point_count);
    } catch (...) {
        // 理论上到不了这里(内部用的是不抛的指针形式 any_cast),留作 ABI 兜底。
        if (io_point_count) *io_point_count = 0;
        return XRSLAM_ERR_INTERNAL;
    }
}

int XRSLAMGetLandmarksEx(double *out_xyz, unsigned char *out_flags,
                         int32_t *io_point_count,
                         XRSLAMLandmarkStats *out_stats) {
    try {
        return xrslam::XRSLAMManager::Instance().GetLandmarksEx(
            out_xyz, out_flags, io_point_count, out_stats);
    } catch (...) {
        if (io_point_count) *io_point_count = 0;
        if (out_stats) *out_stats = XRSLAMLandmarkStats{};
        return XRSLAM_ERR_INTERNAL;
    }
}

int XRSLAMGetHealth(XRSLAMHealth *out) {
    try {
        return xrslam::XRSLAMManager::Instance().GetHealth(out);
    } catch (const std::exception &e) {
        std::cerr << "XRSLAM: exception in XRSLAMGetHealth: " << e.what()
                  << std::endl;
        if (out) *out = XRSLAMHealth{};
        return XRSLAM_ERR_INTERNAL;
    } catch (...) {
        if (out) *out = XRSLAMHealth{};
        return XRSLAM_ERR_INTERNAL;
    }
}

int XRSLAMSetHealthThresholds(double max_abs_cam_imu_delta_sec,
                              double max_baseline_drift_sec,
                              int32_t min_landmarks_for_healthy) {
    try {
        return xrslam::XRSLAMManager::Instance().SetHealthThresholds(
            max_abs_cam_imu_delta_sec, max_baseline_drift_sec,
            min_landmarks_for_healthy);
    } catch (...) {
        return XRSLAM_ERR_INTERNAL;
    }
}

int XRSLAMTryGetLatestPose(double *out_pose7, double *out_timestamp) {
    try {
        return xrslam::XRSLAMManager::Instance().TryGetLatestPose(out_pose7,
                                                                 out_timestamp);
    } catch (const std::exception &e) {
        std::cerr << "XRSLAM: exception in XRSLAMTryGetLatestPose: " << e.what()
                  << std::endl;
        return XRSLAM_ERR_INTERNAL;
    } catch (...) {
        return XRSLAM_ERR_INTERNAL;
    }
}

void XRSLAMGetDepthFusionStats(int *seeded, int *total) {
    // [pw] 出参无条件先清零,与其余 getter 同契约。
    if (seeded) *seeded = 0;
    if (total) *total = 0;
    try {
        xrslam::XRSLAMManager::Instance().GetDepthFusionStats(seeded, total);
    } catch (...) {
        std::cerr << "XRSLAM: unknown exception in XRSLAMGetDepthFusionStats"
                  << std::endl;
    }
}

void XRSLAMDestroy() {
    // [pw] Destroy 会析构整棵 Detail(线程 join、Ceres/OpenCV 资源释放),
    //      同样不能让异常穿过 extern "C"。
    try {
        xrslam::XRSLAMManager::Instance().Destroy();
    } catch (const std::exception &e) {
        std::cerr << "XRSLAM: exception in XRSLAMDestroy: " << e.what()
                  << std::endl;
    } catch (...) {
        std::cerr << "XRSLAM: unknown exception in XRSLAMDestroy" << std::endl;
    }
}
