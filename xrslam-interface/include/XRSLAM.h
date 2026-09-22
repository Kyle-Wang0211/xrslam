/**
 * @file XRSLAM.h
 * @brief XRSlam API
 */

#ifndef _XRSLAM_H_
#define _XRSLAM_H_
#include <stdint.h>


/* [pw] Blocker 02 的 iOS 侧。iOS 走静态库 + libtool 合并,没有我们控制的链接步骤,
 *      --version-script / -exported_symbols_list 都够不着;唯一有效的是编译期可见性。
 *      但一旦对 xrslam-interface 的 TU 开 -fvisibility=hidden,这些 C API 会变成
 *      STV_HIDDEN 而**再也拉不回来**(链接期无法恢复)——实测:没有这个宏就打开
 *      XRSLAM_HIDE_INTERFACE_TU,产物 DEFINED=0,一个符号都不导出。顺序不能反。
 *      __attribute__((used)) 挡的是 Release 下 -dead_strip / LTO 把「没有任何内部
 *      调用者」的导出函数整段丢掉(flutter#62666、opencv_dart#251 都是这个)。*/
#if defined(_WIN32)
#  define XRSLAM_API __declspec(dllexport)
#else
#  define XRSLAM_API __attribute__((visibility("default"))) __attribute__((used))
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief [pw] 返回码。上游的 C API 全部返回 void,库内出错时直接 exit(-1) 杀宿主进程
 *        (iOS 上会被 Apple 视为异常退出;在 Flutter 里会连 Dart VM 一起带走)。
 *        新增 *Checked 变体提供错误通道;旧的 void 版保留为 thin wrapper,ABI 不破。
 *        参考:CERT ERR50-CPP(库不得终止进程)、ERR59-CPP(异常不得穿过 extern "C")。
 */
#define XRSLAM_OK                0
#define XRSLAM_INCOMPLETE        1  /*!< 缓冲区容量不足,已写满,数据被截断。 */
#define XRSLAM_ERR_BAD_ARG      (-1)
#define XRSLAM_ERR_BAD_CHANNEL  (-2)
#define XRSLAM_ERR_NOT_CREATED  (-3)
#define XRSLAM_ERR_INTERNAL     (-4)
#define XRSLAM_ERR_UNAVAILABLE  (-5) /*!< 该数据通道在本次构建中被编译掉了。 */
#define XRSLAM_NO_NEW_DATA       2  /*!< 非阻塞拉取:出参已填,但不比上次新。 */

/* [pw] 注意:XRSLAMCreate 是上游遗留约定 —— 1 成功 / 0 失败,与上面这套
   "0 成功 / 负数失败" 正好相反。所有 *Checked / XRSLAMGet* 新接口用的都是
   上面这套;封装层请只用一个统一的 check 辅助函数,不要各处手写 rc 比较。 */

/******************************************************************************************
 *                                 XRSLAM sensor data
 ******************************************************************************************/
/**
 * @brief input sensor data type
 */
typedef enum XRSLAMSensorType {
    XRSLAM_SENSOR_CAMERA          = 0, /*!< gray image. */
    XRSLAM_SENSOR_DEPTH_CAMERA    = 1, /*!< depth image. */
    XRSLAM_SENSOR_ACCELERATION    = 2, /*!< acceleration. */
    XRSLAM_SENSOR_GYROSCOPE       = 3, /*!< gyroscope. */
    XRSLAM_SENSOR_GRAVITY         = 4, /*!< gravity. */
    XRSLAM_SENSOR_ROTATION_VECTOR = 5, /*!< rotation vector. */
    XRSLAM_SENSOR_UNKNOWN         = 6
} XRSLAMSensorType;

/**
 * @brief Image extension info
 */
typedef struct XRSLAMImageExtension {
    double exposure_time;          /*!< image exposure time. */
    double default_focus_distance; /*!< default focus info. */
    double focal_length;           /*!< current focal length. */
    double focus_distance;         /*!< current focus distance. */
} XRSLAMImageExtension;

/******************************************************************************************
 * [pw] ★ 帧时间戳语义契约(条目 09)★
 *
 * 背景:两端采集层默认给的**不是同一个东西**,而库内没有任何 td(time offset)
 * 状态量 —— `state.h: ES_SIZE == 15`,误差状态只有 Q/P/V/BG/BA。yaml 里那个
 * `camera_time_offset` 是**死旋钮**:只在 yaml_config.cpp 解析、config.cpp 打印,
 * xrslam/src 里零消费。所以时间戳约定必须由采集层负责对齐,库这一层只做声明、
 * 换算与自检。
 *   - Android `SensorTimestamp` / `CaptureResult.SENSOR_TIMESTAMP`:官方文档说明
 *     是**第一行(first row)曝光开始**的时刻。
 *   - iOS `CMSampleBufferGetPresentationTimeStamp`:Apple **没有文档化**它指向
 *     曝光窗口的哪一点。不要假设它等于 Android 的语义。
 *
 * ★ 本库的规范(canonical)约定 ★
 *   XRSLAMImage.timeStamp 一律解释为:
 *      **图像几何中心行(center row)的曝光中点(mid-exposure)**,单位秒,
 *      且与 XRSLAMAcceleration.timestamp / XRSLAMGyroscope.timestamp
 *      **处在同一个时钟域**(同一 epoch、同一单调时钟)。
 *   选这个点的理由:它是全帧曝光能量的一阶矩,也是 rolling shutter 下"平均像素"
 *   的采样时刻,对纯平移/纯旋转都是无偏的一阶近似。
 *
 * 采集层要么自己换算到 canonical 并声明 XRSLAM_TS_CANONICAL,要么如实声明原始
 * 语义 + 填 readout_time,由 PushImage 替你换算。**不声明**(默认 0 =
 * XRSLAM_TS_UNSPECIFIED)时库按原值使用,但会在 XRSLAMHealth 里把
 * `timestamp_convention` 报成 UNSPECIFIED —— 这是"我不知道"的显式记号,不是默认正确。
 ******************************************************************************************/
typedef enum XRSLAMTimestampConvention {
    /*!< 未声明。库按原值使用(与加字段之前的行为逐位一致),并在 health 里
         把 timestamp_convention 报成 0,让上层看得见"没人对齐过"。 */
    XRSLAM_TS_UNSPECIFIED              = 0,
    /*!< 调用方保证已经是"中心行曝光中点"。库原值使用。 */
    XRSLAM_TS_CANONICAL                = 1,
    /*!< 第一行曝光**开始**(Android SENSOR_TIMESTAMP 的官方语义)。
         库换算:t += exposure_time/2 + readout_time/2。 */
    XRSLAM_TS_FIRST_ROW_EXPOSURE_START = 2,
    /*!< 第一行曝光**中点**。库换算:t += readout_time/2。 */
    XRSLAM_TS_FIRST_ROW_EXPOSURE_MID   = 3
} XRSLAMTimestampConvention;

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
    XRSLAMImageExtension *ext; /*!< ext info of image. */
    /* [pw] 以下两个字段是**追加在结构体末尾**的(旧字段的偏移量一个都没动)。
       追加前 sizeof(XRSLAMImage)==40,追加后 ==48;width@40 / height@44。
       为什么必须有:PushImage 以前是拿 yaml 的 cam0.resolution 当图像尺寸去
       构造 cv::Mat,调用方喂进来的图只要比 yaml 矮,clone()/cvtColor() 就直接
       读过调用方缓冲区尾部(实测:实际 640x360、yaml 640x480 ⇒ 一次 307200
       字节的 memcpy 从一块 230400 字节的堆块起读,越界 76800 字节)。

       ⚠️ 必须零初始化。C 用 `XRSLAMImage img = {0};`,C++ 用 `XRSLAMImage img{};`,
       Dart 侧 `calloc<XRSLAMImage>()`。栈上裸声明 `XRSLAMImage img;` 再逐字段赋值
       (上游三个调用点原本的写法)在加了新字段之后拿到的是**垃圾值**而不是 0,
       会被下面的一致性检查判成 XRSLAM_ERR_BAD_ARG 而丢帧。
       同理:用**旧头**(40 字节)编译出来的调用方二进制,配上**新库**,库会读
       调用方那 40 字节结构体之后的 8 字节 —— 追加字段在 caller-allocates 结构体
       上没有自描述前缀,这一点无法在库侧补救,只能靠头与库同版本发布 +
       XRSLAMManager.cpp 里的 sizeof static_assert 挡住"同一次构建里混进陈旧头"。 */
    int width;  /*!< 本帧实际宽度(像素)。0 = 未填,回退到 yaml cam0.resolution 并警告一次。 */
    int height; /*!< 本帧实际高度(像素)。0 = 未填,回退到 yaml cam0.resolution 并警告一次。 */
    /* [pw] 以下三个字段同样是**追加在末尾**的(48 -> 64,旧字段偏移量一个没动)。
       追加规则与 width/height 完全相同:必须零初始化,且头与库必须同版本发布,
       XRSLAMManager.cpp 里的 sizeof static_assert 是唯一的挡板。 */
    /*!< 卷帘快门读出整帧所需时长,单位**秒**。0 = 未知 / 全局快门。
         只有 timestamp_convention 为 FIRST_ROW_* 时才被消费。
         Android:CameraCharacteristics.SENSOR_ROLLING_SHUTTER_SKEW(纳秒)/1e9。
         iOS:AVCaptureDeviceFormat 没有公开这个量,填 0 并自己把 timeStamp
         换算成 canonical(声明 XRSLAM_TS_CANONICAL)。 */
    double readout_time;
    /*!< XRSLAMTimestampConvention。0 = XRSLAM_TS_UNSPECIFIED = 库按原值使用。 */
    int32_t timestamp_convention;
    /*!< 显式填充,保持 sizeof 是 8 的整数倍且没有隐式 padding。必须为 0。 */
    int32_t reserved0;
} XRSLAMImage;

/**
 * @brief input depth image data
 */
typedef struct XRSLAMDepthImage {
    uint16_t *data;       /*!< depth in millimetres (0 = invalid), row-major. */
    uint16_t *confidence; /*!< optional per-pixel confidence (may be null). */
    double timeStamp;     /*!<  timestamp in second. */
    int width;            /*!< depth map width in pixels. */
    int height;           /*!< depth map height in pixels. */
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
    XRSLAM_RESULT_BODY_POSE   = 0, /*!< body pose. */
    XRSLAM_RESULT_CAMERA_POSE = 1, /*!< camera pose. */
    XRSLAM_RESULT_STATE       = 2, /*!< system state. */
    XRSLAM_RESULT_LANDMARKS   = 3, /*!< 3D landmarks. */
    XRSLAM_RESULT_FEATURES    = 4, /*!< 2D features. */
    XRSLAM_RESULT_BIAS        = 5, /*!< imu bias. */
    XRSLAM_RESULT_DEBUG_LOGS  = 6, /*!< [pw] NOT IMPLEMENTED,落到空 break。 */
    XRSLAM_RESULT_VERSION     = 7, /*!< [pw] 已移除,改用 XRSLAMGetVersion。 */
    XRSLAM_RESULT_UNKNOWN     = 8,
    XRSLAM_INFO_INTRINSICS    = 9
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
    XRSLAM_STATE_INITIALIZING     = 0, /*!< SLAM is in initialize state. */
    XRSLAM_STATE_TRACKING_SUCCESS = 1, /*!< tracking, success. */
    XRSLAM_STATE_TRACKING_FAIL    = 2  /*!< tracking, fail. */
} XRSLAMState;

/**
 * @brief  landmark 3d position.
 * @details x, y, z is world position of the point.
 */
typedef struct XRSLAMLandmark {
    double x, y, z;
} XRSLAMLandmark;
/* [pw] 已删除 typedef struct XRSLAMLandmarks { XRSLAMLandmark *landmarks;
   int num_landmarks; }。
   它是 library-allocates 语义:XRSLAMManager::GetResultLandmarks 每次调用
   new XRSLAMLandmark[n],而这套 C API 从来没有任何 Free 函数,两个上游消费者
   也都不释放 ⇒ 每帧泄漏 24×n 字节。改用下面的 XRSLAMGetLandmarks
   (caller-allocates,两段式)。刻意删掉 typedef 而不是只删 XRSLAMGetResult
   的分支:留着 typedef 会让旧调用点编译通过但出参一个字节不写(静默 UB),
   删掉才能逼出硬编译错误。 */

/**
 * @brief  2d corner in image coordinate.
 */
typedef struct XRSLAMFeature {
    double x, y;
} XRSLAMFeature;
/* [pw] 回退到上游 031d812 的纯 C 定义。
   上游 commit 1d4450a(PR #59, 2024-07-17 "update viewer") 把它改成了
   extern "C" 里的 std::vector<Point>,导致纯 C 和 ffigen 直接阻断,
   而 XRSLAMManager::GetResultFeatures 是空函数体、从未写入过数据。 */
/* [pw] NOT IMPLEMENTED —— 预留。XRSLAMManager::GetResultFeatures 目前只把出参
   置空;上游从来没有 2D 特征点的数据源(feature_tracker_painter 那条 inspect
   通道存的是 InspectPainter* 指针,不是点表)。故意不导出类型化的
   XRSLAMGetFeatures,也请在 ffigen 里 exclude 掉,免得 Dart 侧的人去调试
   "为什么特征点永远是空的"。 */
typedef struct XRSLAMFeatures {
    XRSLAMFeature *features;
    int num_features;
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
/******************************************************************************************
 * [pw] ★ 健康状态机(条目 16)★
 *
 * "不崩溃"不等于"没坏掉"。本库有两条**静默退化**路径,错误码一条都盖不住:
 *   (1) t_cam 与 t_imu 不在同一时钟域时,detail.cpp 的归并条件恒假 ⇒
 *       Frame::preintegration.data 为空 ⇒ PreIntegrator::integrate 直接 return
 *       false ⇒ 求解器里**一个 IMU 因子都没有**,系统退化成纯单目 SfM,
 *       却照常返回 XRSLAM_STATE_TRACKING_SUCCESS。
 *   (2) landmark 发布循环只判 TT_VALID,而未三角化的 track 会带着
 *       inv_depth = 0(=> ±Inf)或 -1.0(=> 相机背后的有限垃圾点)被发布。
 * XRSLAMHealth 就是把这两件事(以及时间戳自检的计数)变成**可读数字**。
 *
 * 约束:
 *   - 纯 C 可编译;字段只追加、不重排、不删除;sizeof 由 static_assert 钉住。
 *   - 出参无条件先清零(GetHealth 的第一件事就是 memset)。
 *   - 一律 int32_t / int64_t / double,不用 int(ffigen 在 32/64 位上会漂)。
 ******************************************************************************************/
typedef enum XRSLAMHealthState {
    XRSLAM_HEALTH_NOT_CREATED          = 0, /*!< 未 Create / 已 Destroy。 */
    XRSLAM_HEALTH_HEALTHY              = 1,
    XRSLAM_HEALTH_DEGRADED_NO_IMU      = 2, /*!< 检出上述 (1):IMU 因子为零 / 域错配。 */
    XRSLAM_HEALTH_DEGRADED_LOW_TEXTURE = 3, /*!< 在跟踪,但可用 landmark 数低于阈值。 */
    XRSLAM_HEALTH_LOST                 = 4  /*!< SLAM 自报 TRACKING_FAIL。 */
} XRSLAMHealthState;
/* [pw] ⚠️ HEALTHY **不等于**"位姿可用"。初始化期间 slam_state ==
   XRSLAM_STATE_INITIALIZING,此时既没丢跟踪也不缺 IMU,按定义就是 HEALTHY,
   但 predict_pose 交出的还是零四元数(latest_pose_degenerate == 1)。
   "现在能不能拿位姿"的唯一判据是 **XRSLAMTryGetLatestPose 返回 XRSLAM_OK**。
   overall 回答的是"有没有降级",不是"能不能用"。 */

typedef struct XRSLAMHealth {
    /* ---- 时间戳侧(秒) ---- */
    double last_image_timestamp;    /*!< 最近一次**被接受**的图像时间戳(已按 convention 换算)。 */
    double last_imu_timestamp;      /*!< 最近一次被接受的 IMU 时间戳(acc 与 gyro 取较新者)。 */
    double last_cam_imu_delta;      /*!< last_image_timestamp - last_imu_timestamp。 */
    double baseline_cam_imu_delta;  /*!< 两路都到齐的**第一时刻**记下的差值,作基线。 */
    double max_abs_cam_imu_delta;   /*!< |delta| 的历史最大值。 */
    double cam_imu_delta_threshold; /*!< 当前生效的域错配阈值,默认 1.0 s。 */
    double baseline_drift_threshold;/*!< |delta - baseline| 的告警阈值,默认 0.5 s。 */
    double max_abs_baseline_drift;  /*!< |delta - baseline| 的历史最大值。 */
    double last_frame_ms;           /*!< 最近一次 XRSLAMRunOneFrame 的墙钟耗时(毫秒)。 */

    /* ---- 累计计数 ---- */
    int64_t image_accepted;
    int64_t image_rejected;
    int64_t accel_accepted;
    int64_t accel_rejected;
    int64_t gyro_accepted;
    int64_t gyro_rejected;
    int64_t reject_non_finite_ts;   /*!< 按原因分桶(三路合计):时间戳 NaN/Inf。 */
    int64_t reject_non_monotonic_ts;/*!< 该路时间戳未严格递增(t <= 上一个)。 */
    int64_t reject_bad_arg;         /*!< 其余参数非法(空指针 / 尺寸 / stride / channel)。 */
    int64_t domain_mismatch_events; /*!< |t_cam - t_imu| 超阈值的次数。**不拒样本**,只计数。 */
    int64_t baseline_drift_events;  /*!< |delta - baseline| 超阈值的次数。 */
    int64_t frames_run;             /*!< XRSLAMRunOneFrame 实际跑起来的帧数。 */
    int64_t frames_with_zero_imu;   /*!< 归并窗口内 IMU 样本数为 0 的帧数(见下)。 */
    int64_t shadow_overflow_drops;  /*!< 影子队列溢出丢弃数;>0 本身就是域错配的强信号。 */

    /* ---- 瞬时值 ---- */
    int32_t overall;                /*!< XRSLAMHealthState。 */
    int32_t slam_state;             /*!< XRSLAMState(未创建时为 TRACKING_FAIL)。 */
    /*!< 最近一帧被归并进 preintegration 的 IMU 样本数 —— **C API 层的影子模型**。
         XRSLAMManager 用一对影子队列复刻了 detail.cpp track_imu 的归并谓词
         (imu.t <= frame.t 则入帧,否则派发该帧)。
         ⚠️ 影子模型只吃**加速度计**样本,而 detail.cpp 里一个加速度样本要么立刻
         经插值进 track_imu、要么先缓存到下一枚陀螺到达时才进,还有一类
         (陀螺缓冲为空 / 早于最老陀螺)会被**整个丢弃**。所以本字段是"应该有几个"
         的上界估计,与真值同相位但不逐帧相等。
         ★ 判"这一帧到底有没有用上 IMU"请优先看 `core_imu_samples`(核内直读)。
         本字段的价值在于:它在**图像/IMU 根本没进核**(时间戳被闸拒、域错配导致
         归并谓词恒假)时依然有数,而核内计数器那时压根不会被更新。
         0 = 影子模型认为这一帧一个 IMU 因子都没有;-1 = 还没有帧被派发过。 */
    int32_t imu_samples_last_frame;
    int32_t landmarks_published;             /*!< 发布循环里 TT_VALID 的总数。 */
    int32_t landmarks_usable;                /*!< 三分量 isfinite **且** triangulated 的数量。 */
    int32_t landmarks_rejected_non_finite;   /*!< 因 ±Inf/NaN 被丢弃(inv_depth==0 除零)。 */
    int32_t landmarks_rejected_untriangulated;/*!< 有限但未三角化(inv_depth==-1,相机背后)。 */
    int32_t timestamp_convention;   /*!< 最近一帧声明的 XRSLAMTimestampConvention。 */
    int32_t domain_mismatch_active; /*!< 0/1:当前 |t_cam - t_imu| 是否仍超阈值。 */
    int32_t inspection_compiled_out;/*!< 0/1:XRSLAM_ENABLE_DEBUG_INSPECTION 是否被关掉。 */
    int32_t threading_enabled;      /*!< 0/1:XRSLAM_ENABLE_THREADING(有真后台线程)。 */
    /*!< 0/1:inspection 的 bg/ba 槽位是否真的有人写过。
         ⚠️ 实测:全仓**没有任何一处**写 sliding_window_current_bg /
         sliding_window_current_ba,所以在当前 xrslam/src 下这一位恒为 0,
         XRSLAMGetBias 永远返回全零而 rc == XRSLAM_OK。别把那个零当成"偏置很小"。 */
    int32_t bias_channel_populated;

    /* ================= v2 追加块(只在末尾追加,旧字段偏移量一个没动)=================
     * 来源:xrslam/include/xrslam/xrslam.h 的 `struct FrameHealth` +
     *       `void get_frame_health(FrameHealth&)`(定义在 feature_tracker.cpp)。
     * 这是**核内直读**,不是 C API 层的影子模型 —— 上面 imu_samples_last_frame 那个
     * 是影子,这里这几个是真值。两者都留着是有意的:核内计数器只有在帧真的走到
     * feature_tracker 时才更新,而域错配的典型表现恰恰是"帧永远派发不出去",
     * 那时核内计数器会**停在旧值**,只有影子模型和时间戳计数看得见问题。
     *
     * ⚠️ 逐字段 relaxed 原子读,**不是跨字段一致快照**:同一次调用里不同字段
     *    可能来自相邻两帧。别拿它们做逐帧比值统计。
     * ⚠️ XRSLAM_ENABLE_THREADING=ON(移动端口径)时 feature_tracker 跑在**后台
     *    worker** 上,这一整块相对 XRSLAMRunOneFrame 是**异步落后**的 ——
     *    实测:连推 8 帧后立刻读,core_frame_seq 还是 0;等一会儿才涨到 1。
     *    所以:
     *      · `core_health_available == 0` 是常态,不是故障,更**不能**据此判 NO_IMU;
     *      · 想知道"核落后了多少",比 `frames_run`(C API 层,同步)与
     *        `core_frame_seq`(核内,异步)的差值 —— 差值持续增大 = worker 跟不上;
     *      · THREADING=OFF 的构建里两者逐帧对齐(实测 8 帧 -> seq=8)。
     * 布局:double 块 -> int64 块 -> int32 块,与上面同一套排法,零隐式 padding。
     * ================================================================================ */
    double  core_frame_timestamp;   /*!< 核内最近一帧的图像时间戳(秒)。 */
    int64_t core_frame_seq;         /*!< 每到达一帧 +1。**两次读到同一个值 = 期间没有新帧
                                         走到跟踪器**,是"核卡住了"最直接的判据;
                                         0 = 从来没有帧到达过。 */
    int64_t core_imu_starved_frames;/*!< 累计出现 core_imu_samples == 0 的帧数。 */
    /*!< ★ 判"静默退化成纯单目"就看这个 ★ 该帧**真实** IMU 样本数(补桩之前)。
         0 = 这一帧一个惯性观测都没有;-1 = 还没有任何帧到达跟踪器。 */
    int32_t core_imu_samples;
    /*!< 实际进入预积分的样本数。与上一个字段的差 = feature_tracker 补进去的
         **合成**样本(把上一帧最后一个样本改时间戳)。
         core_imu_samples == 0 而本字段 == 1 ⇒ 纯外推,不是观测,别当成健康。 */
    int32_t core_imu_samples_integrated;
    int32_t detected_keypoints;     /*!< 该帧检出后的关键点总数(含跟踪继承)。-1 = 未知。 */
    int32_t tracked_keypoints;      /*!< 从上一帧成功传递过来的关键点数(LK + 本质矩阵 + 泊松盘之后)。 */
    int32_t inlier_keypoints;       /*!< LK 存活里通过本质矩阵几何校验的内点数。 */
    int32_t mapped_landmarks;       /*!< 滑窗最新帧看到的 VALID&TRIANGULATED&STATIC 轨迹数;
                                         -1 = 后端还没跑起来。 */
    /*!< 0/1:核内遥测这次到底读到没有。0 = 还没有任何帧走到 feature_tracker
         (core_frame_seq == 0 且 core_imu_samples == -1),此时上面 core_* 全是初值,
         **不要**据此判 DEGRADED_NO_IMU。 */
    int32_t core_health_available;
    /*!< 0/1:最近一次可取到的位姿是**退化四元数**(全零 / 非有限)。
         ⚠️ 这不是理论风险,是实测行为:detail.cpp 的 predict_pose 在
         `feature_tracker->get_latest_state()` 还没有值时(跟踪器尚未初始化)
         走 else 分支执行 `output_pose.q.coeffs().setZero()`,而 track_camera
         **照样**把这个零四元数连同一个合法的时间戳存进 latest_pose_。
         零四元数不是旋转:Dart 侧拿去 normalize 会得到 NaN,拿去旋转向量会得到零向量。
         1 = 别用这一帧的位姿。XRSLAMTryGetLatestPose 已经替你挡掉(返回
         XRSLAM_NO_NEW_DATA);XRSLAMGetBodyPose/GetCameraPose 为了不改动既有
         调用方的返回码**仍返回 XRSLAM_OK**,请自己看这一位。 */
    int32_t latest_pose_degenerate;
} XRSLAMHealth;

/**
 * @brief landmark 过滤计账。被丢掉的点必须能被数出来,否则又是一次静默失效。
 */
typedef struct XRSLAMLandmarkStats {
    int32_t published;               /*!< 发布循环里 TT_VALID 的总数(过滤前)。 */
    int32_t rejected_non_finite;     /*!< 三分量任一 NaN/±Inf 而被丢弃。 */
    int32_t rejected_untriangulated; /*!< 有限但 triangulated == false。 */
    int32_t returned;                /*!< 实际写进 out_xyz 的点数(受容量截断)。 */
} XRSLAMLandmarkStats;

/*!< XRSLAMGetLandmarksEx 的 out_flags 位定义。 */
#define XRSLAM_LANDMARK_FLAG_TRIANGULATED  ((unsigned char)0x01)

/* [pw] 已删除 typedef struct XRSLAMStringOutput { int str_length; char *data; }。
   它只被 GetResultVersion 用过,而那里是 output->data = new char[len + 5] +
   全仓零 delete —— 同一类 library-allocates 泄漏。改用 XRSLAMGetVersion
   (caller-allocates)。全仓无任何调用方,删除零风险。 */

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
XRSLAM_API int XRSLAMCreate(const char *slam_config_path, const char *device_config_path,
                 const char *license_path, const char *product_name,
                 void **config);

/**
 * @brief push sensor data to SLAM system
 * @param[in] sensor_type sensor type.
 * @param[in] sensor_data sensor data.
 */
XRSLAM_API void XRSLAMPushSensorData(XRSLAMSensorType sensor_type, void *sensor_data);

/**
 * @brief [pw] 同 XRSLAMPushSensorData,但返回错误码而不是在出错时终止进程。
 * @return XRSLAM_OK 或 XRSLAM_ERR_*
 * @warning 丢帧对 VIO 是危险的(会变成难查的位姿漂移),调用方必须至少对
 *          非 OK 的返回做计数打点,不要静默忽略。
 */
XRSLAM_API int XRSLAMPushSensorDataChecked(XRSLAMSensorType sensor_type, void *sensor_data);

/**
 * @brief end one frame input and run slam
 */
XRSLAM_API void XRSLAMRunOneFrame(void);

/* [pw] 已删除 void XRSLAMSetViewer(void *viewer);
   全仓只有这一行声明,零定义、零调用 —— Dart 侧一旦 lookupFunction 就抛
   ArgumentError,留在公共头里只会误导人以为可用。 */

/**
 * @brief get SLAM results (遗留入口,仅供仓库内 C++ 使用)
 * @param[in]  result_type slam result type.
 * @param[out] result_data result data.
 * @warning [pw] 第二参是裸 void*,传错枚举=可精确构造的越界写(例如给
 *          XRSLAM_RESULT_CAMERA_POSE 传一个 16 字节的结构体,库会写 64 字节)。
 *          Dart 侧类型系统对此零保护 ⇒ **必须在 ffigen 的 functions.exclude
 *          里挡掉本函数**,只用下面的类型化 XRSLAMGet* 接口。
 * @note    XRSLAM_RESULT_LANDMARKS 分支已删除(改用 XRSLAMGetLandmarks);
 *          XRSLAM_RESULT_DEBUG_LOGS 从来没有实现,落到空 break。
 */
XRSLAM_API void XRSLAMGetResult(XRSLAMResultType result_type, void *result_data);

/******************************************************************************************
 *      [pw] 类型化 getter —— dart:ffi 只应该用这一组
 *
 *  共同契约(每一条都在所有返回路径上成立):
 *   - 出参为 NULL          -> 返回 XRSLAM_ERR_BAD_ARG,不写任何内存。
 *   - 出参非 NULL          -> **无条件先被清零/置空**,然后才可能被填。
 *                            所以调用方即使拿到错误码,出参也一定是已定义的。
 *   - SLAM 未 Create 或已 Destroy -> XRSLAM_ERR_NOT_CREATED。
 *   - 该通道被编译掉        -> XRSLAM_ERR_UNAVAILABLE(不是静默返回 0)。
 *   - C++ 异常绝不逃逸出 extern "C" 边界(内部 try/catch 兜底)。
 *
 *  线程约定:这些函数会短暂持有内部互斥锁,**可能阻塞**;而且库内部对 detail_
 *  的读路径没有全覆盖的锁。请把 XRSLAM* 的全部入口收敛到同一个线程/isolate
 *  串行调用(Flutter 推荐:专用 helper isolate 全权持有 SLAM,UI 侧只收快照)。
 *  由于会阻塞,ffigen/@Native 绑定**不要**标 isLeaf: true。
 ******************************************************************************************/

/**
 * @warning [pw] 这两个函数在跟踪器尚未初始化时返回 **XRSLAM_OK 但 quaternion
 *          全为 0** —— 零四元数不是旋转。根因在 detail.cpp:predict_pose 的
 *          else 分支(`output_pose.q.coeffs().setZero()`),而 track_camera 照样
 *          把它存进 latest_pose_。返回码保持 XRSLAM_OK 是为了不改动既有仓内
 *          调用方的行为;**新代码请改用 XRSLAMTryGetLatestPose**(已替你挡掉),
 *          或读 XRSLAMHealth.latest_pose_degenerate。
 */
XRSLAM_API int XRSLAMGetBodyPose(XRSLAMPose *out);
XRSLAM_API int XRSLAMGetCameraPose(XRSLAMPose *out);
XRSLAM_API int XRSLAMGetState(XRSLAMState *out);
XRSLAM_API int XRSLAMGetBias(XRSLAMIMUBias *out);
XRSLAM_API int XRSLAMGetIntrinsics(XRSLAMIntrinsics *out);

/**
 * @brief 版本串,caller-allocates。
 * @param[out]    out_buf  接收缓冲;为 NULL 时只查长度。
 * @param[in,out] io_len   入:out_buf 容量(字节,含结尾 '\0');
 *                         出:实际写入的字符数(不含结尾 '\0')。
 *                         out_buf 为 NULL 时,出=所需字符数(不含 '\0')。
 * @return XRSLAM_OK / XRSLAM_INCOMPLETE(已截断) / XRSLAM_ERR_BAD_ARG
 */
XRSLAM_API int XRSLAMGetVersion(char *out_buf, int32_t *io_len);

/**
 * @brief 取当前滑窗 3D landmarks(caller-allocates,两段式,库内零分配)。
 *
 * 内存 100% 归调用方,本函数不 new 也不 free —— 因此不需要、也不存在配套的
 * Free 函数。取代已删除的 XRSLAM_RESULT_LANDMARKS + XRSLAMLandmarks。
 *
 * @param[out]    out_xyz         接收缓冲,布局是扁平的 [x0,y0,z0, x1,y1,z1, ...],
 *                                即需要 3 * point_count 个 double。
 *                                为 NULL 时只查数量,忽略 *io_point_count 的入值。
 * @param[in,out] io_point_count  入:out_xyz 能装下的**点数**(不是字节数,
 *                                也不是 double 个数);出:实际写入的点数。
 *                                out_xyz 为 NULL 时,出=当前可用点数。
 *                                本参数在**任何**返回路径上都被无条件写入。
 * @return XRSLAM_OK            全部可用点已写入(或查询成功)
 *         XRSLAM_INCOMPLETE    容量不足,已写满 cap 个点,数据被截断
 *         XRSLAM_ERR_BAD_ARG   io_point_count 为 NULL,或 out_xyz 非 NULL 且容量为负
 *         XRSLAM_ERR_NOT_CREATED  SLAM 未创建 / 已销毁
 *         XRSLAM_ERR_UNAVAILABLE  本次构建关掉了 XRSLAM_ENABLE_DEBUG_INSPECTION,
 *                                 landmark 通道被编译掉(不是"当前没有点")
 *
 * @note 跟踪失败(XRSLAM_STATE_TRACKING_FAIL)时返回 0 个点,而不是上一帧的
 *       陈旧点云 —— 内部那个 std::any 槽位是进程级的,不主动抑制就会渲染幽灵点。
 * @note 两次调用之间点数可能变化,所以永远以第二次调用写回的实际点数为准,
 *       不要拿查询结果当长度去遍历。
 * @note 本函数与 XRSLAMGetCameraPose 走的是两把不同的锁、两次独立调用,
 *       **不保证同一帧**。AR 叠加对此敏感的话,只在跟踪稳定时取一次并缓存。
 *
 * @note [pw] ★ 已加过滤(条目 05)★ 本函数只交出**三分量 isfinite 且
 *       triangulated 为真**的点。上游发布循环只判 TT_VALID,于是两类垃圾点会混进来:
 *         - LandmarkState() 构造时 inv_depth = 0,而
 *           get_landmark_point() = bearing / inv_depth ⇒ IEEE754 除零 ⇒ ±Inf;
 *         - refine_window 的 else 分支对未三角化 track 设 inv_depth = -1.0 却
 *           **不清 TT_VALID** ⇒ 有限、但落在相机背后的垃圾点。
 *       被丢掉的数量不会静默消失:用 XRSLAMGetLandmarksEx 的 out_stats,
 *       或读 XRSLAMHealth.landmarks_rejected_*。
 */
XRSLAM_API int XRSLAMGetLandmarks(double *out_xyz, int32_t *io_point_count);

/**
 * @brief 带 triangulated 标志与过滤计账的 landmark 取数(条目 05)。
 *
 * 与 XRSLAMGetLandmarks 的唯一差别:
 *   - **不**过滤 untriangulated —— 交出所有 isfinite 的点,是否采用由调用方按
 *     out_flags 决定(±Inf/NaN 点永远丢弃,它们对任何用途都是垃圾)。
 *   - 额外交出并列的 uint8 标志数组与一份过滤计账。
 * out_xyz 的扁平 [x0,y0,z0,x1,...] 布局**一个字节都没动**,dart:ffi 的
 * asTypedList 零拷贝照旧;out_flags 是**并列的第二块缓冲**,不是交错。
 *
 * @param[out]    out_xyz         同 XRSLAMGetLandmarks;NULL 时只查数量。
 * @param[out]    out_flags       可选,长度 >= 写入点数的 uint8 数组,
 *                                bit0 = XRSLAM_LANDMARK_FLAG_TRIANGULATED。
 *                                只有 out_xyz 非 NULL 时才允许非 NULL,
 *                                否则返回 XRSLAM_ERR_BAD_ARG。
 * @param[in,out] io_point_count  同 XRSLAMGetLandmarks,任何路径上都被写入。
 * @param[out]    out_stats       可选。非 NULL 时**无条件先清零**,再填计账。
 * @return 同 XRSLAMGetLandmarks 的那套码。
 */
XRSLAM_API int XRSLAMGetLandmarksEx(double *out_xyz, unsigned char *out_flags,
                         int32_t *io_point_count, XRSLAMLandmarkStats *out_stats);

/**
 * @brief 读健康快照(条目 16 + Blocker 01 的诊断出口)。
 * @param[out] out 非 NULL;**无条件先整体清零**,然后才可能被填。
 * @return XRSLAM_OK / XRSLAM_ERR_BAD_ARG(out == NULL)。
 *         未 Create 时**也返回 XRSLAM_OK**,out->overall == XRSLAM_HEALTH_NOT_CREATED,
 *         这样"库没起来"和"调用姿势不对"能被区分开。
 * @note 会做一次 O(点数) 的 landmark 扫描以填 landmarks_*,不要每帧调很多次。
 */
XRSLAM_API int XRSLAMGetHealth(XRSLAMHealth *out);

/**
 * @brief 设置自检阈值。可在 XRSLAMCreate 之前调用;Create/Destroy 不会重置它们。
 * @param[in] max_abs_cam_imu_delta_sec  |t_cam - t_imu| 的域错配阈值,<= 0 或非有限
 *                                       则保持原值。默认 1.0。
 * @param[in] max_baseline_drift_sec     |delta - baseline| 的告警阈值,同上。默认 0.5。
 * @param[in] min_landmarks_for_healthy  低于此数且在 TRACKING 中则判 DEGRADED_LOW_TEXTURE,
 *                                       < 0 则保持原值。默认 20。
 * @return XRSLAM_OK
 */
XRSLAM_API int XRSLAMSetHealthThresholds(double max_abs_cam_imu_delta_sec,
                              double max_baseline_drift_sec,
                              int32_t min_landmarks_for_healthy);

/**
 * @brief 非阻塞拉取最新位姿(条目 06 的 API 形状)。
 *
 * 为什么是**拉取**而不是每帧回调:dart:ffi 是同步同线程的。
 * NativeCallable.isolateLocal 从非创建线程调用会**硬 abort**(不是抛异常),
 * 而 XRSLAM_ENABLE_THREADING 打开时库内确有真的后台工作线程;
 * NativeCallable.listener 只支持返回 void 且是异步投递,拿不到同步结果。
 * 所以正确形状是 Dart 侧自己按需 poll 本函数。
 *
 * @param[out] out_pose7     非 NULL,长度 7 的 double 数组。**无条件先清零**。
 *                           布局钉死为 **[qx, qy, qz, qw, px, py, pz]**
 *                           —— 四元数在前、实部 w 在**第 4 位**(与 XRSLAMPose
 *                           的 quaternion[4] = {x,y,z,w} 完全一致),平移在后。
 *                           位姿口径 = **body pose**,与 XRSLAMGetBodyPose 同源
 *                           (即已乘 imu_to_body)。
 * @param[out] out_timestamp 非 NULL,秒。**无条件先清零**。
 * @return XRSLAM_OK           拿到了比上次**更新**、且**几何上合法**的位姿。
 *         XRSLAM_NO_NEW_DATA  没有可用的新位姿。**不阻塞、不等待**。两种情况:
 *                             (a) 时间戳不比上次调用交出的更新;
 *                             (b) 位姿退化 —— 四元数全零或非有限。跟踪器初始化
 *                                 之前 detail.cpp:predict_pose 会 setZero(),而
 *                                 track_camera 照样连着合法时间戳一起存下来,
 *                                 所以"时间戳在前进"**不能**当作位姿可用的判据。
 *                             这两种情况下 out_pose7 保持**全零**、out_timestamp
 *                             保持 0,内部"已交付时间戳"也不前进。
 *         XRSLAM_ERR_BAD_ARG / XRSLAM_ERR_NOT_CREATED / XRSLAM_ERR_INTERNAL
 * @note 位姿只在 XRSLAMRunOneFrame 里前进(Detail::track_camera 才写 latest_pose_);
 *       只 push IMU 不调 RunOneFrame,本函数会一直返回 XRSLAM_NO_NEW_DATA。
 * @note 内部持短锁,可能阻塞在锁上;ffigen/@Native 绑定**不要**标 isLeaf: true。
 */
XRSLAM_API int XRSLAMTryGetLatestPose(double *out_pose7, double *out_timestamp);

/**
 * @brief depth-fusion telemetry from the latest optimization.
 * @param[out] seeded number of active landmarks carrying a LiDAR depth prior.
 * @param[out] total  number of active landmarks in the sliding window.
 */
XRSLAM_API void XRSLAMGetDepthFusionStats(int *seeded, int *total);

/**
 * @brief destroy SLAM system
 */
XRSLAM_API void XRSLAMDestroy(void);

#ifdef __cplusplus
}
#endif

#endif // _XRSLAM_H_
