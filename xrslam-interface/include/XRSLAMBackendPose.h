/**
 * @file XRSLAMBackendPose.h
 * @brief [bench 2026-09-25] 后端(滑动窗口 BA)已优化帧位姿的只读出口。
 *
 * 与 XRSLAMGetSolverCounters / XRSLAMGetInitCounters 一样是**附加**函数:XRSLAM.h 一个字节不动,
 * 现有枚举值、结构体布局、XRSLAMGetResult 的行为全部不变。
 *
 * 对标上游 XRSLAM_RESULT_LANDMARKS:后端在 SlidingWindowTracker::track() 末尾把窗口状态写成快照,
 * 接口层读快照交给调用方(上游那条是 inspection 槽 sliding_window_landmarks → GetResultLandmarks)。
 * 本出口多一条按发生顺序的 First / Final 事件队列,保证调用方轮询慢于后端时也不漏帧。
 *
 * 位姿口径与 XRSLAM_RESULT_CAMERA_POSE 完全相同:同一个 world 系、同一条 body→camera 式子
 * (XRSLAMManager::GetResultCameraPose),四元数 [x, y, z, w],平移是相机中心在 world 系的坐标;
 * 另附 body(IMU)位姿原值。不插值、不平滑、不外推:只有进过后端的帧才会出现在这里。
 */

#ifndef _XRSLAM_BACKEND_POSE_H_
#define _XRSLAM_BACKEND_POSE_H_

#ifdef __cplusplus
extern "C" {
#endif

/** 记录种类。数值固定,只加不改。 */
enum {
    XRSLAM_BACKEND_POSE_FIRST = 1,  /*!< 这一帧自己那次后端 track() 刚结束时的状态(首次后端估计);
                                         初始化成功时窗口里每一帧各一条。 */
    XRSLAM_BACKEND_POSE_FINAL = 2,  /*!< 这一帧离开后端窗口前的最后状态(关键帧边缘化前 /
                                         其子帧随之移出前 / 无平移子帧合并删除前)= 定稿值。 */
    XRSLAM_BACKEND_POSE_WINDOW = 3  /*!< 最近一次 track() 结束时仍在窗口里的帧的当前值(整窗快照)。 */
};

typedef struct XRSLAMBackendPose {
    double timestamp;            /*!< 帧时间,与 PushImage 时 XRSLAMImage.timeStamp 逐位相同。 */
    double quaternion[4];        /*!< 相机位姿旋转 [x, y, z, w](同 XRSLAM_RESULT_CAMERA_POSE)。 */
    double translation[3];       /*!< 相机位姿平移(同 XRSLAM_RESULT_CAMERA_POSE)。 */
    double body_quaternion[4];   /*!< body(IMU)位姿旋转原值 [x, y, z, w](后端 frame->pose)。 */
    double body_translation[3];  /*!< body(IMU)位姿平移原值。 */
    unsigned long long frame_id; /*!< 引擎内部帧号(frame->id())。 */
    int kind;                    /*!< XRSLAM_BACKEND_POSE_*。 */
    int is_keyframe;             /*!< 1 = 此刻是关键帧。 */
} XRSLAMBackendPose; /* 136 字节(XRSLAMManager.cpp 里 static_assert 钉住);调用方如自带声明须同样核对。 */

/**
 * [2026-09-25] 同一条记录再附上后端帧的速度与零偏(frame->motion 原值,不换算、不插值)。
 * 为了不动上面 136 字节的 XRSLAMBackendPose(已有调用方按它的布局分配缓冲区),另起一个结构和
 * 两个函数;pose 成员与 XRSLAMBackendPose 逐字段相同。
 */
typedef struct XRSLAMBackendState {
    XRSLAMBackendPose pose;      /*!< 与 XRSLAMDrainBackendPoses 给的那条完全相同。 */
    double velocity[3];          /*!< body(IMU)在 world 系下的速度 [m/s](后端 frame->motion.v)。 */
    double gyro_bias[3];         /*!< 陀螺零偏 [rad/s](frame->motion.bg,IMU 系)。 */
    double acc_bias[3];          /*!< 加计零偏 [m/s^2](frame->motion.ba,IMU 系)。 */
} XRSLAMBackendState; /* 208 字节(XRSLAMManager.cpp 里 static_assert 钉住)。 */

/**
 * 取走 First / Final 事件(按发生顺序)。最多写 capacity 条到 out,返回写入条数;
 * 队列里多于 capacity 的部分留到下次取。*dropped(可为 NULL)= 引擎内队列满(16384 条)时
 * 丢掉的最旧事件数(自上次取走起)。引擎没在跑时返回 0。
 */
int XRSLAMDrainBackendPoses(XRSLAMBackendPose *out, int capacity, unsigned long long *dropped);

/**
 * 最近一次后端 track() 结束时整个窗口(关键帧 + 子帧)的当前值快照,kind 恒为 WINDOW。
 * 最多写 capacity 条,返回快照总条数(可能大于 capacity,调用方据此扩容重取)。
 */
int XRSLAMGetBackendWindowPoses(XRSLAMBackendPose *out, int capacity);

/**
 * [2026-09-25] 同 XRSLAMDrainBackendPoses / XRSLAMGetBackendWindowPoses,记录多带速度与零偏。
 * 🔴 与 XRSLAMDrainBackendPoses 取的是**同一个**事件队列:一个会话里只用其中一个,否则两边各拿一部分。
 */
int XRSLAMDrainBackendStates(XRSLAMBackendState *out, int capacity, unsigned long long *dropped);
int XRSLAMGetBackendWindowStates(XRSLAMBackendState *out, int capacity);

/**
 * [xr-recon-chain 2026-09-25] 把一条后端帧状态(通常是 FINAL 定稿值)沿引擎缓存的 IMU 外推到 t。
 *
 * 用户 2026-09-25 拍板「后端定稿 + 官方外推」:外推只调用引擎自己的 propagate_state_okvis2
 * (detail.cpp,OKVIS2 离散:端点按时间插值 + 段内梯形),规则与 predict_pose 逐句相同
 * (保留状态时刻处(含)之前最后一个样本;积到 min(t, 已缓存最新样本));不插值、不平滑、不补偿。
 * 样本来源是引擎 track_imu 收到的同一条 IMU 流的留底(Detail::imu_history_,上限 16384 条,
 * 丢最旧)。只能往前推(t ≥ 状态时刻)。
 *
 * 位姿口径与 XRSLAMBackendPose 相同:body 位姿原值 + 相机位姿(先 output_to_body 再 camera_to_body,
 * = XRSLAM_RESULT_CAMERA_POSE)。零偏不随外推改变(OKVIS2 外推同样不改零偏)。
 * 返回值 = out->status。status < 0 时状态未外推,out 里的位姿/速度是输入状态原值(按同一口径换算),
 * timestamp = 状态时刻 —— 调用方必须看 status,不许拿它当 t 时刻的位姿用。
 */
enum {
    XRSLAM_PROPAGATE_OK = 0,                    /*!< 外推到了 t(out->timestamp == t)。 */
    XRSLAM_PROPAGATE_IMU_NOT_YET = 1,           /*!< 缓存的 IMU 还没到 t,按 predict_pose 规则只积到
                                                     最新样本(out->timestamp < t)。 */
    XRSLAM_PROPAGATE_INVALID = -1,              /*!< 空指针 / 引擎没在跑 / 输入四元数全零。 */
    XRSLAM_PROPAGATE_IMU_NOT_COVERED = -2,      /*!< 缓存里没有状态时刻(含)之前的样本(未到或已被挤掉)。 */
    XRSLAM_PROPAGATE_BACKWARD = -3,             /*!< t 早于状态时刻或非有限:只能往前推。 */
    XRSLAM_PROPAGATE_INTEGRATION_REFUSED = -4   /*!< OKVIS2 积分拒绝(样本不足等)。 */
};

typedef struct XRSLAMPropagatedState {
    double timestamp;            /*!< 实际到达的时刻(status == OK 时逐位等于 t)。 */
    double quaternion[4];        /*!< 相机位姿旋转 [x, y, z, w](同 XRSLAMBackendPose.quaternion)。 */
    double translation[3];       /*!< 相机位姿平移(相机中心,world 系)。 */
    double body_quaternion[4];   /*!< body(IMU)位姿旋转 [x, y, z, w]。 */
    double body_translation[3];  /*!< body(IMU)位姿平移。 */
    double velocity[3];          /*!< 外推后的 body 速度 [m/s](world 系)。 */
    double state_timestamp;      /*!< 输入状态的时刻(= state->pose.timestamp)。 */
    double imu_first_t;          /*!< 参与积分的第一个 IMU 样本时刻(≤ 状态时刻);没积分时见 status。 */
    double imu_last_t;           /*!< 参与积分的最后一个 IMU 样本时刻(status == OK 时 ≥ t)。 */
    int imu_samples;             /*!< 参与积分的样本数(t == 状态时刻时为 0)。 */
    int status;                  /*!< XRSLAM_PROPAGATE_*。 */
} XRSLAMPropagatedState; /* 176 字节(XRSLAMManager.cpp 里 static_assert 钉住)。 */

int XRSLAMPropagateBackendState(const XRSLAMBackendState *state, double t,
                                XRSLAMPropagatedState *out);

#ifdef __cplusplus
}
#endif

#endif /* _XRSLAM_BACKEND_POSE_H_ */
