#ifndef XRSLAM_PW_BACKEND_POSE_TAP_H
#define XRSLAM_PW_BACKEND_POSE_TAP_H

// [bench 2026-09-25] 后端位姿出口(只读)。
//
// 为什么要它:对外输出 XRSLAM_RESULT_CAMERA_POSE 是前端在「后端最近一次优化好的状态」上用 IMU
// 预积分外推到当前帧(detail.cpp predict_pose);XRSLAM_LOWLATENCY_POSE 下 FeatureTracker::solve_pnp()
// 只建因子不求解(feature_tracker.cpp,上游 4beb1a9 原样),所以它没被图像校正过。后端
// SlidingWindowTracker 每 sliding_window_tracker_frequent 帧收 1 帧做视觉 + IMU 联合优化,
// 优化后的帧状态本来就在 map 里,只是没有出口。用户 2026-09-25 拍板「重建直接用后端优化位姿」。
//
// 本出口只把后端 map 里**已有**的帧状态原样抄出来,不算任何东西、不改任何状态、不插值:
//   kPwBackendPoseFirst  这一帧自己那次 track() 刚结束时的状态(= 紧接着写进
//                        FrontendWorker::latest_state 的那一份,frontend_worker.cpp);
//                        初始化成功时窗口里每一帧各一条(初始化 BA 的结果)。
//   kPwBackendPoseFinal  这一帧离开后端窗口前的最后状态:关键帧在 slide_window() 边缘化前、
//                        它挂着的子帧随之 untrack 前、无平移子帧在 refine_subwindow() 合并删除前。
//   kPwBackendPoseWindow 每次 track() 结束时整个窗口(关键帧 + 子帧)的当前值快照,整份覆盖
//                        (上游 XRSLAM_RESULT_LANDMARKS 的做法:sliding_window_tracker.cpp 在 track()
//                        末尾把窗口地标快照写进 inspection 槽,接口层 GetResultLandmarks 读)。
//                        用途:收尾时还在窗口里的帧没有 Final,取它们的当前值。
//
// 位姿口径:frame->pose,即 body(IMU)在 world 系下的位姿,这里不做任何换算;换到相机由接口层
// 按 GetResultCameraPose 的同一条式子做(XRSLAMManager.cpp)。
// 线程:写在后端线程(SlidingWindowTracker 的调用栈里),读在调用方线程;一把独立的锁,
// 不碰任何引擎锁,不影响任何计算。

#include <cstddef>
#include <cstdint>
#include <vector>

#include <xrslam/common.h>

namespace xrslam {

class Frame;
class Map;

enum PwBackendPoseKind : int {
    kPwBackendPoseFirst = 1,
    kPwBackendPoseFinal = 2,
    kPwBackendPoseWindow = 3,
};

struct PwBackendPoseRecord {
    double t;          // frame->image->t,与调用方 PushImage 时给的 timeStamp 逐位相同
    uint64_t frame_id; // frame->id()
    Pose pose;         // frame->pose(body 位姿,world 系)
    int kind;          // PwBackendPoseKind
    int is_keyframe;   // frame->tag(FT_KEYFRAME)
};

// ---- 后端线程写 ----
void pw_backend_pose_emit(const Frame *frame, int kind);
void pw_backend_pose_publish_window(const Map *map);

// ---- 调用方读 ----
// 取走至多 max_n 条 First/Final 事件(按发生顺序,余下留在队列里)。
// *dropped = 自上次取走以来因队列满丢掉的最旧事件数。
void pw_backend_pose_drain(std::vector<PwBackendPoseRecord> &out, size_t max_n, uint64_t *dropped);
// 最近一次 track() 结束时的整窗快照(拷贝)。
void pw_backend_pose_window(std::vector<PwBackendPoseRecord> &out);
// 新会话 / 销毁时清空(不让上一会话的记录漏到下一会话)。
void pw_backend_pose_reset();

} // namespace xrslam

#endif // XRSLAM_PW_BACKEND_POSE_TAP_H
