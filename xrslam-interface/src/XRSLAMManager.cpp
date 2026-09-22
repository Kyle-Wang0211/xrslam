#include "XRSLAMManager.h"

#include <atomic>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdint>
#include <cstddef>
#include <cstring>

// [pw] 上游这里硬编码 "0.1.0",而 cmake 生成的 XRSLAM_VERSION_STRING 是
//      "xrslam v0.5.0" —— 两个版本号长期不一致。统一到生成的那个。
#define XRSLAM_VERSION XRSLAM_VERSION_STRING

// [pw] 头文件新鲜度自检。仓库里曾经存在一份 cmake file(COPY) 出来的陈旧
//      include/XRSLAM.h,iOS target 正好吃的是那一份(XRSLAMDepthImage 少了
//      width/height 两个 int ⇒ 24 vs 32 字节的静默 ABI 错位)。任何 TU 若又
//      吃到旧头,在这里编译期就炸,而不是运行期越界读调用方的栈。
static_assert(sizeof(XRSLAMDepthImage) == 32,
              "stale XRSLAM.h on include path (XRSLAMDepthImage missing width/height)");
// [pw] Dart 侧把 landmark 缓冲当扁平 double 数组用(asTypedList 零拷贝),
//      这条断言把"3 个 double 无 padding"从巧合变成契约。
static_assert(sizeof(XRSLAMLandmark) == 3 * sizeof(double),
              "XRSLAMLandmark must stay 3 packed doubles");
// [pw] 同一道闸,给 XRSLAMImage。40 -> 48 是"在末尾追加 width/height"之后的值。
//      任何 TU 若又吃到旧的 40 字节头,PushImage 就会去读调用方结构体后面的
//      8 字节 —— 追加字段在 caller-allocates 结构体上没有自描述前缀,库侧补救
//      不了,只能靠这条断言把"同一次构建里混进陈旧头"变成编译错误。
// [pw] 48 -> 64:末尾又追加了 readout_time / timestamp_convention / reserved0
//      (帧时间戳语义契约,条目 09)。旧字段偏移量一个没动,但**旧头 = 旧 sizeof**,
//      混进来就会让库读到调用方结构体尾部之后的 16 字节。这条断言就是那道闸。
static_assert(sizeof(XRSLAMImage) == 64,
              "stale XRSLAM.h on include path (XRSLAMImage missing "
              "readout_time/timestamp_convention)");
// [pw] 健康结构体的布局是 Dart 侧 Struct 定义的镜像。刻意排成
//      "double 块 -> int64 块 -> int32 块",全程零隐式 padding。
//        v1:  9*8 + 14*8 + 12*4 = 232
//        v2: +1*8 + 2*8  + 8*4  =  56  (核内直读遥测块) => 288
//      字段只许追加在末尾,追加时同步改这个数字。
static_assert(sizeof(XRSLAMHealth) == 288,
              "XRSLAMHealth layout changed; update the Dart mirror and this assert");
// [pw] 上面那条只钉总长,钉不住"字段被重排"。下面这三条钉住 v1/v2 分界线上的
//      三个偏移量:任何一个 v1 字段被插入/删除/换位,总长可能碰巧不变,但偏移会变。
static_assert(offsetof(XRSLAMHealth, overall) == 9 * 8 + 14 * 8,
              "XRSLAMHealth int32 block moved; v1 field order was changed");
static_assert(offsetof(XRSLAMHealth, core_frame_timestamp) == 232,
              "XRSLAMHealth v2 block moved; v1 fields were reordered or resized");
static_assert(offsetof(XRSLAMHealth, latest_pose_degenerate) + sizeof(int32_t) ==
                  sizeof(XRSLAMHealth),
              "XRSLAMHealth has trailing implicit padding; add explicit reserved");
static_assert(sizeof(XRSLAMLandmarkStats) == 4 * sizeof(int32_t),
              "XRSLAMLandmarkStats must stay 4 packed int32");
// [pw] TryGetLatestPose 的 out_pose7 布局 [qx,qy,qz,qw,px,py,pz] 就是把
//      XRSLAMPose 的前两个成员摊平。这条断言保证摊平是合法的(无 padding、
//      quaternion 在前、translation 紧随其后)。
static_assert(offsetof(XRSLAMPose, quaternion) == 0 &&
                  offsetof(XRSLAMPose, translation) == 4 * sizeof(double) &&
                  offsetof(XRSLAMPose, timestamp) == 7 * sizeof(double),
              "XRSLAMPose layout changed; out_pose7 [qx,qy,qz,qw,px,py,pz] no longer holds");

namespace xrslam {

namespace {
// [pw] 清空 landmark 槽位。InspectionSupport 的存储是**进程级 static**,与
//      XRSLAMManager 的生命周期完全解耦,而全仓只有两处写、零处 reset ⇒
//      Destroy 后再 Create,新会话的头几帧会拿到上一次扫描的地图。
//      inspection 关闭时整块被丢弃,是合法的 no-op。
void clear_landmark_slot() {
    inspect_debug(sliding_window_landmarks, lm) { lm.reset(); }
}
// [pw] 旧调用方(width/height 填 0)只警告一次,不按帧率刷屏。
std::atomic<bool> g_warned_image_size_missing{false};
// [pw] 时间戳语义未声明 / 域错配,同样各只警告一次。
std::atomic<bool> g_warned_ts_unspecified{false};
std::atomic<bool> g_warned_domain_mismatch{false};

// [pw] 把调用方声明的时间戳语义换算到本库的 canonical 口径
//      (中心行曝光中点)。见 XRSLAM.h 的"帧时间戳语义契约"。
//      exposure/readout 只有在有限且 >= 0 时才被采用,否则当 0 处理 ——
//      ext 是裸指针且上游调用点大多不填,不能让一个垃圾值污染时间轴。
double canonicalize_image_timestamp(const XRSLAMImage *image) {
    double t = image->timeStamp;
    const double exposure =
        (image->ext && std::isfinite(image->ext->exposure_time) &&
         image->ext->exposure_time >= 0.0)
            ? image->ext->exposure_time
            : 0.0;
    const double readout =
        (std::isfinite(image->readout_time) && image->readout_time >= 0.0)
            ? image->readout_time
            : 0.0;
    switch (image->timestamp_convention) {
    case XRSLAM_TS_FIRST_ROW_EXPOSURE_START:
        t += 0.5 * exposure + 0.5 * readout;
        break;
    case XRSLAM_TS_FIRST_ROW_EXPOSURE_MID:
        t += 0.5 * readout;
        break;
    case XRSLAM_TS_CANONICAL:
    case XRSLAM_TS_UNSPECIFIED:
    default:
        break; // 原值使用
    }
    return t;
}
// [pw] 位姿退化判据。detail.cpp:predict_pose 在 feature_tracker 还没有
//      latest_state 时走 else 分支 `output_pose.q.coeffs().setZero()`,
//      而 track_camera 照样把这个**零四元数**连同一个合法的、还在前进的
//      时间戳存进 latest_pose_。零四元数不是旋转:normalize 出 NaN,
//      旋转向量出零向量。所以"时间戳在前进"不能当作位姿可用的判据。
//      阈值取 1e-6:合法旋转的模恒为 1,数值噪声离 1e-6 有六个数量级。
bool pose_is_degenerate(const XRSLAMPose &p) {
    double n2 = 0.0;
    for (int i = 0; i < 4; ++i) {
        if (!std::isfinite(p.quaternion[i])) return true;
        n2 += p.quaternion[i] * p.quaternion[i];
    }
    for (int i = 0; i < 3; ++i)
        if (!std::isfinite(p.translation[i])) return true;
    return n2 < 1e-12; // ‖q‖ < 1e-6
}
} // namespace
XRSLAMManager &XRSLAMManager::Instance() {
    static XRSLAMManager SLAMManagerInstance;
    return SLAMManagerInstance;
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
    // [pw] 新会话不得继承上一次扫描留在进程级 inspection 槽位里的地图。
    clear_landmark_slot();
    {
        // [pw] 计数器同样是会话级的 —— 不清零的话上一次扫描的域错配会一直挂着。
        //      阈值**不**重置(SetHealthThresholds 可以在 Create 之前调)。
        std::lock_guard<std::mutex> g(health_mutex_);
        reset_health_locked();
    }
    {
        std::lock_guard<std::mutex> g(pose_serve_mutex_);
        last_served_pose_t_ = 0.0;
    }
    detail_ = std::make_unique<XRSLAM::Detail>(config);
    config_ = config;
    log_message(XRSLAM_LOG_INFO, (char *)logo_ascii, XRSLAM_VERSION_STRING);
    config_->log_config();
    std::cout << "-----------------Create XRSLAM v" << XRSLAM_VERSION
              << " successfully-----------" << std::endl;
}

void XRSLAMManager::Destroy() {
    // [pw] 上游这里只有一句 cout,detail_/config_ 从不释放 ⇒ (1) 资源到进程
    //      退出才回收;(2) 任何 `if (!detail_)` 类守卫在 Destroy 之后永远为真,
    //      挡不住 use-after-destroy。这里真正释放,配合 ready() 才有意义。
    //      前提是所有 push/getter 路径都已加 ready() 闸(见本文件下方)。
    detail_.reset();
    config_.reset();
    clear_landmark_slot();
    {
        std::lock_guard<std::mutex> g(health_mutex_);
        reset_health_locked();
    }
    {
        std::lock_guard<std::mutex> g(pose_serve_mutex_);
        last_served_pose_t_ = 0.0;
    }
    {
        std::lock_guard<std::mutex> lck(input_mutex_);
        cur_image_.reset();
        cur_depth_.reset();
    }
    std::cout << "-----------------Destroy XRSLAM v" << XRSLAM_VERSION
              << " successfully-----------" << std::endl;
}

int XRSLAMManager::CheckLicense(const char *license_path,
                                const char *product_name) {
    // [pw] 上游是空壳(恒返回 1)。显式 (void) 掉两个参数,免得
    //      -Wall -Wextra 下两条 -Wunused-parameter 淹掉真正的告警。
    (void)license_path;
    (void)product_name;
    return 1;
}

/******************************************************************************************
 * [pw] ★ 时间戳自检闸(Blocker 01 能在核内做的那一半)★
 *
 * td(camera-IMU time offset)进状态量是大工程,这一轮不做 —— state.h 的
 * ES_SIZE 是 15,误差状态只有 Q/P/V/BG/BA。这一轮做的是**让它不再静默**:
 *   1. 每路时间戳各自**严格单调递增**。不只是卫生问题:detail.cpp 里
 *      track_gyroscope 会算 (acc.t - g[0].t) / (t - g[0].t),重复时间戳
 *      直接除零 ⇒ lambda 为 Inf/NaN ⇒ 插值出来的 w 是 NaN ⇒ 整条预积分被毒化。
 *   2. 跨路合理性:|t_cam - t_imu| 超阈值 = 时钟域错配的强信号。
 *      **不拒样本**(拒了等于整条流全丢),只计数 + 置 domain_mismatch_active
 *      + 警告一次,由上层决定怎么处理。
 *   3. 基线:两路都到齐的第一时刻记下 delta 作基线,之后偏离基线过大同样报警。
 *   4. 影子队列:复刻 detail.cpp track_imu 的归并谓词,算出每帧真正进
 *      preintegration 的 IMU 样本数。为 0 = 这一帧一个 IMU 因子都没有
 *      = 系统已经退化成纯单目,而 get_system_state() 照样报 SYS_TRACKING。
 ******************************************************************************************/

void XRSLAMManager::reset_health_locked() {
    h_last_image_t_ = h_last_imu_t_ = h_last_accel_t_ = h_last_gyro_t_ = 0.0;
    h_last_cam_imu_delta_ = h_baseline_delta_ = 0.0;
    h_max_abs_delta_ = h_max_abs_drift_ = h_last_frame_ms_ = 0.0;
    h_baseline_set_ = false;
    h_domain_mismatch_ = false;
    h_image_accepted_ = h_image_rejected_ = 0;
    h_accel_accepted_ = h_accel_rejected_ = 0;
    h_gyro_accepted_ = h_gyro_rejected_ = 0;
    h_reject_non_finite_ = h_reject_non_monotonic_ = h_reject_bad_arg_ = 0;
    h_domain_events_ = h_drift_events_ = 0;
    h_frames_run_ = h_frames_zero_imu_ = h_shadow_overflow_ = 0;
    h_imu_last_frame_ = -1;
    h_ts_convention_ = 0;
    shadow_imu_ts_.clear();
    shadow_frame_ts_.clear();
    shadow_pending_imu_ = 0;
}

int XRSLAMManager::gate_monotonic_locked(double t, double *last_t) {
    if (!std::isfinite(t)) {
        ++h_reject_non_finite_;
        return XRSLAM_ERR_BAD_ARG;
    }
    // 第一枚样本(*last_t == 0.0)无条件放行。用 0.0 当"未初始化"是安全的:
    // 时间戳为 0 的真实样本对 VIO 没有意义,而 reset_health_locked 会把它清回 0。
    if (*last_t != 0.0 && t <= *last_t) {
        ++h_reject_non_monotonic_;
        return XRSLAM_ERR_BAD_ARG;
    }
    *last_t = t;
    return XRSLAM_OK;
}

void XRSLAMManager::note_cross_channel_locked(bool is_camera, double t) {
    if (is_camera) {
        h_last_image_t_ = t;
    } else if (t > h_last_imu_t_) {
        h_last_imu_t_ = t;
    }
    if (h_last_image_t_ == 0.0 || h_last_imu_t_ == 0.0)
        return; // 还没两路都到齐,没法比

    // [pw] 2026-08-24 修正误报。
    //
    // 原来这一段对**每一次样本推入**都算 delta,包括只推 IMU 的时候。相机一停
    // (拍摄结束、会话被打断、热降频掐掉相机流),h_last_imu_t_ 继续前进而
    // h_last_image_t_ 冻住 ⇒ delta 单调增长 ⇒ domain_mismatch_active 被点亮。
    //
    // 真机实测:相机停后的 140 秒里图像 0 帧、IMU 100.3 Hz,
    // lastCamImuDelta 从 -23.0s 长到 -163.0s,差值 140s **正好等于这段间隔** ——
    // 它量的根本不是时钟漂移,是「距上一帧过了多久」。这个误报把排查方向
    // 带去了时钟域,而真相是相机停了。
    //
    // 时钟域错配的正确判据是**相机帧刚到的那一刻**两路时间戳的差:那时两路
    // 都是新鲜的,差值才反映时钟基准,不反映谁停了。
    // 「相机停了」是另一件事,由调用方单独报(Swift 侧本来就同时知道两路的
    // 最后时刻,在那边算是零 ABI 改动;往 XRSLAMHealth 追加字段要动
    // static_assert 钉住的 sizeof 并重跑 ffigen,不值得)。
    if (!is_camera)
        return;

    const double delta = h_last_image_t_ - h_last_imu_t_;
    h_last_cam_imu_delta_ = delta;
    const double abs_delta = std::fabs(delta);
    if (abs_delta > h_max_abs_delta_)
        h_max_abs_delta_ = abs_delta;

    if (!h_baseline_set_) {
        h_baseline_delta_ = delta;
        h_baseline_set_ = true;
    } else {
        const double drift = std::fabs(delta - h_baseline_delta_);
        if (drift > h_max_abs_drift_)
            h_max_abs_drift_ = drift;
        if (drift > h_thr_drift_)
            ++h_drift_events_;
    }

    const bool mismatch = (abs_delta > h_thr_delta_);
    h_domain_mismatch_ = mismatch;
    if (mismatch) {
        ++h_domain_events_;
        if (!g_warned_domain_mismatch.exchange(true)) {
            std::cerr
                << "XRSLAM: |t_cam - t_imu| = " << abs_delta << " s exceeds "
                << h_thr_delta_
                << " s. The two streams look like they are in DIFFERENT clock "
                   "domains. detail.cpp's merge predicate then never fires, "
                   "Frame::preintegration.data stays empty, PreIntegrator::integrate "
                   "returns false, and the solver runs with ZERO imu factors -- "
                   "i.e. the system silently degrades to monocular SfM while still "
                   "reporting TRACKING_SUCCESS. Samples are NOT dropped; read "
                   "XRSLAMGetHealth (domain_mismatch_active / imu_samples_last_frame). "
                   "This warning is printed once." << std::endl;
        }
    }
}

void XRSLAMManager::shadow_drain_locked() {
    // [pw] 与 XRSLAM::Detail::track_imu 的 while 循环逐条对应:
    //        while (imus.size() && frames.size()) {
    //          if (imus.front().t <= frames.front()->image->t) { 入帧; pop imu; }
    //          else { 派发该帧; pop frame; }
    //        }
    while (!shadow_imu_ts_.empty() && !shadow_frame_ts_.empty()) {
        if (shadow_imu_ts_.front() <= shadow_frame_ts_.front()) {
            ++shadow_pending_imu_;
            shadow_imu_ts_.pop_front();
        } else {
            h_imu_last_frame_ = shadow_pending_imu_;
            if (shadow_pending_imu_ == 0)
                ++h_frames_zero_imu_;
            shadow_pending_imu_ = 0;
            shadow_frame_ts_.pop_front();
        }
    }
    // 有界化。任何一侧持续堆积本身就是域错配:
    //   t_imu >> t_cam -> 帧立刻派发、IMU 堆在 shadow_imu_ts_(纯单目);
    //   t_imu << t_cam -> 帧永远等不到派发、堆在 shadow_frame_ts_。
    while (shadow_imu_ts_.size() > kShadowCap) {
        shadow_imu_ts_.pop_front();
        ++h_shadow_overflow_;
    }
    while (shadow_frame_ts_.size() > kShadowCap) {
        shadow_frame_ts_.pop_front();
        ++h_shadow_overflow_;
    }
}

int XRSLAMManager::PushImage(XRSLAMImage *image) {
    if (!image || !image->data) {
        std::lock_guard<std::mutex> g(health_mutex_);
        ++h_image_rejected_; ++h_reject_bad_arg_;
        return XRSLAM_ERR_BAD_ARG;
    }
    if (!ready()) return XRSLAM_ERR_NOT_CREATED; // [pw] 下面立刻解引用 config_
    // left img
    if (image->camera_id != 0) return XRSLAM_OK; // [pw] 保留上游语义:非 0 相机静默忽略

    // [pw] ---- 时间戳语义换算 + 自检闸(条目 09 + Blocker 01)----
    //      必须在任何解码工作之前:被闸拒的帧不该付 clone/cvtColor 的钱。
    //      注意锁序:这里取的是 health_mutex_,函数尾部才取 input_mutex_,
    //      两把锁全程不嵌套。
    const double t_canonical = canonicalize_image_timestamp(image);
    {
        std::lock_guard<std::mutex> g(health_mutex_);
        h_ts_convention_ = image->timestamp_convention;
        if (image->timestamp_convention == XRSLAM_TS_UNSPECIFIED &&
            !g_warned_ts_unspecified.exchange(true)) {
            std::cerr
                << "XRSLAM: XRSLAMImage.timestamp_convention == "
                   "XRSLAM_TS_UNSPECIFIED; the timestamp is used as-is. Android "
                   "SENSOR_TIMESTAMP is first-row exposure START, iOS "
                   "presentationTimeStamp is UNDOCUMENTED -- the two ends are NOT "
                   "equivalent, and this library has no td state to absorb the "
                   "difference (state.h ES_SIZE == 15, no td). Declare the "
                   "convention (see XRSLAM.h). This warning is printed once."
                << std::endl;
        }
        const int rc = gate_monotonic_locked(t_canonical, &h_last_image_t_);
        if (rc != XRSLAM_OK) {
            ++h_image_rejected_;
            std::cerr << "XRSLAM: image timestamp " << image->timeStamp
                      << " (canonical " << t_canonical
                      << ") is not finite or not strictly increasing, frame dropped"
                      << std::endl;
            return rc;
        }
        note_cross_channel_locked(/*is_camera=*/true, t_canonical);
    }

    // [pw] ---- 分辨率来源 ----
    // 旧代码这里直接拿 yaml 的 cam0.resolution 当图像尺寸。调用方喂进来的图只要
    // 比 yaml 矮,下面的 clone()/cvtColor() 就读过调用方缓冲区尾部(实测:实际
    // 640x360、yaml 640x480 ⇒ 一次 307200 字节 memcpy 从 230400 字节堆块起读,
    // 越界 76800 字节)。现在以调用方声明的 width/height 为准。
    const int cfg_cols = (int)config_->camera_resolution()[0];
    const int cfg_rows = (int)config_->camera_resolution()[1];
    if (cfg_cols <= 0 || cfg_rows <= 0) {
        std::lock_guard<std::mutex> g(health_mutex_);
        ++h_image_rejected_;
        return XRSLAM_ERR_INTERNAL;
    }

    int cols = image->width;
    int rows = image->height;
    if (cols == 0 && rows == 0) {
        // 旧 ABI 兼容:调用方没填 -> 回退到 yaml,并且只警告一次(不是静默)。
        if (!g_warned_image_size_missing.exchange(true)) {
            std::cerr
                << "XRSLAM: XRSLAMImage.width/height are 0; falling back to yaml "
                   "cam0.resolution " << cfg_cols << "x" << cfg_rows
                << ". UNCHECKED: if the real frame is smaller than that, PushImage "
                   "reads past your buffer. Fill width/height (and zero-initialize "
                   "the struct). This warning is printed once." << std::endl;
        }
        cols = cfg_cols;
        rows = cfg_rows;
    } else if (cols <= 0 || rows <= 0) {
        std::cerr << "XRSLAM: XRSLAMImage width/height must both be > 0 or both be 0"
                     " (got " << cols << "x" << rows << "), frame dropped" << std::endl;
        std::lock_guard<std::mutex> g(health_mutex_);
        ++h_image_rejected_; ++h_reject_bad_arg_;
        return XRSLAM_ERR_BAD_ARG;
    } else if (cols != cfg_cols || rows != cfg_rows) {
        // [pw] 双分辨率采集端(预览流 + 大图)最容易在这里出事:整条管线的内参
        //      frame->K 来自 yaml(detail.cpp:107),深度配准的 color_width/height
        //      同样来自 yaml(PushDepth),没有任何一处会按实际图像重算 ⇒ 尺寸
        //      不符必须拒绝,而不是"能跑就跑"。
        std::cerr << "XRSLAM: image " << cols << "x" << rows
                  << " does not match yaml cam0.resolution " << cfg_cols << "x"
                  << cfg_rows << "; intrinsics would be wrong, frame dropped"
                  << std::endl;
        std::lock_guard<std::mutex> g(health_mutex_);
        ++h_image_rejected_; ++h_reject_bad_arg_;
        return XRSLAM_ERR_BAD_ARG;
    }

    int cv_type = 0;
    int elem_size = 0; // bytes per pixel
    if (image->channel == 1) {
        cv_type = CV_8UC1; elem_size = 1;
    } else if (image->channel == 3) {
        cv_type = CV_8UC3; elem_size = 3;
    } else if (image->channel == 4) {
        cv_type = CV_8UC4; elem_size = 4;
    } else {
        // [pw] 原为 exit(-1) —— 库不得终止宿主进程(CERT ERR50-CPP)。
        // 丢这一帧并上报,由调用方决定怎么处理。
        std::cerr << "XRSLAM: unsupported image channel " << image->channel
                  << " (expect 1/3/4), frame dropped" << std::endl;
        std::lock_guard<std::mutex> g(health_mutex_);
        ++h_image_rejected_; ++h_reject_bad_arg_;
        return XRSLAM_ERR_BAD_CHANNEL;
    }

    // [pw] ---- stride ----
    // 单位是**字节/行**,与 cv::Mat::step[0] 同义(仓库里三个调用点都是直接
    // `image.stride = mat.step[0]`),不是像素数。
    // 0 == cv::Mat::AUTO_STEP,含义"紧密打包";实测 OpenCV 5.0.0 会填成 cols*elem_size。
    // 负数经 int->size_t 变成天文数字,而 OpenCV 只检查 step >= minstep,
    // (size_t)(-640) 能通过检查、datalimit 回绕 ⇒ 每行都往缓冲区**前面**读。必须自己挡。
    const long long min_stride = (long long)cols * (long long)elem_size;
    long long stride = (long long)image->stride;
    if (stride == 0) stride = min_stride;
    if (stride < min_stride) {
        // [pw] 不挡的话 OpenCV 5.0.0 在 matrix.cpp:833 抛 cv::Exception
        //      ("_step >= minstep"),经 XRSLAMPushSensorDataChecked 的 catch 变成
        //      XRSLAM_ERR_INTERNAL —— 错误码没有信息量,且异常路径每帧走一遍很贵。
        std::cerr << "XRSLAM: stride " << stride << " bytes/row is smaller than "
                  << min_stride << " (= width " << cols << " x " << elem_size
                  << " bytes/pixel), frame dropped" << std::endl;
        std::lock_guard<std::mutex> g(health_mutex_);
        ++h_image_rejected_; ++h_reject_bad_arg_;
        return XRSLAM_ERR_BAD_ARG;
    }

    std::shared_ptr<xrslam::extra::OpenCvImage> opencv_image =
        std::make_shared<xrslam::extra::OpenCvImage>();
    // [pw] 用换算后的 canonical 时间戳,不是调用方给的原值。
    opencv_image->t = t_canonical;

    // [pw] 契约固定为:库最多读到 data[(rows-1)*stride + cols*elem_size - 1],
    //      rows/cols/stride 全部来自调用方自己的声明,不再有 yaml 与真实图像的错位。
    cv::Mat img = cv::Mat(rows, cols, cv_type, image->data, (size_t)stride);
    if (image->channel == 1) {
        opencv_image->image = img.clone();
    } else if (image->channel == 3) {
        cv::cvtColor(img, opencv_image->image, cv::COLOR_BGR2GRAY);
    } else {
        cv::cvtColor(img, opencv_image->image, cv::COLOR_BGRA2GRAY);
    }
            
    opencv_image->raw = img.clone();

    {
        std::lock_guard<std::mutex> lck(input_mutex_);
        // Attach the depth pushed for this frame (if any) and consume it so it is
        // never reused for a later frame that has no depth.
        opencv_image->depth = cur_depth_;
        cur_depth_.reset();
        cur_image_ = std::shared_ptr<xrslam::Image>(opencv_image);
    } // [pw] 显式作用域:input_mutex_ 必须在取 health_mutex_ 之前放掉(锁序铁律)
    {
        std::lock_guard<std::mutex> g(health_mutex_);
        ++h_image_accepted_;
    }
    return XRSLAM_OK;
}
void XRSLAMManager::PushDepth(XRSLAMDepthImage *depth) {
    if (!ready()) return; // [pw] Create 之前 / Destroy 之后 config_ 为空
    if (!config_->depth_fusion_enabled() || depth == nullptr ||
        depth->data == nullptr || depth->width <= 0 || depth->height <= 0) {
        return;
    }
    auto dm = std::make_shared<xrslam::DepthMap>();
    dm->width = depth->width;
    dm->height = depth->height;
    dm->color_width = (int)config_->camera_resolution()[0];
    dm->color_height = (int)config_->camera_resolution()[1];
    dm->t = depth->timeStamp;
    size_t n = (size_t)depth->width * (size_t)depth->height;
    dm->data.resize(n);
    for (size_t i = 0; i < n; ++i) {
        dm->data[i] = depth->data[i] * 0.001f; // millimetres -> metres (0 stays 0)
    }

    std::lock_guard<std::mutex> lck(input_mutex_);
    cur_depth_ = dm;
}

int XRSLAMManager::PushAcceleration(XRSLAMAcceleration *acc) {
    if (!acc) {
        std::lock_guard<std::mutex> g(health_mutex_);
        ++h_accel_rejected_; ++h_reject_bad_arg_;
        return XRSLAM_ERR_BAD_ARG;
    }
    if (!ready()) return XRSLAM_ERR_NOT_CREATED; // [pw]
    {
        std::lock_guard<std::mutex> g(health_mutex_);
        const int rc = gate_monotonic_locked(acc->timestamp, &h_last_accel_t_);
        if (rc != XRSLAM_OK) {
            ++h_accel_rejected_;
            return rc;
        }
        ++h_accel_accepted_;
        note_cross_channel_locked(/*is_camera=*/false, acc->timestamp);
        // [pw] 影子队列只喂加速度计:detail.cpp 里每一个合成出来的 ImuData 都
        //      带**加速度计**的时间戳(track_imu({acc.t, w, acc.a}) 与
        //      track_imu({t, w, {x,y,z}}),t 都是 acc 的 t),陀螺只出插值出来的 w。
        shadow_imu_ts_.push_back(acc->timestamp);
        shadow_drain_locked();
    }
    detail_->track_accelerometer(acc->timestamp, acc->data[0], acc->data[1],
                                 acc->data[2]);
    return XRSLAM_OK;
}

int XRSLAMManager::PushGyroscope(XRSLAMGyroscope *gyro) {
    if (!gyro) {
        std::lock_guard<std::mutex> g(health_mutex_);
        ++h_gyro_rejected_; ++h_reject_bad_arg_;
        return XRSLAM_ERR_BAD_ARG;
    }
    if (!ready()) return XRSLAM_ERR_NOT_CREATED; // [pw]
    {
        std::lock_guard<std::mutex> g(health_mutex_);
        const int rc = gate_monotonic_locked(gyro->timestamp, &h_last_gyro_t_);
        if (rc != XRSLAM_OK) {
            // [pw] 重复/倒退的陀螺时间戳不是卫生问题:detail.cpp track_gyroscope
            //      会算 (acc.t - g[0].t) / (t - g[0].t),t == g[0].t 时直接除零,
            //      插值出的 w 变成 Inf/NaN 并毒化整条预积分。
            ++h_gyro_rejected_;
            return rc;
        }
        ++h_gyro_accepted_;
        note_cross_channel_locked(/*is_camera=*/false, gyro->timestamp);
    }
    detail_->track_gyroscope(gyro->timestamp, gyro->data[0], gyro->data[1],
                             gyro->data[2]);
    return XRSLAM_OK;
}

void XRSLAMManager::RunOneFrame() {
    if (!ready()) return; // [pw]
    const auto t0 = std::chrono::steady_clock::now();
    double frame_t = 0.0;
    bool ran = false;
    {
    std::lock_guard<std::mutex> lck(input_mutex_);
    // [pw] Detail::track_camera 里 `frame->depth = image ? image->depth : nullptr`
    //      看着像判了空,但紧接着的 `predict_pose(image->t)` 是无条件解引用
    //      (detail.cpp:109 vs :124)⇒ cur_image_ 为空时是空指针解引用,不是 C++
    //      异常,extern "C" 上的 catch(...) 拦不住。PushImage 每多一条拒绝路径,
    //      这条就多一分被踩到的机会(现状:Create 之后第一帧 channel 不合法,
    //      再调 RunOneFrame 就直接 segfault)。
    if (!cur_image_) return;
    frame_t = cur_image_->t;
    detail_->track_camera(cur_image_);
    ran = true;
    } // [pw] input_mutex_ 在这里放掉;下面才取 health_mutex_(锁序铁律,绝不嵌套)
    if (!ran) return;
    const double ms = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - t0)
                          .count();
    std::lock_guard<std::mutex> g(health_mutex_);
    h_last_frame_ms_ = ms;
    ++h_frames_run_;
    // [pw] 影子队列:这一帧被 Detail::track_camera 放进 frames 队列;它到底带
    //      几个 IMU 样本,要等有 IMU 的 t 越过 frame_t 才见分晓(核内就是这么
    //      派发的),所以这里只入队,由 shadow_drain_locked 结算。
    shadow_frame_ts_.push_back(frame_t);
    shadow_drain_locked();
}

// [pw] 全部 getter 遵循同一契约:出参判空 -> 无条件清零 -> ready() 闸 -> 再取数。
//      清零必须发生在任何可能提前 return 的分支之前,因为两个上游消费者都是
//      栈上未初始化声明(`XRSLAMLandmarks landmarks;`),出参不写=读垃圾。

int XRSLAMManager::GetBodyPose(XRSLAMPose *pose) const {
    if (!pose) return XRSLAM_ERR_BAD_ARG;
    *pose = XRSLAMPose{};
    if (!ready()) return XRSLAM_ERR_NOT_CREATED;

    std::tuple<double, Pose> latest_state = detail_->get_latest_pose();
    pose->timestamp = std::get<0>(latest_state);
    Pose latest_pose = std::get<1>(latest_state);
    Pose body_pose;
    body_pose.q = latest_pose.q * config_->imu_to_body_rotation();
    body_pose.p =
        latest_pose.p + latest_pose.q * config_->imu_to_body_translation();
    for (int i = 0; i < 3; i++)
        pose->translation[i] = body_pose.p(i);
    pose->quaternion[0] = body_pose.q.x();
    pose->quaternion[1] = body_pose.q.y();
    pose->quaternion[2] = body_pose.q.z();
    pose->quaternion[3] = body_pose.q.w();
    return XRSLAM_OK;
}

int XRSLAMManager::GetCameraPose(XRSLAMPose *pose) const {
    if (!pose) return XRSLAM_ERR_BAD_ARG;
    *pose = XRSLAMPose{};
    if (!ready()) return XRSLAM_ERR_NOT_CREATED;

    std::tuple<double, Pose> latest_state = detail_->get_latest_pose();
    pose->timestamp = std::get<0>(latest_state);
    Pose latest_pose = std::get<1>(latest_state);
    Pose camera_pose;
    camera_pose.q = latest_pose.q * config_->camera_to_body_rotation();
    camera_pose.p =
        latest_pose.p + latest_pose.q * config_->camera_to_body_translation();
    for (int i = 0; i < 3; i++)
        pose->translation[i] = camera_pose.p(i);
    pose->quaternion[0] = camera_pose.q.x();
    pose->quaternion[1] = camera_pose.q.y();
    pose->quaternion[2] = camera_pose.q.z();
    pose->quaternion[3] = camera_pose.q.w();
    return XRSLAM_OK;
}

int XRSLAMManager::GetState(XRSLAMState *state) const {
    if (!state) return XRSLAM_ERR_BAD_ARG;
    // [pw] 上游的 if/else if 链没有 else ⇒ 新增枚举值会让出参保持未初始化。
    //      先钉死一个安全默认值。
    *state = XRSLAM_STATE_TRACKING_FAIL;
    if (!ready()) return XRSLAM_ERR_NOT_CREATED;

    switch (detail_->get_system_state()) {
    case SYS_INITIALIZING:
        *state = XRSLAM_STATE_INITIALIZING;
        break;
    case SYS_TRACKING:
        *state = XRSLAM_STATE_TRACKING_SUCCESS;
        break;
    default: // SYS_CRASH / SYS_UNKNOWN / 未来新增
        *state = XRSLAM_STATE_TRACKING_FAIL;
        break;
    }
    return XRSLAM_OK;
}

int XRSLAMManager::GetIntrinsics(XRSLAMIntrinsics *intrinsics) const {
    if (!intrinsics) return XRSLAM_ERR_BAD_ARG;
    *intrinsics = XRSLAMIntrinsics{};
    if (!ready()) return XRSLAM_ERR_NOT_CREATED;

    intrinsics->fx = config_->camera_intrinsic()(0, 0);
    intrinsics->fy = config_->camera_intrinsic()(1, 1);
    intrinsics->cx = config_->camera_intrinsic()(0, 2);
    intrinsics->cy = config_->camera_intrinsic()(1, 2);
    return XRSLAM_OK;
}

int XRSLAMManager::GetBias(XRSLAMIMUBias *bias) const {
    if (!bias) return XRSLAM_ERR_BAD_ARG;
    *bias = XRSLAMIMUBias{};
    if (!ready()) return XRSLAM_ERR_NOT_CREATED;

    bool compiled_out = false;
    inspect_debug(sliding_window_current_bg, bg) {
        // [pw] 指针形式的 any_cast 不抛异常,空槽位返回 nullptr。
        if (auto *gyr_bias = std::any_cast<xrslam::vector<3>>(&bg)) {
            bias->gyr_bias.data[0] = (*gyr_bias)(0);
            bias->gyr_bias.data[1] = (*gyr_bias)(1);
            bias->gyr_bias.data[2] = (*gyr_bias)(2);
        }
    } else {
        compiled_out = true;
    }
    inspect_debug(sliding_window_current_ba, ba) {
        if (auto *acc_bias = std::any_cast<xrslam::vector<3>>(&ba)) {
            bias->acc_bias.data[0] = (*acc_bias)(0);
            bias->acc_bias.data[1] = (*acc_bias)(1);
            bias->acc_bias.data[2] = (*acc_bias)(2);
        }
    }
    return compiled_out ? XRSLAM_ERR_UNAVAILABLE : XRSLAM_OK;
}

int XRSLAMManager::GetVersion(char *out_buf, int32_t *io_len) const {
    if (!io_len) return XRSLAM_ERR_BAD_ARG;
    const int32_t cap = out_buf ? *io_len : 0;
    *io_len = 0;
    if (out_buf && cap <= 0) return XRSLAM_ERR_BAD_ARG;

    const char *v = XRSLAM_VERSION;
    const size_t vn = std::strlen(v);
    const int32_t need = (vn > (size_t)INT32_MAX) ? INT32_MAX : (int32_t)vn;
    if (!out_buf) {
        *io_len = need;
        return XRSLAM_OK;
    }
    const int32_t n = (need < cap - 1) ? need : cap - 1; // 留 1 字节给 '\0'
    std::memcpy(out_buf, v, (size_t)n);
    out_buf[n] = '\0';
    *io_len = n;
    return (n < need) ? XRSLAM_INCOMPLETE : XRSLAM_OK;
}

int XRSLAMManager::GetLandmarks(double *out_xyz, int32_t *io_point_count) const {
    // [pw] 严格口径:isfinite && triangulated。见 XRSLAM.h 的说明。
    return GetLandmarksImpl(out_xyz, nullptr, io_point_count, nullptr,
                            /*require_triangulated=*/true);
}

int XRSLAMManager::GetLandmarksEx(double *out_xyz, unsigned char *out_flags,
                                  int32_t *io_point_count,
                                  XRSLAMLandmarkStats *out_stats) const {
    if (out_stats) *out_stats = XRSLAMLandmarkStats{}; // 无条件先清零
    // [pw] 这条 return 排在 GetLandmarksImpl(唯一的 io_point_count 清零点)之前,
    //      原来会让 out_flags!=NULL && out_xyz==NULL 这条路径**完全不写出参**
    //      (实测调用方的 int32_t 保持 0xABABABAB)——破了本文件顶部的共同契约
    //      「出参非 NULL 时无条件先清零,所以拿到错误码时出参也一定是已定义的」。
    if (io_point_count) *io_point_count = 0;
    if (out_flags && !out_xyz) return XRSLAM_ERR_BAD_ARG;
    return GetLandmarksImpl(out_xyz, out_flags, io_point_count, out_stats,
                            /*require_triangulated=*/false);
}

int XRSLAMManager::GetLandmarksImpl(double *out_xyz, unsigned char *out_flags,
                                    int32_t *io_point_count,
                                    XRSLAMLandmarkStats *out_stats,
                                    bool require_triangulated) const {
    // ---- 1. 校验 + 无条件初始化。必须在 inspect_debug 块之外,
    //         否则 inspection 关闭时整块被丢弃,出参一个字节都不会写。 ----
    if (!io_point_count) return XRSLAM_ERR_BAD_ARG;
    const int32_t cap = out_xyz ? *io_point_count : 0;
    *io_point_count = 0; // 自此出参已定义,后面每条 return 都安全
    if (out_xyz && cap < 0) return XRSLAM_ERR_BAD_ARG;
    if (!ready()) return XRSLAM_ERR_NOT_CREATED;

    // [pw] 跟踪失败时不要交出上一帧的陈旧点云 —— 那个 std::any 槽位是进程级的,
    //      没人清,不主动抑制就会在 UI 上渲染幽灵点。
    const SysState sys = detail_->get_system_state();
    if (sys != SYS_TRACKING && sys != SYS_INITIALIZING) return XRSLAM_OK;

    int32_t available = 0, written = 0;
    int32_t published = 0, rej_nonfinite = 0, rej_untri = 0;
    bool compiled_out = false;

    // ---- 2. 取快照。inspection 关闭时走 else 分支,给出明确错误码,
    //         而不是与"正在跟踪但当前无点"无法区分的静默 0。 ----
    inspect_debug(sliding_window_landmarks, swlandmarks) {
        // [pw] 指针形式的 any_cast:不抛 std::bad_any_cast(上游用的是会抛的
        //      值形式且没有 has_value() 守卫,冷启动必崩且异常会穿过
        //      extern "C" 边界 —— Dart 侧无法捕获,直接 terminate)。
        //      顺带去掉了上游那次整张 vector 的值拷贝。
        if (auto *pts =
                std::any_cast<std::vector<xrslam::Landmark>>(&swlandmarks)) {
            const size_t n = pts->size();
            published = (n > (size_t)INT32_MAX) ? INT32_MAX : (int32_t)n;
            // [pw] ★ 条目 05:发布循环只判 TT_VALID,两类垃圾点必须在这里拦掉 ★
            //   - LandmarkState() 构造时 inv_depth = 0,而
            //     get_landmark_point() = bearing / inv_depth ⇒ IEEE754 除零 ⇒ ±Inf;
            //   - refine_window 的 else 分支给未三角化 track 设 inv_depth = -1.0
            //     却**不清 TT_VALID** ⇒ 有限但落在相机背后的垃圾点。
            //   被丢掉的数量一律计账,绝不静默丢弃。
            for (int32_t i = 0; i < published; ++i) {
                const xrslam::Landmark &lm = (*pts)[(size_t)i];
                const double x = lm.p(0), y = lm.p(1), z = lm.p(2);
                if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
                    ++rej_nonfinite;
                    continue;
                }
                if (!lm.triangulated) {
                    ++rej_untri;
                    if (require_triangulated) continue;
                }
                ++available;
                if (out_xyz && written < cap) {
                    out_xyz[3 * written + 0] = x;
                    out_xyz[3 * written + 1] = y;
                    out_xyz[3 * written + 2] = z;
                    if (out_flags) {
                        out_flags[written] =
                            lm.triangulated
                                ? XRSLAM_LANDMARK_FLAG_TRIANGULATED
                                : (unsigned char)0;
                    }
                    ++written;
                }
            }
        }
    } else {
        compiled_out = true;
    }

    // ---- 3. 回填 ----
    if (compiled_out) return XRSLAM_ERR_UNAVAILABLE;
    if (out_stats) {
        out_stats->published = published;
        out_stats->rejected_non_finite = rej_nonfinite;
        out_stats->rejected_untriangulated = rej_untri;
        out_stats->returned = out_xyz ? written : 0;
    }
    if (!out_xyz) {
        *io_point_count = available;
        return XRSLAM_OK;
    }
    *io_point_count = written;
    return (written < available) ? XRSLAM_INCOMPLETE : XRSLAM_OK;
}

/******************************************************************************************
 * [pw] ★ 健康状态机(条目 16)★
 ******************************************************************************************/
int XRSLAMManager::SetHealthThresholds(double max_abs_cam_imu_delta_sec,
                                       double max_baseline_drift_sec,
                                       int32_t min_landmarks_for_healthy) {
    std::lock_guard<std::mutex> g(health_mutex_);
    if (std::isfinite(max_abs_cam_imu_delta_sec) && max_abs_cam_imu_delta_sec > 0.0)
        h_thr_delta_ = max_abs_cam_imu_delta_sec;
    if (std::isfinite(max_baseline_drift_sec) && max_baseline_drift_sec > 0.0)
        h_thr_drift_ = max_baseline_drift_sec;
    if (min_landmarks_for_healthy >= 0)
        h_thr_min_lms_ = min_landmarks_for_healthy;
    return XRSLAM_OK;
}

int XRSLAMManager::GetHealth(XRSLAMHealth *out) const {
    if (!out) return XRSLAM_ERR_BAD_ARG;
    *out = XRSLAMHealth{}; // 无条件先整体清零

#if defined(XRSLAM_ENABLE_THREADING)
    out->threading_enabled = 1;
#endif
#if !defined(XRSLAM_ENABLE_DEBUG_INSPECTION)
    out->inspection_compiled_out = 1;
#endif

    // ---- 时间戳侧 / 计数(阈值即使未 Create 也要能读回) ----
    int32_t min_landmarks_for_healthy = 0; // 在锁内取,别在锁外读成员
    {
        std::lock_guard<std::mutex> g(health_mutex_);
        out->last_image_timestamp     = h_last_image_t_;
        out->last_imu_timestamp       = h_last_imu_t_;
        out->last_cam_imu_delta       = h_last_cam_imu_delta_;
        out->baseline_cam_imu_delta   = h_baseline_delta_;
        out->max_abs_cam_imu_delta    = h_max_abs_delta_;
        out->cam_imu_delta_threshold  = h_thr_delta_;
        out->baseline_drift_threshold = h_thr_drift_;
        out->max_abs_baseline_drift   = h_max_abs_drift_;
        out->last_frame_ms            = h_last_frame_ms_;

        out->image_accepted          = h_image_accepted_;
        out->image_rejected          = h_image_rejected_;
        out->accel_accepted          = h_accel_accepted_;
        out->accel_rejected          = h_accel_rejected_;
        out->gyro_accepted           = h_gyro_accepted_;
        out->gyro_rejected           = h_gyro_rejected_;
        out->reject_non_finite_ts    = h_reject_non_finite_;
        out->reject_non_monotonic_ts = h_reject_non_monotonic_;
        out->reject_bad_arg          = h_reject_bad_arg_;
        out->domain_mismatch_events  = h_domain_events_;
        out->baseline_drift_events   = h_drift_events_;
        out->frames_run              = h_frames_run_;
        out->frames_with_zero_imu    = h_frames_zero_imu_;
        out->shadow_overflow_drops   = h_shadow_overflow_;

        out->imu_samples_last_frame  = h_imu_last_frame_;
        out->timestamp_convention    = h_ts_convention_;
        out->domain_mismatch_active  = h_domain_mismatch_ ? 1 : 0;
        min_landmarks_for_healthy    = h_thr_min_lms_;
    }

    if (!ready()) {
        out->overall    = XRSLAM_HEALTH_NOT_CREATED;
        out->slam_state = XRSLAM_STATE_TRACKING_FAIL;
        return XRSLAM_OK;
    }

    XRSLAMState st = XRSLAM_STATE_TRACKING_FAIL;
    (void)GetState(&st);
    out->slam_state = (int32_t)st;

    // [pw] 位姿是否退化(零四元数)。见 XRSLAM.h 上 latest_pose_degenerate 的说明。
    {
        XRSLAMPose bp{};
        out->latest_pose_degenerate =
            (GetBodyPose(&bp) == XRSLAM_OK && !pose_is_degenerate(bp)) ? 0 : 1;
    }

    // ---- 视觉侧:landmark 一次扫描 ----
    XRSLAMLandmarkStats stats{};
    int32_t n = 0;
    (void)GetLandmarksImpl(nullptr, nullptr, &n, &stats,
                           /*require_triangulated=*/true);
    out->landmarks_published                = stats.published;
    out->landmarks_usable                   = n;
    out->landmarks_rejected_non_finite      = stats.rejected_non_finite;
    out->landmarks_rejected_untriangulated  = stats.rejected_untriangulated;

    // [pw] bg/ba 槽位是否真被写过。实测:全仓没有任何一处写它们,所以这一位
    //      在当前 xrslam/src 下恒为 0 —— XRSLAMGetBias 永远返回全零且 rc == OK。
    inspect_debug(sliding_window_current_bg, bg) {
        if (bg.has_value()) out->bias_channel_populated = 1;
    }

    // ---- 核内直读遥测(Blocker 01 任务 1.3 / 条目 16 的视觉侧)----
    // [pw] 这不是影子模型:xrslam-core 在 feature_tracker.cpp 里把每帧的
    //      preintegration.data.size()、检出/跟踪/内点数发布到一组 relaxed 原子量,
    //      get_frame_health 是它唯一的只读出口(声明在 xrslam/xrslam.h,与
    //      get_depth_fusion_stats 同款)。core_imu_samples == 0 是"这一帧一个惯性
    //      观测都没有"的**硬证据**,而 get_system_state() 那时照样报 SYS_TRACKING。
    {
        xrslam::FrameHealth fh{};
        get_frame_health(fh);
        out->core_frame_timestamp        = fh.frame_t;
        out->core_frame_seq              = (int64_t)fh.frame_seq;
        out->core_imu_starved_frames     = (int64_t)fh.imu_starved_frames;
        out->core_imu_samples            = fh.imu_samples;
        out->core_imu_samples_integrated = fh.imu_samples_integrated;
        out->detected_keypoints          = fh.detected_keypoints;
        out->tracked_keypoints           = fh.tracked_keypoints;
        out->inlier_keypoints            = fh.inlier_keypoints;
        out->mapped_landmarks            = fh.mapped_landmarks;
        // seq 只在帧真的走到 feature_tracker 时才 +1;seq==0 时上面全是初值,
        // 拿 core_imu_samples==-1 去判 NO_IMU 会误报,所以显式标"不可用"。
        out->core_health_available =
            (fh.frame_seq > 0 && fh.imu_samples >= 0) ? 1 : 0;
    }

    // ---- 总体状态:降级路径优先级 LOST > NO_IMU > LOW_TEXTURE > HEALTHY ----
    //
    // [pw] NO_IMU 的证据分两层,**必须都留着**:
    //   (a) 核内硬证据 core_imu_samples == 0 —— 帧确实走到了跟踪器,但预积分里
    //       一个真实样本都没有。
    //   (b) C API 层证据 domain_mismatch / shadow_overflow / 影子样本数为 0 ——
    //       时钟域错配的典型表现是帧**永远派发不出去**(imus.front().t 恒 >
    //       frames.front()->image->t),那时 feature_tracker 根本不会被调用,
    //       核内那组原子量会**停在旧值甚至从未被写过**,只有 (b) 看得见。
    //   只用 (a) 会漏掉最该抓的那一类;只用 (b) 就是我们上一版的样子(影子模型
    //   与真值不逐帧相等)。所以取并集。
    const bool core_says_no_imu =
        (out->core_health_available == 1 && out->core_imu_samples == 0);
    const bool shadow_says_no_imu =
        (out->domain_mismatch_active != 0) || (out->shadow_overflow_drops > 0) ||
        (out->imu_samples_last_frame == 0);
    if (st == XRSLAM_STATE_TRACKING_FAIL) {
        out->overall = XRSLAM_HEALTH_LOST;
    } else if (core_says_no_imu || shadow_says_no_imu) {
        // imu_samples_last_frame == 0 表示影子模型认为最近一帧派发时预积分窗口里
        // 一个样本都没有;-1 表示还没有任何一帧被派发过(尚不能下结论)。
        out->overall = XRSLAM_HEALTH_DEGRADED_NO_IMU;
    } else if (st == XRSLAM_STATE_TRACKING_SUCCESS &&
               out->landmarks_usable < min_landmarks_for_healthy) {
        out->overall = XRSLAM_HEALTH_DEGRADED_LOW_TEXTURE;
    } else {
        out->overall = XRSLAM_HEALTH_HEALTHY;
    }
    return XRSLAM_OK;
}

/******************************************************************************************
 * [pw] ★ 非阻塞位姿拉取(条目 06)★ 布局 [qx,qy,qz,qw,px,py,pz],body pose。
 ******************************************************************************************/
int XRSLAMManager::TryGetLatestPose(double *out_pose7, double *out_timestamp) {
    if (!out_pose7 || !out_timestamp) return XRSLAM_ERR_BAD_ARG;
    for (int i = 0; i < 7; ++i) out_pose7[i] = 0.0; // 无条件先清零
    *out_timestamp = 0.0;
    if (!ready()) return XRSLAM_ERR_NOT_CREATED;

    XRSLAMPose pose{};
    const int rc = GetBodyPose(&pose);
    if (rc != XRSLAM_OK) return rc;

    // [pw] ★ 退化闸 ★ 跟踪器初始化之前,predict_pose 交出的是**零四元数 +
    //      一个正在前进的时间戳**。只看时间戳的话这里会返回 XRSLAM_OK 并把
    //      (0,0,0,0) 当成合法旋转交给 Dart —— 那正是我们要消灭的那类静默失效。
    //      退化时:出参保持全零、内部"已交付时间戳"**不前进**(否则真位姿
    //      来的时候会被误判成"不比上次新"),返回 NO_NEW_DATA。
    if (pose_is_degenerate(pose)) return XRSLAM_NO_NEW_DATA;

    // [pw] 只有严格更新的时间戳才算"有新位姿"。timestamp == 0 表示从未跟踪过。
    //      注意先判时间戳、后写出参:时间戳不新时出参必须保持全零,
    //      否则调用方无法区分"这是新位姿"和"这是上一帧的残影"。
    {
        std::lock_guard<std::mutex> g(pose_serve_mutex_);
        if (!std::isfinite(pose.timestamp) ||
            pose.timestamp <= last_served_pose_t_)
            return XRSLAM_NO_NEW_DATA;
        last_served_pose_t_ = pose.timestamp;
    }

    out_pose7[0] = pose.quaternion[0];
    out_pose7[1] = pose.quaternion[1];
    out_pose7[2] = pose.quaternion[2];
    out_pose7[3] = pose.quaternion[3];
    out_pose7[4] = pose.translation[0];
    out_pose7[5] = pose.translation[1];
    out_pose7[6] = pose.translation[2];
    *out_timestamp = pose.timestamp;
    return XRSLAM_OK;
}

void XRSLAMManager::GetResultFeatures(XRSLAMFeatures *features) const {
    // [pw] NOT IMPLEMENTED。上游是空函数体,出参从不写入 ⇒ 调用方拿到栈上垃圾
    //      并按野长度遍历(UB)。在有真实数据源之前,至少无条件把出参置空。
    if (!features) return;
    features->features = nullptr;
    features->num_features = 0;
}

// Defined in xrslam-core (sliding_window_tracker.cpp); forward-declared here (already
// inside namespace xrslam) to avoid touching a widely-included header.
void get_depth_fusion_stats(int &seeded, int &total);

void XRSLAMManager::GetDepthFusionStats(int *seeded, int *total) const {
    int s = 0, t = 0;
    get_depth_fusion_stats(s, t);
    if (seeded) *seeded = s;
    if (total) *total = t;
}

} // namespace xrslam
