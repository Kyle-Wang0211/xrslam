#ifndef XRSLAM_PREINTEGRATOR_H
#define XRSLAM_PREINTEGRATOR_H

#include <xrslam/common.h>
#include <xrslam/estimation/state.h>

namespace xrslam {

class Frame;

struct PreIntegrator {
    struct Delta {
        double t;
        quaternion q;
        vector<3> p;
        vector<3> v;
        matrix<15> cov; // ordered in q, p, v, bg, ba
        matrix<15> sqrt_inv_cov;
    };

    struct Jacobian {
        matrix<3> dq_dbg;
        matrix<3> dp_dbg;
        matrix<3> dp_dba;
        matrix<3> dv_dbg;
        matrix<3> dv_dba;
    };

    void reset();
    // [pw 2026-09-25 okvis2-preint] 积分区间显式给起止:t0 = 前一帧(状态 i)时刻,t1 = 本帧(状态 j)时刻。
    // 照 OKVIS2 ImuError 的约定,`data` 是按时间递增的原始样本,须覆盖 [t0, t1](首个样本 <= t0、
    // 末个样本 >= t1),两端按时间线性插值、段内梯形。上游 integrate(t, ...) 靠在 data 头部插一个时间戳改成
    // t0 的「上一帧末样本」拷贝来隐含 t0(左端零阶保持),这个做法已删。
    bool integrate(double t0, double t1, const vector<3> &bg,
                   const vector<3> &ba, bool compute_jacobian,
                   bool compute_covariance);
    void compute_sqrt_inv_cov();

    void predict(const Frame *old_frame, Frame *new_frame);

    // [pw 2026-09-25 okvis2-preint] 把相邻两段的样本拼成一条(OKVIS2 的一条按时间排序的测量队列):
    // 保留 earlier 里时间严格早于 later 首样本的部分,再接 later。每帧的 data 两头各带一个越过帧时刻的
    // 原始样本(起点前一个、终点后一个,见 detail.cpp / feature_tracker.cpp),拼接处这两份拷贝在这里去重。
    static void prepend_data(std::vector<ImuData> &later,
                             const std::vector<ImuData> &earlier);
    static void append_data(std::vector<ImuData> &earlier,
                            const std::vector<ImuData> &later);

    matrix<3> cov_w; // continuous noise covariance
    matrix<3> cov_a;
    matrix<3> cov_bg; // continuous random walk noise covariance
    matrix<3> cov_ba;

    Delta delta;
    Jacobian jacobian;

    std::vector<ImuData> data;
};

} // namespace xrslam

#endif // XRSLAM_PREINTEGRATOR_H
