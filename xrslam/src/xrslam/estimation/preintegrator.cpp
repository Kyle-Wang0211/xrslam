#include <algorithm>
#include <xrslam/estimation/okvis2_imu_integration.h>
#include <xrslam/estimation/preintegrator.h>
#include <xrslam/geometry/lie_algebra.h>
#include <xrslam/map/frame.h>

// [pw 2026-09-25 okvis2-preint] 预积分离散方式换成 OKVIS2(BSD-3,版权声明见 okvis2_imu_integration.h)。
//
// 上游做法(已删):每段 [t_k, t_{k+1}] 用样本 k 的值(左端零阶保持),最后一个样本保持到帧时刻,
// 帧起点再插一个「上一帧末样本、时间戳改成上一帧时刻」的拷贝 ⇒ 100 Hz 下积出的姿态代表时间戳之前约 5 ms。
// 现在:OKVIS2 ImuError::redoPreintegration() 的循环逐行照抄(端点按时间线性插值、段内相邻两样本取平均),
// 均值、对 bg/ba 的雅可比、协方差传播都用它的离散式,然后换到 XRSLAM 的参数化。
//
// OKVIS2 → XRSLAM 对应(OKVIS2 名 → 本文件字段;依据是两边残差式逐项对照:
//   OKVIS2 ImuError.cpp:890-925 vs XRSLAM ceres/preintegration_factor.h Evaluate):
//   Delta_q_                      → delta.q        两边都是 q_i^{-1} q_j 的测量
//   acc_doubleintegral_           → delta.p        OKVIS2 error_p = C_S0_W(...) + acc_doubleintegral_ + ...
//   acc_integral_                 → delta.v        OKVIS2 error_v = C_S0_W(...) + acc_integral_ + ...
//   t1_ - t0_                     → delta.t
//   dp_db_g_                      → jacobian.dp_dbg
//   -C_doubleintegral_            → jacobian.dp_dba (OKVIS2 残差里是 -C_doubleintegral_ * Δb_a)
//   dv_db_g_                      → jacobian.dv_dbg
//   -C_integral_                  → jacobian.dv_dba (OKVIS2 残差里是 -C_integral_ * Δb_a)
//   dalpha_db_g_                  → jacobian.dq_dbg = -C_N^T · dalpha_db_g_
//       依据:OKVIS2 的偏置修正是左乘 Dq = Exp(-dalpha_db_g_·Δb_g)·Delta_q_(ImuError.cpp:894-895),
//       XRSLAM 是右乘 dq·Exp(dq_dbg·Δb_g)(preintegration_factor.h r_q);Exp(φ)·R = R·Exp(R^T φ) 恒等,
//       C_N = Delta_q_ 的旋转矩阵。
//   P_delta_(次序 p, α, v, bg, ba;α 左乘扰动、在 S0 系) → delta.cov(次序 q, p, v, bg, ba;q 右乘扰动)
//       换算 delta.cov = M · P_delta_ · M^T,M 把 α 换成 θ = C_N^T α 并重排行列。依据同上(伴随恒等式,
//       线性变量替换,不是近似)。XRSLAM 残差各分量与 OKVIS2 误差整体差一个负号,协方差不受影响。
//
// 搬不过去的地方与处理(除 okvis2_imu_integration.h 头部 (1)-(5) 外):
//   (a) 重新线性化策略:OKVIS2 在 |Δb_g| > 3e-4 且样本 < 50 时自动重做预积分(ImuError.cpp:856-857);
//       XRSLAM 由各调用点在固定时机用 frame_i 当前偏置重积分(滑窗每次 refine_window 都重积分关键帧段)。
//       这是求解器结构,不属于离散方式 ⇒ 保留 XRSLAM 的。
//   (b) 信息矩阵平方根:OKVIS2 用 PseudoInverse::symmSqrtU,XRSLAM 用 LLT(cov^{-1});两者都满足
//       S^T S = cov^{-1},残差二次型相同 ⇒ 保留 XRSLAM 的 compute_sqrt_inv_cov()。
//   (c) 起点时间:上游 integrate(t) 从 data.front().t 起积;这里起点显式传入(OKVIS2 的 t0_),
//       调用点全部改成传前一帧(状态 i)的 image->t。

namespace xrslam {

void PreIntegrator::reset() {
    delta.t = 0;
    delta.q.setIdentity();
    delta.p.setZero();
    delta.v.setZero();
    delta.cov.setZero();
    delta.sqrt_inv_cov.setZero();

    jacobian.dq_dbg.setZero();
    jacobian.dp_dbg.setZero();
    jacobian.dp_dba.setZero();
    jacobian.dv_dbg.setZero();
    jacobian.dv_dba.setZero();
}

bool PreIntegrator::integrate(double t0, double t1, const vector<3> &bg,
                              const vector<3> &ba, bool compute_jacobian,
                              bool compute_covariance) {
    if (data.size() == 0)
        return false;
    reset();
    okvis2::NoiseCov noise{cov_w, cov_a, cov_bg, cov_ba};
    okvis2::Preintegral pi;
    const int steps =
        okvis2::preintegrate(data.data(), data.size(), t0, t1, bg, ba, &noise,
                             compute_jacobian, compute_covariance, pi);
    if (steps < 0) {
        // OKVIS2:测量没覆盖到 t1 时 redoPreintegration 返回 -1、不积(ImuError.cpp:290-293)。
        // 每帧在 Detail::track_imu 派发时都带上了越过帧时刻的那个样本,正常数据流走不到这里。
        log_warning("[okvis2-preint] IMU data does not cover [%.6f, %.6f] (n=%zu)", t0,
                    t1, data.size());
        return false;
    }

    delta.t = t1 - t0;
    delta.q = pi.Delta_q;
    delta.p = pi.acc_doubleintegral;
    delta.v = pi.acc_integral;

    const matrix<3> C_N = pi.Delta_q.toRotationMatrix();
    if (compute_jacobian) {
        jacobian.dq_dbg = -C_N.transpose() * pi.dalpha_db_g;
        jacobian.dp_dbg = pi.dp_db_g;
        jacobian.dp_dba = -pi.C_doubleintegral;
        jacobian.dv_dbg = pi.dv_db_g;
        jacobian.dv_dba = -pi.C_integral;
    }

    if (compute_covariance) {
        // OKVIS2 (p, α, v, bg, ba) → XRSLAM (q, p, v, bg, ba),θ = C_N^T α
        matrix<15> M = matrix<15>::Zero();
        M.block<3, 3>(ES_Q, 3) = C_N.transpose();
        M.block<3, 3>(ES_P, 0).setIdentity();
        M.block<3, 3>(ES_V, 6).setIdentity();
        M.block<3, 3>(ES_BG, 9).setIdentity();
        M.block<3, 3>(ES_BA, 12).setIdentity();
        delta.cov = M * pi.P_delta * M.transpose();
        compute_sqrt_inv_cov();
    }
    return true;
}

void PreIntegrator::compute_sqrt_inv_cov() {
    delta.sqrt_inv_cov =
        Eigen::LLT<matrix<15, 15>>(delta.cov.inverse()).matrixL().transpose();
}

void PreIntegrator::predict(const Frame *old_frame, Frame *new_frame) {
    static const vector<3> gravity = {0, 0, -XRSLAM_GRAVITY_NOMINAL};
    new_frame->motion.bg = old_frame->motion.bg;
    new_frame->motion.ba = old_frame->motion.ba;
    new_frame->motion.v =
        old_frame->motion.v + gravity * delta.t + old_frame->pose.q * delta.v;
    new_frame->pose.p = old_frame->pose.p + 0.5 * gravity * delta.t * delta.t +
                        old_frame->motion.v * delta.t +
                        old_frame->pose.q * delta.p;
    new_frame->pose.q = old_frame->pose.q * delta.q;
}

void PreIntegrator::prepend_data(std::vector<ImuData> &later,
                                 const std::vector<ImuData> &earlier) {
    if (later.empty()) {
        later = earlier;
        return;
    }
    const double t_first = later.front().t;
    auto cut = std::lower_bound(
        earlier.begin(), earlier.end(), t_first,
        [](const ImuData &d, double t) { return d.t < t; });
    later.insert(later.begin(), earlier.begin(), cut);
}

void PreIntegrator::append_data(std::vector<ImuData> &earlier,
                                const std::vector<ImuData> &later) {
    if (later.empty())
        return;
    const double t_first = later.front().t;
    auto cut = std::lower_bound(
        earlier.begin(), earlier.end(), t_first,
        [](const ImuData &d, double t) { return d.t < t; });
    earlier.erase(cut, earlier.end());
    earlier.insert(earlier.end(), later.begin(), later.end());
}

} // namespace xrslam
