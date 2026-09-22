#include <xrslam/estimation/preintegrator.h>
#include <xrslam/geometry/lie_algebra.h>
#include <xrslam/map/frame.h>
#include <xrslam/utility/runtime_budget.h>

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

void PreIntegrator::increment(double dt, const ImuData &data,
                              const vector<3> &bg, const vector<3> &ba,
                              bool compute_jacobian, bool compute_covariance) {
    // [pw] 条目 08 —— dt **来自实测的相邻样本时间戳**(见下面 integrate()),
    //      不是 yaml 频率;yaml 给的 cov_w/cov_a 是**连续时间**噪声密度,
    //      下面 `cov_* * inv_dt` 这一步才是用实测 dt 做的离散化。
    //      原来这里只有一条 runtime_assert(dt >= 0),而 runtime_assert 在
    //      Release 下被展开成空(utility/debug.h:44)⇒ 出货构建里一个回退的
    //      时间戳会带着**负 dt** 一路进 A/B 矩阵,静默污染共分散且无任何痕迹。
    //      现在:计数 + 钳到 0(等价于忽略该样本的时间推进),不再静默。
    if (!(dt >= 0.0)) {
        auto &c = runtime::counters();
        unsigned long long n =
            c.nonmonotonic_imu_dt.fetch_add(1, std::memory_order_relaxed) + 1;
        if (runtime::should_log_at(n)) {
            log_warning("[pw][imu] 预积分收到负 dt=%.9fs,已钳到 0"
                        "(累计 %llu)。上游 IMU 时间戳非单调。",
                        dt, n);
        }
        dt = 0.0;
    } else if (dt == 0.0) {
        runtime::counters().zero_imu_dt.fetch_add(1,
                                                  std::memory_order_relaxed);
    }
    runtime_assert(dt >= 0, "dt cannot be negative.");

    vector<3> w = data.w - bg;
    vector<3> a = data.a - ba;

    if (compute_covariance) {
        matrix<9> A;
        A.setIdentity();
        A.block<3, 3>(ES_Q, ES_Q) = expmap(w * dt).conjugate().matrix();
        A.block<3, 3>(ES_V, ES_Q) = -dt * delta.q.matrix() * hat(a);
        A.block<3, 3>(ES_P, ES_Q) = -0.5 * dt * dt * delta.q.matrix() * hat(a);
        A.block<3, 3>(ES_P, ES_V) = dt * matrix<3>::Identity();

        matrix<9, 6> B;
        B.setZero();
        B.block<3, 3>(ES_Q, ES_BG - ES_BG) = dt * right_jacobian(w * dt);
        B.block<3, 3>(ES_V, ES_BA - ES_BG) = dt * delta.q.matrix();
        B.block<3, 3>(ES_P, ES_BA - ES_BG) = 0.5 * dt * dt * delta.q.matrix();

        matrix<6> white_noise_cov;
        double inv_dt = 1.0 / std::max(dt, 1.0e-7);
        white_noise_cov.setZero();
        white_noise_cov.block<3, 3>(ES_BG - ES_BG, ES_BG - ES_BG) =
            cov_w * inv_dt;
        white_noise_cov.block<3, 3>(ES_BA - ES_BG, ES_BA - ES_BG) =
            cov_a * inv_dt;

        delta.cov.block<9, 9>(ES_Q, ES_Q) =
            A * delta.cov.block<9, 9>(0, 0) * A.transpose() +
            B * white_noise_cov * B.transpose();
        delta.cov.block<3, 3>(ES_BG, ES_BG) += cov_bg * dt;
        delta.cov.block<3, 3>(ES_BA, ES_BA) += cov_ba * dt;
    }

    if (compute_jacobian) {
        jacobian.dp_dbg += dt * jacobian.dv_dbg - 0.5 * dt * dt *
                                                      delta.q.matrix() *
                                                      hat(a) * jacobian.dq_dbg;
        jacobian.dp_dba +=
            dt * jacobian.dv_dba - 0.5 * dt * dt * delta.q.matrix();
        jacobian.dv_dbg -= dt * delta.q.matrix() * hat(a) * jacobian.dq_dbg;
        jacobian.dv_dba -= dt * delta.q.matrix();
        jacobian.dq_dbg =
            expmap(w * dt).conjugate().matrix() * jacobian.dq_dbg -
            dt * right_jacobian(w * dt);
    }

    delta.t = delta.t + dt;
    delta.p = delta.p + dt * delta.v + 0.5 * dt * dt * (delta.q * a);
    delta.v = delta.v + dt * (delta.q * a);
    delta.q = (delta.q * expmap(w * dt)).normalized();
}

bool PreIntegrator::integrate(double t, const vector<3> &bg,
                              const vector<3> &ba, bool compute_jacobian,
                              bool compute_covariance) {
    if (data.size() == 0)
        return false;
    reset();
    // [pw] 条目 08 的确切证据就在这两行:每一步的 dt = 相邻两个 IMU 样本的
    //      **实测时间戳之差**,以及末段 `t - data.back().t`(t = 图像时间戳)。
    //      全树没有任何"采样频率"配置项(yaml 里 imu.* 只有 noise.cov_*),
    //      所以"dt 来自 yaml"这一条在本仓不成立。
    for (size_t i = 0; i + 1 < data.size(); ++i) {
        const ImuData &d = data[i];
        increment(data[i + 1].t - d.t, d, bg, ba, compute_jacobian,
                  compute_covariance);
    }
    increment(t - data.back().t, data.back(), bg, ba, compute_jacobian,
              compute_covariance);
    if (compute_covariance) {
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

} // namespace xrslam
