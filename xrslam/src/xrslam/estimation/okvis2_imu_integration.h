/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *  Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 *  Copyright (c) 2024, Smart Robotics Lab / Technical University of Munich
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab, ETH Zurich, Smart Robotics Lab,
 *     Imperial College London, Technical University of Munich, nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************************/

// [pw 2026-09-25 okvis2-preint] IMU 预积分的离散方式照抄 OKVIS2(BSD-3,上面是它的原版权声明,按条款保留)。
//
// 抄的来源(github.com/ethz-mrl/okvis2 @a2ea006,只读副本):
//   * okvis_ceres/src/ImuError.cpp  ImuError::redoPreintegration()  —— 循环体(端点按时间线性插值、段内梯形
//     = 相邻两样本取平均)、均值、对 bg/ba 的雅可比、协方差传播 F_delta 与噪声 K0..K3,逐行照抄;
//   * okvis_ceres/src/ImuError.cpp  ImuError::propagation()  —— 位姿外推的输出式(调用方 detail.cpp 用);
//   * okvis_kinematics/.../implementation/Transformation.hpp  rightJacobian()、operators.hpp crossMx()、
//     okvis_ceres/.../ode/ode.hpp sinc()  —— 原样搬来,保证数值与 OKVIS2 同一套函数。
//
// 搬不过去、做了处理的地方(逐条,依据写在旁边;汇总见 preintegrator.cpp 顶部):
//   (1) 时间类型:OKVIS2 用整数纳秒 okvis::Time,这里是 double 秒(XRSLAM 全程 double);`nexttime == t1`
//       的收尾判定在 double 下同样成立,因为 nexttime 是直接赋值成 t1 的(不是算出来的)。
//   (2) 饱和放大(g_max / a_max 超限时噪声 ×100):XRSLAM 配置里没有量程参数 ⇒ 不做(等价 g_max=a_max=∞)。
//   (3) dPdsigma_ 四份分解:OKVIS2 为了在线估计噪声 σ 才把 P 拆成四份再按 σ² 求和;XRSLAM 不估噪声,
//       按线性直接累加 P(= propagation() 里 covariance 分支的写法),数学上逐项相同。
//   (4) 噪声:OKVIS2 的 σ²·I 换成 XRSLAM 配置里的 3×3 连续噪声协方差(cov_w/cov_a/cov_bg/cov_ba,数值不改);
//       现行配置是各向同性对角阵,与 σ²·I 完全一致(OKVIS2 注释:各向同性所以旋转可以忽略)。
//   (5) 最后一个样本:OKVIS2 在 it+1==end 时仍读 (it+1)->measurement(越界),靠开头的覆盖检查保证走不到;
//       这里循环只到倒数第二个样本,覆盖检查照抄,行为相同且不越界。

#ifndef XRSLAM_OKVIS2_IMU_INTEGRATION_H
#define XRSLAM_OKVIS2_IMU_INTEGRATION_H

#include <cmath>
#include <xrslam/common.h>

namespace xrslam {
namespace okvis2 {

// okvis_ceres/include/okvis/ceres/ode/ode.hpp  sinc()
inline double sinc(double x) {
    if (fabs(x) > 1e-6) {
        return sin(x) / x;
    } else {
        static const double c_2 = 1.0 / 6.0;
        static const double c_4 = 1.0 / 120.0;
        static const double c_6 = 1.0 / 5040.0;
        const double x_2 = x * x;
        const double x_4 = x_2 * x_2;
        const double x_6 = x_2 * x_2 * x_2;
        return 1.0 - c_2 * x_2 + c_4 * x_4 - c_6 * x_6;
    }
}

// okvis_kinematics/include/okvis/kinematics/operators.hpp  crossMx()
inline Eigen::Matrix3d crossMx(const Eigen::Vector3d &v) {
    Eigen::Matrix3d C;
    C(0, 0) = 0.0;
    C(0, 1) = -v[2];
    C(0, 2) = v[1];
    C(1, 0) = v[2];
    C(1, 1) = 0.0;
    C(1, 2) = -v[0];
    C(2, 0) = -v[1];
    C(2, 1) = v[0];
    C(2, 2) = 0.0;
    return C;
}

// okvis_kinematics/include/okvis/kinematics/implementation/Transformation.hpp  rightJacobian()
inline Eigen::Matrix3d rightJacobian(const Eigen::Vector3d &PhiVec) {
    const double Phi = PhiVec.norm();
    Eigen::Matrix3d retMat = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d Phi_x = crossMx(PhiVec);
    const Eigen::Matrix3d Phi_x2 = Phi_x * Phi_x;
    if (Phi < 1.0e-4) {
        retMat += -0.5 * Phi_x + 1.0 / 6.0 * Phi_x2;
    } else {
        const double Phi2 = Phi * Phi;
        const double Phi3 = Phi2 * Phi;
        retMat += -(1.0 - cos(Phi)) / (Phi2)*Phi_x + (Phi - sin(Phi)) / Phi3 * Phi_x2;
    }
    return retMat;
}

// ImuError::redoPreintegration() 的全部中间量,名字与 OKVIS2 成员一一对应(去掉结尾下划线)。
// 误差状态次序是 OKVIS2 的:[δp(0:3), δα(3:6), δv(6:9), δb_g(9:12), δb_a(12:15)],
// δα 是左乘扰动、表达在起点 S0 系(Dq = Exp(δα)·Delta_q,见 ImuError.cpp:894-895 / 923)。
struct Preintegral {
    Eigen::Quaterniond Delta_q = Eigen::Quaterniond(1, 0, 0, 0);
    Eigen::Matrix3d C_integral = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d C_doubleintegral = Eigen::Matrix3d::Zero();
    Eigen::Vector3d acc_integral = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_doubleintegral = Eigen::Vector3d::Zero();
    Eigen::Matrix3d cross = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d dalpha_db_g = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d dv_db_g = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d dp_db_g = Eigen::Matrix3d::Zero();
    Eigen::Matrix<double, 15, 15> P_delta = Eigen::Matrix<double, 15, 15>::Zero();
    double Delta_t = 0.0; // propagation() 里的 Delta_t(各段 dt 之和)
};

struct NoiseCov { // 连续噪声协方差(XRSLAM 配置原值),对应 OKVIS2 的 σ_g_c²、σ_a_c²、σ_gw_c²、σ_aw_c²
    Eigen::Matrix3d w, a, bg, ba;
};

// 照抄 ImuError::redoPreintegration() 的循环(ImuError.cpp:319-466)。
// data[0..n) 必须按时间递增,覆盖 [t0, t1](data[0].t <= t0 由调用方保证,否则按 OKVIS2 原式外插;
// data[n-1].t >= t1 不满足时与 OKVIS2 一样返回 -1、什么都不积)。
// with_jacobian/with_covariance 为 false 时跳过对应累加(均值与之无关);协方差要用到雅可比中间量,
// 所以 with_covariance 时雅可比中间量照样算。返回积分的段数(OKVIS2 的 i)。
inline int preintegrate(const ImuData *data, size_t n, double t0, double t1,
                        const Eigen::Vector3d &b_g, const Eigen::Vector3d &b_a,
                        const NoiseCov *noise, bool with_jacobian, bool with_covariance,
                        Preintegral &out) {
    out = Preintegral();
    double time = t0;
    const double end = t1;
    if (n < 2)
        return -1;
    if (!(data[n - 1].t >= end))
        return -1; // nothing to do...(ImuError.cpp:290-293)

    const bool jac = with_jacobian || with_covariance;
    bool hasStarted = false;
    int i = 0;
    for (size_t k = 0; k + 1 < n; ++k) {
        const ImuData &it = data[k];
        const ImuData &it1 = data[k + 1];
        Eigen::Vector3d omega_S_0 = it.w;
        Eigen::Vector3d acc_S_0 = it.a;
        Eigen::Vector3d omega_S_1 = it1.w;
        Eigen::Vector3d acc_S_1 = it1.a;

        // time delta
        double nexttime = it1.t;
        double dt = nexttime - time;

        if (end < nexttime) {
            double interval = nexttime - it.t;
            nexttime = t1;
            dt = nexttime - time;
            const double r = dt / interval;
            omega_S_1 = ((1.0 - r) * omega_S_0 + r * omega_S_1).eval();
            acc_S_1 = ((1.0 - r) * acc_S_0 + r * acc_S_1).eval();
        }

        if (dt <= 0.0) {
            continue;
        }
        out.Delta_t += dt;

        if (!hasStarted) {
            hasStarted = true;
            const double r = dt / (nexttime - it.t);
            omega_S_0 = (r * omega_S_0 + (1.0 - r) * omega_S_1).eval();
            acc_S_0 = (r * acc_S_0 + (1.0 - r) * acc_S_1).eval();
        }

        // actual propagation
        // orientation:
        Eigen::Quaterniond dq;
        const Eigen::Vector3d omega_S_true = (0.5 * (omega_S_0 + omega_S_1) - b_g);
        const double theta_half = omega_S_true.norm() * 0.5 * dt;
        const double sinc_theta_half = sinc(theta_half);
        const double cos_theta_half = cos(theta_half);
        dq.vec() = sinc_theta_half * omega_S_true * 0.5 * dt;
        dq.w() = cos_theta_half;
        Eigen::Quaterniond Delta_q_1 = out.Delta_q * dq;
        // rotation matrix integral:
        const Eigen::Matrix3d C = out.Delta_q.toRotationMatrix();
        const Eigen::Matrix3d C_1 = Delta_q_1.toRotationMatrix();
        const Eigen::Vector3d acc_S_true = (0.5 * (acc_S_0 + acc_S_1) - b_a);
        const Eigen::Matrix3d C_integral_1 = out.C_integral + 0.5 * (C + C_1) * dt;
        const Eigen::Vector3d acc_integral_1 =
            out.acc_integral + 0.5 * (C + C_1) * acc_S_true * dt;
        // rotation matrix double integral:
        out.C_doubleintegral += out.C_integral * dt + 0.25 * (C + C_1) * dt * dt;
        out.acc_doubleintegral +=
            out.acc_integral * dt + 0.25 * (C + C_1) * acc_S_true * dt * dt;

        Eigen::Matrix3d cross_1 = out.cross;
        Eigen::Matrix3d dv_db_g_1 = out.dv_db_g;
        if (jac) {
            // Jacobian parts
            out.dalpha_db_g += C_1 * rightJacobian(omega_S_true * dt) * dt;
            cross_1 = dq.inverse().toRotationMatrix() * out.cross +
                      rightJacobian(omega_S_true * dt) * dt;
            const Eigen::Matrix3d acc_S_x = crossMx(acc_S_true);
            dv_db_g_1 = out.dv_db_g +
                        0.5 * dt * (C * acc_S_x * out.cross + C_1 * acc_S_x * cross_1);
            out.dp_db_g += dt * out.dv_db_g +
                           0.25 * dt * dt * (C * acc_S_x * out.cross + C_1 * acc_S_x * cross_1);

            if (with_covariance) {
                // covariance propagation
                Eigen::Matrix<double, 15, 15> F_delta = Eigen::Matrix<double, 15, 15>::Identity();
                // transform
                F_delta.block<3, 3>(0, 3) = -crossMx(out.acc_integral * dt +
                                                     0.25 * (C + C_1) * acc_S_true * dt * dt);
                F_delta.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * dt;
                F_delta.block<3, 3>(0, 9) =
                    dt * out.dv_db_g +
                    0.25 * dt * dt * (C * acc_S_x * out.cross + C_1 * acc_S_x * cross_1);
                F_delta.block<3, 3>(0, 12) = -out.C_integral * dt + 0.25 * (C + C_1) * dt * dt;
                F_delta.block<3, 3>(3, 9) = -dt * C_1;
                F_delta.block<3, 3>(6, 3) = -crossMx(0.5 * (C + C_1) * acc_S_true * dt);
                F_delta.block<3, 3>(6, 9) =
                    0.5 * dt * (C * acc_S_x * out.cross + C_1 * acc_S_x * cross_1);
                F_delta.block<3, 3>(6, 12) = -0.5 * (C + C_1) * dt;

                // Q = K * sigma_sq(ImuError.cpp:433-446,K0..K3 各自乘 σ² 后求和,见文件头 (3)(4))
                Eigen::Matrix<double, 15, 15> Q = Eigen::Matrix<double, 15, 15>::Zero();
                Q.block<3, 3>(3, 3) = dt * noise->w;                   // K0 · σ_g_c²
                Q.block<3, 3>(0, 0) = 0.5 * dt * dt * dt * noise->a;   // K1 · σ_a_c²
                Q.block<3, 3>(6, 6) = dt * noise->a;                   // K1 · σ_a_c²
                Q.block<3, 3>(9, 9) = dt * noise->bg;                  // K2 · σ_gw_c²
                Q.block<3, 3>(12, 12) = dt * noise->ba;                // K3 · σ_aw_c²
                out.P_delta = F_delta * out.P_delta * F_delta.transpose() + Q;
            }
        }

        // memory shift
        out.Delta_q = Delta_q_1;
        out.C_integral = C_integral_1;
        out.acc_integral = acc_integral_1;
        out.cross = cross_1;
        out.dv_db_g = dv_db_g_1;
        time = nexttime;

        ++i;

        if (nexttime == t1)
            break;
    }

    if (with_covariance) {
        // enforce symmetric(ImuError.cpp:473-475)
        out.P_delta = 0.5 * out.P_delta + 0.5 * out.P_delta.transpose().eval();
    }
    return i;
}

} // namespace okvis2
} // namespace xrslam

#endif // XRSLAM_OKVIS2_IMU_INTEGRATION_H
