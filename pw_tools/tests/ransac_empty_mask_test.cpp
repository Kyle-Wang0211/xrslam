// [pw] 2026-08-24 回归测试:RANSAC 在「一个内点都找不到」时必须仍然填满
// inlier_mask,否则调用方拿到空 vector,用 data() 索引就是从地址 0 取字节。
//
// 真机现场:iPhone 14 Pro,Frame::track_keypoints 里
//     for (i = 0; i < status.size(); ++i) if (!mask[i]) status[i] = 0;
// 崩在 ldrb w11, [x11, x10],x11 == 0 (KERN_INVALID_ADDRESS at 0x0)。
//
// ransac.h 是纯头文件模板 ⇒ 本测试不链接 xrslam 任何库,秒级编译。
#include <array>
#include <cstdio>
#include <vector>
#include "xrslam/utility/ransac.h"

namespace {

struct AlwaysOutlierSolver {
    // 返回一个模型,让求解器有东西可评 —— 关键在评估器永远判为外点。
    std::vector<int> operator()(const std::array<double, 2> &,
                                const std::array<double, 2> &) const {
        return {0};
    }
};

struct AlwaysOutlierEvaluator {
    AlwaysOutlierEvaluator(const int &) {}
    // 误差恒为正无穷 ⇒ 每次迭代 current_inlier_count 都是 0
    // ⇒ `current_inlier_count > inlier_count` 恒假 ⇒ 上游从不给 inlier_mask 赋值。
    double operator()(const double &, const double &) const { return 1.0e18; }
};

int failures = 0;

void check(bool ok, const char *what) {
    std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++failures;
}

} // namespace

int main() {
    using R = xrslam::Ransac<2, int, AlwaysOutlierSolver, AlwaysOutlierEvaluator>;

    // ① 主路径:size >= ModelDoF,但零内点。这是崩溃的那条路。
    {
        const size_t N = 64;
        std::vector<double> a(N, 1.0), b(N, 2.0);
        R ransac(1.0 /*threshold*/, 0.99 /*confidence*/, 32 /*max_iter*/, 1 /*seed*/);
        ransac.solve(a, b);
        check(ransac.inlier_mask.size() == N,
              "零内点时 inlier_mask.size() == 输入点数(修复前是 0)");
        check(ransac.inlier_mask.data() != nullptr,
              "零内点时 inlier_mask.data() 非空(修复前是 nullptr ⇒ 索引即崩)");
        bool all_zero = true;
        for (char c : ransac.inlier_mask) if (c) all_zero = false;
        check(all_zero, "零内点时 mask 全 0(语义:找不到模型 == 没有内点)");
        // 复现崩溃现场的索引模式 —— 修复前这一行就是 SIGSEGV。
        size_t marked = 0;
        for (size_t i = 0; i < N; ++i) if (!ransac.inlier_mask[i]) ++marked;
        check(marked == N, "按 status.size() 索引 mask 不再越界");
    }

    // ② 早退路径:size < ModelDoF。上游本来就是对的,守住别回退。
    {
        const size_t N = 1;
        std::vector<double> a(N, 1.0), b(N, 2.0);
        R ransac(1.0, 0.99, 32, 1);
        ransac.solve(a, b);
        check(ransac.inlier_mask.size() == N,
              "size < ModelDoF 时 mask 仍被填满(上游原有行为,不许回退)");
    }

    // ③ size == 0:不能崩,mask 为空是正确的(调用方的循环也不会执行)。
    {
        std::vector<double> a, b;
        R ransac(1.0, 0.99, 32, 1);
        ransac.solve(a, b);
        check(ransac.inlier_mask.empty(), "空输入时 mask 为空且不崩");
    }

    std::printf(failures ? "\n  %d 项失败\n" : "\n  全部通过\n", failures);
    return failures ? 1 : 0;
}
