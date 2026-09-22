#ifndef XRSLAM_RANSAC_H
#define XRSLAM_RANSAC_H

#include <xrslam/common.h>
#include <xrslam/utility/random.h>

namespace xrslam {

template <size_t ModelDoF, typename ModelType, typename ModelSolver,
          typename ModelEvaluator>
struct Ransac {
    double threshold;
    double confidence;
    size_t max_iteration;
    int seed;

    ModelType model;
    size_t inlier_count;
    std::vector<char> inlier_mask;

    Ransac(double threshold, double confidence = 0.999,
           size_t max_iteration = 1000, int seed = 0)
        : threshold(threshold), confidence(confidence),
          max_iteration(max_iteration), seed(seed) {}

    template <typename... DataTypes>
    ModelType solve(const std::vector<DataTypes> &...data) {
        std::tuple<const std::vector<DataTypes> &...> tdata =
            std::make_tuple(std::cref(data)...);
        size_t size = std::get<0>(tdata).size();

        LotBox lotbox(size);
        lotbox.seed(seed);

        double K = log(std::max(1 - confidence, 1.0e-5));

        inlier_count = 0;

        // [pw] 2026-08-24 真机 SIGSEGV 修复(上游 bug)。
        //
        // 崩溃现场:iPhone 14 Pro,`Frame::track_keypoints` 里
        //     for (size_t i = 0; i < status.size(); ++i)
        //         if (!mask[i]) status[i] = 0;
        // 崩在 `ldrb w11, [x11, x10]`,x11 == 0 ⇒ mask.data() 是 nullptr
        // (EXC_BAD_ACCESS / KERN_INVALID_ADDRESS at 0x0,故障线程是
        //  FeatureTracker worker)。
        //
        // 根因就在下面那个主循环:`inlier_mask` **只在**
        //     if (current_inlier_count > inlier_count)
        // 里被赋值,而 inlier_count 初值是 0、比较是**严格大于**。
        // 于是「每一次迭代都找到 0 个内点」时,inlier_mask 一次都不会被赋值,
        // 保持默认构造的空 vector ⇒ 调用方 swap 到手的是空的 ⇒ 用
        // status.size() 去索引它就是从地址 0 取字节。
        //
        // 这不是理论风险:对应关系差到没有任何 5 点本质矩阵模型能解释哪怕
        // 一个点(运动模糊、场景突变、跟踪几乎全丢)时就会发生。
        //
        // 修法与上游自己在 `size < ModelDoF` 分支里的处理**保持一致** ——
        // 那条早退路径本来就把 mask 填成 (size, 0)。这里只是把同一个不变式
        // 提到函数开头,让它对所有出口都成立:
        //     **函数返回时,inlier_mask.size() 恒等于 size。**
        // 「找不到模型」因此等价于「零内点」,语义正确,且不改变任何原本能
        // 正常返回的调用的结果(有内点时下面的 swap 会覆盖掉这里的初值)。
        {
            std::vector<char> zero(size, 0);
            inlier_mask.swap(zero);
        }

        if (size < ModelDoF) {
            return model;
        }

        size_t iter_max = max_iteration;
        for (size_t iter = 0; iter < iter_max; ++iter) {
            std::tuple<std::array<DataTypes, ModelDoF>...> tsample;

            lotbox.refill_all();
            for (size_t si = 0; si < ModelDoF; ++si) {
                size_t sample_index = lotbox.draw_without_replacement();
                make_sample(tdata, tsample, sample_index, si);
            }

            std::vector<ModelType> models{apply(ModelSolver(), tsample)};
            for (const auto &current_model : models) {
                size_t current_inlier_count = 0;
                std::vector<char> current_inlier_mask(size, 0);
                ModelEvaluator eval(current_model);
                for (size_t i = 0; i < size; ++i) {
                    double error = eval(data[i]...);
                    if (error <= threshold) {
                        current_inlier_count++;
                        current_inlier_mask[i] = 1;
                    }
                }

                if (current_inlier_count > inlier_count) {
                    model = current_model;
                    inlier_count = current_inlier_count;
                    inlier_mask.swap(current_inlier_mask);

                    double inlier_ratio = inlier_count / (double)size;
                    double N = K / log(1 - pow(inlier_ratio, 5));
                    if (N < (double)iter_max) {
                        iter_max = (size_t)ceil(N);
                    }
                }
            }
        }

        return model;
    }

  private:
    template <class Data, class Sample, size_t... I>
    static void make_sample_impl(Data &&data, Sample &&sample, size_t idata,
                                 size_t isample, std::index_sequence<I...>) {
        [[maybe_unused]] auto ret = {((void *)&(
            std::get<I>(sample)[isample] = std::get<I>(data)[idata]))...};
    }

    template <class Data, class Sample>
    static void make_sample(Data &&data, Sample &&sample, size_t idata,
                            size_t isample) {
        make_sample_impl(
            std::forward<Data>(data), std::forward<Sample>(sample), idata,
            isample,
            std::make_index_sequence<std::tuple_size<
                typename std::remove_reference<Data>::type>::value>{});
    }
};

} // namespace xrslam

#endif // XRSLAM_RANSAC_H
