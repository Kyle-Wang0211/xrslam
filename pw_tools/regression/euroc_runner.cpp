// euroc_runner.cpp — 无 GUI 的 EuRoC 离线回放器(4beb1a9 谱系 C API 版)。
//
// 来历:主树 pw/vio 的 pw_tools/regression/euroc_runner.cpp(那份用的是主树自己
// 加的 XRSLAMPushSensorDataChecked/XRSLAMGetHealth/XRSLAMTryGetLatestPose 与
// XRSLAMImage.width/height,本树 8a1cc12 里都不存在),这里按本树上游 C API 重写:
//   XRSLAMCreate(文件路径) → PushSensorData → RunOneFrame → GetResult(BODY_POSE)。
// 为什么不用上游 xrslam-pc/player:它链 liteviz(GUI),每帧等窗口(main.cpp:151-153),
// 无头必然卡死(记忆 reference_xrslam_pc_build_recipe_20260921)。
//
// [2026-09-22 逐帧内参] --intrinsics-csv <t_ns,fx,fy,cx,cy>:
//   每帧按时间戳(最近邻,容差 1 ms)配一行 K,塞进 XRSLAMImageExtension 尾部、
//   ext_size=sizeof(XRSLAMImageExtension) 交给引擎(C 臂)。不带此参数 = ext 为空 = A 臂
//   (引擎走 config 常量,与改前逐位相同)。配对规则逐字抄 pwvi_to_euroc.py --exposure-half:
//   按时间戳不按下标、1 ms 容差、配对率 < 99% 拒绝运行。
//   🔑 reader 交出的 t 已经加了 yaml 的 cam0.time_offset(euroc_dataset_reader.cpp:16-19),
//   CSV 里的 t 是转换器写的原始(曝光中点)时间戳 ⇒ 配对键 = t_reader − time_offset。
//
// 输出 TUM(timestamp tx ty tz qx qy qz qw),body 位姿(与主树 runner 的 GetBodyPose 同义),
// 未初始化时 GetResult 给零四元数 ⇒ 跳过;时间戳不严格前进 ⇒ 跳过(同主树 TryGetLatestPose 闸)。
//
//   pw_euroc_runner <slam.yaml> <device.yaml> euroc://<dir> <out.tum> [--intrinsics-csv k.csv]

#include "dataset_reader.h"
#include <XRSLAM.h>
#include <xrslam/extra/yaml_config.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace {

struct PoseRow {
    double t;
    double q[4]; // x y z w
    double p[3];
};

struct KRow {
    double t; // 秒
    double k[4];
};

std::vector<KRow> load_k_csv(const std::string &path) {
    std::ifstream f(path);
    if (!f) {
        fprintf(stderr, "cannot open intrinsics csv: %s\n", path.c_str());
        exit(EXIT_FAILURE);
    }
    std::vector<KRow> rows;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#')
            continue;
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream ss(line);
        long long t_ns;
        KRow r;
        if (!(ss >> t_ns >> r.k[0] >> r.k[1] >> r.k[2] >> r.k[3]))
            continue;
        r.t = double(t_ns) * 1e-9;
        rows.push_back(r);
    }
    std::sort(rows.begin(), rows.end(), [](const KRow &a, const KRow &b) { return a.t < b.t; });
    return rows;
}

// 最近邻配对,容差 1 ms(抄 pwvi_to_euroc.py --exposure-half 的规则)
const KRow *match_k(const std::vector<KRow> &rows, double t) {
    if (rows.empty())
        return nullptr;
    auto it = std::lower_bound(rows.begin(), rows.end(), t,
                               [](const KRow &r, double v) { return r.t < v; });
    const KRow *best = nullptr;
    double best_d = 1e-3;
    for (auto j : {it == rows.begin() ? rows.end() : it - 1, it}) {
        if (j == rows.end())
            continue;
        double d = std::fabs(j->t - t);
        if (d <= best_d) {
            best_d = d;
            best = &*j;
        }
    }
    return best;
}

} // namespace

int main(int argc, char **argv) {
    if (argc < 5) {
        fprintf(stderr,
                "usage: %s <slam_config.yaml> <device_config.yaml> euroc://<dir> <out.tum> "
                "[--intrinsics-csv <t_ns,fx,fy,cx,cy csv>]\n",
                argv[0]);
        return EXIT_FAILURE;
    }
    const std::string slam_cfg_path = argv[1];
    const std::string dev_cfg_path = argv[2];
    const std::string data_path = argv[3];
    const std::string out_tum = argv[4];
    std::string k_csv;
    for (int i = 5; i + 1 < argc; ++i) {
        if (std::string(argv[i]) == "--intrinsics-csv")
            k_csv = argv[i + 1];
    }

    // 本树 XRSLAM_IOS=OFF ⇒ 两个参数是 yaml **文件路径**(yaml_config.cpp:133-148)。
    void *yaml_config = nullptr;
    if (XRSLAMCreate(slam_cfg_path.c_str(), dev_cfg_path.c_str(), "", "pw-euroc-runner",
                     &yaml_config) != 1) {
        fprintf(stderr, "XRSLAMCreate failed\n");
        return EXIT_FAILURE;
    }
    const double time_offset =
        static_cast<xrslam::extra::YamlConfig *>(yaml_config)->camera_time_offset();

    std::vector<KRow> krows;
    if (!k_csv.empty()) {
        krows = load_k_csv(k_csv);
        fprintf(stderr, "[C 臂] 逐帧内参 %zu 行 <- %s;配对键 = t_reader - time_offset(%.4f s)\n",
                krows.size(), k_csv.c_str(), time_offset);
    } else {
        fprintf(stderr, "[A 臂] 无 --intrinsics-csv,ext=nullptr,引擎走 config 常量 K\n");
    }

    // async=false:同步读,回放确定性(不引入读线程的调度差异)。
    auto reader = DatasetReader::create_reader(data_path, yaml_config, /*async=*/false);
    if (!reader) {
        fprintf(stderr, "cannot open dataset: %s\n", data_path.c_str());
        XRSLAMDestroy();
        return EXIT_FAILURE;
    }

    std::vector<PoseRow> traj;
    traj.reserve(4096);
    long n_img = 0, n_gyro = 0, n_acc = 0, n_pose = 0, n_k_matched = 0, n_k_unmatched = 0;
    double last_pose_t = -1.0;
    double fx_min = 1e300, fx_max = -1e300, fx_sum = 0.0;
    const auto t_start = std::chrono::steady_clock::now();

    DatasetReader::NextDataType type;
    while ((type = reader->next()) != DatasetReader::END) {
        switch (type) {
        case DatasetReader::AGAIN:
            continue;
        case DatasetReader::GYROSCOPE: {
            auto [t, g] = reader->read_gyroscope();
            (void)t;
            XRSLAMPushSensorData(XRSLAM_SENSOR_GYROSCOPE, &g);
            ++n_gyro;
        } break;
        case DatasetReader::ACCELEROMETER: {
            auto [t, a] = reader->read_accelerometer();
            (void)t;
            XRSLAMPushSensorData(XRSLAM_SENSOR_ACCELERATION, &a);
            ++n_acc;
        } break;
        case DatasetReader::CAMERA: {
            auto [t, mat] = reader->read_image();
            if (mat.empty())
                break;
            XRSLAMImage img{}; // 零初始化 ⇒ ext_size=0, ext=nullptr(= 出货传输层的形态)
            img.data = mat.data;
            img.timeStamp = t;
            img.stride = static_cast<int>(mat.step[0]);
            img.camera_id = 0;
            img.channel = mat.channels();

            XRSLAMImageExtension ext{};
#ifdef XRSLAM_IMAGE_EXTENSION_LEGACY_SIZE   // 只有逐帧内参版的 XRSLAM.h 才定义;旧头 ⇒ 本 runner 只有 A 臂
            if (!krows.empty()) {
                if (const KRow *kr = match_k(krows, t - time_offset)) {
                    for (int i = 0; i < 4; ++i)
                        ext.intrinsics_fxfycxcy[i] = kr->k[i];
                    ext.has_intrinsics = 1;
                    img.ext = &ext;
                    img.ext_size = static_cast<unsigned int>(sizeof(ext)); // 调用者声明自己的 sizeof
                    ++n_k_matched;
                    fx_min = std::min(fx_min, kr->k[0]);
                    fx_max = std::max(fx_max, kr->k[0]);
                    fx_sum += kr->k[0];
                } else {
                    ++n_k_unmatched; // 配不上的帧如实计数,ext 留空 ⇒ 该帧走 config 常量
                }
            }
#else
            (void)ext;
            if (!krows.empty()) {
                fprintf(stderr, "🔴 本 runner 按旧 XRSLAM.h 编译(无 ext_size),不能跑 C 臂\n");
                return EXIT_FAILURE;
            }
#endif
            XRSLAMPushSensorData(XRSLAM_SENSOR_CAMERA, &img);
            ++n_img;
            XRSLAMRunOneFrame();

            XRSLAMPose pose{};
            XRSLAMGetResult(XRSLAM_RESULT_BODY_POSE, &pose);
            const double qn = std::sqrt(pose.quaternion[0] * pose.quaternion[0] +
                                        pose.quaternion[1] * pose.quaternion[1] +
                                        pose.quaternion[2] * pose.quaternion[2] +
                                        pose.quaternion[3] * pose.quaternion[3]);
            if (qn < 0.5 || !(pose.timestamp > last_pose_t))
                break; // 未初始化(零四元数)或时间戳不前进:不记
            last_pose_t = pose.timestamp;
            PoseRow r;
            r.t = pose.timestamp;
            for (int i = 0; i < 4; ++i)
                r.q[i] = pose.quaternion[i];
            for (int i = 0; i < 3; ++i)
                r.p[i] = pose.translation[i];
            traj.push_back(r);
            ++n_pose;
        } break;
        default:
            break;
        }
    }
    const double wall_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();

    XRSLAMIntrinsics reported{};
    XRSLAMGetResult(XRSLAM_INFO_INTRINSICS, &reported);
    XRSLAMDestroy();

    if (!krows.empty()) {
        const long total = n_k_matched + n_k_unmatched;
        if (total > 0 && n_k_unmatched > 0.01 * total) {
            fprintf(stderr, "🔴 逐帧内参配对率 %.2f%% < 99%%,拒绝(同 --exposure-half 的规则)\n",
                    100.0 * n_k_matched / total);
            return EXIT_FAILURE;
        }
    }

    std::ofstream out(out_tum);
    if (!out) {
        fprintf(stderr, "cannot write %s\n", out_tum.c_str());
        return EXIT_FAILURE;
    }
    out.setf(std::ios::fixed);
    for (const PoseRow &r : traj) {
        out.precision(9);
        out << r.t << ' ';
        out.precision(7);
        out << r.p[0] << ' ' << r.p[1] << ' ' << r.p[2] << ' ' << r.q[0] << ' ' << r.q[1] << ' '
            << r.q[2] << ' ' << r.q[3] << '\n';
    }
    out.close();

    fprintf(stderr,
            "\n=== pw euroc runner (4beb1a9 C API) ===\n"
            "  images      %ld\n"
            "  gyro/acc    %ld / %ld\n"
            "  poses       %ld  -> %s\n"
            "  K matched   %ld  unmatched %ld\n"
            "  fx(t)       min %.3f  max %.3f  mean %.3f  (极差 %.3f px = %.2f%%)\n"
            "  GetInfoIntrinsics 最后报出 fx %.3f fy %.3f cx %.3f cy %.3f\n"
            "  wall        %.1f s\n",
            n_img, n_gyro, n_acc, n_pose, out_tum.c_str(), n_k_matched, n_k_unmatched,
            n_k_matched ? fx_min : 0.0, n_k_matched ? fx_max : 0.0,
            n_k_matched ? fx_sum / n_k_matched : 0.0, n_k_matched ? fx_max - fx_min : 0.0,
            n_k_matched ? 100.0 * (fx_max - fx_min) / (fx_sum / n_k_matched) : 0.0,
            reported.fx, reported.fy, reported.cx, reported.cy, wall_s);

    return traj.empty() ? EXIT_FAILURE : EXIT_SUCCESS; // 轨迹为空 = 失败
}
