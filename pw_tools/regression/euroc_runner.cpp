// euroc_runner.cpp — 无 GUI 的 EuRoC 离线回归器。
//
// 为什么要它,而不是用上游的 xrslam-pc-player:
//   ① player 链着 depends::liteviz(imgui/GLFW/glad),那条链我们已经整条关掉
//      —— 它带零许可声明的 vendored glad,而且本身就编不过(imgui.cmake 没链 glfw)。
//   ② 回归工具**不该**依赖 GUI。CI 上没有窗口系统。
//   ③ 我们改了 29 个文件(拆宏、改 C API、关 ACCELERATESPARSE、-ffp-contract=off),
//      需要一个能证明「这些改动没有把算法改坏」的判据 —— 而 RD-VIO 的 EuRoC
//      ATE 是**公开发表**的(论文表:MH-01 0.109 … 平均 0.147 m),
//      能对上就说明我们的 fork 还是原来那个算法。
//
// 输出 TUM 格式轨迹(timestamp tx ty tz qx qy qz qw),直接喂 evo:
//   evo_ape tum groundtruth.tum ours.tum -a -s --plot
//
// ⚠️ 真值需要从 EuRoC 的 mav0/state_groundtruth_estimate0/data.csv 转 TUM。
//    转换脚本见同目录 euroc_gt_to_tum.py。

#include "dataset_reader.h"
#include <XRSLAM.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace {

std::string slurp(const std::string &path) {
    std::ifstream f(path);
    if (!f) {
        fprintf(stderr, "cannot open config: %s\n", path.c_str());
        exit(EXIT_FAILURE);
    }
    return std::string((std::istreambuf_iterator<char>(f)),
                       std::istreambuf_iterator<char>());
}

struct PoseRow {
    double t;
    double q[4];  // x y z w
    double p[3];
};

} // namespace

int main(int argc, char **argv) {
    if (argc < 5) {
        fprintf(stderr,
                "usage: %s <slam_config.yaml> <device_config.yaml> "
                "<euroc_seq_dir> <out.tum> [--csv <stats.csv>]\n",
                argv[0]);
        return EXIT_FAILURE;
    }
    const std::string slam_cfg_path = argv[1];
    const std::string dev_cfg_path = argv[2];
    const std::string data_path = argv[3];
    const std::string out_tum = argv[4];
    std::string stats_csv;
    // [pw] 每 N 帧才调一次 RunOneFrame。复刻 ARCore 的做法(他们的 VIO 只跑约
    //   10Hz,靠 IMU 前向积分补足)。⚠️ 这里降的是**求解节奏**,PushImage 仍然
    //   每帧都调 —— 帧不丢,只是不立刻触发求解。无损铁律的边界就在这。
    int run_every = 1;
    // [pw] 2026-08-23 --downscale 2:与 iOS 端 downsample2x() **逐位相同**的
    //   2x2 盒式平均。存在的唯一理由是回答一个具体问题:
    //   我们把手机端分辨率提到 960x720(上游 iPhone 配置 640x480 的 1.5 倍),
    //   而 min_parallax=10 / min_keypoint_distance=25 逐字沿用上游没有改。
    //   上游作者只在 640-752px(1.175x)区间验证过这套参数。
    //   1.5x 是外推 —— 需要证据,不是需要我的判断。
    //   EuRoC 只能往下降,所以测的是同一件事的另一侧:
    //   参数不动、分辨率变 2x 时,ATE 是不是塌了。
    int downscale = 1;
    for (int i = 5; i + 1 < argc; ++i) {
        if (std::string(argv[i]) == "--csv") stats_csv = argv[i + 1];
        if (std::string(argv[i]) == "--run-every") run_every = atoi(argv[i + 1]);
        if (std::string(argv[i]) == "--downscale") downscale = atoi(argv[i + 1]);
    }

    // 我们的构建打开了 XRSLAM_CONFIG_FROM_STRING ⇒ 两个参数是 **YAML 正文**
    // 而不是文件路径。这与 iOS 侧同一口径 —— 那正是把它从 XRSLAM_IOS 里
    // 拆出来的理由:同一个 C ABI 不该在两端语义不同。
    const std::string slam_cfg = slurp(slam_cfg_path);
    const std::string dev_cfg = slurp(dev_cfg_path);

    void *yaml_config = nullptr;
    // ⚠️ 1 = 成功,与其余 API 的 0=OK 相反。
    if (XRSLAMCreate(slam_cfg.c_str(), dev_cfg.c_str(), "", "pw-euroc-runner",
                     &yaml_config) != 1) {
        fprintf(stderr, "XRSLAMCreate failed\n");
        return EXIT_FAILURE;
    }

    auto reader = DatasetReader::create_reader(data_path, yaml_config);
    if (!reader) {
        fprintf(stderr, "cannot open dataset: %s\n", data_path.c_str());
        XRSLAMDestroy();
        return EXIT_FAILURE;
    }

    std::vector<PoseRow> traj;
    traj.reserve(4096);

    long n_img = 0, n_gyro = 0, n_acc = 0, n_pose = 0;
    long n_img_rejected = 0;
    double solve_ms_sum = 0.0, solve_ms_max = 0.0;
    double read_ms_sum = 0.0, push_ms_sum = 0.0, core_ms_sum = 0.0;
    double core_ms_max = 0.0;
    double imu_ms_sum = 0.0, imu_ms_max = 0.0;
    // 这几个统计存在的意义:**静默失效必须可见**。
    // imu_zero_frames > 0 就意味着有帧的求解器里一个 IMU 因子都没有 ——
    // 系统在那些帧上已经退化成纯单目,而它不崩、不报错、状态照常 TRACKING。
    long imu_zero_frames = 0, degenerate_frames = 0;

    const auto t_start = std::chrono::steady_clock::now();
    DatasetReader::NextDataType type;
    while ((type = reader->next()) != DatasetReader::END) {
        switch (type) {
        case DatasetReader::AGAIN:
            continue;

        case DatasetReader::GYROSCOPE: {
            auto [t, g] = reader->read_gyroscope();
            (void)t;
            { const auto q0 = std::chrono::steady_clock::now();
              XRSLAMPushSensorDataChecked(XRSLAM_SENSOR_GYROSCOPE, &g);
              const double q = std::chrono::duration<double,std::milli>(
                  std::chrono::steady_clock::now()-q0).count();
              imu_ms_sum += q; if (q > imu_ms_max) imu_ms_max = q; }
            ++n_gyro;
        } break;

        case DatasetReader::ACCELEROMETER: {
            auto [t, a] = reader->read_accelerometer();
            (void)t;
            { const auto q0 = std::chrono::steady_clock::now();
              XRSLAMPushSensorDataChecked(XRSLAM_SENSOR_ACCELERATION, &a);
              const double q = std::chrono::duration<double,std::milli>(
                  std::chrono::steady_clock::now()-q0).count();
              imu_ms_sum += q; if (q > imu_ms_max) imu_ms_max = q; }
            ++n_acc;
        } break;

        case DatasetReader::CAMERA: {
            const auto tr0 = std::chrono::steady_clock::now();
            auto [t, mat] = reader->read_image();
            read_ms_sum += std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - tr0).count();
            if (mat.empty()) break;
            // 与 ios/Runner/PwVioSlamFeeder.swift 的 downsample2x() 同一算法:
            // 2x2 整数和 + 四舍五入 (sum+2)/4。不用 cv::resize —— INTER_AREA
            // 在 2x 整数比例下虽然数学等价,但取整规则未必一致,而这里要的正是
            // "端上跑的和这里量的是同一个像素"。
            if (downscale == 2 && mat.cols % 2 == 0 && mat.rows % 2 == 0) {
                cv::Mat small(mat.rows / 2, mat.cols / 2, CV_8UC1);
                for (int y = 0; y < small.rows; ++y) {
                    const unsigned char *r0 = mat.ptr<unsigned char>(y * 2);
                    const unsigned char *r1 = mat.ptr<unsigned char>(y * 2 + 1);
                    unsigned char *o = small.ptr<unsigned char>(y);
                    for (int x = 0; x < small.cols; ++x) {
                        const int i = x * 2;
                        o[x] = (unsigned char)((int(r0[i]) + int(r0[i + 1]) +
                                                int(r1[i]) + int(r1[i + 1]) + 2) / 4);
                    }
                }
                mat = small;
            }
            XRSLAMImage img{};
            img.data = mat.data;
            img.timeStamp = t;
            img.stride = static_cast<int>(mat.step[0]);
            img.camera_id = 0;
            img.channel = mat.channels();
            // 我们给上游补的字段。原版分辨率取自 yaml,喂进来的图比 yaml 矮
            // 就是静默越界读(ASAN 实测越界 76800 字节)。
            img.width = mat.cols;
            img.height = mat.rows;
            img.ext = nullptr;

            const auto tp0 = std::chrono::steady_clock::now();
            const int rc =
                XRSLAMPushSensorDataChecked(XRSLAM_SENSOR_CAMERA, &img);
            push_ms_sum += std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - tp0).count();
            if (rc != XRSLAM_OK) { ++n_img_rejected; break; }
            ++n_img;

            if ((n_img - 1) % run_every != 0) break;   // 降频:不调求解
            const auto t0 = std::chrono::steady_clock::now();
            XRSLAMRunOneFrame();
            const double ms =
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - t0)
                    .count();
            solve_ms_sum += ms;
            if (ms > solve_ms_max) solve_ms_max = ms;

            XRSLAMHealth h{};
            if (XRSLAMGetHealth(&h) == XRSLAM_OK) {
                if (h.imu_samples_last_frame == 0) ++imu_zero_frames;
                core_ms_sum += h.last_frame_ms;   // 核内自报的耗时
                if (h.last_frame_ms > core_ms_max) core_ms_max = h.last_frame_ms;
                if (h.latest_pose_degenerate) ++degenerate_frames;
            }

            double pose7[7] = {0};
            double pts = 0;
            if (XRSLAMTryGetLatestPose(pose7, &pts) == XRSLAM_OK) {
                PoseRow r;
                r.t = pts;
                r.q[0] = pose7[0]; r.q[1] = pose7[1];
                r.q[2] = pose7[2]; r.q[3] = pose7[3];
                r.p[0] = pose7[4]; r.p[1] = pose7[5]; r.p[2] = pose7[6];
                traj.push_back(r);
                ++n_pose;
            }
        } break;

        default:
            break;
        }
    }
    const double wall_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start)
            .count();

    XRSLAMDestroy();

    // TUM 格式:timestamp tx ty tz qx qy qz qw
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
        out << r.p[0] << ' ' << r.p[1] << ' ' << r.p[2] << ' '
            << r.q[0] << ' ' << r.q[1] << ' ' << r.q[2] << ' ' << r.q[3]
            << '\n';
    }
    out.close();

    fprintf(stderr,
            "\n=== pw euroc runner ===\n"
            "  images      %ld (rejected %ld)\n"
            "  gyro/acc    %ld / %ld\n"
            "  poses       %ld  -> %s\n"
            "  IMU push    mean %.4f  max %.3f  总计 %.1f s  <- 🔑 真正的成本在这\n"
            "  read  ms    mean %.3f\n"
            "  push  ms    mean %.3f\n"
            "  run   ms    mean %.3f  max %.3f   <- 我们量的 RunOneFrame\n"
            "  core  ms    mean %.3f  max %.3f   <- 核内自报 last_frame_ms\n"
            "  run_every   %d   downscale %d\n"
            "  wall        %.1f s\n"
            "  imu_zero_frames    %ld   <- >0 表示那些帧退化成了纯单目\n"
            "  degenerate_frames  %ld\n",
            n_img, n_img_rejected, n_gyro, n_acc, n_pose, out_tum.c_str(),
            (n_gyro+n_acc) ? imu_ms_sum/(n_gyro+n_acc) : 0.0, imu_ms_max, imu_ms_sum/1000.0,
            n_img ? read_ms_sum / n_img : 0.0,
            n_img ? push_ms_sum / n_img : 0.0,
            n_img ? solve_ms_sum / n_img : 0.0, solve_ms_max,
            n_img ? core_ms_sum / n_img : 0.0, core_ms_max, run_every, downscale, wall_s,
            imu_zero_frames, degenerate_frames);

    if (!stats_csv.empty()) {
        std::ofstream s(stats_csv);
        s << "images,images_rejected,gyro,acc,poses,solve_ms_mean,solve_ms_max,"
             "wall_s,imu_zero_frames,degenerate_frames\n"
          << n_img << ',' << n_img_rejected << ',' << n_gyro << ',' << n_acc
          << ',' << n_pose << ',' << (n_img ? solve_ms_sum / n_img : 0.0) << ','
          << solve_ms_max << ',' << wall_s << ',' << imu_zero_frames << ','
          << degenerate_frames << '\n';
    }

    // 轨迹为空 = 失败。别让"跑完了"冒充"跑对了"。
    return traj.empty() ? EXIT_FAILURE : EXIT_SUCCESS;
}
