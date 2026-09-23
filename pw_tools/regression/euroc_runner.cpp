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
// [2026-09-23 solver time budget] --timing-csv <path>:逐帧计时(只读,不改轨迹)。
//   一「帧周期」= 从这一帧 PushSensorData(CAMERA) 起、到下一帧 CAMERA 之前,本 runner
//   对引擎 C API 的全部调用(Push*/RunOneFrame/GetResult)的墙钟之和。threading OFF 时
//   整条流水线(前端 + 滑窗)都同步跑在这些调用里,所以它就是这一帧的算力延迟;
//   读数据集/解 PNG 不计入。另外对引擎导出的遥测计数器(solver.cpp 的 pw_solver_*、
//   frontend_worker.cpp 的 pw_bk_*)取逐帧差分 —— 用 dlsym 找,找不到(比如未改动的
//   引擎)就留空,所以同一个 runner 也能对着旧库跑。
//
//   pw_euroc_runner <slam.yaml> <device.yaml> euroc://<dir> <out.tum> [--intrinsics-csv k.csv]
//                   [--timing-csv timing.csv]

#include "dataset_reader.h"
#include <XRSLAM.h>
#include <xrslam/extra/yaml_config.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <dlfcn.h>
#include <time.h>
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

// ---- [2026-09-23] 逐帧计时 -------------------------------------------------------------
using Clock = std::chrono::steady_clock;

struct Telemetry {
    // solver.cpp(本分支新增);旧库没有 ⇒ nullptr
    const std::atomic<unsigned long long> *scoped_ns = nullptr, *scoped_calls = nullptr,
                                          *scoped_iters = nullptr, *unscoped_ns = nullptr,
                                          *stop_budget = nullptr, *stop_time = nullptr,
                                          *stop_iter = nullptr, *stop_conv = nullptr;
    // frontend_worker.cpp:23-29(已有)
    const double *bk_work_ms = nullptr, *bk_track_ms = nullptr;
    const unsigned long long *bk_track_n = nullptr;

    template <typename T> static const T *sym(const char *name) {
        return static_cast<const T *>(dlsym(RTLD_DEFAULT, name));
    }
    void bind() {
        using A = std::atomic<unsigned long long>;
        scoped_ns = sym<A>("pw_solver_scoped_ns");
        scoped_calls = sym<A>("pw_solver_scoped_calls");
        scoped_iters = sym<A>("pw_solver_scoped_iterations");
        unscoped_ns = sym<A>("pw_solver_unscoped_ns");
        stop_budget = sym<A>("pw_solver_stop_budget");
        stop_time = sym<A>("pw_solver_stop_time_limit");
        stop_iter = sym<A>("pw_solver_stop_iter_limit");
        stop_conv = sym<A>("pw_solver_stop_converged");
        bk_work_ms = sym<double>("pw_bk_work_ms");
        bk_track_ms = sym<double>("pw_bk_track_ms");
        bk_track_n = sym<unsigned long long>("pw_bk_track_n");
    }
};

struct Snap {
    double scoped_ms = NAN, unscoped_ms = NAN, bk_work_ms = NAN, bk_track_ms = NAN;
    long long scoped_calls = -1, scoped_iters = -1, stop_budget = -1, stop_time = -1,
              stop_iter = -1, stop_conv = -1, bk_track_n = -1;
};

Snap take(const Telemetry &tm) {
    Snap s;
    auto a = [](const std::atomic<unsigned long long> *p) -> long long {
        return p ? (long long)p->load(std::memory_order_relaxed) : -1;
    };
    if (tm.scoped_ns) s.scoped_ms = tm.scoped_ns->load() * 1e-6;
    if (tm.unscoped_ns) s.unscoped_ms = tm.unscoped_ns->load() * 1e-6;
    if (tm.bk_work_ms) s.bk_work_ms = *tm.bk_work_ms;
    if (tm.bk_track_ms) s.bk_track_ms = *tm.bk_track_ms;
    s.scoped_calls = a(tm.scoped_calls);
    s.scoped_iters = a(tm.scoped_iters);
    s.stop_budget = a(tm.stop_budget);
    s.stop_time = a(tm.stop_time);
    s.stop_iter = a(tm.stop_iter);
    s.stop_conv = a(tm.stop_conv);
    s.bk_track_n = tm.bk_track_n ? (long long)*tm.bk_track_n : -1;
    return s;
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
    std::string k_csv, timing_csv;
    for (int i = 5; i + 1 < argc; ++i) {
        if (std::string(argv[i]) == "--intrinsics-csv")
            k_csv = argv[i + 1];
        if (std::string(argv[i]) == "--timing-csv")
            timing_csv = argv[i + 1];
    }
    Telemetry tm;
    tm.bind();
    FILE *tf = nullptr;
    if (!timing_csv.empty()) {
        tf = fopen(timing_csv.c_str(), "w");
        if (!tf) {
            fprintf(stderr, "cannot write %s\n", timing_csv.c_str());
            return EXIT_FAILURE;
        }
        fprintf(tf, "frame,t,wall_ms,cpu_ms,sw_solver_ms,sw_solves,sw_iters,stop_budget,stop_time,"
                    "stop_iter,stop_conv,init_solver_ms,bk_work_ms,bk_track_ms,bk_track_n\n");
        fprintf(stderr, "[timing] 遥测 %s / %s\n",
                tm.scoped_ns ? "pw_solver_* 有" : "pw_solver_* 无(旧库)",
                tm.bk_work_ms ? "pw_bk_* 有" : "pw_bk_* 无");
    }
    // 当前帧周期的累计
    long period_frame = -1;
    double period_t = 0.0, period_wall_ms = 0.0, period_cpu_ms = 0.0;
    Snap period_start;
    auto flush_period = [&]() {
        if (!tf || period_frame < 0)
            return;
        Snap e = take(tm);
        auto dl = [](long long b, long long a) { return (a < 0 || b < 0) ? -1LL : b - a; };
        fprintf(tf, "%ld,%.9f,%.4f,%.4f,%.4f,%lld,%lld,%lld,%lld,%lld,%lld,%.4f,%.4f,%.4f,%lld\n",
                period_frame, period_t, period_wall_ms, period_cpu_ms,
                e.scoped_ms - period_start.scoped_ms,
                dl(e.scoped_calls, period_start.scoped_calls),
                dl(e.scoped_iters, period_start.scoped_iters),
                dl(e.stop_budget, period_start.stop_budget),
                dl(e.stop_time, period_start.stop_time),
                dl(e.stop_iter, period_start.stop_iter),
                dl(e.stop_conv, period_start.stop_conv),
                e.unscoped_ms - period_start.unscoped_ms,
                e.bk_work_ms - period_start.bk_work_ms, e.bk_track_ms - period_start.bk_track_ms,
                dl(e.bk_track_n, period_start.bk_track_n));
    };
    // 包住每一次引擎调用,墙钟与本线程 CPU 时间都计入当前帧周期。
    // CPU 时间不受别的进程抢核影响(机器被别的任务压满时墙钟会虚高),用来对照墙钟;
    // 但 Ceres 的预算判据本身用的是墙钟(trust_region_minimizer.cc WallTimeInSeconds)。
    auto thread_cpu_ms = [] {
        timespec ts;
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
        return ts.tv_sec * 1e3 + ts.tv_nsec * 1e-6;
    };
    auto timed = [&](auto &&fn) {
        const double u0 = thread_cpu_ms();
        const auto c0 = Clock::now();
        fn();
        period_wall_ms += std::chrono::duration<double, std::milli>(Clock::now() - c0).count();
        period_cpu_ms += thread_cpu_ms() - u0;
    };

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
            auto *gp = &g; // C++17 lambda 不能捕获结构化绑定
            timed([&] { XRSLAMPushSensorData(XRSLAM_SENSOR_GYROSCOPE, gp); });
            ++n_gyro;
        } break;
        case DatasetReader::ACCELEROMETER: {
            auto [t, a] = reader->read_accelerometer();
            (void)t;
            auto *ap = &a; // C++17 lambda 不能捕获结构化绑定
            timed([&] { XRSLAMPushSensorData(XRSLAM_SENSOR_ACCELERATION, ap); });
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
            flush_period(); // 上一帧周期到此结束
            period_frame = n_img;
            period_t = t;
            period_wall_ms = 0.0;
            period_cpu_ms = 0.0;
            if (tf)
                period_start = take(tm);
            timed([&] { XRSLAMPushSensorData(XRSLAM_SENSOR_CAMERA, &img); });
            ++n_img;
            timed([&] { XRSLAMRunOneFrame(); });

            XRSLAMPose pose{};
            timed([&] { XRSLAMGetResult(XRSLAM_RESULT_BODY_POSE, &pose); });
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
    flush_period();
    if (tf)
        fclose(tf);

    XRSLAMIntrinsics reported{};
    XRSLAMGetResult(XRSLAM_INFO_INTRINSICS, &reported);
    XRSLAMDestroy();
    // [2026-09-23] 上游 player 的 EurocDatasetReader 把 XRSLAMCreate 交出的**非拥有**裸指针
    // 包进 shared_ptr(xrslam-pc/player/src/IO/euroc_dataset_reader.cpp:6-7),而
    // XRSLAMDestroy 已经释放过同一个 YamlConfig ⇒ reader 析构时二次释放,进程退出码 139
    // (未改动的 04c0e83 runner 同样如此,崩在 main 返回之后、输出已写完)。
    // 这里故意泄漏 reader,只为让退出码可信;对轨迹无任何影响。
    (void)reader.release();

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
