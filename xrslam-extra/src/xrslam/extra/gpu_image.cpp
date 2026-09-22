#include <xrslam/extra/gpu_image.h>
#include "../../../../xrslam/src/xrslam/utility/pw_trace.h"
#include <xrslam/extra/poisson_disk_filter.h>
#include <pw_gpu_frontend.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <sys/stat.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <opencv2/features2d.hpp>
#include <opencv2/video.hpp>
namespace xrslam::extra {
void write_stats();   // defined below (outside the anonymous namespace)
namespace {
std::once_flag g_once;
std::unique_ptr<pw::gpufe::FrontEnd> g_fe;
bool g_wanted = false;
std::string g_selfcheck = "{}"; std::string g_init_error; char g_last_fallback[240] = "";
const char *note_fallback(const std::string &e) { snprintf(g_last_fallback, sizeof g_last_fallback, "%s", e.c_str()); return g_last_fallback; }
void marker(const char *stage) {   // crash-localisation trail: $HOME/Documents/xrslam_gpufe_init.log
    const char *home = std::getenv("HOME"); if (!home) return;
    if (FILE *f = fopen((std::string(home) + "/Documents/xrslam_gpufe_init.log").c_str(), "a")) { fprintf(f, "%s\n", stage); fclose(f); }
}
void init_once() {
    const char *e = std::getenv("PW_XRSLAM_GPU_FRONTEND");
    g_wanted = e && e[0] == '1';
    if (!g_wanted) return;
    // (AETHER_GPU_TIMESTAMPS=1 per-kernel timing: works on the Mac; on the iPhone the first timestamped batch never returned — left off.)
    marker("init: create front end");
    std::string err;
    g_fe = pw::gpufe::FrontEnd::create(&err);
    if (!g_fe) { g_init_error = err; marker(("init: FAILED " + err).c_str()); write_stats(); fprintf(stderr, "[xrslam-gpufe] front end unavailable: %s (falling back to CPU)\n", err.c_str()); return; }
    marker("init: front end ok; selfcheck");
    g_selfcheck = g_fe->selfcheck();
    marker(("init: selfcheck done " + g_selfcheck).c_str());
}
} // namespace
size_t g_fallbacks = 0, g_frames = 0; double g_cpu_pyr_ms = 0, g_cpu_lk_ms = 0; size_t g_cpu_pyr_n = 0, g_cpu_lk_n = 0;
inline double now_ms_() { return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count(); } double g_lat_sum = 0; size_t g_lat_n = 0; double g_bucket[8][3] = {{0}}; size_t g_bucket_n[8] = {0};   // stage ms per 1000-frame bucket char g_last_fallback[240] = "";
// On-device numerics audit (every AUDIT_EVERY-th frame): the CPU path is run on the same inputs and compared bit-for-bit.
constexpr size_t AUDIT_EVERY = 50;
// [2026-09-09] PW_XRSLAM_GPUFE_LK=1: run Lucas-Kanade on the GPU as well, the path whose on-device
// audits were bit-exact against the CPU tracker before the hybrid split was chosen. The split was
// chosen on wall clock at throttled GPU clocks; it was never compared on ENERGY, and the GPU path
// avoids a 2.76 MB readback per frame plus the CPU pyramid and LK entirely. Two implementations that
// were verified to produce identical output, so picking the cooler one costs no accuracy.
// [2026-09-09] Read lazily, not at namespace scope. A namespace-scope initialiser runs when the
// library loads, which is BEFORE the app's setenv, so an env-driven arm silently ran the default path
// and its measurement looked like a null result. Both flags below were affected.
bool gpu_lk_enabled() {
    static const bool v = [] { const char *a = std::getenv("PW_XRSLAM_GPUFE_LK"); return a && a[0] == '1'; }();
    return v;
}
bool gpu_pyr_enabled() {
    static const bool v = [] { const char *a = std::getenv("PW_XRSLAM_GPUFE_PYR"); return a && a[0] == '1'; }();
    return v;
}
size_t g_audit_clahe = 0, g_audit_clahe_bad = 0, g_audit_detect = 0, g_audit_detect_bad = 0, g_audit_track = 0, g_audit_track_bad = 0;
size_t g_last_pts = 0, g_last_fwd_bad = 0, g_last_fwd_stbad = 0, g_last_rev_bad = 0; double g_last_fwd_maxdiff = 0; char g_last_example[256] = "";
// Audit trail (装机≠生效): every 50 preprocessed frames, and whenever a fallback happens, dump the front-end
// counters/timings to $HOME/Documents/xrslam_gpufe_stats.json (iOS app sandbox). Pulled by the bench chain.
// [PW 2026-09-16] 后端记账的计数在 xrslam/core/frontend_worker.cpp,经这里的
// stats 出口一起导出,这样「前端 / 后端 / 其余」三笔账在同一个文件里同一场内可比。
extern "C" {
extern double pw_bk_work_ms;
extern double pw_bk_init_ms;
extern double pw_bk_mirror_ms;
extern double pw_bk_track_ms;
extern unsigned long long pw_bk_work_n;
extern unsigned long long pw_bk_init_n;
extern unsigned long long pw_bk_track_n;
}

std::string bucket_json() { std::string j; char b[96]; for (int i = 0; i < 8; ++i) { if (!g_bucket_n[i]) break; snprintf(b, sizeof b, "%s[%.1f,%.1f,%.1f]", i ? "," : "", g_bucket[i][0] / g_bucket_n[i], g_bucket[i][1] / g_bucket_n[i], g_bucket[i][2] / g_bucket_n[i]); j += b; } return j; }
void write_stats() {
    if (!g_fe) { if (const char *home = std::getenv("HOME")) { if (FILE *f = fopen((std::string(home) + "/Documents/xrslam_gpufe_stats.json").c_str(), "w")) { fprintf(f, "{\"gpu_frontend\":false,\"init_error\":\"%s\"}\n", g_init_error.c_str()); fclose(f); } } return; }
    const char *home = std::getenv("HOME"); if (!home) return;
    std::string path = std::string(home) + "/Documents/xrslam_gpufe_stats.json";
    auto st = g_fe->stats();
    FILE *f = fopen(path.c_str(), "w"); if (!f) return;
    fprintf(f, "{\"gpu_frontend\":true,\"gpu_lk\":%d,\"gpu_pyr\":%d,\"frames\":%zu,\"fallbacks\":%zu,\"n_preprocess\":%llu,\"n_detect\":%llu,\"n_track\":%llu,"
               "\"avg_preprocess_ms\":%.3f,\"avg_detect_ms\":%.3f,\"avg_track_ms\":%.3f,\"healthy\":%s,"
               "\"audit\":{\"every\":%zu,\"clahe\":[%zu,%zu],\"detect\":[%zu,%zu],\"track\":[%zu,%zu],"
               "\"last_track\":{\"pts\":%zu,\"fwd_pos_bad\":%zu,\"fwd_status_bad\":%zu,\"rev_bad\":%zu,\"fwd_maxdiff\":%.6g,\"example\":\"%s\"},"
               "\"note\":\"CPU path re-run on the same inputs every 50th frame; [frames, mismatching frames]\"},"
               "\"breakdown_ms\":{\"pre_wait\":%.2f,\"pre_host\":%.2f,\"pre_gpu\":%.2f,\"pre_read\":%.2f,\"det_gpu\":%.2f,\"det_read\":%.2f,\"det_host\":%.2f,\"trk_host\":%.2f,\"trk_gpu\":%.2f,\"trk_read\":%.2f},\"last_fallback\":\"%s\",\"cpu_pyramid_ms\":%.2f,\"cpu_lk_ms\":%.2f,\"prefetch_declined\":%llu,\"empty_submit_ms\":%.2f,\"buckets_per1000\":[%s],\"gpu_kernel_ms\":%s,\"selfcheck\":%s,"
               "\"backend_ms\":{\"work\":%.3f,\"init\":%.3f,\"mirror\":%.3f,\"track\":%.3f,\"work_n\":%llu,\"init_n\":%llu,\"track_n\":%llu}}\n",
            (int)gpu_lk_enabled(), (int)gpu_pyr_enabled(), g_frames, g_fallbacks, (unsigned long long)st.n_preprocess, (unsigned long long)st.n_detect, (unsigned long long)st.n_track,
            st.n_preprocess ? st.preprocess_ms / st.n_preprocess : 0.0, st.n_detect ? st.detect_ms / st.n_detect : 0.0, st.n_track ? st.track_ms / st.n_track : 0.0,
            g_fe->healthy() ? "true" : "false", AUDIT_EVERY, g_audit_clahe, g_audit_clahe_bad, g_audit_detect, g_audit_detect_bad, g_audit_track, g_audit_track_bad,
            g_last_pts, g_last_fwd_bad, g_last_fwd_stbad, g_last_rev_bad, g_last_fwd_maxdiff, g_last_example,
            st.n_preprocess ? st.pre_wait_ms / st.n_preprocess : 0.0, st.n_preprocess ? st.pre_host_ms / st.n_preprocess : 0.0, st.n_preprocess ? st.pre_gpu_ms / st.n_preprocess : 0.0, st.n_preprocess ? st.pre_read_ms / st.n_preprocess : 0.0,
            st.n_detect ? st.det_gpu_ms / st.n_detect : 0.0, st.n_detect ? st.det_read_ms / st.n_detect : 0.0, st.n_detect ? st.det_host_ms / st.n_detect : 0.0,
            st.n_track ? st.trk_host_ms / st.n_track : 0.0, st.n_track ? st.trk_gpu_ms / st.n_track : 0.0, st.n_track ? st.trk_read_ms / st.n_track : 0.0, g_last_fallback, g_cpu_pyr_n ? g_cpu_pyr_ms / g_cpu_pyr_n : 0.0, g_cpu_lk_n ? g_cpu_lk_ms / g_cpu_lk_n : 0.0, (unsigned long long)st.prefetch_declined, g_lat_n ? g_lat_sum / g_lat_n : -1.0, bucket_json().c_str(), g_fe->kernel_times().c_str(), g_selfcheck.c_str(),
            pw_bk_work_n ? pw_bk_work_ms / pw_bk_work_n : 0.0,
            pw_bk_init_n ? pw_bk_init_ms / pw_bk_init_n : 0.0,
            pw_bk_track_n ? pw_bk_mirror_ms / pw_bk_track_n : 0.0,
            pw_bk_track_n ? pw_bk_track_ms / pw_bk_track_n : 0.0,
            (unsigned long long)pw_bk_work_n, (unsigned long long)pw_bk_init_n,
            (unsigned long long)pw_bk_track_n);
    fclose(f);
}
pw::gpufe::FrontEnd *GpuImage::front_end() { std::call_once(g_once, init_once); return g_fe.get(); }
bool GpuImage::enabled() { return front_end() != nullptr; }
std::shared_ptr<OpenCvImage> GpuImage::create_image() {
    if (enabled()) return std::make_shared<GpuImage>();
    return std::make_shared<OpenCvImage>();
}
GpuImage::GpuImage() = default;
GpuImage::~GpuImage() = default;
void GpuImage::prefetch(double clipLimit, int width, int height) {
    PW_ZONE("frontend.gpu.prefetch");   // capture thread: submit the pyramid build now, wait later
    auto *fe = front_end(); if (!fe || frame_) return;
    // The GPU-LK arm needs the frame to carry the pyramid AND the Scharr derivatives, which the fused
    // CLAHE+detect submission does not build; preprocess_async is the pre-hybrid path whose device
    // audits were bit-exact, with detection dispatched separately in detect().
    frame_ = gpu_lk_enabled()
                 ? fe->preprocess_async(image.data, image.cols, image.rows, (int)image.step, clipLimit, width, height, true)
                 : fe->preprocess_detect_async(image.data, image.cols, image.rows, (int)image.step, clipLimit, width, height, 1.0e-3, 0.04, gpu_pyr_enabled());   // CLAHE + GFTT candidates in one submission
    prefetched_ = frame_ != nullptr;
}
void GpuImage::preprocess(double clipLimit, int width, int height) {
    PW_ZONE("frontend.gpu.preprocess");
    auto *fe = front_end();
    std::vector<unsigned char> clahe;
    if (fe && !frame_) frame_ = gpu_lk_enabled()
        ? fe->preprocess(image.data, image.cols, image.rows, (int)image.step, clipLimit, width, height, &clahe)
        : fe->preprocess_detect(image.data, image.cols, image.rows, (int)image.step, clipLimit, width, height, 1.0e-3, 0.04, &clahe, gpu_pyr_enabled());   // no prefetch (declined or none): synchronous, never refused
    else if (fe && frame_ && !fe->finish(*frame_, &clahe)) frame_.reset();
    if (fe) { if (++g_frames % 50 == 1) write_stats(); if (g_frames == 1) marker("first preprocess");
        if (g_frames % 50 == 25) { double l = fe->probe_latency(); if (l >= 0) { g_lat_sum += l; ++g_lat_n; } }
        { auto st = fe->stats(); static double p_pre = 0, p_det = 0, p_trk = 0; static uint64_t n_pre = 0, n_det = 0, n_trk = 0;
          size_t b = std::min<size_t>(7, g_frames / 1000); if (st.n_preprocess > n_pre) { g_bucket[b][0] += st.preprocess_ms - p_pre; } if (st.n_detect > n_det) g_bucket[b][1] += st.detect_ms - p_det; if (st.n_track > n_trk) g_bucket[b][2] += st.track_ms - p_trk; g_bucket_n[b]++;
          p_pre = st.preprocess_ms; p_det = st.detect_ms; p_trk = st.track_ms; n_pre = st.n_preprocess; n_det = st.n_detect; n_trk = st.n_track; } }
    if (!frame_) {
        if (fe) { ++g_fallbacks; write_stats(); fprintf(stderr, "[xrslam-gpufe] preprocess fell back to CPU: %s\n", note_fallback(fe->take_error())); }
        gpu_ok_ = false;
        OpenCvImage::preprocess(clipLimit, width, height);   // CPU path (CLAHE + pyramid)
        return;
    }
    gpu_ok_ = true;
    static const bool audit_on = [] { const char *a = std::getenv("PW_XRSLAM_GPUFE_AUDIT"); return a && a[0] == '1'; }();   // CPU re-run audit costs ~0.5 ms/frame on average: off unless asked
    audit_frame_ = audit_on && (g_frames % AUDIT_EVERY) == 0;
    cv::Mat ref; if (audit_frame_) cv::createCLAHE(clipLimit, cv::Size(width, height))->apply(image, ref);   // CPU reference on the same input (before `image` is replaced)
    if (gpu_pyr_enabled()) {   // GPU-built pyramid: wrap the packed padded readback as the LK pyramid (each level = 21 px REFLECT_101 border, exactly buildOpticalFlowPyramid's layout); zero copy
        std::vector<pw::gpufe::PyrLevel> lay; bool ok = fe->pyramid_layout(*frame_, lay) && lay.size() == level_num() + 1;
        if (ok) { pyr_bytes_ = std::move(clahe); image_pyramid.clear();
            const double t0 = now_ms_();
            for (const auto &q : lay) {   // each level: buildOpticalFlowPyramid(maxLevel 0, withDerivatives) reuses the bordered ROI as-is (tryReuseInputImage) and computes the Scharr derivative once
                cv::Mat full(q.ph, q.pw, CV_8UC1, pyr_bytes_.data() + size_t(q.base) * 4, size_t(q.pwq) * 4); std::vector<cv::Mat> t;
                cv::buildOpticalFlowPyramid(full(cv::Rect(21, 21, q.w, q.h)), t, cv::Size(21, 21), 0, true);
                image_pyramid.push_back(t[0]); image_pyramid.push_back(t[1]); }
            g_cpu_pyr_ms += now_ms_() - t0; ++g_cpu_pyr_n;   // now = Scharr only (the pyrDown levels came from the GPU)
            image = image_pyramid[0];   // CLAHE image = level 0 view (byte-identical to cv::CLAHE); consumers that filter it see the same REFLECT_101 pixels outside the ROI
        } else { note_fallback("pyramid_layout mismatch"); ++g_fallbacks; gpu_ok_ = false; OpenCvImage::preprocess(clipLimit, width, height); return; }
    } else {
        // keep `image` byte-identical to the CPU path (cv::CLAHE output) for every other consumer
        for (int y = 0; y < image.rows; ++y) memcpy(image.ptr(y), clahe.data() + size_t(y) * image.cols, image.cols);
        const double t0 = now_ms_(); ensure_cpu_pyramid(); g_cpu_pyr_ms += now_ms_() - t0; ++g_cpu_pyr_n;   // hybrid: pyramid + LK stay on the CPU (upstream code)
    }
    if (audit_frame_) { size_t bad = 0; for (int y = 0; y < image.rows; ++y) bad += (size_t)std::count_if(ref.ptr(y), ref.ptr(y) + image.cols, [&, y](const unsigned char &v) { return v != image.ptr(y)[&v - ref.ptr(y)]; });
        ++g_audit_clahe; if (bad) ++g_audit_clahe_bad; }
}
void GpuImage::detect_keypoints(std::vector<vector<2>> &keypoints, size_t max_points, double keypoint_distance) const {
    PW_ZONE("frontend.detect");
    auto *fe = front_end();
    if (!gpu_ok_ || !fe) { OpenCvImage::detect_keypoints(keypoints, max_points, keypoint_distance); return; }
    std::vector<vector<2>> ref_keypoints; bool audit = audit_frame_;
    if (audit) { ref_keypoints = keypoints; OpenCvImage::detect_keypoints(ref_keypoints, max_points, keypoint_distance); }
    std::vector<pw::gpufe::Keypoint> kps;
    if (!fe->detect(*frame_, (int)max_points, 1.0e-3, 20, 0.04, kps)) {   // GFTTDetector::create(max_points, 1e-3, 20, 3, true) in OpenCvImage::gftt
        ++g_fallbacks; write_stats(); fprintf(stderr, "[xrslam-gpufe] detect fell back to CPU: %s\n", note_fallback(fe->take_error()));
        OpenCvImage::detect_keypoints(keypoints, max_points, keypoint_distance);
        return;
    }
    if (kps.size() > 0) {
        // Replicate OpenCvImage::detect_keypoints exactly: GFTTDetector (4.0.1 gftt.cpp:116) returns KeyPoint(corner, blockSize)
        // with response == 0 for every corner, and XRSLAM then std::sort()s by response — a sort over equal keys that
        // permutes the goodFeaturesToTrack order in an implementation-defined but deterministic way. The Poisson filter
        // below is order-dependent, so the same std::sort over the same cv::KeyPoint sequence is applied here.
        std::vector<cv::KeyPoint> cvkeypoints; cvkeypoints.reserve(kps.size());
        for (const auto &k : kps) cvkeypoints.emplace_back(cv::Point2f(k.x, k.y), 3.0f);
        std::sort(cvkeypoints.begin(), cvkeypoints.end(),
                  [](const auto &a, const auto &b) {
                      return a.response > b.response;
                  });
        std::vector<vector<2>> new_keypoints;
        for (size_t i = 0; i < cvkeypoints.size(); ++i) {
            new_keypoints.emplace_back(cvkeypoints[i].pt.x,
                                       cvkeypoints[i].pt.y);
        }
        PoissonDiskFilter<2> filter(keypoint_distance);
        filter.preset_points(keypoints);
        filter.insert_points(new_keypoints);
        new_keypoints.erase(std::remove_if(new_keypoints.begin(), new_keypoints.end(),
                                           [this](const auto &keypoint) {
                                               return keypoint.x() < 20 || keypoint.y() < 20 ||
                                                      keypoint.x() >= image.cols - 20 || keypoint.y() >= image.rows - 20;
                                           }),
                            new_keypoints.end());
        keypoints.insert(keypoints.end(), new_keypoints.begin(), new_keypoints.end());
    }
    if (audit) {
        bool same = ref_keypoints.size() == keypoints.size();
        for (size_t i = 0; same && i < keypoints.size(); ++i) same = ref_keypoints[i].x() == keypoints[i].x() && ref_keypoints[i].y() == keypoints[i].y();
        ++g_audit_detect; if (!same) ++g_audit_detect_bad;
    }
}
void GpuImage::ensure_cpu_pyramid() const {   // GPU-preprocessed frame used by a CPU fallback: build the CPU pyramid from the byte-identical CLAHE image
    // The pyramid that actually runs with the GPU front end on; the zone that
    // was placed in OpenCvImage::preprocess never fires on this path.
    PW_ZONE("frontend.cpu_pyramid_for_lk");
    if (image_pyramid.empty() && !image.empty()) cv::buildOpticalFlowPyramid(image, const_cast<std::vector<cv::Mat> &>(image_pyramid), cv::Size(21, 21), (int)level_num(), true);
}
void GpuImage::track_keypoints(const Image *next_image, const std::vector<vector<2>> &curr_keypoints,
                               std::vector<vector<2>> &next_keypoints, std::vector<char> &result_status) const {
    PW_ZONE("frontend.track_lk");
    auto *fe = front_end();
    const GpuImage *next = dynamic_cast<const GpuImage *>(next_image);
    if (!gpu_lk_enabled()) {   // hybrid split: LK on the CPU (the upstream OpenCV path itself); GPU only does CLAHE + detection
        ensure_cpu_pyramid(); if (next) next->ensure_cpu_pyramid();
        std::vector<vector<2>> init; if (audit_frame_) init = next_keypoints;   // XRSLAM passes the IMU-predicted positions here as the LK initial flow: the audit must start from the same
        const double t0 = now_ms_(); OpenCvImage::track_keypoints(next_image, curr_keypoints, next_keypoints, result_status); g_cpu_lk_ms += now_ms_() - t0; ++g_cpu_lk_n;
        if (audit_frame_ && next && gpu_pyr_enabled() && !curr_keypoints.empty()) {   // audit: same upstream LK on CPU-built pyramids (buildOpticalFlowPyramid of the same images) must give identical output
            std::vector<cv::Mat> pa, pb; cv::buildOpticalFlowPyramid(image, pa, cv::Size(21, 21), (int)level_num(), true); cv::buildOpticalFlowPyramid(next->image, pb, cv::Size(21, 21), (int)level_num(), true);
            auto *self = const_cast<GpuImage *>(this); auto *nx = const_cast<GpuImage *>(next);
            std::vector<cv::Mat> sa, sb; sa.swap(self->image_pyramid); self->image_pyramid = pa; sb.swap(nx->image_pyramid); nx->image_pyramid = pb;
            std::vector<vector<2>> n2 = init; std::vector<char> s2; OpenCvImage::track_keypoints(next_image, curr_keypoints, n2, s2);
            self->image_pyramid.swap(sa); nx->image_pyramid.swap(sb);
            size_t pos_bad = 0, st_bad = 0; double maxd = 0; g_last_example[0] = 0;
            for (size_t i = 0; i < curr_keypoints.size(); ++i) {
                if ((s2[i] != 0) != (result_status[i] != 0)) ++st_bad;
                if (s2[i] && result_status[i]) { double d = std::max(std::fabs(n2[i][0] - next_keypoints[i][0]), std::fabs(n2[i][1] - next_keypoints[i][1])); if (d > 0) { ++pos_bad; maxd = std::max(maxd, d); if (!g_last_example[0]) snprintf(g_last_example, sizeof g_last_example, "i=%zu gpu-pyr=(%.6f,%.6f) cpu-pyr=(%.6f,%.6f)", i, next_keypoints[i][0], next_keypoints[i][1], n2[i][0], n2[i][1]); } } }
            g_last_pts = curr_keypoints.size(); g_last_fwd_bad = pos_bad; g_last_fwd_stbad = st_bad; g_last_rev_bad = 0; g_last_fwd_maxdiff = maxd;
            ++g_audit_track; if (pos_bad || st_bad) ++g_audit_track_bad;
        }
        return;
    }
    if (!gpu_ok_ || !fe || !next || !next->gpu_ok_) { ensure_cpu_pyramid(); if (next) next->ensure_cpu_pyramid(); ++g_fallbacks; OpenCvImage::track_keypoints(next_image, curr_keypoints, next_keypoints, result_status); return; }
    std::vector<std::array<float, 2>> curr(curr_keypoints.size()), nxt;
    for (size_t i = 0; i < curr_keypoints.size(); ++i) curr[i] = {(float)curr_keypoints[i].x(), (float)curr_keypoints[i].y()};
    if (next_keypoints.size() > 0) {
        nxt.resize(next_keypoints.size());
        for (size_t i = 0; i < next_keypoints.size(); ++i) nxt[i] = {(float)next_keypoints[i].x(), (float)next_keypoints[i].y()};
    } else {
        next_keypoints.resize(curr_keypoints.size());
        nxt = curr;
    }
    result_status.resize(curr_keypoints.size(), 0);
    if (curr.size() > 0) {
        std::vector<unsigned char> st, rst; std::vector<std::array<float, 2>> rev = curr;
        if (audit_frame_) init_nxt_ = nxt;   // XRSLAM may pass predicted positions as the initial flow: the CPU reference must use the same
        if (!fe->track_fwd_rev(*frame_, *next->frame_, curr, nxt, st, rev, rst)) { ++g_fallbacks; write_stats(); fprintf(stderr, "[xrslam-gpufe] track fell back to CPU: %s\n", note_fallback(fe->take_error())); ensure_cpu_pyramid(); next->ensure_cpu_pyramid(); OpenCvImage::track_keypoints(next_image, curr_keypoints, next_keypoints, result_status); return; }
        if (audit_frame_) { gpu_nxt_ = nxt; gpu_st_ = st; gpu_rev_ = rev; gpu_rst_ = rst; }
        for (size_t i = 0; i < nxt.size(); ++i) {
            result_status[i] = st[i];
            if (nxt[i][0] < 20 || nxt[i][0] >= image.cols - 20 || nxt[i][1] < 20 || nxt[i][1] >= image.rows - 20) result_status[i] = 0;
            if (result_status[i]) {
                cv::Point2f p(nxt[i][0] - curr[i][0], nxt[i][1] - curr[i][1]);
                auto v = vector<2>(p.x, p.y);
                if (v.norm() > image.rows / 4) result_status[i] = 0;
            }
        }
        for (size_t i = 0; i < rst.size(); ++i) {
            if (result_status[i]) {
                if (!rst[i] || cv::norm(cv::Point2f(curr[i][0], curr[i][1]) - cv::Point2f(rev[i][0], rev[i][1])) > 0.5) result_status[i] = 0;
            }
        }
    }
    if (audit_frame_ && curr.size() > 0) {   // CPU LK on CPU-built pyramids of the (GPU-CLAHE'd, byte-identical) images
        std::vector<cv::Mat> pa, pb; cv::buildOpticalFlowPyramid(image, pa, cv::Size(21, 21), (int)level_num(), true); cv::buildOpticalFlowPyramid(next->image, pb, cv::Size(21, 21), (int)level_num(), true);
        std::vector<cv::Point2f> c(curr.size()), n0(curr.size()); for (size_t i = 0; i < curr.size(); ++i) { c[i] = cv::Point2f(curr[i][0], curr[i][1]); n0[i] = next_keypoints.size() == curr.size() && result_status.size() == curr.size() ? c[i] : c[i]; }
        std::vector<cv::Point2f> n1(curr.size()); for (size_t i = 0; i < curr.size(); ++i) n1[i] = cv::Point2f(init_nxt_[i][0], init_nxt_[i][1]); std::vector<unsigned char> s1, s2; std::vector<float> e1, e2;
        cv::calcOpticalFlowPyrLK(pa, pb, c, n1, s1, e1, cv::Size(21, 21), (int)level_num(), cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
        std::vector<cv::Point2f> r1 = c; cv::calcOpticalFlowPyrLK(pb, pa, n1, r1, s2, e2, cv::Size(21, 21), (int)level_num(), cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
        size_t fwd_bad = 0, fwd_stbad = 0, rev_bad = 0; double maxd = 0; g_last_example[0] = 0;
        for (size_t i = 0; i < curr.size(); ++i) {
            if ((s1[i] != 0) != (gpu_st_[i] != 0)) ++fwd_stbad;
            if (s1[i] && gpu_st_[i]) { double d = std::max(std::fabs((double)n1[i].x - gpu_nxt_[i][0]), std::fabs((double)n1[i].y - gpu_nxt_[i][1])); if (d > 0) { ++fwd_bad; maxd = std::max(maxd, d); if (!g_last_example[0]) snprintf(g_last_example, sizeof g_last_example, "i=%zu curr=(%.4f,%.4f) cpu=(%.7g,%.7g) gpu=(%.7g,%.7g)", i, curr[i][0], curr[i][1], n1[i].x, n1[i].y, gpu_nxt_[i][0], gpu_nxt_[i][1]); } }
            if (((s2[i] != 0) != (gpu_rst_[i] != 0)) || (s2[i] && gpu_rst_[i] && (r1[i].x != gpu_rev_[i][0] || r1[i].y != gpu_rev_[i][1]))) ++rev_bad;
        }
        g_last_pts = curr.size(); g_last_fwd_bad = fwd_bad; g_last_fwd_stbad = fwd_stbad; g_last_rev_bad = rev_bad; g_last_fwd_maxdiff = maxd;
        ++g_audit_track; if (fwd_bad || fwd_stbad || rev_bad) ++g_audit_track_bad;
        static bool dumped = false;
        if ((fwd_bad || fwd_stbad || rev_bad) && !dumped) {   // first mismatching audit frame: dump inputs for offline replay
            dumped = true;
            if (const char *home = std::getenv("HOME")) {
                std::string d = std::string(home) + "/Documents/xrslam_gpufe_dump"; mkdir(d.c_str(), 0755);
                if (FILE *f = fopen((d + "/prev_clahe.u8").c_str(), "wb")) { for (int y = 0; y < image.rows; ++y) fwrite(image.ptr(y), 1, image.cols, f); fclose(f); }
                if (FILE *f = fopen((d + "/next_clahe.u8").c_str(), "wb")) { for (int y = 0; y < next->image.rows; ++y) fwrite(next->image.ptr(y), 1, next->image.cols, f); fclose(f); }
                if (FILE *f = fopen((d + "/pts.txt").c_str(), "w")) { fprintf(f, "# w h\n%d %d\n# curr_x curr_y cpu_x cpu_y cpu_st gpu_x gpu_y gpu_st cpu_rx cpu_ry cpu_rst gpu_rx gpu_ry gpu_rst init_x init_y\n", image.cols, image.rows);
                    for (size_t i = 0; i < curr.size(); ++i) fprintf(f, "%.9g %.9g %.9g %.9g %d %.9g %.9g %d %.9g %.9g %d %.9g %.9g %d %.9g %.9g\n", curr[i][0], curr[i][1], n1[i].x, n1[i].y, (int)s1[i], gpu_nxt_[i][0], gpu_nxt_[i][1], (int)gpu_st_[i], r1[i].x, r1[i].y, (int)s2[i], gpu_rev_[i][0], gpu_rev_[i][1], (int)gpu_rst_[i], init_nxt_[i][0], init_nxt_[i][1]);
                    fclose(f); }
            }
        }
    }
    for (size_t i = 0; i < curr_keypoints.size(); ++i) {
        if (result_status[i]) { next_keypoints[i].x() = nxt[i][0]; next_keypoints[i].y() = nxt[i][1]; }
    }
}
void GpuImage::release_image_buffer() { frame_.reset(); OpenCvImage::release_image_buffer(); std::vector<unsigned char>().swap(pyr_bytes_); }
} // namespace xrslam::extra
