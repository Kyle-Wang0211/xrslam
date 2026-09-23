// test_guided_lk.cpp — unit test of OpenCvImage::track_keypoints_guided (tracking-loss
// recovery's projection search, see xrslam/src/xrslam/core/sliding_window_tracker.h).
//
// What it proves, on a synthetic textured image shifted by a known amount:
//   1. the guided search finds the points when started near the truth, even for a shift far
//      beyond the front end's consecutive-frame gate (rows/4) -- the gate that made the
//      plain track_keypoints() reject every candidate on the 1 s gap of run-6e2d4b99;
//   2. the plain track_keypoints() indeed rejects that same shift (positive control);
//   3. negative controls: a start point whose LK result lands outside the window is rejected,
//      and an unrelated (re-randomised) target image yields (almost) nothing.
//
// Build (after build_pc_headless.sh <src> <B>; D = the deps tarball dir it used; OpenCV 5
// module names -- opencv_features, not features2d):
//   c++ -std=c++17 -O2 -DNDEBUG -ffp-contract=off -fno-fast-math -fchar8_t \
//       -Dceres=pw_xrslam_ceres_1_14 -I<B>.ocv_shim -Ixrslam/include -I<B>/xrslam/include \
//       -Ixrslam-extra/include -I<D>/eigen-3.3.7 -I<D>/ceres_pinned/include \
//       -I<D>/ceres_pinned/internal/ceres/miniglog -I<B>/_deps/depends-ceres-solver-build/config \
//       -I/opt/homebrew/opt/opencv/include/opencv5 pw_tools/regression/test_guided_lk.cpp \
//       <B>/xrslam-extra/libxrslam-extra-opencv-image.a -Llib -lxrslam \
//       -L/opt/homebrew/opt/opencv/lib -lopencv_core -lopencv_imgproc -lopencv_video \
//       -lopencv_features -Wl,-rpath,$PWD/lib -Wl,-rpath,/opt/homebrew/opt/opencv/lib \
//       -o /tmp/test_guided_lk && /tmp/test_guided_lk
#include <xrslam/extra/opencv_image.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <random>

using xrslam::vector;
using xrslam::extra::OpenCvImage;

static int failures = 0;
#define CHECK(cond, ...)                                                       \
    do {                                                                       \
        printf("  %s ", (cond) ? "ok  " : "FAIL");                             \
        printf(__VA_ARGS__);                                                   \
        printf("\n");                                                          \
        if (!(cond))                                                           \
            ++failures;                                                        \
    } while (0)

static cv::Mat texture(int rows, int cols, unsigned seed) {
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> u(0, 255);
    cv::Mat small(rows / 8, cols / 8, CV_8UC1);
    for (int r = 0; r < small.rows; ++r)
        for (int c = 0; c < small.cols; ++c)
            small.at<unsigned char>(r, c) = (unsigned char)u(rng);
    cv::Mat big;
    cv::resize(small, big, cv::Size(cols, rows), 0, 0, cv::INTER_CUBIC);
    cv::GaussianBlur(big, big, cv::Size(5, 5), 1.0);
    return big;
}

static std::shared_ptr<OpenCvImage> make(const cv::Mat &m) {
    auto im = std::make_shared<OpenCvImage>();
    im->image = m.clone();
    im->raw = m.clone();
    im->preprocess(6.0, 8, 8); // the bench's clahe_clip_limit / width / height
    return im;
}

int main() {
    const int rows = 480, cols = 640;
    const cv::Mat base = texture(rows + 400, cols + 400, 7);
    // Source frame = crop at (200,200); target frame = crop shifted by (dx,dy): a scene point at
    // p in the source appears at p - (dx,dy) in the target.
    const int dx = 150, dy = 40; // |shift| = 155 px > rows/4 = 120 px
    auto src = make(base(cv::Rect(200, 200, cols, rows)));
    auto dst = make(base(cv::Rect(200 + dx, 200 + dy, cols, rows)));
    auto other = make(texture(rows, cols, 12345));

    std::vector<vector<2>> pts;
    for (int y = 190; y <= 400; y += 30)
        for (int x = 190; x <= 600; x += 40)
            pts.emplace_back(x, y);
    const size_t n = pts.size();
    auto truth = [&](const vector<2> &p) { return vector<2>(p.x() - dx, p.y() - dy); };

    printf("guided search, %zu points, true shift (%d,%d) = %.0f px, window 60 px\n", n, dx, dy,
           std::hypot(dx, dy));

    // 1. started 25 px off the truth (an IMU prediction error), window 60 px.
    {
        std::vector<vector<2>> next;
        for (auto &p : pts)
            next.push_back(truth(p) + vector<2>(18.0, -17.0));
        std::vector<char> st;
        src->track_keypoints_guided(dst.get(), pts, next, st, 60.0);
        size_t found = 0, within_half = 0, within_gate = 0;
        double worst = 0.0;
        for (size_t i = 0; i < n; ++i)
            if (st[i]) {
                ++found;
                const double e = (next[i] - truth(pts[i])).norm();
                worst = std::max(worst, e);
                within_half += e < 0.5 ? 1 : 0;
                within_gate += e < 3.0 ? 1 : 0;
            }
        CHECK(found >= n * 9 / 10, "found %zu/%zu with a 25 px prediction error", found, n);
        // CLAHE is applied per crop, so the two images differ slightly beyond the shift: not
        // every match is sub-pixel. The bound that matters downstream is the tracker's 3 px
        // TT_VALID gate (sliding_window_tracker.cpp), which a correct match must pass.
        CHECK(within_gate == found,
              "%zu/%zu within the 3 px TT_VALID gate (%zu within 0.5 px, worst %.2f px)",
              within_gate, found, within_half, worst);
    }
    // 2. positive control: the front end's own call rejects the same motion (rows/4 gate).
    {
        std::vector<vector<2>> next;
        for (auto &p : pts)
            next.push_back(truth(p) + vector<2>(18.0, -17.0));
        std::vector<char> st;
        src->track_keypoints(dst.get(), pts, next, st);
        size_t found = 0;
        for (char s : st)
            found += s ? 1 : 0;
        CHECK(found == 0, "plain track_keypoints keeps %zu/%zu (expected 0: shift > rows/4)",
              found, n);
    }
    // 3a. negative control: window smaller than the prediction error => rejected.
    {
        std::vector<vector<2>> next;
        for (auto &p : pts)
            next.push_back(truth(p) + vector<2>(18.0, -17.0));
        std::vector<char> st;
        src->track_keypoints_guided(dst.get(), pts, next, st, 10.0);
        size_t found = 0;
        for (char s : st)
            found += s ? 1 : 0;
        CHECK(found == 0, "window 10 px < 25 px error keeps %zu/%zu (expected 0)", found, n);
    }
    // 3b. negative control: unrelated target image => (almost) nothing survives the
    //     forward-backward check.
    {
        std::vector<vector<2>> next;
        for (auto &p : pts)
            next.push_back(truth(p));
        std::vector<char> st;
        src->track_keypoints_guided(other.get(), pts, next, st, 60.0);
        size_t found = 0;
        for (char s : st)
            found += s ? 1 : 0;
        // LK + forward-backward check is not immune to texture that happens to align; what
        // must hold is that such chance matches stay below the recovery threshold (15), so
        // they alone can never declare a recovery.
        CHECK(found < 15, "unrelated image keeps %zu/%zu (%.1f%%; must stay < 15 = the "
                          "recovery threshold)", found, n, 100.0 * found / n);
    }
    // 4. size mismatch / empty input is handled (status all 0, no crash).
    {
        std::vector<vector<2>> next; // wrong size: must hold the predictions
        std::vector<char> st;
        src->track_keypoints_guided(dst.get(), pts, next, st, 60.0);
        size_t found = 0;
        for (char s : st)
            found += s ? 1 : 0;
        CHECK(st.size() == n && found == 0, "missing predictions => %zu found, status size %zu",
              found, st.size());
    }
    printf("%s (%d failure%s)\n", failures ? "FAILED" : "ALL PASSED", failures,
           failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
