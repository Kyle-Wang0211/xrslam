#include <atomic>
#include <xrslam/common.h>
#include <xrslam/core/detail.h>
#include <xrslam/core/feature_tracker.h>
#include <xrslam/core/frontend_worker.h>
#include <xrslam/core/sliding_window_tracker.h>
#include <xrslam/estimation/solver.h>
#include <xrslam/geometry/lie_algebra.h>
#include <xrslam/geometry/stereo.h>
#include <xrslam/geometry/pnp.h>
#include <xrslam/inspection.h>
#include <xrslam/map/frame.h>
#include <xrslam/map/map.h>
#include <xrslam/map/track.h>
#include <xrslam/utility/unique_timer.h>

namespace xrslam {

// [bench 2026-09-17] Read-only telemetry for "should this frame be trusted".
//
// Two quantities the sliding window already computes are dropped on the floor:
//
//   * `Solver::solve()` returns `solver_summary.IsSolutionUsable()` (solver.cpp:189) and all four
//     call sites here ignore it. `initializer.cpp:444` is the only place in the tree that reads it.
//   * the per-track mean reprojection error, computed twice (once to gate TT_VALID, once in
//     check_frames_rpe), is compared against a bare 3.0 px and then discarded. TT_VALID is read in
//     20 places and already decides which tracks reach the solver and the marginalisation, so the
//     quantity behind it is load-bearing but invisible.
//
// Nothing new is computed here and no decision changes: every increment sits beside an existing
// statement, and the rpe sums are the same doubles the gate already formed. Sums are kept in
// milli-pixels so the accumulator can be a plain integer atomic.
enum PwSolverCounter {
    kPwSolveCalls = 0,      // solve() invocations inside SlidingWindowTracker
    kPwSolveUnusable,       // ... of which Ceres reported !IsSolutionUsable()
    kPwTrackEvaluated,      // triangulated tracks passing through the TT_VALID gate
    kPwTrackRejectDepth,    // ... rejected by the y.z() <= 1e-3 || > 50 window
    kPwTrackRejectRpe,      // ... rejected by mean rpe >= 3.0 px
    kPwTrackRpeSamples,     // tracks that contributed a finite mean rpe
    kPwTrackRpeMilliPx,     // sum of those means, in milli-pixels
    kPwFramesRpeCalls,      // check_frames_rpe() invocations
    kPwFramesRpeReject,     // ... that returned false
    kPwFramesRpeSamples,
    kPwFramesRpeMilliPx,
    kPwSolverCounterCount
};
std::atomic<uint64_t> pw_solver_counters[kPwSolverCounterCount] = {};

namespace {
// The gate the two sites share, kept in one place so the telemetry cannot drift from it.
inline void pw_account_rpe(double rpe, double rpe_count, bool depth_ok, bool rpe_ok,
                           int samples_slot, int millipx_slot) {
    if (!depth_ok || rpe_count <= 0.0)
        return;
    ++pw_solver_counters[samples_slot];
    pw_solver_counters[millipx_slot] +=
        (uint64_t)(rpe / std::max(rpe_count, 1.0) * 1000.0);
    (void)rpe_ok;
}
} // namespace

// [bench 2026-09-17] The TT_VALID reprojection gate, in one place.
//
// `rpe` is in REAL pixels: both call sites take the norm after apply_k(), so the number carries the
// camera's own focal length. A bare pixel threshold therefore only means the same thing on the
// camera it was authored for. Upstream only ever ran 640x480 iPhone calibrations (fx ~= 449-525)
// and EuRoC (fx = 458.65), where a bare 3.0 is self-consistent; at our 1920x1440 target
// (COLMAP self-calibrated fx = 1376.65) the same angle spans three times as many pixels.
//
// The scaling convention is this codebase's own: `initializer.cpp:203` already divides a threshold
// by K(0,0) before handing it to RANSAC. The external statement of the same rule is VINS-Mono
// issue #48 (qintonguav, first author, 2017-07-14): "we tolerate 3-pixel noise under 460 focal
// lengths. If you change to 920, the tolerate pixel will be 6 pixels". RTAB-Map implements exactly
// this recovery for VINS in OdometryVINSFusion.cpp:175-183 (BSD-3).
//
// Default reference_focal <= 0 keeps the bare threshold, so an unconfigured build is upstream's
// behaviour byte for byte. That default is what makes this change measurable rather than assumed:
// the same binary runs both arms.
// `fx` is the mean focal length of the very frames that contributed to `rpe`, accumulated in
// the same loop -- the threshold has to be averaged over the same set the error was.
inline double pw_rpe_threshold(const Config *config, double fx) {
    const double px = config->sliding_window_rpe_threshold_px();
    const double ref = config->sliding_window_rpe_reference_focal();
    if (!(ref > 0.0))
        return px;
    return px * fx / ref;
}

SlidingWindowTracker::SlidingWindowTracker(std::unique_ptr<Map> keyframe_map,
                                           std::shared_ptr<Config> config)
    : map(std::move(keyframe_map)), config(config) {
    for (size_t j = 1; j < map->frame_num(); ++j) {
        Frame *frame_i = map->get_frame(j - 1);
        Frame *frame_j = map->get_frame(j);
        frame_j->preintegration.integrate(frame_j->image->t, frame_i->motion.bg,
                                          frame_i->motion.ba, true, true);
    }
    // [pw 2026-09-23] Recovery: "IMU initialization" time for ORB-SLAM3's 15 s rule (see .h).
    // This tracker is constructed by Initializer::initialize() right after the visual-inertial
    // alignment succeeds, so the newest frame of the initial window is that instant.
    if (config->tracking_recovery_enable() && map->frame_num() > 0)
        pw_t_init = map->get_frame(map->frame_num() - 1)->image->t;
}

SlidingWindowTracker::~SlidingWindowTracker() {
    // [pw 2026-09-23] Recovery only: give the pinned reference image back to the feature
    // tracker's normal release. pw_pinned is never set with recovery off.
    if (pw_pinned && detail) {
        synchronized(detail->feature_tracker->map) {
            detail->feature_tracker->pw_unpin_reference_image();
        }
    }
}

void SlidingWindowTracker::mirror_frame(Map *feature_tracking_map,
                                        size_t frame_id) {
    Frame *keyframe = map->get_frame(map->frame_num() - 1);
    Frame *new_frame_i = keyframe;
    if (!keyframe->subframes.empty()) {
        new_frame_i = keyframe->subframes.back().get();
    }

    size_t frame_index_i =
        feature_tracking_map->frame_index_by_id(new_frame_i->id());
    size_t frame_index_j = feature_tracking_map->frame_index_by_id(frame_id);

    if (frame_index_i == nil() || frame_index_j == nil())
        return;

    Frame *old_frame_i = feature_tracking_map->get_frame(frame_index_i);
    Frame *old_frame_j = feature_tracking_map->get_frame(frame_index_j);

    std::unique_ptr<Frame> curr_frame = std::move(old_frame_j->clone());
    std::vector<ImuData> &new_data = curr_frame->preintegration.data;
    for (size_t index = frame_index_j - 1; index > frame_index_i; --index) {
        std::vector<ImuData> old_data =
            feature_tracking_map->get_frame(index)->preintegration.data;
        new_data.insert(new_data.begin(), old_data.begin(), old_data.end());
    }

    map->attach_frame(curr_frame->clone());
    Frame *new_frame_j = map->get_frame(map->frame_num() - 1);

    for (size_t ki = 0; ki < old_frame_i->keypoint_num(); ++ki) {
        if (Track *track = old_frame_i->get_track(ki)) {
            if (size_t kj = track->get_keypoint_index(old_frame_j);
                kj != nil()) {
                Track *new_track = new_frame_i->get_track(ki, map.get());
                new_track->add_keypoint(new_frame_j, kj);
                track->tag(TT_TRASH) =
                    new_track->tag(TT_TRASH) && !new_track->tag(TT_STATIC);
            }
        }
    }

    map->prune_tracks([](const Track *track) {
        return track->tag(TT_TRASH) && !track->tag(TT_STATIC);
    });

    new_frame_j->preintegration.integrate(new_frame_j->image->t,
                                          new_frame_i->motion.bg,
                                          new_frame_i->motion.ba, true, true);
    new_frame_j->preintegration.predict(new_frame_i, new_frame_j);
}

bool SlidingWindowTracker::track() {

    // [pw 2026-09-23] Tracking-loss recovery, OFF by default; see sliding_window_tracker.h.
    // With it off, everything below is the pre-existing function unchanged.
    if (config->tracking_recovery_enable())
        return pw_track_with_recovery();

    if (config->parsac_flag()) {
        if (judge_track_status()) {
            update_track_status();
        }
    }

    localize_newframe();

    if (manage_keyframe()) {
        track_landmark();
        refine_window();
        slide_window();
    } else {
        refine_subwindow();
    }

    inspect_debug(sliding_window_landmarks, landmarks) {
        std::vector<Landmark> points;
        points.reserve(map->track_num());
        for (size_t i = 0; i < map->track_num(); ++i) {
            if (Track *track = map->get_track(i)) {
                if (track->tag(TT_VALID)) {
                    Landmark point;
                    point.p = track->get_landmark_point();
                    point.triangulated = track->tag(TT_TRIANGULATED);
                    points.push_back(point);
                }
            }
        }
        landmarks = std::move(points);
    }

    return true;
}

void SlidingWindowTracker::localize_newframe() {
    auto solver = Solver::create();

    Frame *frame_i = map->get_frame(map->frame_num() - 2);
    if (!frame_i->subframes.empty()) {
        frame_i = frame_i->subframes.back().get();
    }
    Frame *frame_j = map->get_frame(map->frame_num() - 1);

    solver->add_frame_states(frame_j);

    solver->put_factor(Solver::create_preintegration_prior_factor(
        frame_i, frame_j, frame_j->preintegration));

    for (size_t k = 0; k < frame_j->keypoint_num(); ++k) {
        if (Track *track = frame_j->get_track(k)) {
            if (track->all_tagged(TT_VALID, TT_TRIANGULATED, TT_STATIC)) {
                solver->put_factor(
                    Solver::create_reprojection_prior_factor(frame_j, track));
            }
        }
    }

    ++pw_solver_counters[kPwSolveCalls];
    if (!solver->solve())
        ++pw_solver_counters[kPwSolveUnusable];
}

bool SlidingWindowTracker::manage_keyframe() {
    Frame *keyframe_i = map->get_frame(map->frame_num() - 2);
    Frame *newframe_j = map->get_frame(map->frame_num() - 1);

    if (!keyframe_i->subframes.empty()) {
        if (keyframe_i->subframes.back()->tag(FT_NO_TRANSLATION)) {
            if (newframe_j->tag(FT_NO_TRANSLATION)) {
                // [T]...........<-[R]
                //  +-[R]-[R]-[R]
                // ==>
                // [T]
                //  +-[R-R]-[R]-[R]
            } else {
                // [T]...........<-[T]
                //  +-[R]-[R]-[R]
                // ==>
                // [T]........[R]-[T]
                //  +-[R]-[R]
                keyframe_i->subframes.back()->tag(FT_KEYFRAME) = true;
                map->attach_frame(std::move(keyframe_i->subframes.back()),
                                  map->frame_num() - 1);
                keyframe_i->subframes.pop_back();
                newframe_j->tag(FT_KEYFRAME) = true;
                return true;
            }
        } else {
            if (newframe_j->tag(FT_NO_TRANSLATION)) {
                // [T]...........<-[R]
                //  +-[T]-[T]-[T]
                // ==>
                // [T]........[T]
                //  +-[T]-[T]  +-[R]
                std::unique_ptr<Frame> frame_lifted =
                    std::move(keyframe_i->subframes.back());
                keyframe_i->subframes.pop_back();
                frame_lifted->tag(FT_KEYFRAME) = true;
                frame_lifted->subframes.emplace_back(
                    map->detach_frame(map->frame_num() - 1));
                map->attach_frame(std::move(frame_lifted));
                return true;
            } else {
                if (keyframe_i->subframes.size() >=
                    config->sliding_window_subframe_size()) {
                    // [T]...........<-[T]
                    //  +-[T]-[T]-[T]
                    // ==>
                    // [T]............[T]
                    //  +-[T]-[T]-[T]
                    newframe_j->tag(FT_KEYFRAME) = true;
                    return true;
                }
            }
        }
    }
    size_t mapped_landmark_count = 0;
    for (size_t k = 0; k < newframe_j->keypoint_num(); ++k) {
        if (Track *track = newframe_j->get_track(k)) {
            if (track->all_tagged(TT_VALID, TT_TRIANGULATED, TT_STATIC)) {
                mapped_landmark_count++;
            }
        }
    }

    bool is_keyframe = mapped_landmark_count <
                       config->sliding_window_force_keyframe_landmarks();

#if defined(XRSLAM_IOS)
    is_keyframe = is_keyframe || !newframe_j->tag(FT_NO_TRANSLATION);
#endif

    if (is_keyframe) {
        newframe_j->tag(FT_KEYFRAME) = true;
        return true;
    } else {
        keyframe_i->subframes.emplace_back(
            map->detach_frame(map->frame_num() - 1));
        return false;
    }
}

void SlidingWindowTracker::track_landmark() {
    Frame *newframe_j = map->get_frame(map->frame_num() - 1);

    for (size_t k = 0; k < newframe_j->keypoint_num(); ++k) {
        if (Track *track = newframe_j->get_track(k)) {
            if (!track->tag(TT_TRIANGULATED)) {
                if (auto p = track->triangulate()) {
                    track->set_landmark_point(p.value());
                    track->tag(TT_TRIANGULATED) = true;
                    track->tag(TT_VALID) = true;
                    track->tag(TT_STATIC) = true;
                } else {
                    // outlier
                    track->landmark.inv_depth = -1.0;
                    track->tag(TT_TRIANGULATED) = false;
                    track->tag(TT_VALID) = false;
                }
            }
        }
    }
}

void SlidingWindowTracker::refine_window() {
    Frame *keyframe_i = map->get_frame(map->frame_num() - 2);
    Frame *keyframe_j = map->get_frame(map->frame_num() - 1);

    auto solver = Solver::create();
    if (!map->marginalization_factor) {
        map->marginalization_factor =
            Solver::create_marginalization_factor(map.get());
    }
    for (size_t i = 0; i < map->frame_num(); ++i) {
        Frame *frame = map->get_frame(i);
        solver->add_frame_states(frame);
    }
    std::unordered_set<Track *> visited_tracks;
    for (size_t i = 0; i < map->frame_num(); ++i) {
        Frame *frame = map->get_frame(i);
        for (size_t j = 0; j < frame->keypoint_num(); ++j) {
            Track *track = frame->get_track(j);
            if (!track)
                continue;
            if (visited_tracks.count(track) > 0)
                continue;
            visited_tracks.insert(track);
            if (!track->tag(TT_VALID))
                continue;
            if (!track->tag(TT_STATIC))
                continue;
            if (!track->first_frame()->tag(FT_KEYFRAME))
                continue;
            solver->add_track_states(track);
        }
    }

    solver->add_factor(map->marginalization_factor.get());

    for (size_t i = 0; i < map->frame_num(); ++i) {
        Frame *frame = map->get_frame(i);
        for (size_t j = 0; j < frame->keypoint_num(); ++j) {
            Track *track = frame->get_track(j);
            if (!track)
                continue;
            if (!track->all_tagged(TT_VALID, TT_TRIANGULATED, TT_STATIC))
                continue;
            if (!track->first_frame()->tag(FT_KEYFRAME))
                continue;
            if (frame == track->first_frame())
                continue;
            solver->add_factor(frame->reprojection_error_factors[j].get());
        }
    }

    for (size_t j = 1; j < map->frame_num(); ++j) {
        Frame *frame_i = map->get_frame(j - 1);
        Frame *frame_j = map->get_frame(j);

        frame_j->keyframe_preintegration = frame_j->preintegration;
        if (!frame_i->subframes.empty()) {
            std::vector<ImuData> imu_data;
            for (size_t k = 0; k < frame_i->subframes.size(); ++k) {
                auto &sub_imu_data = frame_i->subframes[k]->preintegration.data;
                imu_data.insert(imu_data.end(), sub_imu_data.begin(),
                                sub_imu_data.end());
            }
            frame_j->keyframe_preintegration.data.insert(
                frame_j->keyframe_preintegration.data.begin(), imu_data.begin(),
                imu_data.end());
        }

        if (frame_j->keyframe_preintegration.integrate(
                frame_j->image->t, frame_i->motion.bg, frame_i->motion.ba, true,
                true)) {
            solver->put_factor(Solver::create_preintegration_error_factor(
                frame_i, frame_j, frame_j->keyframe_preintegration));
        }
    }

    ++pw_solver_counters[kPwSolveCalls];
    if (!solver->solve())
        ++pw_solver_counters[kPwSolveUnusable];

    for (size_t k = 0; k < map->track_num(); ++k) {
        Track *track = map->get_track(k);
        if (track->tag(TT_TRIANGULATED)) {
            bool is_valid = true;
            auto x = track->get_landmark_point();
            double rpe = 0.0;
            double rpe_count = 0.0;
            double rpe_fx = 0.0;
            for (const auto &[frame, keypoint_index] : track->keypoint_map()) {
                if (!frame->tag(FT_KEYFRAME))
                    continue;
                PoseState pose = frame->get_pose(frame->camera);
                vector<3> y = pose.q.conjugate() * (x - pose.p);
                if (y.z() <= 1.0e-3 || y.z() > 50) { // todo
                    is_valid = false;
                    break;
                }
                rpe += (apply_k(y, frame->K) -
                        apply_k(frame->get_keypoint(keypoint_index), frame->K))
                           .norm();
                rpe_count += 1.0;
                rpe_fx += frame->K(0, 0);
            }
            const bool pw_depth_ok = is_valid;   // is_valid so far is the depth window's verdict
            is_valid = is_valid && (rpe / std::max(rpe_count, 1.0) <
                                    pw_rpe_threshold(config.get(), rpe_fx / std::max(rpe_count, 1.0)));
            ++pw_solver_counters[kPwTrackEvaluated];
            if (!pw_depth_ok)
                ++pw_solver_counters[kPwTrackRejectDepth];
            else if (!is_valid)
                ++pw_solver_counters[kPwTrackRejectRpe];
            pw_account_rpe(rpe, rpe_count, pw_depth_ok, is_valid,
                           kPwTrackRpeSamples, kPwTrackRpeMilliPx);
            track->tag(TT_VALID) = is_valid;
        } else {
            track->landmark.inv_depth = -1.0;
        }
    }

    for (size_t k = 0; k < map->track_num(); ++k) {
        Track *track = map->get_track(k);
        if (!track->tag(TT_VALID))
            track->tag(TT_TRASH) = true;
    }
}

void SlidingWindowTracker::slide_window() {
    while (map->frame_num() > config->sliding_window_size()) {
        Frame *frame = map->get_frame(0);
        for (size_t i = 0; i < frame->subframes.size(); ++i) {
            map->untrack_frame(frame->subframes[i].get());
        }
        map->marginalize_frame(0);
    }
}

void SlidingWindowTracker::refine_subwindow() {
    Frame *frame = map->get_frame(map->frame_num() - 1);
    if (frame->subframes.empty())
        return;
    if (frame->subframes[0]->tag(FT_NO_TRANSLATION)) {
        if (frame->subframes.size() >= 9) {
            for (size_t i = frame->subframes.size() / 3; i > 0; --i) {
                Frame *tgt_frame = frame->subframes[i * 3 - 1].get();
                std::vector<ImuData> imu_data;
                for (size_t j = i * 3 - 1; j > (i - 1) * 3; --j) {
                    Frame *src_frame = frame->subframes[j - 1].get();
                    imu_data.insert(imu_data.begin(),
                                    src_frame->preintegration.data.begin(),
                                    src_frame->preintegration.data.end());
                    map->untrack_frame(src_frame);
                    frame->subframes.erase(frame->subframes.begin() + (j - 1));
                }
                tgt_frame->preintegration.data.insert(
                    tgt_frame->preintegration.data.begin(), imu_data.begin(),
                    imu_data.end());
            }
        }

        auto solver = Solver::create();
        frame->tag(FT_FIX_POSE) = true;
        frame->tag(FT_FIX_MOTION) = true;

        solver->add_frame_states(frame);
        for (size_t i = 0; i < frame->subframes.size(); ++i) {
            Frame *subframe = frame->subframes[i].get();
            solver->add_frame_states(subframe);
            Frame *prev_frame =
                (i == 0 ? frame : frame->subframes[i - 1].get());
            subframe->preintegration.integrate(
                subframe->image->t, prev_frame->motion.bg,
                prev_frame->motion.ba, true, true);
            solver->put_factor(Solver::create_preintegration_error_factor(
                prev_frame, subframe, subframe->preintegration));
        }

        Frame *last_subframe = frame->subframes.back().get();
        for (size_t k = 0; k < last_subframe->keypoint_num(); ++k) {
            if (Track *track = last_subframe->get_track(k)) {
                if (track->tag(TT_VALID)) {
                    if (track->tag(TT_TRIANGULATED)) {
                        if (track->tag(TT_STATIC))
                            solver->put_factor(
                                Solver::create_reprojection_prior_factor(
                                    last_subframe, track));
                    } else {
                        solver->put_factor(Solver::create_rotation_prior_factor(
                            last_subframe, track));
                    }
                }
            }
        }

        ++pw_solver_counters[kPwSolveCalls];
    if (!solver->solve())
        ++pw_solver_counters[kPwSolveUnusable];
        frame->tag(FT_FIX_POSE) = false;
        frame->tag(FT_FIX_MOTION) = false;
    } else {
        auto solver = Solver::create();
        frame->tag(FT_FIX_POSE) = true;
        frame->tag(FT_FIX_MOTION) = true;
        solver->add_frame_states(frame);
        for (size_t i = 0; i < frame->subframes.size(); ++i) {
            Frame *subframe = frame->subframes[i].get();
            solver->add_frame_states(subframe);
            Frame *prev_frame =
                (i == 0 ? frame : frame->subframes[i - 1].get());
            subframe->preintegration.integrate(
                subframe->image->t, prev_frame->motion.bg,
                prev_frame->motion.ba, true, true);
            solver->put_factor(Solver::create_preintegration_error_factor(
                prev_frame, subframe, subframe->preintegration));
            for (size_t k = 0; k < subframe->keypoint_num(); ++k) {
                if (Track *track = subframe->get_track(k)) {
                    if (track->all_tagged(TT_VALID, TT_TRIANGULATED,
                                          TT_STATIC)) {
                        if (track->first_frame()->tag(FT_KEYFRAME)) {
                            solver->put_factor(
                                Solver::create_reprojection_prior_factor(
                                    subframe, track));
                        } else if (track->first_frame()->id() > frame->id()) {
                            solver->add_factor(
                                frame->reprojection_error_factors[k].get());
                        }
                    }
                }
            }
        }
        ++pw_solver_counters[kPwSolveCalls];
    if (!solver->solve())
        ++pw_solver_counters[kPwSolveUnusable];
        frame->tag(FT_FIX_POSE) = false;
        frame->tag(FT_FIX_MOTION) = false;
    }
} // namespace xrslam

std::tuple<double, PoseState, MotionState>
SlidingWindowTracker::get_latest_state() const {
    const Frame *frame = map->get_frame(map->frame_num() - 1);
    if (!frame->subframes.empty()) {
        frame = frame->subframes.back().get();
    }
    return {frame->image->t, frame->pose, frame->motion};
}

matrix<3> compute_essential_matrix(matrix<3> &R, vector<3> &t) {
    matrix<3> t_ = matrix<3>::Zero();

    t_(0, 1) = -t(2);
    t_(0, 2) = t(1);
    t_(1, 0) = t(2);
    t_(1, 2) = -t(0);
    t_(2, 0) = -t(1);
    t_(2, 1) = t(0);

    matrix<3> E = t_ * R;
    return E;
}

double compute_epipolar_dist(matrix<3> F, vector<2> &pt1, vector<2> &pt2) {
    vector<3> l = F * pt1.homogeneous();
    double dist =
        std::abs(pt2.homogeneous().transpose() * l) / l.segment<2>(0).norm();
    return dist;
}

bool SlidingWindowTracker::check_frames_rpe(Track *track, const vector<3> &x) {
    std::vector<matrix<3, 4>> Ps;
    std::vector<vector<3>> ps;

    bool is_valid = true;
    double rpe = 0.0;
    double rpe_count = 0.0;
    double rpe_fx = 0.0;
    for (const auto &[frame, keypoint_index] : track->keypoint_map()) {
        if (!frame->tag(FT_KEYFRAME))
            continue;
        PoseState pose = frame->get_pose(frame->camera);
        vector<3> y = pose.q.conjugate() * (x - pose.p);
        if (y.z() <= 1.0e-3 || y.z() > 50) { // todo
            is_valid = false;
            break;
        }
        rpe += (apply_k(y, frame->K) -
                apply_k(frame->get_keypoint(keypoint_index), frame->K))
                   .norm();
        rpe_count += 1.0;
        rpe_fx += frame->K(0, 0);
    }
    const bool pw_depth_ok = is_valid;
    is_valid = is_valid && (rpe / std::max(rpe_count, 1.0) <
                            pw_rpe_threshold(config.get(), rpe_fx / std::max(rpe_count, 1.0)));
    ++pw_solver_counters[kPwFramesRpeCalls];
    if (!is_valid)
        ++pw_solver_counters[kPwFramesRpeReject];
    pw_account_rpe(rpe, rpe_count, pw_depth_ok, is_valid,
                   kPwFramesRpeSamples, kPwFramesRpeMilliPx);

    return is_valid;
}

bool SlidingWindowTracker::filter_parsac_2d2d(
    Frame *frame_i, Frame *frame_j, std::vector<char> &mask,
    std::vector<size_t> &pts_to_index) {

    std::vector<vector<2>> pts1, pts2;

    for (size_t ki = 0; ki < frame_i->keypoint_num(); ++ki) {
        if (Track *track = frame_i->get_track(ki)) {
            if (size_t kj = track->get_keypoint_index(frame_j)) {
                if (kj != nil()) {
                    pts1.push_back(frame_i->get_keypoint(ki).hnormalized());
                    pts2.push_back(frame_j->get_keypoint(kj).hnormalized());
                    pts_to_index.push_back(kj);
                }
            }
        }
    }

    if (pts1.size() < 10)
        return false;

    matrix<3> E =
        find_essential_matrix_parsac(pts1, pts2, mask, m_th / frame_i->K(0, 0));

    return true;
}

void SlidingWindowTracker::predict_RT(Frame *frame_i, Frame *frame_j,
                                      matrix<3> &R, vector<3> &t) {

    auto camera = frame_i->camera;
    auto imu = frame_i->imu;

    matrix<4> Pwc = matrix<4>::Identity();
    matrix<4> PwI = matrix<4>::Identity();
    matrix<4> Pwi = matrix<4>::Identity();
    matrix<4> Pwj = matrix<4>::Identity();

    Pwc.block<3, 3>(0, 0) = camera.q_cs.toRotationMatrix();
    Pwc.block<3, 1>(0, 3) = camera.p_cs;
    PwI.block<3, 3>(0, 0) = imu.q_cs.toRotationMatrix();
    PwI.block<3, 1>(0, 3) = imu.p_cs;
    Pwi.block<3, 3>(0, 0) = frame_i->pose.q.toRotationMatrix();
    Pwi.block<3, 1>(0, 3) = frame_i->pose.p;
    Pwj.block<3, 3>(0, 0) = frame_j->pose.q.toRotationMatrix();
    Pwj.block<3, 1>(0, 3) = frame_j->pose.p;

    matrix<4> Pji = Pwj.inverse() * Pwi;

    matrix<4> P = (Pwc.inverse() * PwI * Pji * PwI.inverse() * Pwc);

    R = P.block<3, 3>(0, 0);
    t = P.block(0, 3, 3, 1);
}

bool SlidingWindowTracker::judge_track_status() {

    Frame *curr_frame = map->get_frame(map->frame_num() - 1);
    Frame *keyframe = map->get_frame(map->frame_num() - 2);
    Frame *last_frame = keyframe;
    if (!keyframe->subframes.empty()) {
        last_frame = keyframe->subframes.back().get();
    }

    curr_frame->preintegration.integrate(curr_frame->image->t,
                                         last_frame->motion.bg,
                                         last_frame->motion.ba, true, true);
    curr_frame->preintegration.predict(last_frame, curr_frame);

    m_P2D.clear();
    m_P3D.clear();
    m_lens.clear();
    m_indices_map = std::vector<int>(curr_frame->keypoint_num(), -1);

    for (size_t k = 0; k < curr_frame->keypoint_num(); ++k) {
        if (Track *track = curr_frame->get_track(k)) {
            if (track->all_tagged(TT_VALID, TT_TRIANGULATED)) {
                const vector<3> &bearing = curr_frame->get_keypoint(k);
                const vector<3> &landmark = track->get_landmark_point();
                m_P2D.push_back(bearing.hnormalized());
                m_P3D.push_back(landmark);
                m_lens.push_back(std::max(track->m_life, size_t(0)));
                m_indices_map[k] = m_P3D.size() - 1;
            }
        }
    }

    if (m_P2D.size() < 20)
        return false;

    const PoseState &pose = curr_frame->get_pose(curr_frame->camera);

    std::vector<char> mask;
    matrix<3> Rcw = pose.q.inverse().toRotationMatrix();
    vector<3> tcw = pose.q.inverse() * pose.p * (-1.0);
    matrix<4> T_IMU =
        find_pnp_matrix_parsac_imu(m_P3D, m_P2D, m_lens, Rcw, tcw, 0.20, 1.0,
                                   mask, 1.0 / curr_frame->K(0, 0));

    matrix<3> R;
    vector<3> t;
    predict_RT(keyframe, curr_frame, R, t);

    // check rpe
    {
        std::vector<vector<2>> P2D_inliers, P2D_outliers;
        std::vector<vector<3>> P3D_inliers, P3D_outliers;

        for (int i = 0; i < m_P2D.size(); ++i) {
            if (mask[i]) {
                P2D_inliers.push_back(m_P2D[i]);
                P3D_inliers.push_back(m_P3D[i]);
            } else {
                P2D_outliers.push_back(m_P2D[i]);
                P3D_outliers.push_back(m_P3D[i]);
            }
        }

        std::vector<double> inlier_errs, outlier_errs;
        double inlier_errs_sum = 0, outlier_errs_sum = 0;
        for (int i = 0; i < P2D_inliers.size(); i++) {
            vector<3> p = pose.q.conjugate() * (P3D_inliers[i] - pose.p);
            double proj_err =
                (apply_k(p, curr_frame->K) -
                 apply_k(P2D_inliers[i].homogeneous(), curr_frame->K))
                    .norm();
            inlier_errs.push_back(proj_err);
            inlier_errs_sum += proj_err;
        }

        for (int i = 0; i < P2D_outliers.size(); i++) {
            vector<3> p = pose.q.conjugate() * (P3D_outliers[i] - pose.p);
            double proj_err =
                (apply_k(p, curr_frame->K) -
                 apply_k(P2D_outliers[i].homogeneous(), curr_frame->K))
                    .norm();
            outlier_errs.push_back(proj_err);
            outlier_errs_sum += proj_err;
        }
    }

    matrix<3> E = compute_essential_matrix(R, t);
    matrix<3> F =
        keyframe->K.transpose().inverse() * E * curr_frame->K.inverse();

    std::vector<vector<2>> inlier_set1, inlier_set2;
    std::vector<vector<2>> outlier_set1, outlier_set2;
    for (size_t i = 0; i < curr_frame->keypoint_num(); ++i) {
        if (m_indices_map[i] != -1) {
            if (size_t j =
                    curr_frame->get_track(i)->get_keypoint_index(keyframe);
                j != nil()) {
                if (mask[m_indices_map[i]]) {
                    inlier_set1.push_back(
                        apply_k(keyframe->get_keypoint(j), keyframe->K));
                    inlier_set2.push_back(
                        apply_k(curr_frame->get_keypoint(i), curr_frame->K));
                } else {
                    outlier_set1.push_back(
                        apply_k(keyframe->get_keypoint(j), keyframe->K));
                    outlier_set2.push_back(
                        apply_k(curr_frame->get_keypoint(i), curr_frame->K));
                }
            }
        }
    }

    std::vector<double> inliers_dist, outliers_dist;

    for (int i = 0; i < inlier_set1.size(); i++) {
        vector<2> &p1 = inlier_set1[i];
        vector<2> &p2 = inlier_set2[i];
        double err = compute_epipolar_dist(F, p1, p2) +
                     compute_epipolar_dist(F.transpose(), p2, p1);
        inliers_dist.push_back(err);
    }

    for (int i = 0; i < outlier_set1.size(); i++) {
        vector<2> &p1 = outlier_set1[i];
        vector<2> &p2 = outlier_set2[i];
        double err = compute_epipolar_dist(F, p1, p2) +
                     compute_epipolar_dist(F.transpose(), p2, p1);
        outliers_dist.push_back(err);
    }

    size_t min_num = 20;
    if (inliers_dist.size() < min_num || outliers_dist.size() < min_num)
        return false;

    std::sort(inliers_dist.begin(), inliers_dist.end());
    std::sort(outliers_dist.begin(), outliers_dist.end());

    double th1 = inliers_dist[size_t(inliers_dist.size() * 0.5)];
    double th2 = outliers_dist[size_t(outliers_dist.size() * 0.5)];

    if (th2 < th1 * 2) // mean there is ambiguity
        return false;

    m_th = (th1 + th2) / 2;

    for (size_t k = 0; k < curr_frame->keypoint_num(); ++k) {
        if (Track *track = curr_frame->get_track(k)) {
            // track->tag(TT_STATIC) = true;
            if (m_indices_map[k] != -1) {
                if (mask[m_indices_map[k]]) {
                    curr_frame->get_track(k)->tag(TT_OUTLIER) = false;
                    curr_frame->get_track(k)->tag(TT_STATIC) = true;
                } else {
                    curr_frame->get_track(k)->tag(TT_OUTLIER) = true;
                    curr_frame->get_track(k)->tag(TT_STATIC) = false;
                }
            }
        }
    }

    return true;
}

void SlidingWindowTracker::update_track_status() {

    Frame *curr_frame = map->get_frame(map->frame_num() - 1);
    size_t frame_id = feature_tracking_map->frame_index_by_id(curr_frame->id());

    if (frame_id == nil())
        return;

    Frame *old_frame = feature_tracking_map->get_frame(frame_id);

    std::vector<size_t> outlier_cnts(curr_frame->keypoint_num(), 0);
    std::vector<size_t> matches_cnts(curr_frame->keypoint_num(), 0);
    size_t start_idx = std::min(
        map->frame_num() - 1,
        std::max(map->frame_num() - 1 - config->parsac_keyframe_check_size(),
                 size_t(0)));
    for (size_t i = start_idx; i < map->frame_num() - 1; i++) {
        std::vector<char> mask;
        std::vector<size_t> pts_to_index;
        if (filter_parsac_2d2d(map->get_frame(i), curr_frame, mask,
                               pts_to_index)) {
            for (size_t j = 0; j < mask.size(); j++) {
                if (!mask[j]) {
                    outlier_cnts[pts_to_index[j]] += 1;
                }
                matches_cnts[pts_to_index[j]] += 1;
            }
        }
    }

    for (size_t i = 0; i < curr_frame->keypoint_num(); i++) {
        if (Track *curr_track = curr_frame->get_track(i)) {
            if (size_t j = curr_track->get_keypoint_index(old_frame)) {
                if (j != nil()) {
                    Track *old_track = old_frame->get_track(j);
                    size_t outlier_th = map->frame_num() / 2;
                    if (outlier_cnts[i] > outlier_th / 2 &&
                        outlier_cnts[i] > 0.8 * matches_cnts[i]) {
                        curr_track->tag(TT_STATIC) = false;
                    }
                    if (!old_track->tag(TT_STATIC) ||
                        !curr_track->tag(TT_STATIC)) {
                        curr_track->tag(TT_STATIC) = false;
                        old_track->tag(TT_STATIC) = false;
                    }
                }
            }
        }
    }
}


// =========================================================================================
// [pw 2026-09-23] Short-term tracking-loss recovery. Recipe, sources and every threshold:
// sliding_window_tracker.h. Reached only when Config::tracking_recovery_enable() is true.
// =========================================================================================

namespace {
// Same block as the tail of track(): publishes the window landmarks to the debug inspector.
void pw_publish_landmarks(Map *map) {
    inspect_debug(sliding_window_landmarks, landmarks) {
        std::vector<Landmark> points;
        points.reserve(map->track_num());
        for (size_t i = 0; i < map->track_num(); ++i) {
            if (Track *track = map->get_track(i)) {
                if (track->tag(TT_VALID)) {
                    Landmark point;
                    point.p = track->get_landmark_point();
                    point.triangulated = track->tag(TT_TRIANGULATED);
                    points.push_back(point);
                }
            }
        }
        landmarks = std::move(points);
    }
}
} // namespace

bool SlidingWindowTracker::pw_track_with_recovery() {
    // Same prologue as track().
    if (config->parsac_flag()) {
        if (judge_track_status()) {
            update_track_status();
        }
    }

    Frame *frame_j = map->get_frame(map->frame_num() - 1);
    const double t = frame_j->image->t;
    const size_t lose_below = config->tracking_recovery_min_tracked_landmarks();
    const size_t recover_above = config->tracking_recovery_recover_tracked_landmarks_above();
    const bool long_term_reset = config->tracking_recovery_long_term_reset();

    bool searched = false;
    if (pw_state == PwRecoveryState::SHORT_TERM_LOST) {
        if (t - pw_t_lost > config->tracking_recovery_lost_timeout()) {
            // [S1] Tracking.cc:1993-1997: RECENTLY_LOST -> LOST after 5 s.
            if (long_term_reset) {
                // [P3][S5]: LOST => new map.
                log_info("[pw-recovery] t=%.6f short-term lost for %.3f s > %.3f s: "
                         "long-term lost, map reset",
                         t, t - pw_t_lost, config->tracking_recovery_lost_timeout());
                return false;
            }
            // Reset off (default): stop searching, carry on as the engine always did.
            log_info("[pw-recovery] t=%.6f short-term lost for %.3f s > %.3f s: giving up the "
                     "search, continuing without reset",
                     t, t - pw_t_lost, config->tracking_recovery_lost_timeout());
            pw_state = PwRecoveryState::OK;
            pw_snapshot.clear();
            pw_gave_up = true;
        } else {
            // [S6] Tracking.cc:1981-1990 + :2122-2126: RECENTLY_LOST in VI mode =
            // PredictStateIMU() (here: mirror_frame()'s per-frame prediction from the previous
            // frame) and then TrackLocalMap(): search the local map with the lost-state window,
            // then the pose-inertial optimisation (here: localize_newframe()).
            std::vector<size_t> refound;
            pw_search_map_points(frame_j, refound);
            searched = true;
            localize_newframe();
            size_t rejected = 0;
            if (!refound.empty()) {
                rejected = pw_reject_refound_outliers(frame_j, refound);
                if (rejected > 0)
                    localize_newframe(); // re-solve without the rejected matches
            }
            const size_t tracked = pw_count_tracked_landmarks(frame_j);
            // [S2'] Tracking.cc:3033: while RECENTLY_LOST, TrackLocalMap succeeds with > 10
            // inliers; :2142 then sets OK.
            if (tracked > recover_above) {
                size_t retained = 0;
                for (size_t k = 0; k < frame_j->keypoint_num(); ++k) {
                    Track *track = frame_j->get_track(k);
                    if (!track || !track->all_tagged(TT_VALID, TT_TRIANGULATED, TT_STATIC))
                        continue;
                    for (const PwSnapshotPoint &point : pw_snapshot)
                        if (point.track_id == track->id()) {
                            ++retained;
                            break;
                        }
                }
                log_info("[pw-recovery] t=%.6f RECOVERED after %.3f s: %zu tracked map points "
                         "(> %zu), %zu of them pre-loss (re-found this frame %zu, rejected %zu)",
                         t, t - pw_t_lost, tracked, recover_above, retained,
                         refound.size() - rejected, rejected);
                pw_state = PwRecoveryState::OK;
                pw_snapshot.clear();
            }
        }
    }

    if (!searched) {
        // Tracking OK: the ordinary per-frame pose solve ([P5]).
        localize_newframe();
        const size_t tracked = pw_count_tracked_landmarks(frame_j);
        if (tracked >= lose_below)
            pw_gave_up = false;
        // [P1][S2] Tracking.cc:3039: VI-monocular TrackLocalMap fails with < 15 inliers
        // (IMU initialised); :2144-2162 OK -> RECENTLY_LOST.
        if (tracked < lose_below && !pw_gave_up) {
            if (t - pw_t_init < config->tracking_recovery_min_map_age()) {
                if (long_term_reset) {
                    // [P4][S3] Tracking.cc:2148-2153
                    log_info("[pw-recovery] t=%.6f lost (%zu tracked < %zu) %.3f s after "
                             "initialisation (< %.3f s): map discarded, reset",
                             t, tracked, lose_below, t - pw_t_init,
                             config->tracking_recovery_min_map_age());
                    return false;
                }
                // Reset off: the paper never searches a map this young (it discards it);
                // the frame goes through the pre-existing path only.
            } else {
                pw_take_snapshot();
                pw_state = PwRecoveryState::SHORT_TERM_LOST;
                pw_t_lost = t;
                log_info("[pw-recovery] t=%.6f LOST: %zu tracked map points < %zu; short-term "
                         "lost, %zu retained map points to search",
                         t, tracked, lose_below, pw_snapshot.size());
            }
        }
    }

    // [S7] Tracking.cc:2205-2250: keyframes keep being inserted while RECENTLY_LOST
    // (IMU-monocular NeedNewKeyFrame c4 :3182, mInsertKFsLost default true :1327), so the
    // IMU chain stays a sequence of per-keyframe factors in the local inertial BA. Here that
    // is the unchanged rest of track(): this tracker's own keyframe decision, triangulation,
    // window optimisation and slide -- in every state.
    if (manage_keyframe()) {
        track_landmark();
        refine_window();
        slide_window();
    } else {
        refine_subwindow();
    }
    if (pw_state == PwRecoveryState::OK)
        pw_pin_reference(frame_j);
    pw_publish_landmarks(map.get());
    return true;
}

size_t SlidingWindowTracker::pw_count_tracked_landmarks(Frame *frame) const {
    // "Map points tracked" in the current frame = its keypoints whose track is
    // TT_VALID+TT_TRIANGULATED+TT_STATIC: exactly the set localize_newframe() uses as fixed
    // map points ([P5]). TT_VALID is this tracker's own inlier decision (the mean-rpe gate
    // in refine_window()), i.e. the analogue of ORB-SLAM3's inlier count after pose
    // optimisation ([S2] "mnMatchesInliers"). It includes points created after the loss,
    // as ORB-SLAM3's local map does.
    size_t n = 0;
    for (size_t k = 0; k < frame->keypoint_num(); ++k) {
        Track *track = frame->get_track(k);
        if (track && track->all_tagged(TT_VALID, TT_TRIANGULATED, TT_STATIC))
            ++n;
    }
    return n;
}

size_t SlidingWindowTracker::pw_reject_refound_outliers(Frame *frame,
                                                        const std::vector<size_t> &refound) {
    // A re-found match is kept only if its track would still pass this tracker's own inlier
    // gate with the new observation added: refine_window()'s TT_VALID test, i.e. depth window
    // (1e-3, 50] in every keyframe observation and mean reprojection error over the keyframe
    // observations < pw_rpe_threshold(), here with this frame (at the pose just solved)
    // counted as one more keyframe -- which is what it becomes on recovery. Mirrors
    // ORB-SLAM3 discarding the outliers of its pose optimisation, with this tracker's own
    // criterion (a single-frame 3 px test was tried first and is far stricter than anything
    // the tracker applies in normal operation, where ~20% of per-frame errors exceed 3 px on
    // run-6e2d4b99). Rejected matches are detached from the retained track.
    Frame *ft_frame = nullptr;
    size_t rejected = 0;
    synchronized(detail->feature_tracker->map) {
        const size_t ft_index = feature_tracking_map
                                    ? feature_tracking_map->frame_index_by_id(frame->id())
                                    : nil();
        if (ft_index != nil())
            ft_frame = feature_tracking_map->get_frame(ft_index);
        for (size_t k : refound) {
            Track *track = frame->get_track(k);
            if (!track)
                continue;
            const vector<3> x = track->get_landmark_point();
            bool ok = true;
            double rpe = 0.0, rpe_count = 0.0, rpe_fx = 0.0;
            for (const auto &[f, index] : track->keypoint_map()) {
                if (f != frame && !f->tag(FT_KEYFRAME))
                    continue;
                const PoseState pose = f->get_pose(f->camera);
                const vector<3> y = pose.q.conjugate() * (x - pose.p);
                if (y.z() <= 1.0e-3 || y.z() > 50) {
                    ok = false;
                    break;
                }
                rpe += (apply_k(y, f->K) - apply_k(f->get_keypoint(index), f->K)).norm();
                rpe_count += 1.0;
                rpe_fx += f->K(0, 0);
            }
            ok = ok && rpe_count > 0.0 &&
                 rpe / rpe_count < pw_rpe_threshold(config.get(), rpe_fx / rpe_count);
            if (ok)
                continue;
            track->remove_keypoint(frame, false);
            if (ft_frame && k < ft_frame->keypoint_num())
                if (Track *ft_track = ft_frame->get_track(k))
                    ft_track->tag(TT_PW_RECOVERED) = false;
            ++rejected;
        }
    }
    return rejected;
}

void SlidingWindowTracker::pw_pin_reference(Frame *frame) {
    if (!detail)
        return;
    synchronized(detail->feature_tracker->map) {
        detail->feature_tracker->pw_pin_reference_image(frame->image);
    }
    pw_reference_image = frame->image;
    pw_reference_frame_id = frame->id();
    pw_pinned = true;
}

bool SlidingWindowTracker::pw_take_snapshot() {
    pw_snapshot.clear();
    if (!pw_reference_image || pw_reference_frame_id == nil())
        return false;
    // The reference is the last frame tracked OK; find it among the keyframes and their
    // subframes (normally it is the frame right before the one that just got lost).
    Frame *reference = nullptr;
    for (size_t i = map->frame_num(); i-- > 0 && !reference;) {
        Frame *keyframe = map->get_frame(i);
        if (keyframe->id() == pw_reference_frame_id) {
            reference = keyframe;
            break;
        }
        for (size_t s = keyframe->subframes.size(); s-- > 0;) {
            if (keyframe->subframes[s]->id() == pw_reference_frame_id) {
                reference = keyframe->subframes[s].get();
                break;
            }
        }
    }
    if (!reference || reference->image != pw_reference_image)
        return false;
    for (size_t k = 0; k < reference->keypoint_num(); ++k) {
        Track *track = reference->get_track(k);
        if (!track || !track->all_tagged(TT_VALID, TT_TRIANGULATED, TT_STATIC))
            continue;
        pw_snapshot.push_back(
            {track->id(), apply_k(reference->get_keypoint(k), reference->K)});
    }
    return true;
}

size_t SlidingWindowTracker::pw_search_map_points(Frame *frame,
                                                  std::vector<size_t> &appended) {
    if (!detail || pw_snapshot.empty() || !pw_reference_image)
        return 0;
    size_t found = 0;
    // Held for the whole search: the feature tracker releases frame images under this lock,
    // and the re-found keypoints are also appended to its copy of this frame.
    synchronized(detail->feature_tracker->map) {
        if (pw_reference_image->width() == 0 || !frame->image ||
            frame->image->width() == 0)
            return 0; // pixels already released (only possible with threading on)

        const double width = double(frame->image->width());
        const double height = double(frame->image->height());
        double radius = config->tracking_recovery_search_radius_px();
        const double reference_focal = config->tracking_recovery_search_reference_focal();
        if (reference_focal > 0.0)
            radius *= frame->K(0, 0) / reference_focal;

        const PoseState camera = frame->get_pose(frame->camera);
        std::vector<Track *> candidates;
        std::vector<vector<2>> reference_px, predicted_px;
        for (const PwSnapshotPoint &point : pw_snapshot) {
            Track *track = map->get_track_by_id(point.track_id);
            if (!track || !track->all_tagged(TT_VALID, TT_TRIANGULATED, TT_STATIC))
                continue;
            if (track->get_keypoint_index(frame) != nil())
                continue; // already tracked into this frame
            const vector<3> y =
                camera.q.conjugate() * (track->get_landmark_point() - camera.p);
            if (y.z() <= 1.0e-3 || y.z() > 50) // same depth window as refine_window()
                continue;
            const vector<2> u = apply_k(y, frame->K);
            // same 20 px border as the LK tracker (opencv_image.cpp track_keypoints)
            if (u.x() < 20 || u.y() < 20 || u.x() >= width - 20 || u.y() >= height - 20)
                continue;
            candidates.push_back(track);
            reference_px.push_back(point.ref_px);
            predicted_px.push_back(u);
        }
        if (candidates.empty())
            return 0;

        // The tracker's own matcher, started at the projection (OPTFLOW_USE_INITIAL_FLOW).
        std::vector<vector<2>> matched_px = predicted_px;
        std::vector<char> status;
        pw_reference_image->track_keypoints_guided(frame->image.get(), reference_px,
                                                   matched_px, status, radius);

        Frame *ft_frame = nullptr;
        const size_t ft_index = feature_tracking_map
                                    ? feature_tracking_map->frame_index_by_id(frame->id())
                                    : nil();
        if (ft_index != nil()) {
            ft_frame = feature_tracking_map->get_frame(ft_index);
            if (ft_frame->keypoint_num() != frame->keypoint_num())
                ft_frame = nullptr; // indices would not line up; only the backend gets them
        }
        for (size_t i = 0; i < candidates.size(); ++i) {
            if (i >= status.size() || !status[i])
                continue;
            if ((matched_px[i] - predicted_px[i]).norm() > radius)
                continue;
            const vector<3> bearing = remove_k(matched_px[i], frame->K);
            const size_t k = frame->keypoint_num();
            frame->append_keypoint(bearing);
            candidates[i]->add_keypoint(frame, k);
            appended.push_back(k);
            if (ft_frame) {
                ft_frame->append_keypoint(bearing);
                ft_frame->get_track(k, feature_tracking_map.get())->tag(TT_PW_RECOVERED) =
                    true;
            }
            ++found;
        }
    }
    return found;
}

} // namespace xrslam
