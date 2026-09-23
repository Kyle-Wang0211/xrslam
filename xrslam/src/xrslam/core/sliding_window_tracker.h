#ifndef XRSLAM_SLIDING_WINDOW_TRACKER_H
#define XRSLAM_SLIDING_WINDOW_TRACKER_H

#include <xrslam/common.h>
#include <xrslam/estimation/state.h>

namespace xrslam {

class Config;
class Frame;
class Map;

class SlidingWindowTracker {
  public:
    SlidingWindowTracker(std::unique_ptr<Map> keyframe_map,
                         std::shared_ptr<Config> config);
    ~SlidingWindowTracker();

    void mirror_frame(Map *feature_tracking_map, size_t frame_id);

    void localize_newframe();
    void track_landmark();
    void refine_window();
    void slide_window();
    bool manage_keyframe();

    void refine_subwindow();

    bool judge_track_status();
    void fuse_imu_track();
    bool check_frames_rpe(Track *track, const vector<3> &p);
    void predict_RT(Frame *frame_i, Frame *frame_j, matrix<3> &R, vector<3> &t);
    bool filter_parsac_2d2d(Frame *frame_i, Frame *frame_j,
                            std::vector<char> &mask,
                            std::vector<size_t> &pts_to_index);
    void update_track_status();

    std::vector<vector<3>> m_P3D;
    std::vector<vector<2>> m_P2D;
    std::vector<size_t> m_lens;
    std::vector<int> m_indices_map;

    bool track();

    std::tuple<double, PoseState, MotionState> get_latest_state() const;

    double m_th;
    std::unique_ptr<Map> map;
    std::shared_ptr<Map> feature_tracking_map;

    void set_detail(XRSLAM::Detail *detail) { this->detail = detail; }

    // =====================================================================================
    // [pw 2026-09-23] Short-term tracking-loss recovery ("A-level"), OFF by default
    // (Config::tracking_recovery_enable). With it off none of the members below is touched
    // and track() is the pre-existing code, byte for byte.
    //
    // WHAT WAS THERE BEFORE. track() returns true unconditionally (it always has, upstream
    // included), so the reset branch in frontend_worker.cpp (latest_state cleared + new
    // Initializer) is unreachable: XRSLAM never declares loss. With no images, frames keep
    // becoming keyframes (mapped landmarks < force_keyframe_landmarks), the window slides
    // the pre-loss keyframes out within sliding_window_size frames, the pose is carried by
    // the IMU factors alone, and when images return tracking restarts on brand-new tracks.
    // The drift accumulated in the gap is kept for good; old map points are never
    // re-associated.
    //
    // PUBLISHED RECIPE REPLICATED (Campos, Elvira, Gomez Rodriguez, Montiel, Tardos,
    // "ORB-SLAM3", IEEE T-RO 37(6) 2021; read from arXiv 2007.11898v2. The paper was read;
    // the GPL-3.0 source was only read to see numbers the paper does not give, cited as
    // "src" below at commit 4452a3c4 -- no code was copied):
    //   [P1] Sec. V-D p.8: "Our visual-inertial system enters into visually lost state when
    //        less than 15 point maps are tracked"
    //   [P2] Sec. V-D p.8, short-term lost: "the current body state is estimated from IMU
    //        readings, and map points are projected in the estimated camera pose and
    //        searched for matches within a large image window. The resulting matches are
    //        included in visual-inertial optimization. In most cases this allows to recover
    //        visual tracking. Otherwise, after 5 seconds, we pass to the next stage."
    //   [P3] Sec. V-D p.8, long-term lost: "A new visual-inertial map is initialized as
    //        explained above, and it becomes the active map."
    //   [P4] Sec. V-D p.9: "If the system gets lost within 15 seconds after IMU
    //        initialization, the map is discarded."
    //   [P5] Sec. V-C p.8: tracking optimizes "only the states of the last two frames ...
    //        while map points remain fixed" -- i.e. exactly localize_newframe() here.
    //   [S1] src Tracking.cc:48 time_recently_lost(5.0); :1981-1998 RECENTLY_LOST in VI mode
    //        = PredictStateIMU() then LOST once t - t_lost > 5 s.  (= [P2])
    //   [S2] src Tracking.cc:3039 VI-monocular TrackLocalMap fails when inliers < 15 with the
    //        IMU initialized.  (= [P1])  NOTE :3033 accepts > 10 inliers while RECENTLY_LOST.
    //   [S3] src Tracking.cc:2148-2153 on OK -> lost, if the IMU is not yet fully refined
    //        (BA2, done 15 s after IMU init) the active map is reset at once.  (= [P4])
    //   [S4] src Tracking.cc:3410-3411 search factor th = 15 while (RECENTLY_)LOST (normal
    //        VI tracking uses 2, :3399-3401); ORBmatcher.cc:66-72,215-220 radius =
    //        th * RadiusByViewingCos (2.5 px if the viewing-angle cosine > 0.998, else 4.0)
    //        * scale factor of the predicted octave  => 37.5 / 60 px at octave 0.
    //
    // STATE MACHINE AS IMPLEMENTED (each item: source -> here):
    //   * "tracked map points" [P1]: after localize_newframe() ([P5]), the keypoints of the
    //     new frame whose track is TT_VALID+TT_TRIANGULATED+TT_STATIC -- exactly the fixed
    //     map points localize_newframe() uses. TT_VALID is this tracker's own inlier decision
    //     (refine_window()'s mean-rpe gate), the analogue of ORB-SLAM3 counting the matches
    //     that survive its pose optimisation ("mnMatchesInliers" [S2]).
    //     (A stricter per-frame 3 px gate was tried first and rejected on evidence: on
    //     run-6e2d4b99 it dips below 15 in 40 ordinary frames, e.g. 5 of 38 at t0+18.9 s,
    //     while this count never goes below 27 after the first 0.25 s.)
    //   * The loss detector is armed once a frame has been tracked OK (>= 15). It starts
    //     unarmed after (re)initialisation -- right after initialisation the count is 0-12
    //     for ~0.25 s (measured on run-6e2d4b99, 60 fps) while new points are triangulated --
    //     and is disarmed after a search that gave up (below). Our choice, not the paper's.
    //   * OK -> lost when that count < min_tracked_landmarks = 15 [P1][S2]:
    //       - less than min_map_age = 15 s since initialisation [P4][S3]:
    //           long_term_reset on : return false => frontend_worker resets the map.
    //           long_term_reset off: the paper never searches a map this young, so neither
    //                                do we: the frame goes through the pre-existing path.
    //       - fewer retained map points than 15 to search: the search could never reach the
    //         recovery threshold, the frame goes through the pre-existing path.
    //       - otherwise enter SHORT-TERM LOST, t_lost = frame time [P2].
    //   * SHORT-TERM LOST, per frame:
    //       - body state from IMU [P2]: mirror_frame()'s preintegration prediction is the
    //         starting point of the search and of the pose solve.
    //       - a BLIND frame (fewer than 15 keypoints carried into it by the front end, e.g.
    //         black or blurred) is stored as a subframe, never a keyframe, so the window does
    //         not slide and the pre-loss keyframes and their map points stay searchable.
    //         (ORB-SLAM3 inserts keyframes while lost without evicting old points; in a
    //         10-keyframe window inserting them would evict the map within 10 frames.)
    //       - a SIGHTED frame that does not re-find the map goes through the pre-existing
    //         pipeline (keyframes, triangulation of the new features, window slide) -- the
    //         counterpart of ORB-SLAM3 inserting keyframes and creating new points while
    //         lost (src Tracking.cc:2248/:3182) -- and the search goes on for the retained
    //         points still in the window. (First version froze sighted frames too; on the
    //         3 s gap that left 2 s of pure IMU after the images returned and doubled the
    //         post-gap error, so it was changed.)
    //       - projection search [P2]: every retained map point observed in the reference
    //         frame (the last frame that was tracked OK) is projected with the IMU-predicted
    //         pose, then searched with this tracker's own matcher -- the same pyramidal LK
    //         call as the front end (Image::track_keypoints_guided: 21x21, 3 levels, 20 px
    //         border, 0.5 px forward-backward check) started at the projection, from the
    //         point's pixel in the reference image. A match is kept only within
    //         search_radius_px of the projection ("large image window" [P2]; 60 px = [S4]'s
    //         octave-0 value for the non-frontal case; the paper gives no number). That
    //         window replaces the front end's consecutive-frame motion gate (rows/4 from the
    //         source pixel), which measured 115-212 px on the 1 s gap of run-6e2d4b99 and
    //         would reject every candidate. Optional focal scaling as for the rpe gate
    //         (search_reference_focal <= 0: bare pixels).
    //       - matches are attached to the retained tracks and the per-frame pose solve is run
    //         ("included in visual-inertial optimization" [P2]) -- localize_newframe() with
    //         its IMU term spanning the gap from the reference frame (pw_localize_lost_frame;
    //         the one-frame IMU prior from an IMU-predicted previous frame is so tight that
    //         30 re-found points moved the pose by < 0.05 px). Matches whose track then fails
    //         refine_window()'s TT_VALID gate with the new observation counted are detached
    //         and the pose re-solved (ORB-SLAM3 likewise drops its pose-optimisation
    //         outliers). If the count above is
    //         >= min_tracked_landmarks the tracker is back to OK and this frame is made a
    //         keyframe, so the matches also enter the window optimisation (refine_window)
    //         together with the IMU chain of the whole gap. The count may also be reached
    //         by points created after the loss on sighted frames (ORB-SLAM3 likewise returns
    //         to OK on whatever local-map points it tracks); the log line says how many of
    //         the tracked points are pre-loss ones.
    //         The recovery threshold is the paper's single number [P1]; [S2]'s laxer "> 10"
    //         while lost is not used (conservative choice, labelled as such).
    //       - once t - t_lost > lost_timeout = 5 s the short-term stage is over [P2][S1]:
    //           long_term_reset on : return false => reset = "new map" [P3]. There is no
    //                                Atlas here, so the old map is not kept (as for [P4]).
    //           long_term_reset off: leave the lost state and process the frame with the
    //                                pre-existing pipeline, i.e. carry on the way the engine
    //                                always did (IMU + new features, same world frame, no
    //                                jump). Not in the paper; kept as the DEFAULT until the
    //                                product decides, because a reset re-initialises into a
    //                                new world frame. The detector is disarmed until a frame
    //                                is tracked OK again, so it cannot loop.
    //   * Everything not specified by the paper and chosen here (conservative / plumbing):
    //       - the searched set is limited to map points with a pixel patch in the retained
    //         reference image (LK needs one; ORB-SLAM3 has descriptors for every point).
    //       - the reference image is kept by exempting it from the feature tracker's release
    //         (FeatureTracker::pw_pin_reference_image); one image, no copy.
    //       - re-found keypoints are appended to the frame in both maps with identical indices
    //         so the ordinary LK front end keeps tracking them, and their tracks are tagged
    //         TT_PW_RECOVERED so its Poisson-disk thinning does not drop them in favour of
    //         the younger tracks started on the same corners during the gap.
    //   * Not replicated: ORB-SLAM3's Atlas/multi-map (old maps kept and merged later), BoW
    //     relocalisation, keyframe insertion while lost (Tracking.cc:2248, which in ORB-SLAM3
    //     does not evict old points but here would slide them out of the window), and its
    //     ">10 keyframes in map" precondition (Tracking.cc:1966), which has no analogue in a
    //     10-keyframe window.
    //
    // HOW TO TURN IT ON (slam yaml):
    //   tracking_recovery:
    //     enable: true               # short-term stage: [P1][P2][P5]
    //     long_term_reset: false     # true = also the paper's [P3]+[P4] map resets
    //     # optional, defaults shown (all sourced above):
    //     min_tracked_landmarks: 15
    //     lost_timeout: 5.0          # s
    //     min_map_age: 15.0          # s, only used with long_term_reset
    //     search_radius_px: 60.0
    //     search_reference_focal: 0.0
    // =====================================================================================
    enum class PwRecoveryState { OK = 0, SHORT_TERM_LOST = 1 };
    bool pw_is_lost() const { return pw_state == PwRecoveryState::SHORT_TERM_LOST; }

  private:
    bool pw_track_with_recovery();
    size_t pw_count_tracked_landmarks(Frame *frame) const;
    size_t pw_count_linked_keypoints(Frame *frame) const;
    void pw_demote_newframe_to_subframe();
    void pw_pin_reference(Frame *frame);
    bool pw_take_snapshot();
    size_t pw_search_map_points(Frame *frame, std::vector<size_t> &appended);
    size_t pw_reject_refound_outliers(Frame *frame, const std::vector<size_t> &refound);
    void pw_run_keyframe_pipeline();
    void pw_localize_lost_frame(Frame *frame_j);

    struct PwSnapshotPoint {
        size_t track_id;
        vector<2> ref_px;
    };
    PwRecoveryState pw_state = PwRecoveryState::OK;
    double pw_t_init = 0.0;
    double pw_t_lost = 0.0;
    std::shared_ptr<Image> pw_reference_image;
    size_t pw_reference_frame_id = nil();
    size_t pw_anchor_frame_id = nil();
    std::vector<PwSnapshotPoint> pw_snapshot;
    bool pw_pinned = false;
    bool pw_armed = false;

    std::shared_ptr<Config> config;

    XRSLAM::Detail *detail = nullptr;
};

} // namespace xrslam

#endif // XRSLAM_SLIDING_WINDOW_TRACKER_H
