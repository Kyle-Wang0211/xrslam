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
    // the GPL-3.0 source was only read, at commit 4452a3c4, to see what the paper does not
    // say, cited as "src" -- no code was copied):
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
    //        while map points remain fixed" -- i.e. localize_newframe() here.
    //   [S1] src Tracking.cc:48 time_recently_lost(5.0); :1993-1997 RECENTLY_LOST -> LOST
    //        once t - t_lost > 5 s.  (= [P2])
    //   [S2] src Tracking.cc:3039 VI-monocular TrackLocalMap fails with < 15 inliers (IMU
    //        initialised); :2144-2162 OK -> RECENTLY_LOST, t_lost = frame time.  (= [P1])
    //   [S2'] src Tracking.cc:3033 while RECENTLY_LOST, TrackLocalMap succeeds with > 10
    //        inliers; :2142 then sets OK. Lose below 15, recover above 10 -- two numbers.
    //   [S3] src Tracking.cc:2148-2153 on OK -> lost before the IMU is fully refined (BA2,
    //        15 s after IMU init) the active map is reset at once.  (= [P4])
    //   [S4] src Tracking.cc:3410-3411 search factor th = 15 while (RECENTLY_)LOST (normal VI
    //        tracking 2, :3399-3401); ORBmatcher.cc:66-72,215-220 radius = th *
    //        RadiusByViewingCos (2.5 px if viewing-angle cosine > 0.998, else 4.0) * octave
    //        scale => 37.5 / 60 px at octave 0.
    //   [S5] src Tracking.cc:2011-2026 LOST => CreateMapInAtlas() (or reset if <10 KFs).
    //   [S6] src Tracking.cc:1981-1990 RECENTLY_LOST in VI mode: bOK = true after
    //        PredictStateIMU(); :2122-2126 TrackLocalMap() still runs every lost frame.
    //        PredictStateIMU (:1738-1784) chains from the LAST FRAME with that frame's
    //        preintegration, or from the last keyframe with the preintegration since it when
    //        local mapping changed the map. TrackLocalMap's pose-inertial optimisation
    //        (:2982-2991, Optimizer.cc:5082-5086/5285) links the frame to the last frame or
    //        keyframe with a propagated prior -- no special gap-long IMU term.
    //   [S7] src Tracking.cc:2205-2250 keyframes keep being inserted while RECENTLY_LOST
    //        (IMU-monocular NeedNewKeyFrame condition c4 = "mState==RECENTLY_LOST", :3182;
    //        mInsertKFsLost default true, :1327; gated only by local mapping being idle,
    //        :3196-3207); each new keyframe restarts the preintegration from the last
    //        keyframe (:3244). No special hand-over on recovery: the state just returns to OK.
    //   [S8] src Tracking.cc:3584-3600 the searched local map is the covisible keyframes of
    //        the last frame's matches plus the last Nd = 20 temporal keyframes; while lost the
    //        last frame has no matches, so only the temporal keyframes remain -- old points
    //        stay searchable only while their keyframes are among the last 20.
    //
    // STATE MACHINE AS IMPLEMENTED (source -> here):
    //   * "tracked map points" [P1]: after localize_newframe() ([P5]), the keypoints of the
    //     new frame whose track is TT_VALID+TT_TRIANGULATED+TT_STATIC -- the fixed map points
    //     localize_newframe() uses. TT_VALID is this tracker's own inlier decision
    //     (refine_window()'s mean-rpe gate), the analogue of [S2]'s "mnMatchesInliers"; it
    //     counts points created after the loss too, as ORB-SLAM3's local map does. (A
    //     per-frame 3 px gate was tried and rejected: 40 false losses on run-6e2d4b99.)
    //   * OK -> SHORT-TERM LOST when that count < min_tracked_landmarks = 15 [P1][S2]:
    //       - less than min_map_age = 15 s since initialisation [P4][S3]:
    //           long_term_reset on : return false => frontend_worker resets the map.
    //           long_term_reset off: nothing (the paper never searches a map this young).
    //       - otherwise t_lost = frame time; snapshot the map points observed in the last
    //         frame tracked OK (the reference frame, whose image is kept).
    //   * SHORT-TERM LOST, per frame ([S6]):
    //       - body state from IMU: mirror_frame()'s per-frame prediction from the previous
    //         frame (= PredictStateIMU's last-frame branch).
    //       - projection search [P2][S4]: every snapshot point still in the window is
    //         projected with that pose and searched with this tracker's own matcher -- the
    //         front end's pyramidal LK call (Image::track_keypoints_guided: 21x21, 3 levels,
    //         20 px border, 0.5 px forward-backward check), from the point's pixel in the
    //         reference image, started at the projection, kept only within search_radius_px
    //         = 60 of it (the paper gives no number; [S4]). That window replaces the front
    //         end's consecutive-frame gate (rows/4 from the source pixel), which measured
    //         115-212 px on the 1 s gap of run-6e2d4b99 and rejected every candidate.
    //       - matches attached to the retained tracks, localize_newframe() run with them
    //         ([P2] "included in visual-inertial optimization"); matches whose track then
    //         fails refine_window()'s TT_VALID gate with the new observation counted are
    //         detached and the pose re-solved (ORB-SLAM3 drops pose-optimisation outliers).
    //       - count > recover_tracked_landmarks_above = 10 => OK [S2'].
    //       - [S7] the frame then goes through the unchanged keyframe pipeline, as in every
    //         state: lost frames become keyframes by this tracker's own rule, so the IMU chain
    //         stays per-keyframe factors in the window optimisation, and a matched frame's
    //         observations enter refine_window like any other.
    //       - once t - t_lost > lost_timeout = 5 s [S1]:
    //           long_term_reset on : return false => reset = "new map" [P3][S5] (no Atlas
    //                                here, so the old map is not kept).
    //           long_term_reset off (DEFAULT): stop searching and carry on, no world-frame
    //                                jump (the pre-existing behaviour; the product decides).
    //                                Loss is not re-declared until a frame is tracked OK
    //                                again (otherwise a scene that stays untrackable would
    //                                restart the 5 s search forever).
    //   * DEVIATIONS (closest mapping, labelled):
    //       - [S8]'s horizon of 20 temporal keyframes maps onto this tracker's window
    //         (sliding_window_size keyframes): old points stay searchable only while one of
    //         their keyframes is still in the window. With a keyframe per lost frame this is
    //         ~window frames, much shorter in time than ORB-SLAM3 at 20 Hz with keyframes
    //         throttled by its mapping thread [S7] -- a consequence of the architecture, not
    //         a tuned number. There is no mid-window marginalisation here
    //         (Map::marginalize_frame asserts index == 0) to keep old keyframes longer.
    //       - localize_newframe() holds the previous frame fixed, whereas ORB-SLAM3 also
    //         optimises it under a propagated prior [S6]; the joint correction happens in
    //         refine_window() on the next keyframe.
    //       - the searched set is limited to points with a pixel patch in the kept reference
    //         image (LK needs one; ORB-SLAM3 has a descriptor per point).
    //       - re-found keypoints are appended to the frame in both maps with identical indices
    //         so the LK front end keeps tracking them, and their tracks are tagged
    //         TT_PW_RECOVERED so its Poisson-disk thinning keeps them (ORB-SLAM3 instead
    //         re-projects the local map every frame).
    //       - Not replicated: Atlas/multi-map, BoW relocalisation, ">10 keyframes in map"
    //         (Tracking.cc:1966).
    //   * HISTORY: an earlier version froze the window while lost (lost frames stored as
    //     subframes, never keyframes) to keep the old map searchable for the full 5 s. It
    //     re-found the map after 0.5 s and 1 s black gaps but, when re-finding failed, handed
    //     a single ~3 s IMU factor back to the window and doubled the post-gap error on the
    //     3 s gap of run-6e2d4b99 (7.52 -> 15.93 cm); ORB-SLAM3 never builds such a factor
    //     [S7], so the freeze was removed.
    //
    // HOW TO TURN IT ON (slam yaml):
    //   tracking_recovery:
    //     enable: true               # short-term stage: [P1][P2][P5]
    //     long_term_reset: false     # true = also the paper's [P3]+[P4] map resets
    //     # optional, defaults shown (all sourced above):
    //     min_tracked_landmarks: 15          # lose below   [P1][S2]
    //     recover_tracked_landmarks_above: 10  # recover above [S2']
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
    void pw_pin_reference(Frame *frame);
    bool pw_take_snapshot();
    size_t pw_search_map_points(Frame *frame, std::vector<size_t> &appended);
    size_t pw_reject_refound_outliers(Frame *frame, const std::vector<size_t> &refound);

    struct PwSnapshotPoint {
        size_t track_id;
        vector<2> ref_px;
    };
    PwRecoveryState pw_state = PwRecoveryState::OK;
    double pw_t_init = 0.0;
    double pw_t_lost = 0.0;
    std::shared_ptr<Image> pw_reference_image;
    size_t pw_reference_frame_id = nil();
    std::vector<PwSnapshotPoint> pw_snapshot;
    bool pw_pinned = false;
    bool pw_gave_up = false;

    std::shared_ptr<Config> config;

    XRSLAM::Detail *detail = nullptr;
};

} // namespace xrslam

#endif // XRSLAM_SLIDING_WINDOW_TRACKER_H
