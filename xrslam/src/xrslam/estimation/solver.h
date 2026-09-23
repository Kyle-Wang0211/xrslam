#ifndef XRSLAM_SOLVER_H
#define XRSLAM_SOLVER_H

#include <xrslam/common.h>
#include <xrslam/estimation/marginalization_factor.h>
#include <xrslam/estimation/preintegration_factor.h>
#include <xrslam/estimation/reprojection_factor.h>
#include <xrslam/estimation/rotation_factor.h>

namespace xrslam {

class Map;
class Frame;
class Track;

class Solver {
    struct SolverDetails;
    Solver();

  public:
    virtual ~Solver();

    static void init(Config *config);

    static std::unique_ptr<Solver> create();

    static std::unique_ptr<ReprojectionErrorFactor>
    create_reprojection_error_factor(Frame *frame, Track *track);
    static std::unique_ptr<ReprojectionPriorFactor>
    create_reprojection_prior_factor(Frame *frame, Track *track);
    static std::unique_ptr<RotationPriorFactor>
    create_rotation_prior_factor(Frame *frame, Track *track);
    static std::unique_ptr<PreIntegrationErrorFactor>
    create_preintegration_error_factor(Frame *frame_i, Frame *frame_j,
                                       const PreIntegrator &preintegration);
    static std::unique_ptr<PreIntegrationPriorFactor>
    create_preintegration_prior_factor(Frame *frame_i, Frame *frame_j,
                                       const PreIntegrator &preintegration);
    static std::unique_ptr<MarginalizationFactor>
    create_marginalization_factor(Map *map);

    virtual void add_frame_states(Frame *frame, bool with_motion = true);
    virtual void add_track_states(Track *track);

    virtual void add_factor(ReprojectionErrorFactor *rpecost);
    virtual void add_factor(ReprojectionPriorFactor *rppcost);
    virtual void add_factor(RotationPriorFactor *ropcost);
    virtual void add_factor(PreIntegrationErrorFactor *piecost);
    virtual void add_factor(PreIntegrationPriorFactor *pipcost);
    virtual void add_factor(MarginalizationFactor *marcost);

    template <typename T> void put_factor(std::unique_ptr<T> &&factor) {
        add_factor(factor.get());
        manage_factor(std::move(factor));
    }

    virtual bool solve(bool verbose = false);

    // [pw 2026-09-23 solver time budget] Marks "one frame" for the OKVIS
    // per-frame budget (Config::solver_frame_time_budget()). Construct it at the
    // start of the per-frame estimation; every solve() on the same thread while it
    // is alive shares the frame's budget, the way OKVIS measures from
    // `t0Matching` (ThreadedKFVio.cpp:506, taken just before addStates()) and
    // hands the remainder to the optimiser (ThreadedKFVio.cpp:527-530).
    // Solves outside any scope (the initializer) are never budgeted.
    // Deviation from OKVIS, on purpose: OKVIS's window starts before addStates()
    // and so also pays for descriptor matching (ThreadedKFVio.cpp:506-523); the
    // XRSLAM counterpart of matching is the KLT FeatureTracker, which runs before
    // SlidingWindowTracker::track() (its own thread when threading is ON), so it
    // is NOT charged here. The same number is therefore a looser budget here.
    // thread_local, so it is also correct with XRSLAM_ENABLE_THREADING, where the
    // sliding window runs on the frontend worker thread.
    // With the budget off (< 0, the default) the scope is inert: it records a
    // start time and nothing ever reads it.
    class FrameBudgetScope {
      public:
        FrameBudgetScope();
        ~FrameBudgetScope();
        FrameBudgetScope(const FrameBudgetScope &) = delete;
        FrameBudgetScope &operator=(const FrameBudgetScope &) = delete;

      private:
        const FrameBudgetScope *outer;
        double t0_seconds;
        friend class Solver;
    };

  protected:
    virtual void
    manage_factor(std::unique_ptr<ReprojectionErrorFactor> &&factor);
    virtual void
    manage_factor(std::unique_ptr<ReprojectionPriorFactor> &&factor);
    virtual void manage_factor(std::unique_ptr<RotationPriorFactor> &&factor);
    virtual void
    manage_factor(std::unique_ptr<PreIntegrationErrorFactor> &&factor);
    virtual void
    manage_factor(std::unique_ptr<PreIntegrationPriorFactor> &&factor);
    virtual void manage_factor(std::unique_ptr<MarginalizationFactor> &&factor);
    std::unique_ptr<SolverDetails> details;
};

} // namespace xrslam

#endif // XRSLAM_SOLVER_H
