#include <atomic>
#include <ceres/ceres.h>
#include <chrono>
#include <xrslam/estimation/ceres/marginalization_factor.h>
#include <xrslam/estimation/ceres/okvis_iteration_callback.h>
#include <xrslam/estimation/ceres/preintegration_factor.h>
#include <xrslam/estimation/ceres/quaternion_parameterization.h>
#include <xrslam/estimation/ceres/reprojection_factor.h>
#include <xrslam/estimation/ceres/rotation_factor.h>
#include <xrslam/estimation/solver.h>
#include <xrslam/estimation/state.h>
#include <xrslam/map/frame.h>

// [pw 2026-09-23 solver time budget] Read-only telemetry, one set per process.
// Filled from the Ceres summary AFTER each Solve() returns; nothing here is read
// back by the estimator, so trajectories do not depend on it. Exported with C
// linkage so a replay harness can dlsym() it and take per-frame deltas
// (pw_tools/regression/euroc_runner.cpp --timing-csv). "scoped" = solves inside a
// Solver::FrameBudgetScope (the per-frame sliding-window solves), "unscoped" =
// everything else (initializer). Times are Ceres' own total_time_in_seconds.
extern "C" {
std::atomic<unsigned long long> pw_solver_scoped_ns{0};
std::atomic<unsigned long long> pw_solver_scoped_calls{0};
std::atomic<unsigned long long> pw_solver_scoped_iterations{0};
std::atomic<unsigned long long> pw_solver_unscoped_ns{0};
std::atomic<unsigned long long> pw_solver_unscoped_calls{0};
// how scoped solves ended
std::atomic<unsigned long long> pw_solver_stop_budget{0};     // OKVIS callback (USER_SUCCESS)
std::atomic<unsigned long long> pw_solver_stop_time_limit{0}; // max_solver_time_in_seconds
std::atomic<unsigned long long> pw_solver_stop_iter_limit{0}; // max_num_iterations
std::atomic<unsigned long long> pw_solver_stop_converged{0};  // CONVERGENCE
}

namespace xrslam {

namespace {
thread_local const Solver::FrameBudgetScope *tl_frame_budget_scope = nullptr;

double pw_steady_seconds() {
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}
} // namespace

Solver::FrameBudgetScope::FrameBudgetScope()
    : outer(tl_frame_budget_scope), t0_seconds(pw_steady_seconds()) {
    tl_frame_budget_scope = this;
}

Solver::FrameBudgetScope::~FrameBudgetScope() {
    tl_frame_budget_scope = outer;
}

struct Solver::SolverDetails {
    static Config *&config() {
        static Config *s_config = nullptr;
        return s_config;
    }
    std::unique_ptr<ceres::Problem> problem;
    std::unique_ptr<ceres::LossFunction> cauchy_loss;
    std::unique_ptr<ceres::LocalParameterization> quaternion_parameterization;
    std::vector<std::unique_ptr<ReprojectionErrorFactor>> managed_rpefactors;
    std::vector<std::unique_ptr<ReprojectionPriorFactor>> managed_rppfactors;
    std::vector<std::unique_ptr<RotationPriorFactor>> managed_ropfactors;
    std::vector<std::unique_ptr<PreIntegrationErrorFactor>> managed_piefactors;
    std::vector<std::unique_ptr<PreIntegrationPriorFactor>> managed_pipfactors;
    std::vector<std::unique_ptr<MarginalizationFactor>> managed_marfactors;
};

Solver::Solver() : details(std::make_unique<SolverDetails>()) {
    ceres::Problem::Options problem_options;
    problem_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    problem_options.local_parameterization_ownership =
        ceres::DO_NOT_TAKE_OWNERSHIP;
    details->problem = std::make_unique<ceres::Problem>(problem_options);
    details->cauchy_loss = std::make_unique<ceres::CauchyLoss>(
        1.0); // TODO(jinyu): make configurable
    details->quaternion_parameterization =
        std::make_unique<QuaternionParameterization>();
}

Solver::~Solver() = default;

void Solver::init(Config *config) { SolverDetails::config() = config; }

std::unique_ptr<Solver> Solver::create() {
    return std::unique_ptr<Solver>(new Solver());
}

std::unique_ptr<ReprojectionErrorFactor>
Solver::create_reprojection_error_factor(Frame *frame, Track *track) {
    return std::make_unique<CeresReprojectionErrorFactor>(frame, track);
}

std::unique_ptr<ReprojectionPriorFactor>
Solver::create_reprojection_prior_factor(Frame *frame, Track *track) {
    return std::make_unique<CeresReprojectionPriorFactor>(frame, track);
}

std::unique_ptr<RotationPriorFactor>
Solver::create_rotation_prior_factor(Frame *frame, Track *track) {
    return std::make_unique<CeresRotationPriorFactor>(frame, track);
}

std::unique_ptr<PreIntegrationErrorFactor>
Solver::create_preintegration_error_factor(
    Frame *frame_i, Frame *frame_j, const PreIntegrator &preintegration) {
    return std::make_unique<CeresPreIntegrationErrorFactor>(frame_i, frame_j,
                                                            preintegration);
}

std::unique_ptr<PreIntegrationPriorFactor>
Solver::create_preintegration_prior_factor(
    Frame *frame_i, Frame *frame_j, const PreIntegrator &preintegration) {
    return std::make_unique<CeresPreIntegrationPriorFactor>(frame_i, frame_j,
                                                            preintegration);
}

std::unique_ptr<MarginalizationFactor>
Solver::create_marginalization_factor(Map *map) {
    return std::make_unique<CeresMarginalizationFactor>(map);
}

void Solver::add_frame_states(Frame *frame, bool with_motion) {
    details->problem->AddParameterBlock(
        frame->pose.q.coeffs().data(), 4,
        details->quaternion_parameterization.get());
    details->problem->AddParameterBlock(frame->pose.p.data(), 3);
    if (frame->tag(FT_FIX_POSE)) {
        details->problem->SetParameterBlockConstant(
            frame->pose.q.coeffs().data());
        details->problem->SetParameterBlockConstant(frame->pose.p.data());
    }
    if (with_motion) {
        details->problem->AddParameterBlock(frame->motion.v.data(), 3);
        details->problem->AddParameterBlock(frame->motion.bg.data(), 3);
        details->problem->AddParameterBlock(frame->motion.ba.data(), 3);
        if (frame->tag(FT_FIX_MOTION)) {
            details->problem->SetParameterBlockConstant(frame->motion.v.data());
            details->problem->SetParameterBlockConstant(
                frame->motion.bg.data());
            details->problem->SetParameterBlockConstant(
                frame->motion.ba.data());
        }
    }
}

void Solver::add_track_states(Track *track) {
    details->problem->AddParameterBlock(&(track->landmark.inv_depth), 1);
}

void Solver::add_factor(ReprojectionErrorFactor *rpefactor) {
    CeresReprojectionErrorFactor *rpecost =
        static_cast<CeresReprojectionErrorFactor *>(rpefactor);
    details->problem->AddResidualBlock(
        rpecost, details->cauchy_loss.get(),
        rpecost->frame->pose.q.coeffs().data(), rpecost->frame->pose.p.data(),
        rpecost->track->first_frame()->pose.q.coeffs().data(),
        rpecost->track->first_frame()->pose.p.data(),
        &(rpecost->track->landmark.inv_depth));
}

void Solver::add_factor(ReprojectionPriorFactor *rppfactor) {
    CeresReprojectionPriorFactor *rppcost =
        static_cast<CeresReprojectionPriorFactor *>(rppfactor);
    details->problem->AddResidualBlock(
        rppcost, details->cauchy_loss.get(),
        rppcost->rpefactor.frame->pose.q.coeffs().data(),
        rppcost->rpefactor.frame->pose.p.data());
}

void Solver::add_factor(RotationPriorFactor *ropfactor) {
    CeresRotationPriorFactor *ropcost =
        static_cast<CeresRotationPriorFactor *>(ropfactor);
    details->problem->AddResidualBlock(ropcost, details->cauchy_loss.get(),
                                       ropcost->frame->pose.q.coeffs().data());
}

void Solver::add_factor(PreIntegrationErrorFactor *piefactor) {
    CeresPreIntegrationErrorFactor *piecost =
        static_cast<CeresPreIntegrationErrorFactor *>(piefactor);
    details->problem->AddResidualBlock(
        piecost, nullptr, piecost->frame_i->pose.q.coeffs().data(),
        piecost->frame_i->pose.p.data(), piecost->frame_i->motion.v.data(),
        piecost->frame_i->motion.bg.data(), piecost->frame_i->motion.ba.data(),
        piecost->frame_j->pose.q.coeffs().data(),
        piecost->frame_j->pose.p.data(), piecost->frame_j->motion.v.data(),
        piecost->frame_j->motion.bg.data(), piecost->frame_j->motion.ba.data());
}

void Solver::add_factor(PreIntegrationPriorFactor *pipfactor) {
    CeresPreIntegrationPriorFactor *pipcost =
        static_cast<CeresPreIntegrationPriorFactor *>(pipfactor);
    details->problem->AddResidualBlock(
        pipcost, nullptr, pipcost->piefactor.frame_j->pose.q.coeffs().data(),
        pipcost->piefactor.frame_j->pose.p.data(),
        pipcost->piefactor.frame_j->motion.v.data(),
        pipcost->piefactor.frame_j->motion.bg.data(),
        pipcost->piefactor.frame_j->motion.ba.data());
}

void Solver::add_factor(MarginalizationFactor *factor) {
    std::vector<double *> params;
    for (size_t i = 0; i < factor->linearization_frames().size(); ++i) {
        Frame *frame = factor->linearization_frames()[i];
        params.emplace_back(frame->pose.q.coeffs().data());
        params.emplace_back(frame->pose.p.data());
        params.emplace_back(frame->motion.v.data());
        params.emplace_back(frame->motion.bg.data());
        params.emplace_back(frame->motion.ba.data());
    }
    details->problem->AddResidualBlock(
        static_cast<CeresMarginalizationFactor *>(factor), nullptr, params);
}

bool Solver::solve(bool verbose) {
    ceres::Solver::Options solver_options;
    ceres::Solver::Summary solver_summary;
    solver_options.linear_solver_type = ceres::SPARSE_SCHUR;
    solver_options.trust_region_strategy_type = ceres::DOGLEG;
    solver_options.max_num_iterations =
        (int)details->config()->solver_iteration_limit();
    solver_options.max_solver_time_in_seconds =
        details->config()->solver_time_limit();
    solver_options.num_threads = 1;
    solver_options.minimizer_progress_to_stdout = verbose;
    solver_options.update_state_every_iteration = true;

    // [pw 2026-09-23 solver time budget] OKVIS per-frame budget.
    // Off (budget < 0, the default) => no callback, options identical to before.
    // On  => remaining = budget - (now - frame start), clamped at 0, exactly as
    //        OKVIS ThreadedKFVio.cpp:527-530:
    //          double timeLimit = timeLimitForMatchingAndOptimization
    //                             - (okvis::Time::now() - t0Matching).toSec();
    //          setOptimizationTimeLimit(std::max<double>(0.0, timeLimit),
    //                                   min_iterations);
    //        and the callback is registered the way Estimator.cpp:920-923 does
    //        (pushed onto the ceres options' callbacks).
    //        Our "now" is taken after the problem was built, so problem
    //        construction is charged to the frame as well.
    const FrameBudgetScope *scope = tl_frame_budget_scope;
    std::unique_ptr<OkvisIterationCallback> budget_callback;
    const double frame_budget = details->config()->solver_frame_time_budget();
    if (scope && frame_budget >= 0.0) {
        const double remaining =
            frame_budget - (pw_steady_seconds() - scope->t0_seconds);
        budget_callback = std::make_unique<OkvisIterationCallback>(
            std::max<double>(0.0, remaining),
            (int)details->config()->solver_min_iterations());
        solver_options.callbacks.push_back(budget_callback.get());
    }

    ceres::Solve(solver_options, details->problem.get(), &solver_summary);

    // telemetry only (see top of file); nothing below feeds the estimator.
    const auto ns = (unsigned long long)(solver_summary.total_time_in_seconds * 1e9);
    if (scope) {
        pw_solver_scoped_ns.fetch_add(ns, std::memory_order_relaxed);
        pw_solver_scoped_calls.fetch_add(1, std::memory_order_relaxed);
        pw_solver_scoped_iterations.fetch_add(
            (unsigned long long)(solver_summary.num_successful_steps +
                                 solver_summary.num_unsuccessful_steps),
            std::memory_order_relaxed);
        // Ceres 1.14 (the pinned build): a callback returning
        // SOLVER_TERMINATE_SUCCESSFULLY ends as USER_SUCCESS (minimizer.cc:69-72);
        // both hard limits end as NO_CONVERGENCE and differ only in the message,
        // "Maximum solver time reached. ..." (trust_region_minimizer.cc:609) vs
        // "Maximum number of iterations reached. ..." (:627).
        switch (solver_summary.termination_type) {
        case ceres::USER_SUCCESS:
            pw_solver_stop_budget.fetch_add(1, std::memory_order_relaxed);
            break;
        case ceres::CONVERGENCE:
            pw_solver_stop_converged.fetch_add(1, std::memory_order_relaxed);
            break;
        case ceres::NO_CONVERGENCE:
            if (solver_summary.message.compare(0, 19, "Maximum solver time") == 0)
                pw_solver_stop_time_limit.fetch_add(1, std::memory_order_relaxed);
            else
                pw_solver_stop_iter_limit.fetch_add(1, std::memory_order_relaxed);
            break;
        default:
            break;
        }
    } else {
        pw_solver_unscoped_ns.fetch_add(ns, std::memory_order_relaxed);
        pw_solver_unscoped_calls.fetch_add(1, std::memory_order_relaxed);
    }
    return solver_summary.IsSolutionUsable();
}

void Solver::manage_factor(std::unique_ptr<ReprojectionErrorFactor> &&factor) {
    details->managed_rpefactors.emplace_back(std::move(factor));
}

void Solver::manage_factor(std::unique_ptr<ReprojectionPriorFactor> &&factor) {
    details->managed_rppfactors.emplace_back(std::move(factor));
}

void Solver::manage_factor(std::unique_ptr<RotationPriorFactor> &&factor) {
    details->managed_ropfactors.emplace_back(std::move(factor));
}

void Solver::manage_factor(
    std::unique_ptr<PreIntegrationErrorFactor> &&factor) {
    details->managed_piefactors.emplace_back(std::move(factor));
}

void Solver::manage_factor(
    std::unique_ptr<PreIntegrationPriorFactor> &&factor) {
    details->managed_pipfactors.emplace_back(std::move(factor));
}

void Solver::manage_factor(std::unique_ptr<MarginalizationFactor> &&factor) {
    details->managed_marfactors.emplace_back(std::move(factor));
}

} // namespace xrslam
