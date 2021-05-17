#include "optimization.h"

#include "optimization/opt_forward_warp.h"
#include "optimization/opt_rerender.h"
#include "optimization/opt_analytic.h"
#include "optimization/opt_debug.h"

using namespace std;
using namespace ceres;

////////////////////////////////////////////////////////////////////////////////

void OptVisualizationParams::init(cv::Size size)
{
    projected_intensity = cv::Mat(size, CV_64FC1, cv::Scalar(0));
    projected_gradient  = cv::Mat(size, CV_64FC2, cv::Scalar(0, 0));
    projected_flow      = cv::Mat(size, CV_64FC2, cv::Scalar(0, 0));
    exp_intensity       = cv::Mat(size, CV_64FC1, cv::Scalar(0));
    correlation         = cv::Mat(size, CV_64FC1, cv::Scalar(0));
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

ceres::LocalParameterization* IOptimization::make_pos_parametrization()
{
    // TODO: actually, we need the local parametrization from sophos!
    // https://github.com/strasdat/Sophus/blob/master/test/ceres/local_parameterization_se3.hpp
    return nullptr; // no parametrization required
    /*
    return new ProductParameterization(
            new IdentityParameterization(3),
            new EigenQuaternionParameterization()); // so that we get [x y z w] ordering
            */
}

////////////////////////////////////////////////////////////////////////////////

ceres::LocalParameterization* IOptimization::make_vel_parametrization()
{
    // constrain motion to unit sphere
    // TODO: is local size = 6 correct? shouldn't we have a 5D delta?
    return new AutoDiffLocalParameterization<UnitNormVectorAddition, 6, 6>;
}

////////////////////////////////////////////////////////////////////////////////

void IOptimization::init_optimization(Vector6d& T_ref_cam, Vector6d& V_cam)
{
    // T_ref_cam is relative to T_world_ref
    T_ref_cam.setZero();

    T_world_ref = params.initial_state.pose;

    V_cam = params.initial_state.motion.asVector();
}

////////////////////////////////////////////////////////////////////////////////

Statef IOptimization::finish_optimization(Vector6d& T_ref_cam, Vector6d& V_cam)
{
    return Statef(params.initial_state.stamp, Posef(T_world_cam(T_ref_cam)), Motionf(V_cam));
}

////////////////////////////////////////////////////////////////////////////////

Statef IOptimization::run()
{
    ProfilerTimer __t_total("Optimization::run()");
    ProfilerTimer __t_total_per_iter("Optimization::run() / #Iterations");

    Problem::Options prob_opts;
    prob_opts.cost_function_ownership = DO_NOT_TAKE_OWNERSHIP; // we're passing 'this' instead of a new object

    Problem problem(prob_opts);

    // optimization variables:
    Vector6d T_ref_cam; // these are twist coordinates, relative to reference pose
    Vector6d V_cam;     // linear and angular velocities

    init_optimization(T_ref_cam, V_cam);


    CostFunction* cost_function = make_cost_function();
    LossFunction* loss_function = nullptr;

    if (params.pyramid_lvl.loss_function_scaling > 0) {
        loss_function = new HuberLoss(params.pyramid_lvl.loss_function_scaling);
    }

    if (params.pyramid_lvl.fix_velocity) {
        problem.AddResidualBlock(cost_function, loss_function, T_ref_cam.data());
    } else {
        problem.AddResidualBlock(cost_function, loss_function, T_ref_cam.data(), V_cam.data());
        problem.SetParameterization(V_cam.data(), this->make_vel_parametrization());
    }

    // set local parametrization for quaterionions etc.
    LocalParameterization* pos_param = this->make_pos_parametrization();
    if (pos_param) {
        problem.SetParameterization(T_ref_cam.data(), pos_param);
    }

    Solver::Options solver_opts;
    solver_opts.linear_solver_type = DENSE_QR;
    solver_opts.minimizer_progress_to_stdout = true;
    solver_opts.max_num_iterations = params.pyramid_lvl.max_iterations;

    solver_opts.function_tolerance  = params.pyramid_lvl.solver_function_tolerance;
    solver_opts.gradient_tolerance  = params.pyramid_lvl.solver_gradient_tolerance;
    solver_opts.parameter_tolerance = params.pyramid_lvl.solver_parameter_tolerance;

    // use the power of parallel computing!
    // TODO: this doesn't seem to do anything :(
    solver_opts.num_threads = 8;

    // allow the solver to jump over boulders and hills
    solver_opts.use_nonmonotonic_steps = true;

    OptimizationCallback* cb = nullptr;
    if (params.live_callback) {
        solver_opts.update_state_every_iteration = true;
        cb = new OptimizationCallback(*this, T_ref_cam, V_cam, *params.live_callback);
        solver_opts.callbacks.push_back(cb);
    }

    Solver::Summary summary;

    cout << "staring ceres..." << endl;
    try {
        ProfilerTimer __t_solve("ceres::solve");
        ceres::Solve(solver_opts, &problem, &summary);
    } catch (bad_alloc e) {
        cerr << "solve failed: " << e.what() << endl;
        return Statef();
    }

    // TODO: summary contains much more detailed timing information already!
    __t_total_per_iter.stop(summary.num_successful_steps + summary.num_unsuccessful_steps);

    cout << summary.BriefReport() << endl;
    cout << summary.FullReport() << endl;

    if (cb) {
        delete cb;
    }

    cout << "number of iterations: " << summary.num_successful_steps + summary.num_unsuccessful_steps << endl;
    return finish_optimization(T_ref_cam, V_cam);
}

////////////////////////////////////////////////////////////////////////////////

void IOptimization::check_gradient()
{
    cout << "checking derivation numerically" << endl;

    // optimization variables:
    Vector6d T_ref_cam; // these are twist coordinates, relative to reference pose
    Vector6d V_cam;     // linear and angular velocities

    init_optimization(T_ref_cam, V_cam);

    CostFunction* cost_function = make_cost_function();

    double* parameter_blocks[] = {
        T_ref_cam.data(), V_cam.data()
    };

    std::vector<const LocalParameterization*> parametrization;
    parametrization.push_back(make_pos_parametrization());
    parametrization.push_back(make_vel_parametrization());

    ceres::NumericDiffOptions numeric_diff_options;
    //numeric_diff_options.relative_step_size = 0.0002;
    //numeric_diff_options.ridders_relative_initial_step_size = 0.001;

    ceres::GradientChecker gradient_checker(cost_function, &parametrization, numeric_diff_options);

    ceres::GradientChecker::ProbeResults results;
    if (!gradient_checker.Probe(parameter_blocks, 1e-5, &results)) {
        cout << "Check failed!" << endl;

        std::ofstream check_result("gradient_check_result.txt");
        check_result << results.error_log;
        check_result.close();

        cout << "wrote results to gradient_check_result.txt" << endl;
    } else {
        cout << "Check passed! :)" << endl;
    }
}

////////////////////////////////////////////////////////////////////////////////

std::unique_ptr<IOptimization> IOptimization::create(const OptParams& params)
{
    switch (params.method) {
        case OptParams::Method::FORWARD_WARP:
            return std::make_unique<OptFwdWarp>(params);

        case OptParams::Method::RERENDER:
            return std::make_unique<OptRerender>(params);

        case OptParams::Method::ANALYTIC:
            return std::make_unique<OptAnalytic>(params);

        case OptParams::Method::DEBUG:
            return std::make_unique<OptDebug>(params);

        default:
            throw "Invalid optimization method.";
    }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

ceres::CallbackReturnType OptimizationCallback::operator()(const ceres::IterationSummary& summary)
{
    UNUSED(summary);

    Statef state(opt.getParams().initial_state.stamp, Posef(opt.T_world_cam(T_ref_cam)), Motionf(V_cam));

    callback(state);

    return ceres::SOLVER_CONTINUE;
}

////////////////////////////////////////////////////////////////////////////////
