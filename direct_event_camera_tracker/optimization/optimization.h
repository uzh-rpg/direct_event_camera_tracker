#ifndef OPT_PARAMS_H_9N8JPA6I
#define OPT_PARAMS_H_9N8JPA6I

#include <boost/optional.hpp>
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>
#include <ceres/gradient_checker.h>

#include "utils/ceres.h"
#include "opengl/world_renderer.h"
#include "keyframe.h"
#include "event_buffer.h"
#include "pyramid.h"
#include "utils.h"
#include "utils/profiler.h"


// TODO: it would probably make sense to encapsulate optimization variables
// into a new object

typedef Eigen::Matrix<double,6,1> Vector6d;

////////////////////////////////////////////////////////////////////////////////

struct OptVisualizationParams
{
    // TODO: don't use references to matrices but instead use optional<OptVisParams&>
    AUTO_STRUCT(OptVisualizationParams,
            ATTR(cv::Mat&, projected_intensity),
            ATTR(cv::Mat&, projected_gradient),
            ATTR(cv::Mat&, projected_flow),
            ATTR(cv::Mat&, exp_intensity),
            ATTR(cv::Mat&, correlation)
    );

    void init(cv::Size size);
};

////////////////////////////////////////////////////////////////////////////////

struct OptParams
{
    enum Method {
        FORWARD_WARP = 0,
        RERENDER     = 1,
        ANALYTIC     = 2,
        DEBUG
    };

    typedef boost::optional<boost::function<void(const Statef&)>> OptCB_t;

    AUTO_STRUCT(OptParams,
            ATTR(const Eventframe&, eventframe),
            ATTR(const Keyframe&,   keyframe),
            ATTR(const ImagePyramidLevelConfig&,
                                    pyramid_lvl),
            ATTR(WorldRenderer&,    world_renderer),
            ATTR(Method,            method),
            ATTR(const Statef,      initial_state),
            ATTR(OptCB_t,           live_callback, {}),
            ATTR(bool,              use_numeric_diff, false)
    );
};

////////////////////////////////////////////////////////////////////////////////

class IOptimization
{
public:
    IOptimization(const OptParams& params)
        : params(params) {};

    virtual ~IOptimization() {};

    virtual size_t residual_size() const = 0;

    virtual ceres::LocalParameterization* make_pos_parametrization();
    virtual ceres::LocalParameterization* make_vel_parametrization();

    // for visualization in GUI
    virtual double error(const Statef& state, boost::optional<OptVisualizationParams> viz_params) = 0;

    Statef run();

    void check_gradient();

    virtual ceres::CostFunction* make_cost_function() = 0;

    virtual void   init_optimization  (Vector6d& T_ref_cam, Vector6d& V_cam);
    virtual Statef finish_optimization(Vector6d& T_ref_cam, Vector6d& V_cam);

    static std::unique_ptr<IOptimization> create(const OptParams& params);

    const OptParams& getParams() const { return params; }

    template <typename Scalar>
    Eigen::Transform<Scalar, 3, Eigen::Isometry> T_world_cam_from_T_ref_cam(const Eigen::Matrix<Scalar,6,1> T_ref_cam) const
    {
        Eigen::Transform<Scalar, 3, Eigen::Isometry> T_WC = T_world_ref.T().cast<Scalar>();
        T_WC *= Sophus::SE3<Scalar>::exp(T_ref_cam).matrix();
        return T_WC;
    }

protected:
    OptParams params;

    // reference pose
    Posef T_world_ref;
};

////////////////////////////////////////////////////////////////////////////////

// some useful helper functions to reduce boiler-plate code
// this contains stuff that requires the actual implementation type and
// therefore cannot be directly implemented in IOptimization
// (not so sure if this is really the cleanest way to implement this, but at
// least it saves us from having to implement this in every derived class)
template <class T>
class IOptimizationTraits : public IOptimization
{
public:

    IOptimizationTraits(const OptParams& params)
        : IOptimization(params) {};

    ////////////////////////////////////////////////////////////////////////////

    // build ceres cost function
    virtual ceres::CostFunction* make_cost_function() {
        T& obj = * ((T*) this);
        if (obj.params.use_numeric_diff) {
            std::cout << "WARNING: Employing numerical differentiation!" << std::endl;
            ceres::NumericDiffOptions opts;
            opts.relative_step_size = 0.0005;
            opts.ridders_relative_initial_step_size = 0.0005;

            if (obj.params.pyramid_lvl.fix_velocity) {
                return new ceres::NumericDiffCostFunction<T, ceres::CENTRAL, ceres::DYNAMIC, 6>
                    (&obj, ceres::DO_NOT_TAKE_OWNERSHIP, obj.residual_size(), opts);
            } else {
                return new ceres::NumericDiffCostFunction<T, ceres::CENTRAL, ceres::DYNAMIC, 6, 6>
                    (&obj, ceres::DO_NOT_TAKE_OWNERSHIP, obj.residual_size(), opts);
            }
        } else {
            if (obj.params.pyramid_lvl.fix_velocity) {
                return new ceres::AutoDiffCostFunction<T, ceres::DYNAMIC, 6>   (&obj, obj.residual_size());
            } else {
                return new ceres::AutoDiffCostFunction<T, ceres::DYNAMIC, 6, 6>(&obj, obj.residual_size());
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////////

    template <typename Scalar>
    bool operator()(const Scalar* const p, const Scalar* const v, Scalar* residual) const
    {
        // p is 6 dimensional!

        Motion<Scalar> pose_rel(p);
        Motion<Scalar> motion(v);

        T& obj = * ((T*) this);
        obj.error(pose_rel, motion, {}, residual);

        return true;
    }

    ////////////////////////////////////////////////////////////////////////////

    template <typename Scalar>
    bool operator()(const Scalar* const p, Scalar* residual) const
    {
        Motion<Scalar> pose_rel(p);

        T& obj = * ((T*) this);
        obj.error(pose_rel, params.initial_state.motion, {}, residual);

        return true;
    }

    ////////////////////////////////////////////////////////////////////////////

    // this function is called from the GUI for visualization and rendering the error plot
    // TODO: calculate the relative twist between T_world_cam and T_world_ref and pass this to error()
    double error(const Statef& T_world_cam, boost::optional<OptVisualizationParams> viz_params)
    {
        T& obj = * ((T*) this);

        Posef orig_T_world_ref = T_world_ref;
        T_world_ref = T_world_cam.pose;

        double err = obj.template error<double,double>(Motionf(), T_world_cam.motion, viz_params);

        T_world_ref = orig_T_world_ref;
        return err;
    }
};

////////////////////////////////////////////////////////////////////////////////

struct OptimizationCallback : public ceres::IterationCallback
{
    AUTO_STRUCT(OptimizationCallback,
            ATTR(const IOptimization&, opt),
            ATTR(const Vector6d&, T_ref_cam),
            ATTR(const Vector6d&, V_cam),
            ATTR(boost::function<void(const Statef&)>, callback)
    );

    ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary);
};

////////////////////////////////////////////////////////////////////////////////

// instantiate templates, so we don't have to define the template in the header
// This allows for faster recompilation, as we don't have to change headers
// when editing the error calculation.  As a nice side-effect, we can prevent
// wrong instantations (i.e. only allow double and Jet as template parameters).

#define INSTANTIATE_TEMPLATE(T, T1, T2) \
    template T1 T::error<T1, T2>(Motion<T1>, Motion<T2>, boost::optional<OptVisualizationParams>, T1*) const;

// we need these as we cannot pass a parameter containing a comma to a macro
typedef ceres::Jet<double,6>  Jet6;
typedef ceres::Jet<double,12> Jet12;

#define INSTANTIATE_TEMPLATES(T) \
    INSTANTIATE_TEMPLATE(T, double, double); \
    INSTANTIATE_TEMPLATE(T, Jet6, double); \
    INSTANTIATE_TEMPLATE(T, Jet12, Jet12);

////////////////////////////////////////////////////////////////////////////////


#endif /* end of include guard: OPT_PARAMS_H_9N8JPA6I */
