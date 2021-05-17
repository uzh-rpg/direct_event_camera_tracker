#ifndef OPTIMIZATION_RERENDER_H_GKE0OV2R
#define OPTIMIZATION_RERENDER_H_GKE0OV2R

#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>
#include <ceres/cubic_interpolation.h>
#include "sophus/se3.hpp"

#include "point.h"
#include "keyframe.h"
#include "event_buffer.h"
#include "core/camera_intrinsics.h"
#include "pyramid.h"
#include "opengl/world_renderer.h"
#include "utils/ceres.h"
#include "utils/profiler.h"

#include "optimization/optimization.h"

////////////////////////////////////////////////////////////////////////////////

class KFRenderingJacobian : public ceres::SizedCostFunction<2, 6>
{
    const Keyframe& KF;
public:
    KFRenderingJacobian(const Keyframe& KF) : KF(KF) {};

    // we're only ever using this class with integer image coordinates
    Eigen::Vector2i pos;

    bool Evaluate(const double* const* params, double* residuals, double** jacobians) const;
};

////////////////////////////////////////////////////////////////////////////////
// same as Optimization class, but instead of working with a keyframe a new
// frame is rendered on every iteration

class OptRerender : public IOptimizationTraits<OptRerender>
{
    ////////////////////////////////////////////////////////////////////////////////

    Keyframe renderKeyframe(const Posef& T_WC) const;

    ////////////////////////////////////////////////////////////////////////////////

public:

    ////////////////////////////////////////////////////////////////////////////

    // generate a new keyframe on every iteration step
    OptRerender(const OptParams& params);

    ////////////////////////////////////////////////////////////////////////////

    template <typename Scalar, typename MotionScalar>
    Scalar error(
        Motion<Scalar> twist,
        Motion<MotionScalar> motion,
        boost::optional<OptVisualizationParams> viz_params,
        Scalar residual[]=nullptr) const;

    ////////////////////////////////////////////////////////////////////////////

    size_t residual_size() const
    {
        return params.eventframe.img.size().area();
    }

    ////////////////////////////////////////////////////////////////////////////////

private:
    // calculated from subset fraction:
    // TODO
    //size_t subset_count; // actual number of points (subset_fraction * keyframe.size().area())
    //double subset_grad_thresh; // threshold so that subset_count points have norm(grad) >= thresh
};

////////////////////////////////////////////////////////////////////////////////

#endif /* end of include guard: OPTIMIZATION_RERENDER_H_GKE0OV2R */
