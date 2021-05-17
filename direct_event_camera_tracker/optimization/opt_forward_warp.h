#ifndef OPTIMIZATION_H_JDNCVIN3
#define OPTIMIZATION_H_JDNCVIN3

#include <ceres/cubic_interpolation.h>

#include "point.h"
#include "keyframe.h"
#include "event_buffer.h"
#include "core/camera_intrinsics.h"
#include "pyramid.h"
#include "opengl/world_renderer.h"
#include "utils/ceres.h"
#include "optimization/optimization.h"

#include "utils/profiler.h"

////////////////////////////////////////////////////////////////////////////////

class OptFwdWarp : public IOptimizationTraits<OptFwdWarp>
{
public:

    ////////////////////////////////////////////////////////////////////////////
    // this Optimizer uses *relative* poses, i.e. T_CK
    // but the optimization interface expects global poses (T_WC)
    // so we need the absolute position of the keyframe as well

    // single keyframe
    OptFwdWarp(const OptParams& params);

    ////////////////////////////////////////////////////////////////////////////

    template <typename Scalar, typename MotionScalar>
    Scalar error(
        Motion<Scalar> T_ref_cam,
        Motion<MotionScalar> V_cam,
        boost::optional<OptVisualizationParams> viz_params,
        Scalar residual[]=nullptr) const;

    ////////////////////////////////////////////////////////////////////////////

    // [0 to 1] how many points were transformed to valid points (i.e. lying inside the keyframe)
    double get_overlap_fraction(const Posef& T_CK);

    size_t residual_size() const { return subset_count; }

private:

    // calculated from subset fraction:
    size_t subset_count;
    double subset_grad_thresh; // threshold so that subset_count points have norm(grad) >= thresh

    // image samplers (required to actually get a derivative)
    ceres::Grid2D<double> event_image_ceres;
    ceres::BiCubicInterpolator<ceres::Grid2D<double>> event_image_interpolator;

    ceres::Grid2D<double,2> kf_grad_ceres;
    ceres::BiCubicInterpolator<ceres::Grid2D<double,2>> kf_grad_interpolater;

    ceres::Grid2D<double> kf_depth_ceres;
    ceres::BiCubicInterpolator<ceres::Grid2D<double>> kf_depth_interpolater;
};

////////////////////////////////////////////////////////////////////////////////

#endif /* end of include guard: OPTIMIZATION_H_JDNCVIN3 */
