#ifndef OPT_ANALYTIC_H_RVDYMIZJ
#define OPT_ANALYTIC_H_RVDYMIZJ

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

class OptAnalytic : public IOptimizationTraits<OptAnalytic>
{
public:

    ////////////////////////////////////////////////////////////////////////////
    // this Optimizer uses *relative* poses, i.e. T_CK
    // but the optimization interface expects global poses (T_WC)
    // so we need the absolute position of the keyframe as well

    // single keyframe
    OptAnalytic(const OptParams& params);

    ////////////////////////////////////////////////////////////////////////////

    template <typename Scalar, typename MotionScalar>
    Scalar error(
        Motion<Scalar> T_ref_cam,
        Motion<MotionScalar> V_cam,
        boost::optional<OptVisualizationParams> viz_params,
        Scalar residual[]=nullptr) const;

    ////////////////////////////////////////////////////////////////////////////

    size_t residual_size() const
    {
        return params.eventframe.img.size().area();
    }

    ////////////////////////////////////////////////////////////////////////////

private:
    cv::Mat event_frame_grad;
};

////////////////////////////////////////////////////////////////////////////////

#endif /* end of include guard: OPT_ANALYTIC_H_RVDYMIZJ */
