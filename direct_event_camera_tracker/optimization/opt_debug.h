#ifndef OPT_DEBUG_H_PZWGLVL7
#define OPT_DEBUG_H_PZWGLVL7

#include <ceres/cubic_interpolation.h>

#include "point.h"
#include "keyframe.h"
#include "event_buffer.h"
#include "core/camera_intrinsics.h"
#include "pyramid.h"
#include "opengl/world_renderer.h"
#include "utils/ceres.h"
#include "optimization/optimization.h"

constexpr size_t OPT_DBG_RESIDUAL_SIZE = 11;

class OptDebug : public IOptimizationTraits<OptDebug>
{
public:

    ////////////////////////////////////////////////////////////////////////////

    OptDebug(const OptParams& params);

    ////////////////////////////////////////////////////////////////////////////

    template <typename Scalar, typename MotionScalar>
    Scalar error(
        Motion<Scalar> T_ref_cam,
        Motion<MotionScalar> V_cam,
        boost::optional<OptVisualizationParams> viz_params,
        Scalar residual[]=nullptr) const;

    ////////////////////////////////////////////////////////////////////////////

    size_t residual_size() const { return OPT_DBG_RESIDUAL_SIZE; }
};

#endif /* end of include guard: OPT_DEBUG_H_PZWGLVL7 */
