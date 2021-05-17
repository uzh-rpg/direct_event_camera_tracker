#include "opt_debug.h"

////////////////////////////////////////////////////////////////////////////////

OptDebug::OptDebug(const OptParams& params)
    : IOptimizationTraits(params)
{
}

////////////////////////////////////////////////////////////////////////////////

template <typename Scalar, typename MotionScalar>
Scalar OptDebug::error(
    Motion<Scalar> T_ref_cam,
    Motion<MotionScalar> V_cam,
    boost::optional<OptVisualizationParams> viz_params,
    Scalar residual[]) const
{
    UNUSED(V_cam);

    // init output images
    if (viz_params) {
        viz_params->init(params.eventframe.img.size());
    }

    if (residual) {
        Eigen::Matrix<Scalar,3,1> p = T_ref_cam.velocity;
        for (size_t i = 0; i < residual_size(); i++) {
            Scalar x = p.x() - Scalar(1);
            Scalar y = p.y();
            //Scalar z = p.z() - Scalar(-1.53);

            residual[i] = x*x + Scalar(10)*y;

            /*
            // to verify the jacobians we get are correct
            residual[i] =
                  Scalar( 1 + i*100)*T_WC.position.x()
                + Scalar( 2 + i*100)*T_WC.position.y()
                + Scalar( 3 + i*100)*T_WC.position.z()
                + Scalar( 4 + i*100)*T_WC.orientation.x()
                + Scalar( 5 + i*100)*T_WC.orientation.y()
                + Scalar( 6 + i*100)*T_WC.orientation.z()
                + Scalar( 7 + i*100)*T_WC.orientation.w()
                + Scalar( 8 + i*100)*m.velocity.x()
                + Scalar( 9 + i*100)*m.velocity.y()
                + Scalar(10 + i*100)*m.velocity.z()
                + Scalar(11 + i*100)*m.rotation.x()
                + Scalar(12 + i*100)*m.rotation.y()
                + Scalar(13 + i*100)*m.rotation.z();
                */
        }
    }

    return Scalar(123);
}

////////////////////////////////////////////////////////////////////////////////

INSTANTIATE_TEMPLATES(OptDebug);

////////////////////////////////////////////////////////////////////////////////

