#include "opt_rerender.h"

using namespace std;
using namespace Eigen;
using namespace ceres;

////////////////////////////////////////////////////////////////////////////////

// TODO: run this through ceres::GradientChecker
bool KFRenderingJacobian::Evaluate(const double* const* params, double* residuals, double** jacobians) const
{
    UNUSED(params);
    //const double* const twist  = params[0];

    /*
    // make sure were inside the bound of the image
    if (pos.x() <  0 || pos.y() < 0) return false;
    if (pos.x() >= KF.gradient.cols || pos.y() >= KF.gradient.rows) return false;
    */

    //Vector2d g = bilinear_interp<Vector2d>(KF.gradient, Vector2d(imgpos[0], imgpos[1]));
    Vector2d g = KF.gradient.at<Vector2d>(pos.y(), pos.x());

    residuals[0] = g.x();
    residuals[1] = g.y();

    if (!jacobians) {
        return true;
    }

    /*
    if (jacobians[0]) {
        // calculate Jacobian with respect to pixel coordinates
        // TODO: do we actually need this? Or can we treat x and y as constants?
        double* J = jacobians[0]; // J is 2 x 2

        Vector4d g2 = KF.gradient2.at<Vector4d>(pos.y(), pos.x());
        // TODO: is this correct?
        // J[0] is d residual[0] / d param[0][0] = d gx / dx = dx*dx I
        // J[1] is d residual[0] / d param[0][1] = d gx / dy = dy*dx I
        J[0] = g2[0]; // dx*dx
        J[1] = g2[1]; // dy*dx
        J[2] = g2[2]; // dx*dy
        J[3] = g2[3]; // dy*dy
    }
    */

    if (jacobians[0]) {
        // calcualte Jacobian with respect to twist
        double* J = jacobians[0]; // J is 2 x 6

        double depth = KF.depth.at<double>(pos.y(), pos.x());

        // normalize pixel coordinates
        Vector2d p = (pos.cast<double>().array() - KF.camera.getPrincipalPoint().array())
            / KF.camera.getFocalLength().array();

        // See Corke's 'Robotics, Vision and Control', page 461
        // this assumes OpenCV's coordinate system convention

        // TODO: we're not using the twist here O.o
        // well, we used it when generating the keyframe which we're sampling here
        J[  0] = (-1/depth);
        J[  1] = 0;
        J[  2] = p.x()/depth;
        J[  3] = p.x()*p.y();
        J[  4] = (-(1 + p.x()*p.x()));
        J[  5] = p.y();

        J[6+0] = 0;
        J[6+1] = (-1/depth);
        J[6+2] = p.y()/depth;
        J[6+3] = (1 + p.y()*p.y());
        J[6+4] = (-p.x()*p.y());
        J[6+5] = (-p.x());

        for (size_t i = 0; i < 6; i++) {
            // un-normalize our derivative
            Vector2d flow = Vector2d(J[i], J[i+6]).array() * KF.camera.getFocalLength().array();

            Vector4d g2 = KF.gradient2.at<Vector4d>(pos.y(), pos.x());

            J[i]   = -flow.dot( g2.head<2>() );
            J[i+6] = -flow.dot( g2.tail<2>() );
        }
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

OptRerender::OptRerender(const OptParams& params)
    : IOptimizationTraits(params)
{
    // TODO
    //subset_grad_thresh = find_threshold(keyframe.gradient, cfg.subset_fraction, subset_count);

    cv::normalize(params.eventframe.img, this->params.eventframe.img, 1, 0, cv::NORM_L2, CV_64FC1);
}

////////////////////////////////////////////////////////////////////////////////

Keyframe OptRerender::renderKeyframe(const Posef& T_WC) const
{
    //cout << "rendering new Keyframe" << endl;
    //cout << T_WC << endl;
    Keyframe kf = params.world_renderer.renderPose(T_WC);

    // TODO: scale and blur image
    while (kf.camera.getCameraWidth() > params.eventframe.img.cols) {
        kf = kf.downsample2();
    }

    if (kf.camera.getCameraSize() != toVec2i(params.eventframe.img.size())) {
        std::cerr << "ERROR: failed to downsample keyframe to same size as event image." << std::endl;
        std::cerr << "       KF is now " << kf.camera.getCameraSize().transpose() << " while event image is " << params.eventframe.img.size << std::endl;
        throw "downsampling failed";
    }

    if (kf.intensity.rows == 0 || kf.gradient.rows == 0) {
        throw "keyframe rendering failed";
    }

    kf.blur(params.pyramid_lvl.getTotalBlur());

    return kf;
}

////////////////////////////////////////////////////////////////////////////////

template <typename Scalar, typename MotionScalar>
Scalar OptRerender::error(
    Motion<Scalar> twist,
    Motion<MotionScalar> motion,
    boost::optional<OptVisualizationParams> viz_params,
    Scalar residual[]) const
{
    AssertItsNotFloat(Scalar);

#define PROF_NAME "OptRerender<"+type_name<Scalar>()+">::"
    ProfilerTimer __t_total(PROF_NAME "error()");

    typedef Eigen::Matrix<Scalar,2,1> Vec2;

    Eigen::Matrix<double,6,1> twist_f = CeresCaster::castToDVect(twist);

    Eigen::Isometry3d T_world_cam = T_world_ref.T();
    T_world_cam *= Sophus::SE3d::exp(twist_f).matrix();


    const size_t residual_size = params.eventframe.img.size().area();

    Scalar E_pred_norm = Scalar(0);
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> E_pred(residual_size);

    ProfilerTimer __t_rendering(PROF_NAME "error() rendering");
    // generate a new keyframe based on our current position
    Keyframe kf = renderKeyframe(T_world_cam);
    __t_rendering.stop();

    // set up image samplers (required to actually get a derivative)
    ceres::Grid2D<double> event_image_ceres((double*)params.eventframe.img.data, 0, params.eventframe.img.rows, 0, params.eventframe.img.cols);
    ceres::BiCubicInterpolator<ceres::Grid2D<double>> event_image_interpolator(event_image_ceres);

    KFRenderingJacobian* kf_jac = new KFRenderingJacobian(kf);
    ceres::CostFunctionToFunctor<2, 6> keyframe_sampler(kf_jac);

    // init output images
    if (viz_params) {
        viz_params->init(kf.intensity.size());
        // as we're not warping anything the "projected" image is just our keyframe ;)
        viz_params->projected_intensity = kf.intensity;
        viz_params->projected_gradient  = kf.gradient;
    }

    Scalar twist_arr[6];
    twist.toArray(twist_arr);

    ProfilerTimer __t_innerloop(PROF_NAME "error() inner loop");

    // calculate optical flow and expected intensity change image
    size_t residual_idx = 0;
    for (size_t y = 0; y < params.eventframe.img.rows; y++) {
        for (size_t x = 0; x < params.eventframe.img.cols; x++) {

            if (kf.depth.at<double>(y,x) <= 0) {
                continue; // ignore invalid pixels
            }

            // sample depth
            Scalar depth = Scalar(kf.depth.at<double>(y,x)); // assume depth is a constant

            // sample gradient
            Scalar grad_s[2];
            kf_jac->pos.x() = x;
            kf_jac->pos.y() = y;
            keyframe_sampler(twist_arr, grad_s);
            Vec2 grad(grad_s[0], grad_s[1]);


            //kf_depth_interpolater.Evaluate(Scalar(y), Scalar(x), &depth);
            //kf_grad_interpolater .Evaluate(Scalar(y), Scalar(x), grad.data());
            //CeresInterpolator<Scalar>::eval(kf_depth_interpolater, Scalar(y), Scalar(x), &depth);
            //CeresInterpolator2<Vec2>::eval (kf_grad_interpolater, y, x, &grad);

            //Pixel<Scalar> px = kf.getPixel(x,y).template cast<Scalar>();
            Pixel<Scalar> px(Vec2(Scalar(x), Scalar(y)), depth);

            Vec2 flow = px.calc_flow(kf.camera, motion);

            Scalar E_exp = -grad.dot(flow);

            E_pred_norm += E_exp * E_exp;
            E_pred(residual_idx) = E_exp;

            residual_idx++;

            typedef Eigen::Vector2d Vec2d;

            if (viz_params) {
                viz_params->projected_flow.at<Vec2d>(y,x)  = CeresCaster::toDoubleV(flow);
                viz_params->exp_intensity .at<double>(y,x) = CeresCaster::toDouble(E_exp);
            }
        }
    }
    __t_innerloop.stop();

    // fill up the remaining entries by setting them to 0
    for (; residual_idx < residual_size; residual_idx++) {
        if (residual != nullptr) {
            residual[residual_idx] = Scalar(0);
        }

        E_pred(residual_idx) = Scalar(0);
    }

    E_pred_norm = ceres::sqrt(E_pred_norm);

    Scalar diff = Scalar(0);

    residual_idx = 0;
    for (size_t y = 0; y < params.eventframe.img.rows; y++) {
        for (size_t x = 0; x < params.eventframe.img.cols; x++) {

            if (kf.depth.at<double>(y,x) <= 0) {
                continue; // ignore invalid pixels
            }

            Scalar actual_intensity_change;
            //CeresInterpolator<Scalar>::eval(event_image_interpolator, Scalar(y), Scalar(x), &actual_intensity_change);
            event_image_interpolator.Evaluate(Scalar(y), Scalar(x), &actual_intensity_change);

            // event image has already been normalized in constructor

            Scalar d =  actual_intensity_change - E_pred(residual_idx) / E_pred_norm;
            diff += d*d;

            if (viz_params) {
                viz_params->correlation  .at<double>(y,x)  = CeresCaster::toDouble(d);
                viz_params->exp_intensity.at<double>(y,x) /= CeresCaster::toDouble(E_pred_norm);
            }

            if (residual != nullptr) {
                residual[residual_idx] = d;
            }

            residual_idx++;
        }
    }

    return diff;
}

////////////////////////////////////////////////////////////////////////////////

// instantiate templates, so we don't have to define the template in the header
// This allows for faster recompilation, as we don't have to change headers
// when editing the error calculation.  As a nice side-effect, we can prevent
// wrong instantations (i.e. only allow double and Jet as template parameters).
INSTANTIATE_TEMPLATES(OptRerender);

////////////////////////////////////////////////////////////////////////////////

