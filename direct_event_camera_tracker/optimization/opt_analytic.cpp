#include "opt_analytic.h"

using namespace std;
using namespace Eigen;
using namespace ceres;

////////////////////////////////////////////////////////////////////////////////

class ImageSampler : public ceres::SizedCostFunction<1, 2>
{
    const cv::Mat& img;
    const cv::Mat& grad;
public:
    ImageSampler(const cv::Mat& img, const cv::Mat& grad) : img(img), grad(grad) {};

    bool Evaluate(const double* const* params, double* residuals, double** jacobians) const
    {
        double x = params[0][0];
        double y = params[0][1];
        Vector2d u(x,y);

        residuals[0] = bilinear_interp<double>(img, u);

        if (jacobians && jacobians[0]) {
            double* J = jacobians[0];

            // compute derivative of image
            Vector2d g = bilinear_interp<Vector2d>(grad, u);
            J[0] = g.x();
            J[1] = g.y();
        }

        // TODO: check if we're inside image?
        return true;
    }
};

////////////////////////////////////////////////////////////////////////////////

class ImageGradSampler : public ceres::SizedCostFunction<2, 2>
{
    const cv::Mat& grad;
    const cv::Mat& grad2;
public:
    ImageGradSampler(const cv::Mat& grad, const cv::Mat& grad2) : grad(grad), grad2(grad2) {};

    bool Evaluate(const double* const* params, double* residuals, double** jacobians) const
    {
        // we're only ever sampling at integer positions ;)
        int x = (int) params[0][0];
        int y = (int) params[0][1];

        Vector2d g = grad.at<Vector2d>(y,x);
        residuals[0] = g.x();
        residuals[1] = g.y();

        if (jacobians && jacobians[0]) {
            double* J = jacobians[0];

            // compute second derivative (or rather just sample precomputed ones ;)
            Vector4d g2 = grad2.at<Vector4d>(y,x);

            J[0] = g2[0]; // d/dx gradx
            J[1] = g2[1]; // d/dy gradx
            J[2] = g2[2]; // d/dx grady
            J[3] = g2[3]; // d/dy grady
        }

        return true;
    }
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

OptAnalytic::OptAnalytic(const OptParams& params)
    : IOptimizationTraits(params)
{
    // calculate derivative of eventframe
    image_gradient(params.eventframe.img, event_frame_grad);
}

////////////////////////////////////////////////////////////////////////////////

template <typename Scalar, typename MotionScalar>
Scalar OptAnalytic::error(
    Motion<Scalar> T_ref_cam,
    Motion<MotionScalar> V_cam,
    boost::optional<OptVisualizationParams> viz_params,
    Scalar residual[]) const
{
#define PROF_NAME "OptAnalytic<"+type_name<Scalar>()+">::"

    AssertItsNotFloat(Scalar);
    ProfilerTimer __t_total(PROF_NAME "error()");

    typedef Eigen::Matrix<Scalar,2,1> Vec2;
    typedef Eigen::Matrix<Scalar,3,1> Vec3;

    const CameraIntrinsics& camera = params.keyframe.camera;

    // init output images
    if (viz_params) {
        viz_params->init(params.eventframe.img.size());
    }

    Scalar E_pred_norm = Scalar(0);
    Scalar E_real_norm = Scalar(0);

    size_t residual_idx = 0;
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> E_pred(residual_size());
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> E_real(residual_size());

    // image samplers
    ImageSampler* ev_img_J = new ImageSampler(params.eventframe.img, event_frame_grad);
    ceres::CostFunctionToFunctor<1, 2> eventframe_sampler(ev_img_J);

    ImageGradSampler* grad_img_J = new ImageGradSampler(params.keyframe.gradient, params.keyframe.gradient2);
    ceres::CostFunctionToFunctor<2, 2> gradient_sampler(grad_img_J);

    // ceres already makes sure of this, but we also want a constant size motion vector when plotting
    V_cam = V_cam.normalized();

    if (!params.keyframe.T_WK.valid) {
        throw "Invalid pose for keyframe in forward warp optimization.";
    }

    const Pose<Scalar> T_cam_world = T_world_cam_from_T_ref_cam(T_ref_cam.asVector()).inverse();
    const Pose<Scalar> T_world_kf  = params.keyframe.T_WK.pose.cast<Scalar>();
    const Pose<Scalar> T_cam_kf    = T_cam_world * T_world_kf;

    const Eigen::Translation<Scalar,3> translation_cam_kf = T_cam_kf.position;
    const Eigen::Matrix<Scalar,3,3>    rotation_cam_kf    = T_cam_kf.R();


    // distortion parameters
    Scalar k1 = Scalar(camera.getDistortion()[0]);
    Scalar k2 = Scalar(camera.getDistortion()[1]);
    Scalar p1 = Scalar(camera.getDistortion()[2]);
    Scalar p2 = Scalar(camera.getDistortion()[3]);


    ProfilerTimer __t_innerloop(PROF_NAME "error() inner loop");
    // unproject, warp, project gradient pixels
    for (size_t y = 0; y < params.eventframe.img.rows; y++) {
        for (size_t x = 0; x < params.eventframe.img.cols; x++) {
            // TODO: I think this is actually produces the wrong (ie. no) derivative!
            //Pixel<Scalar> kf_pixel = keyframe.getPixel(x,y).template cast<Scalar>();
            // on the other hand, sampling only depends on x and y which are constant in here

            Scalar depth;
            Vec2 cam_grad;

            //kf_depth_interpolater.Evaluate(Scalar(y), Scalar(x), &depth);
            // we're assuming depth is a constant and independent of x and y
            depth = Scalar(params.keyframe.depth.at<double>(y,x));

            if (depth <= Scalar(0)) {
                // skip invalid pixels (i.e. those where nothing was rendered)
                continue;
            }

            //ProfilerTimer __t1(PROF_NAME "loop reading grad");

            Scalar u[] = {Scalar(x), Scalar(y)};

            gradient_sampler(u, cam_grad.data());


            /*
            Pixel<Scalar> kf_pixel(Vec2(Scalar(x), Scalar(y)), depth, Scalar(0), cam_grad);

            WorldPoint<Scalar> kf_point = kf_pixel.unproject(camera);
            WorldPoint<Scalar> cam_point = T_cam_kf * kf_point; // TODO: this is quite slow! it calls quat->rot.matrix for every iteration!

            Pixel<Scalar> cam_pixel = cam_point.project(camera);
            */


            // unproject
            const Vec3 kf_point = Vec3(
                    (Scalar(x) - Scalar(camera.getPrincipalPoint().x())) / Scalar(camera.getFocalLength().x()) * depth,
                    (Scalar(y) - Scalar(camera.getPrincipalPoint().y())) / Scalar(camera.getFocalLength().y()) * depth,
                    depth
            );

            // transform
            const Vec3 cam_point_r = rotation_cam_kf * kf_point;
            const Vec3 cam_point   = translation_cam_kf * cam_point_r; // yeah, this is simply a vector addition...

            // project
            Vec2 image_pos_normalized = cam_point.template head<2>() / cam_point.z();

            // apply camera distortion
            // TODO: opencv::projectPoints could be used (it can also compute the jacobians)
            // this code is adapted from https://github.com/opencv/opencv/blob/master/modules/calib3d/src/calibration.cpp
            Scalar r2 = image_pos_normalized.squaredNorm();
            Scalar r4 = r2*r2;

            // yeah, we now have x/y and X/Y, this isn't very nice :(
            Scalar X = image_pos_normalized.x();
            Scalar Y = image_pos_normalized.y();
            Scalar a1 = Scalar(2)*X*Y;
            Scalar a2 = r2 + Scalar(2)*X*X;
            Scalar a3 = r2 + Scalar(2)*Y*Y;
            Scalar cdist = Scalar(1) + k1*r2 + k2*r4;
            Scalar xd0 = X*cdist + p1*a1 + p2*a2;
            Scalar yd0 = Y*cdist + p1*a3 + p2*a1;

            // un-normalize coordinates
            Vec2 cam_pixel = Vec2(xd0, yd0).array()
            * camera.getFocalLength().cast<Scalar>().array() + camera.getPrincipalPoint().cast<Scalar>().array();

            double cam_pixel_x = std::floor(CeresCaster::toDouble(cam_pixel.x()));
            double cam_pixel_y = std::floor(CeresCaster::toDouble(cam_pixel.y()));

            if (cam_pixel_x < 0
             || cam_pixel_y < 0
             || cam_pixel_x+1 >= camera.getCameraSize().x()
             || cam_pixel_y+1 >= camera.getCameraSize().y()) {
                // skip pixels that land outside of our event frame
                //__t1.cancel();
                continue;
            }

            //__t1.stop();
            //ProfilerTimer __t2(PROF_NAME "loop calc flow and exp change");

            // calculate optic flow
            //Vec2 flow = cam_pixel.calc_flow(params.keyframe.camera, V_cam);
            // normalize pixel coordinates
            Vec2 p = (cam_pixel - camera.getPrincipalPoint().cast<Scalar>()).array()
                                / camera.getFocalLength().cast<Scalar>().array();

            //cout << "normalized at " << p.transpose() << " = ";

            // See Corke's 'Robotics, Vision and Control', page 461
            // this assumes OpenCV's coordinate system convention
            const Scalar dx =
                + Scalar(V_cam.velocity.x())*(-Scalar(1)/depth)
              //+ Scalar(V_cam.velocity.y())*0
                + Scalar(V_cam.velocity.z())*p.x()/depth
                + Scalar(V_cam.rotation.x())*p.x()*p.y()
                + Scalar(V_cam.rotation.y())*(-(Scalar(1) + p.x()*p.x()))
                + Scalar(V_cam.rotation.z())*p.y();

            const Scalar dy =
              //+ Scalar(V_cam.velocity.x())*0
                + Scalar(V_cam.velocity.y())*(-Scalar(1)/depth)
                + Scalar(V_cam.velocity.z())*p.y()/depth
                + Scalar(V_cam.rotation.x())*(Scalar(1) + p.y()*p.y())
                + Scalar(V_cam.rotation.y())*(-p.x()*p.y())
                + Scalar(V_cam.rotation.z())*(-p.x());

            //cout << "dx: " << dx << " dy: " << dy << "  ->  ";

            // un-normalize our derivative
            Vec2 flow = Vec2(dx, dy).array() * camera.getFocalLength().cast<Scalar>().array();


            // calculate gradient in current frame by taking gradient of keyframe and warping it
            // according to inv(dx(0)/dx(t))
            Vec2 grad = rotation_cam_kf.template block<2,2>(0,0) * cam_grad;
            //Vec2 grad = cam_pixel.gradient;

            // optic flow specifies movement of a constant point along the
            // image plane, but we want the opposite, namely the insity change
            // of a non-moving point (a pixel) => -flow
            Scalar expected_intensity_change = -grad.dot(flow);


            // calculate norm of expected intensity change, so we correlate two normalized vectors
            E_pred_norm += expected_intensity_change * expected_intensity_change;

            /*
            if (std::isnan(DOUBLE(expected_intensity_change))) {
                std::cerr << "NAN: at " << x << ", " << y << ":" << std::endl;
                std::cout << "exp change: " << expected_intensity_change << std::endl;
                std::cout << VEC2F(flow);
                std::cout << VEC2F(grad);
            }
            */

            E_pred(residual_idx) = expected_intensity_change;

            //__t2.stop();
            //ProfilerTimer __t3(PROF_NAME "loop calc actual change");

            // interpolate event image
            Scalar actual_intensity_change;
            eventframe_sampler(cam_pixel.data(), &actual_intensity_change);

            E_real(residual_idx) = actual_intensity_change;
            E_real_norm += actual_intensity_change * actual_intensity_change;

            /*
            if (std::isnan(DOUBLE(actual_intensity_change))) {
                std::cerr << "NAN: at " << x << ", " << y << ":" << std::endl;
                std::cout << "actual change: " << actual_intensity_change << std::endl;
                std::cout << VEC2F(cam_pixel.pos);
            }
            */

            /*
            if (img_projected_intensity != nullptr) {
                cout << "is inside? " << cam_pixel.is_inside_bilinear(params.keyframe.camera.getCameraSize()) << endl;
                cout << "pos: " << cam_pixel.pos << endl;
                cout << "pos.x: " << cam_pixel.pos.x() << endl;
                cout << "pos.y: " << cam_pixel.pos.y() << endl;
                cout << "pos: double: " << CeresCaster::toDoubleV(cam_pixel.pos) << endl;
                cout << "X: double: " << CeresCaster::toDouble(cam_pixel.pos.x()) << endl;
                cout << "X: floor(double): " << ceres::floor(CeresCaster::toDouble(cam_pixel.pos.x())) << endl;
                cout << "camera size: " << params.keyframe.camera.getCameraSize().transpose() << endl;
                cout << CeresCaster::toDouble(ceres::floor(cam_pixel.pos.x())) << endl;
                cout << CeresCaster::toDouble(ceres::floor(cam_pixel.pos.y())) << endl;
                cout << CeresCaster::toDouble(ceres::floor(cam_pixel.pos.x())+Scalar(1)) << endl;
                cout << CeresCaster::toDouble(ceres::floor(cam_pixel.pos.y())+Scalar(1)) << endl;
                cout << endl;
            }
            */

            //__t3.stop();
            //ProfilerTimer __t4(PROF_NAME "loop drawing");

            typedef Eigen::Vector2d Vec2d;

            if (viz_params) {
                draw_bilinear<double,double>(viz_params->projected_intensity, CeresCaster::toDoubleV(cam_pixel), params.keyframe.intensity.at<double>(y,x));
                draw_bilinear<Vec2d,Vec2d> (viz_params->projected_gradient,   CeresCaster::toDoubleV(cam_pixel), CeresCaster::toDoubleV(grad));
                draw_bilinear<Vec2d,Vec2d> (viz_params->projected_flow,       CeresCaster::toDoubleV(cam_pixel), CeresCaster::toDoubleV(flow));
                draw_bilinear<double,double>(viz_params->exp_intensity,       CeresCaster::toDoubleV(cam_pixel), CeresCaster::toDouble(expected_intensity_change));
                viz_params->correlation.at<double>(y,x) = 1; // mark as valid
            }

            residual_idx++;
            if (residual_idx >= residual_size()) {
                goto end_of_mainloop; // wheeee, the raptors are coming!
            }
        }
    }
    __t_innerloop.stop();

    if (residual_idx < 100) {
        std::cerr << "ERROR: only got " << residual_idx << " residuals!" << std::endl;
    }

    // fill up the remaining entries
    for (; residual_idx < residual_size(); residual_idx++) {
        if (residual != nullptr) {
            residual[residual_idx] = Scalar(0);
        }

        E_pred(residual_idx) = Scalar(0);
        E_real(residual_idx) = Scalar(0);
    }

end_of_mainloop:

    //E_pred_norm = Scalar(1.5e7);
    E_pred_norm = ceres::sqrt(E_pred_norm);
    E_real_norm = ceres::sqrt(E_real_norm);

    Scalar diff = Scalar(0);
#if 0
    for (size_t y = 0; y < eventframe.img.rows; y++) {
        for (size_t x = 0; x < eventframe.img.cols; x++) {
            Scalar d = E_real(y,x) / E_real_norm - E_pred(y,x) / E_pred_norm;
            diff += d*d;

            if (correlation != nullptr) {
                correlation->at<double>(y,x) = DOUBLE(d);
            }

            if (img_exp_intensity != nullptr) {
                img_exp_intensity->at<double>(y,x) /= DOUBLE(E_pred_norm);
            }

            if (residual != nullptr) {
                residual[y*eventframe.img.cols+x] = d;
            }
        }
    }
#endif
    cv::MatIterator_<double> err_img_it;
    if (viz_params) {
        err_img_it = viz_params->correlation.begin<double>();
    }

    for (residual_idx = 0; residual_idx < residual_size(); residual_idx++) {
        Scalar d = E_real(residual_idx) / E_real_norm - E_pred(residual_idx) / E_pred_norm;

        diff += d*d;

        if (residual != nullptr) {
            residual[residual_idx] = d;
        }

        if (viz_params) {
            // we're discarding the association between residual index and
            // original pixel coordinates. That's why we have to do this in
            // this cumbersome way :/

            // skip invalid pixels
            while (*err_img_it == 0 && err_img_it != viz_params->correlation.end<double>()) { // skip pixels that were skipped in the original calculation
                err_img_it++;
            }

            if (err_img_it != viz_params->correlation.end<double>()) {
                *err_img_it = CeresCaster::toDouble(d);
                err_img_it++;
            }
        }
    }

    if (viz_params) {
        for (size_t y = 0; y < params.eventframe.img.rows; y++) {
            for (size_t x = 0; x < params.eventframe.img.cols; x++) {
                viz_params->exp_intensity.at<double>(y,x) /= CeresCaster::toDouble(E_pred_norm);
            }
        }
    }

    return diff;
}

////////////////////////////////////////////////////////////////////////////////

// instantiate templates, so we don't have to define the template in the header
// This allows for faster recompilation, as we don't have to change headers
// when editing the error calculation.  As a nice side-effect, we can prevent
// wrong instantations (i.e. only allow double and Jet as template parameters).
INSTANTIATE_TEMPLATES(OptAnalytic);

////////////////////////////////////////////////////////////////////////////////

