#include "opt_forward_warp.h"

using namespace std;
using namespace Eigen;
using namespace ceres;

////////////////////////////////////////////////////////////////////////////////

OptFwdWarp::OptFwdWarp(const OptParams& params)
    :
    IOptimizationTraits(params),
    event_image_ceres((double*)params.eventframe.img.data, 0, params.eventframe.img.rows, 0, params.eventframe.img.cols),
    event_image_interpolator(event_image_ceres),

    kf_grad_ceres((double*)params.keyframe.gradient.data, 0, params.keyframe.gradient.rows, 0, params.keyframe.gradient.cols),
    kf_grad_interpolater(kf_grad_ceres),

    kf_depth_ceres((double*)params.keyframe.depth.data, 0, params.keyframe.depth.rows, 0, params.keyframe.depth.cols),
    kf_depth_interpolater(kf_depth_ceres)
{
    // calculate approximate threshold so that count( |gradient| > threshold ) is subset_fraction*total_count
    // TODO: think this through: Shouldn't we actually look at the second derivative?
    subset_grad_thresh = find_threshold(params.keyframe.gradient, params.pyramid_lvl.subset_fraction, subset_count, params.keyframe.depth);
}

////////////////////////////////////////////////////////////////////////////////


// would be nice if we could calculate this inside the error function itself,
// but err_diff MUST be const for ceres, so there is no clean way of returnig
// additional results.
double OptFwdWarp::get_overlap_fraction(const Posef& T_cam_kf)
{
    size_t residual_idx = 0;

    // unproject, warp, project gradient pixels
    for (size_t y = 0; y < params.eventframe.img.rows; y++) {
        for (size_t x = 0; x < params.eventframe.img.cols; x++) {
            Pixel<double> kf_pixel = params.keyframe.getPixel(x,y);

            if (kf_pixel.gradient.squaredNorm() < subset_grad_thresh) {
                // skip points that don't contribute much
                continue;
            }

            WorldPoint<double> kf_point = kf_pixel.unproject(params.keyframe.camera);
            WorldPoint<double> cam_point = T_cam_kf * kf_point;

            Pixel<double> cam_pixel = cam_point.project(params.keyframe.camera);

            if (!cam_pixel.is_inside_bilinear(params.keyframe.camera.getCameraSize())) {
                // skip pixels that land outside of our event frame
                continue;
            }

            residual_idx++;
            if (residual_idx >= subset_count) {
                return 1; // EVERY pixel was good
            }
        }
    }

    return double(residual_idx) / subset_count;
}

////////////////////////////////////////////////////////////////////////////////

template <typename Scalar, typename MotionScalar>
Scalar OptFwdWarp::error(
    Motion<Scalar> T_ref_cam,
    Motion<MotionScalar> V_cam,
    boost::optional<OptVisualizationParams> viz_params,
    Scalar residual[]) const
{
#define PROF_NAME "OptFwdWarp<"+type_name<Scalar>()+">::"

    AssertItsNotFloat(Scalar);
    ProfilerTimer __t_total(PROF_NAME "error()");

    typedef Eigen::Matrix<Scalar,2,1> Vec2;

    const CameraIntrinsics& camera = params.keyframe.camera;

    // init output images
    if (viz_params) {
        viz_params->init(params.eventframe.img.size());
    }

    Scalar E_pred_norm = Scalar(0);
    Scalar E_real_norm = Scalar(0);

    size_t residual_idx = 0;
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> E_pred(subset_count);
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> E_real(subset_count);

    // ceres already makes sure of this, but we also want a constant size motion vector when plotting
    V_cam = V_cam.normalized();

    if (!params.keyframe.T_WK.valid) {
        throw "Invalid pose for keyframe in forward warp optimization.";
    }

    const auto T_cam_world_raw = T_world_cam_from_T_ref_cam(T_ref_cam.asVector()).inverse();
    const Pose<Scalar> T_cam_world(
        Eigen::Translation<Scalar, 3>(T_cam_world_raw.translation()),
        Eigen::Quaternion<Scalar>(T_cam_world_raw.linear())
    );
    const Pose<Scalar> T_world_kf  = params.keyframe.T_WK.pose.cast<Scalar>();
    const Pose<Scalar> T_cam_kf    = T_cam_world * T_world_kf;


#if 0
    std::cout << "--------------------------------------------------------------------------------" << std::endl;
    std::cout << "T_ref_cam: " << T_ref_cam << std::endl;
    std::cout << "T_cam_world: " << T_cam_world << std::endl;
    std::cout << "T_world_kf: " << T_world_kf << std::endl;
    std::cout << "T_cam_kf: " << T_cam_kf << std::endl;
    std::cout << "--------------------------------------------------------------------------------" << std::endl;
#endif


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

            kf_grad_interpolater .Evaluate(Scalar(y), Scalar(x), cam_grad.data());
            //CeresInterpolator<Scalar>::eval(kf_depth_interpolater, Scalar(y), Scalar(x), &depth);
            //CeresInterpolator2<Vec2>::eval (kf_grad_interpolater, y, x, &grad);


            //Pixel<Scalar> px = kf.getPixel(x,y).template cast<Scalar>();
            Pixel<Scalar> kf_pixel(Vec2(Scalar(x), Scalar(y)), depth, Scalar(0), cam_grad);

            if (kf_pixel.gradient.squaredNorm() < subset_grad_thresh) {
                // skip points that don't contribute much
                //__t1.cancel();
                continue;
            }

            //Eigen::Matrix<Scalar,2,1> flow = kf_pixel.calc_flow(keyframe.camera, motion);
            //Eigen::Matrix<Scalar,2,1> grad = kf_pixel.gradient;



            WorldPoint<Scalar> kf_point = kf_pixel.unproject(camera);
            WorldPoint<Scalar> cam_point = T_cam_kf * kf_point;

            //pixels[y*eventframe.cols+x] = wp.project(params.keyframe.camera);
            Pixel<Scalar> cam_pixel = cam_point.project(camera);

            // project
            Vec2 image_pos_normalized = cam_point.pos.template head<2>() / cam_point.pos.z();

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
            cam_pixel.pos = Vec2(xd0, yd0).array()
            * camera.getFocalLength().cast<Scalar>().array() + camera.getPrincipalPoint().cast<Scalar>().array();

            if (!cam_pixel.is_inside_bilinear(params.keyframe.camera.getCameraSize())) {
                // skip pixels that land outside of our event frame
                //__t1.cancel();
                continue;
            }

            //__t1.stop();
            //ProfilerTimer __t2(PROF_NAME "loop calc flow and exp change");

            Vec2 flow = cam_pixel.calc_flow(params.keyframe.camera, V_cam);


            // calculate gradient in current frame by taking gradient of keyframe and warping it
            // according to inv(dx(0)/dx(t))
            Vec2 grad = T_cam_kf.R().template block<2,2>(0,0) * cam_pixel.gradient;
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
            //double actual_intensity_change = eventframe.at<double>((int)cam_pixel.pos.y(), (int)cam_pixel.pos.x());
            //Scalar actual_intensity_change = (Scalar) bilinear_interp(eventframe, cam_pixel.pos);
            Scalar actual_intensity_change;
            event_image_interpolator.Evaluate(cam_pixel.pos.y(), cam_pixel.pos.x(), &actual_intensity_change);

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
                draw_bilinear<double,double>(viz_params->projected_intensity, CeresCaster::toDoubleV(cam_pixel.pos), params.keyframe.intensity.at<double>(y,x));
                draw_bilinear<Vec2d,Vec2d> (viz_params->projected_gradient,   CeresCaster::toDoubleV(cam_pixel.pos), CeresCaster::toDoubleV(grad));
                draw_bilinear<Vec2d,Vec2d> (viz_params->projected_flow,       CeresCaster::toDoubleV(cam_pixel.pos), CeresCaster::toDoubleV(flow));
                draw_bilinear<double,double>(viz_params->exp_intensity,       CeresCaster::toDoubleV(cam_pixel.pos), CeresCaster::toDouble(expected_intensity_change));
                viz_params->correlation.at<double>(y,x) = 1; // mark as valid
            }

            residual_idx++;
            if (residual_idx >= subset_count) {
                goto end_of_mainloop; // ayeee, the raptors are coming!
            }
        }
    }
    __t_innerloop.stop();

    if (residual_idx < 100) {
        std::cerr << "ERROR: only got " << residual_idx << " residuals out of " << subset_count << "!" << std::endl;
    }

    // fill up the remaining entries
    // TODO: shouldn't we be able to tell ceres that we have less rows than anticipated, so there's
    // less to calculate?
    for (; residual_idx < subset_count; residual_idx++) {
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

    for (residual_idx = 0; residual_idx < subset_count; residual_idx++) {
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
INSTANTIATE_TEMPLATES(OptFwdWarp);

////////////////////////////////////////////////////////////////////////////////

