#include "keyframe.h"

using namespace std;
using namespace Eigen;

////////////////////////////////////////////////////////////////////////////////

Keyframe::Keyframe(const cv::Mat& intensity, const cv::Mat& depth, const CameraIntrinsics& camera, const ros::Time stamp)
    : stamp(stamp), intensity(intensity), depth(depth), camera(camera), valid_px_count(0)
{
    //assert(intensity.type() == CV_64FC1);
    //assert(depth.type() == CV_64FC1);

    if (camera.getCameraWidth() != intensity.cols || camera.getCameraHeight() != intensity.rows) {
        std::cerr << "ERROR: Invalid keyframe: Camera intrinsics says camera size is "
            << camera.getCameraSize().transpose() << ", but intensity image is " << intensity.size() << endl;
    }

    calc_gradient();

    if (camera.getCameraWidth()  != depth.cols
            || camera.getCameraHeight() != depth.rows
            || camera.getCameraWidth()  != gradient.cols
            || camera.getCameraHeight() != gradient.rows
            || camera.getCameraWidth()  != gradient2.cols
            || camera.getCameraHeight() != gradient2.rows) {
        std::cerr << "ERROR: Invalid keyframe: Camera intrinsics says camera size is "
            << camera.getCameraSize().transpose() << ", but gradient is " << gradient.size() <<
            " and depth " << depth.size() << std::endl;
    }

    validate_depth_data();
}

////////////////////////////////////////////////////////////////////////////////

void Keyframe::calc_gradient()
{
    // calculate first and second derivative of intensity image
    image_gradient(intensity, gradient);
    gradient_of_gradient(gradient, gradient2);
}

////////////////////////////////////////////////////////////////////////////////

void Keyframe::cv_show(const std::string& name) const
{
    cv::imshow(name + " color", intensity);

    // map grayscale floating point depth to colored uint8
    cv::Mat depth_viz;
    depth.convertTo(depth_viz, CV_8UC1, 255, 0);
    cv::applyColorMap(depth_viz, depth_viz, cv::COLORMAP_HSV);

    cv::imshow(name + " depth", depth_viz);

    // show gradient
    cv::Mat grad_viz;
    visualize_gradient(gradient, grad_viz);
    cv::imshow(name + " gradient", grad_viz);

    cv::waitKey(1);
}

////////////////////////////////////////////////////////////////////////////////

void Keyframe::flow(cv::Mat& dst, const Motionf& motion, const CameraIntrinsics& camera) const
{
    if (dst.empty()) {
        dst = cv::Mat(camera.getCameraHeight(), camera.getCameraWidth(), CV_64FC2, cv::Scalar(0,0));
    } else {
        assert(dst.type() == CV_64FC2);
        assert(dst.cols == camera.getCameraWidth());
        assert(dst.rows == camera.getCameraHeight());
    }

    for (size_t y = 0; y < dst.rows; y++) {
        for (size_t x = 0; x < dst.cols; x++) {
            dst.at<Vector2d>(y,x) = getPixel(x, y).calc_flow(camera, motion);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

Pixelf Keyframe::getPixel(const size_t& x, const size_t y) const
{
    // TODO: check if valid
    return Pixelf(
        Vector2d(x,y),
        depth.at<double>(y,x),
        intensity.at<double>(y,x),
        gradient.at<Vector2d>(y,x).cast<double>()
    );
}

////////////////////////////////////////////////////////////////////////////////

Keyframe Keyframe::copy()
{
    Keyframe kf;
    kf.T_WK = this->T_WK;
    kf.stamp = this->stamp;
    kf.camera = this->camera;

    this->intensity .copyTo(kf.intensity);
    this->gradient  .copyTo(kf.gradient);
    this->gradient2 .copyTo(kf.gradient2);
    this->depth     .copyTo(kf.depth);

    return kf;
}

////////////////////////////////////////////////////////////////////////////////

Keyframe Keyframe::downsample2()
{
    Keyframe kf;
    kf.T_WK = this->T_WK;
    kf.stamp = this->stamp;
    kf.camera = this->camera;
    kf.camera.downsample2();

    downsample2_img(this->intensity, kf.intensity);

    kf.calc_gradient();
    //downsample2_img(this->gradient,  kf.gradient);
    //downsample2_img(this->gradient2, kf.gradient2);
    downsample2_img(this->depth, kf.depth);

    return kf;
}

////////////////////////////////////////////////////////////////////////////////

void Keyframe::blur(size_t kernel_size)
{
    if (kernel_size == 0) {
        return;
    }

    // kernel size must be odd!
    if (kernel_size % 2 == 0) {
        kernel_size++;
    }

    // TODO: should we also blur the depth?
    cv::GaussianBlur(gradient,  gradient,  cv::Size(kernel_size, kernel_size), 0, 0, cv::BORDER_REPLICATE);
    cv::GaussianBlur(gradient2, gradient2, cv::Size(kernel_size, kernel_size), 0, 0, cv::BORDER_REPLICATE);
}

////////////////////////////////////////////////////////////////////////////////

void Keyframe::validate_depth_data()
{
    cv::Mat invalid_mask = depth >= camera.getFarClipping()-0.001;

    // we're taking the second derivative of our image, so we need at least 2 good pixels around each valid point
    cv::dilate(invalid_mask, invalid_mask, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5)));
    //cv::dilate(invalid_mask, invalid_mask, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3)));

    // set gradient to 0 for invalid pixels
    cv::Mat zero(gradient.size(), CV_64FC2, cv::Scalar(0,0));
    zero.copyTo(gradient, invalid_mask);

    // set depth to -1 for invalid pixels
    cv::Mat neg(depth.size(), CV_64FC1, cv::Scalar(-1));
    neg.copyTo(depth, invalid_mask);

    valid_px_count = camera.getCameraWidth() * camera.getCameraHeight() - cv::countNonZero(invalid_mask);
    //cout << "got " << valid_px_count << " valid pixels out of " << camera.getCameraWidth() * camera.getCameraHeight() << endl;
}

////////////////////////////////////////////////////////////////////////////////

