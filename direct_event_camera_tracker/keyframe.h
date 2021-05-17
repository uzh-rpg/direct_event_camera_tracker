#ifndef INTENSITYFRAME_H_INCLUDED
#define INTENSITYFRAME_H_INCLUDED

#include <Eigen/Geometry>

#include <ros/time.h>

#include <opencv2/opencv.hpp>

#include "utils.h"
#include "point.h"

////////////////////////////////////////////////////////////////////////////////

class Keyframe
{
public:
    Keyframe() {};
    Keyframe(const cv::Mat& intensity, const cv::Mat& depth, const CameraIntrinsics& camera, const ros::Time stamp = ros::Time(0));

    void cv_show(const std::string& name) const;

    ros::Time stamp;
    cv::Mat intensity;
    cv::Mat depth;

    // calculated data
    cv::Mat gradient;
    cv::Mat gradient2; // gradient of gradient (i.e. second image derivative)

    void flow(cv::Mat& dst, const Motionf& motion, const CameraIntrinsics& camera) const;

    Pixelf getPixel(const size_t& x, const size_t y) const;

    // ground truth (if any)
    Statef T_WK;

    inline bool is_valid() const { return !intensity.empty() && !depth.empty() && !stamp.isZero(); }

    CameraIntrinsics camera;

    void calc_gradient();

    Keyframe copy();
    Keyframe downsample2();

    unsigned int get_valid_px_count() { return valid_px_count; }

    void blur(size_t kernel_size);

private:
    // mark pixels as invalid (-1) if they don't have a depth (i.e. because
    // there was nothing rendered there) or if they are neighbouring such a
    // pixel (and thus the gradient would be wrong)
    void validate_depth_data();
    unsigned int valid_px_count;
};

////////////////////////////////////////////////////////////////////////////////

#endif /* end of include guard: INTENSITYFRAME_H_INCLUDED */
