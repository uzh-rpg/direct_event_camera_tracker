#ifndef WARP_H_INCLUDED
#define WARP_H_INCLUDED

#include <opencv2/opencv.hpp>

#include <Eigen/Dense>

////////////////////////////////////////////////////////////////////////////////

//! given the size of an image, camera intrinsics and a transformation matrix,
//! calculates the coordinates of the pixels after the transformation
void project_points(const cv::Size& imgsize, cv::Mat& dst);

////////////////////////////////////////////////////////////////////////////////

#endif /* end of include guard: WARP_H_INCLUDED */
