// standard libraries

#include <iostream> // std::cout
#include <functional> // std::bind

// ROS

#include <ros/ros.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <dvs_msgs/EventArray.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

// Eigen

#include <Eigen/Dense>
#include <Eigen/Geometry>

// OpenCV

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/viz.hpp>

#include <cv_bridge/cv_bridge.h>

// Qt

#include <QApplication>
#include <QSurfaceFormat>
#include <QDir>

// Ceres

#include <ceres/ceres.h>
#include <glog/logging.h>

// project

#include "keyframe.h"
#include "datasource.h"
#include "event_buffer.h"
#include "utils/bag_synchronizer.h"
#include "gui/maingui.h"
#include "gui/application.h"
