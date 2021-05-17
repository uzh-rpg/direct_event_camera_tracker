#ifndef DATASOURCE_H_7ZMAYPCT
#define DATASOURCE_H_7ZMAYPCT

#include <string>
#include <memory>
#include <chrono>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/timer.h>
#include <dvs_msgs/EventArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <yaml-cpp/yaml.h>

#include "keyframe.h"
#include "event_buffer.h"
#include "point.h"
#include "core/camera_intrinsics.h"
#include "utils/bag_synchronizer.h"
#include "opengl/world_renderer.h"

////////////////////////////////////////////////////////////////////////////////

/*!
 * This class provides a more convenient interface to a rosbag (or potentially
 * live data in the future)
 *
 * It is state-less.
 */

class Datasource
{
public:
    Datasource();
    ~Datasource();

    void load_bag(const std::string& bag_path);

    ros::Time get_begin_time();
    ros::Time get_end_time();

    const Keyframe& get_frame(const ros::Time& t) const;
    Statef get_ground_truth(const ros::Time& t) const;
    Motionf get_imu_state(const ros::Time& t) const;
    Eventframe integrate_events(ros::Time from_time_t, bool from_middle_t=false) const;
    const CameraIntrinsics& getCamera() const { return camera; }
    CameraIntrinsics& getCamera() { return camera; }

    EventBuffer events;

    void publish_gt_track(ros::Publisher& pub) const;

    double event_count_density; // how many events should be integrated

    void load_camera_config(const std::string& path);
    void load_hand_eye(const std::string& path);

    const std::vector<Statef>& get_ground_truth_vect() const { return gt_poses; }

protected:
    void camera_info_callback(const sensor_msgs::CameraInfo::ConstPtr& camera_info);
    bool camera_info_valid() const { return camera.getCameraSize().x() != 0; } // hack ;P

    void camera_image_callback(const sensor_msgs::Image::ConstPtr& intensity,const sensor_msgs::Image::ConstPtr& depth);
    void camera_image_callback_with_gt(
            const sensor_msgs::Image::ConstPtr& intensity,
            const sensor_msgs::Image::ConstPtr& depth,
            const geometry_msgs::PoseStamped::ConstPtr& pose,
            const geometry_msgs::TwistStamped::ConstPtr& twist);

    void gt_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose, const geometry_msgs::TwistStamped::ConstPtr& twist);
    void gt_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose);

    void imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg);


    CameraIntrinsics camera;

    std::vector<Statef> gt_poses;
    std::vector<Keyframe> keyframes;
    std::vector<std::pair<ros::Time, Motionf>> imu;

    rosbag::Bag *inbag;

    Eigen::Quaterniond T_EF_VICON;
};

////////////////////////////////////////////////////////////////////////////////

#endif /* end of include guard: DATASOURCE_H_7ZMAYPCT */
