#ifndef EVO_MAP_PUBLISHER_H_IOOPWH1Z
#define EVO_MAP_PUBLISHER_H_IOOPWH1Z

#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_broadcaster.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <QDialog>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QMessageBox>
#include <QPushButton>

#include "opengl/world_renderer.h"
#include "core/camera_intrinsics.h"
#include "point.h"

#include "ui_evo_map_publisher.h"

class EvoMapPublisher : public QDialog
{
    Q_OBJECT

public:
    EvoMapPublisher(ros::NodeHandle& node, WorldRenderer* world, QWidget* parent);

    void update_pose(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void remoteKeyCallback(const std_msgs::String::ConstPtr& msg);

public slots:
    void render_current_pose();

    void gradient_hover(float x, float y);

    void threshold_changed(double) { render_current_pose(); }


private:
    cv::Mat grad_magnitude(const cv::Mat& grad);
    void publish_map();
    void publish_gt(const Statef& s);

    Ui::EvoMapPublisher ui;
    WorldRenderer* world;
    ros::NodeHandle& node;
    tf::TransformBroadcaster tf_broadcaster;

    ros::Publisher map_publisher;
    ros::Subscriber pose_subscriber;
    ros::Subscriber evo_cmd_subscriber;

    Keyframe kf;

    pcl::PointCloud<pcl::PointXYZI> cloud;
};


#endif /* end of include guard: EVO_MAP_PUBLISHER_H_IOOPWH1Z */
