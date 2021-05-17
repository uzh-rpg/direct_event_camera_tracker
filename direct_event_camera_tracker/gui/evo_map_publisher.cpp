#include "evo_map_publisher.h"

using namespace std;
using namespace sensor_msgs;

////////////////////////////////////////////////////////////////////////////////

EvoMapPublisher::EvoMapPublisher(ros::NodeHandle& node, WorldRenderer* world, QWidget* parent)
    : QDialog(parent), world(world), node(node)
{
    ui.setupUi(this);
    cout << "opening EvoMapPublisher" << endl;

    map_publisher = node.advertise<sensor_msgs::PointCloud2>("/map/pointcloud", 5);
    pose_subscriber = node.subscribe("/evo/pose", 1, &EvoMapPublisher::update_pose, this);
    evo_cmd_subscriber = node.subscribe("/evo/remote_key", 1, &EvoMapPublisher::remoteKeyCallback, this);

    cout << "subscribed to " << evo_cmd_subscriber.getTopic() << endl;

    connect(ui.render, &QPushButton::clicked, this, &EvoMapPublisher::render_current_pose);
    connect(ui.gradient, &ImageLabel::pixelHovered, this, &EvoMapPublisher::gradient_hover);

    connect(ui.threshold, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &EvoMapPublisher::threshold_changed);

    cloud.header.frame_id = "world";
}

////////////////////////////////////////////////////////////////////////////////

void EvoMapPublisher::render_current_pose()
{
    Statef s = ui.pose->getState();
    kf = world->renderPose(s.pose);

    ui.gradient->setPixmap(kf.gradient);
    ui.depth   ->setPixmap(kf.depth);

    cv::Mat thresh_grad;
    cv::threshold(grad_magnitude(kf.gradient), thresh_grad, ui.threshold->value(), 1, cv::THRESH_BINARY);

    ui.gradient_thresholded->setPixmap(thresh_grad);

    publish_map();

    // TODO: this is most likely wrong. We don't want the ground-truth of our current keyframe but rather the gt of the events.
    // we probably also want to render the map at the gt position and not at the tracked one
    publish_gt(s);
}

////////////////////////////////////////////////////////////////////////////////

void EvoMapPublisher::gradient_hover(float x, float y)
{
    if (!kf.is_valid())
        return;

    cv::Vec2f g = kf.gradient.at<cv::Vec2d>((int)y, (int)x);
    assert(kf.gradient.type() == CV_64FC2);

    float mag = sqrt(g[0]*g[0] + g[1]*g[1]);

    ui.grad_mag_lbl->setText(QString::number(mag));
}

////////////////////////////////////////////////////////////////////////////////

cv::Mat EvoMapPublisher::grad_magnitude(const cv::Mat& grad)
{
    // split input into x and y gradients
    vector<cv::Mat> gradients;
    split(grad, gradients);

    cv::Mat mag;
    cv::magnitude(gradients[0], gradients[1], mag);
    return mag;
}

////////////////////////////////////////////////////////////////////////////////

void EvoMapPublisher::update_pose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    //cout << "got new pose!" << endl;

    Statef s = ui.pose->getState();
    s.pose = Posef(msg);
    ui.pose->setState(s);

    //render_current_pose();
}

////////////////////////////////////////////////////////////////////////////////

void EvoMapPublisher::publish_map()
{
    //cout << "publishing map" << endl;

    cv::Mat mag = grad_magnitude(kf.gradient);
    assert(mag.type() == CV_64FC1);
    double thresh = ui.threshold->value();

    cloud.clear();

    Eigen::Transform<double,3,Eigen::Isometry> T_WK = kf.T_WK.pose.T();

    for (int y = 0; y < kf.gradient.cols; y++) {
        for(int x = 0; x < kf.gradient.rows; x++) {
            double m = mag.at<double>(y,x);
            if (m >= thresh) {

                double depth = kf.depth.at<double>(y,x);

                // unproject
                const Eigen::Vector3d kf_point(
                        (double(x) - kf.camera.getPrincipalPoint().x()) / kf.camera.getFocalLength().x() * depth,
                        (double(y) - kf.camera.getPrincipalPoint().y()) / kf.camera.getFocalLength().y() * depth,
                        depth
                );

                // transform
                const Eigen::Vector3d world_point = T_WK * kf_point;

                pcl::PointXYZI p;
                p.x = world_point.x();
                p.y = world_point.y();
                p.z = world_point.z();
                p.intensity = 1.0 / p.z;
                cloud.push_back(p);
            }
        }
    }

    // publish our cloud to ROS
    sensor_msgs::PointCloud2::Ptr pc_msg(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(cloud, *pc_msg);
    // pc_msg.header.stamp = // TODO!
    map_publisher.publish(pc_msg);
}

////////////////////////////////////////////////////////////////////////////////

void EvoMapPublisher::publish_gt(const Statef& s)
{
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(s.pose.position.x(), s.pose.position.y(), s.pose.position.z()));
    transform.setRotation(tf::Quaternion(s.pose.orientation.x(), s.pose.orientation.y(), s.pose.orientation.z(), s.pose.orientation.w()));
    tf_broadcaster.sendTransform(tf::StampedTransform(transform, s.stamp, "world", "dvs_groundtruth"));
}

////////////////////////////////////////////////////////////////////////////////

void EvoMapPublisher::remoteKeyCallback(const std_msgs::String::ConstPtr& msg)
{
    std::string command_str = msg->data;
    cout << "Received command: " << command_str << endl;

    if(command_str == "update") {
        render_current_pose();
    }
}

////////////////////////////////////////////////////////////////////////////////

