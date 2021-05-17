#include "datasource.h"

using namespace std;
using namespace Eigen;
using namespace sensor_msgs;
using namespace message_filters;

////////////////////////////////////////////////////////////////////////////////

Datasource::Datasource()
    : event_count_density(0.32), // based on the 200 events for a 25x25 patch of the feature tracker
    inbag(nullptr),
    T_EF_VICON(Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitY())
             * Eigen::AngleAxisd( 0.5*M_PI, Eigen::Vector3d::UnitX()))
{
}

////////////////////////////////////////////////////////////////////////////////

Datasource::~Datasource()
{
    if (inbag) {
        inbag->close();
        delete inbag;
    }
}

////////////////////////////////////////////////////////////////////////////////

void Datasource::load_bag(const std::string& bag_path)
{
    inbag = new rosbag::Bag(bag_path, rosbag::bagmode::Read);

    cout << "getting camera info" << endl;
    rosbag::View view_cam_info(*inbag, rosbag::TopicQuery(std::vector<string>({"/cam0/camera_info", "/dvs/camera_info"})));
    for (rosbag::MessageInstance const m: view_cam_info) {
        camera_info_callback(m.instantiate<sensor_msgs::CameraInfo>());
        break;
    }

    std::vector<string> event_topics = {"/cam0/events", "/dvs/events"};

    cout << "loading events" << endl;
    rosbag::View view_events(*inbag, rosbag::TopicQuery(event_topics));
    for (rosbag::MessageInstance const m: view_events) {
        events.callback(m.instantiate<dvs_msgs::EventArray>());
    }
    cout << "loaded " << events.size() << " events" << endl;

    cout << "loading ground truth poses" << endl;
    BagSynchronizer<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped> gt_sync("/cam0/pose", "/cam0/twist");
    rosbag::View gt_view; gt_sync.set_view(gt_view, *inbag);
    gt_sync.set_callback(boost::bind(&Datasource::gt_pose_callback, this, _1, _2));

    for (const rosbag::MessageInstance& m: gt_view) {
        gt_sync.newMessage(m);
    }

    rosbag::View gt_pose_view(*inbag, rosbag::TopicQuery(std::vector<string>({"/optitrack/DAVIS_BlueFox", "/optitrack/davis_realsense"})));
    for (rosbag::MessageInstance const m: gt_pose_view) {
        gt_pose_callback(m.instantiate<geometry_msgs::PoseStamped>());
    }


    cout << "grabbing keyframes" << endl;
    BagSynchronizer<Image, Image, geometry_msgs::PoseStamped, geometry_msgs::TwistStamped> img_sync("/cam0/image_raw", "/cam0/depthmap", "/cam0/pose", "/cam0/twist");
    rosbag::View v;
    img_sync.set_view(v, *inbag);

    img_sync.set_callback(boost::bind(&Datasource::camera_image_callback_with_gt, this, _1, _2, _3, _4));

    for (const rosbag::MessageInstance& m: v) {
        img_sync.newMessage(m);
    }

    cout << "loading imu data" << endl;
    std::vector<string> imu_topics = {"/dvs/imu", "/imu"};
    rosbag::View view_imu(*inbag, rosbag::TopicQuery(imu_topics));
    for (rosbag::MessageInstance const m: view_imu) {
        imu_callback(m.instantiate<sensor_msgs::Imu>());
    }
}

////////////////////////////////////////////////////////////////////////////////

ros::Time Datasource::get_begin_time()
{
    if (inbag) {
        rosbag::View generic_view(*inbag);
        return generic_view.getBeginTime();
    } else {
        return ros::Time(0);
    }
}

////////////////////////////////////////////////////////////////////////////////

ros::Time Datasource::get_end_time()
{
    if (inbag) {
        rosbag::View generic_view(*inbag);
        return generic_view.getEndTime();
    } else {
        return ros::Time(0);
    }
}

////////////////////////////////////////////////////////////////////////////////

void Datasource::camera_info_callback(const sensor_msgs::CameraInfo::ConstPtr& camera_info)
{
    // hack to see if we got our camera info already ;)
    if (!camera_info_valid()) {
        cout << "got camera info" << endl;
        camera = CameraIntrinsics(camera_info);
        events.set_camera(camera);
    }
}

////////////////////////////////////////////////////////////////////////////////

void Datasource::camera_image_callback(const sensor_msgs::Image::ConstPtr& intensity, const sensor_msgs::Image::ConstPtr& depth)
{
    cv_bridge::CvImagePtr cv_intensity = cv_bridge::toCvCopy(intensity);
    cv_bridge::CvImagePtr cv_depth     = cv_bridge::toCvCopy(depth);

    //cout << "got camera image. depth depth: " << (cv_depth->image.type() == CV_64FC1) << endl;
    keyframes.push_back(Keyframe(cv_intensity->image, cv_depth->image, camera, intensity->header.stamp));
}

////////////////////////////////////////////////////////////////////////////////

void Datasource::camera_image_callback_with_gt(
            const sensor_msgs::Image::ConstPtr& intensity,
            const sensor_msgs::Image::ConstPtr& depth,
            const geometry_msgs::PoseStamped::ConstPtr& pose,
            const geometry_msgs::TwistStamped::ConstPtr& twist)
{
    camera_image_callback(intensity, depth);

    keyframes.back().T_WK = Statef(
            intensity->header.stamp,
            Posef(pose), Motionf(twist));
}

////////////////////////////////////////////////////////////////////////////////

void Datasource::gt_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose, const geometry_msgs::TwistStamped::ConstPtr& twist)
{
    gt_poses.push_back(
            Statef(pose->header.stamp,
                Posef(pose),
                Motionf(twist)
            )
        );
}

////////////////////////////////////////////////////////////////////////////////

// call this function *before* loading hand-eye calibration!
void Datasource::gt_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    gt_poses.push_back(
            Statef(pose->header.stamp,
                Posef(pose),
                Motionf() // no velocity ground-truth (maybe insert IMU measurements here?)
            )
        );
}

////////////////////////////////////////////////////////////////////////////////

void Datasource::imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    Motionf motion = Motionf(imu_msg);

    // transfer angular velocity from IMU to camera's frame
    motion.rotation = camera.getCamIMU().rotation() * motion.rotation;

    // transfer linear acceleration
    // TODO: is it correct to use the rotated angular velocity here?
    motion.velocity = camera.getCamIMU().rotation()*motion.velocity + motion.rotation.cross(motion.rotation.cross(camera.getCamIMU().translation()));

    imu.push_back(std::pair<ros::Time, Motionf>(imu_msg->header.stamp, motion));
}

////////////////////////////////////////////////////////////////////////////////

const Keyframe& Datasource::get_frame(const ros::Time& t) const
{
    if (keyframes.empty() || t > keyframes.back().stamp) {
        throw "not enough keyframes yet";
    }

    // find keyframe closest to time t
    // TODO: we could use something more efficient here than linear search...
    std::vector<Keyframe>::const_iterator kf = keyframes.begin();
    for (std::vector<Keyframe>::const_iterator it = keyframes.begin(); it != keyframes.end(); it++) {
        if (abs((kf->T_WK.stamp - t).toSec()) > abs((it->T_WK.stamp - t).toSec())) {
            kf = it;
        }
    }

    // use our new function
    // check if this results in the same keyframe
    std::vector<Keyframe>::const_iterator otherkf = find_closest<Keyframe>(keyframes, [&t](const Keyframe& k){return (k.T_WK.stamp - t).toSec();});
    if (kf != otherkf) {
        cerr << "BUG in find closest for Keyframe!" << endl;
        cerr << "was looking for t = " << t << endl;
        cerr << "best: " << kf->T_WK.stamp << " with a diff of " << (kf->T_WK.stamp-t).toSec() << endl;
        cerr << "other: " << otherkf->T_WK.stamp << " with a diff of " << (otherkf->T_WK.stamp-t).toSec() << endl;
        cerr << "offset: " << distance(kf, otherkf) << endl;
    }

    cout << "got keyframe at t = " << kf->T_WK.stamp << ", which is off by " << (kf->T_WK.stamp-t).toSec()*1000 << " ms" << endl;

    return *kf;
}

////////////////////////////////////////////////////////////////////////////////

Eventframe Datasource::integrate_events(ros::Time from_time_t, bool from_middle_t) const
{
    if (camera.getCameraWidth() <= 0) {
        throw "cannot integrate events: no camera intrinsics";
    }

    const size_t event_count = event_count_density * camera.getCameraHeight() * camera.getCameraWidth();
    cout << "integrating events, looking for a count of " << event_count << endl;

    Eventframe ef = events.integrate(from_time_t, event_count, from_middle_t);
    if (ef.img.empty()) {
        // not enough events since last time, wait some more
        cout << "not enough events yet, got " << events.get().size() << " in total" << endl;
    }

    return ef;
}

////////////////////////////////////////////////////////////////////////////////

Statef Datasource::get_ground_truth(const ros::Time& t) const
{
    if (gt_poses.empty() || t > gt_poses.back().stamp) {
        // not enough ground truth data yet, wait some more
        cout << "not enough ground truth data yet, pausing tracking." << endl;
        return Statef();
    }

    // TODO: we could use something more efficient here than linear search...
    Statef T_WK;
    for (const Statef& s: gt_poses) {
        if (abs((T_WK.stamp - t).toSec()) > abs((s.stamp - t).toSec())) {
            T_WK = s;
        }
    }

    if (T_WK.stamp != find_closest<Statef>(gt_poses, [&t](const Statef& k){return (k.stamp - t).toSec();})->stamp) {
        cerr << "BUG in find closest for Statef!" << endl;
    }


    return T_WK;

    //cout << "got groundtruth at t = " << gt.stamp << ", which is off by " << (gt.stamp-t).toSec()*1000 << " ms" << endl;
}

////////////////////////////////////////////////////////////////////////////////

Motionf Datasource::get_imu_state(const ros::Time& t) const
{
    return find_closest<std::pair<ros::Time, Motionf>>(imu,
            [&t](const std::pair<ros::Time, Motionf> p){return (p.first - t).toSec();})->second;
}

////////////////////////////////////////////////////////////////////////////////

void Datasource::publish_gt_track(ros::Publisher& pub) const
{
    visualization_msgs::Marker line;

    line.scale.x = 0.01;
    line.scale.y = 0.01;
    line.scale.z = 0.01;

    line.color.r = 0;
    line.color.g = 1;
    line.color.b = 0;
    line.color.a = 1;

    line.lifetime = ros::Duration();

    line.header.frame_id = "map";
    line.header.stamp = ros::Time(0);

    line.ns = "gt_track"; // namespace
    line.id = 0;

    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.action = visualization_msgs::Marker::ADD;

    for (const Statef& gt: gt_poses) {

        geometry_msgs::Point pt;
        pt.x = gt.pose.position.x();
        pt.y = gt.pose.position.y();
        pt.z = gt.pose.position.z();

        line.points.push_back(pt);
    }

    cout << "publishing " << line.points.size() << " ground truth poses to " << pub.getTopic() << endl;

    pub.publish(line);
}

////////////////////////////////////////////////////////////////////////////////

void Datasource::load_camera_config(const std::string& path)
{
    try {
        camera = CameraIntrinsics(YAML::LoadFile(path));
    } catch (const YAML::BadFile& e) {
        cerr << "ERROR: Failed to read camera configuration file '" << path << "':" << endl;
	cerr << "Bad file. Does it exist?" << endl;
        throw "Could not read camera calibration file.";
    }
    events.set_camera(camera);
}

////////////////////////////////////////////////////////////////////////////////

void Datasource::load_hand_eye(const std::string& path)
{
    camera.load_hand_eye(YAML::LoadFile(path));
    events.set_camera(camera);

    for (Statef& state: gt_poses) {
        // correct ground truth poses (this assumes we loaded GT before calling this function)
        // apply hand-eye calibration to go from motion-capture coordinates to absolute ones
        state.pose = T_EF_VICON * state.pose.T() * camera.getHandEye();
    }


    /*
    cout << "applying hand eye:" << endl;
    cout << "vicion: " << endl;
    cout << T_EF_VICON.matrix() << endl;

    cout << "GT.T:" << endl;
    cout << Posef(pose).T().matrix() << endl;

    cout << "hand eye:" << endl;
    cout << camera.getHandEye().matrix() << endl;

    cout << "final gt:" << endl;
    cout << p.matrix() << endl;
    */
}

////////////////////////////////////////////////////////////////////////////////
