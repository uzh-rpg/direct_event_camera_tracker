#ifndef POINT_H_Y4NWHRFC
#define POINT_H_Y4NWHRFC

#include <cmath>

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include "sophus/se3.hpp"

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <QTableWidget>
#include <QTableWidgetItem>
#include <QString>
#include <QMatrix4x4>

#include "utils.h"
#include "core/camera_intrinsics.h"

////////////////////////////////////////////////////////////////////////////////

// forward declarations
template <typename Scalar> struct Pixel;
template <typename Scalar> struct WorldPoint;
template <typename Scalar> struct Pose;
template <typename Scalar> struct Motion;
template <typename Scalar> struct State;

// typedefs
typedef Pixel<double> Pixelf;
typedef WorldPoint<double> WorldPointf;

typedef Pose<double>     Posef;
typedef Motion<double>   Motionf;
typedef State<double>    Statef;

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename Scalar>
struct Pixel
{
    typedef Eigen::Matrix<Scalar,2,1> Vec2;

    ///////////////////////////////////////////////////////////////////////////

    Pixel(Vec2 pos = Vec2(Scalar(0),Scalar(0)),
            Scalar depth = Scalar(0),
            Scalar intensity = Scalar(0),
            Vec2 gradient = Vec2(Scalar(0),Scalar(0)),
            Vec2 flow = Vec2(Scalar(0),Scalar(0)))
        : pos(pos), intensity(intensity), depth(depth), gradient(gradient), flow(flow) {}

    Pixel(Vec2 pos, const Pixel& px)
        : pos(pos), intensity(px.intensity), depth(px.depth), gradient(px.gradient), flow(px.flow) {}

    ///////////////////////////////////////////////////////////////////////////

    Vec2 pos;
    Scalar intensity;
    Scalar depth;
    Vec2 gradient;
    Vec2 flow;

    ///////////////////////////////////////////////////////////////////////////

    template <typename T>
    Pixel<T> cast() { return Pixel<T>(pos.template cast<T>(), (T) depth, (T) intensity, gradient.template cast<T>(), flow.template cast<T>()); }

    ///////////////////////////////////////////////////////////////////////////

    template <typename T>
    bool inline is_inside(const Eigen::Matrix<T,2,1> rect) const {
        return pos.x() >= Scalar(0)
            && pos.y() >= Scalar(0)
            && pos.x() < Scalar(rect.x())
            && pos.y() < Scalar(rect.y());
    }

    ///////////////////////////////////////////////////////////////////////////

    bool inline is_inside_bilinear(const Eigen::Matrix<int,2,1> rect) const {
        return floor(pos.x()) >= Scalar(0)
            && floor(pos.y()) >= Scalar(0)
            && floor(pos.x())+Scalar(1) < Scalar(rect.x())
            && floor(pos.y())+Scalar(1) < Scalar(rect.y());
    }

    ////////////////////////////////////////////////////////////////////////////

    WorldPoint<Scalar> unproject(const CameraIntrinsics& camera) const
    {
        Vec2 space_pos = (this->pos - camera.getPrincipalPoint().cast<Scalar>()).array() / camera.getFocalLength().cast<Scalar>().array() * this->depth;
        return WorldPoint<Scalar>(typename WorldPoint<Scalar>::Vec3(space_pos.x(), space_pos.y(), this->depth), *this);
    }

    ////////////////////////////////////////////////////////////////////////////

    template <typename MotionScalar>
    Vec2 calc_flow(const CameraIntrinsics& camera, const Motion<MotionScalar>& motion)
    {
        // verified against ground truth from esim

        //cout << "FLOW: ";

        // normalize pixel coordinates
        Vec2 p = (pos - camera.getPrincipalPoint().cast<Scalar>()).array()
                      / camera.getFocalLength().cast<Scalar>().array();

        //cout << "normalized at " << p.transpose() << " = ";

        // See Corke's 'Robotics, Vision and Control', page 461
        // this assumes OpenCV's coordinate system convention
        const Scalar dx =
            + Scalar(motion.velocity.x())*(-Scalar(1)/depth)
          //+ Scalar(motion.velocity.y())*0
            + Scalar(motion.velocity.z())*p.x()/depth
            + Scalar(motion.rotation.x())*p.x()*p.y()
            + Scalar(motion.rotation.y())*(-(Scalar(1) + p.x()*p.x()))
            + Scalar(motion.rotation.z())*p.y();

        const Scalar dy =
          //+ Scalar(motion.velocity.x())*0
            + Scalar(motion.velocity.y())*(-Scalar(1)/depth)
            + Scalar(motion.velocity.z())*p.y()/depth
            + Scalar(motion.rotation.x())*(Scalar(1) + p.y()*p.y())
            + Scalar(motion.rotation.y())*(-p.x()*p.y())
            + Scalar(motion.rotation.z())*(-p.x());

        //cout << "dx: " << dx << " dy: " << dy << "  ->  ";

        // I think we have to un-normalize our derivative here?
        flow = Vec2(dx, dy).array() * camera.getFocalLength().cast<Scalar>().array();

        //cout << flow.transpose() << endl;

        return flow;
    }
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename Scalar>
struct WorldPoint
{
    typedef Eigen::Matrix<Scalar,3,1> Vec3;

    ///////////////////////////////////////////////////////////////////////////

    WorldPoint(Vec3 pos, Pixel<Scalar> p = Pixel<Scalar>())
        : pos(pos), pixel(p) {}

    ///////////////////////////////////////////////////////////////////////////

    Vec3 pos;
    Pixel<Scalar> pixel;

    ///////////////////////////////////////////////////////////////////////////

    Pixel<Scalar> project(const CameraIntrinsics& camera) const
    {
        typename Pixel<Scalar>::Vec2 image_pos = (pos.template head<2>() / pos.z()).array()
            * camera.getFocalLength().cast<Scalar>().array() + camera.getPrincipalPoint().cast<Scalar>().array();

        return Pixel<Scalar>(image_pos, pixel);
    }
};

////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename Scalar>
struct Pose
{
    Pose(const Eigen::Translation<Scalar,3> position    = Eigen::Translation<Scalar,3>(0,0,0),
         const Eigen::Quaternion<Scalar>    orientation = Eigen::Quaternion<Scalar>(1,0,0,0))
        : position(position), orientation(orientation)
    {};

    Pose(Eigen::Transform<Scalar,3,Eigen::Isometry> pose)
        : position(pose.translation()), orientation(pose.rotation())
    {};

    Pose(const geometry_msgs::PoseStamped::ConstPtr& p)
        : position(p->pose.position.x, p->pose.position.y, p->pose.position.z),
          orientation(p->pose.orientation.w, p->pose.orientation.x, p->pose.orientation.y, p->pose.orientation.z)
    {};

    Pose(const Scalar pose_vector[7])
        : position   (pose_vector[0], pose_vector[1], pose_vector[2]),
          orientation(pose_vector[6], pose_vector[3], pose_vector[4], pose_vector[5]) // pose vector has [x y z w], and Eigen::Quaternion() expects [w x y z]!
    {};

    Pose(const Eigen::Matrix<Scalar,7,1> pose_vector)
        : position   (pose_vector[0], pose_vector[1], pose_vector[2]),
          orientation(pose_vector[6], pose_vector[3], pose_vector[4], pose_vector[5]) // pose vector has [x y z w], and Eigen::Quaternion() expects [w x y z]!
    {};

    Eigen::Translation<Scalar,3> position;    // t
    Eigen::Quaternion<Scalar>    orientation; // r, attitude

    inline Eigen::Matrix<Scalar,3,3>                  R() const { Eigen::Matrix<Scalar,3,3> m; m = orientation; return m; }
    inline Eigen::Transform<Scalar,3,Eigen::Isometry> T() const { return position*orientation; }

    QMatrix4x4 toQMatrix() const {
        Eigen::Matrix<float, 4, 4, Eigen::RowMajor> M = cast<float>().T().matrix();
        return QMatrix4x4(M.data());
    }

    // TODO: OPTIMIZE: this seems to be quite slow!
    Pose<Scalar> inverse() const {
        Eigen::Quaternion<Scalar> Rinv = orientation.inverse(); // conjugate could be used to as orientation should always be normalized
        return Pose<Scalar>(
                Eigen::Translation<Scalar,3>((Rinv * position.inverse()).translation()),
                Rinv);
    };

    template <typename T>
    Pose<T> cast() const {
        return Pose<T>(position.template cast<T>(), orientation.template cast<T>());
    }

    template <typename S>
    friend std::ostream& operator<<(std::ostream& out, const Pose<S>& p);

    /*
    bool operator==(const Pose<Scalar>& other) const {
        // TODO: use Eigen::isApprox?
        return (this->position.array()    == other.position.array()).all()
            && (this->orientation.array() == other.orientation.array()).all();
    }
    */

    boost::shared_ptr<geometry_msgs::PoseStamped> as_msg(const ros::Time& t) const
    {
        boost::shared_ptr<geometry_msgs::PoseStamped> msg(new geometry_msgs::PoseStamped);
        msg->header.frame_id = "map"; // global frame
        msg->header.stamp = t;
        msg->pose.position.x = position.x();
        msg->pose.position.y = position.y();
        msg->pose.position.z = position.z();
        msg->pose.orientation.x = orientation.x();
        msg->pose.orientation.y = orientation.y();
        msg->pose.orientation.z = orientation.z();
        msg->pose.orientation.w = orientation.w();
        return msg;
    }

    ////////////////////////////////////////////////////////////////////////////

    Pose<Scalar> operator*(const Pose<Scalar>& other) const
    {
        return this->T() * other.T();
    }

    ////////////////////////////////////////////////////////////////////////////

    //! return 'this - other', i.e. the transformation from other to this
    //! E.g. if this is T_WB and other is T_WA then we get T_AB
    //Pose operator-(const Pose& T_WA) const;
    WorldPoint<Scalar> operator*(const WorldPoint<Scalar>& A_wp) const
    {
        WorldPoint<Scalar> B_wp = A_wp;
        B_wp.pos = T() * A_wp.pos;
        return B_wp;
    }
};

////////////////////////////////////////////////////////////////////////////////

template <typename Scalar>
std::ostream& operator<<(std::ostream& out, const Pose<Scalar>& p)
{
    out << "  Pose:" << std::endl;
    out << "\tPosition: " << p.position.vector().transpose() << std::endl;
    out << "\tOrientation: " << p.orientation.coeffs().transpose() << std::endl;
    return out;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

template <typename Scalar>
struct Motion
{
    typedef Eigen::Matrix<Scalar,3,1> Vec3;

    Motion(Vec3 velocity = Vec3(0,0,0), Vec3 rotation = Vec3(0,0,0))
        : velocity(velocity), rotation(rotation)
    {};

    Motion(const geometry_msgs::TwistStamped::ConstPtr& twist)
        : velocity(twist->twist.linear.x,  twist->twist.linear.y,  twist->twist.linear.z),
          rotation(twist->twist.angular.x, twist->twist.angular.y, twist->twist.angular.z)
    {};

    Motion(const Scalar motion_vector[6])
        : velocity(motion_vector[0], motion_vector[1], motion_vector[2]),
          rotation(motion_vector[3], motion_vector[4], motion_vector[5])
    {};

    Motion(const Eigen::Matrix<Scalar,6,1> motion_vector)
        : velocity(motion_vector[0], motion_vector[1], motion_vector[2]),
          rotation(motion_vector[3], motion_vector[4], motion_vector[5])
    {};

    // Note, we're storing the linear *acceleration* in the *velocity* field here!
    Motion(const sensor_msgs::Imu::ConstPtr& imu)
        : velocity(imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z),
          rotation(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z)
    {};

    template <typename OtherScalar>
    Motion(const Motion<OtherScalar>& m)
        : velocity(m.velocity.template cast<Scalar>()),
          rotation(m.rotation.template cast<Scalar>())
    {};

    Vec3 velocity; // v
    Vec3 rotation; // ω

    template <typename S>
    friend std::ostream& operator<<(std::ostream& out, const Motion<S>& m);

    /*
    bool operator==(const Motion<Scalar>& other) const {
        // TODO: use Eigen::isApprox?
        return (this->velocity.array() == other.velocity.array()).all()
            && (this->rotation.array() == other.rotation.array()).all();
    }
    */

    template <typename T>
    Motion<T> cast() const {
        return Motion<T>(velocity.template cast<T>(), rotation.template cast<T>());
    }

    void toArray(Scalar* arr) const {
        arr[0] = velocity.x();
        arr[1] = velocity.y();
        arr[2] = velocity.z();
        arr[3] = rotation.x();
        arr[4] = rotation.y();
        arr[5] = rotation.z();
    }

    Scalar norm() const
    {
        Scalar len =
            + velocity.x() * velocity.x()
            + velocity.y() * velocity.y()
            + velocity.z() * velocity.z()
            + rotation.x() * rotation.x()
            + rotation.y() * rotation.y()
            + rotation.z() * rotation.z();

        return ceres::sqrt( len );
    }

    Eigen::Matrix<Scalar,6,1> asVector() const {
        Eigen::Matrix<Scalar,6,1> v; v <<
            velocity.x(),
            velocity.y(),
            velocity.z(),
            rotation.x(),
            rotation.y(),
            rotation.z();
        return v;
    }

    Motion<Scalar> normalized() const
    {
        Scalar n = norm();
        return Motion<Scalar>(velocity/n, rotation/n);
    }

    bool isNull() const {
        return (velocity.array() == 0).all() && (rotation.array() == 0).all();
    }

    Sophus::SE3<Scalar> exp() const {
        return Sophus::SE3<Scalar>::exp(asVector());
    }
};

///////////////////////////////////////////////////////////////////////////////

template <typename Scalar>
std::ostream& operator<<(std::ostream& out, const Motion<Scalar>& m)
{
    out << "  Motion:" << std::endl;
    out << "\tVelocity: " << m.velocity.transpose() << std::endl;
    out << "\tRotation: " << m.rotation.transpose() << std::endl;
    return out;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename Scalar>
struct State
{
    typedef Eigen::Matrix<Scalar,2,1> Vec2;
    typedef Eigen::Matrix<Scalar,3,1> Vec3;

    ////////////////////////////////////////////////////////////////////////////

    State(const ros::Time& t = ros::Time(0), const Pose<Scalar>& pose = Pose<Scalar>(), const Motion<Scalar>& motion = Motion<Scalar>())
        : stamp(t), pose(pose), motion(motion), valid(t!=ros::Time(0))
    {};

    State(const ros::Time& t, const Scalar pose_vector[7], const Scalar motion_vector[6])
        : stamp(t), pose(pose_vector), motion(motion_vector), valid(t!=ros::Time(0))
    {};

    State(const QStringList& csv_row)
    {
        if (csv_row.length() < 15) {
            std::cerr << "WARNING: Invalid CSV row, only " << csv_row.length() << " columns" << std::endl;
            throw "Invalid CSV";
        }

        stamp = ros::Time(csv_row[1].toDouble());
        pose = Posef(
                Eigen::Translation3d(
                    csv_row[2].toDouble(), // x
                    csv_row[3].toDouble(), // y
                    csv_row[4].toDouble()  // z
                    ),
                Eigen::Quaterniond(
                    csv_row[8].toDouble(), // w
                    csv_row[5].toDouble(), // x
                    csv_row[6].toDouble(), // y
                    csv_row[7].toDouble()  // z
                    ));
        motion = Motionf(
                Eigen::Vector3d(
                    csv_row[ 9].toDouble(), // x
                    csv_row[10].toDouble(), // y
                    csv_row[11].toDouble()  // z
                    ),
                Eigen::Vector3d(
                    csv_row[12].toDouble(), // x
                    csv_row[13].toDouble(), // y
                    csv_row[14].toDouble()  // z
                    ));
    }

    ////////////////////////////////////////////////////////////////////////////

    // setter/non-const
    Scalar& attr(const size_t& idx)
    {
        switch (idx) {
            case  0: return pose.position.x();
            case  1: return pose.position.y();
            case  2: return pose.position.z();
            case  3: return pose.orientation.x();
            case  4: return pose.orientation.y();
            case  5: return pose.orientation.z();
            case  6: return pose.orientation.w();
            case  7: return motion.velocity.x();
            case  8: return motion.velocity.y();
            case  9: return motion.velocity.z();
            case 10: return motion.rotation.x();
            case 11: return motion.rotation.y();
            case 12: return motion.rotation.z();
            default: std::cerr << "invalid State property " << idx << std::endl;
                     throw "invalid property index";
        }
    }

    ////////////////////////////////////////////////////////////////////////////

    // const getter
    Scalar attr(const size_t& idx) const
    {
        switch (idx) {
            case  0: return pose.position.x();
            case  1: return pose.position.y();
            case  2: return pose.position.z();
            case  3: return pose.orientation.x();
            case  4: return pose.orientation.y();
            case  5: return pose.orientation.z();
            case  6: return pose.orientation.w();
            case  7: return motion.velocity.x();
            case  8: return motion.velocity.y();
            case  9: return motion.velocity.z();
            case 10: return motion.rotation.x();
            case 11: return motion.rotation.y();
            case 12: return motion.rotation.z();
            default: std::cerr << "invalid State property " << idx << std::endl;
                     throw "invalid property index";
        }
    }

    ////////////////////////////////////////////////////////////////////////////

    static QStringList get_attr_names()
    {
        QStringList headers;
        headers
            << "Position X"
            << "Position Y"
            << "Position Z"
            << "Rotation X"
            << "Rotation Y"
            << "Rotation Z"
            << "Rotation W"
            << "Lin. Velocity X"
            << "Lin. Velocity Y"
            << "Lin. Velocity Z"
            << "Ang. Velocity X"
            << "Ang. Velocity Y"
            << "Ang. Velocity Z";
        return headers;
    }

    ////////////////////////////////////////////////////////////////////////////

    static void init_table(QTableWidget& table);
    void set_table(const QTableWidget& table) const;
    static State from_table(const QTableWidget& table);

    ////////////////////////////////////////////////////////////////////////////

    template <typename T>
    State<T> cast() const {
        return State<T>(stamp, pose.template cast<T>(), motion.template cast<T>());
    }


    ////////////////////////////////////////////////////////////////////////////

    ros::Time stamp;
    Pose<Scalar> pose;
    Motion<Scalar> motion;
    bool valid;

    ////////////////////////////////////////////////////////////////////////////

    template <typename S>
    friend std::ostream& operator<<(std::ostream& out, const State<S>& s);

    /*
    bool operator==(const State<Scalar>& other) const {
        return this->stamp  == other.stamp
            && this->pose   == other.pose
            && this->motion == other.motion
            && this->valid  == other.valid;
    }
    */
};

////////////////////////////////////////////////////////////////////////////////

#endif /* end of include guard: POINT_H_Y4NWHRFC */
