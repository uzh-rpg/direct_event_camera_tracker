#ifndef CAMERA_INTRINSICS_H_INCLUDED
#define CAMERA_INTRINSICS_H_INCLUDED

#include <Eigen/Dense>

#include <yaml-cpp/yaml.h>
#include <sensor_msgs/CameraInfo.h>

class CameraIntrinsics
{
public:
    CameraIntrinsics();
    //CameraIntrinsics(unsigned int W, unsigned int H, double focal, double near = 0, double far = 0);
    CameraIntrinsics(const Eigen::Vector2i& camera_size,
            const Eigen::Vector2d& principal_point,
            const Eigen::Vector2d& focal_length,
            const Eigen::Vector2d clipping = Eigen::Vector2d(0,0));

    CameraIntrinsics(const sensor_msgs::CameraInfo::ConstPtr& camera_info);

    CameraIntrinsics(YAML::Node config);

    void load_hand_eye(YAML::Node config);

    //void loadFromCSV(const std::string& filename);

    inline unsigned int getCameraWidth()  const { return camera_size.x(); }
    inline unsigned int getCameraHeight() const { return camera_size.y(); }
    inline const Eigen::Vector2i& getCameraSize() const { return camera_size; }

    inline double getPrincipalPointX()     const { return principal_point.x(); }
    inline double getPrincipalPointY()     const { return principal_point.y(); }
    inline const Eigen::Vector2d& getPrincipalPoint() const { return principal_point; }

    inline void setClipping(double near, double far) { clipping.x() = near; clipping.y() = far; }
    inline double getNearClipping()        const { return clipping.x(); }
    inline double getFarClipping()         const { return clipping.y(); }
    inline const Eigen::Vector2d& getFocalLength() const { return focal_length; }

    inline double getHorizontalAngle()     const { return 2 * atan( camera_size.x()/focal_length.x() ); }
    inline double getVerticalAngle()       const { return 2 * atan( camera_size.y()/focal_length.y() ); }

    inline Eigen::Isometry3d getHandEye() const { return T_HE_position*T_HE_orientation; }
    inline Eigen::Isometry3d getCamIMU()  const { Eigen::Isometry3d i; i.matrix() = T_cam_imu; return i; }

    void downsample2();

    friend std::ostream& operator <<(std::ostream &output, const CameraIntrinsics &intrinsics);

    Eigen::Vector4d getDistortion() const {
        Eigen::Vector4d d; d << k1, k2, r1, r2;
        return d;
    };

private:

    Eigen::Vector2i camera_size;        // size of image [pixels]
    Eigen::Vector2d principal_point;    // center of image [pixels]
    Eigen::Vector2d focal_length;       // [1/pixels or meter/pixels]

    // optional parameters
    Eigen::Vector2d clipping; // for mapping raw depth values to 'real' values (meters)

    // distortion
    double k1, k2, r1, r2;

    // hand_eye
    Eigen::Translation<double,3> T_HE_position;    // t
    Eigen::Quaternion<double>    T_HE_orientation; // r, attitude

    // IMU hand eye
    Eigen::Matrix4d T_cam_imu;
};

#endif /* end of include guard: CAMERA_INTRINSICS_H_INCLUDED */
