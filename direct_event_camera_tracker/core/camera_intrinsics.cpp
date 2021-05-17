#include "camera_intrinsics.h"

using namespace std;

///////////////////////////////////////////////////////////////////////////////

CameraIntrinsics::CameraIntrinsics()
  : camera_size(0,0),
    principal_point(0,0),
    focal_length(0, 0),
    clipping(0, 0),
    k1(0), k2(0), r1(0), r2(0),
    T_HE_position(0,0,0), T_HE_orientation(1,0,0,0),
    T_cam_imu(Eigen::Matrix4d::Identity())
{
}

///////////////////////////////////////////////////////////////////////////////

/*
CameraIntrinsics::CameraIntrinsics(unsigned int W, unsigned int H, double focal, double near, double far)
  : camera_width(W), camera_height(H),
    focal_length(focal), baseline(0), is_disparity(false),
    near_clipping(near), far_clipping(far)
{
    this->principal_point_x = this->camera_width  / 2.0f - 0.5;
    this->principal_point_y = this->camera_height / 2.0f - 0.5;
}
*/

///////////////////////////////////////////////////////////////////////////////
CameraIntrinsics::CameraIntrinsics(const Eigen::Vector2i& camera_size,
        const Eigen::Vector2d& principal_point,
        const Eigen::Vector2d& focal_length,
        const Eigen::Vector2d clipping)
  : camera_size(camera_size),
    principal_point(principal_point),
    focal_length(focal_length),
    clipping(clipping),
    k1(0), k2(0), r1(0), r2(0),
    T_HE_position(0,0,0), T_HE_orientation(1,0,0,0),
    T_cam_imu(Eigen::Matrix4d::Identity())
{
}

////////////////////////////////////////////////////////////////////////////////

CameraIntrinsics::CameraIntrinsics(const sensor_msgs::CameraInfo::ConstPtr& camera_info)
  : camera_size(camera_info->width, camera_info->height),
    principal_point(camera_info->K[2], camera_info->K[5]),
    focal_length(camera_info->K[0], camera_info->K[4]),
    clipping(0, 0),
    k1(0), k2(0), r1(0), r2(0),
    T_HE_position(0,0,0), T_HE_orientation(1,0,0,0),
    T_cam_imu(Eigen::Matrix4d::Identity())
{
    if (camera_info->D.size() >= 4) {
        k1 = camera_info->D[0];
        k2 = camera_info->D[1];
        r1 = camera_info->D[2];
        r2 = camera_info->D[3];

        if (camera_info->D.size() > 4 && camera_info->D[4] != 0) {
            cerr << "WARNING: CameraInfo from ROS contains non-zero r3 distortion parameter! This will be ignored." << endl;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

CameraIntrinsics::CameraIntrinsics(YAML::Node config)
  : camera_size(0,0),
    principal_point(0,0),
    focal_length(0, 0),
    clipping(0, 0),
    k1(0), k2(0), r1(0), r2(0),
    T_HE_position(0,0,0), T_HE_orientation(1,0,0,0),
    T_cam_imu(Eigen::Matrix4d::Identity())
{
    if (config["cam0"]) {
    // parse kalibr-style camchain file
        YAML::Node c = config["cam0"];

        // check a few things
        if (c["camera_model"].as<string>() != "pinhole") {
            cerr << "ERROR: only pinhole camera model is supported." << endl;
        }

        if (c["distortion_model"].as<string>() != "radtan") {
            cerr << "ERROR: only radtan (=plumb_bob) distortion model is supported." << endl;
        }

        camera_size.x()     = c["resolution"][0].as<int>();
        camera_size.y()     = c["resolution"][1].as<int>();
        focal_length.x()    = c["intrinsics"][0].as<double>();
        focal_length.y()    = c["intrinsics"][1].as<double>();
        principal_point.x() = c["intrinsics"][2].as<double>();
        principal_point.y() = c["intrinsics"][3].as<double>();

        k1 = c["distortion_coeffs"][0].as<double>();
        k2 = c["distortion_coeffs"][1].as<double>();
        r1 = c["distortion_coeffs"][2].as<double>();
        r2 = c["distortion_coeffs"][3].as<double>();

        if (c["distortion_coeffs"].size() > 4 and c["distortion_coeffs"][4].as<double>() != 0) {
            cerr << "WARNING: CameraInfo from YAML-file contains non-zero r3 distortion parameter! This will be ignored." << endl;
        }

        if (c["T_cam_imu"]) {
            YAML::Node T = c["T_cam_imu"];
            if (T.size() != 4) {
                cerr << "ERROR: Invalid camera imu transform in camera calibration." << endl;
            } else {
                for (size_t row = 0; row < 4; row++) {
                    if (T[row].size() != 4) {
                        cerr << "ERROR: Invalid camera imu transform in camera calibration." << endl;
                    } else {
                        for (size_t col = 0; col < 4; col++) {
                            T_cam_imu(row, col) = T[row][col].as<double>();
                        }
                    }
                }
            }
        }

    } else if (config["camera_name"]) {
        // ROS calibration format

        // check a few things
        if (config["distortion_model"].as<string>() != "plumb_bob") {
            cerr << "ERROR: only radtan (=plumb_bob) distortion model is supported." << endl;
        }

        camera_size.x()     = config["image_width"].as<int>();
        camera_size.y()     = config["image_height"].as<int>();
        focal_length.x()    = config["camera_matrix"]["data"][0].as<double>();
        focal_length.y()    = config["camera_matrix"]["data"][4].as<double>();
        principal_point.x() = config["camera_matrix"]["data"][2].as<double>();
        principal_point.y() = config["camera_matrix"]["data"][5].as<double>();

        k1 = config["distortion_coefficients"]["data"][0].as<double>();
        k2 = config["distortion_coefficients"]["data"][1].as<double>();
        r1 = config["distortion_coefficients"]["data"][2].as<double>();
        r2 = config["distortion_coefficients"]["data"][3].as<double>();

        if (config["distortion_coefficients"]["data"].size() > 4 and config["distortion_coefficients"]["data"][4].as<double>() != 0) {
            cerr << "WARNING: CameraInfo from YAML-file contains non-zero r3 distortion parameter! This will be ignored." << endl;
        }


    } else {
        throw "ERROR: Invalid camera calibration file. Please use kalibr or ROS format.";
    }

    if (camera_size.x() == 0 || camera_size.y() == 0) {
        throw "ERROR: Something went wrong while loading camera intrinsics: Camera size is zero.";
    }
}

///////////////////////////////////////////////////////////////////////////////

void CameraIntrinsics::load_hand_eye(YAML::Node config)
{
    YAML::Node he = config["calibration"];
    T_HE_position = Eigen::Translation3d(
            he["translation"]["x"].as<double>(),
            he["translation"]["y"].as<double>(),
            he["translation"]["z"].as<double>()
        );
    T_HE_orientation = Eigen::Quaterniond(
            he["rotation"]["w"].as<double>(),
            he["rotation"]["x"].as<double>(),
            he["rotation"]["y"].as<double>(),
            he["rotation"]["z"].as<double>()
        );

    //cout << "loaded hand-eye transformation:" << endl;
    //cout << getHandEye().matrix() << endl;
}

///////////////////////////////////////////////////////////////////////////////

/*
void CameraIntrinsics::loadFromCSV(const std::string& filename)
{
    // read file
    ///////////////////////////////////

    ifstream inputfile(filename.c_str(), ifstream::in);

    if (!inputfile.is_open()) {
        // TODO: throw error 404
        cerr << "could not open CSV file '" << filename << "'" << endl;
        return;
    }

    // get second line with data
    string line;
    getline(inputfile, line);
    getline(inputfile, line);
    inputfile.close();

    assert(!line.empty());

    // parse data
    ///////////////////////////////////

    tokenizer< escaped_list_separator<char> > tok(line);

    tokenizer<escaped_list_separator<char> >::iterator it = tok.begin();

    this->focal_length  = lexical_cast<double>(trim(*it++));
    ++it; // skip focal length in mm
    this->camera_width  = lexical_cast<double>(trim(*it++)); // actually an uint, but Matlab/Blender writes this as double
    this->camera_height = lexical_cast<double>(trim(*it++));
    ++it; // skip sensor height
    ++it; // skip sensor width
    this->near_clipping = lexical_cast<double>(trim(*it++));
    this->far_clipping  = lexical_cast<double>(trim(*it++));
    // skip rest of csv

    // calculate other parameters
    ///////////////////////////////////

    // [0, 0]     is at the center of the first pixel at the top left
    // [W-1, H-1] is at the center of the last pixel at the bottom right
    // i.e. integer coordinates directly corespond to pixel values
    this->principal_point_x = this->camera_width  / 2.0f - 0.5;
    this->principal_point_y = this->camera_height / 2.0f - 0.5;
}
*/

///////////////////////////////////////////////////////////////////////////////

void CameraIntrinsics::downsample2()
{
    this->camera_size     /= 2;
    this->principal_point /= 2;
    this->focal_length    /= 2;
}

///////////////////////////////////////////////////////////////////////////////

std::ostream& operator <<(std::ostream &output, const CameraIntrinsics &intrinsics)
{
    output << "Camera Intrinsics:" << endl;
    output << "size = " << intrinsics.camera_size << endl;
    output << "focal length = " << intrinsics.focal_length << endl;
    output << "principal point = " << intrinsics.principal_point << endl;
    output << "clipping = " << intrinsics.clipping << endl;

    return output;
}

////////////////////////////////////////////////////////////////////////////////

