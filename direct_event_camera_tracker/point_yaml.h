#ifndef POINT_YAML_H_EK4MODSF
#define POINT_YAML_H_EK4MODSF

#include "point.h"
#include <ros/time.h>

// various conversion routines to make it easy to load/store classes as YAML

namespace YAML {

////////////////////////////////////////////////////////////////////////////////
// ros::Time

template <>
struct convert<ros::Time> {
    static Node encode(const ros::Time& rhs) {
        Node node;
        node["sec"]  = rhs.sec;
        node["nsec"] = rhs.nsec;
        return node;
    }

    static bool decode(const Node& node, ros::Time& rhs) {
        if (!node.IsMap() || node.size() != 2) {
            std::cerr << "ERROR: Cannot convert YAML to ros::Time: wrong type (" << node.Type() << ") or size (" << node.size() << ")" << std::endl;
            return false;
        }

        if (!node["sec"] || !node["nsec"]) {
            std::cerr << "ERROR: Cannot convert YAML to ros::Time: Missing field." << std::endl;
            return false;
        }

        rhs = ros::Time(node["sec"].as<long int>(), node["nsec"].as<long int>());

        return true;
    }
};

////////////////////////////////////////////////////////////////////////////////
// POSE

template <typename Scalar>
struct convert<Pose<Scalar>> {
    static Node encode(const Pose<Scalar>& rhs) {
        Node node;
        node["translation"]["x"] = rhs.position.x();
        node["translation"]["y"] = rhs.position.y();
        node["translation"]["z"] = rhs.position.z();
        node["rotation"]["x"] = rhs.orientation.x();
        node["rotation"]["y"] = rhs.orientation.y();
        node["rotation"]["z"] = rhs.orientation.z();
        node["rotation"]["w"] = rhs.orientation.w();
        return node;
    }

    static bool decode(const Node& node, Pose<Scalar>& rhs) {
        if (!node.IsMap() || node.size() != 2) {
            std::cerr << "ERROR: Cannot convert YAML to Pose: wrong type (" << node.Type() << ") or size (" << node.size() << ")" << std::endl;
            return false;
        }

        if (!node["translation"] || !node["rotation"]) {
            std::cerr << "ERROR: Cannot convert YAML to Pose: missing field." << std::endl;
            return false;
        }

        if (!node["translation"].IsMap() || node["translation"].size() != 3) {
            std::cerr << "ERROR: Cannot convert YAML to Motion: Translation is ill formed." << std::endl;
            return false;
        }

        if (!node["rotation"].IsMap() || node["rotation"].size() != 4) {
            std::cerr << "ERROR: Cannot convert YAML to Motion: Rotation is ill formed." << std::endl;
            return false;
        }

        rhs = Pose<Scalar>(
                Eigen::Translation<Scalar,3>(
                    node["translation"]["x"].as<Scalar>(),
                    node["translation"]["y"].as<Scalar>(),
                    node["translation"]["z"].as<Scalar>()
                    ),
                Eigen::Quaternion<Scalar>(
                    node["rotation"]["w"].as<Scalar>(),
                    node["rotation"]["x"].as<Scalar>(),
                    node["rotation"]["y"].as<Scalar>(),
                    node["rotation"]["z"].as<Scalar>()
                    )
                );

        return true;
    }
};

////////////////////////////////////////////////////////////////////////////////
// MOTION

template <typename Scalar>
struct convert<Motion<Scalar>> {
    static Node encode(const Motion<Scalar>& rhs) {
        Node node;
        node["velocity"]["x"] = rhs.velocity.x();
        node["velocity"]["y"] = rhs.velocity.y();
        node["velocity"]["z"] = rhs.velocity.z();
        node["rotation"]["x"] = rhs.rotation.x();
        node["rotation"]["y"] = rhs.rotation.y();
        node["rotation"]["z"] = rhs.rotation.z();
        return node;
    }

    static bool decode(const Node& node, Motion<Scalar>& rhs) {
        if (!node.IsMap() || node.size() != 2) {
            std::cerr << "ERROR: Cannot convert YAML to Motion: wrong type (" << node.Type() << ") or size (" << node.size() << ")" << std::endl;
            return false;
        }

        if (!node["velocity"] || !node["rotation"]) {
            std::cerr << "ERROR: Cannot convert YAML to Motion: missing field." << std::endl;
            return false;
        }

        if (!node["velocity"].IsMap() || node["velocity"].size() != 3) {
            std::cerr << "ERROR: Cannot convert YAML to Motion: Velocity is ill formed." << std::endl;
            return false;
        }

        if (!node["rotation"].IsMap() || node["rotation"].size() != 3) {
            std::cerr << "ERROR: Cannot convert YAML to Motion: Rotation is ill formed." << std::endl;
            return false;
        }

        rhs = Motion<Scalar>(
                Eigen::Matrix<Scalar,3,1>(
                    node["velocity"]["x"].as<Scalar>(),
                    node["velocity"]["y"].as<Scalar>(),
                    node["velocity"]["z"].as<Scalar>()
                    ),
                Eigen::Matrix<Scalar,3,1>(
                    node["rotation"]["x"].as<Scalar>(),
                    node["rotation"]["y"].as<Scalar>(),
                    node["rotation"]["z"].as<Scalar>()
                    )
                );

        return true;
    }
};

////////////////////////////////////////////////////////////////////////////////
// STATE


template <typename Scalar>
struct convert<State<Scalar>> {
    static Node encode(const State<Scalar>& rhs) {
        Node node;
        node["stamp"]  = rhs.stamp;
        node["pose"]   = rhs.pose;
        node["motion"] = rhs.motion;
        return node;
    }

    static bool decode(const Node& node, State<Scalar>& rhs) {
        if (!node.IsMap() || node.size() != 3) {
            std::cerr << "ERROR: Cannot convert YAML to State: not a map (" << node.Type() << ") or not the correct size (" << node.size() << ")" << std::endl;
            return false;
        }

        if (!node["stamp"] || !node["pose"] || !node["motion"]) {
            std::cerr << "ERROR: Cannot convert YAML to State: missing field." << std::endl;
            return false;
        }

        rhs = State<Scalar>(
                node["stamp"].as<ros::Time>(),
                node["pose"].as<Pose<Scalar>>(),
                node["motion"].as<Motion<Scalar>>()
                );

        return true;
    }
};

////////////////////////////////////////////////////////////////////////////////

} // namespace YAML

////////////////////////////////////////////////////////////////////////////////

template <typename Scalar>
std::ostream& operator<<(std::ostream& out, const State<Scalar>& s)
{
    out << "State:" << (s.valid?"":"INVALID!") << std::endl;
    // convert to YAML and print
    YAML::Node n; n = s;
    out << n;
    return out;
}

////////////////////////////////////////////////////////////////////////////////

#endif /* end of include guard: POINT_YAML_H_EK4MODSF */
