#include <ros/ros.h>
#include <ros/package.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <fstream>
#include <string>
#include <map>

// FOR TEST RUN: $ rosrun smm_synthesis inertia_extractor_kdl _inertia_output_file:=Mscomi0.yaml
//           OR  $ roslaunch smm_synthesis extract_zero_inertia.launch inertia_output_file:=Mscomi0.yaml

struct AggregatedInertia {
    double mass = 0.0;
    Eigen::Vector3d weighted_com = Eigen::Vector3d::Zero();
    Eigen::Matrix3d inertia_world = Eigen::Matrix3d::Zero();
};

Eigen::Matrix3d KDLRotationToEigen(const KDL::Frame& frame) {
    Eigen::Matrix3d mat;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            mat(i,j) = frame.M(i,j);
    return mat;
}

Eigen::Matrix3d SkewSymmetric(const Eigen::Vector3d& v) {
    Eigen::Matrix3d skew;
    skew <<    0, -v(2),  v(1),
             v(2),    0, -v(0),
            -v(1),  v(0),    0;
    return skew;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "inertia_extractor_kdl");
    ros::NodeHandle nh("~");

    std::string output_filename;
    if (!nh.getParam("output_file", output_filename)) {
        ROS_ERROR("No output_file parameter given.");
        return -1;
    }

    ros::Duration(2.0).sleep();

    urdf::Model model;
    if (!model.initParam("robot_description")) {
        ROS_ERROR("Failed to parse URDF.");
        return -1;
    }

    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(model, kdl_tree)) {
        ROS_ERROR("Failed to construct KDL tree.");
        return -1;
    }

    urdf::LinkConstSharedPtr current_link = model.getRoot();
    std::string root_name = kdl_tree.getRootSegment()->first;
    if (!current_link) {
        ROS_ERROR("Root link is NULL.");
        return -1;
    }

    AggregatedInertia current_agg;
    std::string current_group = current_link->name;

    std::map<std::string, Eigen::Matrix<double, 6, 6>> spatial_inertias;

    std::map<std::string, std::string> output_key_map = {
        {"base_link", "Mscom00"},
        {"active_module_b_1", "Mscom10"},
        {"active_module_b_2", "Mscom20"}
    };

    while (current_link) {
        KDL::Chain chain;
        if (!kdl_tree.getChain(root_name, current_link->name, chain)) {
            ROS_ERROR_STREAM("Chain extraction failed for link: " << current_link->name);
            break;
        }

        KDL::Frame world_T = KDL::Frame::Identity();
        for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i)
            world_T = world_T * chain.getSegment(i).pose(0.0);

        Eigen::Matrix3d R_world = KDLRotationToEigen(world_T);
        Eigen::Vector3d p_world(world_T.p.x(), world_T.p.y(), world_T.p.z());

        if (current_link->inertial) {
            double m = current_link->inertial->mass;
            Eigen::Vector3d c_local(
                current_link->inertial->origin.position.x,
                current_link->inertial->origin.position.y,
                current_link->inertial->origin.position.z
            );

            Eigen::Matrix3d I_local;
            I_local << current_link->inertial->ixx, current_link->inertial->ixy, current_link->inertial->ixz,
                       current_link->inertial->ixy, current_link->inertial->iyy, current_link->inertial->iyz,
                       current_link->inertial->ixz, current_link->inertial->iyz, current_link->inertial->izz;

            Eigen::Vector3d c_world = p_world + R_world * c_local;
            Eigen::Matrix3d I_rotated = R_world * I_local * R_world.transpose();
            Eigen::Matrix3d I_shifted = I_rotated + m * ((c_world.dot(c_world)) * Eigen::Matrix3d::Identity() - c_world * c_world.transpose());

            current_agg.mass += m;
            current_agg.weighted_com += m * c_world;
            current_agg.inertia_world += I_shifted;
        }

        if (current_link->child_joints.empty()) {
            if (current_agg.mass > 0.0 && output_key_map.count(current_group)) {
                Eigen::Vector3d com = current_agg.weighted_com / current_agg.mass;
                Eigen::Matrix3d skew = SkewSymmetric(com);

                Eigen::Matrix<double, 6, 6> M;
                M.setZero();
                M.block<3,3>(0,0) = current_agg.mass * Eigen::Matrix3d::Identity();
                M.block<3,3>(0,3) = -skew;
                M.block<3,3>(3,0) = skew;
                M.block<3,3>(3,3) = current_agg.inertia_world;

                spatial_inertias[output_key_map[current_group]] = M;
            }
            break;
        }

        auto child_joint = current_link->child_joints[0];
        auto next_link = current_link->child_links[0];
        if (!child_joint || !next_link) break;

        if (child_joint->type == urdf::Joint::FIXED) {
            current_link = next_link;
        } else {
            if (current_agg.mass > 0.0 && output_key_map.count(current_group)) {
                Eigen::Vector3d com = current_agg.weighted_com / current_agg.mass;
                Eigen::Matrix3d skew = SkewSymmetric(com);

                Eigen::Matrix<double, 6, 6> M;
                M.setZero();
                M.block<3,3>(0,0) = current_agg.mass * Eigen::Matrix3d::Identity();
                M.block<3,3>(0,3) = -skew;
                M.block<3,3>(3,0) = skew;
                M.block<3,3>(3,3) = current_agg.inertia_world;

                spatial_inertias[output_key_map[current_group]] = M;
            }

            current_agg = AggregatedInertia();
            current_group = next_link->name;
            current_link = next_link;
        }
    }

    std::string save_path = ros::package::getPath("smm_synthesis") + "/config/yaml/" + output_filename;
    YAML::Emitter out;
    out << YAML::BeginMap;

    for (const auto& item : spatial_inertias) {
        out << YAML::Key << item.first << YAML::Value << YAML::BeginSeq;
        const auto& M = item.second;
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                out << M(i,j);
        out << YAML::EndSeq;
    }

    out << YAML::EndMap;
    std::ofstream fout(save_path);
    fout << out.c_str();
    fout.close();

    ROS_INFO_STREAM("Saved to " << save_path);
    ros::spin();
    return 0;
}
