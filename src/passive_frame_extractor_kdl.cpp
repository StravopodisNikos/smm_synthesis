#include <ros/ros.h>
#include <ros/package.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <map>

Eigen::Matrix4d KDLFrameToEigen(const KDL::Frame& frame)
{
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            mat(i, j) = frame.M(i, j);
    mat(0, 3) = frame.p.x();
    mat(1, 3) = frame.p.y();
    mat(2, 3) = frame.p.z();
    return mat;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "passive_frame_extractor_kdl");
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh_global("/");

    std::string output_filename;
    if (!nh_private.getParam("output_file", output_filename)) {
        ROS_ERROR("No output_file parameter given.");
        return -1;
    }

    int s2, s3, s5, s6;
    if (!nh_global.getParam("s2", s2) ||
        !nh_global.getParam("s3", s3) ||
        !nh_global.getParam("s5", s5) ||
        !nh_global.getParam("s6", s6)) {
        ROS_ERROR("Missing one or more structure parameters (s2, s3, s5, s6).");
        return -1;
    }

    std::vector<std::string> frames_to_extract;
    if (s2 != 9) frames_to_extract.push_back("metalink_1_pseudo1_b");
    if (s3 != 9) frames_to_extract.push_back("metalink_1_pseudo2_b");
    if (s5 != 9) frames_to_extract.push_back("metalink_2_pseudo1_b");
    if (s6 != 9) frames_to_extract.push_back("metalink_2_pseudo2_b");

    urdf::Model model;
    if (!model.initParam("robot_description")) {
        ROS_ERROR("Failed to parse URDF from parameter server.");
        return -1;
    }

    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(model, kdl_tree)) {
        ROS_ERROR("Failed to construct KDL tree.");
        return -1;
    }

    std::string root_name = "base_link";
    ROS_INFO_STREAM("Setting root frame to: " << root_name << " with rotz(-pi) compensation");

    std::map<std::string, Eigen::Matrix4d> extracted_transforms;

    // Define rotz(-pi)
    Eigen::Matrix4d Rz_pi = Eigen::Matrix4d::Identity();
    Rz_pi(0,0) = -1.0;
    Rz_pi(1,1) = -1.0;

    for (const auto& frame_name : frames_to_extract)
    {
        KDL::Chain chain;
        if (!kdl_tree.getChain(root_name, frame_name, chain)) {
            ROS_ERROR_STREAM("Failed to extract chain from " << root_name << " to " << frame_name);
            continue;
        }

        KDL::ChainFkSolverPos_recursive fk_solver(chain);
        KDL::JntArray q(chain.getNrOfJoints());
        for (unsigned int i = 0; i < chain.getNrOfJoints(); ++i)
            q(i) = 0.0;

        KDL::Frame tf;
        if (fk_solver.JntToCart(q, tf) >= 0) {
            Eigen::Matrix4d tf_eigen = KDLFrameToEigen(tf);
            Eigen::Matrix4d transformed = Rz_pi * tf_eigen;
            extracted_transforms[frame_name] = transformed;

            std::cout << "\n Spatial Transform (" << root_name << " -> " << frame_name << ") with rotz(-pi):\n";
            std::cout << transformed << "\n";
        } else {
            ROS_WARN_STREAM("FK failed for frame: " << frame_name);
        }
    }

    std::string path = ros::package::getPath("smm_synthesis") + "/config/yaml/" + output_filename;
    YAML::Emitter out;
    out << YAML::BeginMap;
    for (const auto& entry : extracted_transforms)
    {
        out << YAML::Key << entry.first << YAML::Value << YAML::BeginSeq;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                out << entry.second(i, j);
        out << YAML::EndSeq;
    }
    out << YAML::EndMap;

    std::ofstream fout(path);
    fout << out.c_str();
    fout.close();

    ROS_INFO_STREAM("Saved selected passive frame transforms to: " << path);
    return 0;
}
