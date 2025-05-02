#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <smm_screws/ScrewsKinematics.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "twist_extractor_screws");
    ros::NodeHandle nh("~");

    // Get YAML file name from param
    std::string yaml_file;
    if (!nh.getParam("input_file", yaml_file))
    {
        ROS_ERROR("No input_file param provided.");
        return -1;
    }

    // Build input path
    std::string input_path = ros::package::getPath("smm_synthesis") + "/config/yaml/" + yaml_file;
    YAML::Node root = YAML::LoadFile(input_path);

    ScrewsKinematics kin;
    std::map<std::string, Eigen::Matrix<double, 6, 1>> twists;

    for (YAML::const_iterator it = root.begin(); it != root.end(); ++it)
    {
        std::string frame_name = it->first.as<std::string>();
        const YAML::Node& values = it->second;

        if (!values.IsSequence() || values.size() != 16)
        {
            ROS_WARN_STREAM("Skipping " << frame_name << " due to invalid format.");
            continue;
        }

        // Convert to Eigen 4x4 matrix
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        for (int i = 0; i < 16; ++i)
            T(i / 4, i % 4) = values[i].as<double>();

        // Extract q
        Eigen::Vector3d q = T.block<3,1>(0,3);

        // Extract omega
        Eigen::Vector3d omega;
        if (frame_name == "base_link")
            omega = T.block<3,1>(0,2);  // Z axis is the local rotation axis for configuration
        else
            omega = T.block<3,1>(0,0);  // X axis is the local rotation axis for configuration 

        Eigen::Matrix<double, 6, 1> twist = kin.createTwist(omega, q);
              
        twists[frame_name] = twist;

        std::cout << "\nFrame: " << frame_name << "\n";
        std::cout << "Twist: \n" << twist.transpose() << "\n";
    }

    // Save twists to YAML
    YAML::Emitter out;
    out << YAML::BeginMap;
    for (const auto& pair : twists)
    {
        out << YAML::Key << pair.first << YAML::Value << YAML::BeginSeq;
        for (int i = 0; i < 6; ++i)
            out << pair.second(i);
        out << YAML::EndSeq;
    }
    out << YAML::EndMap;

    std::string output_path = ros::package::getPath("smm_synthesis") + "/config/yaml/xi_ai_anat.yaml";
    std::ofstream fout(output_path);
    fout << out.c_str();
    fout.close();

    ROS_INFO_STREAM("Twists saved to: " << output_path);

    return 0;
}
