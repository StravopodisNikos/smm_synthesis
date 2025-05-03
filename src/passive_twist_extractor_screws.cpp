#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <smm_screws/ScrewsKinematics.h>

// Run for test:
// $ rosrun smm_synthesis passive_twist_extractor_screws _input_file:=gspj0.yaml 
// OR
// $ roslaunch smm_synthesis extract_passive_twists.launch input_file:=gspj0.yaml

int main(int argc, char** argv)
{
    ros::init(argc, argv, "passive_twist_extractor_screws");
    ros::NodeHandle nh("~");

    // Read structure parameters
    std::string structure_file = ros::package::getPath("smm_synthesis") + "/config/yaml/assembly.yaml";
    YAML::Node assembly = YAML::LoadFile(structure_file);

    std::map<std::string, std::string> frame_map = {
        {"s2", "metalink_1_pseudo1_b"},
        {"s3", "metalink_1_pseudo2_b"},
        {"s5", "metalink_2_pseudo1_b"},
        {"s6", "metalink_2_pseudo2_b"}
    };

    std::vector<std::string> active_frames;
    for (const auto& param : frame_map)
    {
        if (assembly[param.first] && assembly[param.first].as<int>() != 9)
            active_frames.push_back(param.second);
    }

    if (active_frames.empty())
    {
        ROS_ERROR("No active passive joints found (s2, s3, s5, s6 all equal to 9).");
        return -1;
    }

    // Load input file
    std::string input_file;
    if (!nh.getParam("input_file", input_file))
    {
        ROS_ERROR("No input_file param provided.");
        return -1;
    }

    std::string input_path = ros::package::getPath("smm_synthesis") + "/config/yaml/" + input_file;
    YAML::Node frame_data = YAML::LoadFile(input_path);

    ScrewsKinematics kin;
    std::map<std::string, Eigen::Matrix<double, 6, 1>> twists;

    int index = 0;
    for (const auto& frame_name : active_frames)
    {
        if (!frame_data[frame_name])
        {
            ROS_WARN_STREAM("Frame " << frame_name << " not found in input file.");
            continue;
        }

        const YAML::Node& values = frame_data[frame_name];
        if (!values.IsSequence() || values.size() != 16)
        {
            ROS_WARN_STREAM("Skipping " << frame_name << " due to invalid format.");
            continue;
        }

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        for (int i = 0; i < 16; ++i)
            T(i / 4, i % 4) = values[i].as<double>();

        Eigen::Vector3d omega = T.block<3, 1>(0, 0);
        Eigen::Vector3d q = T.block<3, 1>(0, 3);

        Eigen::Matrix<double, 6, 1> twist = kin.createTwist(omega, q);
        std::string twist_key = "xi_p" + std::to_string(index++) + "_0";
        twists[twist_key] = twist;

        std::cout << "\nFrame: " << frame_name << " → " << twist_key;
        std::cout << "\nTwist: " << twist.transpose() << "\n";
    }

    // Save to xi_pi_anat.yaml
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

    std::string output_path = ros::package::getPath("smm_synthesis") + "/config/yaml/xi_pj_anat.yaml";
    std::ofstream fout(output_path);
    fout << out.c_str();
    fout.close();

    ROS_INFO_STREAM("Passive twists saved to: " << output_path);
    return 0;
}
