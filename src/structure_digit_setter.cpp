#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <string>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "structure_digit_setter");
    ros::NodeHandle nh;

    std::string yaml_path = ros::package::getPath("smm_synthesis") + "/config/yaml/assembly.yaml";

    YAML::Node assembly = YAML::LoadFile(yaml_path);
    int s2 = assembly["s2"].as<int>();
    int s3 = assembly["s3"].as<int>();
    int s5 = assembly["s5"].as<int>();
    int s6 = assembly["s6"].as<int>();

    int count = 0;
    if (s2 != 9) count++;
    if (s3 != 9) count++;
    if (s5 != 9) count++;
    if (s6 != 9) count++;

    if (count < 2 || count > 4) {
        ROS_ERROR_STREAM("[structure_digit_setter] Invalid structure count: " << count);
        return -1;
    }

    nh.setParam("/STRUCTURE_DIGIT", count);
    ROS_INFO_STREAM("[structure_digit_setter] Set STRUCTURE_DIGIT = " << count);

    return 0;
}
