#include <env_detection_core/EnvDetection.hpp>

#include <ros/ros.h>

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "environment_detection_basic_node");
    ros::NodeHandle nodeHandle("~");
    bool success;
    env_detection::EnvDetection envDetection(nodeHandle, success);
    if (!success) exit(1);

    ros::Rate rate(5.0);
    while(nodeHandle.ok())
    {
        envDetection.publishGridMap();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}