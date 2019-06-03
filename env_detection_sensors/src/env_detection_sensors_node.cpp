#include <env_detection_sensors/Sensors.hpp>

#include <ros/ros.h>

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "environment_detection_sensors_node");
    ros::NodeHandle nodeHandle("~");
    bool success;
    env_detection::Sensors sensors(nodeHandle, success);
    if (!success) exit(1);
    ros::spin();
    return 0;
}