/* */

#pragma once

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <string>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include "env_detection_msgs/Sensors.h"

using namespace grid_map;

namespace env_detection {

class EnvDetection  
{
    public:

    EnvDetection(ros::NodeHandle& nodeHandle, bool& success);

    virtual ~EnvDetection();

    bool readParameters();

    void sensorsCallback(const env_detection_msgs::Sensors& msg);
    void inputMapCallback(const nav_msgs::OccupancyGrid& msg);

    private:
    
    ros::NodeHandle& nodeHandle_;

    std::string sensorsTopic_;
    std::string inputMapTopic_;
    std::string inputMapMetaDataTopic_;

    ros::Subscriber sensorsSubscriber_;
    ros::Subscriber inputMapSubscriber_;

    GridMap map_;
    GridMapRosConverter converter_;
    bool setup_done_;

};

}