/* */

#pragma once

#include <string>
#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
//#include <grid_map_ros/GridMapRosConverter.hpp>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <env_detection_msgs/Sensors.h>

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
    void publishGridMap();
    
    private:
    
    ros::NodeHandle& nodeHandle_;

    std::string sensorsTopic_;
    std::string inputMapTopic_;
    std::string inputMapMetaDataTopic_;
    std::string gridMapTopic_;

    std::string baseFrame_;
    std::string mapFrame_;

    ros::Subscriber sensorsSubscriber_;
    ros::Subscriber inputMapSubscriber_;

    ros::Publisher gridMapPublisher_;

    tf2_ros::TransformListener *transformListener_;
    tf2_ros::Buffer transformBuffer_;

    geometry_msgs::Transform pose_;

    GridMap map_;
    GridMapRosConverter converter_;


    bool setup_done_;

};

}