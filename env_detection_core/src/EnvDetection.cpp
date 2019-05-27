#include "env_detection_core/EnvDetection.hpp"

using namespace grid_map;

namespace env_detection {

EnvDetection::EnvDetection(ros::NodeHandle& nodeHandle, bool& success) 
    : nodeHandle_(nodeHandle)    
{
    if (!readParameters()) 
    {
        success = false;
        return;
    }

    inputMapSubscriber_ = nodeHandle_.subscribe(inputMapTopic_, 1, &EnvDetection::inputMapCallback, this);
    sensorsSubscriber_ = nodeHandle_.subscribe(sensorsTopic_, 10, &EnvDetection::sensorsCallback, this);

    map_ = GridMap({"base", "temperature", "humidity", "pressure", "uvlight", "roof"});
    map_.setFrameId("map");

    converter_ = GridMapRosConverter();

    setup_done_ = false;
    success = true;
}

EnvDetection::~EnvDetection()
{
}

bool EnvDetection::readParameters()
{
    if (!nodeHandle_.getParam("input_map_topic", inputMapTopic_))
    {
        ROS_ERROR("Could not read parameter 'inputMapTopic'.");
        return false;
    }

    if (!nodeHandle_.getParam("input_map_metadata_topic", inputMapMetaDataTopic_))
    {
        ROS_ERROR("Could not read parameter 'inputMapMetaDataTopic'.");
        return false;
    }

    if (!nodeHandle_.getParam("sensors_topic", sensorsTopic_))
    {
        ROS_ERROR("Could not read parameter 'sensorsTopic'.");
        return false;
    }        

    return true;
}

void EnvDetection::sensorsCallback(const env_detection_msgs::Sensors& msg)
{
    if (!setup_done_) return;
}

void EnvDetection::inputMapCallback(const nav_msgs::OccupancyGrid& msg)
{
    if (!converter_.fromOccupancyGrid(msg, "base", map_)) 
    {
        ROS_ERROR("Could not transform occupancy grid into the GridMap");
    } else 
    {
        ROS_INFO("MAP -- size (%f x %f m), %i x %i cells -- center (%f, %f) -- frame %s", map_.getLength().x(), map_.getLength().y(),
            map_.getSize()(0), map_.getSize()(1), map_.getPosition().x(), map_.getPosition().y(), map_.getFrameId().c_str());
        if (!setup_done_) setup_done_ = true;
    }    

}

}