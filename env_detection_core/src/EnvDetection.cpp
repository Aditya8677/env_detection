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

    map_ = GridMap({"base", "temperature", "humidity", "air_pressure", "uv_voltage", "uv_light", "roof"});
    map_.setBasicLayers({"base"});
    map_.setFrameId("map");

    transformListener_ = new tf2_ros::TransformListener(transformBuffer_);

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
    nodeHandle_.param<std::string>("base_frame", baseFrame_, "base_link");
    nodeHandle_.param<std::string>("map_frame", mapFrame_, "map");

    return true;
}

void EnvDetection::sensorsCallback(const env_detection_msgs::Sensors& msg)
{
    if (!setup_done_) return;
    geometry_msgs::TransformStamped transform;
    try
    {
        transform = transformBuffer_.lookupTransform(mapFrame_, baseFrame_, ros::Time(0));
        pose_= transform.transform;
    } catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());    
    }
    Position position(pose_.translation.x, pose_.translation.y);
    Index indexPosition; 
    map_.getIndex(position, indexPosition);
    map_.at("temperature", indexPosition) = msg.temperature;
    map_.at("humidity", indexPosition) = msg.humidity;
    map_.at("air_pressure", indexPosition) = msg.air_pressure;
    map_.at("uv_voltage", indexPosition) = msg.uv_voltage;
    map_.at("uv_index", indexPosition) = msg.uv_index;
    map_.at("roof", indexPosition) = msg.distance;

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