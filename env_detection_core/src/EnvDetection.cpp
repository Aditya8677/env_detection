#include <env_detection_core/EnvDetection.hpp>

/* 
 *  Environment detection system
 *
 *  Class EnvDetection is an entry point of the system.
 *
 *
 */
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

    envValueSubscriber_ = nodeHandle_.subscribe(envValueTopic_, 100, &EnvDetection::envValueCallback, this);
    inputMapSubscriber_ = nodeHandle_.subscribe(inputMapTopic_, 1, &EnvDetection::inputMapCallback, this);
    gridMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>(gridMapTopic_, 1, true);

    map_ = GridMap({"base"});
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

    if (!nodeHandle_.getParam("env_value_topic", envValueTopic_))
    {
        ROS_ERROR("Could not read parameter 'sensorsTopic'.");
        return false;
    }  
    nodeHandle_.param<std::string>("base_frame", baseFrame_, "base_link");
    nodeHandle_.param<std::string>("map_frame", mapFrame_, "map");
    nodeHandle_.param<std::string>("grid_map_topic", gridMapTopic_, "env_grid_map");

    return true;
}

void EnvDetection::envValueCallback(const env_detection_msgs::EnvValue& msg)
{
    if (!setup_done_) return;
    if (!map_.exists(msg.layer)) {
        map_.add(msg.layer);
    }    
    Position position(pose_.translation.x, pose_.translation.y);
    Index indexPosition;
    map_.getIndex(position, indexPosition);
    map_.at(msg.layer, indexPosition) = msg.value;
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

void EnvDetection::publishGridMap()
{
    ros::Time time = ros::Time::now();
    map_.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap msg;
    converter_.toMessage(map_, msg);
    gridMapPublisher_.publish(msg);
    ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", msg.info.header.stamp.toSec());
}


void EnvDetection::getLayers(env_detection_msgs::GetLayers::Request &req, env_detection_msgs::GetLayers::Response &res)
{
    std::vector<std::string> layers = map_.getLayers();
    for(std::size_t i=0; i < layers.size(); ++i)
    {
        res.layers.push_back(layers[i]);
    }
}

void EnvDetection::getLayer(env_detection_msgs::GetLayer::Request &req, env_detection_msgs::GetLayer::Response &res)
{
    GridMap map_msg = GridMap();
    grid_map_msgs::GridMap msg;

    if (map_.exists(req.layer))
    {
        std::vector<std::string> layers;
        layers.push_back("base");
        layers.push_back(req.layer);
        res.valid = true;
        map_msg.addDataFrom(map_, true, true, false, layers);
        converter_.toMessage(map_msg, msg);
        res.map = msg;
    } else
    {
        res.valid = false;
    }
}

}