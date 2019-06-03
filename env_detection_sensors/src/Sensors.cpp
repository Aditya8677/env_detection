#include <env_detection_sensors/Sensors.hpp>

/* 
 *  Environment detection system
 *
 *  Class EnvDetection is an entry point of the system.
 *
 *
 */

namespace env_detection {

Sensors::Sensors(ros::NodeHandle& nodeHandle, bool& success) 
    : nodeHandle_(nodeHandle)    
{
    if (!readParameters()) 
    {
        success = false;
        return;
    }

    sensorsSubscriber_ = nodeHandle_.subscribe(sensorsTopic_, 10, &Sensors::sensorsCallback, this);
    sensorsPublisher_ = nodeHandle_.advertise<env_detection_msgs::EnvValue>(EnvValueTopic_, 100, true);

    success = true;
}

EnvDetection::~EnvDetection()
{
}

bool Sensors::readParameters()
{
    if (!nodeHandle_.getParam("sensors_topic", sensorsTopic_))
    {
        ROS_ERROR("Could not read parameter 'sensorsTopic'.");
        return false;
    }     
    if (!nodeHandle_.getParam("env_value_topic", envValueTopic_))
    {
        ROS_ERROR("Could not read parameter 'envValueTopic'.");
        return false;
    }       

    return true;
}

void Sensors::sensorsCallback(const env_detection_msgs::Sensors& msg)
{
    map_.getIndex(position, indexPosition);
    map_.at("temperature", indexPosition) = msg.temperature;
    map_.at("humidity", indexPosition) = msg.humidity;
    map_.at("air_pressure", indexPosition) = msg.air_pressure;
    map_.at("uv_voltage", indexPosition) = msg.uv_voltage;
    map_.at("uv_index", indexPosition) = msg.uv_index;
    map_.at("roof", indexPosition) = msg.distance;

    EnvValue message = EnvValue();
    EnvValue.layer = "temperature";
    EnvValue.value = msg.temperature;
    envValuePublisher_.publish(EnvValue)
    EnvValue.layer = "humidity";
    EnvValue.value = msg.humidity;
    envValuePublisher_.publish(EnvValue)
    EnvValue.layer = "air_pressure";
    EnvValue.value = msg.air_pressure;
    envValuePublisher_.publish(EnvValue)
    EnvValue.layer = "uv_voltage";
    EnvValue.value = msg.uv_voltage;
    envValuePublisher_.publish(EnvValue)
    EnvValue.layer = "uv_index";
    EnvValue.value = msg.uv_index;
    envValuePublisher_.publish(EnvValue) 
    EnvValue.layer = "roof";
    EnvValue.value = msg.distance;
    envValuePublisher_.publish(EnvValue)               

}

}
