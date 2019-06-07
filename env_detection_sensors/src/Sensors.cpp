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
    envValuePublisher_ = nodeHandle_.advertise<env_detection_msgs::EnvValue>(envValueTopic_, 100, true);

    success = true;
}

Sensors::~Sensors()
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
    env_detection_msgs::EnvValue message = env_detection_msgs::EnvValue();
    message.layer = "temperature";
    message.value = msg.temperature;
    envValuePublisher_.publish(message);
    message.layer = "humidity";
    message.value = msg.humidity;
    envValuePublisher_.publish(message);
    message.layer = "air_pressure";
    message.value = msg.air_pressure;
    envValuePublisher_.publish(message);
    message.layer = "uv_voltage";
    message.value = msg.uv_voltage;
    envValuePublisher_.publish(message);
    message.layer = "uv_index";
    message.value = msg.uv_index;
    envValuePublisher_.publish(message);
    message.layer = "roof";
    message.value = msg.distance;
    envValuePublisher_.publish(message);
}

}
