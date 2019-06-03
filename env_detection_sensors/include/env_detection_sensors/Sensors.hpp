/**
 *  Environment Detection System (EDS)
 * 
 *  Class EnvDetection
 * 
 *  Main Class of the EDS
 * 
 *  @author Petr Neduchal
 *  @mail neduchal@kky.zcu.cz
 *  
 */
#pragma once

// CPP Headers
#include <string>
// ROS headers
#include <ros/ros.h>
#include <env_detection_msgs/Sensors.h>
#include <env_detection_msgs/EnvValue.h>

namespace env_detection {

class Sensors  
{
    public:

    Sensors(ros::NodeHandle& nodeHandle, bool& success);

    virtual ~Sensors();

    bool readParameters();
    void sensorsCallback(const env_detection_msgs::Sensors& msg);

    
    private:
    
    ros::NodeHandle& nodeHandle_;

    std::string sensorsTopic_;
    std::string envValueTopic_;

    std::string baseFrame_;

    ros::Subscriber sensorsSubscriber_;
    ros::Publisher envValuePublisher_;

};

}