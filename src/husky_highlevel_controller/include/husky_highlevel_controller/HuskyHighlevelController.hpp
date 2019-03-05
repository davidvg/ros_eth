#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

namespace husky_highlevel_controller {

// Class containing the HuskyHighlevelController
class HuskyHighlevelController 
{
    public:
    // Constructor
    HuskyHighlevelController(ros::NodeHandle& nodeHandle);
    // Destructor
    virtual ~HuskyHighlevelController();

    static void laserscan_callback(const sensor_msgs::LaserScan::ConstPtr&);

    private:
    ros::NodeHandle nodeHandle_;
};


} // Namespace: husky_highlevel_controller
