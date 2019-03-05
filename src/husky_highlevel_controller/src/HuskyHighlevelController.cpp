#include "husky_highlevel_controller/HuskyHighlevelController.hpp"
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>                                  // fmin

namespace husky_highlevel_controller {

// Constructor
HuskyHighlevelController::HuskyHighlevelController(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle)
{
    nodeHandle_ = nodeHandle_;
}

// Destructor
HuskyHighlevelController::~HuskyHighlevelController()
{

}

// Callback
void HuskyHighlevelController::laserscan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    ROS_INFO("Callback entered: laserscan_callback");
    int num_elem = sizeof(msg);
    float min_val = 9999.99;

    // Search for the minimum value in the array
    for (int i=0; i<num_elem; i++)
    {
        if (msg->ranges[i] < min_val) 
        {
            min_val = msg->ranges[i];
        }
    }
    // Check minimun distance is in range
    min_val = ((min_val > msg->range_min) ? min_val : msg->range_min);

    ROS_INFO("Minimum measurement: %f", min_val);
}

}; // Namespace: husky_highlevel_controller
