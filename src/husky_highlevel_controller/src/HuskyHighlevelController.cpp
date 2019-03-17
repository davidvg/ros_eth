#include "husky_highlevel_controller/HuskyHighlevelController.hpp"
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/LaserScan.h>


namespace husky_highlevel_controller {

// Constructor
HuskyHighlevelController::HuskyHighlevelController()
    : nodeHandle_("~")
{
    ROS_INFO("HuskyHighlevelController Init.");

    //ROS_ASSERT(init());
    // Get parameter: topic
    std::string topic;
    if (!nodeHandle_.getParam("laser/topic", topic)) 
    {
        ROS_ERROR("Could not get parameter: topic" );
    }
    // Get parameter: queue_size
    int queue_size;
    if (!nodeHandle_.getParam("laser/queue_size", queue_size)) 
    {
        ROS_ERROR("Could not get parameter: queue_size" );
    }

    laserscan_sub = nodeHandle_.subscribe(topic, queue_size, &HuskyHighlevelController::LaserscanCallback, this);
    
    // ^^^ Code above should be included in tht init() function ^^^
}

// Destructor
HuskyHighlevelController::~HuskyHighlevelController()
{
    ROS_INFO("HuskyHighlevelController stopped.");
    ros::shutdown();

}

// Function getParameter
// The container can be any type, so it's not easily generalized.
// There must be some way to do it.
/*
void HuskyHighlevelController::getParameter(std::string parameter, std::string container)
{
    if (!nodeHandle_.getParam(parameter, container))
    {
        ROS_ERROR("Could not get parameter: %s", parameter);
    }
}
*/

void HuskyHighlevelController::LaserscanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    int num_elem = sizeof(msg); 
    double min_val = msg->range_max;

    // Search for minimum measurement
    for (int i=0; i<num_elem; i++)
    {
        min_val = ((min_val > msg->ranges[i]) ? msg->ranges[i] : min_val);
    }
    // Check minimum measurement is in range
    min_val = ((min_val > msg->range_min) ? min_val : msg->range_min);

    ROS_INFO("Callback: Minimum measurement = %f", min_val);
    
    // Store measurement as a class member
    min_measure = min_val;
}

/*
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
    //ROS_INFO("Callback entered: laserscan_callback");
    int num_elem = sizeof(msg);
    float min_val = msg->range_max;

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
*/

}; // Namespace: husky_highlevel_controller
