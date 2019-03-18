#include "husky_highlevel_controller/HuskyHighlevelController.hpp"
//#include <std_msgs/String.h>
//#include <std_msgs/Int16.h>
#include <sensor_msgs/LaserScan.h>


namespace husky_highlevel_controller {

// Constructor
HuskyHighlevelController::HuskyHighlevelController()
    : nodeHandle_("~")
{
    ROS_INFO("HuskyHighlevelController Init.");
    ROS_ASSERT(init());
}

// Destructor
HuskyHighlevelController::~HuskyHighlevelController()
{
    ROS_INFO("HuskyHighlevelController stopped.");
    ros::shutdown();
}

void HuskyHighlevelController::LaserscanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    int num_elem = msg->ranges.size();
    double min_distance = msg->range_max;

    // Index for the minimum measurement
    min_distance_ix = num_elem;

    // Search for minimum measurement
    for (size_t i=0; i<num_elem; i++)
    {
        //min_val = ((min_val < msg->ranges[i]) ? min_val : msg->ranges[i]);
        if (msg->ranges[i] < min_distance)
        {
            min_distance = msg->ranges[i];
            min_distance_ix = i;
        }
    }
    // Check minimum measurement is in range
    min_distance = ((min_distance < msg->range_min) ? msg->range_min : min_distance);

    ROS_INFO("Minimum laser measurement = %f (ix = %d)", min_distance, min_distance_ix);
    
    // Store measurement as a class member
    min_distance = min_distance;
}

bool HuskyHighlevelController::init(void)
{
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
    
    // Subscribe to topic
    laserscan_sub = nodeHandle_.subscribe(topic, queue_size, &HuskyHighlevelController::LaserscanCallback, this);

    return true;
}

// Function getParameter: string
/*
void HuskyHighlevelController::getParameter(std::string parameter, std::string container)
{
    if (!nodeHandle_.getParam(parameter, container))
    {
        ROS_ERROR("Could not get parameter: %s", parameter);
    }
}

// Function getParameter: int
void HuskyHighlevelController::getParameter(std::string parameter, int container)
{
    if (!nodeHandle_.getParam(parameter, container))
    {
        //ROS_ERROR("Could not get parameter: %s", parameter);
        ROS_ERROR("Could not get parameter: %s", parameter);
    }
}
*/

}; // Namespace: husky_highlevel_controller
