#include "husky_highlevel_controller/HuskyHighlevelController.hpp"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>


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

void HuskyHighlevelController::laserscanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
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
    
    // Initialize subscriber
    laserscan_sub = nodeHandle_.subscribe(topic, queue_size, &HuskyHighlevelController::laserscanCallback, this);

    // Initialize publisher
    cmd_vel_pub = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    return true;
}

void HuskyHighlevelController::updateCommandVelocity(double linear, double angular)
{
    geometry_msgs::Twist cmd_vel;

    cmd_vel.linear.x = linear;
    cmd_vel.angular.z = angular;

    cmd_vel_pub.publish(cmd_vel);
}

void HuskyHighlevelController::controlLoop(void)
{
    updateCommandVelocity(MAX_VELOCITY, MAX_ANGULAR);
}

/*  getParameter Functions cause the code to interfere with subscriptions and
 *  not outputing responses.
 */  
// Function getParameter: string
/*
void HuskyHighlevelController::getParameter(std::string parameter, std::string container)
{
    if (!nodeHandle_.getParam(parameter, container))
    {
        ROS_ERROR_STREAM("Could not get parameter: " <<  parameter);
    }
    ROS_INFO_STREAM("Parameter loaded: " << parameter << " = " << container << std::endl);
}
*/

// Function getParameter: int
/*
void HuskyHighlevelController::getParameter(std::string parameter, int container)
{
    if (!nodeHandle_.getParam(parameter, container))
    {
        ROS_ERROR_STREAM("Could not get parameter: " <<  parameter);
    }
    ROS_INFO_STREAM("Parameter loaded: " << parameter << " = " << container << std::endl);
}
*/

}; // Namespace: husky_highlevel_controller
