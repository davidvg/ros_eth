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

    // Initiate laser params to not obtained from laser messages
    //laser_params.ready = false;
}

// Destructor
HuskyHighlevelController::~HuskyHighlevelController()
{
    ROS_INFO("HuskyHighlevelController stopped.");
    ros::shutdown();
}

bool HuskyHighlevelController::init(void)
{
    ROS_DEBUG("HuskyHighlevelController::init called.");
    // Get parameter: laser/topic
    std::string laser_topic = getParameterString("laser/topic");
    // Get parameter: laser/queue_size
    int laser_queue_size = getParameterInt("laser/queue_size");
    // Get parameter: controller/topic
    std::string controller_topic = getParameterString("controller/topic");
    // Get parameter: controller/queue_size
    int controller_queue_size = getParameterInt("controller/queue_size");
    // Get parameter: controller/p_vel
    controller_p_vel = getParameterDouble("controller/p_vel");
    // Get parameter: controller/p_ang
    controller_p_ang = getParameterDouble("controller/p_ang");

    
    // Initialize subscriber
    laserscan_sub = nodeHandle_.subscribe(
            laser_topic,
            laser_queue_size, 
            &HuskyHighlevelController::laserscanCallback, 
            this);

    // Initialize publisher
    cmd_vel_pub = nodeHandle_.advertise<geometry_msgs::Twist>(
            controller_topic, 
            controller_queue_size);

    return true;
}

void HuskyHighlevelController::laserscanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    ROS_DEBUG("HuskyHighlevelController::laserscanCallback called.");
    // Store the message in the class
    laserResponse = *msg;
    ROS_DEBUG("Message from laserscanCallback stored in HuskyHighlevelController::laserResponse");
    
    // Check if laser parameters have been obtained from messages
    // This way parameters are read only once.
    // Should this be in other place?
    if (!laser_params.ready) {
        ROS_DEBUG("Obtaining laser parameters from laser message.");
        getLaserParameters();
        ROS_ASSERT_MSG(laser_params.ready, "Laser parameters not obtained.");
    }
}

void HuskyHighlevelController::updateCommandVelocity(double linear, double angular)
{
    ROS_DEBUG("HuskyHighlevelController::updateCommandVelocity called.");

    // Compute linear and angular velocity from laser measurement
    double min_val = laserResponse.range_max;
    int min_val_ix = 0;
    size_t num_elems = laserResponse.ranges.size();
    for (size_t i=0; i<num_elems; i++)
    {
        if (laserResponse.ranges[i] < min_val)
        {
            min_val = laserResponse.ranges[i];
            min_val_ix = i;
        }
    }
    // Check minimum distance is in range
    min_val = ((min_val < laserResponse.range_min) ? laserResponse.range_min : min_val ); 
    
    min_distance = min_val;
    min_distance_ix = min_val_ix;
    ROS_DEBUG("Minimum laser measurement = %f (ix = %d)", min_distance, min_val_ix);

    // Generate control message
    geometry_msgs::Twist cmd_vel;

    // Linear velocity
    cmd_vel.linear.x = controller_p_vel * min_distance;
    ROS_DEBUG("Computed linear velocity: %f", cmd_vel.linear.x);
    if (cmd_vel.linear.x > MAX_LINEAR)
    {
        cmd_vel.linear.x = MAX_LINEAR;
        ROS_DEBUG("Linear velocity limited to %f", MAX_LINEAR);
    }

    // Angular velocity
    double angle = (min_distance_ix * laserResponse.angle_increment) + laserResponse.angle_min;
    angle = -angle;
    ROS_DEBUG("Computed direction: %f rad (%f deg)", angle, angle*180/3.1415);
    cmd_vel.angular.z = controller_p_ang * angle;
    ROS_DEBUG("Computed angular velocity: %f", cmd_vel.angular.z);
    if (cmd_vel.angular.z > MAX_ANGULAR)
    {
        cmd_vel.angular.z = MAX_ANGULAR;
        ROS_DEBUG("Angular velocity limited to %f", MAX_ANGULAR);
    }

    ROS_DEBUG("Publishing linear velocity:  [%f, %f, %f]",
            cmd_vel.linear.x,
            cmd_vel.linear.y,
            cmd_vel.linear.z);
    ROS_DEBUG("Publishing angular velocity: [%f, %f, %f]",
            cmd_vel.angular.x,
            cmd_vel.angular.y,
            cmd_vel.angular.z);
    
    cmd_vel_pub.publish(cmd_vel);
}

void HuskyHighlevelController::getLaserParameters(void)
{
    ROS_DEBUG("HuskyHighlevelController::getLaserParameters called.");
    laser_params.range_min = laserResponse.range_min;
    ROS_INFO_STREAM("Read parameter from laser: range_min = " << laser_params.range_min);
    laser_params.range_max = laserResponse.range_max;
    ROS_INFO_STREAM("Read parameter from laser: range_max = " << laser_params.range_max);
    laser_params.angle_min = laserResponse.angle_min;
    ROS_INFO_STREAM("Read parameter from laser: angle_min = " << laser_params.angle_min);
    laser_params.angle_max = laserResponse.angle_max;
    ROS_INFO_STREAM("Read parameter from laser: angle_max = " << laser_params.angle_max);
    laser_params.angle_increment = laserResponse.angle_increment;
    ROS_INFO_STREAM("Read parameter from laser: angle_increment = " << laser_params.angle_increment);
    // Compute number of elements
    laser_params.num_measurements = laserResponse.ranges.size();
    ROS_INFO_STREAM("Read parameter from laser: num_measurements = " << laser_params.num_measurements);
    // Mark parameters as obtained and skip for the following loops
    laser_params.ready = true;
}

void HuskyHighlevelController::controlLoop(void)
{
    ROS_DEBUG("HuskyHighlevelController::updateCommandVelocity called.");
    updateCommandVelocity(MAX_LINEAR, MAX_ANGULAR);
}

// Function getParameter: string
std::string HuskyHighlevelController::getParameterString(std::string parameter)
{
    std::string container;
    if (!nodeHandle_.getParam(parameter, container))
    {
        ROS_ERROR_STREAM("Could not get parameter: " <<  parameter);
    }
    ROS_INFO_STREAM("Parameter loaded: " << parameter << " = " << container << std::endl);
    return container; 
}

// Function getParameter: int
int HuskyHighlevelController::getParameterInt(std::string parameter)
{
    int container;
    if (!nodeHandle_.getParam(parameter, container))
    {
        ROS_ERROR_STREAM("Could not get parameter: " <<  parameter);
    }
    ROS_INFO_STREAM("Parameter loaded: " << parameter << " = " << container << std::endl);
    return container;
}

// Function getParameter: double
double HuskyHighlevelController::getParameterDouble(std::string parameter)
{
    double container;
    if (!nodeHandle_.getParam(parameter, container))
    {
        ROS_ERROR_STREAM("Could not get parameter: " <<  parameter);
    }
    ROS_INFO_STREAM("Parameter loaded: " << parameter << " = " << container << std::endl);
    return container;
}

}; // Namespace: husky_highlevel_controller
