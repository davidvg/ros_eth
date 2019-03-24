#include "husky_highlevel_controller/HuskyHighlevelController.hpp"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <math.h>


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

    // Initialize cmd_vel publisher
    cmd_vel_pub = nodeHandle_.advertise<geometry_msgs::Twist>(
            controller_topic, 
            controller_queue_size);

    // Initialize marker publisher
    marker_pub = nodeHandle_.advertise<visualization_msgs::Marker>(
            "visualization_marker",
            0);

    return true;
}

void HuskyHighlevelController::laserscanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    ROS_DEBUG("HuskyHighlevelController::laserscanCallback called.");
    // Store the message in the class
    laserResponse = *msg;
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
    ROS_DEBUG("Minimum laser measurement = %.2f m (ray number %d)", min_distance, min_val_ix);

    // Generate control message
    geometry_msgs::Twist cmd_vel;

    // Linear velocity
    cmd_vel.linear.x = controller_p_vel * min_distance;
    ROS_DEBUG("Computed linear velocity: %.2f m/s", cmd_vel.linear.x);
    if (cmd_vel.linear.x > MAX_LINEAR)
    {
        cmd_vel.linear.x = MAX_LINEAR;
        ROS_DEBUG("Linear velocity limited to %.2f m/s", MAX_LINEAR);
    }

    // Angular velocity
    min_distance_angle = (min_distance_ix * laserResponse.angle_increment) + laserResponse.angle_min;
    // Frames base_laser and base_link have opposite Y axes, so angles are
    // measured with changed sign
    min_distance_angle = -min_distance_angle;
    ROS_DEBUG("Computed direction: %.2f rad (%.2f deg)", min_distance_angle, min_distance_angle*180/3.1415);
    
    cmd_vel.angular.z = controller_p_ang * min_distance_angle;
    ROS_DEBUG("Computed angular velocity: %.2f rad/s", cmd_vel.angular.z);
    if (cmd_vel.angular.z > MAX_ANGULAR)
    {
        cmd_vel.angular.z = MAX_ANGULAR;
        ROS_DEBUG("Angular velocity limited to %.2f rad/s", MAX_ANGULAR);
    }

    ROS_DEBUG("Publishing linear velocity (m/s):    [%.2f, %.2f, %.2f]",
            cmd_vel.linear.x,
            cmd_vel.linear.y,
            cmd_vel.linear.z);
    ROS_DEBUG("Publishing angular velocity (rad/s): [%.2f, %.2f, %.2f]",
            cmd_vel.angular.x,
            cmd_vel.angular.y,
            cmd_vel.angular.z);
    
    cmd_vel_pub.publish(cmd_vel);
}

// publishMarker uses the computed minimum distance and angle to the pillar
// and publishes this position in the laser scan frame.
// The publisher uses a marker message that can be used in rviz to show the
// pillar's position
void HuskyHighlevelController::publishMarker(void)
{
    double marker_x {0}, marker_y {0}, marker_z {0};
    visualization_msgs::Marker marker;

    // Angles have opposite sign in laser and odom frames
    marker_x = min_distance * cos(-min_distance_angle);
    marker_y = min_distance * sin(-min_distance_angle);

    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.header.frame_id = "base_laser";
    //ROS_INFO("Marker: topic = %s", marker.header.frame_id.c_str());
    marker.pose.position.x = marker_x;
    marker.pose.position.y = marker_y;
    marker.pose.position.z = marker_z;
    ROS_INFO("Marker: [x, y, z] = [%.2f, %.2f, %.2f]", marker_x, marker_y, marker_z);
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 1;
    ROS_INFO("Marker: scale = [%.2f, %.2f, %.2f]", marker.scale.x, marker.scale.y, marker.scale.z);
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker_pub.publish(marker);
}

void HuskyHighlevelController::controlLoop(void)
{
    ROS_DEBUG("HuskyHighlevelController::updateCommandVelocity called.");
    updateCommandVelocity(MAX_LINEAR, MAX_ANGULAR);
    publishMarker();
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
