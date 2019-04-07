#include "husky_highlevel_controller/HuskyHighlevelController.hpp"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
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

    // start_robot_server
    start_robot_server = nodeHandle_.advertiseService(
            "/husky_highlevel_controller/start_robot",
            &HuskyHighlevelController::startrobotCallback,
            this);
    
    // start_robot client
    start_robot_client = nodeHandle_.serviceClient<std_srvs::SetBool>("start_robot");

    // Exit HuskyHighlevelController::init()
    return true;
}

void HuskyHighlevelController::laserscanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    ROS_DEBUG("HuskyHighlevelController::laserscanCallback called.");
    // Store the message in the class
    laserResponse = *msg;
}

bool HuskyHighlevelController::startrobotCallback(std_srvs::SetBool::Request &req,
                                             std_srvs::SetBool::Response &res)
{
    // Update the swith value
    robot_run = req.data;

    res.success = (robot_run == req.data);
    res.message = "start_robot set.";

    return true;
}

/*
To-Do:
Allow to pass (0, 0) to updateCommandVelocity to ensure correct stop when
stopped using service.

The function takes two arguments (linear, angular) that currently do NOTHING.
*/
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
    ROS_DEBUG("Computed direction: %.2f rad (%.2f deg)", min_distance_angle, min_distance_angle*180/3.141592654);
    
    cmd_vel.angular.z = controller_p_ang * min_distance_angle;
    ROS_DEBUG("Computed angular velocity: %.2f rad/s", cmd_vel.angular.z);
    if (cmd_vel.angular.z > MAX_ANGULAR)
    {
        cmd_vel.angular.z = MAX_ANGULAR;
        ROS_DEBUG("Angular velocity limited to %.2f rad/s", MAX_ANGULAR);
    }

    ROS_DEBUG("Publishing linear velocity (m/s): [%.2f, %.2f, %.2f]",
            cmd_vel.linear.x,
            cmd_vel.linear.y,
            cmd_vel.linear.z);
    ROS_DEBUG("Publishing angular velocity (rad/s): [%.2f, %.2f, %.2f]",
            cmd_vel.angular.x,
            cmd_vel.angular.y,
            cmd_vel.angular.z);
    
    cmd_vel_pub.publish(cmd_vel);
}

void HuskyHighlevelController::updateMarker(void)
{
    /*
    Uses the computed minimum distance and angle to the pillar
    and publishes this position in the laser scan frame.
    The publisher uses a marker message that can be used in rviz to show the
    pillar's position
    */
    // Distance to target in the /base_laser frame
    double measure_x_laser {0}, measure_y_laser {0}, measure_z_laser {0};
    // Distance to target in the /base_link frame
    double measure_x_link {0}, measure_y_link {0}, measure_z_link {0};
    // Distance to /base_laser in /odom (global) frame
    double laser_pose_x {0}, laser_pose_y {0}, laser_pose_z {0};

    visualization_msgs::Marker marker;

    // Y coordinates are inverted in the /base_laser and /base_link frames
    measure_x_laser = min_distance * cos(-min_distance_angle);
    measure_y_laser = min_distance * sin(-min_distance_angle);

    measure_x_link = measure_x_laser;
    measure_y_link = -measure_y_laser;

    ROS_DEBUG("Measured distance from laser: [%.2f, %.2f, %.2f]",
        measure_x_laser,
        measure_y_laser,
        measure_z_laser);

    // Marker parameters
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.header.frame_id = "base_laser";
    marker.header.stamp = ros::Time::now();
    marker.pose.position.x = measure_x_laser;
    marker.pose.position.y = measure_y_laser;
    marker.pose.position.z = measure_z_laser;
    marker.scale.x = 0.1;
    marker.scale.y = marker.scale.x;
    marker.scale.z = 1;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker_pub.publish(marker);
}

void HuskyHighlevelController::updateMarkerTF(void)
{
    /*
    Uses the computed minimum distance and angle to the pillar and transforms
    it to the /odom frame using transforms. This computed point is used to 
    define a visualization_msgs::Marker object, that is published in the
    /odom frame.
    */

    // Distance to target in the /base_laser frame
    double measure_x_laser {0}, measure_y_laser {0}, measure_z_laser {0};

    // Y coordinates are inverted in the /base_laser and /base_link frames
    measure_x_laser = min_distance * cos(-min_distance_angle);
    measure_y_laser = min_distance * sin(-min_distance_angle);

    tf::StampedTransform transform;

    // Define points in base_laser and odom frames
    geometry_msgs::PointStamped laser_point;
    // The converted point, odom_point, is defined in the header file to
    // make it a class member, thus keeping previous values in case the 
    // transform is not found. This avoids the marker showing in the position
    // [0, 0, 0] in the /odom frame, as a result of a default definition of the
    // marker.
     
    //geometry_msgs::PointStamped odom_point;

    try
    {
        tfListener.waitForTransform("/odom", "/base_laser", ros::Time(0), ros::Duration(0.1));
        tfListener.lookupTransform("/odom", "/base_laser", ros::Time(0), transform);
        //ROS_INFO("canTransform: TRUE");
        ROS_DEBUG("Transform base_laser -> odom: [%.2f, %.2f, %.2f]",
                 transform.getOrigin().x(),
                 transform.getOrigin().y(),
                 transform.getOrigin().z());

        // Populate point in base_laser frame
        laser_point.header.frame_id = "base_laser";
        laser_point.header.stamp = ros::Time();
        laser_point.point.x = measure_x_laser;
        laser_point.point.y = measure_y_laser;
        laser_point.point.z = 0.0;

        // Get transformed point
        tfListener.transformPoint("odom", laser_point, odom_point);

        ROS_DEBUG("base_laser: [%.2f, %.2f, %.2f]",
                 laser_point.point.x,
                 laser_point.point.y,
                 laser_point.point.z);

        ROS_DEBUG("odom      : [%.2f, %.2f, %.2f]",
                odom_point.point.x,
                odom_point.point.y,
                odom_point.point.z);

    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        ros::Duration(1);
    }

    // Define marker object
    visualization_msgs::Marker marker;
    
    // Populate marker 
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time::now();
    marker.pose.position.x = odom_point.point.x;
    marker.pose.position.y = odom_point.point.y;
    marker.pose.position.z = odom_point.point.z;
    marker.scale.x = 0.1;
    marker.scale.y = marker.scale.x;
    marker.scale.z = 1;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker_pub.publish(marker);
}

void HuskyHighlevelController::controlLoop(void)
{
    // Run the loop only if the robot has been started via start_robot service
    if (robot_run)
    {
        ROS_DEBUG("HuskyHighlevelController::controlLoop called.");
        updateCommandVelocity(MAX_LINEAR, MAX_ANGULAR);
        //updateMarker();
        updateMarkerTF();
    }
    else
    {
        // Uncomment when updateCommandVelocity is corrected.
        //updateCommandVelocity(0.0, 0.0);
    }
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
