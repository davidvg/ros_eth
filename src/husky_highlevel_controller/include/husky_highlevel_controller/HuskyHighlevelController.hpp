#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <std_srvs/SetBool.h>

#define MAX_LINEAR 2.0
#define MAX_ANGULAR 1.0
#define MAX_ACCEL 1.0

/*
To-Do
-----
- Implement acceleration ramp
*/

namespace husky_highlevel_controller {

// Class containing the HuskyHighlevelController
class HuskyHighlevelController 
{
    public:
        HuskyHighlevelController();
        virtual ~HuskyHighlevelController();
        
        // Function Prototypes
        void controlLoop();

        ros::NodeHandle nh;

    private:
        // NodeHandles
        ros::NodeHandle nodeHandle_;

        // Variables
        sensor_msgs::LaserScan laserResponse;  // Response from laserscanCallback
        double min_distance; 
        int min_distance_ix;
        double min_distance_angle;
        // Controller gains
        double controller_p_vel{};
        double controller_p_ang{};
        // Transformed laserscan point
        geometry_msgs::PointStamped odom_point;
        // Switch variable for starting/stopping the robot
        bool robot_run = true;

        // Subscribers
        ros::Subscriber laserscan_sub;
        
        // Publishers
        ros::Publisher cmd_vel_pub;
        ros::Publisher marker_pub;

        // Transform listeners
        tf::TransformListener tfListener;

        // Service servers
        ros::ServiceServer start_robot_server;

        // Service Clients
        ros::ServiceClient start_robot_client;

        // Callbacks
        void laserscanCallback(const sensor_msgs::LaserScan::ConstPtr&);
        bool startrobotCallback(std_srvs::SetBool::Request&,
                                std_srvs::SetBool::Response&);

        // Function Prototypes
        bool init(void);
        std::string getParameterString(std::string);
        int getParameterInt(std::string);
        double getParameterDouble(std::string);
        void getLaserParameters();
        void computeMinDistance(void);
        void updateCommandVelocity(double linear=MAX_LINEAR,
                                   double angular=MAX_ANGULAR);
        void updateMarker();
        void updateMarkerTF();

};

} // Namespace: husky_highlevel_controller
