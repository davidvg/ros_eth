#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#define MAX_LINEAR 2.0
#define MAX_ACCEL 1.0
#define MAX_ANGULAR 1.0

namespace husky_highlevel_controller {

// Class containing the HuskyHighlevelController
class HuskyHighlevelController 
{
    public:
        HuskyHighlevelController();
        virtual ~HuskyHighlevelController();
        
        // Function Prototypes
        void controlLoop();

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

        // Subscribers
        ros::Subscriber laserscan_sub;
        
        // Publishers
        ros::Publisher cmd_vel_pub;
        ros::Publisher marker_pub;

        // Function Prototypes
        bool init(void);
        std::string getParameterString(std::string);
        int getParameterInt(std::string);
        double getParameterDouble(std::string);
        void laserscanCallback(const sensor_msgs::LaserScan::ConstPtr&);
        void getLaserParameters();
        void updateCommandVelocity(double, double);
        void updateMarker();

};


} // Namespace: husky_highlevel_controller
