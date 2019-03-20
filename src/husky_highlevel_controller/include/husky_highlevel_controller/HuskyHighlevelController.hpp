#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#define MAX_LINEAR 5.0
#define MAX_ANGULAR 1.0

namespace husky_highlevel_controller {

// Class containing the HuskyHighlevelController
class HuskyHighlevelController 
{
    public:
        HuskyHighlevelController();
        virtual ~HuskyHighlevelController();
        
        // Function Prototypes
        bool init(void);
        void controlLoop();

    private:
        // NodeHandles
        ros::NodeHandle nodeHandle_;

        // Variables
        sensor_msgs::LaserScan laserResponse;  // Response from laserscanCallback
        double min_distance; 
        int min_distance_ix;
        // Parameters from laser
        struct laserParams {
            bool ready{false};
            double num_measurements{};
            double range_min{};
            double range_max{};
            double angle_min{};
            double angle_max{};
            double angle_increment{};
        } laser_params;
        double controller_p_vel{};
        double controller_p_ang{};

        // Subscribers
        ros::Subscriber laserscan_sub;
        
        // Publishers
        ros::Publisher cmd_vel_pub;
       
        // Function Prototypes
        std::string getParameterString(std::string);
        int getParameterInt(std::string);
        double getParameterDouble(std::string);
        void laserscanCallback(const sensor_msgs::LaserScan::ConstPtr&);
        void getLaserParameters();
        void updateCommandVelocity(double, double);

};


} // Namespace: husky_highlevel_controller
