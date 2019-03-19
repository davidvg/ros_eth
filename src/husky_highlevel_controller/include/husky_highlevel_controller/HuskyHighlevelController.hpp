#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#define MAX_VELOCITY 0.3
#define MAX_ANGULAR 0.5

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
        double min_distance; 
        int min_distance_ix;

        // Subscribers
        ros::Subscriber laserscan_sub;
        
        // Publishers
        ros::Publisher cmd_vel_pub;
       
        // Function Prototypes
        //void getParameter(std::string, std::string);
        //void getParameter(std::string, int);
        void laserscanCallback(const sensor_msgs::LaserScan::ConstPtr&);
        void updateCommandVelocity(double, double);

};


} // Namespace: husky_highlevel_controller
