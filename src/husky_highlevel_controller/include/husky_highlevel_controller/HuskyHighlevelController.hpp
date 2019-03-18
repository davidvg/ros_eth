#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

namespace husky_highlevel_controller {

// Class containing the HuskyHighlevelController
class HuskyHighlevelController 
{
    public:
        HuskyHighlevelController();
        virtual ~HuskyHighlevelController();
        
        bool init(void);

    private:
        // NodeHandles
        ros::NodeHandle nodeHandle_;

        // Variables
        double min_distance; 
        int min_distance_ix;

        // Subscribers
        ros::Subscriber laserscan_sub;
        
        // Publishers
       
        // Function Prototypes
        //void getParameter(std::string, std::string);
        //void getParameter(std::string, int);
        void LaserscanCallback(const sensor_msgs::LaserScan::ConstPtr&);

};


} // Namespace: husky_highlevel_controller
