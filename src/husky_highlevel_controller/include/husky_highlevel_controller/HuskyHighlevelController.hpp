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
        
        double min_measure; 
        //bool init();

    private:
        // NodeHandles
        ros::NodeHandle nodeHandle_;

        // Variables

        // Subscribers
        ros::Subscriber laserscan_sub;
        
        // Publishers
       
        // Function Prototypes
        //void getParameter(std::string, std::string);
        void LaserscanCallback(const sensor_msgs::LaserScan::ConstPtr&);


    /*
    public:
    HuskyHighlevelController(ros::NodeHandle& nodeHandle);
    virtual ~HuskyHighlevelController();

    static void laserscan_callback(const sensor_msgs::LaserScan::ConstPtr&);

    private:
    ros::NodeHandle nodeHandle_;
    */
};


} // Namespace: husky_highlevel_controller
