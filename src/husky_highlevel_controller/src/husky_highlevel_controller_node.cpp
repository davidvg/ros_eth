#include <ros/ros.h>
#include "husky_highlevel_controller/HuskyHighlevelController.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "husky_highlevel_controller");

    husky_highlevel_controller::HuskyHighlevelController controller;
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        controller.controlLoop();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
