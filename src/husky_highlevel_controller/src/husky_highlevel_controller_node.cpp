#include <ros/ros.h>
#include "husky_highlevel_controller/HuskyHighlevelController.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "husky_highlevel_controller");
    ros::NodeHandle nodeHandle("~");

    ROS_INFO("Node launched: husky_highlevel_controller");

    // Get parameter: topic
    std::string topic;
    // Get parameter: queue_size
    int queue_size;
    
    //if (!nodeHandle.getParam("laser/topic", topic))
    //{
        //ROS_ERROR("Could not find parameter: %s", "topic");
    //}
    topic = "/scan";
    ROS_INFO("Parameter: topic = %s", topic.c_str());
    queue_size = 1;
    ROS_INFO("Parameter: queue_size = %d", queue_size); 

    // Create instance of the controller
    husky_highlevel_controller::HuskyHighlevelController controller(nodeHandle);

    // Subscriber to the topic
    ros::Subscriber subs = 
        nodeHandle.subscribe(topic, queue_size, controller.laserscan_callback);
    ROS_INFO("Subscriber created");

    ros::spin();
    return 0;
}
