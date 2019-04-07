#include <ros/ros.h>
#include <std_srvs/SetBool.h>


int main (int argc, char** argv)
{
    ros::init(argc, argv, "start_robot_node");
    ros::NodeHandle nh;

    ros::ServiceClient start_robot_client=
        nh.serviceClient<std_srvs::SetBool>("/husky_highlevel_controller/start_robot");

    std_srvs::SetBool srv_msg;
    int val {0};
    //srv_msg.request.data = (unsigned char) *argv[1];
    
    val = atoi(argv[1]);
    ROS_INFO("start_robot - Read value: %d", val);
    

    ROS_INFO_STREAM("Request.data: " << srv_msg.request.data);

    if (start_robot_client.call(srv_msg))
    {
        ROS_INFO("OK");
    }
    else
    {
        ROS_ERROR("Failed to call start_robot client.");
    }

    ros::spin();
    return 0;
}
