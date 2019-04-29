#include <ros/ros.h>
#include <panda_control/base_controller_server.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "panda_control_node");
    ros::NodeHandle node("~");

    std::string arm, gripper;
    if(!(node.getParam("panda_arm_command_topic",arm))){
        ROS_ERROR("No panda_arm_command_topic parameters provided");
        return 1;
    }
    if(!(node.getParam("panda_gripper_command_topic",gripper))){
        ROS_ERROR("No panda_gripper_command_topic parameters provided");
        return 1;
    }
    
    //实例化panda_control对象
    panda_controller_server pcs(node, arm, gripper);

    ros::spin();

    return 0;
}