#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <panda_msgs/BaseControlGoal.h>
#include <panda_msgs/BaseControlAction.h>
#include <panda_msgs/ControlStep.h>
#include <ros/duration.h>

using namespace std;

typedef actionlib::SimpleActionClient<panda_msgs::BaseControlAction> Client;

int main(int argc, char **argv)
{
	// 初始化，节点命名为"greetings_client"
	ros::init(argc, argv, "control_client");
	
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();
	Client client("/panda_controller", true);
    panda_msgs::BaseControlGoal goal;
    vector<panda_msgs::ControlStep>  step_list;
    const double degree = 3.1415926/180;
   // message declarations
    std::vector<trajectory_msgs::JointTrajectoryPoint> points_n(1);    
    step_list.resize(1);
    nh.getParam("panda_arm_controller/joints",step_list[0].joint_trajectory.joint_trajectory.joint_names);
    points_n[0].positions.resize(7);
    step_list[0].joint_trajectory.joint_trajectory.points.resize(2);
    //0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4
    points_n[0].positions[0] = 0.0;
    points_n[0].positions[1] = -0.785;
    points_n[0].positions[2] = 0.0;
    points_n[0].positions[3] = -3*0.785;
    points_n[0].positions[4] = 0;
    points_n[0].positions[5] = 1.57;
    points_n[0].positions[6] = 0.785;
    double t =0.0;
    int count = 0;
    ros::Rate loop_rate(30);
    while(ros::ok()){
        if(count <= 1000) {
            points_n[0].time_from_start.sec = 3;
            points_n[0].time_from_start.nsec = 0;
            step_list[0].barm = true;
            step_list[0].joint_trajectory.joint_trajectory.points[0] = points_n[0];
            points_n[0].positions[0] = points_n[0].positions[0] - 0.5*sin(0.01*t);
            points_n[0].positions[1] = points_n[0].positions[1] - 0.5*sin(0.01*t);
            points_n[0].time_from_start.sec = 6;
            points_n[0].time_from_start.nsec = 0;
            step_list[0].joint_trajectory.joint_trajectory.points[1] = points_n[0];
            step_list[0].bgrasp = false;

            ROS_INFO("==============================================");

            goal.header.stamp = ros::Time::now();
            goal.header.task_id = "123";
            goal.header.seq = count++;
            goal.csp.resize(1);
            goal.csp = step_list;
            goal.exec_duration = ros::Duration(6.5).toSec();
            if(count <= 500)
                goal.header.task_weight = 3;
            else 
                goal.header.task_weight = 4;
            client.sendGoal(goal);
            t = t+0.03;
            loop_rate.sleep();
            ROS_INFO("send %d", count);
        }
    }
	return 0;
}