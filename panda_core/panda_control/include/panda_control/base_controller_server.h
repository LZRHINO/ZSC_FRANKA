/*
机械臂的控制Action服务端 也是最低层的通信单元
Author: LINZHUORONG 
Email: 593421541@qq.com
*/

#ifndef PANDA_UTIL_H
#define PANDA_UTIL_H

#include <iostream>
#include <queue>
#include <vector>
#include <functional>
#include <algorithm>
#include <thread>
#include <boost/thread/thread.hpp>

#include <ros/service.h>
#include <ros/time.h>
#include <ros/duration.h>

#include <franka_gripper/franka_gripper.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/action_server.h>

//msg
#include <moveit_msgs/RobotTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/GripperCommandAction.h>
#include <panda_msgs/BaseControlGoal.h>
#include <panda_msgs/ControlStep.h>
#include <panda_msgs/BaseControlAction.h>
using namespace std;

typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> GripperCommandClient;
typedef actionlib::ActionServer<panda_msgs::BaseControlAction>   BaseControlServer;
typedef BaseControlServer::GoalHandle GoalHandle;
struct goal_cmp {
    bool operator () (const GoalHandle &a, const GoalHandle &b){
        //权重相同时，序列号最小值优先
        if(a.getGoal()->header.task_weight == b.getGoal()->header.task_weight){
            return a.getGoal()->header.seq > b.getGoal()->header.seq;
        }
        return a.getGoal()->header.task_weight < b.getGoal()->header.task_weight;
    } 
};

//panda机械臂的控制服务端
class panda_controller_server {
public:
    panda_controller_server(ros::NodeHandle &n, string arm_topic, string gripper_topic);
    ~panda_controller_server();
    void Stop();//用于抢占的停止程序
private:
    void listenState(const ros::TimerEvent &e); //监听目标队列程序
    void serverExecute(); //多线程运行函数
    void judgeAbort();//判断是否抢占目标
    void watchdog(const ros::TimerEvent &e); //看门狗程序  定时运行和监听错误
    void goalCB(GoalHandle gh);
    void cancelCB(GoalHandle gh);
//夹爪的Action客户端
    void ActiveCb();
    void DoneCb(
        const actionlib::SimpleClientGoalState& state,
        const control_msgs::GripperCommandResultConstPtr& result);
    void FeedbackCb(
        const control_msgs::GripperCommandFeedbackConstPtr& feedback);
//服务端请求处理函数
    // bool GripperCommand(double target_width, double max_effort, double speed);
    bool GripperCommand(bool on_off);
    void JointCommand(trajectory_msgs::JointTrajectory trajectory);
    ros::NodeHandle node;
    ros::Publisher pub_joint_command;
    ros::Timer watchdog_timer;
    ros::Timer execute_timer;
    
    std::priority_queue<GoalHandle,std::vector<GoalHandle>, goal_cmp>goal_queue;//目标的队列，进行保存
    std::priority_queue<GoalHandle,std::vector<GoalHandle>, goal_cmp>cancel_queue;//空间换时间，O(1)删除

    GoalHandle active_goal;
    GripperCommandClient client;
    BaseControlServer   server;
    std::vector<std::string> joint_names;
    bool has_active_goal;
    double start_time; //开始时间
    double duration_time;  //等待时间
};

#endif