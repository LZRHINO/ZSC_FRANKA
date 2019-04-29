/*
机械臂的控制Action服务端 也是最低层的通信单元
Author: LINZHUORONG 
Email: 593421541@qq.com
*/

#include "panda_control/base_controller_server.h"

using namespace std;

panda_controller_server::panda_controller_server(ros::NodeHandle &n, string arm_topic, string gripper_topic):
    node(n),
    client(gripper_topic, true),
    server(node, "/panda_controller",
        boost::bind(&panda_controller_server::goalCB, this, _1),
        boost::bind(&panda_controller_server::cancelCB, this, _1),
        false),
    has_active_goal(false)
{
    pub_joint_command = node.advertise<trajectory_msgs::JointTrajectory>(arm_topic, 1000);
    execute_timer  = node.createTimer(ros::Duration(0.0333333), &panda_controller_server::listenState, this);
    watchdog_timer = node.createTimer(ros::Duration(1.0), &panda_controller_server::watchdog, this);
    server.start();
};

panda_controller_server::~panda_controller_server(){
    pub_joint_command.shutdown();
    watchdog_timer.stop();
    execute_timer.stop();
    ros::shutdown();
};

void panda_controller_server::listenState(const ros::TimerEvent &e){
    if(!has_active_goal && !goal_queue.empty()){
        has_active_goal = true;
        std::thread exec_thread(std::bind(&panda_controller_server::serverExecute,this));
        exec_thread.detach();
    }
};

void panda_controller_server::judgeAbort(){
    //如果当前有活动的目标 判断执行情况是否正常 如果不正常就停止
    if(has_active_goal){
        double now = ros::Time::now().toSec();
        if(now > (start_time + duration_time + 5.0)){
            ROS_WARN("Aborted! goal time out!");
            Stop();
            has_active_goal = false;
            return;
        }
    }
};

void panda_controller_server::serverExecute(){
    if(!goal_queue.empty()){
        GoalHandle gh = goal_queue.top();
        goal_queue.pop();
        if(!cancel_queue.empty() && (gh == cancel_queue.top())){
            cancel_queue.pop();
        }else{
            active_goal = gh;
            start_time = ros::Time::now().toSec();
            duration_time = gh.getGoal()->exec_duration;
            duration_time += 0.03333333333; //30HZ 容错
            //输出当前运行的目标
            ROS_INFO("Current Control Goal_queue size: =  [%d]", (int)goal_queue.size());
            ROS_INFO("Current Active Control Goal -- task_id: [%s], task_weight: [%d] , goal_seq [%d] ",
                    gh.getGoal()->header.task_id.c_str(), (int)gh.getGoal()->header.task_weight, (int)gh.getGoal()->header.seq);
            bool gripper_state = false;
            vector<panda_msgs::ControlStep> current_csp = gh.getGoal()->csp;
            for(int i = 0; i < current_csp.size(); i++){
                if(!current_csp[i].barm && !current_csp[i].bgrasp){
                    ROS_ERROR("Step [%d] error --- goal param [barm] , [bgrasp] can't be false at the same step!",i);
                    active_goal.setCanceled();
                    Stop();
                    return;
                }else{
                    double wait_time;
                    if(current_csp[i].barm){
                        JointCommand(current_csp[i].joint_trajectory.joint_trajectory);
                        wait_time = current_csp[i].joint_trajectory.joint_trajectory.points.back().time_from_start.toSec();
                    }
                    if(current_csp[i].bgrasp && current_csp[i].on_off != gripper_state){
                        gripper_state = ~gripper_state;
                        if(!GripperCommand(gripper_state))
                            return;
                    }
                    ros::Duration(wait_time + 2.0).sleep();
                }
            }
        }
        
    }
    has_active_goal = false;
};

void panda_controller_server::watchdog(const ros::TimerEvent &e) {
    judgeAbort();
};

void panda_controller_server::Stop(){
    has_active_goal = true; //用于避免进入目标队列监听程序
    
    //发布空的轨迹话题抢占手臂
    trajectory_msgs::JointTrajectory empty;
    empty.joint_names = joint_names;
    pub_joint_command.publish(empty);
    GripperCommand(false);
    ros::Duration(1.0).sleep();

    //夹爪的action客户端关闭所有的目标
    client.cancelAllGoals();
    active_goal.setCanceled();
    while(!goal_queue.empty())
        goal_queue.pop();
    while(!cancel_queue.empty())
        cancel_queue.pop();
    has_active_goal = false;
};

void panda_controller_server::goalCB(GoalHandle gh){
    goal_queue.push(gh);
    ROS_INFO("Goal pushed into the queue!"); 
    gh.setAccepted();
};

void panda_controller_server::cancelCB(GoalHandle gh){
    if (active_goal == gh){
        Stop();
    }else{
        cancel_queue.push(gh); //保存在关闭队列 等待比对
    }
};

void panda_controller_server::ActiveCb(){
  ROS_INFO("Goal just went active");
};

void panda_controller_server::DoneCb(
    const actionlib::SimpleClientGoalState& state,
    const control_msgs::GripperCommandResultConstPtr& result) 
{
    ROS_INFO("Gripper Current Width  --- [%f]", result->position);
    ROS_INFO("Gripper Current Effort --- [%f]", result->effort);
    if((bool)result->stalled)
        ROS_INFO("Gripper is exerting max effort and not moving!");
    if((bool)result->reached_goal)
        ROS_INFO("Gripper position has reached the commanded setpoint");
    else
        ROS_INFO("Gripper Command failed!");
    node.setParam("franka_gripper/default_speed", 1.0);
    //ros::shutdown();     
};

void panda_controller_server::FeedbackCb(
    const control_msgs::GripperCommandFeedbackConstPtr& feedback) {
    ROS_INFO("Got Feedback of Gripper Width  --- [%f]", feedback->position);
    ROS_INFO("Got Feedback of Gripper Effort --- [%f]", feedback->effort);
};

bool panda_controller_server::GripperCommand(bool on_off) {
    double target_width, max_effort;
    if(!(node.getParam("panda_config/gripper_effort",max_effort))){
        ROS_ERROR("No gripper_effort parameters provided");
        return false;
    }
    if(on_off){
        target_width = 0.0001;
    }else{
        target_width = 0.0799;
    }
    client.waitForServer();
    control_msgs::GripperCommandGoal goal;
    goal.command.position = target_width / 2.0;
    goal.command.max_effort = max_effort;
    client.sendGoal(goal,
        boost::bind(&panda_controller_server::DoneCb, this, _1, _2),
        boost::bind(&panda_controller_server::ActiveCb, this),
        boost::bind(&panda_controller_server::FeedbackCb, this, _1));
};

void panda_controller_server::JointCommand(trajectory_msgs::JointTrajectory trajectory) {
    trajectory.header.stamp = ros::Time::now();
    pub_joint_command.publish(trajectory); 
};