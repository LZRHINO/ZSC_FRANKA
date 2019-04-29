bool barm                                           #是否进行控制关节
moveit_msgs/RobotTrajectory joint_trajectory        #arm的运动轨迹
bool bgrasp                                         #是否进行控制夹爪
bool on_off                                         #夹爪的命令  true 开到关， false 关到开  任务结束回到false的状态