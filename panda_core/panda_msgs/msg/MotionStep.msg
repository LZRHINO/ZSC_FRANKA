#panda_msgs/Header header       #任务的头文件添加时间戳 任务id等

#0:机械臂静止
#1:关节角按期望路径移动;
#2:末端执行器转换到期望的姿态;
#3:末端执行器移动到目标位置;
#4:姿态跟随;
#5:迪卡尔空间的移动;
#6:特定关节的控制;
int8 move_type

bool barm                       #是否控制机械臂
bool bgrasp                     #是否控制夹爪
bool on_off                     #夹爪的命令  true 为关到开， false 开到关
geometry_msgs/Pose F_pose       #末端目标姿态
string joint_id                 #控制的关节
float64[] joint_goal            #控制的目标关节的关节角
trajectory_msgs/JointTrajectoryPoint[] waypoints  #期望经过的路径中间点
float64 follow_duration         #姿态跟随的运行的时长限制