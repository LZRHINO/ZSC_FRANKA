#panda_msgs/Header header       #任务的头文件添加时间戳 任务id等

# 机械臂的每一个任务分步的类型:
# 0:抓取;
# 1:放置;
# 2:移动;
# 3:末端笛卡尔方向移动;
# 4:姿态跟随;
# 5:特定关节的控制;
# 6.示教记忆;
# 7:任务开始;
# 8:任务结束;
int8 task_type

bool track_bgrasp               #夹爪命令
string str_id                   #示教记忆的储存名
geometry_msgs/Pose target_pose  #末端目标位置在世界坐标系的姿态
string  object_name             #操作物体的名字
string  joint_id                #控制的关节
float64[] joint_goal            #控制单个关节运动的关节角或者夹爪的宽度
float64 task_duration          #姿态跟随的运行的时长限制