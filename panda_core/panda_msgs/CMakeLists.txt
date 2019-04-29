cmake_minimum_required(VERSION 2.8.3)
project(panda_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib_msgs
  trajectory_msgs
  moveit_msgs
  control_msgs
  geometry_msgs
  message_generation
)

add_message_files(DIRECTORY msg
FILES
  Header.msg
  ControlStep.msg
  TaskStep.msg
  MotionStep.msg
)

## Generate services in the 'srv' folder
add_service_files(DIRECTORY srv
)

## Generate actions in the 'action' folder
add_action_files(DIRECTORY action 
FILES
  BaseControl.action
  MotionPlan.action
  PandaTask.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
  actionlib_msgs
  moveit_msgs
  trajectory_msgs
  control_msgs
  geometry_msgs
)


catkin_package(
  CATKIN_DEPENDS moveit_msgs actionlib_msgs trajectory_msgs geometry_msgs message_runtime control_msgs std_msgs
)