cmake_minimum_required(VERSION 2.8.3)
project(niryo_one_commander)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  diagnostic_msgs
  geometry_msgs
  moveit_msgs
  roscpp
  rospy
  sensor_msgs
  shape_msgs
  std_msgs
  std_srvs
  tf
  trajectory_msgs
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

