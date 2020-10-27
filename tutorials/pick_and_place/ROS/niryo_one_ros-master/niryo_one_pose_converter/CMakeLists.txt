cmake_minimum_required(VERSION 2.8.3)
project(niryo_one_pose_converter)

find_package(catkin REQUIRED COMPONENTS
  niryo_one_commander
  geometry_msgs
  rospy
  std_msgs
  std_srvs
  tf
  niryo_one_msgs
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
