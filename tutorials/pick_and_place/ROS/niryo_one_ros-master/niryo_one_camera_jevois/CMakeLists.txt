cmake_minimum_required(VERSION 2.8.3)
project(niryo_one_camera_jevois)

find_package(catkin REQUIRED COMPONENTS
  niryo_one_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
