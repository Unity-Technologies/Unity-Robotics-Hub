cmake_minimum_required(VERSION 2.8.3)
project(niryo_one_user_interface)

find_package(catkin REQUIRED COMPONENTS
  niryo_one_msgs
  roscpp
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
