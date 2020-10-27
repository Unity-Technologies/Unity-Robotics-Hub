cmake_minimum_required(VERSION 2.8.3)
project(niryo_one_tcp_server)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  niryo_one_msgs
  niryo_one_python_api)

catkin_python_setup()

catkin_package()

include_directories(
 ${catkin_INCLUDE_DIRS}
)
