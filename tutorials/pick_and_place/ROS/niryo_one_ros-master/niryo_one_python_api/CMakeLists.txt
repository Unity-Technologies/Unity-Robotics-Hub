cmake_minimum_required(VERSION 2.8.3)
project(niryo_one_python_api)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)
catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
