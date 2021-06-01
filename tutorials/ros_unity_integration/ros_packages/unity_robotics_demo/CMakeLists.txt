cmake_minimum_required(VERSION 2.8.3)
project(unity_robotics_demo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  ros_tcp_endpoint
  message_generation
)

catkin_package(CATKIN_DEPENDS
    ros_tcp_endpoint
    message_runtime)

catkin_install_python(PROGRAMS
  scripts/position_service.py
  scripts/color_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_robotics_demo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
