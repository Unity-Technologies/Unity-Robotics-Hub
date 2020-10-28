cmake_minimum_required(VERSION 2.8.3)
project(niryo_one_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  controller_manager
  actionlib
  control_msgs
  geometry_msgs
  roscpp
  rospy
  std_msgs
  trajectory_msgs
  niryo_one_msgs
  mcp_can_rpi
  dynamixel_sdk
)
 
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    hardware_interface 
    controller_manager 
    actionlib control_msgs 
    geometry_msgs 
    sensor_msgs 
    trajectory_msgs
    niryo_one_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(niryo_one_driver
    src/utils/change_hardware_version.cpp
    src/utils/motor_offset_file_handler.cpp
    src/hw_driver/niryo_one_can_driver.cpp
    src/hw_driver/dxl_driver.cpp
    src/hw_driver/xl320_driver.cpp
    src/hw_driver/xl430_driver.cpp
    src/hw_comm/dxl_communication.cpp
    src/hw_comm/can_communication.cpp
    src/hw_comm/niryo_one_communication.cpp
    src/hw_comm/fake_communication.cpp
    src/ros_interface.cpp
    src/rpi_diagnostics.cpp
    src/niryo_one_hardware_interface.cpp
    src/niryo_one_driver_node.cpp
)

#
# wiringPi should be installed only on a Raspberry Pi board
#

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}" )

if (${ARCHITECTURE} MATCHES "arm")
    message(STATUS "wiringPi library is required - arm processor")
    target_link_libraries(niryo_one_driver
        ${catkin_LIBRARIES} 
        -lwiringPi
    )
else()
    message(STATUS "wiringPi library not required")
    target_link_libraries(niryo_one_driver
        ${catkin_LIBRARIES} 
    )
endif()

add_dependencies(niryo_one_driver niryo_one_msgs_gencpp)

