cmake_minimum_required(VERSION 2.8.3)
project(niryo_one_debug)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  niryo_one_msgs
  mcp_can_rpi
  dynamixel_sdk
)
 
find_package(Boost REQUIRED COMPONENTS system program_options)

catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    niryo_one_msgs
    mcp_can_rpi
    dynamixel_sdk
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(dxl_debug_tools
    src/dxl_debug.cpp
    src/dxl_tools.cpp
)

#
# wiringPi should be installed only on a Raspberry Pi board
#

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}" )

if (${ARCHITECTURE} MATCHES "arm")
    message(STATUS "wiringPi library is required - arm processor")
    target_link_libraries(dxl_debug_tools
        ${catkin_LIBRARIES} 
        ${Boost_LIBRARIES}
        -lwiringPi
    )
else()
    message(STATUS "wiringPi library not required")
    target_link_libraries(dxl_debug_tools
        ${catkin_LIBRARIES} 
        ${Boost_LIBRARIES}
    )
endif()


