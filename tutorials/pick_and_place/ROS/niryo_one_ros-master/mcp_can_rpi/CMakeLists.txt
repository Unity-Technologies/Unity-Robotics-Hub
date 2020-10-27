cmake_minimum_required(VERSION 2.8.3)
project(mcp_can_rpi)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mcp_can_rpi
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(mcp_can_rpi
    src/mcp_can_rpi.cpp
)

add_dependencies(mcp_can_rpi ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}" )

if (${ARCHITECTURE} MATCHES "arm")
    message(STATUS "wiringPi library is required for mcp_can_rpi (ARM processor)")
    target_link_libraries(mcp_can_rpi
        ${catkin_LIBRARIES} 
        -lwiringPi
    )
else()
    message(STATUS "wiringPi library not required")
    target_link_libraries(mcp_can_rpi 
        ${catkin_LIBRARIES} 
    )
endif()

#############
## Install ##
#############
install(TARGETS mcp_can_rpi
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
