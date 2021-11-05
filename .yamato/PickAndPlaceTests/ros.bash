#!/bin/bash
# Assuming this script is invoked from the root of the repository...
help() {
  echo "usage: $0 [COMMAND] [ROS]"
  echo "COMMAND:"
  echo "  - stop"
  echo "  - build_pick_and_place"
  echo "  - start_pick_and_place"
  echo "  - build_ros"
  echo "  - start_ros"
  echo "  - run_ros_color_publisher"
  echo "  - run_ros_pose_service_client"
  echo "  - run_ros_position_service"
  echo "ROS"
  echo "  - ros1"
  echo "  - ros2"
}

COMMAND=$1
ROS=$2

if [ "$COMMAND" == "stop" ]; then
	echo "Terminating process $3"
	pkill -15 -P $3
  sleep 10

elif [ "$COMMAND" == "build_pick_and_place" ]; then
  source /opt/ros/noetic/setup.bash
  pushd $PWD
  cd tutorials/pick_and_place/ROS
  catkin_make
  source devel/setup.bash
  popd

elif [ "$COMMAND" == "start_pick_and_place" ]; then
	echo "Starting ROS for Pick and Place"
  source tutorials/pick_and_place/ROS/devel/setup.bash
  roslaunch niryo_moveit part_3.launch

elif [ "$COMMAND" == "build_ros" ]; then
  if [ "$ROS" == "ros1" ]; then
    export ROS_WORKSPACE=$(pwd)/ros1_ws
    mkdir -p $ROS_WORKSPACE/src
    cp -r tutorials/ros_unity_integration/ros_packages/ $ROS_WORKSPACE/src/
    git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint $ROS_WORKSPACE/src/ros_tcp_endpoint -b main
    /bin/bash tutorials/ros_unity_integration/ros_docker/set-up-workspace
    chmod +x $ROS_WORKSPACE/src/ros_tcp_endpoint/src/ros_tcp_endpoint/*.py
  elif [ "$ROS" == "ros2" ]; then
    export ROS_WORKSPACE=$(pwd)/ros2_ws
    mkdir -p $ROS_WORKSPACE/src
    cp -r tutorials/ros_unity_integration/ros2_packages/ $ROS_WORKSPACE/src/
    git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint $ROS_WORKSPACE/src/ros_tcp_endpoint -b main-ros2
    source /opt/ros/$ROS_DISTRO/setup.sh
    pushd $(pwd)
    cd $ROS_WORKSPACE
    colcon build
    popd
  else
    help
  fi

elif [ "$COMMAND" == "start_ros" ]; then
  if [ "$ROS" == "ros1" ]; then
    source ros1_ws/devel/setup.bash
    echo "Starting ROS1 master"
    roscore &
    sleep 5  # Wait ROS master to stand up
    rosparam set ROS_IP 127.0.0.1
    echo "Starting ROS1 default server endpoint"
    rosrun ros_tcp_endpoint default_server_endpoint.py
  elif [ "$ROS" == "ros2" ]; then
    source ros2_ws/install/setup.bash
    echo "Starting ROS2 default server endpoint"
    ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1
  else
    help
  fi

elif [ "$COMMAND" == "run_ros_color_publisher" ]; then
  if [ "$ROS" == "ros1" ]; then
    source ros1_ws/devel/setup.bash
  elif [ "$ROS" == "ros2" ]; then
    source ros2_ws/install/setup.bash
  else
    help
  fi
  echo "Starting to run $ROS color publisher every 30 seconds"
  count=0
  while [[ $count -le 6 ]]
  do
    sleep 5
    if [ "$ROS" == "ros1" ]; then
      rosrun unity_robotics_demo color_publisher.py
    elif [ "$ROS" == "ros2" ]; then
      ros2 run unity_robotics_demo color_publisher
    else
      help
    fi
    count=$(( $count + 1 ))
  done
  echo "Completed run: $ROS color publisher"

elif [ "$COMMAND" == "run_ros_pose_service_client" ]; then
  if [ "$ROS" == "ros1" ]; then
    source ros1_ws/devel/setup.bash
  elif [ "$ROS" == "ros2" ]; then
    source ros2_ws/install/setup.bash
  else
    help
  fi
  echo "Starting to run $ROS pose service client and send requests every 30 seconds"
  count=0
  while [[ $count -le 6 ]]
  do
    sleep 5
    if [ "$ROS" == "ros1" ]; then
      rosservice call /obj_pose_srv Cube
    elif [ "$ROS" == "ros2" ]; then
      ros2 service call obj_pose_srv unity_robotics_demo_msgs/ObjectPoseService "{object_name: Cube}"
    else
      help
    fi
    count=$(( $count + 1 ))
  done
  echo "Completed run: $ROS pose service client"

elif [ "$COMMAND" == "run_ros_position_service" ]; then
  if [ "$ROS" == "ros1" ]; then
    source ros1_ws/devel/setup.bash
  elif [ "$ROS" == "ros2" ]; then
    source ros2_ws/install/setup.bash
  else
    help
  fi
  echo "Starting $ROS position service"
  if [ "$ROS" == "ros1" ]; then
    rosrun unity_robotics_demo position_service.py
  elif [ "$ROS" == "ros2" ]; then
    ros2 run unity_robotics_demo position_service
  else
    help
  fi
  
else
  help
fi
