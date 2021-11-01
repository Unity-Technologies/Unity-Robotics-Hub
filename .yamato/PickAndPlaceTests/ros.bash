source /opt/ros/noetic/setup.bash

set -e

help() {
  echo "usage: $0 [start_pick_and_place|stop]"
}

if [ $1 == "stop" ]; then
	echo "Terminating process $2"
	pkill -15 -P $2
  sleep 10
elif [ $1 == "build_pick_and_place" ]; then
  pushd $PWD
  cd tutorials/pick_and_place/ROS
  catkin_make
  source devel/setup.bash
  popd
elif [ $1 == "start_pick_and_place" ]; then
	echo "Starting ROS for Pick and Place"
  source tutorials/pick_and_place/ROS/devel/setup.bash
  roslaunch niryo_moveit part_3.launch
elif [ $1 == "build_ros1" ]; then
  export ROS_WORKSPACE=$(pwd)/ros1_ws
  mkdir -p $ROS_WORKSPACE/src
  cp -r tutorials/ros_unity_integration/ros_packages/ $ROS_WORKSPACE/src/
  cp -r tutorials/pick_and_place/ROS/src/ros_tcp_endpoint $ROS_WORKSPACE/src/
  /bin/bash tutorials/ros_unity_integration/ros_docker/set-up-workspace
  chmod +x $ROS_WORKSPACE/src/ros_tcp_endpoint/src/ros_tcp_endpoint/*.py
elif [ $1 == "start_ros1" ]; then
  source ros1_ws/devel/setup.bash
  echo "Starting ROS1 master"
  roscore &
  sleep 5  # Wait ROS master to stand up
  rosparam set ROS_IP 127.0.0.1
  echo "Starting ROS1 default server endpoint"
  rosrun ros_tcp_endpoint default_server_endpoint.py
elif [ $1 == "run_ros1_color_publisher" ]; then
  source ros1_ws/devel/setup.bash
  echo "Starting to run ROS1 color publisher every 30 seconds"
  count=0
  while [[ $counter -le 10 ]]
  do
    sleep 30
    rosrun unity_robotics_demo color_publisher.py
    count=$(( $count + 1 ))
  done
  echo "Completed to run ROS1 color publisher"
elif [ $1 == "run_ros1_pose_service_client" ]; then
  source ros1_ws/devel/setup.bash
  echo "Starting to run ROS1 pose service client and send requests every 30 seconds"
  count=0
  while [[ $counter -le 10 ]]
  do
    sleep 30
    rosservice call /obj_pose_srv Cube
    count=$(( $count + 1 ))
  done
  echo "Completed to run ROS1 pose service client"
elif [ $1 == "run_ros1_position_service" ]; then
  source ros1_ws/devel/setup.bash
  echo "Starting ROS1 position service"
  rosrun unity_robotics_demo position_service.py
else
  help
fi
# Assuming this script is invoked from the root of the repository...
