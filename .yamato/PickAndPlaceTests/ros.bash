source /opt/ros/noetic/setup.bash

set -e

help() {
        echo "usage: $0 [start_pick_and_place|stop]"
}

if [ $1 == "stop" ]; then
	echo "Terminating process $2"
	pkill -15 -P $2
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
fi
# Assuming this script is invoked from the root of the repository...
