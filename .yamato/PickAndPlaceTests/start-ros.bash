source /opt/ros/noetic/setup.bash

set -e

# Assuming this script is invoked from the root of the repository...
DIR_ORIGINAL=$PWD
cd tutorials/pick_and_place/ROS
catkin_make
source devel/setup.bash
cd "$DIR_ORIGINAL"
roslaunch niryo_moveit part_3.launch &
