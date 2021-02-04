#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# Robotiq repository https://github.com/ros-industrial/robotiq.git
git init $DIR/ROS/src/robotiq
cd $DIR/ROS/src/robotiq
git remote add origin https://github.com/ros-industrial/robotiq.git
git config core.sparseCheckout true
echo "robotiq_2f_140_gripper_visualization/*" >> .git/info/sparse-checkout
echo "LICENSE" >> .git/info/sparse-checkout
echo ".gitignore" >> .git/info/sparse-checkout
echo "CONTRIBUTING.md" >> .git/info/sparse-checkout 
echo ".travis.yml" >> .git/info/sparse-checkout 
git pull --depth=1 origin kinetic-devel

# Universal Robot repository https://github.com/ros-industrial/universal_robot
git init $DIR/ROS/src/universal_robot
cd $DIR/ROS/src/universal_robot
git remote add origin https://github.com/ros-industrial/universal_robot
git config core.sparseCheckout true
echo "ur_description/*" >> .git/info/sparse-checkout
echo "ur_gazebo/*" >> .git/info/sparse-checkout
echo ".gitignore" >> .git/info/sparse-checkout
echo "CONTRIBUTING.md" >> .git/info/sparse-checkout 
echo ".travis.yml" >> .git/info/sparse-checkout 
git pull --depth=1 origin melodic-devel