#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash
. install/local_setup.bash
ros2 launch ros2_test test_launcher.py
