#!/usr/bin/env python

# niryo_one_commander_node.py
# Copyright (C) 2018 Niryo
# All rights reserved.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.


import rospy
from position_manager import PositionManager
from trajectory_manager import TrajectoryManager
from niryo_one_robot_state_publisher import NiryoRobotStatePublisher
from robot_commander import RobotCommander


class NiryoOneCommanderNode:

    def __init__(self):
        # Publish robot state (position, orientation, tool)
        self.niryo_one_robot_state_publisher = NiryoRobotStatePublisher()

        # Position Manager
        positions_dir = rospy.get_param("~positions_dir")
        self.pos_manager = PositionManager(positions_dir)
        # trajectory_manager
        trajectories_dir = rospy.get_param("~trajectories_dir")
        self.traj_manager = TrajectoryManager(trajectories_dir)
        # robot commander
        self.robot_commander = RobotCommander(self.pos_manager, self.traj_manager)
        self.robot_commander.start()


if __name__ == '__main__':
    rospy.init_node('niryo_one_commander')
    niryo_one_commander_node = NiryoOneCommanderNode()
    rospy.spin()
