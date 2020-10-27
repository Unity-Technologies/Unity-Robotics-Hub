#!/usr/bin/env python

# tool_commander.py
# Copyright (C) 2017 Niryo
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

import rospy, actionlib

from niryo_one_msgs.msg import ToolAction
from niryo_one_msgs.msg import ToolGoal

from niryo_one_commander.command_status import CommandStatus
from niryo_one_commander.robot_commander_exception import RobotCommanderException
from actionlib_msgs.msg import GoalStatus


class ToolCommander:

    def __init__(self):
        self.action_client = actionlib.SimpleActionClient(
            'niryo_one/tool_action', ToolAction)
        rospy.loginfo("Waiting for action server : niryo_one/tool_action...")
        self.action_client.wait_for_server()
        rospy.loginfo("Found action server : niryo_one/tool_action")

        rospy.loginfo("Tool Commander has been started")

    def send_tool_command(self, cmd):
        goal = self.create_goal(cmd)
        self.action_client.send_goal(goal)
        rospy.loginfo("Tool command sent")

        # wait for goal transition to DONE
        self.action_client.wait_for_result()

        # if goal has been rejected/aborted, stop tracking it and return error
        if self.has_problem():
            message = self.action_client.get_result().message
            self.action_client.stop_tracking_goal()
            raise RobotCommanderException(CommandStatus.TOOL_FAILED, message)

    def stop_tool_command(self):
        pass
        # todo cancel goal

    @staticmethod
    def create_goal(cmd):
        goal = ToolGoal()
        goal.cmd = cmd
        return goal

    # Returns LOST if this SimpleActionClient isn't tracking a goal. 
    # see documentation : http://docs.ros.org/diamondback/api/actionlib/html/classactionlib_1_1simple__action__client_1_1SimpleActionClient.html#a6bdebdd9f43a470ecd361d2c8b743188
    def get_command_status(self):
        return self.action_client.get_state()

    def has_problem(self):
        status = self.get_command_status()
        # rospy.loginfo("STATUS : " + str(status))
        return status == GoalStatus.ABORTED or status == GoalStatus.REJECTED
