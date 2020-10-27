#!/usr/bin/env python

# tool_ros_command_interface.py
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

import rospy

from niryo_one_msgs.srv import PingDxlTool
from niryo_one_msgs.srv import OpenGripper
from niryo_one_msgs.srv import CloseGripper
from niryo_one_msgs.srv import PullAirVacuumPump
from niryo_one_msgs.srv import PushAirVacuumPump
from niryo_one_msgs.srv import SetDigitalIO


class ToolRosCommandInterface:

    def __init__(self):
        rospy.wait_for_service('niryo_one/tools/ping_and_set_dxl_tool')
        rospy.wait_for_service('niryo_one/tools/open_gripper')
        rospy.wait_for_service('niryo_one/tools/close_gripper')
        rospy.wait_for_service('niryo_one/tools/pull_air_vacuum_pump')
        rospy.wait_for_service('niryo_one/tools/push_air_vacuum_pump')

        self.service_ping_dxl_tool = rospy.ServiceProxy('niryo_one/tools/ping_and_set_dxl_tool', PingDxlTool)

        self.service_open_gripper = rospy.ServiceProxy('niryo_one/tools/open_gripper', OpenGripper)
        self.service_close_gripper = rospy.ServiceProxy('niryo_one/tools/close_gripper', CloseGripper)

        self.service_pull_air_vacuum_pump = rospy.ServiceProxy('niryo_one/tools/pull_air_vacuum_pump',
                                                               PullAirVacuumPump)
        self.service_push_air_vacuum_pump = rospy.ServiceProxy('niryo_one/tools/push_air_vacuum_pump',
                                                               PushAirVacuumPump)

        self.service_setup_digital_output_tool = rospy.ServiceProxy('niryo_one/rpi/set_digital_io_mode', SetDigitalIO)
        self.service_activate_digital_output_tool = rospy.ServiceProxy('niryo_one/rpi/set_digital_io_state',
                                                                       SetDigitalIO)

        rospy.loginfo("Interface between Tools Controller and Ros Control has been started.")

    def ping_dxl_tool(self, tool_id, tool_name):
        try:
            if tool_id == 0:
                tool_name = "No Dxl Tool"
            resp = self.service_ping_dxl_tool(tool_id, tool_name)
            return resp.state
        except rospy.ServiceException, e:
            return ROS_COMMUNICATION_PROBLEM

    def open_gripper(self, gripper_id, open_position, open_speed, open_hold_torque):
        try:
            resp = self.service_open_gripper(gripper_id, open_position, open_speed, open_hold_torque)
            return resp.state
        except rospy.ServiceException, e:
            return ROS_COMMUNICATION_PROBLEM

    def close_gripper(self, gripper_id, close_position, close_speed, close_hold_torque, close_max_torque):
        try:
            resp = self.service_close_gripper(gripper_id, close_position, close_speed, close_hold_torque,
                                              close_max_torque)
            return resp.state
        except rospy.ServiceException, e:
            return ROS_COMMUNICATION_PROBLEM

    def pull_air_vacuum_pump(self, vp_id, vp_pull_air_position, vp_pull_air_hold_torque):
        try:
            resp = self.service_pull_air_vacuum_pump(vp_id, vp_pull_air_position, vp_pull_air_hold_torque)
            return resp.state
        except rospy.ServiceException, e:
            return ROS_COMMUNICATION_PROBLEM

    def push_air_vacuum_pump(self, vp_id, vp_push_air_position):
        try:
            resp = self.service_push_air_vacuum_pump(vp_id, vp_push_air_position)
            return resp.state
        except rospy.ServiceException, e:
            return ROS_COMMUNICATION_PROBLEM

    def digital_output_tool_setup(self, gpio_pin):
        try:
            rospy.wait_for_service('niryo_one/rpi/set_digital_io_mode', 2)
        except rospy.ROSException:
            return 400, "Digital IO panel service is not connected"
        try:
            resp = self.service_setup_digital_output_tool(gpio_pin, 0)  # set output
            return resp.status, resp.message
        except rospy.ServiceException, e:
            return 400, "Digital IO panel service failed"

    def digital_output_tool_activate(self, gpio_pin, activate):
        try:
            rospy.wait_for_service('niryo_one/rpi/set_digital_io_state', 2)
        except rospy.ROSException:
            return 400, "Digital IO panel service is not connected"
        try:
            resp = self.service_activate_digital_output_tool(gpio_pin, activate)
            return resp.status, resp.message
        except rospy.ServiceException, e:
            return 400, "Digital IO panel service failed"
