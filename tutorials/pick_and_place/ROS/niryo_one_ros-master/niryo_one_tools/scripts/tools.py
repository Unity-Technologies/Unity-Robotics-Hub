#!/usr/bin/env python

# tools.py
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

class ToolValidationException(Exception): pass


TOOL_STATE_PING_OK = 0x01
TOOL_STATE_PING_ERROR = 0x02
TOOL_STATE_WRONG_ID = 0x03
TOOL_STATE_TIMEOUT = 0x04

GRIPPER_STATE_OPEN = 0x10
GRIPPER_STATE_CLOSE = 0x11

VACUUM_PUMP_STATE_PULLED = 0x20
VACUUM_PUMP_STATE_PUSHED = 0x21

ROS_COMMUNICATION_PROBLEM = 0xA0


#
# Base class for any tool
#
class ToolBase(object):

    # To override
    def get_type(self):
        pass

    # To override
    def validate_command(self, cmd):
        pass

    def set_as_active(self):
        self.is_active = True

    def set_as_non_active(self):
        self.is_active = False

    def get_id(self):
        return self.id

    def set_available_commands(self, cmd_list):
        self.available_commands = cmd_list

    def get_available_commands(self):
        return self.available_commands

    def __init__(self, tool_id, tool_name, ros_command_interface):
        self.id = tool_id
        self.name = tool_name
        self.ros_command_interface = ros_command_interface
        self.is_active = False
        self.available_commands = []


class Gripper(ToolBase):

    def get_type(self):
        return "gripper"

    def validate_command(self, cmd):
        if (cmd.gripper_open_speed < 0 or cmd.gripper_open_speed > 1023 or
                cmd.gripper_close_speed < 0 or cmd.gripper_close_speed > 1023):
            raise ToolValidationException("Gripper open/close speed must be in ( 0 , 1023 )")

    @staticmethod
    def return_gripper_status(state):
        if state == GRIPPER_STATE_OPEN:
            return True, "Successfully opened gripper"
        if state == GRIPPER_STATE_CLOSE:
            return True, "Successfully closed gripper"
        if state == TOOL_STATE_PING_OK:
            return True, "Gripper is connected"
        if state == TOOL_STATE_PING_ERROR:
            return False, "Gripper not detected"
        if state == TOOL_STATE_TIMEOUT:
            return False, "Gripper action - Timeout"
        if state == TOOL_STATE_WRONG_ID:
            return False, "This gripper is not the one attached"
        if state == ROS_COMMUNICATION_PROBLEM:
            return False, "A communication problem occured, please retry"

    def open_gripper(self, cmd):
        state = self.ros_command_interface.open_gripper(
            self.id, self.open_position, cmd.gripper_open_speed, self.open_hold_torque)
        return self.return_gripper_status(state)

    def close_gripper(self, cmd):
        state = self.ros_command_interface.close_gripper(
            self.id, self.close_position, cmd.gripper_close_speed, self.close_hold_torque, self.close_max_torque)
        return self.return_gripper_status(state)

    def is_connected(self):
        state = self.ros_command_interface.ping_dxl_tool(self.id, self.name)
        return state == TOOL_STATE_PING_OK

    def __init__(self, tool_id, tool_name, ros_command_interface, open_position, open_hold_torque,
                 close_position, close_hold_torque, close_max_torque):
        super(Gripper, self).__init__(tool_id, tool_name, ros_command_interface)
        self.open_position = open_position
        self.open_hold_torque = open_hold_torque
        self.close_position = close_position
        self.close_hold_torque = close_hold_torque
        self.close_max_torque = close_max_torque

    def update_params(self, open_position, open_hold_torque, close_position,
                      close_hold_torque, close_max_torque):
        self.open_position = open_position
        self.open_hold_torque = open_hold_torque
        self.close_position = close_position
        self.close_hold_torque = close_hold_torque
        self.close_max_torque = close_max_torque


class Electromagnet(ToolBase):

    def get_type(self):
        return "electromagnet"

    def validate_command(self, cmd):
        pass

    def setup_digital_io(self, cmd):
        status, message = self.ros_command_interface.digital_output_tool_setup(cmd.gpio)
        if status == 200:
            return True, 'Successfully setup digital output PIN  ' + str(cmd.gpio) + ' for electromagnet'
        else:
            return False, message

    def activate_digital_io(self, cmd):
        status, message = self.ros_command_interface.digital_output_tool_activate(cmd.gpio, 1)
        if status == 200:
            return True, 'Successfully activated eletromagnet on PIN  ' + str(cmd.gpio)
        else:
            return False, message

    def deactivate_digital_io(self, cmd):
        status, message = self.ros_command_interface.digital_output_tool_activate(cmd.gpio, 0)
        if status == 200:
            return True, 'Successfully deactivated eletromagnet on PIN  ' + str(cmd.gpio)
        else:
            return False, message

    def __init__(self, tool_id, tool_name, ros_command_interface):
        super(Electromagnet, self).__init__(tool_id, tool_name, ros_command_interface)


class VacuumPump(ToolBase):

    def get_type(self):
        return "vacuum_pump"

    def validate_command(self, cmd):
        pass  # activate True/False

    def is_connected(self):
        state = self.ros_command_interface.ping_dxl_tool(self.id, self.name)
        return state == TOOL_STATE_PING_OK

    @staticmethod
    def return_vaccump_pump_state(state):
        if state == VACUUM_PUMP_STATE_PULLED:
            return True, 'Successfully pulled air'
        elif state == VACUUM_PUMP_STATE_PUSHED:
            return True, 'Successfully pushed air'
        elif state == TOOL_STATE_PING_OK:
            return True, 'Vacuum Pump is connected'
        elif state == TOOL_STATE_PING_ERROR:
            return False, 'Vacuum pump not detected'
        elif state == TOOL_STATE_TIMEOUT:
            return False, 'Vacump pump action - Timeout'
        elif state == TOOL_STATE_WRONG_ID:
            return False, 'This vacuum pump is not the one attached'
        elif state == ROS_COMMUNICATION_PROBLEM:
            return False, "A communication problem occured, please retry"
        else:
            return False, "Error : Unknown vacuum pump return" + str(state)

    def pull_air_vacuum_pump(self, cmd):
        state = self.ros_command_interface.pull_air_vacuum_pump(
            self.id, self.pull_air_position, self.pull_air_hold_torque)
        return self.return_vaccump_pump_state(state)

    def push_air_vacuum_pump(self, cmd):
        state = self.ros_command_interface.push_air_vacuum_pump(
            self.id, self.push_air_position)
        return self.return_vaccump_pump_state(state)

    def __init__(self, tool_id, tool_name, ros_command_interface, pull_air_position,
                 pull_air_hold_torque, push_air_position):
        super(VacuumPump, self).__init__(tool_id, tool_name, ros_command_interface)
        self.pull_air_position = pull_air_position
        self.pull_air_hold_torque = pull_air_hold_torque
        self.push_air_position = push_air_position

    def update_params(self, pull_air_position, pull_air_hold_torque, push_air_position):
        self.pull_air_position = pull_air_position
        self.pull_air_hold_torque = pull_air_hold_torque
        self.push_air_position = self.push_air_position
