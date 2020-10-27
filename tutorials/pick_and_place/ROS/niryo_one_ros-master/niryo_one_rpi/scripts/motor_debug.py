#!/usr/bin/env python

# motor_debug.py
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
import threading

import niryo_one_rpi.rpi_ros_utils as rpi_utils
from niryo_one_msgs.srv import ChangeMotorConfig


class MotorDebug:

    def __init__(self):
        self.hw_version = rospy.get_param("/niryo_one/hardware_version")
        self.change_motor_config_server = rospy.Service('/niryo_one/debug_change_motor_config',
                                                        ChangeMotorConfig, self.callback_change_motor_config)
        rospy.loginfo("Init motor debug OK")

    def callback_change_motor_config(self, req):
        rospy.loginfo("Change motor config")
        can_ids = list(req.can_required_motor_id_list)
        dxl_ids = list(req.dxl_required_motor_id_list)

        # 0. Remove impossible IDs (axis 1-6)
        if self.hw_version == 1:
            for identif in can_ids:
                if identif not in [1, 2, 3, 4]:
                    can_ids.remove(identif)
            for identif in dxl_ids:
                if identif not in [4, 5, 6]:
                    can_ids.remove(identif)
        elif self.hw_version == 2:
            for identif in can_ids:
                if identif not in [1, 2, 3]:
                    can_ids.remove(identif)
            for identif in dxl_ids:
                if identif not in [2, 3, 6]:
                    dxl_ids.remove(identif)

        enable_can_bus = len(can_ids) != 0
        enable_dxl_bus = len(dxl_ids) != 0

        # 1. Enable/Disable motor buses
        enable_bus_result = rpi_utils.enable_bus_motors_in_config_file(
            enable_can_bus=enable_can_bus, enable_dxl_bus=enable_dxl_bus)

        if enable_bus_result == rpi_utils.ENABLE_BUS_MOTORS_READ_FAIL:
            return {'status': 400, 'message': 'Failed to read file - see ROS log'}
        elif enable_bus_result == rpi_utils.ENABLE_BUS_MOTORS_WRITE_FAIL:
            return {'status': 400, 'message': 'Failed to write file - see ROS log'}

        # 2. Change motor config file
        motor_config_result = rpi_utils.change_motor_config_file(
            self.hw_version, can_ids, dxl_ids)

        if motor_config_result == rpi_utils.CHANGE_MOTOR_CONFIG_READ_FAIL:
            return {'status': 400, 'message': 'Failed to read file - see ROS log'}
        elif motor_config_result == rpi_utils.CHANGE_MOTOR_CONFIG_WRITE_FAIL:
            return {'status': 400, 'message': 'Failed to write file - see ROS log'}
        elif motor_config_result == rpi_utils.CHANGE_MOTOR_CONFIG_WRONG_VERSION:
            return {'status': 400, 'message': 'Wrong hardware version!'}

        # 3. Reboot
        send_reboot_command_thread = threading.Timer(1.0, rpi_utils.send_reboot_command)
        send_reboot_command_thread.start()
        return {'status': 200, 'message': 'Motor config successfully changed. Now rebooting...'}
