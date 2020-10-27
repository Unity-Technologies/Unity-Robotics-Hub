#!/usr/bin/env python

# shutdown_manager.py
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

from niryo_one_rpi.rpi_ros_utils import send_shutdown_command
from niryo_one_rpi.rpi_ros_utils import send_reboot_command

from niryo_one_msgs.srv import SetInt


class ShutdownManager:
    def __init__(self):
        self.shutdown_rpi_sever = rospy.Service('/niryo_one/rpi/shutdown_rpi',
                                                SetInt, self.callback_shutdown_rpi)
        rospy.loginfo("Shutdown Manager OK")

    @staticmethod
    def callback_shutdown_rpi(req):
        if req.value == 1:
            send_shutdown_command_thread = threading.Timer(1.0, send_shutdown_command)
            send_shutdown_command_thread.start()
            return {'status': 200, 'message': 'Robot is shutting down'}
        elif req.value == 2:
            send_reboot_command_thread = threading.Timer(1.0, send_reboot_command)
            send_reboot_command_thread.start()
            return {'status': 200, 'message': 'Robot is rebooting'}
        else:
            return {'status': 400,
                    'message': 'Incorrect value: 1 for shutdown, 2 for reboot'}
