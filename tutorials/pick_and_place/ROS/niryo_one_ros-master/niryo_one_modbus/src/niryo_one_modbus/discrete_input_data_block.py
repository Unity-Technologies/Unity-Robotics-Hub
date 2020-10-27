#!/usr/bin/env python

# discrete_input_data_block.py
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
from niryo_one_msgs.msg import DigitalIOState

from niryo_one_modbus.niryo_one_data_block import NiryoOneDataBlock

"""
 - Each address contains a 1 bit value
 - READ ONLY registers

 --> State of the robot
"""

DI_DIGITAL_IO_MODE = 0
DI_DIGITAL_IO_STATE = 100


class DiscreteInputDataBlock(NiryoOneDataBlock):

    def __init__(self):
        super(DiscreteInputDataBlock, self).__init__()
        self.digital_io_state_sub = None

    def start_ros_subscribers(self):
        self.digital_io_state_sub = rospy.Subscriber('/niryo_one/rpi/digital_io_state', DigitalIOState,
                                                     self.sub_digital_io_state)

    def stop_ros_subscribers(self):
        self.digital_io_state_sub.unregister()

    def sub_digital_io_state(self, msg):
        self.setValuesOffset(DI_DIGITAL_IO_MODE, list(msg.modes))
        self.setValuesOffset(DI_DIGITAL_IO_STATE, list(msg.states))
