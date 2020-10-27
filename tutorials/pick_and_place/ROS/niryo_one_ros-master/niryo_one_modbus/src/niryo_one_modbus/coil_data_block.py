#!/usr/bin/env python

# coil_data_block.py
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

from niryo_one_msgs.srv import SetDigitalIO

from niryo_one_modbus.niryo_one_data_block import NiryoOneDataBlock

"""
 - Each address contains a 1 bit value
 - READ/WRITE registers

 --> Used to give commands to the robot 
 ( ! the stored values correspond to the last given command,
 not the current robot state !)
"""

CO_DIGITAL_IO_MODE = 0
CO_DIGITAL_IO_STATE = 100

GPIO_1_A = 2
GPIO_1_B = 3
GPIO_1_C = 16
GPIO_2_A = 26
GPIO_2_B = 19
GPIO_2_C = 6


class CoilDataBlock(NiryoOneDataBlock):

    def __init__(self):
        super(CoilDataBlock, self).__init__()

    # Override
    def setValues(self, address, values):
        self.process_command(address, values)
        super(CoilDataBlock, self).setValues(address, values)

    def process_command(self, address, values):
        address -= 1
        if len(values) == 0:
            return

        value = values[0]
        pin = 0

        if address == 0 or address == 100:
            pin = GPIO_1_A
        elif address == 1 or address == 101:
            pin = GPIO_1_B
        elif address == 2 or address == 102:
            pin = GPIO_1_C
        elif address == 3 or address == 103:
            pin = GPIO_2_A
        elif address == 4 or address == 104:
            pin = GPIO_2_B
        elif address == 5 or address == 105:
            pin = GPIO_2_C

        if 0 <= address < 100:
            self.set_pin_mode(pin, value)
        elif 100 <= address < 200:
            self.set_pin_state(pin, value)

    def set_pin_mode(self, pin, mode):
        self.call_ros_service('/niryo_one/rpi/set_digital_io_mode', SetDigitalIO, [pin, mode])

    def set_pin_state(self, pin, state):
        self.call_ros_service('/niryo_one/rpi/set_digital_io_state', SetDigitalIO, [pin, state])
