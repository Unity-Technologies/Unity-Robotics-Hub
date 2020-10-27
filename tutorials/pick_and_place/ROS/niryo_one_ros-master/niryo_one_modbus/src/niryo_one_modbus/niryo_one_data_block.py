#!/usr/bin/env python

# niryo_one_data_block.py
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
from pymodbus.datastore import ModbusSparseDataBlock


class NiryoOneDataBlock(ModbusSparseDataBlock):

    def __init__(self):
        values = [0] * 1000
        super(NiryoOneDataBlock, self).__init__(values)

    # Called from internal functions
    # Modbus addresses start at 1
    # There is an offset with what the client is asking
    def setValuesOffset(self, address, values):
        self.setValues(address + 1, values)

    def getValuesOffset(self, address, count=1):
        return self.getValues(address + 1, count)

    @staticmethod
    def call_ros_service(service_name, service_msg_type, args):
        # Connect to service
        try:
            rospy.wait_for_service(service_name, 0.1)
        except rospy.ROSException, e:
            return

            # Call service
        try:
            service = rospy.ServiceProxy(service_name, service_msg_type)
            response = service(*args)
            return response
        except rospy.ServiceException, e:
            return
