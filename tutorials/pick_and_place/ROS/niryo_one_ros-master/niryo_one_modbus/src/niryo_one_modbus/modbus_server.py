#!/usr/bin/env python

# modbus_server.py
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

from threading import Thread

from pymodbus.server.sync import ModbusTcpServer
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext

from niryo_one_modbus.coil_data_block import CoilDataBlock
from niryo_one_modbus.discrete_input_data_block import DiscreteInputDataBlock
from niryo_one_modbus.input_register_data_block import InputRegisterDataBlock
from niryo_one_modbus.holding_register_data_block import HoldingRegisterDataBlock


class ModbusServer:

    def __init__(self, address, port):
        self.coil = CoilDataBlock()
        self.discrete_input = DiscreteInputDataBlock()
        self.input_register = InputRegisterDataBlock()
        self.holding_register = HoldingRegisterDataBlock()
        self.store = ModbusSlaveContext(di=self.discrete_input,
                                        co=self.coil, hr=self.holding_register, ir=self.input_register)
        self.context = ModbusServerContext(slaves=self.store, single=True)

        self.identity = ModbusDeviceIdentification()
        self.identity.VendorName = 'pymodbus'
        self.identity.VendorName = 'pymodbus'
        self.identity.VendorUrl = 'http://github.com/bashwork/pymodbus/'
        self.identity.ProductName = 'pymodbus Server'
        self.identity.ModelName = 'pymodbus Server'
        self.identity.MajorMinorRevision = '1.0'

        self.server = ModbusTcpServer(context=self.context,
                                      framer=None, identity=self.identity, address=(address, port))

    def start(self):
        rospy.loginfo("Start Modbus Server")
        t = Thread(target=self.__start_server)
        t.start()

    def __start_server(self):
        self.discrete_input.start_ros_subscribers()
        self.input_register.start_ros_subscribers()
        self.server.serve_forever()

    def stop(self):
        rospy.loginfo("Stop Modbus Server")
        self.discrete_input.stop_ros_subscribers()
        self.input_register.stop_ros_subscribers()
        rospy.sleep(0.1)
        self.server.server_close()
        self.server.shutdown()
