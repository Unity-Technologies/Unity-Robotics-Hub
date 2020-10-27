#!/usr/bin/env python

# user_interface.py
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

from joystick_interface import JoystickInterface

from sequence_manager import SequenceManager
from sequence_action_server import SequenceActionServer
from sequence_autorun import SequenceAutorun
from matlab_manager import MatlabManager
from niryo_one_tcp_server.tcp_server import TcpServer


class UserInterface:

    def __init__(self):
        # Joystick
        self.joy = JoystickInterface()
        self.joy.disable_joy()

        # Sequence Manager
        sequences_dir = rospy.get_param("~sequences_dir")
        self.sequence_manager = SequenceManager(sequences_dir)

        # Sequence Action Server
        self.sequence_action_server = SequenceActionServer(self.sequence_manager)
        self.sequence_action_server.start()

        # Sequence Autorun
        self.sequence_autorun = SequenceAutorun()

        # Matlab node manager
        self.matlab_manager = MatlabManager()

        self.tcp_server = TcpServer()
        self.tcp_server.start()

    def shutdown(self):
        self.sequence_manager.shutdown()
        self.tcp_server.quit()


if __name__ == '__main__':
    rospy.init_node('user_interface')
    ui = UserInterface()
    rospy.on_shutdown(ui.shutdown)
    rospy.spin()
