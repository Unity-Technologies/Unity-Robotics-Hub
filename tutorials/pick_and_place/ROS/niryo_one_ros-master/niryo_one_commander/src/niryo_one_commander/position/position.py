#!/usr/bin/env python
# position.py
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


class Position:
    class RPY:

        def __init__(self, roll=0, pitch=0, yaw=0):
            self.roll = roll
            self.pitch = pitch
            self.yaw = yaw

        def __str__(self):
            msg = str(self.roll) + "\n"
            msg += str(self.pitch) + "\n"
            msg += str(self.yaw) + "\n"
            return msg

        def __repr__(self):
            return self.__str__()

    class Point:

        def __init__(self, x=0, y=0, z=0):
            self.x = x
            self.y = y
            self.z = z

        def __str__(self):
            msg = str(self.x) + "\n"
            msg += str(self.y) + "\n"
            msg += str(self.z) + "\n"
            return msg

        def __repr__(self):
            return self.__str__()

    class Quaternion:

        def __init__(self, x=0, y=0, z=0, w=0):
            self.x = x
            self.y = y
            self.z = z
            self.w = w

        def __str__(self):
            msg = str(self.x) + "\n"
            msg += str(self.y) + "\n"
            msg += str(self.z) + "\n"
            msg += str(self.w) + "\n"
            return msg

        def __repr__(self):
            return self.__str__()

    def __init__(self, name="", joints=None, rpy=None, point=None, quaternion=None):
        self.name = name
        self.joints = joints if joints is not None else [0, 0, 0, 0, 0, 0]
        self.rpy = rpy if rpy is not None else self.RPY()
        self.point = point if point is not None else self.Point()
        self.quaternion = quaternion if quaternion is not None else self.Quaternion()

    def __str__(self):
        msg = self.name + "\n"
        msg += str(self.joints) + "\n"
        msg += str(self.rpy) + "\n"
        msg += str(self.point) + "\n"
        msg += str(self.quaternion) + "\n"
        return msg

    def __repr__(self):
        return self.__str__()
