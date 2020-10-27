#!/usr/bin/env python

# command_type.py
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


class CommandType(object):
    JOINTS = 1
    POSE = 2
    POSITION = 3
    RPY = 4
    SHIFT_POSE = 5
    TOOL = 6
    EXECUTE_TRAJ = 7
    POSE_QUAT = 8
    SAVED_POSITION = 9
    SAVED_TRAJECTORY = 10
