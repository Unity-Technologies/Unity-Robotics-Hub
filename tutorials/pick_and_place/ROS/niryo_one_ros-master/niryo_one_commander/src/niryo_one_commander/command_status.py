#!/usr/bin/env python

# command_status.py
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


class CommandStatus:
    # Success
    def __init__(self):
        pass

    SUCCESS = 1
    STOPPED = 2
    WAITING = 6

    # Rejected
    GOAL_STILL_ACTIVE = 10
    INVALID_PARAMETERS = 11
    LEARNING_MODE_ON = 12
    JOYSTICK_ENABLED = 13
    HARDWARE_NOT_OK = 14

    # Aborted
    NO_PLAN_AVAILABLE = 20
    PLAN_FAILED = 21
    CONTROLLER_PROBLEMS = 22
    STILL_RUNNING = 23
    TOOL_FAILED = 24
    SEQUENCE_FAILED = 25

    # ROS error
    ROS_ERROR = 30

    # Restart traj execution (internal only)
    SHOULD_RESTART = 40
