#!/usr/bin/env python

# sequence.py
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


class Sequence:

    def __init__(self, identif=0, name="", description="", blockly_xml="", python_code=""):
        self.id = identif
        self.name = name
        self.description = description
        self.created = rospy.Time.now().secs
        self.updated = rospy.Time.now().secs
        self.blockly_xml = blockly_xml
        self.python_code = python_code
