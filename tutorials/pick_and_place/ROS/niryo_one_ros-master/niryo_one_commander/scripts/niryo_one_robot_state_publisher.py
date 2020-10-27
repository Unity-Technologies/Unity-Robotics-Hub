#!/usr/bin/env python

# niryo_one_robot_state_publisher.py
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
import tf
from niryo_one_commander.moveit_utils import get_rpy_from_quaternion
from tf.transformations import quaternion_from_euler

# from std_msgs.msg import Float64
from niryo_one_msgs.msg import RobotState
from geometry_msgs.msg import Quaternion

PI = 3.14159


class NiryoRobotStatePublisher:

    def __init__(self):

        # Tf listener (position + rpy) of end effector tool
        self.position = [0, 0, 0]
        self.rpy = [0, 0, 0]
        self.tf_listener = tf.TransformListener()

        # State publisher
        self.niryo_one_robot_state_publisher = rospy.Publisher(
            '/niryo_one/robot_state', RobotState, queue_size=5)

        # Get params from rosparams
        rate_tf_listener = rospy.get_param("/niryo_one/robot_state/rate_tf_listener")
        rate_publish_state = rospy.get_param("/niryo_one/robot_state/rate_publish_state")

        rospy.Timer(rospy.Duration(1.0 / rate_tf_listener), self.get_robot_pose)
        rospy.Timer(rospy.Duration(1.0 / rate_publish_state), self.publish_state)

        rospy.loginfo("Started Niryo One robot state publisher")

    @staticmethod
    def get_orientation_from_angles(r, p, y):
        quaternion = quaternion_from_euler(r, p, y)
        orientation = Quaternion()
        orientation.x = quaternion[0]
        orientation.y = quaternion[1]
        orientation.z = quaternion[2]
        orientation.w = quaternion[3]
        return orientation

    def get_robot_pose(self, event):
        try:
            (pos, rot) = self.tf_listener.lookupTransform('base_link', 'tool_link', rospy.Time(0))
            self.position = pos
            self.rpy = get_rpy_from_quaternion(rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("TF fail")

    def publish_state(self, event):
        msg = RobotState()
        msg.position.x = self.position[0]
        msg.position.y = self.position[1]
        msg.position.z = self.position[2]
        msg.rpy.roll = self.rpy[0]
        msg.rpy.pitch = self.rpy[1]
        msg.rpy.yaw = self.rpy[2]
        self.niryo_one_robot_state_publisher.publish(msg)
