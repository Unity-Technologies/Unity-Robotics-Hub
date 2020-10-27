#!/usr/bin/env python

# moveit_utils.py
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


import rospy
import tf
from niryo_one_commander.position.position import Position

from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK
from std_msgs.msg import Header


def get_forward_kinematic(joints):
    try:
        rospy.wait_for_service('compute_fk', 2)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr("Service call failed:", e)
        return None
    try:
        moveit_fk = rospy.ServiceProxy('compute_fk', GetPositionFK)
        fk_link = ['base_link', 'tool_link']
        joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        header = Header(0, rospy.Time.now(), "/world")
        rs = RobotState()
        rs.joint_state.name = joint_names
        rs.joint_state.position = joints
        response = moveit_fk(header, fk_link, rs)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed:", e)
        return None

    quaternion = [response.pose_stamped[1].pose.orientation.x, response.pose_stamped[1].pose.orientation.y,
                  response.pose_stamped[1].pose.orientation.z, response.pose_stamped[1].pose.orientation.w]
    rpy = get_rpy_from_quaternion(quaternion)
    quaternion = Position.Quaternion(round(quaternion[0], 3), round(quaternion[1], 3), round(quaternion[2], 3),
                                     round(quaternion[3], 3))
    point = Position.Point(round(response.pose_stamped[1].pose.position.x, 3),
                           round(response.pose_stamped[1].pose.position.y, 3),
                           round(response.pose_stamped[1].pose.position.z, 3))
    rpy = Position.RPY(round(rpy[0], 3), round(rpy[1], 3), round(rpy[2], 3))
    rospy.loginfo("kinematic forward has been calculated ")
    return point, rpy, quaternion


def get_rpy_from_quaternion(rot):
    PI = 3.14159
    euler = tf.transformations.euler_from_quaternion(rot)
    rpy = [0, 0, 0]
    rpy[0] = euler[0]
    rpy[1] = euler[1]
    rpy[2] = euler[2]

    # force angle between -PI/PI
    for i, angle in enumerate(rpy):
        if angle > PI:
            rpy[i] = angle % (2 * PI) - 2 * PI
        elif angle < -PI:
            rpy[i] = angle % (2 * PI)
    return rpy


if __name__ == '__main__':
    pass
