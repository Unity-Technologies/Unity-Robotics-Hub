#!/usr/bin/env python

from __future__ import print_function

import rospy

from niryo_moveit.srv import MoverService, MoverServiceRequest, MoverServiceResponse

import sys
import copy
import math
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume

from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion, Pose
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

joint_names = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']

def plan_trajectory(move_group, destination_pose, start_joint_angles): 
    current_joint_state = JointState()
    current_joint_state.name = joint_names
    current_joint_state.position = start_joint_angles

    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = current_joint_state
    move_group.set_start_state(moveit_robot_state)

    move_group.set_pose_target(destination_pose)
    plan = move_group.go(wait=True)

    if not plan:
        print("RAISE NO PLAN ERROR")

    return move_group.plan()


# https://github.com/ros-planning/moveit/blob/master/moveit_commander/src/moveit_commander/move_group.py
def plan_pick_and_place(req):
    response = MoverServiceResponse()

    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # TODO: Make message type a list instead
    current_robot_joint_configuration = [
        math.radians(req.joints_input.joint_00),
        math.radians(req.joints_input.joint_01),
        math.radians(req.joints_input.joint_02),
        math.radians(req.joints_input.joint_03),
        math.radians(req.joints_input.joint_04),
        math.radians(req.joints_input.joint_05),
    ]


    # Pre grasp
    pre_grasp_pose = plan_trajectory(move_group, req.pick_pose, current_robot_joint_configuration)

    if not pre_grasp_pose:
        print("RAISE ERROR PRE GRASP POSE")

    previous_ending_joint_angles = pre_grasp_pose.joint_trajectory.points[-1].positions
    response.trajectories.append(pre_grasp_pose)

    # Grasp
    pick_pose = req.pick_pose
    pick_pose.position.z += 0.05 # Static value coming from Unity, TODO: pass along with request
    grasp_pose = plan_trajectory(move_group, pick_pose, previous_ending_joint_angles)

    if not grasp_pose:
        print("RAISE ERROR GRASP POSE")

    previous_ending_joint_angles = grasp_pose.joint_trajectory.points[-1].positions
    response.trajectories.append(grasp_pose)

    # Pick Up
    pick_up_pose = plan_trajectory(move_group, req.pick_pose, previous_ending_joint_angles)

    if not pick_up_pose:
        print("RAISE ERROR PICK UP POSE")

    previous_ending_joint_angles = pick_up_pose.joint_trajectory.points[-1].positions
    response.trajectories.append(pick_up_pose)

    # Place
    place_pose = plan_trajectory(move_group, req.place_pose, previous_ending_joint_angles)

    if not place_pose:
        print("RAISE ERROR PLACE POSE")

    previous_ending_joint_angles = place_pose.joint_trajectory.points[-1].positions
    response.trajectories.append(place_pose)

    move_group.clear_pose_targets()

    return response

def moveit_server():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('niryo_moveit_server')

    s = rospy.Service('niryo_moveit', MoverService, plan_pick_and_place)
    print("Ready to plan")
    rospy.spin()

if __name__ == "__main__":
    moveit_server()