#!/usr/bin/env python

# arm_commander.py
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
import threading

from actionlib_msgs.msg import GoalStatus

from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import FollowJointTrajectoryActionGoal
from control_msgs.msg import FollowJointTrajectoryActionResult

# from niryo_one_commander.move_group_arm import MoveGroupArm
from niryo_one_commander.robot_commander_exception import RobotCommanderException
from niryo_one_commander.command_status import CommandStatus

TrajectoryTimeOutMin = 3


class ArmCommander:

    def __init__(self, move_group_arm):
        self.move_group_arm = move_group_arm
        self.traj_finished_event = threading.Event()
        self.current_goal_id = None
        self.current_goal_result = GoalStatus.LOST
        rospy.Subscriber('/niryo_one_follow_joint_trajectory_controller/follow_joint_trajectory/goal',
                         FollowJointTrajectoryActionGoal, self.callback_current_goal)

        rospy.Subscriber('/niryo_one_follow_joint_trajectory_controller/follow_joint_trajectory/result',
                         FollowJointTrajectoryActionResult, self.callback_goal_result)

        # Direct topic to joint_trajectory_controller
        # Used ONLY when goal is aborted, to enter position hold mode
        self.joint_trajectory_publisher = rospy.Publisher(
            '/niryo_one_follow_joint_trajectory_controller/command',
            JointTrajectory, queue_size=10)

    def execute_plan(self, plan):
        if plan:
            # reset event
            self.traj_finished_event.clear()
            self.current_goal_id = None
            self.current_goal_result = GoalStatus.LOST
            # send traj and wait
            self.move_group_arm.execute(plan, wait=False)
            trajectory_time_out = 1.5 * self.get_plan_time(plan)
            # if trajectory_time_out is less than 3 seconds, the default value will be 3 seconds
            if trajectory_time_out < TrajectoryTimeOutMin:
                trajectory_time_out = TrajectoryTimeOutMin
            if self.traj_finished_event.wait(trajectory_time_out):
                if self.current_goal_result == GoalStatus.SUCCEEDED:
                    return CommandStatus.SUCCESS, "Command has been successfully processed"
                elif self.current_goal_result == GoalStatus.PREEMPTED:
                    return CommandStatus.STOPPED, "Command has been successfully stopped"
                elif self.current_goal_result == GoalStatus.ABORTED:
                    # if joint_trajectory_controller aborts the goal, it will still try to
                    # finish executing the trajectory --> so we ask it to stop from here
                    self.set_position_hold_mode()
                    abort_str = "Command has been aborted due to a collision or " \
                                "a motor not able to follow the given trajectory"
                    return CommandStatus.CONTROLLER_PROBLEMS, abort_str

                else:  # problem from ros_control itself
                    self.current_goal_id = None
                    return CommandStatus.SHOULD_RESTART, ""
            else:
                # this timeout will happen if something fails in ros_control
                # not related to a trajectory failure
                self.current_goal_id = None
                return CommandStatus.SHOULD_RESTART, ""
        else:
            raise RobotCommanderException(CommandStatus.NO_PLAN_AVAILABLE,
                                          "You are trying to execute a plan which doesn't exist")

    # http://wiki.ros.org/joint_trajectory_controller -> preemption policy
    # Send an empty trajectory from the topic interface

    def set_position_hold_mode(self):
        msg = JointTrajectory()
        msg.header.stamp = rospy.Time.now()
        msg.points = []
        self.joint_trajectory_publisher.publish(msg)

    def stop_current_plan(self):
        rospy.loginfo("Send STOP to arm")
        self.move_group_arm.stop()

    def set_joint_target(self, joint_array):
        try:
            self.move_group_arm.set_joint_value_target(joint_array)
        except Exception, e:
            raise RobotCommanderException(CommandStatus.INVALID_PARAMETERS, str(e))

    def set_position_target(self, x, y, z):
        try:
            self.move_group_arm.set_position_target(x, y, z)
        except Exception, e:
            raise RobotCommanderException(CommandStatus.INVALID_PARAMETERS, str(e))

    def set_rpy_target(self, roll, pitch, yaw):
        try:
            self.move_group_arm.set_rpy_target(roll, pitch, yaw)
        except Exception, e:
            raise RobotCommanderException(CommandStatus.INVALID_PARAMETERS, str(e))

    def set_pose_target(self, x, y, z, roll, pitch, yaw):
        try:
            self.move_group_arm.set_pose_target(x, y, z, roll, pitch, yaw)
        except Exception, e:
            raise RobotCommanderException(CommandStatus.INVALID_PARAMETERS, str(e))

    def set_pose_quat_target(self, pose_msg):
        try:
            self.move_group_arm.set_pose_quat_target(pose_msg)
        except Exception, e:
            raise RobotCommanderException(CommandStatus.INVALID_PARAMETERS, str(e))

    def set_shift_pose_target(self, axis_number, value):
        try:
            self.move_group_arm.set_shift_pose_target(axis_number, value)
        except Exception, e:
            raise RobotCommanderException(CommandStatus.INVALID_PARAMETERS, str(e))

    def set_max_velocity_scaling_factor(self, percentage):
        try:
            self.move_group_arm.set_max_velocity_scaling_factor(percentage)
        except Exception, e:
            raise RobotCommanderException(400, str(e))

    @staticmethod
    def get_plan_time(plan):
        if plan:
            return plan.joint_trajectory.points[-1].time_from_start.to_sec()
        else:
            raise RobotCommanderException(CommandStatus.NO_PLAN_AVAILABLE,
                                          "No current plan found")

    def callback_current_goal(self, msg):
        self.current_goal_id = msg.goal_id.id
        rospy.loginfo("Arm commander - Got a goal id : " + str(self.current_goal_id))

    def callback_goal_result(self, msg):
        if msg.status.goal_id.id == self.current_goal_id:
            # rospy.loginfo("Receive result, goal_id matches.")
            self.current_goal_result = msg.status.status
            rospy.loginfo("Arm commander - Result : " + str(self.current_goal_result))
            self.traj_finished_event.set()
        else:
            rospy.loginfo("Arm commander - Received result, WRONG GOAL ID")
