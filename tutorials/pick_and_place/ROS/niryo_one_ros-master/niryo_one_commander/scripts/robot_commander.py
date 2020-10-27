#!/usr/bin/env python

# robot_commander.py
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
import sys
import moveit_commander
import actionlib
import threading

# Lib
from niryo_one_commander.command_type import CommandType
from niryo_one_commander.command_status import CommandStatus
from niryo_one_commander.robot_commander_exception import RobotCommanderException
# Messages
from std_msgs.msg import Empty
from niryo_one_msgs.msg import HardwareStatus
from std_msgs.msg import Bool
from std_msgs.msg import Int32

# Services
from std_srvs.srv import SetBool
from niryo_one_msgs.srv import GetInt
from niryo_one_msgs.srv import SetInt
# Action msgs
from niryo_one_msgs.msg import RobotMoveAction
from niryo_one_msgs.msg import RobotMoveResult
# Commanders
from arm_commander import ArmCommander
from tool_commander import ToolCommander
from niryo_one_commander.move_group_arm import MoveGroupArm
from niryo_one_commander.parameters_validation import ParametersValidation

# State publisher
from niryo_one_robot_state_publisher import NiryoRobotStatePublisher

"""
This class handles the arm and tools through a service interface 
- before you execute a command here, you need to validate params
  and check if no other action is being processed

"""


class RobotCommander:

    def __init__(self, position_manager, trajectory_manager):
        self.trajectory_manager = trajectory_manager
        self.pos_manager = position_manager
        moveit_commander.roscpp_initialize(sys.argv)

        # Load all the sub-commanders
        self.move_group_arm = MoveGroupArm()
        self.arm_commander = ArmCommander(self.move_group_arm)
        self.tool_commander = ToolCommander()

        self.stop_trajectory_server = rospy.Service(
            'niryo_one/commander/stop_command', SetBool, self.callback_stop_command)

        self.reset_controller_pub = rospy.Publisher('/niryo_one/steppers_reset_controller',
                                                    Empty, queue_size=1)

        # robot action server
        self.server = actionlib.ActionServer('niryo_one/commander/robot_action',
                                             RobotMoveAction, self.on_goal, self.on_cancel, auto_start=False)
        self.current_goal_handle = None
        self.learning_mode_on = False
        self.joystick_enabled = False
        self.hardware_status = None
        self.is_active_server = rospy.Service(
            'niryo_one/commander/is_active', GetInt, self.callback_is_active)

        self.learning_mode_subscriber = rospy.Subscriber(
            '/niryo_one/learning_mode', Bool, self.callback_learning_mode)
        self.joystick_enabled_subscriber = rospy.Subscriber('/niryo_one/joystick_interface/is_enabled',
                                                            Bool, self.callback_joystick_enabled)
        self.hardware_status_subscriber = rospy.Subscriber(
            '/niryo_one/hardware_status', HardwareStatus, self.callback_hardware_status)

        self.validation = rospy.get_param("/niryo_one/robot_command_validation")
        self.parameters_validation = ParametersValidation(self.validation)

        # arm velocity
        self.max_velocity_scaling_factor = 100
        self.max_velocity_scaling_factor_pub = rospy.Publisher(
            '/niryo_one/max_velocity_scaling_factor', Int32, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_arm_max_velocity_scaling_factor)
        self.max_velocity_scaling_factor_server = rospy.Service(
            '/niryo_one/commander/set_max_velocity_scaling_factor', SetInt,
            self.callback_set_max_velocity_scaling_factor)

    def compute_and_execute_plan(self):
        status = None
        message = None
        tries = 0
        while True:
            tries += 1

            if tries > 3:
                rospy.logerr("Big failure from the controller. Try to restart the robot")
                return CommandStatus.ROS_ERROR, "Please restart the robot and try again."

            # if we are re-trying, first stop current plan
            if tries > 1:
                self.arm_commander.stop_current_plan()
                rospy.sleep(0.1)
            plan = self.move_group_arm.compute_plan()
            if not plan:
                raise RobotCommanderException(
                    CommandStatus.PLAN_FAILED, "Moveit failed to compute the plan.")

            self.reset_controller()
            rospy.loginfo("Send Moveit trajectory to controller.")
            status, message = self.arm_commander.execute_plan(plan)

            if status != CommandStatus.SHOULD_RESTART:
                break
            rospy.logwarn("WILL RETRY COMPUTING AND EXECUTING TRAJECTORY.")

        return status, message

    def set_plan_and_execute(self, traj):
        self.reset_controller()
        rospy.loginfo("Send newly set trajectory to execute")
        if traj is None:
            raise RobotCommanderException(
                CommandStatus.PLAN_FAILED, "Moveit failed to execute plan.")
        return self.arm_commander.execute_plan(traj.trajectory)

    def reset_controller(self):
        msg = Empty()
        self.reset_controller_pub.publish(msg)
        # very important delay to avoid unexpected issues from ros_control
        rospy.sleep(0.05)

    @staticmethod
    def activate_learning_mode(activate):
        try:
            rospy.wait_for_service('/niryo_one/activate_learning_mode', 1)
            srv = rospy.ServiceProxy('/niryo_one/activate_learning_mode', SetInt)
            resp = srv(int(activate))
            return resp.status == 200
        except (rospy.ServiceException, rospy.ROSException), e:
            return False

    def publish_arm_max_velocity_scaling_factor(self, event):
        msg = Int32()
        msg.data = self.max_velocity_scaling_factor
        self.max_velocity_scaling_factor_pub.publish(msg)

    def callback_set_max_velocity_scaling_factor(self, req):
        if req.value <= 0 or req.value > 100:
            return {'status': 400, 'message': 'Value must be between 1 and 100'}
        try:
            self.arm_commander.set_max_velocity_scaling_factor(req.value / 100.0)
        except RobotCommanderException as e:
            return {'status': 400, 'message': e.message}
        self.max_velocity_scaling_factor = req.value
        self.publish_arm_max_velocity_scaling_factor(None)
        return {'status': 200, 'message': 'Success'}

    def set_saved_position(self, cmd):
        rospy.loginfo("set saved position")
        pos = self.pos_manager.get_position(cmd.saved_position_name)
        self.arm_commander.set_joint_target(pos.joints)

    def set_saved_trajectory(self, cmd):
        traj = self.trajectory_manager.get_trajectory(cmd.saved_trajectory_id)
        return self.set_plan_and_execute(traj.trajectory_plan)

    def execute_command(self, cmd):
        cmd_type = cmd.cmd_type
        status = CommandStatus.ROS_ERROR
        message = "Unknown problem occured during command execution"

        if cmd_type == CommandType.TOOL:
            self.tool_commander.send_tool_command(cmd.tool_cmd)
            status = CommandStatus.SUCCESS  # todo get status from send_tool_command
            message = "Tool command has been successfully processed"
        else:  # move command
            if cmd_type == CommandType.EXECUTE_TRAJ:
                status, message = self.set_plan_and_execute(cmd.Trajectory)
            elif cmd_type == CommandType.SAVED_TRAJECTORY:
                status, message = self.set_saved_trajectory(cmd)
            else:
                if cmd_type == CommandType.JOINTS:
                    self.arm_commander.set_joint_target(cmd.joints)
                elif cmd_type == CommandType.POSE:
                    self.arm_commander.set_pose_target(cmd.position.x, cmd.position.y, cmd.position.z,
                                                       cmd.rpy.roll, cmd.rpy.pitch, cmd.rpy.yaw)
                elif cmd_type == CommandType.POSITION:
                    self.arm_commander.set_position_target(cmd.position.x, cmd.position.y, cmd.position.z)
                elif cmd_type == CommandType.RPY:
                    self.arm_commander.set_rpy_target(cmd.rpy.roll, cmd.rpy.pitch, cmd.rpy.yaw)
                elif cmd_type == CommandType.SHIFT_POSE:
                    self.arm_commander.set_shift_pose_target(cmd.shift.axis_number, cmd.shift.value)
                elif cmd_type == CommandType.POSE_QUAT:
                    self.arm_commander.set_pose_quat_target(cmd.pose_quat)
                elif cmd_type == CommandType.SAVED_POSITION:
                    self.set_saved_position(cmd)

                status, message = self.compute_and_execute_plan()
        return status, message

    def cancel_command(self):
        self.arm_commander.stop_current_plan()
        self.tool_commander.stop_tool_command()  # todo handle goal cancelation for tools (client side)

    def callback_stop_command(self, req):
        self.cancel_command()
        return True, "Command stopped"

    # Robot action server functions 
    # - Check if no other command is being processed
    # - Validate params
    # - Execute command and return status + message
    # - Cancel command if asked
    def start(self):
        self.server.start()
        rospy.loginfo("Action Server started (Robot Commander)")

    @staticmethod
    def create_result(status, message):
        result = RobotMoveResult()
        result.status = status
        result.message = message
        return result

    def callback_learning_mode(self, msg):
        activate = msg.data

        if not self.learning_mode_on and activate:
            self.cancel_current_command()

        self.learning_mode_on = activate

    def callback_joystick_enabled(self, msg):
        self.joystick_enabled = msg.data

    def callback_hardware_status(self, msg):
        self.hardware_status = msg

    def callback_is_active(self, req):
        if self.current_goal_handle is not None:
            return 1
        return 0

    def on_goal(self, goal_handle):
        rospy.loginfo("Robot Action Server - Received goal. Check if exists")

        # Check if hw status has been received at least once
        if self.hardware_status is None:
            result = self.create_result(CommandStatus.HARDWARE_NOT_OK,
                                        "Hardware Status still not received, please restart the robot")
            goal_handle.set_rejected(result)
            return

        # Check if motor connection problem
        if not self.hardware_status.connection_up:
            result = self.create_result(CommandStatus.HARDWARE_NOT_OK,
                                        "Motor connection problem, you can't send a command now")
            goal_handle.set_rejected(result)
            return

        # Check if calibration is needed
        if self.hardware_status.calibration_needed == 1:
            result = self.create_result(CommandStatus.HARDWARE_NOT_OK,
                                        "You need to calibrate the robot before sending a command")
            goal_handle.set_rejected(result)
            return

        # Check if calibration is in progress
        if self.hardware_status.calibration_in_progress:
            result = self.create_result(CommandStatus.HARDWARE_NOT_OK,
                                        "Calibration in progress, wait until it ends to send a command")
            goal_handle.set_rejected(result)
            return

        # Check if joystick enabled
        if self.joystick_enabled:
            result = self.create_result(CommandStatus.JOYSTICK_ENABLED,
                                        "You need to deactivate joystick to execute a new command")
            goal_handle.set_rejected(result)
            return

        # check if still have a goal -> set_rejected() 
        if self.current_goal_handle is not None:
            result = self.create_result(CommandStatus.GOAL_STILL_ACTIVE,
                                        "Current command is still active. Cancel it if you want to execute a new one")
            goal_handle.set_rejected(result)
            return

        # validate parameters -> set_rejected (msg : validation or commander error)
        try:
            rospy.loginfo("Robot Action Server - Checking parameters Validity")
            self.validate_params(goal_handle.goal.goal.cmd)
        except RobotCommanderException as e:
            result = self.create_result(e.status, e.message)
            goal_handle.set_rejected(result)
            rospy.loginfo("Robot Action Server - Invalid parameters")
            return

        # Check if learning mode ON
        if self.learning_mode_on:
            if not self.activate_learning_mode(False):
                result = self.create_result(CommandStatus.LEARNING_MODE_ON,
                                            "Learning mode could not be deactivated")
                goal_handle.set_rejected(result)
                return

        # set accepted
        self.current_goal_handle = goal_handle
        self.current_goal_handle.set_accepted()
        rospy.loginfo("Robot Action Server - Goal has been accepted")

        # Launch compute + execution in a new thread
        w = threading.Thread(name="worker", target=self.execute_command_action)
        w.start()
        rospy.loginfo("Robot Action Server : Executing command in a new thread")

    def on_cancel(self, goal_handle):
        rospy.loginfo("Received cancel command")

        if goal_handle == self.current_goal_handle:
            self.cancel_current_command()
        else:
            rospy.loginfo("Robot Action Server - No current goal, nothing to do")

    def execute_command_action(self):
        cmd = self.current_goal_handle.goal.goal.cmd
        result = self.create_result(CommandStatus.ROS_ERROR, "error with executing command")
        response = None
        try:
            (status, message) = self.execute_command(cmd)
            response = self.create_result(status, message)
            result = response
        except RobotCommanderException as e:
            result = self.create_result(e.status, e.message)
            rospy.loginfo("An exception was thrown during command execution")

        if not response:
            self.current_goal_handle.set_aborted(result)
            rospy.logwarn("Execution has been aborted")
        elif response.status == CommandStatus.SUCCESS:
            self.current_goal_handle.set_succeeded(result)
            rospy.loginfo("Goal has been set as succeeded")
        elif response.status == CommandStatus.STOPPED:
            self.current_goal_handle.set_canceled(result)
            rospy.loginfo("Goal has been successfully canceled")
        elif response.status == CommandStatus.CONTROLLER_PROBLEMS:
            self.current_goal_handle.set_aborted(result)
            rospy.logwarn("Controller failed during execution : Goal has been aborted.\n" + \
                          "This is due to either a collision, or a motor unable to follow a given command" + \
                          " (overload, extreme positions, ...)")
        else:
            self.current_goal_handle.set_aborted(result)
            rospy.loginfo("Unknown result, goal has been set as aborted")
        self.current_goal_handle = None

    # Send a cancel signal to Moveit interface
    def cancel_current_command(self):
        try:
            self.cancel_command()
        except RobotCommanderException:
            rospy.logwarn("Could not cancel current command ")

    def validate_params(self, cmd):
        cmd_type = cmd.cmd_type
        if cmd_type == CommandType.JOINTS:
            self.parameters_validation.validate_joints(cmd.joints)
        elif cmd_type == CommandType.POSE:
            self.parameters_validation.validate_position(cmd.position)
            self.parameters_validation.validate_orientation(cmd.rpy)
        elif cmd_type == CommandType.POSITION:
            self.parameters_validation.validate_position(cmd.position)
        elif cmd_type == CommandType.RPY:
            self.parameters_validation.validate_orientation(cmd.rpy)
        elif cmd_type == CommandType.SHIFT_POSE:
            self.parameters_validation.validate_shift_pose(cmd.shift)
        elif cmd_type == CommandType.EXECUTE_TRAJ:
            self.parameters_validation.validate_trajectory(cmd.Trajectory)
        elif cmd_type == CommandType.TOOL:
            self.parameters_validation.validate_tool_command(cmd.tool_cmd)
        elif cmd_type == CommandType.POSE_QUAT:
            self.parameters_validation.validate_position(cmd.pose_quat.position)
            self.parameters_validation.validate_orientation_quaternion(cmd.pose_quat.orientation)
        elif cmd_type == CommandType.SAVED_POSITION:
            self.validate_saved_position(cmd.saved_position_name)
        elif CommandType.SAVED_TRAJECTORY:
            self.validate_saved_trajectory(cmd)

        else:
            raise RobotCommanderException(CommandStatus.INVALID_PARAMETERS, "Wrong command type")

    def validate_saved_trajectory(self, cmd):
        rospy.loginfo("Checking saved trajectory validity")
        saved_traj = self.trajectory_manager.get_trajectory(cmd.saved_trajectory_id)
        if saved_traj is None:
            raise RobotCommanderException(CommandStatus.INVALID_PARAMETERS, "Saved trajectory  not found")
        self.parameters_validation.validate_trajectory(saved_traj.trajectory_plan)

    def validate_saved_position(self, position_name):
        rospy.loginfo("Checking joints validity")
        saved_position = self.pos_manager.get_position(position_name)
        if saved_position is None:
            raise RobotCommanderException(CommandStatus.INVALID_PARAMETERS, "Saved position not found")
        self.parameters_validation.validate_joints(saved_position.joints)


if __name__ == '__main__':
    pass
