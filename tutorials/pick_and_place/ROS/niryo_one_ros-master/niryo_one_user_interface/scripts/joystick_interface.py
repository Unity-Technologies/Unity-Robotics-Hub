#!/usr/bin/env python

# joystick_interface.py
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
import actionlib
import threading

from std_msgs.msg import Bool
from std_msgs.msg import Int32
from sensor_msgs.msg import Joy

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from niryo_one_msgs.srv import GetInt
from niryo_one_msgs.srv import SetInt

from niryo_one_msgs.msg import ToolAction
from niryo_one_msgs.msg import ToolGoal
from niryo_one_msgs.msg import ToolCommand

# AXES index table
AXE_JOY_L_H = 0
AXE_JOY_L_V = 1
AXE_LT = 2
AXE_JOY_R_H = 3
AXE_JOY_R_V = 4
AXE_RT = 5
AXE_ARROW_H = 6
AXE_ARROW_V = 7

# BUTTON index table
BUTTON_A = 0
BUTTON_B = 1
BUTTON_X = 2
BUTTON_Y = 3
BUTTON_LB = 4
BUTTON_RB = 5
BUTTON_BACK = 6
BUTTON_START = 7
BUTTON_SELECT = 8
BUTTON_JOY_L = 9
BUTTON_JOY_R = 10

POSITION_MODE = 100
JOINT_MODE = 101

MIN_DELTA_XYZ = 0.01
MAX_DELTA_XYZ = 0.05
MIN_DELTA_RPY = 0.1
MAX_DELTA_RPY = 0.5

MAX_MULTIPLIER = 0.15
MIN_MULTIPLIER = 0.01
DEFAULT_MULTIPLIER = 0.07
STEP_MULTIPLIER = 0.01

# Tools IDs (need to match tools ids in niryo_one_tools package)
TOOL_NONE = 0
TOOL_GRIPPER_1_ID = 11
TOOL_GRIPPER_2_ID = 12
TOOL_GRIPPER_3_ID = 13
TOOL_ELECTROMAGNET_1_ID = 30
TOOL_VACUUM_PUMP_1_ID = 31

# Tool commands
TOOL_OPEN_GRIPPER = 1
TOOL_CLOSE_GRIPPER = 2
TOOL_ACTIVATE_VP = 10
TOOL_DEACTIVATE_VP = 11


class JointMode:

    def __init__(self):

        # Get params from rosparams
        self.timer_rate = rospy.get_param("~joystick_timer_rate_sec")
        self.validation = rospy.get_param("/niryo_one/robot_command_validation")
        self.joint_mode_timer = None

        self.synchronization_needed = True
        self.is_enabled = False

        self.axes = [0, 0, 0, 0, 0, 0, 0, 0]
        self.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.joint_states = JointState()
        self.joint_cmd = [0, 0, 0, 0, 0, 0]
        self.multiplier = DEFAULT_MULTIPLIER
        self.learning_mode_on = True
        self.current_tool_id = 0
        self.is_tool_active = False

        self.joint_state_subscriber = rospy.Subscriber('/joint_states',
                                                       JointState, self.callback_joint_states)

        self.learning_mode_subscriber = rospy.Subscriber(
            '/niryo_one/learning_mode', Bool, self.callback_learning_mode)

        self.current_tool_id_subscriber = rospy.Subscriber(
            "/niryo_one/current_tool_id", Int32, self.callback_current_tool_id)

        self.joint_trajectory_publisher = rospy.Publisher(
            '/niryo_one_follow_joint_trajectory_controller/command',
            JointTrajectory, queue_size=10)

        # Publisher used to send info to Niryo One Studio, so the user can add a move block
        # by pressing a button on the joystick controller
        self.save_point_publisher = rospy.Publisher(
            "/niryo_one/blockly/save_current_point", Int32, queue_size=10)

        self.tool_action_client = actionlib.SimpleActionClient(
            'niryo_one/tool_action', ToolAction)
        self.tool_action_client.wait_for_server()

        self.joint_mode_timer = rospy.Timer(rospy.Duration(self.timer_rate), self.send_joint_trajectory)
        self.time_debounce_start_button = rospy.Time.now()
        self.time_debounce_A_button = rospy.Time.now()
        self.time_debounce_B_X_button = rospy.Time.now()  # common debounce for both buttons

    def increase_speed(self):
        self.multiplier += STEP_MULTIPLIER
        if self.multiplier > MAX_MULTIPLIER:
            self.multiplier = MAX_MULTIPLIER

    def decrease_speed(self):
        self.multiplier -= STEP_MULTIPLIER
        if self.multiplier < MIN_MULTIPLIER:
            self.multiplier = MIN_MULTIPLIER

    def blockly_save_current_point(self):
        msg = Int32()
        msg.data = 1
        self.save_point_publisher.publish(msg)

    def process_joy_message(self, joy):
        self.axes = joy.axes
        self.buttons = joy.buttons

        if self.buttons[BUTTON_RB]:
            self.increase_speed()
        elif self.buttons[BUTTON_LB]:
            self.decrease_speed()

        if self.buttons[BUTTON_START]:
            if rospy.Time.now() > self.time_debounce_start_button:
                self.time_debounce_start_button = rospy.Time.now() + rospy.Duration(0.5)  # debounce 0.5 sec
                self.set_learning_mode()
                self.synchronization_needed = True

        if self.buttons[BUTTON_A]:
            if rospy.Time.now() > self.time_debounce_A_button:
                self.time_debounce_A_button = rospy.Time.now() + rospy.Duration(0.2)
                self.blockly_save_current_point()

        if self.buttons[BUTTON_X]:
            if rospy.Time.now() > self.time_debounce_B_X_button:
                self.time_debounce_B_X_button = rospy.Time.now() + rospy.Duration(0.1)
                self.execute_tool_action(True)

        if self.buttons[BUTTON_B]:
            if rospy.Time.now() > self.time_debounce_B_X_button:
                self.time_debounce_B_X_button = rospy.Time.now() + rospy.Duration(0.1)
                self.execute_tool_action(False)

    def set_learning_mode(self):
        rospy.wait_for_service('niryo_one/activate_learning_mode')
        try:
            reset = rospy.ServiceProxy('niryo_one/activate_learning_mode', SetInt)
            if self.learning_mode_on:
                reset(0)
            else:
                reset(1)
        except rospy.ServiceException, e:
            rospy.logwarn("Could not set learning mode")

    def execute_tool_action(self, activate):
        if self.is_tool_active:
            return

        cmd_type = 0
        # Check which tool
        if self.current_tool_id in [TOOL_GRIPPER_1_ID, TOOL_GRIPPER_2_ID, TOOL_GRIPPER_3_ID]:
            # Send gripper command
            cmd_type = TOOL_OPEN_GRIPPER if activate else TOOL_CLOSE_GRIPPER
        elif self.current_tool_id in [TOOL_VACUUM_PUMP_1_ID]:
            # Send vacuum pump command
            cmd_type = TOOL_ACTIVATE_VP if activate else TOOL_DEACTIVATE_VP
        else:
            # No selected tool or (no gripper and no vacuum pump)
            return

        w = threading.Thread(target=self.send_tool_command, args=[self.current_tool_id, cmd_type])
        w.start()

    def send_tool_command(self, tool_id, cmd_type):
        self.is_tool_active = True
        if self.learning_mode_on:
            self.set_learning_mode()
        tool_cmd = ToolCommand()
        tool_cmd.tool_id = tool_id
        tool_cmd.cmd_type = cmd_type
        tool_cmd.gripper_close_speed = 500
        tool_cmd.gripper_open_speed = 500
        goal = ToolGoal()
        goal.cmd = tool_cmd
        self.tool_action_client.send_goal(goal)
        self.tool_action_client.wait_for_result()
        self.is_tool_active = False

    def enable(self):
        self.is_enabled = True

    def disable(self):
        self.is_enabled = False

    def callback_joint_states(self, joint_states):
        self.joint_states = joint_states

        if self.synchronization_needed:
            self.synchronization_needed = False
            self.joint_cmd = list(joint_states.position)
            return

        if (not self.is_enabled) or self.learning_mode_on:
            self.joint_cmd = list(joint_states.position)
            return

    def callback_learning_mode(self, msg):
        self.learning_mode_on = msg.data

    def callback_current_tool_id(self, msg):
        self.current_tool_id = msg.data

    def send_joint_trajectory(self, event):
        if not self.is_enabled:
            return

        if self.learning_mode_on:
            return

        can_send_trajectory = False
        for axe in self.axes:
            if abs(axe) > 0.1:
                can_send_trajectory = True

        if can_send_trajectory:
            positions = self.joint_cmd

            multiplier = self.multiplier

            positions[0] = positions[0] + self.axes[AXE_JOY_R_H] * multiplier
            positions[1] = positions[1] - self.axes[AXE_JOY_R_V] * multiplier
            positions[2] = positions[2] + self.axes[AXE_JOY_L_V] * multiplier
            positions[3] = positions[3] - self.axes[AXE_JOY_L_H] * multiplier
            positions[4] = positions[4] + self.axes[AXE_ARROW_V] * multiplier
            positions[5] = positions[5] - self.axes[AXE_ARROW_H] * multiplier

            self.validate_joints(positions)

            self.publish_joint_trajectory(positions, self.timer_rate)

    def publish_joint_trajectory(self, positions, duration):
        msg = JointTrajectory()
        msg.header.stamp = rospy.Time.now()
        msg.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(duration)
        msg.points = [point]

        self.joint_trajectory_publisher.publish(msg)

    def validate_joints(self, joint_array):
        v = self.validation['joint_limits']
        pos = joint_array
        safety = 0.0

        if joint_array[0] < v['j1']['min'] + safety:
            joint_array[0] = v['j1']['min'] + safety
        elif joint_array[0] > v['j1']['max'] - safety:
            joint_array[0] = v['j1']['max'] - safety

        if joint_array[1] < v['j2']['min'] + safety:
            joint_array[1] = v['j2']['min'] + safety
        elif joint_array[1] > v['j2']['max'] - safety:
            joint_array[1] = v['j2']['max'] - safety

        if joint_array[2] < v['j3']['min'] + safety:
            joint_array[2] = v['j3']['min'] + safety
        elif joint_array[2] > v['j3']['max'] - safety:
            joint_array[2] = v['j3']['max'] - safety

        if joint_array[3] < v['j4']['min'] + safety:
            joint_array[3] = v['j4']['min'] + safety
        elif joint_array[3] > v['j4']['max'] - safety:
            joint_array[3] = v['j4']['max'] - safety

        if joint_array[4] < v['j5']['min'] + safety:
            joint_array[4] = v['j5']['min'] + safety
        elif joint_array[4] > v['j5']['max'] - safety:
            joint_array[4] = v['j5']['max'] - safety

        if joint_array[5] < v['j6']['min'] + safety:
            joint_array[5] = v['j6']['min'] + safety
        elif joint_array[5] > v['j6']['max'] - safety:
            joint_array[5] = v['j6']['max'] - safety


class JoystickInterface:

    @staticmethod
    def can_enable():
        rospy.wait_for_service('/niryo_one/commander/is_active')
        try:
            is_active = rospy.ServiceProxy('/niryo_one/commander/is_active', GetInt)
            response = is_active()
            return response.value == 0
        except rospy.ServiceException, e:
            return False

    def enable_joy(self):
        rospy.loginfo("Enable joystick")
        self.joy_enabled = True
        self.joint_mode.enable()
        self.publish_joystick_enabled(None)

    def disable_joy(self):
        rospy.loginfo("Disable joystick")
        self.joy_enabled = False
        self.joint_mode.disable()
        self.publish_joystick_enabled(None)

    def callback_joy(self, joy):
        if self.joy_enabled:
            self.joint_mode.process_joy_message(joy)

    def __init__(self):
        self.joy_enabled = False
        self.joint_mode = JointMode()

        joy_subscriber = rospy.Subscriber('joy', Joy, self.callback_joy)

        self.joystick_server = rospy.Service(
            '/niryo_one/joystick_interface/enable', SetInt, self.callback_enable_joystick)

        self.joystick_enabled_publisher = rospy.Publisher('/niryo_one/joystick_interface/is_enabled',
                                                          Bool, queue_size=1)

        rospy.Timer(rospy.Duration(2), self.publish_joystick_enabled)

    def publish_joystick_enabled(self, event):
        msg = Bool()
        msg.data = self.joy_enabled
        self.joystick_enabled_publisher.publish(msg)

    def callback_enable_joystick(self, req):
        if req.value == 1:
            if self.can_enable():
                self.enable_joy()
                return {'status': 200, 'message': 'Joystick has been enabled'}
            else:
                return {'status': 400, 'message': 'Wait for the end of command to enable Joystick'}
        else:
            self.disable_joy()
            return {'status': 200, 'message': 'Joystick has been disabled'}


if __name__ == '__main__':
    # rospy.init_node('niryo_one_joystick')
    # joy = JoystickInterface()
    # joy.disable_joy()
    # rospy.spin()
    pass
