#!/usr/bin/env python

# holding_register_data_block.py
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
from niryo_one_msgs.srv import SetInt
from niryo_one_msgs.srv import ControlConveyor, SetConveyor, UpdateConveyorId


from niryo_one_modbus.niryo_one_data_block import NiryoOneDataBlock

import actionlib
from actionlib_msgs.msg import GoalStatus
from niryo_one_msgs.msg import RobotMoveAction
from niryo_one_msgs.msg import RobotMoveGoal
from niryo_one_commander.command_type import CommandType as MoveCommandType

import threading

"""
 - Each address contains a 16 bits value
 - READ/WRITE registers

 --> Used to give commands to the robot 
 ( ! the stored values correspond to the last given command,
 not the current robot state !)
"""
# define Conveyor ids  
CONVEYOR_ID_ONE = 6
CONVEYOR_ID_TWO = 7

HR_JOINTS = 0
HR_POSITION_X = 10
HR_POSITION_Y = 11
HR_POSITION_Z = 12
HR_ORIENTATION_X = 13
HR_ORIENTATION_Y = 14
HR_ORIENTATION_Z = 15

HR_MOVE_JOINTS_COMMAND = 100
HR_MOVE_POSE_COMMAND = 101
HR_STOP_COMMAND = 110

# You should not write any value on those 2 addresses
# Only read to get info about command execution
HR_IS_EXECUTING_CMD = 150
HR_LAST_ROBOT_CMD_RESULT = 151

HR_LEARNING_MODE = 300
HR_JOYSTICK_ENABLED = 301

HR_NEW_CALIBRATION_REQUEST = 310
HR_START_AUTO_CALIBRATION = 311
HR_START_MANUAL_CALIBRATION = 312

HR_GRIPPER_OPEN_SPEED = 401
HR_GRIPPER_CLOSE_SPEED = 402

HR_SELECT_TOOL_FROM_ID = 500

HR_OPEN_GRIPPER = 510
HR_CLOSE_GRIPPER = 511
HR_PULL_AIR_VACUUM_PUMP = 512
HR_PUSH_AIR_VACUUM_PUMP = 513

# conveyor commands 
HR_SET_CONVEYOR_FROM_ID = 520
HR_DETACH_CONVEYOR_FROM_ID = 521
HR_CONTROL_CONVEYOR = 522
HR_CONTROL_CONVEYOR_DIRECTION = 523
HR_CONTROL_CONVEYOR_SPEED = 524
HR_UPDATE_CONVEYOR_ID_TO_NEW_ID = 525
HR_STOP_CONVEYOR_WITH_ID = 526

# Positive number : 0 - 32767
# Negative number : 32768 - 65535
def handle_negative_hr(val):
    if (val >> 15) == 1:
        val = - (val & 0x7FFF)
    return val

class HoldingRegisterDataBlock(NiryoOneDataBlock):

    def __init__(self):
        super(HoldingRegisterDataBlock, self).__init__()
        self.execution_thread = threading.Thread()
        self.is_action_client_running = False
        self.cmd_action_client = None
        self.tool_command_list = rospy.get_param("/niryo_one_tools/command_list")

        # Override

    def setValues(self, address, values):
        self.process_command(address, values)
        super(HoldingRegisterDataBlock, self).setValues(address, values)

    def process_command(self, address, values):
        address -= 1
        if len(values) == 0:
            return

        if address == HR_LEARNING_MODE:
            self.activate_learning_mode(values[0])
        elif address == HR_JOYSTICK_ENABLED:
            self.enable_joystick(values[0])
        elif address == HR_MOVE_JOINTS_COMMAND:
            self.move_joints_command()
        elif address == HR_MOVE_POSE_COMMAND:
            self.move_pose_command()
        elif address == HR_STOP_COMMAND:
            self.stop_current_command()
        elif address == HR_NEW_CALIBRATION_REQUEST:
            self.request_new_calibration()
        elif address == HR_START_AUTO_CALIBRATION:
            self.start_auto_calibration()
        elif address == HR_START_MANUAL_CALIBRATION:
            self.start_manual_calibration()
        elif address == HR_SELECT_TOOL_FROM_ID:
            self.select_tool(values[0])
        elif address == HR_OPEN_GRIPPER:
            self.open_gripper_command(values[0])
        elif address == HR_CLOSE_GRIPPER:
            self.close_gripper_command(values[0])
        elif address == HR_PULL_AIR_VACUUM_PUMP:
            self.pull_air_vacuum_pump_command(values[0])
        elif address == HR_PUSH_AIR_VACUUM_PUMP:
            self.push_air_vacuum_pump_command(values[0])
        elif address == HR_SET_CONVEYOR_FROM_ID:
            self.set_conveyor(values[0])
        elif address == HR_DETACH_CONVEYOR_FROM_ID: 
            self.detach_conveyor(values[0])
        elif address == HR_UPDATE_CONVEYOR_ID_TO_NEW_ID: 
            self.update_conveyor_id(values[0])
        elif address == HR_CONTROL_CONVEYOR: 
            self.control_conveyor(values[0])
        elif address == HR_STOP_CONVEYOR_WITH_ID: 
            self.stop_conveyor(values[0])


    def request_new_calibration(self):
        self.call_ros_service('/niryo_one/request_new_calibration', SetInt, [1])

    def start_auto_calibration(self):
        self.call_ros_service('/niryo_one/calibrate_motors', SetInt, [1])

    def start_manual_calibration(self):
        self.call_ros_service('/niryo_one/calibrate_motors', SetInt, [2])

    def activate_learning_mode(self, activate):
        activate = int(activate >= 1)
        self.call_ros_service('/niryo_one/activate_learning_mode', SetInt, [activate])

    def enable_joystick(self, enable):
        enable = int(enable >= 1)
        self.call_ros_service('/niryo_one/joystick_interface/enable', SetInt, [enable])

    def stop_current_command(self):
        if self.is_action_client_running:
            self.cmd_action_client.cancel_goal()

    def select_tool(self, tool_id):
        self.call_ros_service('/niryo_one/change_tool', SetInt, [int(tool_id)])

    def open_gripper_command(self, tool_id):
        speed = self.getValuesOffset(HR_GRIPPER_OPEN_SPEED, 1)[0]
        if speed < 100:
            speed = 100
        elif speed > 1000:
            speed = 1000
        self.open_gripper(tool_id, speed)

    def close_gripper_command(self, tool_id):
        speed = self.getValuesOffset(HR_GRIPPER_CLOSE_SPEED, 1)[0]
        if speed < 100:
            speed = 100
        elif speed > 1000:
            speed = 1000
        self.close_gripper(tool_id, speed)

    def pull_air_vacuum_pump_command(self, tool_id):
        self.pull_air_vacuum_pump(tool_id)

    def push_air_vacuum_pump_command(self, tool_id):
        self.push_air_vacuum_pump(tool_id)

    def move_joints_command(self):
        joints_raw_values = self.getValuesOffset(HR_JOINTS, 6)
        joints = []
        for j in joints_raw_values:
            joints.append(handle_negative_hr(j) / 1000.0)
        self.move_joints(joints)

    def move_pose_command(self):
        pose_raw_values = self.getValuesOffset(HR_POSITION_X, 6)
        pose = []
        for p in pose_raw_values:
            pose.append(handle_negative_hr(p) / 1000.0)
        self.move_pose(pose)

    def open_gripper(self, gripper_id, speed):
        goal = RobotMoveGoal()
        goal.cmd.cmd_type = MoveCommandType.TOOL
        goal.cmd.tool_cmd.tool_id = int(gripper_id)
        goal.cmd.tool_cmd.cmd_type = self.tool_command_list['open_gripper']
        goal.cmd.tool_cmd.gripper_open_speed = speed
        self.start_execution_thread(goal)

    def close_gripper(self, gripper_id, speed):
        goal = RobotMoveGoal()
        goal.cmd.cmd_type = MoveCommandType.TOOL
        goal.cmd.tool_cmd.tool_id = int(gripper_id)
        goal.cmd.tool_cmd.cmd_type = self.tool_command_list['close_gripper']
        goal.cmd.tool_cmd.gripper_close_speed = speed
        self.start_execution_thread(goal)

    def pull_air_vacuum_pump(self, vacuum_pump_id):
        goal = RobotMoveGoal()
        goal.cmd.cmd_type = MoveCommandType.TOOL
        goal.cmd.tool_cmd.tool_id = int(vacuum_pump_id)
        goal.cmd.tool_cmd.cmd_type = self.tool_command_list['pull_air_vacuum_pump']
        self.start_execution_thread(goal)

    def push_air_vacuum_pump(self, vacuum_pump_id):
        goal = RobotMoveGoal()
        goal.cmd.cmd_type = MoveCommandType.TOOL
        goal.cmd.tool_cmd.tool_id = int(vacuum_pump_id)
        goal.cmd.tool_cmd.cmd_type = self.tool_command_list['push_air_vacuum_pump']
        self.start_execution_thread(goal)

    def move_pose(self, pose):
        goal = RobotMoveGoal()
        goal.cmd.cmd_type = MoveCommandType.POSE
        goal.cmd.position.x = pose[0]
        goal.cmd.position.y = pose[1]
        goal.cmd.position.z = pose[2]
        goal.cmd.rpy.roll = pose[3]
        goal.cmd.rpy.pitch = pose[4]
        goal.cmd.rpy.yaw = pose[5]
        self.start_execution_thread(goal)

    def move_joints(self, joints):
        goal = RobotMoveGoal()
        goal.cmd.cmd_type = MoveCommandType.JOINTS
        goal.cmd.joints = joints
        self.start_execution_thread(goal)

    def start_execution_thread(self, goal):
        if not self.execution_thread.is_alive():
            self.execution_thread = threading.Thread(target=self.execute_action,
                                                     args=['niryo_one/commander/robot_action', RobotMoveAction, goal])
            self.execution_thread.start()

    def execute_action(self, action_name, action_msg_type, goal):
        self.setValuesOffset(HR_IS_EXECUTING_CMD, [1])
        self.setValuesOffset(HR_LAST_ROBOT_CMD_RESULT, [0])
        self.cmd_action_client = actionlib.SimpleActionClient(action_name, action_msg_type)

        # Connect to server
        if not self.cmd_action_client.wait_for_server(rospy.Duration(0.5)):
            self.setValuesOffset(HR_IS_EXECUTING_CMD, [0])
            self.setValuesOffset(HR_LAST_ROBOT_CMD_RESULT, [7])
            return

        # Send goal and check response
        self.cmd_action_client.send_goal(goal)

        self.is_action_client_running = True
        if not self.cmd_action_client.wait_for_result(timeout=rospy.Duration(15.0)):
            self.cmd_action_client.cancel_goal()
            self.cmd_action_client.stop_tracking_goal()
            self.setValuesOffset(HR_IS_EXECUTING_CMD, [0])
            self.setValuesOffset(HR_LAST_ROBOT_CMD_RESULT, [6])
            self.is_action_client_running = False
            return

        self.is_action_client_running = False
        goal_state = self.cmd_action_client.get_state()
        response = self.cmd_action_client.get_result()

        if goal_state != GoalStatus.SUCCEEDED:
            self.cmd_action_client.stop_tracking_goal()

        self.setValuesOffset(HR_IS_EXECUTING_CMD, [0])

        if goal_state == GoalStatus.REJECTED:
            self.setValuesOffset(HR_LAST_ROBOT_CMD_RESULT, [2])
        elif goal_state == GoalStatus.ABORTED:
            self.setValuesOffset(HR_LAST_ROBOT_CMD_RESULT, [3])
        elif goal_state == GoalStatus.PREEMPTED:
            self.setValuesOffset(HR_LAST_ROBOT_CMD_RESULT, [4])
        elif goal_state != GoalStatus.SUCCEEDED:
            self.setValuesOffset(HR_LAST_ROBOT_CMD_RESULT, [5])
        else:
            self.setValuesOffset(HR_LAST_ROBOT_CMD_RESULT, [1])

    def set_conveyor(self, conveyor_id):
        if conveyor_id == 1: 
            self.call_ros_service('/niryo_one/kits/ping_and_set_conveyor',
                            SetConveyor, [CONVEYOR_ID_ONE, True])
        elif conveyor_id == 2: 
            self.call_ros_service('/niryo_one/kits/ping_and_set_conveyor',
                            SetConveyor, [CONVEYOR_ID_TWO, True]) 
        else: pass
    
    def detach_conveyor(self, conveyor_id):
        if conveyor_id == 1: 
            self.call_ros_service('/niryo_one/kits/ping_and_set_conveyor',
                            SetConveyor, [CONVEYOR_ID_ONE, False])
        elif conveyor_id == 2: 
            self.call_ros_service('/niryo_one/kits/ping_and_set_conveyor',
                            SetConveyor, [CONVEYOR_ID_TWO, False]) 
        else: pass

    def update_conveyor_id(self, new_id): 
        if new_id == 1:
            self.call_ros_service('/niryo_one/kits/update_conveyor_id',
                            UpdateConveyorId, [CONVEYOR_ID_TWO, CONVEYOR_ID_ONE])
        elif new_id == 2: 
            self.call_ros_service('/niryo_one/kits/update_conveyor_id',
                    UpdateConveyorId, [CONVEYOR_ID_ONE, CONVEYOR_ID_TWO])
        else: 
            pass 
    
    def control_conveyor(self, conveyor_id):
        conveyor_speed = self.getValuesOffset(HR_CONTROL_CONVEYOR_SPEED, 1)[0]
        if conveyor_speed > 100:
            conveyor_speed = 100
        elif conveyor_speed < 0:
            conveyor_speed = 0
        
        conveyor_direction = self.getValuesOffset(HR_CONTROL_CONVEYOR_DIRECTION, 1)[0]
        conveyor_direction = handle_negative_hr(conveyor_direction)
        
        if conveyor_id == 1: 
            conveyor_id = CONVEYOR_ID_ONE
        elif conveyor_id == 2: 
            conveyor_id = CONVEYOR_ID_TWO
        
        self.call_ros_service('/niryo_one/kits/control_conveyor',
                        ControlConveyor, [conveyor_id, True, int(conveyor_speed), conveyor_direction])

    def stop_conveyor(self, conveyor_id):   
        if conveyor_id == 1: 
            conveyor_id = CONVEYOR_ID_ONE
        elif conveyor_id == 2: 
            conveyor_id = CONVEYOR_ID_TWO
        self.call_ros_service('/niryo_one/kits/control_conveyor',
                        ControlConveyor, [conveyor_id, False, 0, 1])