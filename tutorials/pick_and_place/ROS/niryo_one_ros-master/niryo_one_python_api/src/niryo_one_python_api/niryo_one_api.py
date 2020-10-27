#!/usr/bin/env python

# niryo_one_api.py
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

from actionlib_msgs.msg import GoalStatus

from std_msgs.msg import Int32
from std_msgs.msg import String
from std_msgs.msg import Bool

from sensor_msgs.msg import JointState
from sensor_msgs.msg import CompressedImage

from niryo_one_msgs.msg import ConveyorFeedback
from niryo_one_msgs.msg import RobotMoveAction
from niryo_one_msgs.msg import RobotMoveGoal
from niryo_one_msgs.msg import HardwareStatus
from niryo_one_msgs.msg import RobotState
from niryo_one_msgs.msg import DigitalIOState

from niryo_one_msgs.srv import SetInt
from niryo_one_msgs.srv import GetDigitalIO
from niryo_one_msgs.srv import SetDigitalIO
from niryo_one_msgs.srv import GetPositionList
from niryo_one_msgs.srv import EditWorkspace
from niryo_one_msgs.srv import GetTargetPose, GetTargetPoseRequest
from niryo_one_msgs.srv import ObjDetection, ObjDetectionResponse
from niryo_one_msgs.srv import GetWorkspaceList
from niryo_one_msgs.srv import GetWorkspaceRatio
from niryo_one_msgs.srv import GetCalibrationCam

from niryo_one_msgs.srv import ControlConveyor, SetConveyor, UpdateConveyorId

from niryo_one_commander.command_type import CommandType as MoveCommandType

# Tools IDs (need to match tools ids in niryo_one_tools package)
TOOL_NONE = 0
TOOL_GRIPPER_1_ID = 11
TOOL_GRIPPER_2_ID = 12
TOOL_GRIPPER_3_ID = 13
TOOL_ELECTROMAGNET_1_ID = 30
TOOL_VACUUM_PUMP_1_ID = 31

# GPIOs
PIN_MODE_OUTPUT = 0
PIN_MODE_INPUT = 1
PIN_HIGH = 1
PIN_LOW = 0

GPIO_1A = 2
GPIO_1B = 3
GPIO_1C = 16
GPIO_2A = 26
GPIO_2B = 19
GPIO_2C = 6

SW_1 = 12
SW_2 = 13

# Status
OK = 200
ERROR = 400

# for shift_pose function
AXIS_X = 0
AXIS_Y = 1
AXIS_Z = 2
ROT_ROLL = 3
ROT_PITCH = 4
ROT_YAW = 5

# for vision functions
COLOR_RED = "RED"
COLOR_GREEN = "GREEN"
COLOR_BLUE = "BLUE"
COLOR_ANY = "ANY"

SHAPE_CIRCLE = "CIRCLE"
SHAPE_SQUARE = "SQUARE"
SHAPE_ANY = "ANY"

# Conveyor
CONVEYOR_DIRECTION_BACKWARD = -1
CONVEYOR_DIRECTION_FORWARD = 1

CONVEYOR_ID_ONE = 6
CONVEYOR_ID_TWO = 7


class NiryoOneException(Exception):
    pass


class NiryoOne:
    # Singleton
    instance = None

    def __init__(self):
        if not NiryoOne.instance:
            NiryoOne.instance = NiryoOne.__NiryoOne()

    def __getattr__(self, name):
        return getattr(self.instance, name)

    class __NiryoOne:

        def __init__(self):
            self.service_timeout = rospy.get_param("/niryo_one/python_api/service_timeout")
            self.action_connection_timeout = rospy.get_param("/niryo_one/python_api/action_connection_timeout")
            self.action_execute_timeout = rospy.get_param("/niryo_one/python_api/action_execute_timeout")
            self.action_preempt_timeout = rospy.get_param("/niryo_one/python_api/action_preempt_timeout")
            self.tool_command_list = rospy.get_param("/niryo_one_tools/command_list")

            # Subscribers
            self.joint_states_sub = rospy.Subscriber('/joint_states', JointState, self.sub_joint_states)
            self.robot_state_sub = rospy.Subscriber('/niryo_one/robot_state', RobotState, self.sub_robot_state)
            self.hardware_status_sub = rospy.Subscriber('/niryo_one/hardware_status', HardwareStatus,
                                                        self.sub_hardware_status)
            self.learning_mode_sub = rospy.Subscriber('/niryo_one/learning_mode', Bool, self.sub_learning_mode)
            self.digital_io_state_sub = rospy.Subscriber('/niryo_one/rpi/digital_io_state', DigitalIOState,
                                                         self.sub_digital_io_state)
            self.tool_id_sub = rospy.Subscriber('/niryo_one/current_tool_id', Int32,
                                                self.sub_current_tool_id)
            self.conveyor_1_feedback_sub = rospy.Subscriber('/niryo_one/kits/conveyor_1_feedback', ConveyorFeedback,
                                                            self.sub_conveyor_1_feedback)
            self.conveyor_2_feedback_sub = rospy.Subscriber('/niryo_one/kits/conveyor_2_feedback', ConveyorFeedback,
                                                            self.sub_conveyor_2_feedback)

            # Highlight publisher (to highlight blocks in Blockly interface)
            self.highlight_block_publisher = rospy.Publisher('/niryo_one/blockly/highlight_block', String,
                                                             queue_size=10)

            # Break point publisher (for break point blocks in Blockly interface)
            self.break_point_publisher = rospy.Publisher('/niryo_one/blockly/break_point', Int32, queue_size=10)

            self.video_stream_subscriber = rospy.Subscriber('/niryo_one_vision/compressed_video_stream',
                                                            CompressedImage, self.sub_stream_video,
                                                            queue_size=1)

            self.joints = None
            self.pose = None
            self.hw_status = None
            self.learning_mode_on = None
            self.digital_io_state = None
            self.current_tool_id = None
            self.compressed_image_message = None

            self.conveyor_1_feedback = None
            self.conveyor_2_feedback = None

            rospy.sleep(0.1)

        #
        # Subscribers callbacks
        #

        def sub_joint_states(self, joint_states):
            self.joints = list(joint_states.position)

        def sub_hardware_status(self, hw_status):
            self.hw_status = hw_status

        def sub_robot_state(self, pose):
            self.pose = pose

        def sub_learning_mode(self, learning_mode):
            self.learning_mode_on = learning_mode.data

        def sub_digital_io_state(self, digital_io_state):
            self.digital_io_state = digital_io_state

        def sub_stream_video(self, compressed_image_message):
            self.compressed_image_message = compressed_image_message

        def sub_current_tool_id(self, msg):
            self.current_tool_id = msg.data

        def sub_conveyor_1_feedback(self, conveyor_feedback):
            self.conveyor_1_feedback = conveyor_feedback

        def sub_conveyor_2_feedback(self, conveyor_feedback):
            self.conveyor_2_feedback = conveyor_feedback

        #
        # Service client
        #

        def call_service(self, service_name, service_msg_type, args):
            """
            Wait for the service called service_name
            Then call the service with args

            :param service_name:
            :param service_msg_type:
            :param args: Tuple of arguments
            :raises NiryoOneException: Timeout during waiting of services
            :return: Response
            """

            # Connect to service
            try:
                rospy.wait_for_service(service_name, self.service_timeout)
            except rospy.ROSException, e:
                raise NiryoOneException(e)

            # Call service
            try:
                service = rospy.ServiceProxy(service_name, service_msg_type)
                response = service(*args)
                return response
            except rospy.ServiceException, e:
                raise NiryoOneException(e)

        #
        # Action client
        #

        def execute_action(self, action_name, action_msg_type, goal):

            client = actionlib.SimpleActionClient(action_name, action_msg_type)

            # Connect to server
            if not client.wait_for_server(rospy.Duration(self.action_connection_timeout)):
                raise NiryoOneException('Action Server is not up : ' + str(action_name))

            # Send goal and check response
            client.send_goal(goal)

            if not client.wait_for_result(timeout=rospy.Duration(self.action_execute_timeout)):
                client.cancel_goal()
                client.stop_tracking_goal()
                raise NiryoOneException('Action Server timeout : ' + str(action_name))

            goal_state = client.get_state()
            response = client.get_result()

            if goal_state != GoalStatus.SUCCEEDED:
                client.stop_tracking_goal()

            if goal_state == GoalStatus.REJECTED:
                raise NiryoOneException('Goal has been rejected : ' + str(response.message))
            elif goal_state == GoalStatus.ABORTED:
                raise NiryoOneException('Goal has been aborted : ' + str(response.message))
            elif goal_state != GoalStatus.SUCCEEDED:
                raise NiryoOneException('Error when processing goal : ' + str(response.message))

            return response.message

        #
        # Interface
        #
        def __calibrate(self, calib_type_int):
            """
            Call service to calibrate motors then wait for its end. If failed, raise NiryoOneException

            :param calib_type_int: 1 for auto-calibration & 2 for manual calibration
            :return: status, message
            :rtype: (GoalStatus, str)
            """
            result = self.call_service('niryo_one/calibrate_motors',
                                       SetInt, [calib_type_int])
            if result.status != OK:
                raise NiryoOneException(result.message)
            # Wait until calibration is finished
            rospy.sleep(1)
            calibration_finished = False
            while not calibration_finished:
                try:
                    hw_status = rospy.wait_for_message('niryo_one/hardware_status',
                                                       HardwareStatus, timeout=5)
                    if not hw_status.calibration_in_progress:
                        calibration_finished = True
                except rospy.ROSException as e:
                    raise NiryoOneException(str(e))

        # Calibration

        def calibrate_auto(self):
            """
            Call service to calibrate motors then wait for its end. If failed, raise NiryoOneException

            :return: status, message
            :rtype: (GoalStatus, str)
            """
            self.__calibrate(calib_type_int=1)

        def calibrate_manual(self):
            """
            Call service to calibrate motors then wait for its end. If failed, raise NiryoOneException

            :return: status, message
            :rtype: (GoalStatus, str)
            """
            self.__calibrate(calib_type_int=2)

        # Learning mode

        def activate_learning_mode(self, activate):
            """
            Call service to activate_learning_mode according to set_bool. If failed, raise NiryoOneException

            :param set_bool: True to activate, False to deactivate
            :return: status, message
            :rtype: (GoalStatus, str)
            """
            result = self.call_service('niryo_one/activate_learning_mode',
                                       SetInt, [activate])
            if result.status != OK:
                raise NiryoOneException(result.message)
            rospy.sleep(0.1)

        def get_learning_mode(self):
            """
            Use /niryo_one/learning_mode/state topic subscriber to get learning mode status

            :return: True if activate else False
            """
            timeout = rospy.get_time() + 2.0
            while self.learning_mode_on is None:
                rospy.sleep(0.05)
                if rospy.get_time() > timeout:
                    raise NiryoOneException('Timeout: could not get learning mode (/niryo_one/learning_mode topic)')
            return self.learning_mode_on

        # Move

        def move_joints(self, joints):
            """
            Execute Move joints action

            :param joints: list of joint
            :return: status, message
            :rtype: (GoalStatus, str)
            """
            goal = RobotMoveGoal()
            goal.cmd.cmd_type = MoveCommandType.JOINTS
            goal.cmd.joints = joints
            return self.execute_action('niryo_one/commander/robot_action', RobotMoveAction, goal)

        def move_pose(self, x, y, z, roll, pitch, yaw):
            goal = RobotMoveGoal()
            goal.cmd.cmd_type = MoveCommandType.POSE
            goal.cmd.position.x = x
            goal.cmd.position.y = y
            goal.cmd.position.z = z
            goal.cmd.rpy.roll = roll
            goal.cmd.rpy.pitch = pitch
            goal.cmd.rpy.yaw = yaw
            return self.execute_action('niryo_one/commander/robot_action', RobotMoveAction, goal)

        def shift_pose(self, axis, value):
            """
            Execute Shift pose action

            :param axis: Value of RobotAxis enum corresponding to where the shift happens
            :type axis: ShiftPose
            :param value: shift value
            :type value: float
            :return: status, message
            :rtype: (GoalStatus, str)
            """
            goal = RobotMoveGoal()
            goal.cmd.cmd_type = MoveCommandType.SHIFT_POSE
            goal.cmd.shift.axis_number = axis
            goal.cmd.shift.value = value
            return self.execute_action('niryo_one/commander/robot_action', RobotMoveAction, goal)

        def set_arm_max_velocity(self, percentage):
            """
            Set relative max velocity (in %)

            :type percentage: int
            :return: status, message
            :rtype: (GoalStatus, str)
            """
            result = self.call_service('/niryo_one/commander/set_max_velocity_scaling_factor',
                                       SetInt, [percentage])
            if result.status != OK:
                raise NiryoOneException(result.message)

        def enable_joystick(self, enable):
            """
            Turn joystick On or Off

            :type enable: bool
            :return: status, message
            :rtype: (GoalStatus, str)
            """
            result = self.call_service('/niryo_one/joystick_interface/enable',
                                       SetInt, [enable])
            if result.status != OK:
                raise NiryoOneException(result.message)

        # Get Robot positions

        def get_saved_position_list(self):
            """
            Ask the pose manager service which positions are available

            :return: list of positions name
            :rtype: list[str]
            """
            result = self.call_service('niryo_one/position/get_position_list',
                                       GetPositionList, [])
            return result.positions

        def get_joints(self):
            """
            Use /joint_states topic to get joints status

            :return: list of joint value
            """
            timeout = rospy.get_time() + 2.0
            while self.joints is None:
                rospy.sleep(0.05)
                if rospy.get_time() > timeout:
                    raise NiryoOneException('Timeout: could not get joints (/joint_states topic)')
            return self.joints

        def get_arm_pose(self):
            """
            Use /niryo_one/robot_state topic to get pose status

            :return: list corresponding to [x, y, z, roll, pitch, yaw]
            """
            timeout = rospy.get_time() + 2.0
            while self.pose is None:
                rospy.sleep(0.05)
                if rospy.get_time() > timeout:
                    raise NiryoOneException('Timeout: could not get pose (/niryo_one/robot_state topic)')
            return self.pose

        # I/O

        def pin_mode(self, pin, mode):
            """
            Set pin number pin_id to mode pin_mode

            :type pin_id: PinID
            :type pin_mode: PinMode
            :return: status, message
            :rtype: (GoalStatus, str)
            """
            result = self.call_service('niryo_one/rpi/set_digital_io_mode',
                                       SetDigitalIO, [pin, mode])
            if result.status != OK:
                raise NiryoOneException(result.message)

        def digital_write(self, pin, state):
            """
            Set pin_id state to pin_state

            :type pin_id: PinID
            :type pin_state: PinState
            :return: status, message
            :rtype: (GoalStatus, str)
            """
            result = self.call_service('niryo_one/rpi/set_digital_io_state',
                                       SetDigitalIO, [pin, state])
            if result.status != OK:
                raise NiryoOneException(result.message)

        def digital_read(self, pin):
            """
            Read pin number pin_id and return its state

            :type pin_id: PinID
            :return: state
            :rtype: PinState
            """
            result = self.call_service('niryo_one/rpi/get_digital_io',
                                       GetDigitalIO, [pin])
            if result.status != OK:
                raise NiryoOneException(result.message)
            return result.state

        def get_digital_io_state(self):
            """
            Get Digital IO state : Names, modes, states

            :return: Infos contains in a DigitalIOState object (see niryo_one_msgs)
            :rtype: DigitalIOState
            """
            timeout = rospy.get_time() + 2.0
            while self.digital_io_state is None:
                rospy.sleep(0.05)
                if rospy.get_time() > timeout:
                    raise NiryoOneException(
                        'Timeout: could not get digital io state (/niryo_one/rpi/digital_io_state topic)')
            return self.digital_io_state

        # End effectors

        def change_tool(self, tool_id):
            """
            Call service niryo_one/change_tool to change tool

            :type tool_id: ToolID
            :return: status, message
            :rtype: (GoalStatus, str)
            """
            result = self.call_service('niryo_one/change_tool',
                                       SetInt, [int(tool_id)])
            if result.status != OK:
                raise NiryoOneException(result.message)
            self.current_tool_id = tool_id

        def get_current_tool_id(self):
            """
            Use /niryo_one/tools/current_tool_id  topic to get current tool id

            :return: Tool Id
            :rtype: ToolID
            """
            timeout = rospy.get_time() + 2.0
            while self.current_tool_id is None:
                rospy.sleep(0.05)
                if rospy.get_time() > timeout:
                    raise NiryoOneException(
                        'Timeout: could not get current tool id (/niryo_one/current_tool_id topic)')
            return self.current_tool_id

        def __deal_with_gripper(self, gripper_id, speed, command_str):
            goal = RobotMoveGoal()
            goal.cmd.cmd_type = MoveCommandType.TOOL
            goal.cmd.tool_cmd.tool_id = int(gripper_id)
            goal.cmd.tool_cmd.cmd_type = self.tool_command_list[command_str]
            if "open" in command_str:
                goal.cmd.tool_cmd.gripper_open_speed = speed
            else:
                goal.cmd.tool_cmd.gripper_close_speed = speed
            return self.execute_action('niryo_one/commander/robot_action', RobotMoveAction, goal)

        def open_gripper(self, gripper_id, speed):
            """
            Open gripper associated to 'gripper_id' with a speed 'speed'

            :param gripper_id: Gripper ID
            :type gripper_id: ToolID
            :param speed: Default -> 500
            :type speed: int
            :return: status, message
            :rtype: (GoalStatus, str)
            """
            return self.__deal_with_gripper(gripper_id, speed, "open_gripper")

        def close_gripper(self, gripper_id, speed):
            """
            Close gripper associated to 'gripper_id' with a speed 'speed'

            :param gripper_id: Gripper ID
            :type gripper_id: ToolID
            :param speed: Default -> 500
            :type speed: int
            :return: status, message
            :rtype: (GoalStatus, str)
            """
            return self.__deal_with_gripper(gripper_id, speed, "close_gripper")

        def __deal_with_vacuum_pump(self, vacuum_pump_id, command_str):
            goal = RobotMoveGoal()
            goal.cmd.cmd_type = MoveCommandType.TOOL
            goal.cmd.tool_cmd.tool_id = int(vacuum_pump_id)
            goal.cmd.tool_cmd.cmd_type = self.tool_command_list[command_str]
            return self.execute_action('niryo_one/commander/robot_action', RobotMoveAction, goal)

        def pull_air_vacuum_pump(self, vacuum_pump_id):
            """
            Pull air of vacuum associated to 'vacuum_pump_id'

            :param vacuum_pump_id: Vacuum ID
            :type vacuum_pump_id: ToolID
            :return: status, message
            :rtype: (GoalStatus, str)
            """
            return self.__deal_with_vacuum_pump(vacuum_pump_id, "pull_air_vacuum_pump")

        def push_air_vacuum_pump(self, vacuum_pump_id):
            """
            Pull air of vacuum associated to 'vacuum_pump_id'

            :param vacuum_pump_id: Vacuum ID
            :type vacuum_pump_id: ToolID
            :return: status, message
            :rtype: (GoalStatus, str)
            """
            return self.__deal_with_vacuum_pump(vacuum_pump_id, "push_air_vacuum_pump")

        def __deal_with_electromagnet(self, electromagnet_id, pin, command_str):
            goal = RobotMoveGoal()
            goal.cmd.cmd_type = MoveCommandType.TOOL
            goal.cmd.tool_cmd.tool_id = int(electromagnet_id)
            goal.cmd.tool_cmd.cmd_type = self.tool_command_list[command_str]
            goal.cmd.tool_cmd.gpio = pin
            return self.execute_action('niryo_one/commander/robot_action', RobotMoveAction, goal)

        def setup_electromagnet(self, electromagnet_id, pin):
            """
            Setup electromagnet on pin

            :param electromagnet_id: Tool ID
            :type electromagnet_id: ToolID
            :param pin_id: Pin ID
            :type pin_id:  PinID
            :return: status, message
            :rtype: (GoalStatus, str)
            """
            return self.__deal_with_electromagnet(electromagnet_id, pin, 'setup_digital_io')

        def activate_electromagnet(self, electromagnet_id, pin):
            """
            Activate electromagnet associated to electromagnet_id on pin_id

            :param electromagnet_id: Tool ID
            :type electromagnet_id: ToolID
            :param pin_id: Pin ID
            :type pin_id:  PinID
            :return: status, message
            :rtype: (GoalStatus, str)
            """
            return self.__deal_with_electromagnet(electromagnet_id, pin, 'activate_digital_io')

        def deactivate_electromagnet(self, electromagnet_id, pin):
            """
            Deactivate electromagnet associated to electromagnet_id on pin_id

            :param electromagnet_id: Tool ID
            :type electromagnet_id: ToolID
            :param pin_id: Pin ID
            :type pin_id:  PinID
            :return: status, message
            :rtype: (GoalStatus, str)
            """
            return self.__deal_with_electromagnet(electromagnet_id, pin, 'deactivate_digital_io')

        def grab_with_tool(self, tool_id):
            """
            Grasp with the tool linked to tool_id. If no tool_id is given, will get current tool id
            This action correspond to
            - Close gripper for Grippers
            - Pull Air for Vacuum pump
            - Activate for Electromagnet

            :type tool_id: ToolID
            :return: status, message
            :rtype: (GoalStatus, str)
            """
            if tool_id in (TOOL_GRIPPER_1_ID, TOOL_GRIPPER_2_ID, TOOL_GRIPPER_3_ID):
                self.close_gripper(tool_id, 450)
            elif tool_id == TOOL_VACUUM_PUMP_1_ID:
                self.pull_air_vacuum_pump(tool_id)
            elif tool_id == TOOL_ELECTROMAGNET_1_ID:
                self.activate_electromagnet(tool_id, GPIO_1A)

        def release_tool(self, tool_id):
            """
            Release with the tool associated to tool_id. If no tool_id is given, will get current tool id
            This action correspond to
            - Open gripper for Grippers
            - Push Air for Vacuum pump
            - Deactivate for Electromagnet

            :param tool_id: Tool ID
            :type tool_id: ToolID
            :param pin_id: [Only required for electromagnet] Pin ID of the electromagnet
            :type: PinID
            :return: status, message
            :rtype: (GoalStatus, str)
            """
            if tool_id in (TOOL_GRIPPER_1_ID, TOOL_GRIPPER_2_ID, TOOL_GRIPPER_3_ID):
                self.open_gripper(tool_id, 450)
            elif tool_id == TOOL_VACUUM_PUMP_1_ID:
                self.push_air_vacuum_pump(tool_id)
            elif tool_id == TOOL_ELECTROMAGNET_1_ID:
                self.deactivate_electromagnet(tool_id, GPIO_1A)

        # Others

        def get_hardware_status(self):
            """
            Get hardware status : Temperature, Hardware version, motors names & types ...

            :return: Infos contains in a HardwareStatus object (see niryo_one_msgs)
            :rtype: HardwareStatus
            """
            timeout = rospy.get_time() + 2.0
            while self.hw_status is None:
                rospy.sleep(0.05)
                if rospy.get_time() > timeout:
                    raise NiryoOneException('Timeout: could not get hardware status (/niryo_one/hardware_status topic)')
            return self.hw_status

        @staticmethod
        def wait(time_sleep):
            rospy.sleep(time_sleep)

        # Workspace

        def create_workspace(self, name, pose_origin, pose_1, pose_2, pose_3):
            """
            Create workspace by giving the poses of the robot to point its 4 corners
            with the calibration Tip. Corners should be in the good order

            :param name: workspace name
            :type name: str
            :param list_poses_raw: list of 4 corners pose
            :type list_poses_raw: list[list]
            :return: status, message
            :rtype: (GoalStatus, str)
            """
            result = self.call_service('niryo_one/pose_converter/edit_workspace',
                                       EditWorkspace, [1, name, pose_origin, pose_1, pose_2, pose_3])
            if result.status != OK:
                raise NiryoOneException(result.message)

        def remove_workspace(self, name):
            """
            Call worksspace manager to remove a certain workspace

            :param name: workspace name
            :type name: str
            :return: status, message
            :rtype: (GoalStatus, str)
            """
            r = RobotState()  # dummy state needed to call the service
            result = self.call_service('niryo_one/pose_converter/edit_workspace',
                                       EditWorkspace, [-1, name, r, r, r, r])
            if result.status != OK:
                raise NiryoOneException(result.message)

        def get_workspace_ratio(self, name):
            """
            Give the length over width ratio of a certain workspace

            :param name: workspace name
            :type name: str
            :return: ratio
            :rtype: float
            """
            result = self.call_service('niryo_one/pose_converter/get_workspace_ratio',
                                       GetWorkspaceRatio, [name])
            if result.status != OK:
                raise NiryoOneException(result.message)
            return result.ratio

        def get_workspace_list(self):
            """
            Ask the workspace manager service names of the available workspace

            :return: list of workspaces name
            :rtype: list[str]
            """
            result = self.call_service('niryo_one/pose_converter/get_workspace_list',
                                       GetWorkspaceList, [])
            return result.workspaces

        # Compute relative pose

        def get_target_pose_from_rel(self, workspace, height_offset, x_rel, y_rel, yaw_rel):
            """
            Given a pose (x_rel, y_rel, yaw_rel_) relative to a workspace, this function
            returns the robot pose in which the current tool will be able to pick an object at this pose.
            The height_offset argument (in m) defines how high the tool will hover over the workspace. If height_offset = 0,
            the tool will nearly touch the workspace.

            :param workspace: name of the workspace
            :type workspace: str
            :param height_offset: offset between the workspace and the target height
            :type height_offset: float
            :param x_rel:
            :type x_rel: float
            :param y_rel:
            :type y_rel: float
            :param yaw_rel:
            :type yaw_rel:
            :return: target_pose
            :rtype RobotState
            """
            result = self.call_service('niryo_one/pose_converter/get_target_pose',
                                       GetTargetPose, [workspace, GetTargetPoseRequest.GRIP_AUTO,
                                                       self.get_current_tool_id(), height_offset, x_rel, y_rel,
                                                       yaw_rel])

            if result.status != OK:
                raise NiryoOneException(result.message)
            else:
                return result.target_pose

        def get_target_pose_from_cam(self, workspace, height_offset, shape, color):
            """
            First detects the specified object using the camera and then returns the robot pose in which the object can
            be picked with the current tool

            :param workspace: name of the workspace
            :type workspace: str
            :param height_offset: offset between the workspace and the target height
            :type height_offset: float
            :param shape: shape of the target
            :type shape: ObjectShape
            :param color: color of the target
            :type color: ObjectColor
            :return: object_found, object_pose, object_shape, object_color
            :rtype: (bool, RobotState, str, str)
            """
            object_found, rel_pose, obj_shape, obj_color = self.detect_object(workspace, shape, color)
            if not object_found:
                return False, None, "", ""
            obj_pose = self.get_target_pose_from_rel(
                workspace, height_offset, rel_pose.x, rel_pose.y, rel_pose.yaw)
            return True, obj_pose, obj_shape, obj_color

        # Vision Pick and place functions

        def vision_pick(self, workspace, height_offset, shape, color):
            """
            Picks the specified object from the workspace. This function has multiple phases:
            1. detect object using the camera
            2. prepare the current tool for picking
            3. approach the object
            4. move down to the correct picking pose
            5. actuate the current tool
            6. lift the object

            :param workspace: name of the workspace
            :type workspace: str
            :param height_offset: offset between the workspace and the target height
            :type height_offset: float
            :param shape: shape of the target
            :type shape: ObjectShape
            :param color: color of the target
            :type color: ObjectColor
            :return: object_found, object_shape, object_color
            :rtype: (bool, ObjectShape, ObjectColor)
            """
            object_found, rel_pose, obj_shape, obj_color = self.detect_object(workspace, shape, color)
            if not object_found:
                return False, "", ""

            pick_pose = self.get_target_pose_from_rel(
                workspace, height_offset, rel_pose.x, rel_pose.y, rel_pose.yaw)
            approach_pose = self.get_target_pose_from_rel(
                workspace, height_offset + 0.05, rel_pose.x, rel_pose.y, rel_pose.yaw)

            tool_id = self.get_current_tool_id()
            self.release_tool(tool_id)

            self.move_pose(*self.robot_state_msg_to_list(approach_pose))
            self.move_pose(*self.robot_state_msg_to_list(pick_pose))

            self.grab_with_tool(tool_id)

            self.move_pose(*self.robot_state_msg_to_list(approach_pose))
            return True, obj_shape, obj_color

        def move_to_object(self, workspace, height_offset, shape, color):
            """
            Same as `get_target_pose_from_cam` but directly moves to this position

            :param workspace: name of the workspace
            :type workspace: str
            :param height_offset: offset between the workspace and the target height
            :type height_offset: float
            :param shape: shape of the target
            :type shape: ObjectShape
            :param color: color of the target
            :type color: ObjectColor
            :return: object_found, object_shape, object_color
            :rtype: (bool, ObjectShape, ObjectColor)
            """
            obj_found, obj_pose, obj_shape, obj_color = self.get_target_pose_from_cam(
                workspace, height_offset, shape, color)
            if not obj_found:
                return False, "", ""
            self.move_pose(*self.robot_state_msg_to_list(obj_pose))
            return True, obj_shape, obj_color

        def pick_from_pose(self, x, y, z, roll, pitch, yaw):
            """
            Execute a picking from a position. If an error happens during the movement, error will be raised.
            A picking is described as :
            - going over the object
            - going down until height = z
            - grasping with tool
            - going back over the object
            """
            tool_id = self.get_current_tool_id()

            self.release_tool(tool_id)

            self.move_pose(x, y, z + 0.05, roll, pitch, yaw)
            self.move_pose(x, y, z, roll, pitch, yaw)

            self.grab_with_tool(tool_id)

            self.move_pose(x, y, z + 0.05, roll, pitch, yaw)

        def place_from_pose(self, x, y, z, roll, pitch, yaw):
            """
            Execute a placing from a position. If an error happens during the movement, error will be raised.
            A placing is described as :
            - going over the place
            - going down until height = z
            - releasing the object with tool
            - going back over the place
            """
            tool_id = self.get_current_tool_id()
            self.move_pose(x, y, z + 0.05, roll, pitch, yaw)
            self.move_pose(x, y, z, roll, pitch, yaw)
            self.release_tool(tool_id)

            self.move_pose(x, y, z + 0.05, roll, pitch, yaw)

        def detect_object(self, workspace, shape, color):
            """

            :param workspace: name of the workspace
            :type workspace: str
            :param shape: shape of the target
            :type shape: ObjectShape
            :param color: color of the target
            :type color: ObjectColor
            :return: object_found, object_pose, object_shape, object_color
            :rtype: (bool, RobotState, str, str)
            """
            ratio = self.get_workspace_ratio(workspace)
            response = self.call_service("/niryo_one_vision/obj_detection_rel", ObjDetection,
                                         [shape, color, ratio, False])
            if response.status == ObjDetectionResponse.SUCCESSFUL:
                return True, response.obj_pose, response.obj_type, response.obj_color
            elif response.status == ObjDetectionResponse.MARKERS_NOT_FOUND:
                rospy.logwarn_throttle(1, 'Markers Not Found')
            elif response.status == ObjDetectionResponse.VIDEO_STREAM_NOT_RUNNING:
                rospy.logwarn_throttle(1, 'Video Stream not running')
            return False, None, "", ""

        # Camera image

        def get_compressed_image(self):
            """
            Get last stream image in a compressed format

            :return: string containing a JPEG compressed image
            :rtype: str
            """
            timeout = rospy.get_time() + 2.0
            while self.compressed_image_message is None:
                rospy.sleep(0.05)
                if rospy.get_time() > timeout:
                    raise NiryoOneException(
                        'Timeout: could not video stream message (/niryo_one_vision/compressed_video_stream topic)')
            return self.compressed_image_message.data

        def get_calibration_object(self):
            """
            Get calibration object: camera intrinsics, distortions coefficients

            :return: IsSet, raw camera intrinsics, distortions coefficients, camera intrinsics after correction
            :rtype: (bool, list,list)
            """
            result = self.call_service('/niryo_one_vision/get_calibration_camera',
                                       GetCalibrationCam, [])
            return result.is_set, result.camera_info.K, result.camera_info.D

        # Conveyor

        def set_conveyor(self, conveyor_id, activate):
            """
            Activate or deactivate the connection to the conveyor associated to conveyor_id

            :param conveyor_id: Basically, 6 or 7
            :type conveyor_id: int
            :param activate: True for activate, False for deactivate
            :type activate: bool
            :raises NiryoOneException
            :return: status, message
            :rtype: (GoalStatus, str)
            """
            result = self.call_service('/niryo_one/kits/ping_and_set_conveyor',
                                       SetConveyor, [conveyor_id, activate])
            if result.status != OK:
                raise NiryoOneException(result.message)

            return result.status, result.message

        def control_conveyor(self, conveyor_id, control_on, speed, direction):
            """
            Control conveyor associated to conveyor_id.
            Then stops it if bool_control_on is False, else refreshes it speed and direction

            :param conveyor_id: Basically, ConveyorID.ONE or ConveyorID.TWO
            :type conveyor_id: ConveyorID
            :param control_on: True for activate, False for deactivate
            :type control_on: bool
            :param speed: target speed
            :type speed: int
            :param direction: Target direction
            :type direction: ConveyorDirection
            :return: status, message
            :rtype: (GoalStatus, str)
            """
            result = self.call_service('/niryo_one/kits/control_conveyor',
                                       ControlConveyor, [conveyor_id, control_on, speed, direction])
            if result.status != OK:
                raise NiryoOneException(result.message)
            return result.status, result.message

        def update_conveyor_id(self, old_id, new_id):
            """
            Change conveyor id to another one

            :param old_id:
            :type old_id: ConveyorID
            :param new_id:
            :type new_id: ConveyorID
            :return: status, message
            :rtype: (GoalStatus, str)
            """
            result = self.call_service('/niryo_one/kits/update_conveyor_id',
                                       UpdateConveyorId, [old_id, new_id])
            if result.status != OK:
                raise NiryoOneException(result.message)
            return result.status, result.message

        def get_conveyor_1_feedback(self):
            """
            Give conveyor 1 feedback

            :return: ID, connection_state, running, speed, direction
            :rtype: (int, bool, bool, int, int)
            """
            timeout = rospy.get_time() + 2.0
            while self.conveyor_1_feedback is None:
                rospy.sleep(0.05)
                if rospy.get_time() > timeout:
                    raise NiryoOneException(
                        'Timeout: could not get conveyor 1 feedback (/niryo_one/kits/conveyor_1_feedback topic)')
            fb = self.conveyor_1_feedback
            return fb.conveyor_id, fb.connection_state, fb.running, fb.speed, fb.direction

        def get_conveyor_2_feedback(self):
            """
            Give conveyor 2 feedback

            :return: ID, connection_state, running, speed, direction
            :rtype: (int, bool, bool, int, int)
            """
            timeout = rospy.get_time() + 2.0
            while self.conveyor_2_feedback is None:
                rospy.sleep(0.05)
                if rospy.get_time() > timeout:
                    raise NiryoOneException(
                        'Timeout: could not get conveyor 2 feedback (/niryo_one/kits/conveyor_2_feedback topic)')
            fb = self.conveyor_2_feedback
            return fb.conveyor_id, fb.connection_state, fb.running, fb.speed, fb.direction

        # Will highlight a block on a Blockly interface
        # This is just graphical, no real functionality here
        def highlight_block(self, block_id):
            # rospy.logwarn("Highlight block : " + str(block_id))
            msg = String()
            msg.data = block_id
            self.highlight_block_publisher.publish(msg)

        # Will stop the execution of the current sequence
        # Need to press on "play" button on Niryo One Studio
        # to resume execution
        def break_point(self):
            msg = Int32()
            msg.data = 1
            self.break_point_publisher.publish(msg)
            # This delay makes sure the program has time
            # to stop, before executing next command
            rospy.sleep(0.5)

        @staticmethod
        def list_to_robot_state_msg(list_pos):
            """
            Translate (x, y, z, roll, pitch, yaw) to a RobotState Object
            """
            r = RobotState()
            r.position.x = list_pos[0]
            r.position.y = list_pos[1]
            r.position.z = list_pos[2]
            r.rpy.roll = list_pos[3]
            r.rpy.pitch = list_pos[4]
            r.rpy.yaw = list_pos[5]
            return r

        @staticmethod
        def robot_state_msg_to_list(robot_state):
            """
            Translate a RobotState Object to (x, y, z, roll, pitch, yaw)
            """
            return [robot_state.position.x, robot_state.position.y, robot_state.position.z,
                    robot_state.rpy.roll, robot_state.rpy.pitch, robot_state.rpy.yaw]
