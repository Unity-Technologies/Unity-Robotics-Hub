#!/usr/bin/env python

# tcp_client.py
# Copyright (C) 2019 Niryo
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

import socket
import sys
import numpy as np
from .pose_object import PoseObject
from .hardware_status_object import HardwareStatusObject
from .packet_builder import PacketBuilder
from .enums import Command, RobotTool, Shape, Color, ConveyorDirection
from .digital_pin_object import DigitalPinObject
import re
import ast
import time


class NiryoOneClient:
    class HostNotReachableException(Exception):
        def __init__(self):
            super(Exception, self).__init__("Unable to communicate with robot server, please verify your network.")

    class ClientNotConnectedException(Exception):
        def __init__(self):
            super(Exception, self).__init__("You're not connected to  the robot.")

    class InvalidAnswerException(Exception):
        def __init__(self, answer):
            super(Exception, self).__init__(
                "An invalid answer has been received. Format expected: COMMAND:[OK[, data_answer]] / [KO, reason].\n"
                + "A problem occurred with: '" + answer + "'")

    def __init__(self, timeout=5):
        self.__port = 40001
        self.__is_running = True
        self.__is_connected = False
        self.__timeout = timeout
        self.__client_socket = None
        self.__packet_builder = PacketBuilder()

    def __del__(self):
        self.quit()

    def quit(self):
        self.__is_running = False
        self.__shutdown_connection()
        self.__client_socket = None

    def __shutdown_connection(self):
        if self.__client_socket is not None and self.__is_connected is True:
            try:
                self.__client_socket.shutdown(socket.SHUT_RDWR)
                self.__client_socket.close()
            except socket.error as _:
                pass
            self.__is_connected = False

    def connect(self, ip_address):
        self.__client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__client_socket.settimeout(self.__timeout)
        try:
            self.__client_socket.connect((ip_address, self.__port))
        except socket.timeout:
            print("Unable to connect to the robot.")
            self.__shutdown_connection()
            self.__client_socket = None
        except socket.error as e:
            print("An error occurred while attempting to connect: {}".format(e))
            self.__shutdown_connection()
            self.__client_socket = None
        else:
            print("Connected to server ({}) on port: {}".format(ip_address, self.__port))
            self.__is_connected = True
            self.__client_socket.settimeout(None)

        return self.__is_connected

    def calibrate(self, calibrate_mode):
        self.send_command(Command.CALIBRATE, [calibrate_mode])
        return self.receive_answer()

    def need_calibration(self):
        res, obj_data = self.get_hardware_status()
        return obj_data.calibration_needed

    def set_learning_mode(self, enabled):
        self.send_command(Command.SET_LEARNING_MODE, [enabled])
        return self.receive_answer()

    def move_joints(self, j1, j2, j3, j4, j5, j6):
        self.send_command(Command.MOVE_JOINTS, [j1, j2, j3, j4, j5, j6])
        return self.receive_answer()

    def move_pose(self, x_pos, y_pos, z_pos, roll_rot, pitch_rot, yaw_rot):
        self.send_command(Command.MOVE_POSE, [x_pos, y_pos, z_pos, roll_rot, pitch_rot, yaw_rot])
        return self.receive_answer()

    def shift_pose(self, axis, shift_value):
        self.send_command(Command.SHIFT_POSE, [axis, shift_value])
        return self.receive_answer()

    def set_arm_max_velocity(self, percentage_speed):
        self.send_command(Command.SET_ARM_MAX_VELOCITY, [percentage_speed])
        return self.receive_answer()

    def enable_joystick(self, enabled):
        self.send_command(Command.ENABLE_JOYSTICK, [enabled])
        return self.receive_answer()

    def set_pin_mode(self, pin, pin_mode):
        self.send_command(Command.SET_PIN_MODE, [pin, pin_mode])
        return self.receive_answer()

    def digital_write(self, pin, digital_state):
        self.send_command(Command.DIGITAL_WRITE, [pin, digital_state])
        return self.receive_answer()

    def digital_read(self, pin):
        self.send_command(Command.DIGITAL_READ, [pin])
        status, data = self.receive_answer()
        if status is True:
            return status, int(data)
        return status, data

    def change_tool(self, tool):
        self.send_command(Command.CHANGE_TOOL, [tool])
        return self.receive_answer()

    def open_gripper(self, gripper, speed):
        self.send_command(Command.OPEN_GRIPPER, [gripper, speed])
        return self.receive_answer()

    def close_gripper(self, gripper, speed):
        self.send_command(Command.CLOSE_GRIPPER, [gripper, speed])
        return self.receive_answer()

    def pull_air_vacuum_pump(self, vacuum_pump):
        self.send_command(Command.PULL_AIR_VACUUM_PUMP, [vacuum_pump])
        return self.receive_answer()

    def push_air_vacuum_pump(self, vacuum_pump):
        self.send_command(Command.PUSH_AIR_VACUUM_PUMP, [vacuum_pump])
        return self.receive_answer()

    def setup_electromagnet(self, electromagnet, pin):
        self.send_command(Command.SETUP_ELECTROMAGNET, [electromagnet, pin])
        return self.receive_answer()

    def activate_electromagnet(self, electromagnet, pin):
        self.send_command(Command.ACTIVATE_ELECTROMAGNET, [electromagnet, pin])
        return self.receive_answer()

    def deactivate_electromagnet(self, electromagnet, pin):
        self.send_command(Command.DEACTIVATE_ELECTROMAGNET, [electromagnet, pin])
        return self.receive_answer()

    def get_saved_position_list(self):
        self.send_command(Command.GET_SAVED_POSITION_LIST)
        return self.receive_answer()

    def wait(self, duration):
        # self.send_command(Command.WAIT, [duration])
        # return self.receive_answer()
        time.sleep(duration)

    @staticmethod
    def pose_to_list(pose):
        list_pos = [pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw]
        return list(map(float, list_pos))

    def get_joints(self):
        self.send_command(Command.GET_JOINTS)
        status, data = self.receive_answer()
        if status is True:
            joint_array = list(map(float, data.split(',')))
            return status, joint_array
        return status, data

    def get_pose(self):
        self.send_command(Command.GET_POSE)
        status, data = self.receive_answer()
        if status is True:
            pose_array = list(map(float, data.split(',')))
            pose_object = PoseObject(*pose_array)
            return status, pose_object
        return status, data

    def get_hardware_status(self):
        self.send_command(Command.GET_HARDWARE_STATUS)
        status, data = self.receive_answer()
        # print status
        if status is True:
            first_infos = data[0:data.index(",[")].split(",")
            rpi_temperature = int(first_infos[0])
            hardware_version = int(first_infos[1])
            connection_up = bool(first_infos[2])
            error_message = first_infos[3].strip('\'')
            calibration_needed = int(first_infos[4])
            calibration_in_progress = bool(first_infos[5])
            motor_names = ast.literal_eval(data[data.index("["):data.index("]") + 1])
            motor_types = ast.literal_eval(data[data.index("],[") + 2:data.index("],(") + 1])

            last_infos = data[data.index("],(") + 2:].split("),(")
            temperatures = ast.literal_eval(last_infos[0] + ")")
            voltages = ast.literal_eval("(" + last_infos[1] + ")")
            hardware_errors = ast.literal_eval("(" + last_infos[2])

            hardware_status = HardwareStatusObject(rpi_temperature, hardware_version, connection_up, error_message,
                                                   calibration_needed, calibration_in_progress,
                                                   motor_names, motor_types,
                                                   temperatures, voltages, hardware_errors)
            return status, hardware_status
        return status, data

    def get_learning_mode(self):
        self.send_command(Command.GET_LEARNING_MODE)
        status, data = self.receive_answer()
        if status is True:
            return status, bool(data)
        return status, data

    def get_digital_io_state(self):
        self.send_command(Command.GET_DIGITAL_IO_STATE)
        status, data = self.receive_answer()
        if status is True:
            matches = re.findall('(\[\d+, ?\'\w+\', ?[0-1], \d+\])+', data)
            digital_pin_array = []
            for match in matches:
                elements = match.split(', ')
                pin_id = elements[0].lstrip('[')
                name = elements[1]
                mode = int(elements[2])
                state = int(elements[3].rstrip(']'))
                digital_pin_array.append(DigitalPinObject(pin_id, name, mode, state))
            return status, digital_pin_array
        return status, data

    def create_workspace(self, name, pose_origin, pose_1, pose_2, pose_3):
        param_list = [name]
        param_list.extend(self.pose_to_list(pose_origin))
        param_list.extend(self.pose_to_list(pose_1))
        param_list.extend(self.pose_to_list(pose_2))
        param_list.extend(self.pose_to_list(pose_3))
        self.send_command(Command.CREATE_WORKSPACE, param_list)
        return self.receive_answer()

    def remove_workspace(self, name):
        self.send_command(Command.REMOVE_WORKSPACE, [name])
        return self.receive_answer()

    def get_workspace_ratio(self, workspace_name):
        param_list = [workspace_name]
        self.send_command(Command.GET_WORKSPACE_RATIO, param_list)
        status, data = self.receive_answer()
        if status is True:
            return status, float(data)
        return status, data

    def get_workspace_list(self):
        self.send_command(Command.GET_WORKSPACE_LIST)
        status, data = self.receive_answer_long()
        if status is True:
            workspace_list = data.split(',')
            return status, workspace_list
        return status, data

    def get_img_compressed(self):
        self.send_command(Command.GET_IMAGE_COMPRESSED)
        status, data = self.receive_answer_long()
        return status, data

    def get_target_pose_from_rel(self, workspace, height_offset, x_rel, y_rel, yaw_rel):
        param_list = [workspace, height_offset, x_rel, y_rel, yaw_rel]
        self.send_command(Command.GET_TARGET_POSE_FROM_REL, param_list)
        status, data = self.receive_answer()
        if status is True:
            pose_array = list(map(float, data.split(',')))
            pose_object = PoseObject(*pose_array)
            return status, pose_object
        return status, data

    def get_target_pose_from_cam(self, workspace, height_offset, shape, color):
        param_list = [workspace, height_offset, shape, color]
        self.send_command(Command.GET_TARGET_POSE_FROM_CAM, param_list)
        status, data = self.receive_answer()

        if status is True:
            parameters_string_array = data.split(',')
            obj_found = parameters_string_array[0] == "True"
            if obj_found is True:
                # print parameters_string_array, obj_found
                pose_array = list(map(float, parameters_string_array[1:7]))
                pose_object = PoseObject(*pose_array)
                shape_ret = parameters_string_array[7]
                color_ret = parameters_string_array[8]
                return status, obj_found, pose_object, Shape[shape_ret], Color[color_ret]
        return status, False, None, "", ""

    def detect_object(self, workspace, shape, color):
        param_list = [workspace, shape, color]
        self.send_command(Command.DETECT_OBJECT, param_list)
        status, data = self.receive_answer()

        if not status:
            return False, False, [], "", ""

        parameters_string_array = data.split(',')
        obj_found = parameters_string_array[0] == "True"
        if not obj_found:
            return True, False, [], "", ""
        rel_pose_array = list(map(float, parameters_string_array[1:4]))
        shape = parameters_string_array[4]
        color = parameters_string_array[5]

        return status, obj_found, rel_pose_array, Shape[shape], Color[color]

    def __move_with_vision(self, workspace, height_offset, shape, color, command):
        param_list = [workspace, height_offset, shape, color]
        self.send_command(command, param_list)
        status, data = self.receive_answer()

        if status is True:
            parameters_string_array = data.split(',')
            obj_found = parameters_string_array[0] == "True"
            if obj_found is True:
                shape_ret = parameters_string_array[1]
                color_ret = parameters_string_array[2]
                return status, obj_found, Shape[shape_ret], Color[color_ret]
        return status, False, "", ""

    def vision_pick(self, workspace, height_offset, shape, color):
        return self.__move_with_vision(workspace, height_offset, shape, color, Command.VISION_PICK)

    def move_to_object(self, workspace, height_offset, shape, color):
        return self.__move_with_vision(workspace, height_offset, shape, color, Command.MOVE_TO_OBJECT)

    def activate_conveyor(self, conveyor_id):
        return self.set_conveyor(conveyor_id, activate=True)

    def deactivate_conveyor(self, conveyor_id):
        return self.set_conveyor(conveyor_id, activate=False)

    def set_conveyor(self, conveyor_id, activate):
        self.send_command(Command.SET_CONVEYOR, [conveyor_id, activate])
        status, data = self.receive_answer()
        return status, data

    def stop_conveyor(self, conveyor_id):
        return self.control_conveyor(conveyor_id, control_on=False, speed=50, direction=ConveyorDirection.FORWARD)

    def control_conveyor(self, conveyor_id, control_on, speed, direction):
        param_list = [conveyor_id, control_on, speed, direction]

        self.send_command(Command.CONTROL_CONVEYOR, param_list)
        status, data = self.receive_answer()
        return status, data

    def update_conveyor_id(self, old_id, new_id):
        self.send_command(Command.UPDATE_CONVEYOR_ID, [old_id, new_id])
        status, data = self.receive_answer()
        return status, data

    def get_current_tool_id(self):
        self.send_command(Command.GET_CURRENT_TOOL_ID)
        status, data = self.receive_answer()
        if status is True:
            return status, RobotTool[data]
        return status, data

    def pick_from_pose(self, x_pos, y_pos, z_pos, roll_rot, pitch_rot, yaw_rot):
        self.send_command(Command.PICK_FROM_POSE, [x_pos, y_pos, z_pos, roll_rot, pitch_rot, yaw_rot])
        return self.receive_answer()

    def place_from_pose(self, x_pos, y_pos, z_pos, roll_rot, pitch_rot, yaw_rot):
        self.send_command(Command.PLACE_FROM_POSE, [x_pos, y_pos, z_pos, roll_rot, pitch_rot, yaw_rot])
        return self.receive_answer()

    def get_calibration_object(self):
        self.send_command(Command.GET_CALIBRATION_OBJECT, [])
        _, data_raw = self.receive_answer_long()

        status, data = data_raw.split(",", 1)
        if not status:
            return status, None, None, None
        list_data = ast.literal_eval(data)

        mtx = np.reshape(list_data[0], (3, 3))
        dist = np.expand_dims(list_data[1], axis=0)
        return status, mtx, dist

    def send_command(self, command_type, parameter_list=None):
        if self.__is_connected is False:
            raise self.ClientNotConnectedException()
        send_success = False
        if self.__client_socket is not None:
            try:
                packet = self.__packet_builder.build_command_packet(command_type, parameter_list)
                if sys.version_info[0] == 3:
                    packet = packet.encode()
                self.__client_socket.send(packet)
            except socket.error as e:
                print(e)
                raise self.HostNotReachableException()
        return send_success

    def receive_answer(self):
        READ_SIZE = 512
        try:
            received = self.__client_socket.recv(READ_SIZE)
        except socket.error as e:
            print(e)
            raise self.HostNotReachableException()
        if not received:
            raise self.HostNotReachableException()
        if sys.version_info[0] == 3:
            received = received.decode()
        received_split_list = received.split(':', 1)
        if len(received_split_list) != 2:
            raise self.InvalidAnswerException(received)
        command_answer = received_split_list[1]

        # If 'OK' with data or 'KO' with reason
        if ',' in command_answer:
            command_answer_split_list = command_answer.split(',', 1)
            if len(command_answer_split_list) != 2:
                raise self.InvalidAnswerException(command_answer)
            answer_status = command_answer_split_list[0]
            if answer_status == "OK" and answer_status == "KO":
                raise self.InvalidAnswerException(answer_status)
            answer_data = command_answer_split_list[1]
            return answer_status == "OK", answer_data

        if command_answer == "OK" and command_answer == "KO":
            raise self.InvalidAnswerException(command_answer)
        return command_answer == "OK", None

    def receive_answer_long(self):
        READ_SIZE = 512  # Should be enough to read cmd string, status and size
        try:
            received = self.__client_socket.recv(READ_SIZE)
        except socket.error as e:
            raise self.HostNotReachableException()
        if not received:
            raise self.HostNotReachableException()
        if sys.version_info[0] == 2:
            received_split_list = received.split(':', 1)
            if len(received_split_list) != 2:
                raise self.InvalidAnswerException(received)
            command_answer = received_split_list[1]
            command_answer_split_list = command_answer.split(',', 2)
            if len(command_answer_split_list) < 3:
                raise self.InvalidAnswerException("long answer needs to have format: " \
                                                  "STATUS,PAYLOAD_SIZE,PAYLOAD. Got: {}".format(command_answer))
            answer_status = command_answer_split_list[0]
            try:
                payload_size = int(command_answer_split_list[1])
            except ValueError:
                raise self.InvalidAnswerException("PAYLOAD_SIZE needs to be integer. " \
                                                  "But '{}' cannot be converted to integer".format(
                    command_answer_split_list[1]))
            payload = command_answer_split_list[2]
            while len(payload) < payload_size:
                try:
                    received = self.__client_socket.recv(READ_SIZE)
                except socket.error as e:
                    raise self.HostNotReachableException()
                if not received:
                    raise self.HostNotReachableException()
                payload += received
        else:
            index_double_point = received.find(b':')
            received_split_list = [received[:index_double_point].decode(), received[index_double_point + 1:]]
            rest = received_split_list[1]
            index_first_comma = rest.find(b',')
            first_split = [rest[:index_first_comma].decode(), rest[index_first_comma + 1:]]
            answer_status, rest = first_split
            index_second_comma = rest.find(b',')
            second_split = [rest[:index_second_comma].decode(), rest[index_second_comma + 1:]]
            payload_size_str, payload = second_split
            try:
                payload_size = int(payload_size_str)
            except ValueError:
                raise self.InvalidAnswerException("PAYLOAD_SIZE needs to be integer. " \
                                                  "But '{}' cannot be converted to integer".format(payload_size_str))
            while len(payload) < payload_size:
                try:
                    received = self.__client_socket.recv(READ_SIZE)
                except socket.error as e:
                    raise self.HostNotReachableException()
                if not received:
                    raise self.HostNotReachableException()
                payload += received
            try:
                payload = payload.decode()
            except UnicodeDecodeError:
                pass
        return answer_status == "OK", payload
