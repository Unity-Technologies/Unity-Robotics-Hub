#!/usr/bin/env python

# rpi_ros_utils.py
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
import rospkg
import subprocess
import re

from niryo_one_msgs.srv import SetInt

ENABLE_BUS_MOTORS_SUCCESS = 1
ENABLE_BUS_MOTORS_READ_FAIL = -1
ENABLE_BUS_MOTORS_WRITE_FAIL = -2

CHANGE_MOTOR_CONFIG_SUCCESS = 1
CHANGE_MOTOR_CONFIG_READ_FAIL = -1
CHANGE_MOTOR_CONFIG_WRITE_FAIL = -2
CHANGE_MOTOR_CONFIG_WRONG_VERSION = -3


class LedState:
    def __init__(self):
        pass

    SHUTDOWN = 1
    HOTSPOT = 2
    HW_ERROR = 3
    OK = 4
    WAIT_HOTSPOT = 5


def send_hotspot_command():
    rospy.loginfo("HOTSPOT")
    send_led_state(LedState.WAIT_HOTSPOT)
    rospy.wait_for_service('/niryo_one/wifi/set_hotspot')
    try:
        set_hotspot = rospy.ServiceProxy('/niryo_one/wifi/set_hotspot', SetInt)
        set_hotspot()
    except rospy.ServiceException, e:
        rospy.logwarn("Could not call set_hotspot service")


def send_trigger_sequence_autorun():
    rospy.loginfo("Trigger sequence autorun from button")
    try:
        rospy.wait_for_service('/niryo_one/sequences/trigger_sequence_autorun', 0.1)
        trigger = rospy.ServiceProxy('/niryo_one/sequences/trigger_sequence_autorun', SetInt)
        trigger(1)  # value doesn't matter, it will switch state on the server
    except (rospy.ServiceException, rospy.ROSException), e:
        return


def send_reboot_motors_command():
    rospy.loginfo("Send reboot motor command")
    try:
        rospy.wait_for_service('/niryo_one/reboot_motors', 1)
    except rospy.ROSException, e:
        pass
    try:
        reboot_motors = rospy.ServiceProxy('/niryo_one/reboot_motors', SetInt)
        reboot_motors(1)
    except rospy.ServiceException, e:
        pass


def send_shutdown_command():
    rospy.loginfo("SHUTDOWN")
    send_led_state(LedState.SHUTDOWN)
    rospy.loginfo("Activate learning mode")
    try:
        rospy.wait_for_service('/niryo_one/activate_learning_mode', 1)
    except rospy.ROSException, e:
        pass
    try:
        activate_learning_mode = rospy.ServiceProxy('/niryo_one/activate_learning_mode', SetInt)
        activate_learning_mode(1)
    except rospy.ServiceException, e:
        pass
    send_reboot_motors_command()
    rospy.sleep(0.2)
    rospy.loginfo("Command 'sudo shutdown now'")
    try:
        output = subprocess.check_output(['sudo', 'shutdown', 'now'])
    except subprocess.CalledProcessError:
        rospy.loginfo("Can't exec shutdown cmd")


def send_reboot_command():
    rospy.loginfo("REBOOT")
    send_led_state(LedState.SHUTDOWN)
    rospy.loginfo("Activate learning mode")
    try:
        rospy.wait_for_service('/niryo_one/activate_learning_mode', 1)
    except rospy.ROSException, e:
        pass
    try:
        activate_learning_mode = rospy.ServiceProxy('/niryo_one/activate_learning_mode', SetInt)
        activate_learning_mode(1)
    except rospy.ServiceException, e:
        pass
    send_reboot_motors_command()
    rospy.sleep(0.2)
    rospy.loginfo("Command 'sudo reboot'")
    try:
        output = subprocess.check_output(['sudo', 'reboot'])
    except subprocess.CalledProcessError:
        rospy.loginfo("Can't exec reboot cmd")


def send_led_state(state):
    rospy.wait_for_service('/niryo_one/rpi/set_led_state')
    try:
        set_led = rospy.ServiceProxy('/niryo_one/rpi/set_led_state', SetInt)
        set_led(state)
    except rospy.ServiceException, e:
        rospy.logwarn("Could not call set_led_state service")


def enable_bus_motors_in_config_file(enable_can_bus=True, enable_dxl_bus=True):
    rospack = rospkg.RosPack()
    folder_path = rospack.get_path('niryo_one_bringup')
    file_path = folder_path + '/launch/controllers.launch'
    text = ""
    rospy.loginfo("Change launch file (for debug purposes): " + str(file_path))
    rospy.loginfo("Enable can_bus: " + str(enable_can_bus))
    rospy.loginfo("Enable dxl bus: " + str(enable_dxl_bus))

    # Open file and get text
    try:
        with open(file_path, 'r') as f:
            text = f.read()
    except EnvironmentError as e:
        rospy.logerr(e)
        return ENABLE_BUS_MOTORS_READ_FAIL

    # Change disable_can
    new_line_can_bus = "<arg name=\"disable_can_for_debug\" default=\"false\""
    if not enable_can_bus:
        new_line_can_bus = "<arg name=\"disable_can_for_debug\" default=\"true\""
    text = re.sub(r"\<arg name=\"disable_can_for_debug\" default=\"\w+\"", new_line_can_bus, text)

    # Change disable_dxl
    new_line_dxl_bus = "<arg name=\"disable_dxl_for_debug\" default=\"false\""
    if not enable_dxl_bus:
        new_line_dxl_bus = "<arg name=\"disable_dxl_for_debug\" default=\"true\""
    text = re.sub(r"\<arg name=\"disable_dxl_for_debug\" default=\"\w+\"", new_line_dxl_bus, text)

    # Rewrite file with new text
    try:
        with open(file_path, 'w') as f:
            f.write(text)
    except EnvironmentError as e:
        rospy.logerr(e)
        return ENABLE_BUS_MOTORS_WRITE_FAIL
    return ENABLE_BUS_MOTORS_SUCCESS


def change_motor_config_file(hw_version, can_id_list, dxl_id_list):
    rospack = rospkg.RosPack()
    folder_path = rospack.get_path('niryo_one_bringup')
    file_path = folder_path + '/config/v' + str(int(hw_version)) + '/niryo_one_motors.yaml'
    lines = []
    can_bus_enabled = len(can_id_list) > 0
    dxl_bus_enabled = len(dxl_id_list) > 0
    rospy.loginfo("Change motor config file (for debug purposes): " + str(file_path))
    rospy.loginfo("Hw version: " + str(hw_version))
    rospy.loginfo("CAN ID list: " + str(can_id_list))
    rospy.loginfo("DXL ID list: " + str(dxl_id_list))

    if hw_version != 1 and hw_version != 2:
        return CHANGE_MOTOR_CONFIG_WRONG_VERSION

    # Open file, get text and split lines
    try:
        with open(file_path, 'r') as f:
            lines = f.read().splitlines()
    except EnvironmentError as e:
        rospy.logerr(e)
        return CHANGE_MOTOR_CONFIG_READ_FAIL

    text_begin_lines = []
    text_can_lines = []
    text_dxl_lines = []
    text_end_lines = []

    # Get beginning of file (not modified)
    for line in lines:
        if not line.startswith('#'):
            break
        text_begin_lines.append(line)

    # Get end of file (not modified)
    for i, line in enumerate(lines):
        if line.startswith('dxl_authorized_motors'):
            text_end_lines = lines[i:]
            break

    # Fill CAN motor IDs depending on given ID array
    # If CAN bus is already disabled, then don't disable motors here
    if hw_version == 1:
        text_can_lines.append(
            'can_required_motors: # Axis 1-4 of Niryo One (stepper 1 -> id 1, stepper 2 -> id 2, ...)')
    else:
        text_can_lines.append(
            'can_required_motors: # Axis 1-3 of Niryo One (stepper 1 -> id 1, stepper 2 -> id 2, ...)')
    text_can_lines.append(
        '    # Edit only for debug purposes (ex : you want to test some motors separately and detached from the robot)')
    text_can_lines.append(
        '    # --> Commented ids will make associated motor disable (and thus not trigger an error if not connected)')
    text_can_lines.append('    # Before editing, please be sure that you know what you\'re doing')
    if 1 in can_id_list or not can_bus_enabled:
        text_can_lines.append('    - 1 # Axis 1 enabled if not commented')
    else:
        text_can_lines.append('    #- 1 # Axis 1 enabled if not commented')
    if 2 in can_id_list or not can_bus_enabled:
        text_can_lines.append('    - 2 # Axis 2 enabled if not commented')
    else:
        text_can_lines.append('    #- 2 # Axis 2 enabled if not commented')
    if 3 in can_id_list or not can_bus_enabled:
        text_can_lines.append('    - 3 # Axis 3 enabled if not commented')
    else:
        text_can_lines.append('    #- 3 # Axis 3 enabled if not commented')
    if hw_version == 1:
        if 4 in can_id_list or not can_bus_enabled:
            text_can_lines.append('    - 4 # Axis 4 enabled if not commented')
        else:
            text_can_lines.append('    #- 4 # Axis 4 enabled if not commented')

    # Fill DXL motor IDs depending on given ID array
    # If DXL bus is already disabled, then don't disable motors here
    if hw_version == 1:
        text_dxl_lines.append('dxl_required_motors: # axis 5 and 6 of Niryo One.')
    else:
        text_dxl_lines.append('dxl_required_motors: # axis 4, 5 and 6 of Niryo One.')
    text_dxl_lines.append(
        '    # Edit only for debug purposes (ex : you want to test some motors separately and detached from the robot)')
    text_dxl_lines.append(
        '    # --> Commented ids will make associated motor disable (and thus not trigger an error if not connected)')
    text_dxl_lines.append('    # Before editing, please be sure that you know what you\'re doing')
    if hw_version == 1:
        text_dxl_lines.append(
            '    # - Note : Axis 5 has 2 motors, but you can use only one motor for this axis when debugging')
    if hw_version == 1:
        if 4 in dxl_id_list or not dxl_bus_enabled:
            text_dxl_lines.append('    - 4 # -> id of Axis 5_1')
        else:
            text_dxl_lines.append('    #- 4 # -> id of Axis 5_1')
        if 5 in dxl_id_list or not dxl_bus_enabled:
            text_dxl_lines.append('    - 5 # -> id of Axis 5_2')
        else:
            text_dxl_lines.append('    #- 5 # -> id of Axis 5_2')
        if 6 in dxl_id_list or not dxl_bus_enabled:
            text_dxl_lines.append('    - 6 # -> id of Axis 6')
        else:
            text_dxl_lines.append('    #- 6 # -> id of Axis 6')
    else:
        if 2 in dxl_id_list or not dxl_bus_enabled:
            text_dxl_lines.append('    - 2 # -> id of Axis 4')
        else:
            text_dxl_lines.append('    #- 2 # -> id of Axis 4')
        if 3 in dxl_id_list or not dxl_bus_enabled:
            text_dxl_lines.append('    - 3 # -> id of Axis 5')
        else:
            text_dxl_lines.append('    #- 3 # -> id of Axis 5')
        if 6 in dxl_id_list or not dxl_bus_enabled:
            text_dxl_lines.append('    - 6 # -> id of Axis 6')
        else:
            text_dxl_lines.append('    #- 6 # -> id of Axis 6')

    new_text = '\n'.join(
        text_begin_lines + [''] + text_can_lines + [''] + text_dxl_lines + [''] + text_end_lines + [''])

    # Rewrite file with new text
    try:
        with open(file_path, 'w') as f:
            f.write(new_text)
    except EnvironmentError as e:
        rospy.logerr(e)
        return CHANGE_MOTOR_CONFIG_WRITE_FAIL
    return CHANGE_MOTOR_CONFIG_SUCCESS
