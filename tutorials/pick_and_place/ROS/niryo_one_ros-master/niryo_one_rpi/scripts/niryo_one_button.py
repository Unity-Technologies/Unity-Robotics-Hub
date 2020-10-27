#!/usr/bin/env python

# niryo_one_button.py
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
import RPi.GPIO as GPIO
import subprocess

from niryo_one_rpi.rpi_ros_utils import *

from std_msgs.msg import Int32, Bool
from niryo_one_msgs.srv import SetInt

BUTTON_GPIO = 4


class ButtonMode:
    def __init__(self):
        pass

    DO_NOTHING = 0
    TRIGGER_SEQUENCE_AUTORUN = 1
    BLOCKLY_SAVE_POINT = 2


class NiryoButton:

    def read_value(self):
        return GPIO.input(self.pin)

    def __init__(self):
        self.pin = BUTTON_GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        rospy.loginfo("GPIO setup : " + str(self.pin) + " as input")
        rospy.sleep(0.1)

        self.last_state = self.read_value()
        self.consecutive_pressed = 0
        self.activated = True

        self.timer_frequency = 20.0
        self.shutdown_action = False
        self.hotspot_action = False

        self.button_mode = ButtonMode.TRIGGER_SEQUENCE_AUTORUN
        self.change_button_mode_server = rospy.Service(
            "/niryo_one/rpi/change_button_mode", SetInt, self.callback_change_button_mode)
        self.monitor_button_mode_timer = rospy.Timer(rospy.Duration(3.0), self.monitor_button_mode)
        self.last_time_button_mode_changed = rospy.Time.now()

        # Publisher used to send info to Niryo One Studio, so the user can add a move block
        # by pressing the button
        self.save_point_publisher = rospy.Publisher(
            "/niryo_one/blockly/save_current_point", Int32, queue_size=10)

        self.button_state_publisher = rospy.Publisher(
            "/niryo_one/rpi/is_button_pressed", Bool, queue_size=10)

        self.button_timer = rospy.Timer(rospy.Duration(1.0 / self.timer_frequency), self.check_button)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Niryo One Button started")

    def shutdown(self):
        rospy.loginfo("Shutdown button, cleanup GPIO")
        self.button_timer.shutdown()

    def blockly_save_current_point(self):
        msg = Int32()
        msg.data = 1
        self.save_point_publisher.publish(msg)

    def callback_change_button_mode(self, req):
        message = ""
        if req.value == ButtonMode.TRIGGER_SEQUENCE_AUTORUN:
            message = "Successfully changed button mode to trigger sequence autorun"
        elif req.value == ButtonMode.BLOCKLY_SAVE_POINT:
            message = "Successfully changed button mode to save point"
        elif req.value == ButtonMode.DO_NOTHING:
            message = "Successfully changed button mode to disabled"
        else:
            return {"status": 400, "message": "Incorrect button mode."}
        self.button_mode = req.value
        self.last_time_button_mode_changed = rospy.Time.now()
        return {"status": 200, "message": message}

    def monitor_button_mode(self, event):
        duration = rospy.Time.now() - self.last_time_button_mode_changed
        # Make sure the button's behavior goes back to default if no news from the Blockly interface
        # This is to prevent the button from being stuck in BLOCKLY_SAVE_POINT mode in case of a 
        # deconnection or broken communication
        # --> To keep the non-default mode, you have to call the change_button_mode service at least
        # every 3 seconds
        if self.button_mode == ButtonMode.BLOCKLY_SAVE_POINT and duration.to_sec() > 3.0:
            self.button_mode = ButtonMode.TRIGGER_SEQUENCE_AUTORUN

    def check_button(self, event):
        if not self.activated:
            return

        # Execute action if flag True
        if self.hotspot_action:
            send_hotspot_command()
            self.hotspot_action = False
            self.shutdown_action = False
        elif self.shutdown_action:
            send_shutdown_command()
            self.hotspot_action = False
            self.shutdown_action = False

        # Read button state
        state = self.read_value()
        
        # Publish button is_pressed
        msg = Bool()
        msg.data = state == 0
        self.button_state_publisher.publish(msg)

        # Check if there is an action to do
        if state == 0:
            self.consecutive_pressed += 1
        elif state == 1:  # button released
            if self.consecutive_pressed > self.timer_frequency * 20:
                self.activated = False  # deactivate button if pressed more than 20 seconds
            elif self.consecutive_pressed > self.timer_frequency * 6:
                self.hotspot_action = True
            elif self.consecutive_pressed > self.timer_frequency * 3:
                self.shutdown_action = True
            elif self.consecutive_pressed >= 1:
                if self.button_mode == ButtonMode.TRIGGER_SEQUENCE_AUTORUN:
                    send_trigger_sequence_autorun()
                elif self.button_mode == ButtonMode.BLOCKLY_SAVE_POINT:
                    self.blockly_save_current_point()
            self.consecutive_pressed = 0

        # Use LED to help user know which action to execute
        if self.consecutive_pressed > self.timer_frequency * 20:
            send_led_state(LedState.SHUTDOWN)
        elif self.consecutive_pressed > self.timer_frequency * 6:
            send_led_state(LedState.WAIT_HOTSPOT)
        elif self.consecutive_pressed > self.timer_frequency * 3:
            send_led_state(LedState.SHUTDOWN)
