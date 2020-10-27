#!/usr/bin/env python

# fans_manager.py
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
from std_msgs.msg import Bool

import RPi.GPIO as GPIO

FAN_1_GPIO = 27
FAN_2_GPIO = 23


class FansManager:

    def __init__(self):
        self.setup_fans()
        self.learning_mode_on = True
        # Activate fans for 5 seconds to give an audio signal to the user
        self.set_fans(True)
        rospy.sleep(5)
        self.set_fans(not self.learning_mode_on)

        self.learning_mode_subscriber = rospy.Subscriber(
            '/niryo_one/learning_mode', Bool, self.callback_learning_mode)

    @staticmethod
    def setup_fans():
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(FAN_1_GPIO, GPIO.OUT)
        GPIO.setup(FAN_2_GPIO, GPIO.OUT)
        rospy.sleep(0.05)
        rospy.loginfo("------ RPI FANS SETUP OK ------")

    @staticmethod
    def set_fans(activate):
        if activate:
            GPIO.output(FAN_1_GPIO, GPIO.HIGH)
            GPIO.output(FAN_2_GPIO, GPIO.HIGH)
        else:
            GPIO.output(FAN_1_GPIO, GPIO.LOW)
            GPIO.output(FAN_2_GPIO, GPIO.LOW)

    def callback_learning_mode(self, msg):
        if msg.data != self.learning_mode_on:
            self.learning_mode_on = msg.data
            self.set_fans(not self.learning_mode_on)
