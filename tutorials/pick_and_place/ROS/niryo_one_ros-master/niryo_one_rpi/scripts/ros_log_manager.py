#!/usr/bin/env python

# ros_log_manager.py
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
import subprocess
import os

from niryo_one_msgs.msg import LogStatus
from niryo_one_msgs.srv import SetInt

#
# This class will handle ROS logs on Raspberry Pi
#

"""
todo : 

- service to get latest ros log nicely formated
- detect when ros log is too big (> 1GB) -> alert message

"""


class RosLogManager:

    def __init__(self):
        self.log_size_treshold = rospy.get_param("~ros_log_size_treshold")
        self.log_path = rospy.get_param("~ros_log_location")
        self.should_purge_log_on_startup_file = rospy.get_param("~should_purge_ros_log_on_startup_file")
        self.purge_log_on_startup = self.should_purge_log_on_startup()

        # clean log on startup if param is true
        if self.purge_log_on_startup:
            rospy.logwarn("Purging ROS log on startup !")
            print self.purge_log()

        self.purge_log_server = rospy.Service('/niryo_one/rpi/purge_ros_logs', SetInt,
                                              self.callback_purge_log)

        self.change_purge_log_on_startup_server = rospy.Service('/niryo_one/rpi/set_purge_ros_log_on_startup', SetInt,
                                                                self.callback_change_purge_log_on_startup)

        self.log_status_publisher = rospy.Publisher('/niryo_one/rpi/ros_log_status', LogStatus, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(3), self.publish_log_status)

        rospy.loginfo("Init Ros Log Manager OK")

    @staticmethod
    def get_available_disk_size():
        try:
            process = subprocess.Popen(['df', '--output=avail', '/'], stdout=subprocess.PIPE)
            output, error = process.communicate()
            lines = output.split(os.linesep)
            if len(lines) >= 2:
                return int(lines[1]) / 1024
            return -1
        except subprocess.CalledProcessError:
            return -1

    def get_log_size(self):
        try:
            if not os.path.isdir(self.log_path):
                return -1
            output = subprocess.check_output(['du', '-sBM', self.log_path])
            output_array = output.split()
            if len(output_array) >= 1:
                return int(output_array[0].replace('M', ''))
            return -1
        except subprocess.CalledProcessError:
            return -1

    # !! ros logs after using this method will not be saved
    # need to restart ros completly to save new logs
    def purge_log(self):
        try:
            subprocess.call(['mkdir', '-p', self.log_path])
            subprocess.call(['rm', '-rf', "{}/*".format(self.log_path)])
            return True
        except subprocess.CalledProcessError:
            return False

    def should_purge_log_on_startup(self):
        if os.path.isfile(self.should_purge_log_on_startup_file):
            with open(self.should_purge_log_on_startup_file, 'r') as f:
                for line in f:
                    if not (line.startswith('#') or len(line) == 0):
                        condition = line.rstrip()
                        if condition == "true":
                            return True
                        return False
        return False

    def change_purge_log_on_startup(self, condition):
        with open(self.should_purge_log_on_startup_file, 'w') as f:
            if condition:
                value = "true"
            else:
                value = "false"
            f.write(value)

        # After writing, read new value from file
        self.purge_log_on_startup = self.should_purge_log_on_startup()

    # 
    # ----- ROS Interface below ----- 
    #

    @staticmethod
    def create_response(status, message):
        return {'status': status, 'message': message}

    def callback_purge_log(self, _):
        rospy.logwarn("Purge ROS logs on user request")
        if self.purge_log():
            return self.create_response(200, "ROS logs have been purged. " +
                                        "Following logs will be discarded. If you want to get logs, you " +
                                        "need to restart the robot")
        return self.create_response(400, "Unable to remove ROS logs")

    def callback_change_purge_log_on_startup(self, req):
        if req.value == 1:
            self.change_purge_log_on_startup(True)
        else:
            self.change_purge_log_on_startup(False)
        return self.create_response(200, "Purge log on startup value has been changed")

    def publish_log_status(self, _):
        msg = LogStatus()
        msg.header.stamp = rospy.Time.now()
        msg.log_size = self.get_log_size()
        msg.available_disk_size = self.get_available_disk_size()
        msg.purge_log_on_startup = self.purge_log_on_startup
        self.log_status_publisher.publish(msg)
