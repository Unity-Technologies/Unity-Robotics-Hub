#!/usr/bin/env python

# niryo_one_ros_setup.py
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
import time
import subprocess

from niryo_one_msgs.msg import ProcessState
from niryo_one_msgs.srv import ManageProcess

PROCESS_TIMEOUT_RESTART = 5.0  # sec


def kill_rosmaster():
    subprocess.Popen(['killall', '-9', 'rosmaster'])


class ProcessNotFound(Exception): pass


class ProcessActionType(object):
    START = 1
    STOP = 2
    RESTART = 3
    KILL = 4
    START_ALL = 5
    STOP_ALL = 6


class Process:

    def __init__(self, name, cmd, args=None, launch_on_startup=False,
                 delay_before_start=0.0, dependencies=None):
        self.name = name
        self.cmd = cmd
        self.args = args if args is not None else []
        self.dependencies = dependencies if dependencies is not None else []
        self.launch_on_startup = launch_on_startup
        self.delay_before_start = delay_before_start
        self.process = None

    def start(self):
        if not self.is_active():
            cmd = self.cmd.split(' ') + self.args
            if self.delay_before_start:
                rospy.sleep(self.delay_before_start)
            self.process = subprocess.Popen(cmd)

    def restart(self):
        self.stop()
        timeout = time.time() + PROCESS_TIMEOUT_RESTART
        while self.is_active():
            if time.time() > timeout:
                break
            rospy.sleep(0.1)
        self.start()

    def stop(self):
        if self.process:
            self.process.terminate()

    def kill(self):
        if self.process:
            self.process.kill()

    def is_active(self):
        if not self.process:
            return False

        return_code = self.process.poll()
        if return_code is None or return_code < 0:
            return True
        return False


class NiryoOneRosSetup:

    def __init__(self):
        self.process_list = []
        self.process_config = rospy.get_param("~processes")
        self.create_processes()

        rospy.on_shutdown(self.clean_ros_processes)

        self.process_state_publish_rate = rospy.get_param("~process_state_publish_rate")

        self.process_state_publisher = rospy.Publisher(
            '/niryo_one/rpi/process_state', ProcessState, queue_size=1)

        rospy.Timer(rospy.Duration(1.0 / self.process_state_publish_rate), self.publish_process_state)

        self.manage_process_server = rospy.Service(
            '/niryo_one/rpi/manage_process', ManageProcess, self.callback_manage_process)

        self.start_init_processes()
        # self.start_all_processes()

    @staticmethod
    def create_response(status, message):
        return {'status': status, 'message': message}

    def publish_process_state(self, event):
        msg = ProcessState()
        for p in self.process_list:
            msg.name.append(p.name)
            msg.is_active.append(p.is_active())
        self.process_state_publisher.publish(msg)

    def callback_manage_process(self, req):
        process_name = req.name
        action = req.action

        try:
            if action == ProcessActionType.START_ALL:
                self.start_all_processes()
                return self.create_response(200, "All processes have been started")

            if action == ProcessActionType.STOP_ALL:
                self.stop_all_processes()
                return self.create_response(200, "All processes have been stopped")

            if action == ProcessActionType.START:
                self.start_process_from_name(process_name, start_dependencies=True)
                return self.create_response(200, "Process has been started")
            elif action == ProcessActionType.STOP:
                self.stop_process_from_name(process_name)  # also stop processes that depends on this process ?   
                return self.create_response(200, "Process has been stopped")
            elif action == ProcessActionType.RESTART:
                self.restart_process_from_name(process_name)
                return self.create_response(200, "Process has been restarted")
            elif action == ProcessActionType.KILL:
                self.kill_process_from_name(process_name)
                return self.create_response(200, "Process has been killed")

        except ProcessNotFound as e:
            return self.create_response(400, str(e))

    def clean_ros_processes(self):
        self.stop_all_processes()
        kill_rosmaster()

    def create_processes(self):
        rospy.loginfo("Start creating processes from rosparams")
        for p in self.process_config:
            self.process_list.append(Process(name=p['name'], cmd=p['cmd'], args=p['args'],
                                             launch_on_startup=p['launch_on_startup'],
                                             delay_before_start=p['delay_before_start'],
                                             dependencies=p['dependencies']))

    def start_init_processes(self):
        for process in self.process_list:
            if process.launch_on_startup:
                self.start_process(process, start_dependencies=True)

    def start_all_processes(self):
        for process in self.process_list:
            self.start_process(process, start_dependencies=True)

    def stop_all_processes(self):
        for p in self.process_list:
            # rospy.loginfo("STOPPING PROCESS : " + str(p.name))
            self.stop_process(p)

    def get_process_from_name(self, name):
        p = None
        for process in self.process_list:
            if process.name == name:
                p = process
                break
        if p is None:
            raise ProcessNotFound("Process not found : " + str(name))
        return p

    def get_dependency_process_list(self, process):
        dep_name_list = process.dependencies
        try:
            return list(map(lambda dep_name: self.get_process_from_name(dep_name), dep_name_list))
        except ProcessNotFound, e:  # should never happen if yaml file is correct
            rospy.logwarn("Some dependency names are incorrect. Check your setup.yaml file to fix it")
            return []

    def are_dependencies_met(self, process):
        process_dep_list = self.get_dependency_process_list(process)
        for p in process_dep_list:
            if not p.is_active():  # is_active doesn't mean all the nodes are fully started (not a problem if nodes wait for services and actions)
                rospy.loginfo("Unmet dependency for " + str(process.name) + " (depends on : " + str(p.name) + ") !")
                return False
        return True

    # CAREFUL : recursion - todo mettre une securite (pas plus de 5 depth)
    def check_and_start_dependencies(self, process):
        process_dep_list = self.get_dependency_process_list(process)
        for p in process_dep_list:
            if not p.is_active():  # is_active doesn't mean all the nodes are fully started (not a problem if nodes wait for services and actions)
                rospy.loginfo("Unmet dependency for " + str(process.name) + " (depends on : " + str(p.name) + ") !")
                rospy.loginfo("Starting dependency process...")
                self.start_process(p, start_dependencies=True)

    def start_process(self, p, start_dependencies=False):
        rospy.loginfo("Handle process : " + str(p.name))
        if start_dependencies:
            self.check_and_start_dependencies(p)
            rospy.loginfo("Start process : " + str(p.name))
            p.start()
        else:
            if self.are_dependencies_met(p):
                rospy.loginfo("Start process : " + str(p.name))
                p.start()

    @staticmethod
    def stop_process(p):
        p.stop()

    @staticmethod
    def restart_process(p):
        p.restart()

    @staticmethod
    def kill_process(p):
        p.kill()

    def start_process_from_name(self, name, start_dependencies=False):
        p = self.get_process_from_name(name)
        self.start_process(p, start_dependencies=start_dependencies)

    def stop_process_from_name(self, name):
        p = self.get_process_from_name(name)
        self.stop_process(p)

    def restart_process_from_name(self, name):
        p = self.get_process_from_name(name)
        self.restart_process(p)

    def kill_process_from_name(self, name):
        p = self.get_process_from_name(name)
        self.kill_process(p)


if __name__ == '__main__':
    pass
    # rospy.init_node('niryo_one_ros_setup')
    # n = NiryoOneRosSetup()
    # rospy.spin()
