#!/usr/bin/env python

# blockly_code_generator.py
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
import socket
import os
import signal
import ast

HOST = '127.0.0.1'


def cleanup_node_tcp_port(port_number):
    try:
        output = subprocess.check_output(['lsof', '-i', 'tcp:' + str(port_number)])
        for line in output.split(os.linesep):
            if 'node' in line:
                lines = line.split()
                if len(lines) > 2:
                    subprocess.call(['kill', '-9', str(lines[1])])
    except subprocess.CalledProcessError, e:
        pass  # No process found


def create_directory(directory_path):
    if not os.path.exists(directory_path):
        os.makedirs(directory_path)


def save_to_file(filename, text):
    with open(filename, 'w') as target:
        target.write(text)


def read_file(filename):
    with open(filename, 'r') as target:
        return target.read()


class BlocklyCodeGenerator:

    def __init__(self):
        self.blockly_dir = rospy.get_param("~niryo_one_blockly_path")
        self.tcp_port = rospy.get_param("~niryo_one_blockly_tcp_port")
        self.blockly_xml_file = str(self.blockly_dir) + '/blockly_xml'
        self.blockly_python_file = str(self.blockly_dir) + '/blockly_python'
        create_directory(self.blockly_dir)

        # Cleanup tcp port if another Nodejs server didn't shutdown normally
        cleanup_node_tcp_port(self.tcp_port)

        # Start Nodejs server
        self.blockly_generator_server = subprocess.Popen("blockly_code_generator_server", shell=True)
        rospy.loginfo("Blockly code generator started")

        self.socket = None

    def shutdown(self):
        if self.socket:
            self.socket.close()
        rospy.loginfo("Shutdown blockly code generator : Kill PID : " + str(self.blockly_generator_server.pid))
        os.kill(self.blockly_generator_server.pid, signal.SIGINT)

    #
    # input : correctly formatted XML on one line
    # output : generated Python code from XML
    #
    def get_generated_python_code(self, xml_code):

        # 1. Save xml to file
        save_to_file(self.blockly_xml_file, xml_code)

        # 2. Create socket
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(6)
        except socket.error, msg:
            return {'status': 400, 'message': msg}

        # 3. Connect to nodejs server
        try:
            self.socket.connect((HOST, self.tcp_port))
        except socket.error, msg:
            self.socket.close()
            return {'status': 400, 'message': msg}

        # 4. Send filename to nodejs server
        try:
            self.socket.sendall(self.blockly_dir)
        except socket.error, msg:
            self.socket.close()
            return {'status': 400, 'message': msg}

        # 5. Wait for response
        try:
            reply = self.socket.recv(1024)
        except socket.timeout, msg:
            self.socket.close()
            return {'status': 400, 'message': 'Could not generate Python code in time'}

        # 6. Close socket
        self.socket.close()

        # 7. Analyze response	
        try:
            reply = ast.literal_eval(reply)
        except Exception, e:
            return {'status': 400, 'message': e}

        if reply['status'] != 200:
            return reply

        # 8. Return generated code
        code = read_file(self.blockly_python_file)
        return {'status': 200, 'code': code}
