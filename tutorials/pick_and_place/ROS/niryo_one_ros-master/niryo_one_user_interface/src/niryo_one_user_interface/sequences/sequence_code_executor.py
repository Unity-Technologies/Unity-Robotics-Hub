#!/usr/bin/env python

import rospy
import os, signal
import subprocess

from std_msgs.msg import String
from std_srvs.srv import SetBool


def create_directory(directory_path):
    if not os.path.exists(directory_path):
        os.makedirs(directory_path)


def save_to_file(filename, text):
    with open(filename, 'w') as target:
        target.write(text)


def read_file(filename):
    with open(filename, 'r') as target:
        return target.read()


class SequenceCodeExecutor:

    def __init__(self):
        self.blockly_dir = rospy.get_param("~sequence_code_to_execute_path")
        self.python_file = str(self.blockly_dir) + '/generated_code.py'
        create_directory(self.blockly_dir)
        self.process = None
        self.cancel_flag = False
        self.is_paused = False

        # Highlight publisher (to highlight blocks in Blockly interface)
        self.highlight_block_publisher = rospy.Publisher('/niryo_one/blockly/highlight_block', String, queue_size=10)

    def execute_generated_code(self, python_code):
        # 1. Check if still executing
        if self.is_executing_code():
            return {'status': 400, 'message': "A generated code is already running"}

        # 2. Save code in python file
        save_to_file(self.python_file, python_code)

        # 3. Start executing code with process
        self.process = subprocess.Popen(['python', str(self.python_file)],
                                        stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        # 4. Wait for process to finish
        output = self.process.communicate()
        self.is_paused = False

        # 5. Check if cmd was canceled
        if self.cancel_flag:
            self.cancel_flag = False
            self.stop_robot_action()
            return {'status': 300, 'message': 'Execution of Sequence has been canceled'}

        # 6. Check error in output
        error = output[1]

        if error != '':
            str_to_find = 'niryo_one_python_api.niryo_one_api.NiryoOneException:'
            exception_index = error.find(str_to_find)

            # found NiryoOneException -> Niryo error
            if exception_index != -1:
                exception_index = exception_index + len(str_to_find) + 1
                return {'status': 400, 'message': error[exception_index:].rstrip()}
            # Any other exception 
            else:
                return {'status': 400, 'message': 'Error while executing generated code : ' + str(error)}

        # 7. Return OK if no error
        return {'status': 200, 'message': 'Successfully executed Sequence code'}

    def cancel_execution(self):
        if self.process is None:
            return
        if self.is_executing_code():
            rospy.logwarn("Stopping sequence code execution")
            self.cancel_flag = True
            self.resume_execution()
            self.process.terminate()
            # Publish empty block ID for Blockly
            # Only visual, no functionality here
            msg = String()
            msg.data = ''
            self.highlight_block_publisher.publish(msg)

    def break_point(self):
        if self.process is None:
            return
        if self.is_executing_code():
            rospy.logwarn("Break point - pause sequence interruption")
            os.kill(self.process.pid, signal.SIGSTOP)
            self.is_paused = True

    def resume_execution(self):
        if self.process is None:
            return
        rospy.logwarn("Resume sequence execution")
        os.kill(self.process.pid, signal.SIGCONT)
        self.is_paused = False

    def is_execution_paused(self):
        return self.is_paused

    @staticmethod
    def stop_robot_action():
        # Stop current move command
        try:
            rospy.wait_for_service('/niryo_one/commander/stop_command', 1)
            stop_cmd = rospy.ServiceProxy('/niryo_one/commander/stop_command', SetBool)
            stop_cmd()
        except (rospy.ServiceException, rospy.ROSException), e:
            pass

    def is_executing_code(self):
        if self.process is None:
            return False
        return_code = self.process.poll()
        return return_code is None
