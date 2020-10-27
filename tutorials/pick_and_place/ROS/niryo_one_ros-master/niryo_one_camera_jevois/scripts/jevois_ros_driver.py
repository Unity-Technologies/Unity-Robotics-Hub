#!/usr/bin/env python

import rospy
import os
from niryo_one_msgs.srv import SetString
from std_msgs.msg import String

from jevois_serial import JevoisSerial
from jevois_camera import JevoisCamera
from jevois_manager import JevoisManager

from modules.qr_code_module import QRCodeModule
from modules.dice_counter_module import DiceCounterModule

SERIAL_PORT = '/dev/ttyACM0'
USER_HOME_DIR = os.path.expanduser('~')
VIDEO_DEVICES_PATH = '/sys/class/video4linux/'

class JevoisRosDriver:

    def ros_callback_set_module(self, req):
        module_name = req.value
        success = False
        message = ''
        if module_name == '':
            self.jevois_manager.unload_current_module()
            success = True
            message = "Current module has been unloaded"
        else:
            success, message = self.jevois_manager.load_module(module_name)
        return {'status': 200 if success else 400, 'message': message}

    def callback_data(self, data):
        self.data_publisher.publish(data)

    def __init__(self, jevois_manager):
        self.jevois_manager = jevois_manager

        self.change_module_server = rospy.Service(
            '/niryo_one/jevois/set_module', SetString,
            self.ros_callback_set_module)

        self.data_publisher = rospy.Publisher(
            '/niryo_one/jevois/data', String, queue_size=10)

        self.jevois_manager.set_data_callback(self.callback_data)

    def start(self):
        return self.jevois_manager.start()

    def stop(self):
        self.jevois_manager.stop()


if __name__ == "__main__":
    rospy.init_node("jevois_ros_driver")
    serial = JevoisSerial(port=SERIAL_PORT)
    camera = JevoisCamera(video_devices_path=VIDEO_DEVICES_PATH,
                          user_home_dir=USER_HOME_DIR)
    jevois_manager = JevoisManager(serial, camera)
    jevois_ros_driver = JevoisRosDriver(jevois_manager)
    rospy.on_shutdown(jevois_ros_driver.stop)

    # Try to connect to camera serial
    # Program will exit on failure
    # Once the connection is made, if it's broken later,
    # the program will try to reconnect
    success, message = jevois_ros_driver.start()
    if not success:
        rospy.logerr("Failed to start Jevois ROS driver:")
        rospy.logerr(message)
        jevois_ros_driver.stop()
        exit()

    # Add modules to JevoisManager and give them a name
    # For each module you create, add a line below
    qrcode = QRCodeModule("qr_code")
    dicecounter = DiceCounterModule("dice_counter")

    jevois_manager.add_module(qrcode)
    jevois_manager.add_module(dicecounter)

    # No module is loaded by default
    # Uncomment this to load a module on startup
    # jevois_manager.load_module("qr_code")

    rospy.loginfo("Jevois camera ROS Driver has been started")
    rospy.spin()
