from jevois_serial import JevoisSerial
from modules.jevois_module import JevoisModule

import time
import threading
import rospy


class JevoisManager:

    def __init__(self, jevois_serial, jevois_camera):
        self.serial = jevois_serial
        self.camera = jevois_camera
        self.module_list = []
        self.current_module = None
        self.load_lock = threading.RLock()
        self.data_callback = None

    def set_data_callback(self, callback):
        self.data_callback = callback

    def add_module(self, module):
        self.module_list.append(module)

    def load_module(self, module_name):
        # 0. if module is the same, return
        if self.current_module and \
                self.current_module.name == module_name:
            return True, "Module already loaded"

        module_to_load = None
        # 1. Check that module exists
        for module in self.module_list:
            if module.name == module_name:
                module_to_load = module
        if module_to_load is None:
            message = "Could not find module " + str(module_name)
            return False, message

        rospy.loginfo("Found module " + str(module_name) + ", now loading module...")

        with self.load_lock:
            # 2. if a module still loaded, unload it first
            if self.current_module:
                self.unload_current_module()
            self.current_module = module_to_load

            # 3. set guvcview fps
            rospy.loginfo("Set camera FPS to " + str(self.current_module.fps))
            self.camera.set_guvcview_fps(self.current_module.fps)

            # 4. start guvcview if needed
            if self.current_module.should_start_guvcview:
                rospy.loginfo("Start guvcview")
                self.camera.start_guvcview(self.current_module.res_x,
                    self.current_module.res_y, self.current_module.video_format)
                rospy.sleep(1)

            # 5. send params to serial
            rospy.loginfo("Send module params to Jevois serial")
            params = self.current_module.get_serial_params()
            for param in params:
                self.serial.send_command(param)

            rospy.loginfo("Module successfully loaded.")
            return True, "Module has been started"

    def unload_current_module(self):
        # 1. Check if a module is set
        if self.current_module is None:
            return

        with self.load_lock:
            # 2. stop guvcview
            self.camera.stop_guvcview()
            # let some time for guvcview to empty buffers
            # to avoid possible crashes
            if self.current_module.should_start_guvcview:
                time.sleep(0.1)

            # 3. Reset current module
            self.current_module = None

    def start(self):
        # setup serial communication
        success, message = self.serial.start()
        if success:
            self.serial.set_callbacks(
                self.callback_data_serial,
                self.callback_on_serial_error)
            # setup camera
            success, message = self.camera.setup()
        return success, message

    def stop(self):
        self.serial.stop()
        self.camera.stop_guvcview()

    def callback_data_serial(self, data):
        # rospy.loginfo(data)
        # remove info that is not data related to modules
        if data.startswith('OK') or \
                data.startswith('INF') or \
                data.startswith('ERR'):
            return
        if self.data_callback:
            self.data_callback(data)

    def callback_on_serial_error(self, error):
        rospy.logerr("ERROR: " + str(error))
        self.camera.stop_guvcview()
        if self.serial.reconnect():
            if self.current_module:
                module_to_load = self.current_module.name
                self.current_module = None
                self.load_module(module_to_load)
