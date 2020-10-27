import serial
import time
import threading
import rospy


class JevoisSerial:

    def __init__(self, port,
                 baudrate=115200, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self.keep_alive = False
        self.read_thread = None
        self.data_callback = None
        self.error_callback = None

    def set_callbacks(self, data_callback, error_callback):
        self.data_callback = data_callback
        self.error_callback = error_callback

    def start(self):
        rospy.loginfo("Starting Serial communication")
        try:
            self.serial = serial.Serial(port=self.port,
                                        baudrate=self.baudrate,
                                        timeout=self.timeout)
            self.keep_alive = True
            self.read_thread = threading.Thread(target=self.read_loop)
            self.read_thread.start()
            return True, ''
        except serial.serialutil.SerialException as e:
            self.keep_alive = False
            return False, str(e)

    def send_command(self, cmd):
        self.serial.write(cmd + '\n')

    def stop(self):
        self.keep_alive = False
        if self.serial is not None:
            rospy.loginfo("Closing Serial communication")
            self.serial.close()

    def reconnect(self):
        while self.keep_alive:
            rospy.loginfo("Trying to reconnect...")
            if self.serial:
                self.serial.close()
                try:
                    self.serial = serial.Serial(port=self.port,
                                                baudrate=self.baudrate,
                                                timeout=self.timeout)
                    rospy.loginfo("Reconnected!")
                    return True
                except:
                    rospy.logwarn(
                        "Failed to reconnect, will retry in one second...")
                    time.sleep(1)
        return False

    def read_loop(self):
        rospy.loginfo("Starting serial read loop")
        out = ''
        while self.keep_alive:
            try:
                read_value = self.serial.read(1)
            except IOError as e:
                if self.error_callback:
                    self.error_callback(str(e))
                else:
                    self.keep_alive = False
                out = ''
            if read_value == '\n' or read_value == '\r':
                if out != '':
                    if self.data_callback:
                        self.data_callback(str(out))
                out = ''
            else:
                out += read_value
