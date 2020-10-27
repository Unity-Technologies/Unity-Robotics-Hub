"""
This script shows few functions you can use with the TCP API
"""

# Imports
from niryo_one_tcp_client import *

# Connecting to robot
niryo_one_client = NiryoOneClient()
niryo_one_client.connect("192.168.1.202")  # =< Replace by robot ip address

# Trying to calibrate
status, data = niryo_one_client.calibrate(CalibrateMode.AUTO)
if status is False:
    print("Error: " + data)

# Getting pose
status, data = niryo_one_client.get_pose()
initial_pose = None
if status is True:
    initial_pose = data
else:
    print("Error: " + data)

# Move Joints
status, data = niryo_one_client.move_joints(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
if status is False:
    print("Error: " + data)

# Shift pose
status, data = niryo_one_client.shift_pose(RobotAxis.Y, 0.15)
if status is False:
    print("Error: " + data)

# Going back to initial pose
if initial_pose is not None:
    status, data = niryo_one_client.move_pose(*niryo_one_client.pose_to_list(initial_pose))
    if status is False:
        print("Error: " + data)

# Getting hardware information
status, data = niryo_one_client.get_digital_io_state()
if status is True:
    digital_pin_array = data
    for digital_pin in digital_pin_array:
        print("Pin: " + digital_pin.pin_id
              + ", name: " + digital_pin.name
              + ", mode: " + str(digital_pin.mode)
              + ", state: " + str(digital_pin.state))

# Turning learning mode ON
status, data = niryo_one_client.set_learning_mode(True)
if status is False:
    print("Error: " + data)

niryo_one_client.quit()
