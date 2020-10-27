#!/usr/bin/env python

from niryo_one_tcp_client import *

niryo_one_client = NiryoOneClient()
niryo_one_client.connect("10.10.10.10")  # =< Replace by robot ip address

initial_pose = None

status, data = niryo_one_client.calibrate(CalibrateMode.AUTO)
if status is False:
    print("Error: " + data)

status, data = niryo_one_client.get_pose()
if status is True:
    initial_pose = data
else:
    print("Error: " + data)

status, data = niryo_one_client.move_joints(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
if status is False:
    print("Error: " + data)

status, data = niryo_one_client.shift_pose(RobotAxis.Y, 0.15)
if status is False:
    print("Error: " + data)

if initial_pose is not None:
    status, data = niryo_one_client.move_pose(initial_pose.x, initial_pose.y, initial_pose.z,
                                              initial_pose.roll, initial_pose.pitch, initial_pose.yaw)
    if status is False:
        print("Error: " + data)

status, data = niryo_one_client.get_digital_io_state()
if status is True:
    digital_pin_array = data
    for digital_pin in digital_pin_array:
        print("Pin: " + digital_pin.pin_id
              + ", name: " + digital_pin.name
              + ", mode: " + str(digital_pin.mode)
              + ", state: " + str(digital_pin.state))

status, data = niryo_one_client.set_learning_mode(True)
if status is False:
    print("Error: " + data)

niryo_one_client.quit()
