#!/usr/bin/env python

# To use the API, copy these 4 lines on each Python file you create
from niryo_one_python_api.niryo_one_api import *
import rospy
import time
import math

rospy.init_node('niryo_one_example_python_api')

print "--- Start"

n = NiryoOne()

try:
    # Calibrate robot first
    n.calibrate_auto()
    #n.calibrate_manual()
    print "Calibration finished !\n"

    time.sleep(1)

    # Test learning mode
    n.activate_learning_mode(False)
    print "Learning mode activated? "
    print n.get_learning_mode()

    # Move
    n.set_arm_max_velocity(30)

    joint_target = [math.radians(45), -math.pi/4.0, math.pi/4.0, 1.57/2, 0.0, 0.0]
    n.move_joints(joint_target)

    n.move_pose(0.2, 0, 0.2, 0, math.radians(90), 0)
    next_pose = [0.25, 0.1, 0.2, 0.0, math.radians(90), 0.0]
    n.move_pose(*next_pose)

    n.shift_pose(AXIS_Y, 0.1)
    n.shift_pose(ROT_YAW, math.radians(-45))

    #Robot positions
    saved_positions = n.get_saved_position_list()
    print "\nSaved positions: "
    print saved_positions

    current_joints_array = n.get_joints()
    print "\nCurrent joints: "
    print current_joints_array
    current_position = n.get_arm_pose()
    print "\nCurrent pose: "
    print current_position


    # I/O
    pin = GPIO_1B
    n.pin_mode(pin, PIN_MODE_OUTPUT)
    n.digital_write(pin, PIN_HIGH)
    n.wait(0.2)
    n.digital_write(pin, PIN_LOW)
    n.wait(0.2)
    n.pin_mode(pin, PIN_MODE_INPUT)

    pin = GPIO_1A
    n.pin_mode(pin, PIN_MODE_INPUT)
    print "\nRead pin GPIO 1_A 1: " + str(n.digital_read(pin))

    print "\nCurrent IO states: "
    print n.get_digital_io_state()


    #End effector

    # Test gripper
    n.change_tool(TOOL_GRIPPER_2_ID)
    print "\nCurrent tool id:"
    print n.get_current_tool_id()
    n.close_gripper(TOOL_GRIPPER_2_ID,500)
    n.wait(0.2)
    n.open_gripper(TOOL_GRIPPER_2_ID,500)

    # Test vacuum pump
    # n.change_tool(TOOL_VACUUM_PUMP_1_ID)
    # n.pull_air_vacuum_pump(TOOL_VACUUM_PUMP_1_ID)
    # time.sleep(1)
    # n.push_air_vacuum_pump(TOOL_VACUUM_PUMP_1_ID)

    # Test electromagnet on GPIO 2
    # pin = GPIO_1A
    # n.change_tool(TOOL_ELECTROMAGNET_1_ID) # Mount
    # n.setup_electromagnet(TOOL_ELECTROMAGNET_1_ID, pin)
    # n.activate_electromagnet(TOOL_ELECTROMAGNET_1_ID, pin)
    # time.sleep(2)
    # n.deactivate_electromagnet(TOOL_ELECTROMAGNET_1_ID, pin)

    n.change_tool(TOOL_NONE) # Unmount

    # Others
    print "\nHardware status: "
    print n.get_hardware_status()

    n.activate_learning_mode(True)


except NiryoOneException as e:
    print e
    # handle exception here
    # you can also make a try/except for each command separately

print "--- End"
