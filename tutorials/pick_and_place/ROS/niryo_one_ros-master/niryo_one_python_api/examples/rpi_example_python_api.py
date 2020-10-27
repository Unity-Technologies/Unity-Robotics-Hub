#!/usr/bin/env python

# To use the API, copy these 4 lines on each Python file you create
from niryo_one_python_api.niryo_one_api import *
import rospy
import time

rospy.init_node('niryo_one_example_python_api')

print "--- Start"

n = NiryoOne()

try:
    # Calibrate robot first
    n.calibrate_auto()
    print "Calibration finished !"

    time.sleep(1)

    # Test learning mode
    n.activate_learning_mode(False)

    # Test electromagnet on GPIO 2
    pin = GPIO_1A
    n.change_tool(TOOL_ELECTROMAGNET_1_ID)
    n.setup_electromagnet(TOOL_ELECTROMAGNET_1_ID, pin)
    n.activate_electromagnet(TOOL_ELECTROMAGNET_1_ID, pin)
    time.sleep(2)
    n.deactivate_electromagnet(TOOL_ELECTROMAGNET_1_ID, pin)

    n.activate_learning_mode(True)

    # Test digital I/O
    pin = GPIO_1B
    n.pin_mode(pin, PIN_MODE_OUTPUT)
    n.digital_write(pin, PIN_HIGH)
    time.sleep(2)
    n.digital_write(pin, PIN_LOW)
    time.sleep(2)
    n.pin_mode(pin, PIN_MODE_INPUT)

    pin = GPIO_1A
    n.pin_mode(pin, PIN_MODE_INPUT)
    for i in range(0, 10):
        print "Read pin GPIO 1_A : " + str(n.digital_read(pin))
        time.sleep(0.2)

    # Test vacuum pump
    # n.change_tool(TOOL_VACUUM_PUMP_1_ID)
    # n.pull_air_vacuum_pump(TOOL_VACUUM_PUMP_1_ID)
    # time.sleep(1)
    # n.push_air_vacuum_pump(TOOL_VACUUM_PUMP_1_ID)
except NiryoOneException as e:
    print e
    # handle exception here
    # you can also make a try/except for each command separately

print "--- End"
