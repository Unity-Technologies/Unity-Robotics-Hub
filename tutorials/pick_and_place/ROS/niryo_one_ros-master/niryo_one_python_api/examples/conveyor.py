#!/usr/bin/env python

# To use the API, copy these 4 lines on each Python file you create
from niryo_one_python_api.niryo_one_api import *
import math
import rospy
import time

rospy.init_node('niryo_one_example_python_api')

print "--- Start"

n = NiryoOne()

try:
    # Calibrate robot first
    n.calibrate_auto()
    print "Calibration finished !"

    n.activate_learning_mode(True)

    # Conveyor
    conveyor_id = CONVEYOR_ID_ONE
    
    # Setup conveyor
    n.set_conveyor(conveyor_id, True)
    print "Conveyor initialized"

    # Turn on conveyor forward
    n.control_conveyor(conveyor_id, True, 100, CONVEYOR_DIRECTION_FORWARD)
    print "Start conveyor forward at 100% speed"
    n.wait(1)
    conveyor_id, connection_state, is_running, speed, direction = n.get_conveyor_1_feedback()
    print "conveyor_id: " + str(conveyor_id)
    print "connection_state: " + str(connection_state)
    print "is_running: " + str(is_running)
    print "speed: " + str(speed)
    print "direction: " + str(direction)
    raw_input("Press enter for next function")

    # Turn off conveyor
    n.control_conveyor(conveyor_id, True, 0, CONVEYOR_DIRECTION_FORWARD)
    print "Stop conveyor"
    raw_input("Press enter for next function")

    # Turn on conveyor backward at 50% speed
    n.control_conveyor(conveyor_id, True, 50, CONVEYOR_DIRECTION_BACKWARD)
    print "Start conveyor backward at 50% speed"
    raw_input("Press enter for next function")

    n.control_conveyor(conveyor_id, False, 50, CONVEYOR_DIRECTION_BACKWARD)
    print "Stop conveyor"
    raw_input("Press enter for next function and update conveyor")

    n.update_conveyor_id(conveyor_id,CONVEYOR_ID_TWO)
    n.control_conveyor(CONVEYOR_ID_TWO, True, 30, CONVEYOR_DIRECTION_FORWARD)
    n.wait(5)
    n.control_conveyor(CONVEYOR_ID_TWO, False, 30, CONVEYOR_DIRECTION_FORWARD)
    n.set_conveyor(CONVEYOR_ID_TWO, False)

except NiryoOneException as e:
    print e
    # handle exception here
    # you can also make a try/except for each command separately

print "--- End"