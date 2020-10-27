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
    n.change_tool(TOOL_NONE)

    # Workspace
    markers_pose = []
    #markers_pose.append(n.list_to_robot_state_msg([0.31771132845, 0.0790193649892, 0.0213579726515, -0.427132245935,1.45567609886,0.448715574241]))
    #markers_pose.append(n.list_to_robot_state_msg([0.311330642353, -0.0812446470693, 0.0204788404218, -0.483396553297,1.2065375664,-0.249100212125]))
    #markers_pose.append(n.list_to_robot_state_msg([0.150904489662, -0.0801233550758, 0.0312721628892, 3.03847870326,1.36710811344,2.63988429935]))
    #markers_pose.append(n.list_to_robot_state_msg([0.152169323436, 0.0804375590226, 0.0313652150805, 2.99990155758, 1.35408319938,-2.663926013]))
    #n.create_workspace("my_workspace", markers_pose[0], markers_pose[1], markers_pose[2], markers_pose[3])

    print(
        "Workspace calibration: \nThe pose arguments have to be such that the calibration tip touches center of the markers.")
    for marker_number in range(4):
        raw_input("Marker " + str(marker_number) + " (Press enter when the robot is correctly positioned")
        markers_pose.append(n.get_arm_pose())
    n.create_workspace("my_workspace", markers_pose[0], markers_pose[1], markers_pose[2], markers_pose[3])
    print "Workspace calibration finished"
    print "Workspace markers poses:"
    print markers_pose

    print "Workspace ratio: " + str(n.get_workspace_ratio("my_workspace"))
    print "Workspace list: " + str(n.get_workspace_list())

    n.activate_learning_mode(False)
    print "Go to observation position"
    observation_pose = n.get_target_pose_from_rel("my_workspace", 0.35, 0.5, 0.5, 0)
    observation_pose = n.robot_state_msg_to_list(observation_pose)
    n.move_pose(*observation_pose)

    # Conveyor
    conveyor_id = CONVEYOR_ID_ONE
    n.set_conveyor(CONVEYOR_ID_ONE, True)
    n.control_conveyor(conveyor_id, True, 100, CONVEYOR_DIRECTION_FORWARD)

    # Pick/Place
    raw_input("Connect Gripper2 and press 'Enter'")
    n.change_tool(TOOL_GRIPPER_2_ID)
    object_found = False
    # Wait object
    while not object_found:
        n.wait(0.1)
        object_found, rel_pose, obj_shape, obj_color = n.detect_object("my_workspace", SHAPE_SQUARE, COLOR_RED)
    print "Object seen"

    n.control_conveyor(conveyor_id, True, 0, CONVEYOR_DIRECTION_FORWARD)
    # Wait conveyor speed=0
    conveyor_id, connection_state, running, speed, direction = n.get_conveyor_1_feedback()
    while not speed == 0:
        n.wait(0.1)
        conveyor_id, connection_state, running, speed, direction = n.get_conveyor_1_feedback()

    # Pick
    n.vision_pick("my_workspace", 0.0025, SHAPE_SQUARE, COLOR_RED)

    #Place
    place_pose = n.get_target_pose_from_rel("my_workspace", 0.0025, 0.5, 0.5, 0.45)
    place_pose_raw = n.robot_state_msg_to_list(place_pose)
    n.place_from_pose(*place_pose_raw)

    n.activate_learning_mode(True)

except NiryoOneException as e:
    print e
    # handle exception here
    # you can also make a try/except for each command separately

print "--- End"