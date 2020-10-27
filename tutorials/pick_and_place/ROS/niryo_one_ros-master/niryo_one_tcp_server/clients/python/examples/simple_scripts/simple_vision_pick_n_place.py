"""
This script shows 2 ways of making a vision pick and place.
You will need one workspace recorded in the Niryo studio
"""

# Imports
from niryo_one_tcp_client import *

# -- MUST Change these variables
robot_ip_address = "192.168.1.202"  # IP address of the Niryo One
tool_used = RobotTool.GRIPPER_2  # Tool used for picking
workspace_name = "workspace_4"  # Robot's Workspace Name

# -- Should Change these variables
# The pose from where the image processing happens
observation_pose = PoseObject(
    x=0.18, y=0.0, z=0.35,
    roll=0.0, pitch=1.57, yaw=-0.2,
)

# Center of the conditioning area
place_pose = PoseObject(
    x=0.0, y=-0.23, z=0.12,
    roll=0.0, pitch=1.57, yaw=-1.57
)

# Pose where the robot goes at the end of its process
sleep_joints = [0.0, 0.55, -1.2, 0.0, 0.0, 0.0]


# -- MAIN PROGRAM

def vision_pick_n_place_1(niryo_one_client):
    # Loop
    try_without_success = 0
    while try_without_success < 5:
        # Moving to observation pose
        niryo_one_client.move_pose(*observation_pose.to_list())
        # Trying to get target pose using camera
        ret = niryo_one_client.get_target_pose_from_cam(workspace_name,
                                                        height_offset=0.0,
                                                        shape=Shape.ANY,
                                                        color=Color.ANY)
        status, obj_found, obj_pose, shape, color = ret
        if not status or not obj_found:
            try_without_success += 1
            continue

        # At this point, we can do specific operation on the obj_pose if we want
        # to catch the object with a tilted angle for instance
        # ---
        # ---

        # Everything is good, so we going to object
        niryo_one_client.pick_from_pose(*obj_pose.to_list())

        niryo_one_client.place_from_pose(*place_pose.to_list())
        break


def vision_pick_n_place_2(niryo_one_client):
    # Loop
    try_without_success = 0
    while try_without_success < 5:
        # Moving to observation pose
        niryo_one_client.move_pose(*observation_pose.to_list())
        # Trying to pick target using camera
        ret = niryo_one_client.vision_pick(workspace_name,
                                           height_offset=0.0,
                                           shape=Shape.ANY,
                                           color=Color.ANY)
        status, obj_found, shape_ret, color_ret = ret
        if not status or not obj_found:
            try_without_success += 1
            continue
        # Vision pick has succeed which means that Niryo One should have already catch the object !

        # Everything is good, so we going to place the object
        niryo_one_client.place_from_pose(*place_pose.to_list())
        break


if __name__ == '__main__':
    # Connect to robot
    client = NiryoOneClient()
    client.connect(robot_ip_address)
    # Changing tool
    client.change_tool(tool_used)
    # Calibrate robot if robot needs calibration
    client.calibrate(CalibrateMode.AUTO)
    # Launching main process
    print("Starting Version 1")
    vision_pick_n_place_1(client)
    print("Starting Version 2")
    vision_pick_n_place_2(client)
    # Ending
    client.move_joints(*sleep_joints)
    client.set_learning_mode(True)
    # Releasing connection
    client.quit()
