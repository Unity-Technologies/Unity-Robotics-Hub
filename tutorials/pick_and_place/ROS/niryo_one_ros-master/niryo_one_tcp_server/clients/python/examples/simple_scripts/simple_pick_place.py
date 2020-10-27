"""
This file shows how to use Python TCP API to do a simple pick and place with the Niryo One
"""

# Imports
from niryo_one_tcp_client import *

# -- MUST Change these variables
robot_ip_address = "192.168.1.202"  # IP address of the Niryo One
gripper_used = RobotTool.GRIPPER_2  # Tool used for picking

# -- Should Change these variables
# The pick pose
pick_pose = PoseObject(
    x=0.25, y=0., z=0.14,
    roll=-0., pitch=1.57, yaw=0.0,
)
# The Place pose
place_pose = PoseObject(
    x=-0.01, y=-0.23, z=0.12,
    roll=-0., pitch=1.57, yaw=-1.57)
# Sleep Joints
sleep_joints = [0.0, 0.55, -1.2, 0.0, 0.0, 0.0]


# Version 1
def pick_n_place_version_1(niryo_one_client):
    height_offset = 0.05  # Offset according to Z-Axis to go over pick & place poses
    gripper_speed = 400

    # Going Over Object
    niryo_one_client.move_pose(pick_pose.x, pick_pose.y, pick_pose.z + height_offset,
                               pick_pose.roll, pick_pose.pitch, pick_pose.yaw)
    # Opening Gripper
    niryo_one_client.open_gripper(gripper_used, gripper_speed)
    # Going to picking place and closing gripper
    niryo_one_client.move_pose(pick_pose.x, pick_pose.y, pick_pose.z,
                               pick_pose.roll, pick_pose.pitch, pick_pose.yaw)
    niryo_one_client.close_gripper(gripper_used, gripper_speed)

    # Raising
    niryo_one_client.move_pose(pick_pose.x, pick_pose.y, pick_pose.z + height_offset,
                               pick_pose.roll, pick_pose.pitch, pick_pose.yaw)

    # Going Over Place pose
    niryo_one_client.move_pose(place_pose.x, place_pose.y, place_pose.z + height_offset,
                               place_pose.roll, place_pose.pitch, place_pose.yaw)
    # Going to Place pose
    niryo_one_client.move_pose(place_pose.x, place_pose.y, place_pose.z,
                               place_pose.roll, place_pose.pitch, place_pose.yaw)
    # Opening Gripper
    niryo_one_client.open_gripper(gripper_used, gripper_speed)
    # Raising
    niryo_one_client.move_pose(place_pose.x, place_pose.y, place_pose.z + height_offset,
                               place_pose.roll, place_pose.pitch, place_pose.yaw)


# Version 2
def pick_n_place_version_2(niryo_one_client):
    height_offset = 0.05  # Offset according to Z-Axis to go over pick & place poses
    gripper_speed = 400

    pick_pose_high = pick_pose.copy_with_offsets(z_offset=height_offset)
    place_pose_high = place_pose.copy_with_offsets(z_offset=height_offset)

    # Going Over Object
    niryo_one_client.move_pose(*niryo_one_client.pose_to_list(pick_pose_high))
    # Opening Gripper
    niryo_one_client.open_gripper(gripper_used, gripper_speed)
    # Going to picking place and closing gripper
    niryo_one_client.move_pose(*niryo_one_client.pose_to_list(pick_pose))
    niryo_one_client.close_gripper(gripper_used, gripper_speed)
    # Raising
    niryo_one_client.move_pose(*niryo_one_client.pose_to_list(pick_pose_high))

    # Going Over Place pose
    niryo_one_client.move_pose(*niryo_one_client.pose_to_list(place_pose_high))
    # Going to Place pose
    niryo_one_client.move_pose(*niryo_one_client.pose_to_list(place_pose))
    # Opening Gripper
    niryo_one_client.open_gripper(gripper_used, gripper_speed)
    # Raising
    niryo_one_client.move_pose(*niryo_one_client.pose_to_list(place_pose_high))


# Version 3
def pick_n_place_version_3(niryo_one_client):
    # Pick
    niryo_one_client.pick_from_pose(*pick_pose.to_list())
    # Place
    niryo_one_client.place_from_pose(*place_pose.to_list())


# -- MAIN PROGRAM

if __name__ == '__main__':
    # Connect to robot
    client = NiryoOneClient()
    client.connect(robot_ip_address)
    # Changing tool
    client.change_tool(gripper_used)
    # Calibrate robot if robot needs calibration
    client.calibrate(CalibrateMode.AUTO)

    # - Main process
    # Version 1
    print("Playing version 1")
    pick_n_place_version_1(client)
    # Version 2
    print("Playing version 2")
    pick_n_place_version_2(client)
    # Version 3
    print("Playing version 3")
    pick_n_place_version_3(client)

    # Ending
    client.move_joints(*sleep_joints)
    client.set_learning_mode(True)
    # Releasing connection
    client.quit()
