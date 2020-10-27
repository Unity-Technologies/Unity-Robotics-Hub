"""
This script shows an example of how to use Niryo One's vision with 2
workspaces : 1 for picking & 1 for packing

The robot will firstly detect one pack place, then it will take one object
from the picking area. This object will be pack and then, the pack will be
place on the side of the working area
"""

from niryo_one_tcp_client import *

# -- MUST Change these variables
robot_ip_address = "192.168.1.202"  # IP address of the Niryo One
tool_used = RobotTool.GRIPPER_1  # Tool used for picking
workspace_pick_name = "workspace_1"  # Robot's picking Workspace Name
workspace_place_name = "workspace_place"  # Robot's placing Workspace Name

# -- Should Change these variables
# The pose from where the image processing for picking happens
observation_pose_wks_pick = PoseObject(
    x=0.20, y=0., z=0.3,
    roll=0.0, pitch=1.57, yaw=0.0,
)
# The pose from where the image processing for placing happens
observation_pose_wks_place = PoseObject(
    x=0.00, y=-0.2, z=0.35,
    roll=0.0, pitch=1.57, yaw=-1.57,
)
# First conditioning pose in the cubes row
pos_conditioning1 = PoseObject(
    x=0.1, y=-0.29, z=0.12,
    roll=-0., pitch=1.57, yaw=-1.57
)
# First conditioning pose in the circles row
pos_conditioning2 = PoseObject(
    x=-0.145, y=-0.282, z=0.12,
    roll=-0., pitch=1.57, yaw=-1.57
)

# Joints where the robot goes at the end of its process
sleep_joints = [0.0, 0.55, -1.2, 0.0, 0.0, 0.0]


def process(niryo_one_client):
    catch_count_category_1 = 0
    catch_count_category_2 = 0
    try_without_success = 0
    # Loop until too much failures or 6 objects have been already picked
    while try_without_success < 3 and catch_count_category_1 + catch_count_category_2 < 6:
        obj_pose = None
        # Moving to observation pose over the packing workspace
        niryo_one_client.move_pose(*observation_pose_wks_place.to_list())
        # Trying to get place pose from Niryo One API
        ret = niryo_one_client.get_target_pose_from_cam(workspace_place_name,
                                                        height_offset=0.0,
                                                        shape=Shape.ANY,
                                                        color=Color.ANY)
        # Unpacking return result
        status, obj_found, place_pose, shape, color = ret
        if not status or not obj_found:  # Aborting iteration if issue
            try_without_success += 1
            continue

        # Moving to observation pose over the picking workspace
        niryo_one_client.move_pose(*observation_pose_wks_pick.to_list())

        try_without_success_within = 0
        obj_found_within = False
        should_leave = False
        shape_obj = None
        while not obj_found_within:  # Looping over picking area
            ret = niryo_one_client.get_target_pose_from_cam(workspace_pick_name,
                                                            height_offset=0.0,
                                                            shape=Shape.ANY,
                                                            color=Color.ANY)
            status, obj_found_within, obj_pose, shape_obj, color_obj = ret
            if not status or not obj_found_within:
                try_without_success_within += 1
                if try_without_success_within > 3:
                    should_leave = True
                    break
                continue
        if should_leave:  # If nothing has been found, nothing more to pack, leave
            break
        # Everything is good, so we going to pick the object
        niryo_one_client.pick_from_pose(*obj_pose.to_list())

        # Packing
        place_pose_high = place_pose.copy_with_offsets(z_offset=0.05)
        niryo_one_client.move_pose(*place_pose_high.to_list())
        niryo_one_client.move_pose(*niryo_one_client.pose_to_list(place_pose.copy_with_offsets(z_offset=0.01)))
        # Opening gripper to pack
        niryo_one_client.open_gripper(tool_used, 450)
        # Going down to pick package
        niryo_one_client.move_pose(*niryo_one_client.pose_to_list(place_pose.copy_with_offsets(z_offset=-0.011)))
        niryo_one_client.close_gripper(tool_used, 450)
        niryo_one_client.move_pose(*niryo_one_client.pose_to_list(place_pose.copy_with_offsets(z_offset=0.05)))

        # Getting place position
        if shape_obj == Shape.SQUARE:
            target_pose = pos_conditioning1.copy_with_offsets(
                y_offset=0.05 * catch_count_category_1 % 3,
            )
            catch_count_category_1 += 1
        else:
            target_pose = pos_conditioning2.copy_with_offsets(
                y_offset=0.05 * catch_count_category_2 % 3,
            )
            catch_count_category_2 += 1

        niryo_one_client.place_from_pose(*target_pose.to_list())

    niryo_one_client.move_pose(*observation_pose_wks_pick.to_list())


if __name__ == '__main__':
    # Connect to robot
    client = NiryoOneClient()
    client.connect(robot_ip_address)
    # Changing tool
    client.change_tool(tool_used)
    # Calibrate robot if robot needs calibration
    client.calibrate(CalibrateMode.AUTO)
    # Launching main process
    process(client)
    # Ending
    client.move_joints(*sleep_joints)
    client.set_learning_mode(True)
    # Releasing connection
    client.quit()
