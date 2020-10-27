"""
This script shows an example of how to use Niryo One's vision to
make a conditioning according to the objects' color.

The objects will be conditioned in a grid of dimension grid_dimension. The Y axis corresponds
to the Color : BLUE / RED / GREEN. It will be 3
The X axis corresponds to how many objects can be put on the same line before increasing
the conditioning height.
Once a line is completed, objects will be pack over the lower level
"""

from niryo_one_tcp_client import *

# -- MUST Change these variables
robot_ip_address = "192.168.1.202"  # IP address of the Niryo One
tool_used = RobotTool.GRIPPER_1  # Tool used for picking
workspace_name = "workspace_1"  # Robot's Workspace Name

# -- Can change these variables
grid_dimension = (3, 3)

# -- Should Change these variables
# The pose from where the image processing happen
observation_pose = PoseObject(
    x=0.20, y=0., z=0.3,
    roll=0.0, pitch=1.57, yaw=0.0,
)

# Center of the conditioning area
center_conditioning_pose = PoseObject(
    x=0.0, y=-0.25, z=0.12,
    roll=-0., pitch=1.57, yaw=-1.57
)

# Joints where the robot goes at the end of its process
sleep_joints = [0.0, 0.55, -1.2, 0.0, 0.0, 0.0]


# -- MAIN PROGRAM
def process(niryo_one_client):
    try_without_success = 0
    count_dict = {
        "BLUE": 0,
        "RED": 0,
        "GREEN": 0,
    }
    # Loop until too much failures
    while try_without_success < 3:
        # Moving to observation pose
        niryo_one_client.move_pose(*observation_pose.to_list())
        # Trying to get object from Niryo One API
        ret = niryo_one_client.vision_pick(workspace_name,
                                           height_offset=0.0,
                                           shape=Shape.ANY,
                                           color=Color.ANY)
        # Unpacking return result
        status, obj_found, shape, color = ret
        if not status or not obj_found:
            try_without_success += 1
            continue
        # Choose Y position according to Color
        color_val = color.value
        if color_val == "BLUE":
            offset_y = -1
        elif color_val == "RED":
            offset_y = 0
        else:
            offset_y = 1
        # Choose X & Z position according to how the color line is filled
        offset_x = count_dict[color_val] % grid_dimension[0] - grid_dimension[0] // 2
        offset_z = count_dict[color_val] // grid_dimension[0]
        # Going to place the object
        place_pose = center_conditioning_pose.copy_with_offsets(0.05 * offset_x, 0.05 * offset_y, 0.025 * offset_z)
        niryo_one_client.place_from_pose(*niryo_one_client.pose_to_list(place_pose))
        # Increment count
        count_dict[color_val] += 1
        try_without_success = 0


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
