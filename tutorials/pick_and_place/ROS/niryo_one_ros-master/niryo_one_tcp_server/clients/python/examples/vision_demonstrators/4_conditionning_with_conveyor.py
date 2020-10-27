from niryo_one_tcp_client import *

# -- MUST Change these variables
tool_used = RobotTool.GRIPPER_1
robot_ip_address = "192.168.1.200"
workspace_name = "wks_conveyor"

# -- Can change these variables
grid_dimension = (3, 3)
conveyor_id = ConveyorID.ID_1

# -- Should Change these variables
observation_pose = PoseObject(
    x=0.20, y=0., z=0.3,
    roll=0.0, pitch=1.57, yaw=0.0,
)

center_conditioning_pose = PoseObject(
    x=0.0, y=0.25, z=0.13,
    roll=0.0, pitch=1.57, yaw=1.57,
)

# Joints where the robot goes at the end of its process
sleep_joints = [0.0, 0.55, -1.2, 0.0, 0.0, 0.0]


def process(niryo_one_client):
    def run_conveyor():
        niryo_one_client.control_conveyor(conveyor_id, control_on=True, speed=50, direction=ConveyorDirection.FORWARD)

    place_count = 0
    while place_count < 9:
        # Turning conveyor on
        run_conveyor()
        # Moving to observation pose
        niryo_one_client.move_pose(*observation_pose.to_list())
        # Check if object is in the workspace with Niryo One API
        ret = niryo_one_client.detect_object(workspace_name,
                                             shape=Shape.ANY,
                                             color=Color.ANY)
        # Unpacking return result
        status, obj_found, pos_array, shape, color = ret
        if not status:  # Aborting iteration if issue
            break
        niryo_one_client.wait(0.5)  # Wait to let the conveyor turn a bit
        if not obj_found:
            continue
        # Stopping conveyor
        niryo_one_client.stop_conveyor(conveyor_id)
        # Realizing a vision pick
        ret = niryo_one_client.vision_pick(workspace_name,
                                           height_offset=0.0,
                                           shape=Shape.ANY,
                                           color=Color.ANY)
        # Unpacking return result
        status, obj_found, shape, color = ret
        if not status or not obj_found:  # If visual pick did not work
            continue
        # Calculating offset relative to conditioning center position
        offset_x = place_count % grid_dimension[0] - grid_dimension[0] // 2
        offset_y = (place_count // grid_dimension[1]) % 3 - grid_dimension[1] // 2
        place_pose = center_conditioning_pose.copy_with_offsets(0.05 * offset_x, 0.05 * offset_y)
        # Going to place
        niryo_one_client.place_from_pose(*place_pose.to_list())
        place_count += 1
    # Stopping conveyor
    niryo_one_client.stop_conveyor(conveyor_id)

    # Going to initial Observation pose
    niryo_one_client.move_pose(*observation_pose.to_list())


if __name__ == '__main__':
    # Connect to robot
    client = NiryoOneClient()
    client.connect(robot_ip_address)
    # Changing tool
    client.change_tool(tool_used)
    # Activating conveyor if needed
    client.activate_conveyor(conveyor_id)
    # Calibrate robot if robot needs calibration
    client.calibrate(CalibrateMode.AUTO)
    # Launching main process
    process(client)
    # Ending
    client.move_joints(*sleep_joints)
    client.set_learning_mode(True)
    # Deactivating the connection to the conveyor
    client.deactivate_conveyor(conveyor_id)
    # Releasing connection
    client.quit()
