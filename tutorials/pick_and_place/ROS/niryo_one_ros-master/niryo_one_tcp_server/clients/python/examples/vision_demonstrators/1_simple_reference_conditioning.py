"""
This script shows an example of how to use Niryo One's vision to
make a conditioning with any of the objects supplied with the Vision Kit
The script works in 2 ways:
- One where all the vision process is made on the robot
- One where the vision process is made on the computer

The first one shows how easy it is to use Niryo One's vision kit with Python TCP API
The second demonstrates a way to do image processing from user's computer. It highlights
the fact that the user can imagine every type of process on his computer.

The objects will be conditioned in a grid of dimension grid_dimension. If the grid is completed,
objects will be pack over the lower level
"""

from niryo_one_tcp_client import *
from niryo_one_camera import *

# -- MUST Change these variables
robot_ip_address = "192.168.1.202"  # IP address of the Niryo One
tool_used = RobotTool.GRIPPER_1  # Tool used for picking
workspace_name = "workspace_1"  # Robot's Workspace Name

# -- Can Change these variables
grid_dimension = (3, 3)  # conditioning grid dimension
vision_process_on_robot = True  # boolean to indicate if the image processing append on the Robot
display_stream = True  # Only used if vision on computer

# -- Should Change these variables
# The pose from where the image processing happens
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
    # Initializing variables
    obj_pose = None
    try_without_success = 0
    count = 0
    if vision_process_on_robot:
        status_calib, mtx, dist = niryo_one_client.get_calibration_object()
    else:
        status_calib = False
        mtx = dist = None
    # Loop
    while try_without_success < 5:
        # Moving to observation pose
        niryo_one_client.move_pose(*observation_pose.to_list())

        if vision_process_on_robot:
            ret = niryo_one_client.get_target_pose_from_cam(workspace_name,
                                                            height_offset=0.0,
                                                            shape=Shape.ANY,
                                                            color=Color.ANY)
            status, obj_found, obj_pose, shape, color = ret

        else:  # Home made image processing
            status, img_compressed = niryo_one_client.get_img_compressed()
            if status is not True:
                print("error with Niryo One's service")
                break
            img = uncompress_image(img_compressed)
            if status_calib:
                img = undistort_image(img, mtx, dist)
            # extracting working area
            im_work = extract_img_workspace(img, workspace_ratio=1.0)
            if im_work is None:
                print("Unable to find markers")
                try_without_success += 1
                if display_stream:
                    cv2.imshow("Last image saw", img)
                    cv2.waitKey(25)
                continue

            # Applying Threshold on Color
            color_hsv_setting = ColorHSV.ANY.value
            img_thresh = threshold_hsv(im_work, *color_hsv_setting)

            if display_stream:
                show_img("Last image saw", img, wait_ms=0)
                show_img("Image thresh", img_thresh, wait_ms=30)
            # Getting biggest contour/blob from threshold image
            contour = biggest_contour_finder(img_thresh)
            if contour is None or len(contour) == 0:
                print("No blob found")
                obj_found = False
            else:
                img_thresh_rgb = cv2.cvtColor(img_thresh, cv2.COLOR_GRAY2BGR)
                draw_contours(img_thresh_rgb, [contour])
                show_img("Image thresh", img_thresh, wait_ms=30)

                # Getting contour/blob center and angle
                cx, cy = get_contour_barycenter(contour)
                cx_rel, cy_rel = relative_pos_from_pixels(im_work, cx, cy)
                angle = get_contour_angle(contour)

                # Getting object world pose from relative pose
                status, obj_pose = niryo_one_client.get_target_pose_from_rel(workspace_name,
                                                                             height_offset=0.0,
                                                                             x_rel=cx_rel, y_rel=cy_rel,
                                                                             yaw_rel=angle)
                if status is not True:
                    try_without_success += 1
                    continue
                obj_found = True
        if not status or not obj_found:
            try_without_success += 1
            continue
        # Everything is good, so we going to object
        niryo_one_client.pick_from_pose(*obj_pose.to_list())

        # Computing new place pose
        offset_x = count % grid_dimension[0] - grid_dimension[0] // 2
        offset_y = (count // grid_dimension[1]) % 3 - grid_dimension[1] // 2
        offset_z = count // (grid_dimension[0] * grid_dimension[1])
        place_pose = center_conditioning_pose.copy_with_offsets(0.05 * offset_x, 0.05 * offset_y, 0.025 * offset_z)

        # Placing
        niryo_one_client.place_from_pose(*place_pose.to_list())

        try_without_success = 0
        count += 1


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
