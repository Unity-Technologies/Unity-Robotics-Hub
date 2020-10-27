"""
This file shows how to use Python TCP API to do a simple pick and place with the Niryo One using a conveyor
"""

# Imports
from niryo_one_tcp_client import *

# -- MUST Change these variables
robot_ip_address = "192.168.1.202"  # IP address of the Niryo One
gripper_used = RobotTool.GRIPPER_2  # Tool used for picking
gpio_sensor = RobotPin.GPIO_1A  # Pin of the sensor
conveyor_id = ConveyorID.ID_1  # Id of the used conveyor

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


def pick_n_place_w_conveyor(niryo_one_client):
    # Enable connection with conveyor
    niryo_one_client.activate_conveyor(conveyor_id)
    # Turn conveyor on
    niryo_one_client.control_conveyor(conveyor_id=conveyor_id, control_on=True,
                                      speed=50, direction=ConveyorDirection.FORWARD)
    # Wait for sensor to turn to low state which means it has something in front of it
    while not niryo_one_client.digital_read(gpio_sensor) == DigitalState.LOW:
        niryo_one_client.wait(0.1)
    # Turn conveyor off
    niryo_one_client.control_conveyor(conveyor_id=conveyor_id, control_on=False,
                                      speed=0, direction=ConveyorDirection.FORWARD)
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
    pick_n_place_w_conveyor(client)

    # Ending
    client.move_joints(*sleep_joints)
    client.set_learning_mode(True)
    # Releasing connection
    client.quit()
