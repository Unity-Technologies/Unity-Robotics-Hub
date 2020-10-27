"""
This script allows to take 4 positions and to create a workspace from them
"""

# Imports
from niryo_one_tcp_client import *

# Connecting to robot
niryo_one_client = NiryoOneClient()
niryo_one_client.connect("192.168.1.202")  # =< Replace by robot ip address

niryo_one_client.set_learning_mode(True)
niryo_one_client.change_tool(RobotTool.NONE)

# Initializing useful variables
points = []
id_point = 1
# Asking user to type the new workspace's name
ws_name = input("Enter name of new workspace. Name: ")
while id_point < 5:  # Iterating over 4 markers
    input("Press enter when on point".format(id_point + 1))
    # Getting pose
    status, pose = niryo_one_client.get_pose()
    # If no error happened,
    if status is True:
        points.append(pose)
        id_point += 1
    else:
        print("error", status, pose)
# Creating workspace
ret = niryo_one_client.create_workspace(ws_name, *points)
print("Result", ret)
# Leaving
niryo_one_client.quit()
