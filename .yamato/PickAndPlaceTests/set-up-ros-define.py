from unityparser import UnityDocument

import argparse
import os

parser = argparse.ArgumentParser(description='Add ROS define symbols')
parser.add_argument('ros', type=str, help='ROS version: ros1 or ros2')

project_settings_filepath = os.path.join(".", "tutorials", "pick_and_place", "PickAndPlaceProject", "ProjectSettings", "ProjectSettings.asset")
if not os.path.exists(project_settings_filepath):
    raise FileNotFoundError("Not found %s".format(project_settings_filepath))
settings = UnityDocument.load_yaml(project_settings_filepath)
symbols = settings.entries[0].scriptingDefineSymbols

args = parser.parse_args()
if args.ros == "ros1":
		if symbols[1] is None:
				symbols[1] = "ROS1"
		else:
				symbols[1] += ";ROS1"
elif args.ros == "ros2":
		if symbols[1] is None:
				symbols[1] = "ROS2"
		else:
				symbols[1] += ";ROS2"
else:
    raise ValueError("Invalid input ROS version. Must be either ros1 or ros2")
settings.dump_yaml(project_settings_filepath)

