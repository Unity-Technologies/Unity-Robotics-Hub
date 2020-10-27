#!/usr/bin/env python

import rospy
import tf_conversions as tfc
import math

from niryo_one_pose_converter.transform_handler import TransformHandler
from niryo_one_pose_converter.grip_manager import GripManager
from niryo_one_pose_converter.workspace_manager import WorkspaceManager
from niryo_one_msgs.msg import RobotState
from niryo_one_msgs.srv import ObjDetection, ObjDetectionResponse
from niryo_one_msgs.srv import EditGrip, EditGripResponse
from niryo_one_msgs.srv import EditWorkspace, EditWorkspaceResponse
from niryo_one_msgs.srv import GetTargetPose, GetTargetPoseResponse
from niryo_one_msgs.srv import GetWorkspaceRatio
from niryo_one_msgs.srv import GetWorkspaceList
from niryo_one_msgs.srv import GetWorkspaceRobotPoses


class PoseConverter(object):
    """
    This class is an interface for manipulating object poses and converting them to robot poses.
    In particular, it allows
        - creation of workspaces
        - created of grips
        - convertion of workspace-relative poses (e.g. from camera) to robot poses
    """

    def __init__(self):
        ws_dir = rospy.get_param("~workspace_dir", "~/niryo_one_workspaces/")
        grip_dir = rospy.get_param("~grip_dir", "~/catkin_ws/src/niryo_one_pose_converter/grips/")

        tool_config_list = rospy.get_param("niryo_one_tools/tool_list")
        self.tool_id_gripname_dict = {tool["id"]: "default_" + tool["name"].replace(" ", "_")
                                      for tool in tool_config_list}
        self.tool_id_gripname_dict[0] = "default_Calibration_Tip"

        self.ws_manager = WorkspaceManager(ws_dir)
        self.grip_manager = GripManager(grip_dir, self.tool_id_gripname_dict.values())
        self.transform_handler = TransformHandler()
        self.edit_grip_server = rospy.Service(
            'niryo_one/pose_converter/edit_grip', EditGrip, self.__callback_edit_grip)
        self.edit_ws_server = rospy.Service(
            'niryo_one/pose_converter/edit_workspace', EditWorkspace,
            self.__callback_edit_workspace)
        self.get_ws_ratio_server = rospy.Service(
            'niryo_one/pose_converter/get_workspace_ratio', GetWorkspaceRatio,
            self.__callback_workspace_ratio)
        self.get_ws_list_server = rospy.Service(
            'niryo_one/pose_converter/get_workspace_list', GetWorkspaceList,
            self.__callback_workspace_list)
        self.get_target_pose_server = rospy.Service(
            'niryo_one/pose_converter/get_workspace_poses', GetWorkspaceRobotPoses,
            self.__callback_workspace_poses)
        self.get_target_pose_server = rospy.Service(
            'niryo_one/pose_converter/get_target_pose', GetTargetPose,
            self.__callback_target_pose)

    # ROS CALLBACKS

    def __callback_edit_grip(self, req):
        if req.cmd == req.CREATE:
            try:
                self.create_grip(req.name, req.workspace, req.robot_pose)
                return 200, "Created grip '{}'".format(req.name)
            except Exception as e:
                return 400, str(e)
        elif req.cmd == req.REMOVE:
            try:
                self.remove_grip(req.name)
                return 200, "Removed grip '{}'".format(req.name)
            except Exception as e:
                return 400, str(e)
        else:
            return 400, "cmd '" + str(req.cmd) + "' not found."

    def __callback_edit_workspace(self, req):
        if req.cmd == req.CREATE:
            try:
                self.create_workspace(req.name, req.pose_origin, req.pose_1,
                                      req.pose_2, req.pose_3)
                return 200, "Created workspace '{}'".format(req.name)
            except Exception as e:
                return 400, str(e)
        elif req.cmd == req.REMOVE:
            try:
                self.remove_workspace(req.name)
                return 200, "Removed workspace '{}'".format(req.name)
            except Exception as e:
                return 400, str(e)
        else:
            return 400, "cmd '" + str(req.cmd) + "' not found."

    def __callback_target_pose(self, req):
        try:
            if req.grip == req.GRIP_AUTO:
                pose = self.get_target_pose_auto_grip(req.workspace, req.tool_id, req.height_offset,
                                                      req.x_rel, req.y_rel, req.yaw_rel)
            else:
                pose = self.get_target_pose(req.workspace, req.grip, req.height_offset,
                                            req.x_rel, req.y_rel, req.yaw_rel)
            return 200, "SUCCESS", pose
        except Exception as e:
            return 400, str(e), RobotState()

    def __callback_workspace_ratio(self, req):
        try:
            ratio = self.get_workspace_ratio(req.workspace)
            return 200, "SUCCESS", ratio
        except Exception as e:
            return 400, str(e), 0

    def __callback_workspace_list(self, _):
        try:
            ws_list = self.get_available_workspaces()
            return {"workspaces": ws_list}
        except Exception as e:
            rospy.logerr("Error occured when getting workspace list: {}".format(e))
            return {"workspaces": []}

    def __callback_workspace_poses(self, req):
        try:
            ws = self.ws_manager.read(req.workspace)
            return (200, "SUCCESS", ws.robot_poses[0], ws.robot_poses[1],
                    ws.robot_poses[2], ws.robot_poses[3])
        except Exception as e:
            return 400, str(e), RobotState(), RobotState(), RobotState(), RobotState()

    # REGULAR CLASS FUNCTIONS

    def create_workspace(self, name, robot_pose_point_0, robot_pose_point_1,
                         robot_pose_point_2, robot_pose_point_3):
        """
        Creates a new workspace based on 4 recoded robot poses. The 0th point
        needs to be the workspace origin. Points ordered clockwise.

        :param name: name of the new workspace
        :param robot_pose_point_0: pose when pointing at origin
        :param robot_pose_point_1: pose when pointing at point 1
        :param robot_pose_point_2: pose when pointing at point 2
        :param robot_pose_point_3: pose when pointing at point 3
        """
        robot_poses = [robot_pose_point_0, robot_pose_point_1,
                       robot_pose_point_2, robot_pose_point_3]
        points = []
        for pose in robot_poses:
            point = self.transform_handler.get_calibration_tip_position(pose)
            points.append([point.x, point.y, point.z])
        self.ws_manager.create(name, robot_poses, points)

    def remove_workspace(self, name):
        """
        Removes a workspace
        :param name: name of the workspace to remove
        """
        self.ws_manager.remove(name)

    def create_grip(self, name, workspace, robot_pose):
        """
        Creates a new grip from a recorded robot pose.
        Place object on origin of specified workspace and move robot to grip
        the object.

        :param name: name of the new grip
        :param workspace: name of the workspace the object is placed on
        :param robot_pose: pose when gripping the object
        """
        current_ws = self.ws_manager.read(workspace)
        self.transform_handler.set_workspace(current_ws)
        transform = self.transform_handler.get_grip_transform(current_ws, robot_pose)
        self.grip_manager.create(name, transform)

    def remove_grip(self, name):
        """
        Removes a grip
        :param name: name of the grip to remove
        """
        self.grip_manager.remove(name)

    def get_target_pose_auto_grip(self, workspace, tool_id, height_offset, x_rel, y_rel, yaw_rel):
        """
        Loads the default grip corresponding to the tool_id and calls get_target_pose.

        See arguments of 'get_target_pose'
        """
        return self.get_target_pose(workspace, self.tool_id_gripname_dict[tool_id], height_offset,
                                    x_rel, y_rel, yaw_rel)

    def get_workspace_ratio(self, workspace):
        """
        Reads the ratio (width/height) of the specified workspace. This function is used by the
        camera to do the perspective transform.

        :param workspace: name of the workspace
        """
        current_ws = self.ws_manager.read(workspace)
        return current_ws.x_width / current_ws.y_width

    def get_available_workspaces(self):
        """
        Returns a list of all available workspace names.
        """
        return self.ws_manager.get_all_names()

    def get_target_pose(self, workspace, grip, height_offset, x_rel, y_rel, yaw_rel):
        """
        Computes the robot pose that can be used to grab an object which is
        positioned relative to the given workspace using the specified grip.

        :param workspace: name of the workspace the object is in
        :param grip: name of the grip to use to pick the object
        :param height_offset: z-offset that is added to the grip
        :param x_rel: x relative position of the object inside working zone
        :param y_rel: y relative position of the object inside working zone
        :param yaw_rel: angle of the object inside working zone
        """

        current_ws = self.ws_manager.read(workspace)
        current_grip = self.grip_manager.read(grip)
        current_grip.transform.transform.translation.z += height_offset

        self.transform_handler.set_grip(current_grip)
        self.transform_handler.set_relative_pose_object(current_ws, x_rel, y_rel, yaw_rel,
                                                        yaw_center=current_ws.yaw_center)

        base_link_to_tool_target = self.transform_handler.get_gripping_transform()
        q = base_link_to_tool_target.transform.rotation

        roll, pitch, yaw = tfc.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

        pose = RobotState()
        pose.position.x = base_link_to_tool_target.transform.translation.x
        pose.position.y = base_link_to_tool_target.transform.translation.y
        pose.position.z = base_link_to_tool_target.transform.translation.z

        pose.rpy.roll = roll
        pose.rpy.pitch = pitch
        pose.rpy.yaw = yaw

        return pose


if __name__ == "__main__":
    rospy.init_node("pose_converter")
    pc = PoseConverter()
    rospy.spin()
