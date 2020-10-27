#!/usr/bin/env python
import numpy as np

from geometry_msgs.msg import Point  # , TransformStamped
# from tf import transformations

from niryo_one_pose_converter.file_manager import FileManager
from niryo_one_msgs.msg import RobotState, RPY


class Workspace(object):
    """
    Represents a workspace that is defined by 4 points that represent the
    visual markers.

    :param name: unique (!) name that will be used to reference it
    :type name: str
    :param points: points defining the workspace, 0th element is the origin.
    :type points: [float, float, float]
    :param x_width: width of the workspace in m (between point_0 and point_1)
    :type x_width: float
    :param y_width: width of the workspace in m (between point_0 and point_3)
    :type y_width: float
    :param transform: tf transform msg from base_link to workspace origin
    :type transform: geometry_msgs::TransformStamped
    """

    def __init__(self, name=""):
        self.name = name
        self.robot_poses = []
        self.points = []
        self.x_width = 0.0
        self.y_width = 0.0
        self.yaw_center = "0"
        self.position_matrix = np.array([])
        self.rotation_matrix = np.array([])

    def to_dict(self):
        dict_ = dict()
        dict_["name"] = self.name
        dict_["robot_poses"] = [[pose.position.x, pose.position.y, pose.position.z,
                                 pose.rpy.roll, pose.rpy.pitch, pose.rpy.yaw]
                                for pose in self.robot_poses]
        dict_["points"] = self.points
        dict_["x_width"] = self.x_width
        dict_["y_width"] = self.y_width
        dict_["yaw_center"] = self.yaw_center
        dict_["position_matrix"] = self.position_matrix.tolist()
        dict_["rotation_matrix"] = self.rotation_matrix.tolist()
        return dict_

    @classmethod
    def from_dict(cls, dict_):
        ws = cls(dict_["name"])
        ws.robot_poses = [RobotState(position=Point(*pose_list[:3]), rpy=RPY(*pose_list[3:]))
                          for pose_list in dict_["robot_poses"]]
        ws.points = dict_["points"]
        ws.x_width = dict_["x_width"]
        ws.y_width = dict_["y_width"]
        ws.yaw_center = dict_["yaw_center"]
        ws.position_matrix = np.array(dict_["position_matrix"])
        ws.rotation_matrix = np.array(dict_["rotation_matrix"])
        return ws


class WorkspaceManager(FileManager):
    """
    Manages the creation, storage and loading of worspaces.

    :raises NiryoOneFileException:
    """
    object_type = Workspace

    def __init__(self, ws_dir):
        FileManager.__init__(self, ws_dir, "workspace")

    @staticmethod
    def __validate_points(points):
        robot_points = np.array(points)
        if robot_points.shape != (4, 3):
            raise ValueError("Points must contain 4 points with 3 coordinates each.")

        x_1 = robot_points[1] - robot_points[0]
        x_2 = robot_points[2] - robot_points[3]
        y_1 = robot_points[3] - robot_points[0]
        y_2 = robot_points[2] - robot_points[1]

        # Check that the workspace as a sane size
        min_ws_size = 0.01  # 1cm
        for vec in (x_1, x_2, y_1, y_2):
            if np.linalg.norm(vec) < min_ws_size:
                raise ValueError("Workspace points are too close. "
                                 "Make sure that there are no duplicates in the workspace points")

        # Check that the workspace is approx. rectangular
        allowed_error = 0.02  # 2cm
        if np.linalg.norm(x_1 - x_2) > allowed_error or np.linalg.norm(y_1 - y_2) > allowed_error:
            raise ValueError("Workspace points don't form a rectangle. "
                             "Check that the points are in the right order.")

    def create(self, workspace_name, robot_poses, points):
        """
        Create a new workspace based on 4 recorded points corresponding to the
        real positions of the visual markers.

        :param workspace_name: unique (!) name that will be used to reference it
        :param robot_poses: ?
        :type workspace_name: str
        :param points: real positions of the visual markers in robot coordinates
        :type points: [float, float, float]
        """
        self.__validate_points(points)

        robot_points = np.array(points)
        relative_points = np.array(
            [[0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]], dtype=float)
        position_matrix = np.linalg.lstsq(relative_points, robot_points)[0].T

        # Calculate coordinate system for rotation
        ws_points = np.dot(position_matrix, relative_points.T).T
        ws_x_vec = ws_points[1] - ws_points[0]  # x vector is from point 0 to point 1
        ws_y_vec = ws_points[3] - ws_points[0]  # preliminary y vecor is from point 0 to point 3
        # but not necessarily perpendicular to the x vector. We just need it to calculate the normal vector
        ws_z_vec = np.cross(ws_x_vec, ws_y_vec)  # The z vector is the normal vector to our workspace plane
        ws_y_vec = np.cross(ws_z_vec, ws_x_vec)  # Recalculate the y vector s.th. it is perpendicular to x and y

        # Normalize
        ws_x_vec = ws_x_vec / np.linalg.norm(ws_x_vec)
        ws_y_vec = ws_y_vec / np.linalg.norm(ws_y_vec)
        ws_z_vec = ws_z_vec / np.linalg.norm(ws_z_vec)

        workspace_rotation = np.eye(4)
        workspace_rotation[:3, :3] = np.stack([ws_x_vec, ws_y_vec, ws_z_vec]).T

        workspace = Workspace(workspace_name)
        workspace.points = points
        workspace.robot_poses = robot_poses
        workspace.position_matrix = position_matrix
        workspace.rotation_matrix = workspace_rotation
        workspace.x_width = np.linalg.norm(ws_points[1] - ws_points[0])
        workspace.y_width = np.linalg.norm(ws_points[3] - ws_points[0])
        workspace.yaw_center = self.__compute_yaw_center(ws_points)

        self._write(workspace_name, workspace)

    @staticmethod
    def __compute_yaw_center(ws_points):
        x_mean = np.mean([point[0] for point in ws_points])
        y_mean = np.mean([point[1] for point in ws_points])

        if 0.05 < x_mean > abs(y_mean):  # Workspace in front robot
            return 0
        elif 0.05 < y_mean > abs(x_mean):  # Workspace on the robot's left
            return np.pi/2
        elif -0.05 > y_mean and abs(y_mean) > abs(x_mean):  # Workspace on the robot's right
            return -np.pi/2

        return 0


if __name__ == '__main__':
    pass
