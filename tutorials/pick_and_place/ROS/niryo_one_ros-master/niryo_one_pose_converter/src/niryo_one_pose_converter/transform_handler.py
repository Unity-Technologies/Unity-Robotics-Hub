import threading
import numpy as np

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from tf import transformations


class TransformHandler(object):
    """
    This class uses a tfBuffer to handle transforms related to the vision kit.
    """

    def __init__(self):
        self.__tf_buffer = tf2_ros.Buffer()
        self.__debug_stop_event = threading.Event()
        self.__debug_thread = None
        self.__debug_current_ws = None  # only for debugging purposes

    def __del__(self):
        self.disable_debug()

    def set_relative_pose_object(self, workspace, x_rel, y_rel, yaw_rel, yaw_center=None):
        """
        Updates the transform base_link -> object_base in local tfBuffer

        :param workspace: reference workspace object
        :param x_rel: object base x position relative to workspace
        :param y_rel: object base y position relative to workspace
        :param yaw_rel: object base rotation on z relative to workspace
        :param yaw_center: Avoid over rotation
        """
        position = np.dot(workspace.position_matrix, np.array([x_rel, y_rel, 1]))
        camera_rotation = transformations.euler_matrix(0, 0, yaw_rel)
        # Here we correct the object orientation to be similar to base_link if
        # the object in on the ground. Not neccessarily needed to be honest...
        convention_rotation = np.array([[0, -1, 0, 0],
                                        [-1, 0, 0, 0],
                                        [0, 0, -1, 0],
                                        [0, 0, 0, 1]])

        object_rotation = transformations.concatenate_matrices(
            workspace.rotation_matrix, camera_rotation, convention_rotation)
        roll, pitch, yaw = transformations.euler_from_matrix(object_rotation)

        # Correcting yaw to avoid out of reach targets
        if yaw_center is not None:
            if yaw < yaw_center - np.pi / 2:
                yaw += np.pi
            elif yaw > yaw_center + np.pi / 2:
                yaw -= np.pi

        q = transformations.quaternion_from_euler(roll, pitch, yaw)

        t = TransformStamped()
        t.transform.translation.x = position[0]
        t.transform.translation.y = position[1]
        t.transform.translation.z = position[2]

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        t.header.frame_id = "base_link"
        t.child_frame_id = "object_base"

        self.__tf_buffer.set_transform(t, "default_authority")

    def set_grip(self, grip):
        """
        Updates the transform object_base -> tool_link_target in local tfBuffer
        :param grip:

        """
        if grip.transform.header.frame_id != "object_base":
            print "Grip transform need to have header frame 'object_base'"
            return False

        if grip.transform.child_frame_id != "tool_link_target":
            print "Grip transform need to have child frame 'tool_link_target'"
            return False

        self.__tf_buffer.set_transform(grip.transform, "default_authority")
        return True

    def get_object_base_transform(self):
        """
        Reads the transform base_link -> object_base from local tfBuffer

        :returns: transform base_link -> object_base
        """
        return self.__tf_buffer.lookup_transform("base_link", "object_base",
                                                 rospy.Time(0))

    def get_gripping_transform(self):
        """
        Reads the transform base_link -> tool_link_target from local tfBuffer

        :returns: transform base_link -> tool_link_target
        """
        return self.__tf_buffer.lookup_transform(
            "base_link", "tool_link_target", rospy.Time(0))

    def get_calibration_tip_position(self, robot_pose):
        """
        Retrieves the position of the calibration tip from a given robot pose.

        :param robot_pose: pose of the robot's tool_link
        :returns: xyz position of calibration tip in robot coordinates
        """
        # First apply transform for robot pose
        base_link_to_tool_link = self.transform_from_euler(
            robot_pose.position.x, robot_pose.position.y, robot_pose.position.z,
            robot_pose.rpy.roll, robot_pose.rpy.pitch, robot_pose.rpy.yaw,
            "base_link", "tool_link"
        )
        self.__tf_buffer.set_transform(base_link_to_tool_link,
                                       "default_authority")

        # Manually apply transform for the calibration tool
        tool_link_to_calib_tip = self.transform_from_euler(
            0.025, 0, 0, 0, 0, 0, "tool_link", "calibration_tip"
        )
        self.__tf_buffer.set_transform(tool_link_to_calib_tip,
                                       "default_authority")

        base_link_to_calib_tip = self.__tf_buffer.lookup_transform(
            "base_link", "calibration_tip", rospy.Time(0))
        return base_link_to_calib_tip.transform.translation

    def get_grip_transform(self, ws_name, robot_pose):
        """
        Retrieves the transform needed to create a grip supposing the object
        is placed on the origin of the given workspace.

        :param ws_name: name of the workspace the object is placed on
        :param robot_pose: pose of the robot's tool_link
        """
        # First apply transform for robot pose
        base_link_to_tool_link = self.transform_from_euler(
            robot_pose.position.x, robot_pose.position.y, robot_pose.position.z,
            robot_pose.rpy.roll, robot_pose.rpy.pitch, robot_pose.rpy.yaw,
            "base_link", "tool_link"
        )
        self.__tf_buffer.set_transform(base_link_to_tool_link,
                                       "default_authority")

        # Manually place object on origin
        self.set_relative_pose_object(ws_name, 0, 0, 0)

        # Lookup the grip
        t = self.__tf_buffer.lookup_transform("object_base", "tool_link",
                                              rospy.Time(0))
        t.child_frame_id = "tool_link_target"
        return t

    @staticmethod
    def transform_from_euler(x, y, z, roll, pitch, yaw, header_frame_id,
                             child_frame_id):
        """
        Creates a new stamped transform from translation and euler-orientation

        :param x: x translation
        :param y: y translation
        :param z: z translation
        :param roll: orientation roll
        :param pitch: orientation pitch
        :param yaw: orientation yaw
        :param header_frame_id: transform from this frame
        :param child_frame_id: transform to this frame

        :returns: transform
        """
        t = TransformStamped()
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z

        q = transformations.quaternion_from_euler(roll, pitch, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        t.header.frame_id = header_frame_id
        t.child_frame_id = child_frame_id

        return t

    def enable_debug(self):
        """
        Start publishing debug information on /tf and /visualization_marker for
        debugging using rviz. This will happen in a separate thread.
        """
        self.__debug_thread = threading.Thread(target=self.__debug_loop)
        self.__debug_thread.start()

    def disable_debug(self):
        """
        Stop publishing debug inforation
        """
        self.__debug_stop_event.set()
        if self.__debug_thread is not None:
            self.__debug_thread.join()

    def __debug_loop(self):
        """
        Debug loop that will run in a separate thread.
        (tfBuffer should be threadsafe)
        """
        broadcaster = tf2_ros.TransformBroadcaster()
        rviz_marker_pub = rospy.Publisher('/visualization_marker', Marker,
                                          queue_size=1000)
        rate = rospy.Rate(5)
        while not self.__debug_stop_event.is_set() and not rospy.is_shutdown():
            if self.__debug_current_ws is None:
                print "Could not publish debug tf, no workspace set."
                rate.sleep()
                continue

            try:
                broadcaster.sendTransform(
                    self.__tf_buffer.lookup_transform(
                        "base_link", self.__debug_current_ws.name,
                        rospy.Time(0))
                )
                broadcaster.sendTransform(
                    self.__tf_buffer.lookup_transform(
                        self.__debug_current_ws.name, "object_base",
                        rospy.Time(0))
                )
                broadcaster.sendTransform(
                    self.__tf_buffer.lookup_transform(
                        "object_base", "tool_link_target", rospy.Time(0))
                )
            except tf2_ros.LookupException as e:
                print "Could not publish debug tf: ", e

            for i in range(4):  # Iterate over the 4 markers defining the workspace
                msg = Marker()
                msg.header.frame_id = "base_link"
                msg.id = i
                msg.type = 2  # It correspond to a sphere which will be drawn

                msg.pose.position.x = self.__debug_current_ws.points[i][0]
                msg.pose.position.y = self.__debug_current_ws.points[i][1]
                msg.pose.position.z = self.__debug_current_ws.points[i][2]

                msg.scale.x = 0.005
                msg.scale.y = 0.005
                msg.scale.z = 0.005

                msg.color.r = 1.0 if i == 0 or i == 3 else 0.0
                msg.color.g = 1.0 if i == 1 or i == 3 else 0.0
                msg.color.b = 1.0 if i == 2 or i == 3 else 0.0
                msg.color.a = 1.0

                rviz_marker_pub.publish(msg)

        rate.sleep()
