#!/usr/bin/env python
from geometry_msgs.msg import TransformStamped

from niryo_one_pose_converter.file_manager import FileManager


class Grip(object):
    """
    Represents the robot tool_0 position (the one we plan for) with respect to
    an object we want to approach.

    :param name: unique (!) name for the grip that will be used to reference it
    :param transform: the tf transform msg from object to tool_0
    """

    def __init__(self, name, transform):
        self.name = name
        self.transform = transform

    def to_dict(self):
        dict_ = dict()
        dict_["name"] = self.name
        transform_dict = dict()
        transform_dict["header_frame_id"] = self.transform.header.frame_id
        transform_dict["child_frame_id"] = self.transform.child_frame_id
        transform_dict["translation"] = [self.transform.transform.translation.x,
                                         self.transform.transform.translation.y,
                                         self.transform.transform.translation.z]
        transform_dict["quaternion"] = [self.transform.transform.rotation.x,
                                        self.transform.transform.rotation.y,
                                        self.transform.transform.rotation.z,
                                        self.transform.transform.rotation.w]
        dict_["transform"] = transform_dict
        return dict_

    @classmethod
    def from_dict(cls, dict_):
        t = TransformStamped()
        t.transform.translation.x = dict_["transform"]["translation"][0]
        t.transform.translation.y = dict_["transform"]["translation"][1]
        t.transform.translation.z = dict_["transform"]["translation"][2]

        t.transform.rotation.x = dict_["transform"]["quaternion"][0]
        t.transform.rotation.y = dict_["transform"]["quaternion"][1]
        t.transform.rotation.z = dict_["transform"]["quaternion"][2]
        t.transform.rotation.w = dict_["transform"]["quaternion"][3]

        t.header.frame_id = dict_["transform"]["header_frame_id"]
        t.child_frame_id = dict_["transform"]["child_frame_id"]

        return cls(dict_["name"], t)


class GripManager(FileManager):
    """
    Manages the creation, storage and loading of grips.

    :raises NiryoOneFileException:
    """
    object_type = Grip

    def __init__(self, grip_dir, protected_grips=None):
        FileManager.__init__(self, grip_dir, "grip", protected_names=protected_grips)

    def create(self, grip_name, transform):
        grip = Grip(grip_name, transform)
        self._write(grip_name, grip)


if __name__ == '__main__':
    pass
