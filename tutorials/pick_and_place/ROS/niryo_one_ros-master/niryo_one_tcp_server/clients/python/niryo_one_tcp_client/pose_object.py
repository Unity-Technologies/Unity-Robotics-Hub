#!/usr/bin/env python


class PoseObject:
    def __init__(self, x, y, z, roll, pitch, yaw):
        # X (meter)
        self.x = float(x)
        # Y (meter)
        self.y = float(y)
        # Z (meter)
        self.z = float(z)
        # Roll (radian)
        self.roll = float(roll)
        # Pitch (radian)
        self.pitch = float(pitch)
        # Yaw (radian)
        self.yaw = float(yaw)

    def __str__(self):
        return "x = {:.3f}, y = {:.3f}, z = {:.3f},\nroll = {:.3f}, pitch = {:.3f}, yaw = {:.3f}".format(self.x, self.y,
                                                                                                         self.z,
                                                                                                         self.roll,
                                                                                                         self.pitch,
                                                                                                         self.yaw)

    def __repr__(self):
        return self.__str__()

    def copy_with_offsets(self, x_offset=0, y_offset=0, z_offset=0, roll_offset=0, pitch_offset=0, yaw_offset=0):
        return PoseObject(self.x + x_offset,
                          self.y + y_offset,
                          self.z + z_offset,
                          self.roll + roll_offset,
                          self.pitch + pitch_offset,
                          self.yaw + yaw_offset)

    def to_list(self):
        list_pos = [self.x, self.y, self.z, self.roll, self.pitch, self.yaw]
        return list(map(float, list_pos))
