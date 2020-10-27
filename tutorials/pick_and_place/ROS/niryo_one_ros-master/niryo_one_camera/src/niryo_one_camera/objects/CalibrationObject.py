import cv2
import numpy as np


class CalibrationObject:
    """
    Class containing intrinsic parameters of the camera used
    """

    def __init__(self):
        self.__mtx = None
        self.__dist = None

        self.__fx = None
        self.__fy = None
        self.__cx = None
        self.__cy = None

        self._is_set = False

    def set_from_values(self, mtx, dist):
        self.__mtx = mtx
        self.__dist = dist
        self.__finish_setting()

    def set_from_yaml(self, yaml_file):
        self.__mtx = np.reshape(yaml_file["mtx"], (3, 3))
        self.__dist = np.expand_dims(yaml_file["dist"], axis=0)
        self.__finish_setting()
        return self

    def __finish_setting(self):
        self.__fx = self.__mtx[0][0]
        self.__fy = self.__mtx[1][1]
        self.__cx = self.__mtx[0][2]
        self.__cy = self.__mtx[1][2]

        self._is_set = True

    def __str__(self):
        big_string = "mtx\n" + str(self.__mtx) + "\n"
        big_string += "dist\n" + str(self.__dist) + "\n"
        return big_string

    def get_center_position(self):
        return tuple([self.__cx, self.__cy])

    def get_center_position_int(self):
        return tuple([int(round(self.__cx)), int(round(self.__cy))])

    def get_intrinsic_parameters(self):
        return {
            "fx": self.__fx,
            "fy": self.__fy,
            "cx": self.__cx,
            "cy": self.__cy,
        }

    def get_cam_mtx_and_dist_coefs(self):
        return self.__mtx, self.__dist

    def undistort_image(self, img):
        return cv2.undistort(src=img, cameraMatrix=self.__mtx,
                             distCoeffs=self.__dist, newCameraMatrix=None)

    def meters_to_pixels(self, length, width, z_offset):
        u = (length / z_offset) * self.__fx
        v = (width / z_offset) * self.__fy

        return tuple([u, v])

    def get_camera_info(self):
        """
        Return value for ROS CameraInfo message
        """
        return self.__mtx, self.__dist

    def is_set(self):
        return self._is_set
