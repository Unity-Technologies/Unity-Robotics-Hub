#!/usr/bin/env python
import os
import rospkg
import yaml
from threading import Lock
import time

import cv2
import numpy as np
import rospy

from sensor_msgs.msg import CameraInfo, CompressedImage
from std_srvs.srv import SetBool, SetBoolResponse

from niryo_one_msgs.msg import ObjectPose

from niryo_one_msgs.srv import GetCalibrationCam, GetCalibrationCamResponse
from niryo_one_msgs.srv import ObjDetection, ObjDetectionResponse
from niryo_one_msgs.srv import DebugColorDetection
from niryo_one_msgs.srv import DebugMarkers, DebugMarkersResponse
from niryo_one_msgs.srv import SetCalibrationCam, SetCalibrationCamResponse
from niryo_one_msgs.srv import TakePicture, TakePictureResponse

from niryo_one_camera.enums import ObjectType, ColorHSV
from niryo_one_camera.image_functions import compress_image, debug_threshold_color, debug_markers
from niryo_one_camera.objects.CalibrationObject import CalibrationObject
from niryo_one_camera.objects.ObjectDetector import ObjectDetector


class VisionPublisher:
    """
    Object which will contains all ROS Publishers & Services relate to image processing
    """

    def __init__(self):
        # -- ROS
        rospy.init_node('vision_node', anonymous=False)
        self.__path_package = rospkg.RosPack().get_path('niryo_one_camera')
        # PUBLISHERS
        self.__publisher_compressed_stream = rospy.Publisher('~compressed_video_stream',
                                                             CompressedImage, queue_size=1)
        # - SERVICES
        self.__service_start_stop = rospy.Service('~start_stop_video_streaming',
                                                  SetBool,
                                                  self.__callback_start_stop)
        # CALIBRATION
        self.__calibration_object_name = rospy.get_param("~obj_calib_name")
        self.__calibration_object = self.__generate_calib_object_from_setup()

        self.__service_get_calibration_obj = rospy.Service('~get_calibration_camera',
                                                           GetCalibrationCam,
                                                           self.__callback_get_calibration_object)

        self.__service_take_picture = rospy.Service('~take_picture',
                                                    TakePicture,
                                                    self.__callback_take_picture)

        self.__service_debug_markers = rospy.Service('~debug_markers',
                                                     DebugMarkers,
                                                     self.__callback_debug_markers)
        self.__service_debug_colors = rospy.Service('~debug_colors',
                                                    DebugColorDetection,
                                                    self.__callback_debug_color)

        self.__debug_compression_quality = rospy.get_param("~debug_compression_quality")

        # OBJECT DETECTION
        self.__service_obj_detection_relative = rospy.Service('~obj_detection_rel',
                                                              ObjDetection, self.__get_obj_relative_pose)
        # -- Stuff
        self.__lock = Lock()
        self.__running = False
        self.__undistort_stream = rospy.get_param('~undistort_stream')

        self.__calibration_grid_shape = tuple(rospy.get_param('~calibration_grid_shape'))

        # -- VIDEO STREAM
        self.__cam_port = rospy.get_param("~camera_port")
        self.__display = rospy.get_param("~display")

        self.__frame_rate = rospy.get_param("~frame_rate")
        self.__subsample_read = rospy.get_param("~subsampling")

        self.__stream_compression_quality = rospy.get_param("~stream_compression_quality")

        self.__actualization_rate_ros = rospy.Rate(1.1 * int(self.__frame_rate))
        self.__actualization_rate_no_stream = rospy.Rate(0.5)

        self.__video_stream = None
        self.__frame_raw = None
        self.__frame_undistort = None
        self.__count_grab_failed = 0

        # -- Starting

        self.__should_run = True
        self.__loop()

    # -- PUBLIC

    def read_undistorted(self):
        if not self.__running or not self.__should_run:
            return None
        with self.__lock:
            ret, frame = self.__video_stream.retrieve()
        if ret is False:
            return None
        if self.__calibration_object.is_set():
            return self.__calibration_object.undistort_image(frame)
        else:
            return frame

    def read_raw_img(self):
        if not self.__running or not self.__should_run:
            return None
        with self.__lock:
            ret, frame = self.__video_stream.retrieve()
        if ret is False:
            return None
        return frame

    # -- Private
    # - Initialization

    def __setup_stream_settings(self):
        # Set compression format
        self.__video_stream.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        # Set Buffer size
        # -- Not available in opencv 3.4 -- #
        self.__video_stream.set(cv2.CAP_PROP_BUFFERSIZE, rospy.get_param("~buffer_size"))
        # Set image size
        w, h = rospy.get_param("~frame_size")
        self.__video_stream.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.__video_stream.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        # Set frame rate
        self.__video_stream.set(cv2.CAP_PROP_FPS, self.__frame_rate)

    def __generate_calib_object_from_setup(self):
        path_yaml = os.path.join(self.__path_package, "config/{}.yaml".format(self.__calibration_object_name))
        if not os.path.isfile(path_yaml):
            rospy.logwarn("intrinsics file '{}' does not exist".format(self.__calibration_object_name))
            return CalibrationObject()
        with open(path_yaml, "r") as input_file:
            yaml_file = yaml.load(input_file)
        return CalibrationObject().set_from_yaml(yaml_file)

    def __loop(self):
        while not rospy.is_shutdown():
            if not self.__should_run:
                self.__running = False
                self.__actualization_rate_no_stream.sleep()
                continue
            # rospy.loginfo("Stream video should run")
            self.__video_stream = cv2.VideoCapture(self.__cam_port)
            if not self.__video_stream.isOpened():
                rospy.logwarn("Cannot open video stream. Check camera is well plugged.")
                rospy.logwarn("Closing Video Stream")
                self.__should_run = False
                self.__running = False
                self.__actualization_rate_no_stream.sleep()
                continue

            rospy.loginfo("Video Stream Open")
            self.__setup_stream_settings()
            self.__running = True

            index_read = -1
            # init_time = time.time()
            while not rospy.is_shutdown() and self.__should_run:
                index_read = (index_read + 1) % self.__subsample_read
                with self.__lock:
                    bool_grabbed = self.__video_stream.grab()
                if not bool_grabbed:
                    self.__should_run = False
                    rospy.logwarn("Image not grabbed. Check camera is well plugged.")
                    rospy.logwarn("Closing Video Stream")
                    continue
                if index_read == 0:
                    _, frame = self.__video_stream.retrieve()
                    # after_time = time.time()
                    # rospy.logwarn("{:.2f} FPS".format(1 / (after_time - init_time)))
                    # init_time = after_time
                    self.__frame_raw = frame
                    if self.__undistort_stream:
                        self.__frame_undistort = self.__calibration_object.undistort_image(frame)
                    else:
                        self.__frame_undistort = None
                    stream_image = self.__frame_undistort if self.__undistort_stream else self.__frame_raw
                    if self.__display:
                        cv2.imshow("Video Stream", stream_image)
                        cv2.waitKey(1)
                    result, msg = generate_msg_from_image(stream_image,
                                                          compression_quality=self.__stream_compression_quality)

                    if not result:
                        continue
                    rospy.logdebug("Publishing an image")
                    self.__publisher_compressed_stream.publish(msg)

                self.__actualization_rate_ros.sleep()

            self.__video_stream.release()
            rospy.loginfo("Video Stream Stopped")
            self.__running = False

    # - CALLBACK
    def __get_obj_relative_pose(self, req):
        # Reading last image
        img = self.read_undistorted()
        if img is None:
            return ObjDetectionResponse.VIDEO_STREAM_NOT_RUNNING, ObjectPose(), "", "", CompressedImage()

        # Extracting parameters from request
        obj_type = ObjectType[req.obj_type]
        obj_color = ColorHSV[req.obj_color]
        workspace_ratio = req.workspace_ratio
        ret_image = req.ret_image

        # Creating ObjectDetector, an object for object detection
        self.__object_detector = ObjectDetector(
            obj_type=obj_type, obj_color=obj_color,
            workspace_ratio=workspace_ratio,
            ret_image_bool=ret_image,
        )

        # rospy.logdebug("Getting Object")
        # Launching pipeline
        status, msg_res_pos, obj_type, obj_color, im_draw = self.__object_detector.extract_object_with_hsv(img)
        if self.__object_detector.should_ret_image():
            _, msg_img = generate_msg_from_image(im_draw, compression_quality=self.__debug_compression_quality)
        else:
            msg_img = CompressedImage()
        return status, msg_res_pos, obj_type, obj_color, msg_img

    def __callback_set_calibration_object(self, req):
        yaml_path = os.path.join(self.__path_package, "config/{}.yaml".format(req.name))

        if not os.path.isfile(yaml_path):
            rospy.logwarn("intrinsics file '{}' does not exist".format(self.__calibration_object_name))
            return SetCalibrationCamResponse.NOT_SET

        if not self.__calibration_object.is_set():
            response = SetCalibrationCamResponse.SUCCESSFULLY_SET
        else:
            response = SetCalibrationCamResponse.OVERWRITTEN
        with open(yaml_path, "r") as input_file:
            yaml_file = yaml.load(input_file)
        self.__calibration_object.set_from_yaml(yaml_file)
        rospy.loginfo("New calibration object set : {}.yaml".format(req.name))
        return response

    def __callback_get_calibration_object(self, _):
        if not self.__calibration_object.is_set():
            return GetCalibrationCamResponse(False, CameraInfo())
        msg_camera_info = CameraInfo()
        mtx, dist = self.__calibration_object.get_camera_info()
        msg_camera_info.K = list(mtx.flatten())
        msg_camera_info.D = list(dist.flatten())
        return GetCalibrationCamResponse(True, msg_camera_info)

    def __callback_take_picture(self, req):
        path = os.path.expanduser(req.path)
        if not os.path.isdir(path):
            os.makedirs(path)
        time_string = time.strftime("%Y%m%d-%H%M%S")
        # img_name = "{:03d}.jpg".format(len([f for f in os.listdir(self.__images_path) if f.endswith(".jpg")]))

        img_full_path = "{}.jpg".format(os.path.join(path, time_string))
        im = self.read_raw_img()
        res_bool = cv2.imwrite(img_full_path, im)
        return TakePictureResponse(res_bool)

    def __callback_start_stop(self, req):
        command = ["stopping", "starting"][int(req.data)]
        if command == "stopping":
            if not self.__running:
                return SetBoolResponse(False, "The streaming is already stopped. Cannot stop it")

            else:
                self.__should_run = False
                return SetBoolResponse(True, "Stopping the stream")

        else:
            if self.__running:
                return SetBoolResponse(False, "The streaming is already started. Cannot start it")
            else:
                self.__should_run = True
                return SetBoolResponse(True, "Trying to launch the stream ...")

    def __callback_debug_markers(self, _):
        img = self.read_undistorted()
        if img is None:
            return False, CompressedImage()
        markers_detected, img_res = debug_markers(img)
        _, msg_img = generate_msg_from_image(img_res, compression_quality=self.__debug_compression_quality)

        return DebugMarkersResponse(markers_detected, msg_img)

    def __callback_debug_color(self, req):
        img = self.read_undistorted()
        if img is None:
            return CompressedImage()
        color = req.color
        img_res = debug_threshold_color(img, color)
        _, msg_img = generate_msg_from_image(img_res, compression_quality=self.__debug_compression_quality)

        return msg_img

    def __undistort(self, frame):
        if self.__calibration_object.is_set():
            return self.__calibration_object.undistort_image(frame)
        else:
            return frame

    @property
    def __raw_img(self):
        return self.__frame_raw.copy()


def generate_msg_from_image(img, compression_quality=90):
    """
    Generate ROS CompressedImage message from an OpenCV Image
    :param img: OpenCV Image
    :param compression_quality: integer between 1 - 100. The higher it is, the less information will be lost,
    but the heavier the compressed image will be
    :return: success, msg
    """
    result, compressed_img = compress_image(img, quality=compression_quality)
    if not result:
        return False, None

    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpg"
    msg.data = compressed_img

    return True, msg


if __name__ == '__main__':
    try:
        publisher = VisionPublisher()

    except rospy.ROSInterruptException:
        pass
