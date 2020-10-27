#!/usr/bin/env python

import cv2
import rospy
from threading import Lock
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Empty

from niryo_one_msgs.srv import ObjDetection, ObjDetectionResponse
from niryo_one_msgs.srv import SetCalibrationCam, SetCalibrationCamRequest, SetCalibrationCamResponse
from niryo_one_msgs.srv import GetCalibrationCam
from niryo_one_msgs.srv import TakePicture, TakePictureRequest
from niryo_one_msgs.srv import DebugMarkers, DebugMarkersRequest
from niryo_one_msgs.srv import DebugColorDetection

from niryo_one_camera.image_functions import extract_img_from_ros_msg


class VisionServicesClient:
    def __init__(self):
        rospy.init_node('vision_client_node', anonymous=True)

        self.video_stream_subscriber = rospy.Subscriber('/niryo_one_vision/compressed_video_stream',
                                                        CompressedImage, self.__callback_subscriber_video_stream,
                                                        queue_size=1)
        self.__lock = Lock()
        self.__img_read = None
        self.__services_name = {
            "object_pose_service": "/niryo_one_vision/obj_detection_rel",
            "set_calibration_camera_service": "/niryo_one_vision/set_calibration_camera",
            "get_calibration_camera_service": "/niryo_one_vision/get_calibration_camera",
            "take_picture": "/niryo_one_vision/take_picture",
            "debug_markers": "/niryo_one_vision/debug_markers",
            "debug_colors": "/niryo_one_vision/debug_colors",
        }

        self.__status_interpreter_obj_detection = {
            ObjDetectionResponse.SUCCESSFUL: "SUCCESSFUL",
            ObjDetectionResponse.OBJECT_NOT_FOUND: "OBJECT_NOT_FOUND",
            ObjDetectionResponse.MARKERS_NOT_FOUND: "MARKERS_NOT_FOUND",
        }

        self.__status_interpreter_calibration = {
            SetCalibrationCamResponse.SUCCESSFULLY_SET: "SUCCESSFULLY_SET",
            SetCalibrationCamResponse.OVERWRITTEN: "OVERWRITTEN",
            SetCalibrationCamResponse.NOT_SET: "NOT_SET",
        }

    def __callback_subscriber_video_stream(self, ros_data):
        with self.__lock:
            self.__img_read = extract_img_from_ros_msg(ros_data)

    def get_stream_img(self):
        with self.__lock:
            return self.__img_read

    def get_object_relative_pose(self, obj_type="SQUARE", obj_color="BLUE", ret_image=True):
        service_name = self.__services_name['object_pose_service']
        rospy.wait_for_service(service_name)

        try:
            detection_service = rospy.ServiceProxy(service_name, ObjDetection)
            response = detection_service(obj_type=obj_type, obj_color=obj_color,
                                         workspace_ratio=25. / 16, ret_image=ret_image)
            ret_status = self.__status_interpreter_obj_detection[response.status]
            if ret_status != "SUCCESSFUL":
                print "Object not detected: " + ret_status

            msg_res = response.obj_pose
            obj_type = response.obj_type
            obj_color = response.obj_color

            if ret_image:
                im_ret = extract_img_from_ros_msg(response.img)
            else:
                im_ret = None
            return ret_status, msg_res, obj_type, obj_color, im_ret

        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def set_calibration_camera(self, obj_name):
        service_name = self.__services_name['set_calibration_camera_service']
        rospy.wait_for_service(service_name)
        msg = SetCalibrationCamRequest()
        # msg.label = "MDRRR"
        msg.name = obj_name
        try:
            calibra_service = rospy.ServiceProxy(service_name, SetCalibrationCam)
            res = calibra_service(msg)
            return self.__status_interpreter_calibration[res.status]
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def get_calibration_camera(self):
        service_name = self.__services_name['get_calibration_camera_service']
        rospy.wait_for_service(service_name)

        try:
            calibra_service = rospy.ServiceProxy(service_name, GetCalibrationCam)
            res = calibra_service(Empty())
            return res
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def take_picture(self):
        service_name = self.__services_name['take_picture']
        rospy.wait_for_service(service_name)

        try:
            take_picture_service = rospy.ServiceProxy(service_name, TakePicture)
            res = take_picture_service(TakePictureRequest("~/Images/Webcam/"))
            return res
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def get_debug_markers(self):
        service_name = self.__services_name['debug_markers']
        rospy.wait_for_service(service_name)

        try:
            service = rospy.ServiceProxy(service_name, DebugMarkers)
            response = service()
            im_ret = extract_img_from_ros_msg(response.img)
            return im_ret
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def get_debug_colors(self, color):
        service_name = self.__services_name['debug_colors']
        rospy.wait_for_service(service_name)

        try:
            service = rospy.ServiceProxy(service_name, DebugColorDetection)
            response = service(color)
            im_ret = extract_img_from_ros_msg(response.img)
            return im_ret
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e


if __name__ == "__main__":
    # Creating Client Object
    vsc = VisionServicesClient()

    # Getting Object
    while "The User doesn't quit":
        # ind = (ind % 3) + 1
        img_stream = vsc.get_stream_img()
        if img_stream is None:
            continue
        cv2.imshow('cv_img', img_stream)
        img_color = vsc.get_debug_colors("BLUE")
        if img_color is not None:
            cv2.imshow('img_color', img_color)
        key = cv2.waitKey(25)
        if key in [ord("q"), 27]:
            break
