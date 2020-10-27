#!/usr/bin/env python

# To use the API, copy these 4 lines on each Python file you create
from niryo_one_python_api.niryo_one_api import *
import math
import rospy
import time
import numpy as np
import cv2


def f(compressed_image):
    np_arr = np.fromstring(compressed_image, np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    return img


rospy.init_node('niryo_one_example_python_api')
print "--- Start"

n = NiryoOne()

try:
    # Calibrate robot first
    n.calibrate_auto()
    print "Calibration finished !"

    n.activate_learning_mode(False)

    print "Go to observation position"
    observation_pose = [0.12, 0.002, 0.35, 0, 1.57, 0]
    n.move_pose(*observation_pose)

    n.wait(1)
    img = n.get_compressed_image()
    img = f(img)

    #filename = 'savedImage.jpg'
    #cv2.imwrite(filename, img)

    n.activate_learning_mode(True)

except NiryoOneException as e:
    print e
    # handle exception here
    # you can also make a try/except for each command separately

print "--- End"
