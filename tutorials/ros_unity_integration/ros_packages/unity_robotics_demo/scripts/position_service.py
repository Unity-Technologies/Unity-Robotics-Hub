#!/usr/bin/env python

from __future__ import print_function

import random
import rospy

from unity_robotics_demo_msgs.srv import PositionService, PositionServiceResponse


def new_position(req):
    print("Request: \n{}".format(req.input))
    req.input.pos_x = random.uniform(-4.0, 4.0)
    req.input.pos_z = random.uniform(-4.0, 4.0)

    return PositionServiceResponse(req.input)


def translate_position_server():
    rospy.init_node('position_server')
    s = rospy.Service('pos_srv', PositionService, new_position)
    print("Ready to move cubes!")
    rospy.spin()


if __name__ == "__main__":
    translate_position_server()
