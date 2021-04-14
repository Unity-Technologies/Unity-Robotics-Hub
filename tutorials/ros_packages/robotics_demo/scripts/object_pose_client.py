#!/usr/bin/env python

from __future__ import print_function

import random
import rospy
import sys

from robotics_demo.srv import ObjectPoseService, ObjectPoseServiceResponse


def get_object_pose_client(name):
    rospy.wait_for_service('obj_pose_srv')
    try:
        get_obj_pose = rospy.ServiceProxy('obj_pose_srv', ObjectPoseService)
        obj_pose_resp = get_obj_pose(name)
        return obj_pose_resp.object_pose
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def usage():
    return "%s [object_name]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        name = str(sys.argv[1])
    else:
        print(usage())
        sys.exit(1)
    print("Requestinng pose for %s"%(name))
    print("Pose for %s: %s"%(name, get_object_pose_client(name)))
