#!/usr/bin/env python

import random
import rospy
from robotics_demo.msg import UnityColor


TOPIC_NAME = 'color'
NODE_NAME = 'color_publisher'


def post_color():
    pub = rospy.Publisher(TOPIC_NAME, UnityColor, queue_size=10)
    rospy.init_node(NODE_NAME, anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():

        r = random.randint(0, 255)
        g = random.randint(0, 255)
        b = random.randint(0, 255)
        color = UnityColor(r, g, b, 1)
        pub.publish(color)
        rate.sleep()
        break


if __name__ == '__main__':
    try:
        post_color()
    except rospy.ROSInterruptException:
        pass
