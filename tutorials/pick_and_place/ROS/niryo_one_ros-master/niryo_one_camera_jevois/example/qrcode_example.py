#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from niryo_one_msgs.srv import SetString


class JevoisQRCodeParser:

    # Useful docs to know how to parse the data:
    # - http://jevois.org/doc/UserSerialStyle.html
    # - http://jevois.org/moddoc/DemoQRcode/modinfo.html
    def callback_jevois_data(self, msg):
        data = msg.data.split()
        # Check that it's a QRCode frame
        if data[1] != 'QR-Code':
            return

        # Get coordinates of the corners of the rectangle
        # around the qr code
        x1, y1, x2, y2, x3, y3, x4, y4 = \
            data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10]

        # Get qr code string
        qrcode_str = ' '.join(data[11:])

        rospy.loginfo('Found QR code!')
        rospy.loginfo('Coordinates of the rectangle:' +
                      ' (' + x1 + ', ' + y1 + ')' +
                      ' (' + x2 + ', ' + y2 + ')' +
                      ' (' + x3 + ', ' + y3 + ')' +
                      ' (' + x4 + ', ' + y4 + ')')
        rospy.loginfo('QR code decoded: ' + qrcode_str)
        rospy.loginfo('---')

    def load_module(self, module_name):
        rospy.wait_for_service('/niryo_one/jevois/set_module')
        try:
            load_module = rospy.ServiceProxy(
                '/niryo_one/jevois/set_module', SetString)
            load_module(module_name)
        except rospy.ServiceException as e:
            rospy.logerr(str(e))

    def __init__(self):
        self.jevois_sub = rospy.Subscriber(
            '/niryo_one/jevois/data', String, self.callback_jevois_data)

        self.load_module('qr_code')


if __name__ == '__main__':
    rospy.init_node('jevois_test_node')
    JevoisQRCodeParser()
    rospy.spin()
