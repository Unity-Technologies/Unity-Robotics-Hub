#!/usr/bin/env python

import rospy

from ros_tcp_endpoint import TCPServer, RosPublisher, RosSubscriber, RosService

from niryo_moveit.msg import NiryoMoveitJoints, NiryoTrajectory
from niryo_moveit.srv import MoverService


def main():
    ros_node_name = rospy.get_param("/TCP_NODE_NAME", 'TCPServer')
    tcp_server = TCPServer(ros_node_name)

    # Create ROS communication objects dictionary for routing messages
    tcp_server.source_destination_dict = {
        'SourceDestination_input': RosPublisher('SourceDestination', NiryoMoveitJoints, queue_size=10),
        'NiryoTrajectory': RosSubscriber('NiryoTrajectory', NiryoTrajectory, tcp_server),
        'niryo_moveit': RosService('niryo_moveit', MoverService)
    }

    # Start the Server Endpoint
    rospy.init_node(ros_node_name, anonymous=True)
    tcp_server.start()
    rospy.spin()


if __name__ == "__main__":
    main()
