#!/usr/bin/env python

import rospy

from ros_tcp_endpoint import TcpServer, RosPublisher, RosSubscriber, RosService
from ur3_moveit.msg import *
from ur3_moveit.srv import *

def main():
    ros_node_name = rospy.get_param("/TCP_NODE_NAME", 'TCPServer')
    tcp_server = TcpServer(ros_node_name)
    rospy.init_node(ros_node_name, anonymous=True)

    # Start the Server Endpoint with a ROS communication objects dictionary for routing messages
    tcp_server.start({
        'UR3Trajectory': RosSubscriber('UR3Trajectory', UR3Trajectory, tcp_server),
        'ur3_moveit': RosService('ur3_moveit', MoverService),
        'pose_estimation_srv': RosService('pose_estimation_service', PoseEstimationService)
    })

    rospy.spin()


if __name__ == "__main__":
    main()
