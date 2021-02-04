#!/usr/bin/env python


import rospy

from tcp_endpoint.RosTCPServer import TCPServer
from tcp_endpoint.RosPublisher import RosPublisher
from tcp_endpoint.RosSubscriber import RosSubscriber
from tcp_endpoint.RosService import RosService

from ur3_moveit.msg import *
from ur3_moveit.srv import *

def main():
    ros_node_name = rospy.get_param("/TCP_NODE_NAME", 'TCPServer')
    tcp_server = TCPServer(ros_node_name)

    # Create ROS communication objects dictionary for routing messages
    tcp_server.source_destination_dict = {
        'UR3Trajectory': RosSubscriber('UR3Trajectory', UR3Trajectory, tcp_server),
        'ur3_moveit': RosService('ur3_moveit', MoverService),
        'pose_estimation_srv': RosService('pose_estimation_service', PoseEstimationService)
    }

    # Start the Server Endpoint
    rospy.init_node(ros_node_name, anonymous=True)
    tcp_server.start()
    rospy.spin()


if __name__ == "__main__":
    main()
