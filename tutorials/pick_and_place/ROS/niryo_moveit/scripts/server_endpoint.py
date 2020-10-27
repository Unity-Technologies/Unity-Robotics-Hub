#!/usr/bin/env python

import rospy

from tcp_endpoint.RosTCPServer import TCPServer
from tcp_endpoint.RosPublisher import RosPublisher
from tcp_endpoint.RosSubscriber import RosSubscriber
from tcp_endpoint.RosService import RosService

from niryo_moveit.msg import NiryoMoveitJoints, NiryoTrajectory


def main():
    # Get variables set in rosparam used for
    # server/node communication
    ros_tcp_ip = rospy.get_param("/ROS_IP")
    ros_tcp_port = rospy.get_param("/ROS_TCP_PORT")

    ros_node_name = rospy.get_param("/TCP_NODE_NAME", 'TCPServer')

    unity_machine_ip = rospy.get_param("/UNITY_IP")
    unity_machine_port = rospy.get_param("/UNITY_SERVER_PORT")

    rospy.init_node(ros_node_name, anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    # Create ROS communication objects dictionary for routing messages
    source_destination_dict = {
        'SourceDestination_input': RosPublisher('SourceDestination', NiryoMoveitJoints, queue_size=10),
        'NiryoTrajectory': RosSubscriber('NiryoTrajectory', NiryoTrajectory, unity_machine_ip, unity_machine_port)
    }

    # Start the Server Endpoint
    tcp_server = TCPServer(ros_tcp_ip, ros_tcp_port, ros_node_name, source_destination_dict)
    tcp_server.start()
    rospy.spin()


if __name__ == "__main__":
    main()
