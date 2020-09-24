#!/usr/bin/env python

import rospy

from tcp_endpoint.RosTCPServer import TCPServer
from tcp_endpoint.RosPublisher import RosPublisher
from tcp_endpoint.RosSubscriber import RosSubscriber
from tcp_endpoint.RosService import RosService

from robotics_demo.msg import PosRot, UnityColor
from robotics_demo.srv import PositionService


def main():
    ros_tcp_ip = rospy.get_param("/ROS_IP")
    ros_tcp_port = rospy.get_param("/ROS_TCP_PORT")

    ros_node_name = rospy.get_param("/TCP_NODE_NAME", 'TCPServer')
    buffer_size = rospy.get_param("/TCP_BUFFER_SIZE", 1024)
    connections = rospy.get_param("/TCP_CONNECTIONS", 10)

    unity_machine_ip = rospy.get_param("/UNITY_IP")
    unity_machine_port = rospy.get_param("/UNITY_SERVER_PORT")

    rospy.init_node(ros_node_name, anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    source_destination_dict = {
        'pos_srv': RosService('position_service', PositionService),
        'pos_rot': RosPublisher('pos_rot', PosRot, queue_size=10),
        'color': RosSubscriber('color', UnityColor, unity_machine_ip, unity_machine_port),
    }

    tcp_server = TCPServer(ros_tcp_ip, ros_tcp_port, ros_node_name, source_destination_dict, buffer_size, connections)
    tcp_server.start()
    rospy.spin()


if __name__ == "__main__":
    main()
