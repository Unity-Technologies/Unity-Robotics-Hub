#!/usr/bin/env python
import StringIO

import rospy
import socket

from RosCommunication import RosReceiver
from RosTCPClientThread import ClientThread


class RosSubscriber(RosReceiver):
    """
    Class to send messages outside of ROS network
    """

    def __init__(self, topic, message_class, server_ip, server_port, queue_size=10):
        """

        Args:
            topic:         Topic name to publish messages to
            message_class: The message class in catkin workspace
            queue_size:    Max number of entries to maintain in an outgoing queue
        """
        self.topic = topic
        self.node_name = "{}_subsciber".format(topic)
        self.msg = message_class
        self.tcp_id = server_ip
        self.tcp_port = server_port
        self.queue_size = queue_size

        # Start Subscriber listener function
        self.listener()

    def send(self, data):
        """
        Connect to TCP endpoint on client and pass along message
        Args:
            data: message data to send outside of ROS network

        Returns:
            self.msg: The deserialize message

        """

        serialized_message = ClientThread.serialize_message(self.topic, data)

        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((self.tcp_id, self.tcp_port))
            s.send(serialized_message)
            s.close()
        except Exception as e:
            rospy.loginfo("Exception {}".format(e))

        return self.msg

    def listener(self):
        """

        Returns:

        """
        rospy.Subscriber(self.topic, self.msg, self.send)
