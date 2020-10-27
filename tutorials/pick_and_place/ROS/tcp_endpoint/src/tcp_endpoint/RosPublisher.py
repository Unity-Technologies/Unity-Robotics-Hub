#!/usr/bin/env python

import rospy

from RosCommunication import RosSender


class RosPublisher(RosSender):
    """
    Class to publish messages to a ROS topic
    """
    # TODO: surrface latch functionality
    def __init__(self, topic, message_class, queue_size=10):
        """

        Args:
            topic:         Topic name to publish messages to
            message_class: The message class in catkin workspace
            queue_size:    Max number of entries to maintain in an outgoing queue
        """
        self.msg = message_class()
        self.pub = rospy.Publisher(topic, message_class, queue_size=queue_size)

    def send(self, data):
        """
        Takes in serialized message data from source outside of the ROS network,
        deserializes it into it's message class, and publishes the message to ROS topic.

        Args:
            data: The already serialized message_class data coming from outside of ROS

        Returns:
            None: Explicitly return None so behaviour can be
        """
        self.msg.deserialize(data)
        self.pub.publish(self.msg)

        return None
