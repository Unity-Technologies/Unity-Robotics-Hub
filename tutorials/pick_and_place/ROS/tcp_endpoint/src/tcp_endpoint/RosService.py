#!/usr/bin/env python

import rospy

from rospy.service import ServiceException
from RosCommunication import RosSender


class RosService(RosSender):
    """
    Class to send messages to a ROS service.
    """
    def __init__(self, service, service_class):
        """
        Args:
            service:        The service name in ROS
            service_class:  The service class in catkin workspace
        """
        self.srv_class = service_class._request_class()
        self.srv = rospy.ServiceProxy(service, service_class)

    def send(self, data):
        """
        Takes in serialized message data from source outside of the ROS network,
        deserializes it into it's class, calls the service with the message, and returns
        the service's response.

        Args:
            data: The already serialized message_class data coming from outside of ROS

        Returns:
            service response
        """
        message = self.srv_class.deserialize(data)

        attempt = 0

        while attempt < 3:
            try:
                service_response = self.srv(message)
                return service_response
            except ServiceException:
                attempt += 1
                print("Service Exception raised. Attempt: {}".format(attempt))
            except Exception as e:
                print("Exception Raised: {}".format(e))

        return None
