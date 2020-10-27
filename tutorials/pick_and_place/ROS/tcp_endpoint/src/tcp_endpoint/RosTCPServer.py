#!/usr/bin/env python

import rospy
import socket

from RosTCPClientThread import ClientThread

class TCPServer:
    """
    Initializes ROS node and TCP server.
    """

    def __init__(self, tcp_ip, tcp_port, node_name, source_destination_dict, buffer_size=1024, connections=10):
        """
        Initializes ROS node and class variables.

        Args:
            tcp_ip:                  The IP used to host the TCP server
            tcp_port:                The port that should be used for incoming connections
            node_name:               ROS node name for executing code
            source_destination_dict: Dictionary of source name to instantiated RosCommunication classes
            buffer_size:             The read buffer size used when reading from a socket
            connections:             Max number of queued connections. See Python Socket documentation
        """
        self.tcp_ip = tcp_ip
        self.tcp_port = tcp_port
        self.node_name = node_name
        self.source_destination_dict = source_destination_dict
        self.buffer_size = buffer_size
        self.connections = connections

    def start(self):
        """
            Creates and binds sockets using TCP variables then listens for incoming connections.
            For each new connection a client thread will be created to handle communication.
        """
        rospy.loginfo("Starting server on {}:{}".format(self.tcp_ip, self.tcp_port))
        tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        tcp_server.bind((self.tcp_ip, self.tcp_port))
        threads = []

        while True:
            tcp_server.listen(self.connections)

            (conn, (ip, port)) = tcp_server.accept()
            new_thread = ClientThread(conn, self.source_destination_dict)
            new_thread.start()
            threads.append(new_thread)

        for t in threads:
            t.join()
