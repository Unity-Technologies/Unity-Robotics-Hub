# Frequently Asked Questions

How does your Unity integration compare to [ROS#](https://github.com/siemens/ros-sharp)?
---
Two of the Unity Robotics repos (URDF Importer and TCP Connector) have been forked from the Siemens ROS# repo.

In the URDF Importer we have added the functionality to instantiate a robot from URDF into a Unity scene with [Articulation Body](https://docs.unity3d.com/2020.1/Documentation/ScriptReference/ArticulationBody.html) components on their corresponding joints. 

Aside from facilitating communication with the TCP Endpoint, the TCP Connector contains the `MessageGeneration` code from ROS#. We added the extra functionality that when generating a C# class from a ROS message, functions are also generated that will serialize and deserialize the messages as ROS would internally.

How does the TCP Endpoint compare to [Rosbridge Server](http://wiki.ros.org/rosbridge_server)?
---
To put it simply, the TCP Endpoint does not have the extra overhead of having to serialize and deserialize from JSON as its only function is to pass 'ROS serialized' messages between Unity and ROS. That being said the TCP Endpoint is not as general as ROS Bridge and has the strict requirement that all messages be serialized by the TCP Connector code.

Here are some prelminary numbers from a few initial tests done during the discovery stage of this project. We will publish more test results publicly after we go through more rigorous testing but these results should be generally close enough for those curious about performance improvements.

**Note:** These tests were run on a single machine that was only running ROS and a Unity executable.

1. Sending 100 (1036x1698) images, one per frame, from Unity to ROS. The time was logged when the message was sent from Unity, before being serialized, and again when the message was received by a ROS Subscriber and deserialized into a message object.

	- ROS# with ROS Bridge Suite on average took about 10 seconds per image. The bridge slowed down dramatically after the first 10 or so messages.
	- TCP Connector with the TCP Endpoint on average took ~0.6 seconds per image.

2. Only testing the TCP Connector and TCP Endpoint to determine how long it would take to send 1000  912x1698 images, one per frame, we found that it took ~12 seconds for the ROS subscriber to receive and deserialize all messages with only ~10 messages never being received.

3. Finally a test was done calling a ROS service where the service accepts a small message of a few strings and numerics and returns the same message.

The time was logged when the message was sent from Unity before being serialized, again when the message was received and deserialized by the ROS Service, and one more time when Unity receives and deserializes the service response into the message object.

- ROS# with ROS Bridge Suite took ~2 seconds
- TCP Connector with TCP Endpoint took ~0.17 seconds
