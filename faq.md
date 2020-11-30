# Frequently Asked Questions

How does your Unity integration compare to [ROS#](https://github.com/siemens/ros-sharp)?
---
To provide a little bit of a background. Two of the repos we have just released, URDF Importer and TCP Connector, have been forked from the Siemens ROS# repo.

In the URDF Importer we have added the functionality to instantiate a robot from URDF into a Unity scene with [Articulation Body](https://docs.unity3d.com/2020.1/Documentation/ScriptReference/ArticulationBody.html) components on their corresponding joints. 

Aside from facilitating communication with the TCP Endpoint, the TCP Connector contains the `MessageGeneration` code from ROS#. We added the extra functionality that when generating a C# class from a ROS message, functions are also generated that will serialize and deserialize the messages as ROS would internally.

How does the TCP Endpoint compare to [Rosbridge Server](http://wiki.ros.org/rosbridge_server)?
---
To put it simply, the TCP Endpoint does not have the extra overhead of having to serialize and deserialize from JSON as its only function is to pass 'ROS serialized' messages between Unity and ROS. That being said the TCP Endpoint is not as general as ROS Bridge and has the strict requirement that all messages be serialized by the TCP Connector code.

