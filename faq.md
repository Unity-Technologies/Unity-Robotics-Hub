# Frequently Asked Questions & Troubleshooting

- [Frequently Asked Questions & Troubleshooting](#frequently-asked-questions--troubleshooting)
- [General Questions](#general-questions)
	- [Is ROS 2 support planned?](#is-ros-2-support-planned)
	- [How does your Unity integration compare to ROS#?](#how-does-your-unity-integration-compare-to-ros)
	- [How can I install the Unity Packages without starting from a template project?](#how-can-i-install-the-unity-packages-without-starting-from-a-template-project)
- [ROS-TCP Endpoint/Connector](#ros-tcp-endpointconnector)
	- [How does the TCP Endpoint compare to Rosbridge Server?](#how-does-the-tcp-endpoint-compare-to-rosbridge-server)
	- [I receive a `Starting server on ...` message in my ROS terminal, but see a `SocketException: A connection attempt failed because the connected party did not respond after a period of time, or established connection failed because connected host has failed to respond.` in my Unity console.](#i-receive-a-starting-server-on--message-in-my-ros-terminal-but-see-a-socketexception-a-connection-attempt-failed-because-the-connected-party-did-not-respond-after-a-period-of-time-or-established-connection-failed-because-connected-host-has-failed-to-respond-in-my-unity-console)
	- [I'm getting the error: `...failed because unknown error handler name 'rosmsg'`.](#im-getting-the-error-failed-because-unknown-error-handler-name-rosmsg)
	- [Unity can't connect to ROS!](#unity-cant-connect-to-ros)
- [URDF-Importer](#urdf-importer)
	- [I don't see an option to Import Robot from URDF, or I have compile errors upon importing the URDF-Importer.](#i-dont-see-an-option-to-import-robot-from-urdf-or-i-have-compile-errors-upon-importing-the-urdf-importer)
- [Can't find what you're looking for?](#cant-find-what-youre-looking-for)

# General Questions
Is ROS 2 support planned?
---
We definitely plan to support ROS 2 in the future. Let us know more about your use case in order to prioritize our work by reaching out to us at [unity-robotics@unity3d.com](mailto:unity-robotics@unity3d.com).

How does your Unity integration compare to [ROS#](https://github.com/siemens/ros-sharp)?
---
Two of the Unity Robotics repos (URDF Importer and TCP Connector) have been forked from the Siemens ROS# repo.

In the URDF Importer we have added the functionality to instantiate a robot from URDF into a Unity scene with [Articulation Body](https://docs.unity3d.com/2020.2/Documentation/Manual/class-ArticulationBody.html) components on their corresponding joints. 

Aside from facilitating communication with the TCP Endpoint, the TCP Connector contains the `MessageGeneration` code from ROS#. We added the extra functionality that when generating a C# class from a ROS message, functions are also generated that will serialize and deserialize the messages as ROS would internally.

You can find the ROS# description of the differences [here](https://github.com/siemens/ros-sharp/wiki/Ext_RosSharp_RoboticsHub#differences-between-unity-robotics-hub-and-ros), which also includes some of their future plans.

How can I install the Unity Packages without starting from a template project?
---
Refer to the [Quick Start](tutorials/quick_setup.md) instructions on how to import these packages.


# ROS-TCP Endpoint/Connector

How does the TCP Endpoint compare to [Rosbridge Server](http://wiki.ros.org/rosbridge_server)?
---
To put it simply, the TCP Endpoint does not have the extra overhead of having to serialize and deserialize from JSON as its only function is to pass 'ROS serialized' messages between Unity and ROS. That being said the TCP Endpoint is not as general as ROS Bridge and has the strict requirement that all messages be serialized by the TCP Connector code.

Here are some preliminary numbers from a few initial tests done during the discovery stage of this project. We will publish more test results publicly after we go through more rigorous testing but these results should be generally close enough for those curious about performance improvements.

**Note:** These tests were run on a single machine that was only running ROS and a Unity executable.

1. Sending 100 (1036x1698) images, one per frame, from Unity to ROS. The time was logged when the message was sent from Unity, before being serialized, and again when the message was received by a ROS Subscriber and deserialized into a message object.

	- ROS# with ROS Bridge Suite on average took about 10 seconds per image. The bridge slowed down dramatically after the first 10 or so messages.
	- TCP Connector with the TCP Endpoint on average took ~0.6 seconds per image.

2. Only testing the TCP Connector and TCP Endpoint to determine how long it would take to send 1000  912x1698 images, one per frame, we found that it took ~12 seconds for the ROS subscriber to receive and deserialize all messages with only ~10 messages never being received.

3. Finally a test was done calling a ROS service where the service accepts a small message of a few strings and numerics and returns the same message.

The time was logged when the message was sent from Unity before being serialized, again when the message was received and deserialized by the ROS Service, and one more time when Unity receives and deserializes the service response into the message object.

- ROS# with ROS Bridge Suite took ~2 seconds
- TCP Connector with TCP Endpoint took ~0.17 seconds

I receive a `Starting server on ...` message in my ROS terminal, but see a `SocketException: A connection attempt failed because the connected party did not respond after a period of time, or established connection failed because connected host has failed to respond.` in my Unity console.
---
This is likely an issue with how your network adapters are set up if you are using a virtual machine. You may need to do some troubleshooting to ensure that your guest OS can talk to your host OS and vice versa. One simple way to do this is to set up a "Host-Only" style network, but this varies based on what virtualization software you're using. Try to ensure that you can `ping` your host OS's IP from inside the guest, and can `ping` the guest from the host, then try this last step again. 

I'm getting the error: `...failed because unknown error handler name 'rosmsg'`.
---
This is due to a bug in an outdated package version. Try running `sudo apt-get update && sudo apt-get upgrade` to upgrade.

Unity can't connect to ROS!
---
Ensure that the ROS IP address is entered correctly as the `Host Name` in the RosConnect component in Unity, and if you are using a `params.yaml` file, that the appropriate IP addresses are filled in.

# URDF-Importer

I don't see an option to Import Robot from URDF, or I have compile errors upon importing the URDF-Importer.
---
The [ArticulationBody](https://docs.unity3d.com/2020.2/Documentation/Manual/class-ArticulationBody.html) has dependencies on Unity Editor versions [2020.2.0](https://unity3d.com/unity/whats-new/2020.2.0)+. Try updating your project to the latest 2020.2 release.

# Can't find what you're looking for?
Connect directly with the Robotics team at [unity-robotics@unity3d.com](mailto:unity-robotics@unity3d.com)!