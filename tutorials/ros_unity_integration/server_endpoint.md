# ROS–Unity Integration: Server Endpoint

A walkthrough of the important components of a ROS TCP endpoint script using the `robotics_demo` package as a example.

The following is an example of a server endpoint Python script that:

- Gets parameters from `rosparam`
- Creates corresponding ROS Publisher, Subscriber, and Service objects to interact with topics and services running in ROS network
- Starts TCP Server process to handle incoming and outgoing connections


```python
#!/usr/bin/env python

import rospy

from ros_tcp_endpoint import TcpServer, RosPublisher, RosSubscriber, RosService, UnityService
from robotics_demo.msg import PosRot, UnityColor
from robotics_demo.srv import PositionService, ObjectPoseService

def main():
    ros_node_name = rospy.get_param("/TCP_NODE_NAME", 'TCPServer')
    buffer_size = rospy.get_param("/TCP_BUFFER_SIZE", 1024)
    connections = rospy.get_param("/TCP_CONNECTIONS", 10)
    tcp_server = TcpServer(ros_node_name, buffer_size, connections)
    rospy.init_node(ros_node_name, anonymous=True)

    tcp_server.start({
        'pos_rot': RosPublisher('pos_rot', PosRot, queue_size=10),
        'color': RosSubscriber('color', UnityColor, tcp_server),
        'pos_srv': RosService('pos_srv', PositionService),
        'obj_pose_srv': UnityService('obj_pose_srv', ObjectPoseService, tcp_server),
    })

    rospy.spin()


if __name__ == "__main__":
    main()
```


## Import Statements for Services and Messages
```python
from ros_tcp_endpoint import TcpServer, RosPublisher, RosSubscriber, RosService, UnityService
from robotics_demo.msg import PosRot, UnityColor
from robotics_demo.srv import PositionService, ObjectPoseService
```

## Creating the Server

Requires:

- The ROS node name

```python
    tcp_server = TcpServer(ros_node_name, buffer_size, connections)
```

The `ros_node_name` argument is required and the `buffer_size` and `connections` are optional. They are set to `1024` and `10` by default if not provided in the constructor arguments.

## Instantiate the ROS Node

```python
    rospy.init_node(ros_node_name, anonymous=True)
```

## Starting the Server

```python
    tcp_server.start({
        'pos_rot': RosPublisher('pos_rot', PosRot, queue_size=10),
        'color': RosSubscriber('color', UnityColor, tcp_server),
        'pos_srv': RosService('pos_srv', PositionService),
        'obj_pose_srv': UnityService('obj_pose_srv', ObjectPoseService, tcp_server),
    })

    rospy.spin()
```

## Source Destination Dictionary

The argument to start() is a dictionary keyed by topic or service with the corresponding ROS communication class as the value. The dictionary is used by the TCP server to direct messages to and from the ROS network.

## ROS Publisher
A ROS Publisher allows a Unity component to send messages on a given topic to other ROS nodes. It requires three components:

- Topic name
- ROS message class generated from running `catkin_make` command
- Queue size (optional)

`RosPublisher('pos_rot', PosRot, queue_size=10)`

## ROS Subscriber
A ROS Subscriber allows a Unity component to receive messages from other ROS nodes on a given topic. It requires three components:

- Topic name
- ROS message class generated from running `catkin_make` command
- The tcp server that will connect to Unity

`RosSubscriber('color', UnityColor, tcp_server)`

## ROS Service
A ROS Service is similar to a RosPublisher, in that a Unity component sends a Request message to another ROS node. Unlike a Publisher, the Unity component then waits for a Response back. It requires two components:

- Service name
- ROS Service class generated from running `catkin_make` command

`RosService('pos_srv', PositionService)`

## Unity Service

A Unity Service is similar to a RosSubscriber, in that a Unity component receives a Request message from another ROS node. It then sends a Response back. It requires three components:

- Service name
- ROS Service class generated from running `catkin_make` command
- The tcp server that will connect to Unity

`UnityService('obj_pose_srv', ObjectPoseService, tcp_server)`


## Parameters

The following parameters can be hardcoded, but for the sake of portability, we recommend setting the parameters using the `rosparam set` command, or a `rosparam` YAML file.

```python
    ros_node_name = rospy.get_param("/TCP_NODE_NAME", 'TCPServer')
    buffer_size = rospy.get_param("/TCP_BUFFER_SIZE", 1024)
    connections = rospy.get_param("/TCP_CONNECTIONS", 10)
```

In addition, the TCPServer class uses the ROS parameters ROS_IP and ROS_TCP_PORT to determine what ip & port to listen on.

> Note: Read more about the ROS Parameter Server [here](http://wiki.ros.org/Parameter%20Server).

## Launch File
An example launch file that will set the appropriate ROSPARAM values required for a parameterized TCP Endpoint script.

```
<launch>

    <env name="ROS_IP" value="127.0.0.1"/>
    <env name="ROS_HOSTNAME" value="$(env ROS_IP)"/>

    <param name="ROS_IP" type="str" value="$(env ROS_IP)" />
    <param name="ROS_TCP_PORT" type="int" value="10000" />
    <param name="TCP_NODE_NAME" type="str" value="TCPServer" />

	<group ns="position_service_and_endpoint">
	       <node pkg="robotics_demo" name="position_service" type="position_service.py"/>
	       <node pkg="robotics_demo" name="server_endpoint" type="server_endpoint.py"/>
	</group>
</launch>
```