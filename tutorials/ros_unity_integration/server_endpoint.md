# ROS–Unity Integration: Server Endpoint

A walkthrough of the important components of a ROS TCP endpoint script using the `robotics_demo` package as a example.

The following is an example of a server endpoint Python script that:

- Gets parameters from `rosparam`
- Creates corresponding ROS Publisher, Subscriber, and Service objects to interact with topics and services running in in ROS network
- Starts TCP Server process to handle incoming and outgoing connections


```python
#!/usr/bin/env python

import rospy

from tcp_endpoint.RosTCPServer import TCPServer
from tcp_endpoint.RosPublisher import RosPublisher
from tcp_endpoint.RosSubscriber import RosSubscriber
from tcp_endpoint.RosService import RosService

from robotics_demo.msg import PosRot, UnityColor
from robotics_demo.srv import PositionService

def main():
    ros_node_name = rospy.get_param("/TCP_NODE_NAME", 'TCPServer')
    buffer_size = rospy.get_param("/TCP_BUFFER_SIZE", 1024)
    connections = rospy.get_param("/TCP_CONNECTIONS", 10)
    tcp_server = TCPServer(ros_node_name, buffer_size, connections)

    tcp_server.source_destination_dict = {
        'pos_srv': RosService('position_service', PositionService),
        'pos_rot': RosPublisher('pos_rot', PosRot, queue_size=10),
        'color': RosSubscriber('color', UnityColor, tcp_server)
    }

    rospy.init_node(ros_node_name, anonymous=True)
    tcp_server.start()
    rospy.spin()


if __name__ == "__main__":
    main()
```


## Import Statements for Services and Messages
```python
# TCP Endpoint package imports to enable ROS communication
from tcp_endpoint.RosTCPServer import TCPServer
from tcp_endpoint.RosPublisher import RosPublisher
from tcp_endpoint.RosSubscriber import RosSubscriber
from tcp_endpoint.RosService import RosService

# Import specific ROS Messages and Services
from robotics_demo.msg import PosRot, UnityColor
from robotics_demo.srv import PositionService
```


## Instantiate the ROS Node

```python
    rospy.init_node(ros_node_name, anonymous=True)
```

## ROS Publisher
A ROS Publisher requires three components:

- Topic name
- ROS message class generated from running `catkin_make` command
- Queue size

`RosPublisher('pos_rot', PosRot, queue_size=10)`
## ROS Subscriber
A ROS Subscriber requires three components:

- Topic name
- ROS message class generated from running `catkin_make` command
- The tcp server that will connect to Unity

`RosSubscriber('color', UnityColor, tcp_server)`

## ROS Service
A ROS Service requires two components:

- Service name
- ROS Service class generated from running `catkin_make` command

`RosService('position_service', PositionService)`

## Creating the Server

Requires:

- The ROS node name

```python
    tcp_server = TCPServer(ros_node_name)
```

## Source Destination Dictionary

Create a dictionary keyed by topic or service with the corresponding ROS communication class as the value. The dictionary is used by the TCP server to direct messages to and from the ROS network.

```python
    tcp_server.source_destination_dict = {
        'pos_srv': RosService('position_service', PositionService),
        'pos_rot': RosPublisher('pos_rot', PosRot, queue_size=10),
        'color': RosSubscriber('color', Color, tcp_server),
    }
```

## Starting the Server

```python
    tcp_server.start()
    rospy.spin()
    
```


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

    <env name="ROS_IP" value="192.168.1.116"/>
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