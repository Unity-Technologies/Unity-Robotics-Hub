# ROS-Unity Integration Server Endpoint

A walkthrough of the important components of a ROS TCP endpoint script using the `robotics_demo` package as a example.

The following is an example of a server endpoint python script that:

- Gets parameters from `rosparam`
- Creates correspondng ROS Publisher, Subscriber, and Service objects to interact with topics and services runninig in in ROS network
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
	 # Get variables set in rosparam used for 
	 # server/node communication 
    ros_tcp_ip = rospy.get_param("/ROS_IP")
    ros_tcp_port = rospy.get_param("/ROS_TCP_PORT")

    ros_node_name = rospy.get_param("/TCP_NODE_NAME", 'TCPServer')

    unity_machine_ip = rospy.get_param("/UNITY_IP")
    unity_machine_port = rospy.get_param("/UNITY_SERVER_PORT")

    rospy.init_node(ros_node_name, anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    # Create ROS communication objects dictionary for routing messages
    source_destination_dict = {
        'pos_srv': RosService('position_service', PositionService),
        'pos_rot': RosPublisher('pos_rot', PosRot, queue_size=10),
        'color': RosSubscriber('color', UnityColor, unity_machine_ip, unity_machine_port),
    }

    # Start the Server Endpoint
    tcp_server = TCPServer(ros_tcp_ip, ros_tcp_port, ros_node_name, source_destination_dict)
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
    rate = rospy.Rate(10)  # 10hz
```

## ROS Publisher
A ROS Publisher requires three components:

- Topic name
- ROS message class generated from running `catkin_make` command
- Queue size

`RosPublisher('pos_rot', PosRot, queue_size=10)`
## ROS Subscriber
A ROS Subscriber requires four components:

- Topic name
- ROS message class generated from running `catkin_make` command
- IP address of machine running Unity
- Corresponding TCP port of machine running Unity

`RosSubscriber('color', UnityColor, unity_machine_ip, unity_machine_port)`

## ROS Service
A ROS Service requires two components:

- Service name
- ROS Service class generated from running `catkin_make` command

`RosService('position_service', PositionService)`

## Source Destination Dictionary

Create a dictionary keyed by topic or service with the corresponding ROS communication class as the value. The dictionary is used by the TCP server to direct messages to and from the ROS network.

```python
    source_destination_dict = {
        'pos_srv': RosService('position_service', PositionService),
        'pos_rot': RosPublisher('pos_rot', PosRot, queue_size=10),
        'color': RosSubscriber('color', Color, unity_machine_ip, unity_machine_port),
    }
```


## Starting the Server

Requires:

- IP of machine the script will be executing on
- The port to open for incoming connections
- The ROS node name
- Dictionary of service or topic names to corresponding class

```python
    tcp_server = TCPServer(ros_tcp_ip, ros_tcp_port, ros_node_name, source_destination_dict)
    tcp_server.start()
    rospy.spin()
    
```


These values can be hardcoded but for portability sake we recommend setting the parameters using the `rosparam set` command.

```python
    ros_tcp_ip = rospy.get_param("/ROS_IP")
    ros_tcp_port = rospy.get_param("/ROS_TCP_PORT")

    ros_node_name = rospy.get_param("/TCP_NODE_NAME", 'TCPServer')

    unity_machine_ip = rospy.get_param("/UNITY_IP")
    unity_machine_port = rospy.get_param("/UNITY_SERVER_PORT")
```

## Launch File
An example launch file that will set the appropriate ROSPARAM values required for a parameterized TCP Endpoint script.

```
<launch>

    <env name="ROS_IP" value="192.168.1.116"/>
    <env name="ROS_HOSTNAME" value="$(env ROS_IP)"/>
    
    <param name="ROS_IP" type="str" value="$(env ROS_IP)" />
    <param name="ROS_TCP_PORT" type="int" value="10000" />
    <param name="TCP_NODE_NAME" type="str" value="TCPServer" />

    <param name="UNITY_IP" type="str" value="192.168.1.105" />
    <param name="UNITY_SERVER_PORT" type="int" value="5005" />

	<group ns="position_service_and_endpoint">
	       <node pkg="robotics_demo" name="position_service" type="position_service.py"/>
	       <node pkg="robotics_demo" name="server_endpoint" type="server_endpoint.py"/>
	</group>
</launch>
```