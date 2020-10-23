# Unity ROS Initial Setup

The minimum requirements for a ROS Unity integration.

## ROS Environment

- Add the [TCP Endpoint](https://github.com/Unity-Technologies/ROS_TCP_Endpoint) package to the `src` folder in your Catkin workspace.
- Run the `catkin_make` command and source the directory

## Unity Scene
- Create a new directory and name it `Plugins` under the `Assets` directory.
- Download or clone [ROS TCP Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector) repository and copy the `TCPConnector` directory to the newly created `Plugins` directory in Unity.