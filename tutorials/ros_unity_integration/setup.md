# ROS–Unity Initial Setup

The minimum requirements for a ROS–Unity integration.

## ROS Environment

1. Follow these steps if using ROS (melodic or noetic):

1. Download and copy the [TCP Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint) package to the `src` folder in your Catkin workspace.

1. Navigate to your Catkin workspace and run `catkin_make && source devel/setup.bash`. Ensure there are no errors.

1. Open a new terminal, navigate to your Catkin workspace, and run:

   ```bash
   source devel/setup.bash
   roscore &
   ```

Once ROS Core has started, it will print `started core service [/rosout]` to the terminal window.

1. Note that in the `server_endpoint`, the script fetches parameters for the TCP connection. You will need to know the IP address of your ROS machine, which should be the same value as the one set as `Host Name` on the RosConnect component in Unity.

1. The ROS parameter values can be set using a YAML file. Create a `params.yaml` file in your package, e.g. `./config/params.yaml`. Open the file for editing.

1. Update the `ROS_IP` below with the appropriate address and copy the contents into the `params.yaml` file.

    ```yaml
    ROS_IP: <your ROS IP>
    ROS_TCP_PORT: 10000
    ```

    e.g.

    ```yaml
    ROS_IP: 127.0.0.1
    ROS_TCP_PORT: 10000
    ```

    Ensure that the `ROS_TCP_PORT` is set to 10000.

1. Set these newly defined parameters by running `rosparam load`, e.g.:

    ```bash
    rosparam load PATH/TO/config/params.yaml
    ```
    Alternatively, this YAML can be loaded from a launch file, e.g.:

    ```xml
    <launch>
        <rosparam file="$(find <PACKAGE_NAME>)/config/params.yaml" command="load"/>
    </launch>
    ```

> Read more about rosparam YAML options [here](http://wiki.ros.org/rosparam).
>
> Read more about the ROS Parameter Server [here](http://wiki.ros.org/Parameter%20Server).

## ROS2 Environment

1. Follow these steps if using ROS2:

1. Download the [ROS2 TCP Endpoint](https://github.com/Unity-Technologies/ROS2-TCP-Endpoint) repository and copy the folders `ROS2_packages/ros2_tcp_endpoint` and `ROS2_packages/unity_interfaces` into the `src` folder in your Colcon workspace.

1. Navigate to your Colcon workspace and run `colcon build && source install/setup.bash`. Ensure there are no errors.

1. Note that in the `server_endpoint`, the script fetches parameters for the TCP connection. You will need to know the IP address of your ROS2 machine - this should be the same value as the one set as `Host Name` on the RosConnect component in Unity.

## Unity Scene
1. Launch Unity and create a new scene.
2. Open Package Manager and click the + button at the top left corner. Select "add package from git URL" and enter "https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector" to install the [ROS TCP Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector) package.

![](images/add_package.png)

![](images/add_package_2.png)