# ROS–Unity Initial Setup

The minimum requirements for a ROS–Unity integration.

## ROS Environment

- Download and copy the [TCP Endpoint](https://github.com/Unity-Technologies/ROS_TCP_Endpoint) package to the `src` folder in your Catkin workspace.

- Navigate to your Catkin workspace and run `catkin_make && source devel/setup.bash`. Ensure there are no errors.

- Open a new terminal, navigate to your Catkin workspace, and run:
   
   ```bash
   source devel/setup.bash
   roscore &
   ```

Once ROS Core has started, it will print `started core service [/rosout]` to the terminal window.

- Note that in the `server_endpoint`, the script fetches parameters for the TCP connection. You will need to know the IP address of your ROS machine as well as the IP address of the machine running Unity. 
   - The ROS machine IP, i.e. `ROS_IP` should be the same value as the one set as `Host Name` on the RosConnect component in Unity.
   - Finding the IP address of your local machine (the one running Unity), i.e. `UNITY_IP` depends on your operating system. 
     - On a Mac, open `System Preferences > Network`. Your IP address should be listed on the active connection.
     - On Windows, click the Wi-Fi icon on the taskbar, and open `Properties`. Your IP address should be listed near the bottom, next to "IPv4 address."

- Set the ROS parameter values by navigating to the tcp_endpoint configs, i.e. `./ROS-TCP-Endpoint/tcp_endpoint/config/params.yaml`. Open the file for editing, and update the `ROS_IP` and `UNITY_IP` with the appropriate addresses.

    ```yaml
    ROS_IP: <your ROS IP>
    ROS_TCP_PORT: 10000
    UNITY_IP: <your Unity IP>
    UNITY_SERVER_PORT: 5005
    rosdistro: 'melodic'
    ```
    
    e.g.

    ```yaml
    ROS_IP: 192.168.50.149
    ROS_TCP_PORT: 10000
    UNITY_IP: 192.168.50.13
    UNITY_SERVER_PORT: 5005
    rosdistro: 'melodic'
    ```

    Ensure that the `ROS_TCP_PORT` is set to 10000, and the `UNITY_SERVER_PORT` is set to 5005.

- Load these newly set parameters by running:

    ```bash
    rosparam load src/ROS-TCP-Endpoint/tcp_endpoint/config/params.yaml
    ```

## Unity Scene
- Create a new directory under the `Assets` directory, and name it `Plugins`.
- Download or clone [ROS TCP Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector) repository and copy the `TCPConnector` and `MessageGeneration` directories into the newly created `Plugins` directory.

Messages being passed between Unity and ROS need to be serialized exactly as ROS serializes them internally. This is achieved with the MessageGeneration plugin, which generates C# classes, including serialization and deserialization functions, based on ROS message files. Adding the `MessageGeneration` plugin should have created a new Unity menu option, “RosMessageGeneration”, which we will use to generate these messages later.