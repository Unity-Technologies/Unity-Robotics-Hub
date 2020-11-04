# Unity ROS Initial Setup

The minimum requirements for a ROS Unity integration.

## ROS Environment

- Download and copy the [TCP Endpoint](https://github.com/Unity-Technologies/ROS_TCP_Endpoint) package to the `src` folder in your Catkin workspace.
- Copy the `tutorials/ros_packages/robotics_demo` folder of this repo into the `src` folder in your Catkin workspace. (You can skip this step if you're not interested in following the ROS Unity Integration tutorials.)

- Run `cd ~/catkin_ws/ && catkin_make && source devel/setup.bash`. Ensure there are no errors.

- Open a new terminal and run:
   ```bash
   cd ~/catkin_ws/
   source devel/setup.bash
   roscore &
   ```

Once ROS Core has started, it will print `started core service [/rosout]` to the terminal window.

- Note that in the `server_endpoint`, the script fetches parameters for the TCP connection. You will need to know the IP address of your ROS machine as well as the IP address of the machine running Unity. 
   - The ROS machine IP, i.e. `ROS_IP` should be the same value as the one set as `hostName` on the RosConnect component in Unity.
   - Finding the IP address of your local machine (the one running Unity), i.e. `UNITY_IP` depends on your operating system. 
     - On a Mac, open `System Preferences > Network`. Your IP address should be listed on the active connection.
     - On Windows, click the Wi-Fi icon on the taskbar, and open `Properties`. Your IP address should be listed near the bottom, next to "IPv4 address."

- Set the parameter values by running the following commands:

   ```bash
   rosparam set ROS_IP <your ROS IP>
   rosparam set ROS_TCP_PORT 10000
   rosparam set UNITY_IP <your Unity IP>
   rosparam set UNITY_SERVER_PORT 5005
   ```
   e.g.,

   ```bash
   rosparam set ROS_IP 192.168.50.149
   rosparam set ROS_TCP_PORT 10000
   rosparam set UNITY_IP 192.168.50.13
   rosparam set UNITY_SERVER_PORT 5005
   ```

- Once the parameter values have been set, you can run the server_endpoint. In this same terminal, run:
  
   ```bash
   rosrun robotics_demo server_endpoint.py
   ```
Once the server_endpoint has started, it will print something similar to `[INFO] [1603488341.950794]: Starting server on 192.168.50.149:10000`.

## Unity Scene
- Create a new directory under the `Assets` directory, and name it `Plugins`.
- Download or clone [ROS TCP Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector) repository and copy the `TCPConnector` and `MessageGeneration` directories into the newly created `Plugins` directory.

Messages being passed between Unity and ROS need to be serialized exactly as ROS serializes them internally. This is achieved with the MessageGeneration plugin, which generates C# classes, including serialization and deserialization functions, based on ROS message files. Adding the `MessageGeneration` plugin should have created a new Unity menu option, “RosMessageGeneration”, which we will use to generate these messages later.