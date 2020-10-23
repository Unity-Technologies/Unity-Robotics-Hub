## Setting Up ROS
- Download and copy the `robotics_demo` directory at `tutorials/ros_packages/` of this repo to your Catkin workspace.
- Run the `catkin_make` command and source the directory
- Run the command below to start ROS core service
	- `roscore`

- Run each of the following commands with values that reflect your current environment

```bash
    rosparam set ROS_IP YOUR_ROS_CORE_IP_OR_HOSTNAME
    rosparam set ROS_TCP_PORT 10000
    rosparam set UNITY_IP MACHINCE_RUNNING_UNITY_IP
    rosparam set UNITY_SERVER_PORT 5005
```

- Run each of the following commands in a separate terminal window:
	- `rosrun robotics_demo server_endpoint.py`
