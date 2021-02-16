# Object Pose Estimation Demo Tutorial: Part 0

This page provides steps on how to manually set up a catkin workspace for the Pose Estimation tutorial.

1. Navigate to the `object_pose_estimation/` directory of this downloaded repository. This directory will be used as the ROS catkin workspace.

2. Copy or download this directory to your ROS operating system if you are doing ROS operations in another machine, VM, or container.

3. If the following packages are not already installed on your ROS machine, run the following commands to install them:

```bash
sudo apt-get update && sudo apt-get upgrade
sudo apt-get install python3-pip ros-noetic-robot-state-publisher ros-noetic-moveit ros-noetic-rosbridge-suite ros-noetic-joy ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-tf* ros-noetic-gazebo-ros-pkgs ros-noetic-joint-state-publisher
sudo pip3 install rospkg numpy jsonpickle scipy easydict torch==1.7.1+cu101 torchvision==0.8.2+cu101 torchaudio==0.7.2 -f https://download.pytorch.org/whl/torch_stable.html
```

> Note: If you encounter errors installing Pytorch via the above `pip3` command, try the following instead:
> ```bash 
> sudo pip3 install rospkg numpy jsonpickle scipy easydict torch==1.7.1 torchvision==0.8.2 torchaudio==0.7.2 -f https://download.pytorch.org/whl/torch_stable.html
> ```


Most of the ROS setup has been provided via the `ur3_moveit` package. This section will describe the provided files.

4. If you have not already built and sourced the ROS workspace since importing the new ROS packages, navigate to your ROS workplace, and run: 

```bash 
catkin_make
source devel/setup.bash
```

Ensure there are no unexpected errors.

The ROS parameters will need to be set to your configuration in order to allow the server endpoint to fetch values for the TCP connection. 

5. Navigate to your ROS workspace (e.g. `~/catkin_ws`). Assign the ROS IP in the `params.yaml` file as follows:

```bash
echo "ROS_IP: $(hostname -I)" > src/ur3_moveit/config/params.yaml
```

>Note: You can also manually assign this value by navigating to the `src/ur3_moveit/config/params.yaml` file and opening it for editing.
>```yaml
>ROS_IP: <your ROS IP>
>```
>e.g.
>```yaml
>ROS_IP: 10.0.0.250
>```

> Note: Learn more about the server endpoint and ROS parameters [here](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/server_endpoint.md).

This YAML file is a rosparam set from the launch files provided for this tutorial, which has been copied below for reference. Additionally, the `server_endpoint`, `pose estimation`, and `mover` nodes are launched from this file.

```xml
<launch>
    <rosparam file="$(find ur3_moveit)/config/params.yaml" command="load"/>
    '<include file="$(find ur3_moveit)/launch/demo.launch" />
    <node name="server_endpoint" pkg="ur3_moveit" type="server_endpoint.py" args="--wait" output="screen" respawn="true" />
    <node name="pose_estimation" pkg="ur3_moveit" type="pose_estimation_script.py" args="--wait" output="screen"/>
    <node name="mover" pkg="ur3_moveit" type="mover.py" args="--wait" output="screen" respawn="true" respawn_delay="2.0"/>
</launch>
```

>Note: The launch files for this project are available in the package's launch directory, i.e. `src/ur3_moveit/launch/`.

The ROS workspace is now ready to accept commands! Return to [Part 4: Set up the Unity side](4_pick_and_place.md#step-3) to continue the tutorial.
