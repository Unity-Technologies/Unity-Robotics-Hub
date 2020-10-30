# Pick and Place Tutorial

![](img/0_pick_place.gif)

This tutorial will go through the steps necessary to integrate ROS with Unity, from installing the Unity Editor to creating a scene with an imported URDF to completing a naive pick and place task using MoveIt trajectory planning. 

## Table of Contents
- [Pick and Place Tutorial](#pick-and-place-tutorial)
  - [Table of Contents](#table-of-contents)
  - [Step 1: Create Unity scene with imported URDF](#step-1-create-unity-scene-with-imported-urdf)
  - [Step 2: Unity & ROS Integration](#step-2-unity--ros-integration)
  - [Step 3: Naive Pick & Place](#step-3-naive-pick--place)
  - [MoveIt Launch & Config Files Description](moveit_file_descriptions.md)
  
---

## [Step 1: Create Unity scene with imported URDF](1_urdf.md) 

<img src="img/1_end.gif" width="400"/>

This step includes downloading and installing the Unity Editor, setting up a basic Unity scene, and importing a robot--the [Niryo One](https://niryo.com/niryo-one/)--using the URDF importer. 

---

## [Step 2: Unity & ROS Integration](2_ros_tcp.md)

<img src="img/2_echo.png" width="400"/>

This step assumes you have access to a functional ROS workspace. Steps covered include creating a TCP connection between Unity and ROS, generating C# scripts from a ROS msg and srv files, and publishing and subscribing to a ROS topic.

---

## [Step 3: Naive Pick & Place](3_naive.md)
 
<img src="img/0_pick_place.gif" width="400"/>

This step includes the preparation and setup necessary to run a naive pick and place task using MoveIt. Steps covered include creating and invoking a motion planning service in ROS, moving a Unity Articulation Body based on a calculated trajectory, and controlling a gripping tool to successfully grasp and drop an object.