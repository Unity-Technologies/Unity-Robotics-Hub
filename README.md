![](images/unity-wide.png)

<p align="center"><img src="tutorials/pick_and_place/img/0_pick_place.gif" width="600"/></p>

# Unity Robotics Hub

This is a central repository for tools, tutorials, resources, and documentation for robotic simulation in Unity.

If you have questions please feel free to contact us [here](mailto:unity-robotics@unity3d.com).

> The contents of this repository are in active development. Its features and API are subject to significant change as development progresses.

## Prerequisites
- A working [ROS environment](https://www.ros.org/install/)
	- These features and demos were created using ROS Melodic and Python 2.
- Unity 2020.2 or newer

## Getting Started with Unity Robotics

| Section | Description |
|---|---|
| [Pick-and-Place End-to-End](tutorials/pick_and_place/README.md) | A complete end-to-end robotics environment - a simulated robot defined in URDF, controlled via two-way communication with ROS |
| [ROS–Unity Integration](tutorials/ros_unity_integration/README.md) | Set up simple communication between ROS and Unity |
| [URDF Importer](tutorials/urdf_importer/urdf_tutorial.md) | Steps on using the Unity plugin for loading [URDF](http://wiki.ros.org/urdf) files (Unified Robot Description Format) | 
| [Articulations Robot Demo](https://github.com/Unity-Technologies/articulations-robot-demo) | A robot simulation demonstrating Unity's new physics solver


## Component Repos

| Repo | Usage |
|---|---|---|
| [TCP Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint) | ROS node for sending/receiving messages from Unity |
| [TCP Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector) | Unity package for sending/receiving messages from ROS |

### Additional Resources

We have published a series of blog posts that are relevant for Unity Robotics:

- (November 12, 2020)
[Unite Now 2020: Simulating Robots with ROS and Unity](https://resources.unity.com/unitenow/onlinesessions/simulating-robots-with-ros-and-unity)
- (August 26, 2020)
[Announcing Unity robotic simulation](https://unity.com/solutions/automotive-transportation-manufacturing/robotics)
- (May 20, 2020)
[Use articulation bodies to easily prototype industrial designs with realistic motion and behavior](https://blogs.unity3d.com/2020/05/20/use-articulation-bodies-to-easily-prototype-industrial-designs-with-realistic-motion-and-behavior/)

In addition to robot simulation, here are some additional, relevant Unity simulation products:

| Resource | Description |
|---|---|
| [Unity Industrial Simulation](https://unity.com/products/unity-simulation) |  |
| [Training a performant object detection ML model on synthetic data using Unity Perception tools](https://blogs.unity3d.com/2020/09/17/training-a-performant-object-detection-ml-model-on-synthetic-data-using-unity-perception-tools/)|  |
| [Unity Perception Github Repository](https://github.com/Unity-Technologies/com.unity.perception) |  |
