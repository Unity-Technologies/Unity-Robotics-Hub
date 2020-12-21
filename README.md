<p align="center"><img src="images/warehouse.gif"/></p>

# Unity Robotics Hub

This is a central repository for tools, tutorials, resources, and documentation for robotic simulation in Unity.

> The contents of this repository are in active development. Its features and API are subject to significant change as development progresses.

## Introduction

Simulation plays an important role in robotics development, and we’re here to ensure that roboticists can use Unity for these simulations. We have released our first set of tools to make it easier to use Unity with existing ROS-based workflows. Try out some of our samples below to get started quickly.

For any questions or feedback, connect directly with the Robotics team at [unity-robotics@unity3d.com](mailto:unity-robotics@unity3d.com).

## Getting Started with Unity Robotics

| Tutorial | Description |
|---|---|
| [Quick Installation Instructions]() | Brief steps on installing the Unity Robotics packages |
| [Pick-and-Place Demo](tutorials/pick_and_place/README.md) | A complete end-to-end demonstration, including how to set up the Unity environment, how to import a robot from URDF, and how to set up two-way communication with ROS for control |
| [ROS–Unity Integration](tutorials/ros_unity_integration/README.md) | A set of component-level tutorials showing how to set up communication between ROS and Unity |
| [URDF Importer](tutorials/urdf_importer/urdf_tutorial.md) | Steps on using the Unity package for loading [URDF](http://wiki.ros.org/urdf) files (Unified Robot Description Format) | 
| [Articulations Robot Demo](https://github.com/Unity-Technologies/articulations-robot-demo) | A robot simulation demonstrating Unity's new physics solver


## Component Repos

| Repo | Usage |
|---|---|
| [TCP Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint) | ROS node for sending/receiving messages from Unity |
| [TCP Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector) | Unity package for sending/receiving messages from ROS |
| [URDF Importer](https://github.com/Unity-Technologies/URDF-Importer) | Unity package for loading [URDF](http://wiki.ros.org/urdf) files (Unified Robot Description Format) |

---

## Additional Resources

We have published a series of blog posts and talks that are relevant for Unity Robotics:

- (November 19, 2020) Robotics simulation in Unity is as easy as 1, 2, 3! [blog post](https://blogs.unity3d.com/2020/11/19/robotics-simulation-in-unity-is-as-easy-as-1-2-3/)
- (November 12, 2020)
Unite Now 2020: Simulating Robots with ROS and Unity [video](https://resources.unity.com/unitenow/onlinesessions/simulating-robots-with-ros-and-unity)
- (August 26, 2020)
Announcing Unity Robotic Simulation [blog post](https://unity.com/solutions/automotive-transportation-manufacturing/robotics)
- (May 20, 2020)
Use articulation bodies to easily prototype industrial designs with realistic motion and behavior [blog post](https://blogs.unity3d.com/2020/05/20/use-articulation-bodies-to-easily-prototype-industrial-designs-with-realistic-motion-and-behavior/) 

In addition to robot simulation, here are some additional, relevant Unity simulation products:

- Unity Industrial Simulation [site](https://unity.com/products/unity-simulation)
- Training a performant object detection ML model on synthetic data using Unity Perception tools [blog post](https://blogs.unity3d.com/2020/09/17/training-a-performant-object-detection-ml-model-on-synthetic-data-using-unity-perception-tools/)
  - Unity Perception [repository](https://github.com/Unity-Technologies/com.unity.perception)

## FAQs
- [FAQs](faq.md)

## License
[Apache License 2.0](LICENSE)