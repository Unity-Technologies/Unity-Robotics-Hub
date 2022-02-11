<p align="center"><img src="images/warehouse.gif"/></p>

# Unity Robotics Hub

<!-- [![Version](https://img.shields.io/github/v/tag/Unity-Technologies/Unity-Robotics-Hub)](https://github.com/Unity-Technologies/Unity-Robotics-Hub/releases) -->
[![License](https://img.shields.io/badge/license-Apache--2.0-green.svg)](LICENSE.md)

This is a central repository for tools, tutorials, resources, and documentation for robotic simulation in Unity.

> The contents of this repository are in active development. Its features and API are subject to significant change as development progresses.

---

We're currently working on lots of things! Please take a short moment fill out our [survey](https://unitysoftware.co1.qualtrics.com/jfe/form/SV_0ojVkDVW0nNrHkW) to help us identify what products and packages to build next.

---

## Introduction

Simulation plays an important role in robotics development, and we’re here to ensure that roboticists can use Unity for these simulations. We're starting off with a set of tools to make it easier to use Unity with existing ROS-based workflows. Try out some of our samples below to get started quickly.

## Getting Started
### [Quick Installation Instructions](tutorials/quick_setup.md)

Brief steps on installing the Unity Robotics packages.

### [Pick-and-Place Tutorial](tutorials/pick_and_place/README.md)

A complete end-to-end demonstration, including how to set up the Unity environment, how to import a robot from URDF, and how to set up two-way communication with ROS for control.

### [Object Pose Estimation Tutorial](https://github.com/Unity-Technologies/Robotics-Object-Pose-Estimation)

A complete end-to-end demonstration in which we collect training data in Unity and use that data to train a deep neural network to predict the pose of a cube. This model is then deployed in a simulated robotic pick-and-place task.

### [Articulations Robot Demo](https://github.com/Unity-Technologies/articulations-robot-demo)

A robot simulation demonstrating Unity's new physics solver (no ROS dependency).

### [**New!**] [Navigation 2 SLAM Example](https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example)

An example simulation environment, integrated with ROS 2 and **[New!] Visualizations**, which enables the exercise of ROS 2's Navigation 2 and slam_toolbox packages using a simulated Turtlebot 3.

## Documentation

| Tutorial | Description |
|---|---|
| [ROS–Unity Integration](tutorials/ros_unity_integration/README.md) | A set of component-level tutorials showing how to set up communication between ROS and Unity |
| [URDF Importer](tutorials/urdf_importer/urdf_tutorial.md) | Steps on using the Unity package for loading [URDF](http://wiki.ros.org/urdf) files |
| [**New!**] [Visualizations](https://github.com/Unity-Technologies/ROS-TCP-Connector/blob/main/com.unity.robotics.visualizations/Documentation~/README.md) | Usage instructions for adding visualizations for incoming and outgoing ROS messages |

## Component Repos

| Repo | Functionality |
|---|---|
| [ROS TCP Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint) | ROS node for sending/receiving messages from Unity |
| [ROS TCP Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector) | Unity package for sending, receiving, and visualizing messages from ROS |
| [URDF Importer](https://github.com/Unity-Technologies/URDF-Importer) | Unity package for loading [URDF](http://wiki.ros.org/urdf) files |



## Additional Resources

### Blog Posts and Talks

- [**New!**] (October 4, 2021) Introducing: Unity Robotics Visualizations Package [blog post](https://blog.unity.com/manufacturing/Introducing-Unity-Robotics-Visualizations-Package)
- (August 13, 2021) Advance your robot autonomy with ROS 2 and Unity [blog post](https://blog.unity.com/manufacturing/advance-your-robot-autonomy-with-ros-2-and-unity)
- (March 2, 2021) Teaching robots to see with Unity [blog post](https://blogs.unity3d.com/2021/03/02/teaching-robots-to-see-with-unity/)
- (November 19, 2020) Robotics simulation in Unity is as easy as 1, 2, 3! [blog post](https://blogs.unity3d.com/2020/11/19/robotics-simulation-in-unity-is-as-easy-as-1-2-3/)
- (November 12, 2020)
Unite Now 2020: Simulating Robots with ROS and Unity [video](https://resources.unity.com/unitenow/onlinesessions/simulating-robots-with-ros-and-unity)
- (August 26, 2020)
Announcing Unity Robotic Simulation [blog post](https://unity.com/solutions/automotive-transportation-manufacturing/robotics)
- (May 20, 2020)
Use articulation bodies to easily prototype industrial designs with realistic motion and behavior [blog post](https://blogs.unity3d.com/2020/05/20/use-articulation-bodies-to-easily-prototype-industrial-designs-with-realistic-motion-and-behavior/)

### More from Unity

- [Unity Industrial Simulation](https://unity.com/products/unity-simulation)
- [Unity Computer Vision](https://unity.com/computer-vision)
- [Unity ML-Agents Toolkit](https://github.com/Unity-Technologies/ml-agents)

## ROS 2
ROS2 support is now available! You can get started by following [this tutorial](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/publisher.md).

## Community and Feedback

The Unity Robotics projects are open-source and we encourage and welcome contributions.
If you wish to contribute, be sure to review our [contribution guidelines](CONTRIBUTING.md)
and [code of conduct](CODE_OF_CONDUCT.md).

## Support

For questions or discussions about Unity Robotics package installations or how to best set up and integrate your robotics projects, please create a new thread on the [Unity Robotics forum](https://forum.unity.com/forums/robotics.623/) and make sure to include as much detail as possible.

For feature requests, bugs, or other issues, please file a [GitHub issue](https://github.com/Unity-Technologies/Unity-Robotics-Hub/issues) using the provided templates and the Robotics team will investigate as soon as possible.

For any other questions or feedback, connect directly with the
Robotics team at [unity-robotics@unity3d.com](mailto:unity-robotics@unity3d.com).

## Newsletter
To get notified about new updates and features, [sign up for our newsletter](https://create.unity3d.com/robotics-simulation-newsletter-sign-up)!

## FAQs
[FAQs](faq.md)

## License
[Apache License 2.0](LICENSE)
