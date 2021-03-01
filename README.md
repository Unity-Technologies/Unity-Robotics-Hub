<p align="center"><img src="images/warehouse.gif"/></p>

# Unity Robotics Hub

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

This is a central repository for tools, tutorials, resources, and documentation for robotic simulation in Unity.

> The contents of this repository are in active development. Its features and API are subject to significant change as development progresses.

## Introduction

Simulation plays an important role in robotics development, and we’re here to ensure that roboticists can use Unity for these simulations. We're starting off with a set of tools to make it easier to use Unity with existing ROS-based workflows. Try out some of our samples below to get started quickly.

## Getting Started
### [Quick Installation Instructions](tutorials/quick_setup.md)

Brief steps on installing the Unity Robotics packages.

### [Pick-and-Place Demo](tutorials/pick_and_place/README.md)

A complete end-to-end demonstration, including how to set up the Unity environment, how to import a robot from URDF, and how to set up two-way communication with ROS for control.

### [**New!**] [Object Pose Estimation Demo](https://github.com/Unity-Technologies/Object-Pose-Estimation)

A complete end-to-end demonstration in which we collect training data in Unity and use that data to train a deep neural network to predict the pose of a cube. This model is then deployed in a simulated robotic pick-and-place task.

### [Articulations Robot Demo](https://github.com/Unity-Technologies/articulations-robot-demo)

A robot simulation demonstrating Unity's new physics solver (no ROS dependency).
## Documentation

| Tutorial | Description |
|---|---|
| [ROS–Unity Integration](tutorials/ros_unity_integration/README.md) | A set of component-level tutorials showing how to set up communication between ROS and Unity |
| [URDF Importer](tutorials/urdf_importer/urdf_tutorial.md) | Steps on using the Unity package for loading [URDF](http://wiki.ros.org/urdf) files | 


## Component Repos

| Repo | Functionality |
|---|---|
| [ROS TCP Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint) | ROS node for sending/receiving messages from Unity |
| [ROS TCP Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector) | Unity package for sending/receiving messages from ROS |
| [URDF Importer](https://github.com/Unity-Technologies/URDF-Importer) | Unity package for loading [URDF](http://wiki.ros.org/urdf) files |



## Additional Resources

### Blog Posts and Talks

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

## Coming Soon - New Physics Features in Unity!
Here’s a peek into what our Physics Team is hard at work on…

- **Contact Modification API**. This API will allow users to define custom contact reactions, such as ignoring subsets of contact points, in order to help simulate holes, slippery surfaces, soft contacts, and more. [Read more about the new Contact Modification API](https://forum.unity.com/threads/experimental-contacts-modification-api.924809/).
- **Wheel Collider shapes**. This feature will allow the user to specify the shape of the collider to be used for collision detection. Currently the collider shape is fixed to a cylinder, and collision detection is performed by casting a ray from the center of the cylinder. Custom shapes will improve the accuracy of simulating wheels over rough terrains, holes, etc. [Read more about Wheel Collider](https://docs.unity3d.com/Manual/class-WheelCollider.html).
- **Collision detection modes exposed for ArticulationBody: discrete, sweep-based ccd, and speculative ccd**. New continuous collision detection (ccd) modes will ensure that fast-moving objects collide with objects, instead of tunneling or passing through those objects, which can happen in the default “discrete” mode. [Read more about continuous collision detection](https://docs.unity3d.com/Manual/ContinuousCollisionDetection.html).
- **Force/Torque Sensor API**. This API will allow users to get the force and torque acting on an articulation body (useful for simulating a force/torque sensor!), as well as to get the motor torque applied by an articulation drive.
- **Query primitives**. These simple, GameObject-less shapes allow for collision detection without requiring simulation (i.e., without calling Physics.Simulate). This feature will allow users to initialize objects in feasible locations, and can also be used for motion planning.

## ROS 2
Interested in early access to ROS 2 integration? Email [unity-robotics@unity3d.com](mailto:unity-robotics@unity3d.com) to join our alpha program.

## FAQs
[FAQs](faq.md)

## Support
For general questions, feedback, or feature requests, connect directly with the Robotics team at [unity-robotics@unity3d.com](mailto:unity-robotics@unity3d.com).

For bugs or other issues, please file a GitHub issue and the Robotics team will investigate the issue as soon as possible.

## License
[Apache License 2.0](LICENSE)