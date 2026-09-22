![Intro to Robotics GIF](imgs/amrgif.gif)

---

# Introduction to Unity Robotics and Simulation Pro

[![License](https://img.shields.io/badge/license-Apache--2.0-green.svg)](LICENSE.md)
![Unity](https://img.shields.io/badge/unity-6.5.3-brightgreen)
![ROS](https://img.shields.io/badge/ros2-lyrical-brightgreen)

A written companion to the on-demand video course. Five modules covering the Simulation Pro package:
importing a robot, exchanging ROS 2 messages, configuring simulated sensors, connecting to a live
ROS 2 environment, and exporting the data the simulation produces.

**Start here: [Module 1 — Introduction](docs/M1-Introduction.md)**

---

## What this course is for

Unity Robotics is a set of packages that turn Unity from a game engine into a **simulation engine**
for robots. This course covers one of them, **Simulation Pro** — SimPro — which adds robotics
functionality directly to the Unity Editor.

The reason to simulate a robot in Unity is that the Editor already does the things a robot test rig
needs. You can build an environment out of prefabs, drop a robot into it, feed that robot simulated
sensor data, and watch how it behaves — before any of it touches hardware. Change the environment and
run the same robot software again. Break things in a virtual living room instead of a real one.

By the end of the course you will have built a working simulation in which a Python node running on a
separate machine navigates a robot around a Unity Scene using its LiDAR, and you will have exported
images, point clouds, and logs from that simulation to disk.

## Unity Academy

If you would rather follow along with the video tutorials, you can access
them on the [Unity Academy](https://academy.unity.com/learn/course/introduction-to-unity-robotics-and-simulation) learning platform.

All courses are built by Unity’s own training experts and Unity trainers, 
so you’re always learning from the source.

New and refreshed courses are added continuously to keep pace with Unity’s latest features and product releases.

## Course learning objectives

By the end of this course you will be able to:

1. Explain the role of Unity in robotics simulation and identify the core components of the
   Simulation Pro package.
2. Import Simulation Pro into a Unity 6 project and bring in a robot using the URDF importer.
3. Publish and receive ROS 2 messages.
4. Configure the camera, LiDAR, and IMU sensors to generate simulated sensor data.
5. Build a complete simulation that connects Unity to a live ROS 2 environment.
6. Capture and export simulation output such as sensor images, point clouds, and logged data.

## Who this is for

**Difficulty: Intermediate.** The course assumes you are comfortable in the Unity Editor and can read
and write C#. No robotics or ROS 2 experience is assumed - the concepts are introduced as they come
up.

## The modules

| Module | Covers | What you end up with |
|---|---|---|
| **[1 — Introduction](docs/M1-Introduction.md)** | Unity's role in robotics, the SimPro feature set, the URDF importer | A Unity 6 project with SimPro installed and a Panda arm imported from URDF, configured for stable physics |
| **[2 — Messages](docs/M2-Messages.md)** | Nodes, topics, the publisher/subscriber pattern, SimPro message types, TF broadcasting | A publisher and subscriber exchanging messages over a Dummy Connection, and transform data broadcasting for every joint on the arm |
| **[3 — Sensors](docs/M3-Sensors.md)** | Camera, fisheye, stereo depth, time of flight, three LiDAR implementations, IMU | Hands-on configuration and inspection of every sensor SimPro ships |
| **[4 — Creating a Simulation](docs/M4-Creating-a-Simulation.md)** | Simulation time, coordinate systems, the ROS Endpoint Connector, differential drive | A complete simulation with a live ROS 2 connection and an `rclpy` node navigating the robot by LiDAR and odometry |
| **[5 — Capturing Data](docs/M5-Capturing-Data.md)** | Image and point cloud savers, CSV logging | Images, point clouds, and CSV logs exported from the running simulation |

Work through them in order. Each module builds on the project state left by the one before it.

## Requirements

| Requirement | Detail |
|---|---|
| **Unity** | **6.5.3**. Later 6.x versions should work, but 6.5.3 is the version the course was built and tested against. |
| **Simulation Pro** | Installed through the Package Manager. See Module 1, §2. |
| **Samples** | Simulation Pro Basic and Prebuilt LiDAR Sensors. |
| **Course assets** | The Panda URDF, the material fixer script, the AMR prefab, and the ROS 2 test files. |
| **A ROS 2 environment** | **Modules 4 and 5 only.** The course uses Ubuntu 26.04 with ROS 2 Lyrical Luth on a virtual machine. |

> **Setting up ROS 2 is out of scope.** This course does not cover installing or configuring ROS 2 or
> the Unity TCP endpoint. If you want to follow Modules 4 and 5 on your own environment, 
> the [ROS 2 Wiki](https://docs.ros.org/en/lyrical/) has the installation steps.

## How to read these guides

Each module file follows the same shape:

- **Concept blocks** before each procedure, explaining what a thing is and how it works. These exist
  so that when you deviate from the steps — different robot, different sensor, different Scene — you
  can still reason about what to do.
- **Numbered procedural sections** with settings tables for every value you need to enter.
- **Checkpoints** after each meaningful section, listing observable states. If a checkpoint fails,
  fix it before continuing; later steps assume it passed.

## What comes after

- **Robotics and physics fundamentals** — driving ArticulationBodies directly, in a later course in
  this pathway.
- **ROS 2 setup** — building the environment Modules 4 and 5 assume.
- **Deployment.** The `rclpy` node in Module 4 is the same code that runs on physical hardware. Once
  it works in a Unity room, the next step is a real one.

---

## See archived content

The previous Unity Robotics Hub — the ROS 1 / ROS 2 TCP-based workflow, the Pick-and-Place tutorial,
the URDF Importer guide, and the component repo index — is still here and still maintained at its
original paths.

**[→ See archived content](ARCHIVE.md)**

## Support

For questions or discussions about Unity Robotics package installations or how to best set up and
integrate your robotics projects, please create a new thread on the
[Unity Robotics forum](https://forum.unity.com/forums/robotics.623/) and make sure to include as much
detail as possible.

For feature requests, bugs, or other issues, please file a
[GitHub issue](https://github.com/Unity-Technologies/Unity-Robotics-Hub/issues) using the provided
templates and the Robotics team will investigate as soon as possible.

## FAQs

[FAQs](faq.md)

## License

[Apache License 2.0](LICENSE.md)

---

*Course author: Sarah Applebaum. Reviewer: Isaac Seah.*
