# Pick-and-Place Tutorial: Part 0

This part provides two options for setting up your ROS workspace: using Docker, or manually setting up a catkin workspace.

**Table of Contents**
  - [Option A: Use Docker](#option-a-use-docker)
  - [Option B: Manual Setup](#option-b-manual-setup)
  - [Troubleshooting](#troubleshooting)
  - [Resources](#resources)
  - [Proceed to Part 1](#proceed-to-part-1)

---

If you have not already cloned this project to your local machine, do so now:

```bash
git clone --recurse-submodules https://github.com/Unity-Technologies/Unity-Robotics-Hub.git
```

## Option A: Use Docker

> The Docker-related files (Dockerfile, setup scripts) are located in the [`docker/`](docker/) directory.

1. Follow the steps to install [Docker Engine](https://docs.docker.com/engine/install/) for your platform if it is not already installed.

1. Start the Docker daemon.
    > Note: The system-independent `docker info` command can verify whether or not Docker is running. This command will throw a `Server: ERROR` if the Docker daemon is not currently running, and will print the appropriate [system-wide information](https://docs.docker.com/engine/reference/commandline/info/) otherwise.

1. Build the provided ROS Docker image:

    ```bash
    cd /PATH/TO/Unity-Robotics-Hub/tutorials/pick_and_place &&
    git submodule update --init --recursive &&
    docker build -t unity-robotics:pick-and-place -f docker/Dockerfile .
    ```

    > Note: The provided Dockerfile uses the [ROS Melodic base Image](https://hub.docker.com/_/ros/). Building the image will install the necessary packages, copy the [provided ROS packages and submodules](ROS/) to the container, and build the catkin workspace.

1. Start the newly built Docker container:

    ```docker
    docker run -it --rm -p 10000:10000 unity-robotics:pick-and-place /bin/bash
    ```

    When this is complete, it will print: `Successfully tagged unity-robotics:pick-and-place`. This console should open into a bash shell at the ROS workspace root, e.g. `root@8d88ed579657:/catkin_ws#`.

The ROS workspace is now ready to accept commands!

---

## Option B: Manual Setup

1. Navigate to the `/PATH/TO/Unity-Robotics-Hub/tutorials/pick_and_place/ROS` directory of this downloaded repo.
   - This directory will be used as the [ROS catkin workspace](http://wiki.ros.org/catkin/Tutorials/using_a_workspace).
   - If you cloned the project and forgot to use `--recurse-submodules`, or if any submodule in this directory doesn't have content, you can run the command `git submodule update --init --recursive` to download packages for Git submodules.
   - Copy or download this directory to your ROS operating system if you are doing ROS operations in another machine, VM, or container.
    > Note: This contains the ROS packages for the pick-and-place task, including [ROS TCP Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint), [Niryo One ROS stack](https://github.com/NiryoRobotics/niryo_one_ros), [MoveIt Msgs](https://github.com/ros-planning/moveit_msgs), `niryo_moveit`, and `niryo_one_urdf`.

1. The provided files require the following packages to be installed. ROS Melodic users should run the following commands if the packages are not already present:

   ```bash
   sudo apt-get update && sudo apt-get upgrade
   sudo apt-get install python-pip ros-melodic-robot-state-publisher ros-melodic-moveit ros-melodic-rosbridge-suite ros-melodic-joy ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-tf2-web-republisher
   sudo -H pip install rospkg jsonpickle
   ```

   ROS Noetic users should run:

   ```bash
   sudo apt-get update && sudo apt-get upgrade
   sudo apt-get install python3-pip ros-noetic-robot-state-publisher ros-noetic-moveit ros-noetic-rosbridge-suite ros-noetic-joy ros-noetic-ros-control ros-noetic-ros-controllers
   sudo -H pip3 install rospkg jsonpickle
   ```

1. If you have not already built and sourced the ROS workspace since importing the new ROS packages, navigate to your ROS workplace, and run `catkin_make && source devel/setup.bash`. Ensure there are no errors.

The ROS workspace is now ready to accept commands!

---

## Troubleshooting
- Building the Docker image may throw an `Could not find a package configuration file provided by...` exception if one or more of the directories in ROS/ appears empty. Try downloading the submodules again via `git submodule update --init --recursive`.

- `...failed because unknown error handler name 'rosmsg'` This is due to a bug in an outdated package version. Try running `sudo apt-get update && sudo apt-get upgrade` to upgrade packages.

---

## Resources
- [Getting started with Docker](https://docs.docker.com/get-started/)
- Setting up a ROS workspace:

   > Note: this tutorial has been tested with ROS Melodic as well as ROS Noetic.
   -  http://wiki.ros.org/ROS/Installation
   -  http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
   - http://wiki.ros.org/catkin/Tutorials/create_a_workspace

---


### Proceed to [Part 1](1_urdf.md).
