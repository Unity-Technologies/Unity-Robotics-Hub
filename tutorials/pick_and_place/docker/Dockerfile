FROM ros:melodic-ros-base

RUN sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654

RUN sudo apt-get update && sudo apt-get install -y vim iputils-ping net-tools python-pip ros-melodic-robot-state-publisher ros-melodic-moveit ros-melodic-rosbridge-suite ros-melodic-joy ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-tf2-web-republisher dos2unix

RUN sudo -H pip install rospkg jsonpickle

ENV ROS_WORKSPACE=/catkin_ws

# Copy each directory explicitly to avoid workspace cruft
COPY ./ROS/src/moveit_msgs $ROS_WORKSPACE/src/moveit_msgs
COPY ./ROS/src/niryo_moveit $ROS_WORKSPACE/src/niryo_moveit
COPY ./ROS/src/niryo_one_ros $ROS_WORKSPACE/src/niryo_one_ros
COPY ./ROS/src/niryo_one_urdf $ROS_WORKSPACE/src/niryo_one_urdf
COPY ./ROS/src/ros_tcp_endpoint $ROS_WORKSPACE/src/ros_tcp_endpoint

COPY ./docker/set-up-workspace /setup.sh
COPY docker/tutorial /

RUN /bin/bash -c "find $ROS_WORKSPACE -type f -print0 | xargs -0 dos2unix"

RUN dos2unix /tutorial && dos2unix /setup.sh && chmod +x /setup.sh && /setup.sh && rm /setup.sh

WORKDIR $ROS_WORKSPACE

# making sure the file modes are executable
RUN chmod +x src/niryo_moveit/scripts/*.py

ENTRYPOINT ["/tutorial"]

