FROM ros:melodic-ros-base

RUN sudo apt-get update && sudo apt-get install -y vim iputils-ping net-tools python-pip ros-melodic-robot-state-publisher ros-melodic-moveit ros-melodic-rosbridge-suite ros-melodic-joy ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-tf2-web-republisher

RUN sudo -H pip install rospkg jsonpickle

ENV ros_workspace=/catkin_ws

COPY ./ROS $ros_workspace

RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && cd $ros_workspace && catkin_make && source ./devel/setup.bash"

COPY docker/tutorial /

ENTRYPOINT ["/tutorial"]