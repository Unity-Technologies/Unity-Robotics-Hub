FROM ros:melodic-ros-base

ENV ROS_WORKSPACE=/catkin_ws

# Copy each directory explicitly to avoid workspace cruft
COPY ./ros_packages/ $ROS_WORKSPACE/src/

COPY ./ros_docker/set-up-workspace /setup.sh

WORKDIR $ROS_WORKSPACE

RUN echo "ROS_IP: $(hostname -i)" > $ROS_WORKSPACE/src/ros-tcp-endpoint/config/params.yaml

# Build the workspace
RUN . /opt/ros/melodic/setup.bash && catkin_make

# Source the workspace on sign in
RUN echo ". devel/setup.bash" >> ~/.bashrc
