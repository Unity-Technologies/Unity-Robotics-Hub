FROM ros:melodic-ros-base

ENV ROS_WORKSPACE=/catkin_ws

# Copy packages
COPY ./ros_packages/ $ROS_WORKSPACE/src/
	
RUN git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint $ROS_WORKSPACE/src/ros_tcp_endpoint -b v0.7.0

COPY ./ros_docker/set-up-workspace /setup.sh
#COPY docker/tutorial /

RUN chmod +x /setup.sh && /setup.sh && rm /setup.sh

WORKDIR $ROS_WORKSPACE

# Source the workspace on sign in
RUN echo ". devel/setup.bash" >> ~/.bashrc

# making sure the file modes are executable
RUN chmod +x src/ros_tcp_endpoint/src/ros_tcp_endpoint/*.py

#ENTRYPOINT ["/tutorial"]

