FROM ros:noetic-ros-base

RUN sudo apt-get update && sudo apt-get install -y vim iputils-ping net-tools python3-pip ros-noetic-robot-state-publisher ros-noetic-moveit ros-noetic-rosbridge-suite ros-noetic-joy ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-tf* ros-noetic-gazebo-ros-pkgs ros-noetic-joint-state-publisher dos2unix git

RUN sudo -H pip3 --no-cache-dir install rospkg numpy jsonpickle scipy easydict torch==1.7.1+cu101 torchvision==0.8.2+cu101 torchaudio==0.7.2 -f https://download.pytorch.org/whl/torch_stable.html

ENV ROS_WORKSPACE=/catkin_ws

# Robotiq repository https://github.com/ros-industrial/robotiq.git
RUN git init $ROS_WORKSPACE/src/robotiq
WORKDIR $ROS_WORKSPACE/src/robotiq
RUN git remote add origin https://github.com/ros-industrial/robotiq.git
RUN git config core.sparseCheckout true
RUN echo "robotiq_2f_140_gripper_visualization/*" >> .git/info/sparse-checkout
RUN echo "LICENSE" >> .git/info/sparse-checkout
RUN echo ".gitignore" >> .git/info/sparse-checkout
RUN echo "CONTRIBUTING.md" >> .git/info/sparse-checkout 
RUN echo ".travis.yml" >> .git/info/sparse-checkout 
RUN git pull --depth=1 origin kinetic-devel

# Universal Robot repository https://github.com/ros-industrial/universal_robot
RUN git init $ROS_WORKSPACE/src/universal_robot
WORKDIR $ROS_WORKSPACE/src/universal_robot
RUN git remote add origin https://github.com/ros-industrial/universal_robot
RUN git config core.sparseCheckout true
RUN echo "ur_description/*" >> .git/info/sparse-checkout
RUN echo "ur_gazebo/*" >> .git/info/sparse-checkout
RUN echo ".gitignore" >> .git/info/sparse-checkout
RUN echo "CONTRIBUTING.md" >> .git/info/sparse-checkout 
RUN echo ".travis.yml" >> .git/info/sparse-checkout 
RUN git pull --depth=1 origin melodic-devel

WORKDIR $ROS_WORKSPACE

# Copy each directory explicitly to avoid workspace cruft
COPY ./ROS/src/moveit_msgs $ROS_WORKSPACE/src/moveit_msgs
COPY ./ROS/src/robotiq $ROS_WORKSPACE/src/robotiq
COPY ./ROS/src/ros_tcp_endpoint $ROS_WORKSPACE/src/ros_tcp_endpoint
COPY ./ROS/src/universal_robot $ROS_WORKSPACE/src/universal_robot
COPY ./ROS/src/ur3_moveit $ROS_WORKSPACE/src/ur3_moveit

COPY ./docker/set-up-workspace /setup.sh
COPY docker/tutorial /

RUN /bin/bash -c "find $ROS_WORKSPACE -type f -print0 | xargs -0 dos2unix"

RUN dos2unix /tutorial && dos2unix /setup.sh && chmod +x /setup.sh && /setup.sh && rm /setup.sh

# pre-load model
RUN python3 -c 'import src.ur3_moveit.src.ur3_moveit.setup_and_run_model as model; model.preload()'

ENTRYPOINT ["/tutorial"]