ARG ROS_DISTRO=jazzy
FROM gitlab-extern.atb-potsdam.de:5050/am/ros/utils/ros_base_containers/${ROS_DISTRO}-ros-base

# System tools
RUN apt update && apt install -y iputils-ping nano 

# Add a user to not stick with root
#RUN useradd -ms /bin/bash atb
RUN adduser ubuntu sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER ubuntu
