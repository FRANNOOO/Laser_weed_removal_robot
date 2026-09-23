ARG ROS_DISTRO=jazzy
FROM gitlab-extern.atb-potsdam.de:5050/am/ros/utils/ros_base_containers/${ROS_DISTRO}-ros-base

# Install dependencies
RUN apt update && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY ./src /workspace/src
RUN rosdep update  --rosdistro=${ROS_DISTRO}&& apt update && rosdep install --from-paths src --ignore-src -r -y
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build

# Add source to bashrc
RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT [ "/ros_entrypoint.sh" ]