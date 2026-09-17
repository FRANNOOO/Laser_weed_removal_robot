rosdep update
sudo apt update
rosdep install --from-paths src --ignore-src -r -y
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
echo "source /workspace/install/setup.bash" >> ~/.bashrc