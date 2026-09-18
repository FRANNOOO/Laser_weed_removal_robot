rosdep update
sudo apt update
rosdep install --from-paths src --ignore-src -r -y
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_zenoh_cpp" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
echo "source /workspace/install/setup.bash" >> ~/.bashrc