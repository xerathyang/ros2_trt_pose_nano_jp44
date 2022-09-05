cd /ros2_ws
rosdep install --from-paths src --ignore-src -rosdistro eloquent -y
colcon build
source install/local_setup.sh