# in ros2_ws
# compile
source /opt/ros/foxy/setup.bash
rosdep install -i --from-path src --rosdistro foxy -y
colcon build --packages-select bb3
. install/setup.bash

# run
sudo pigpiod
ros2 run bb3 bb3
