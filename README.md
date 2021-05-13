# beepboop3000 [bb3]
A cool, SLAM-based, autonomous robotic platform.

## Platform
- Raspberry Pi 3B
- Ubuntu Server 20 with ROS2 Foxy
- beepboop3000

## Installation
#### Install pigpio
Download and build using cmake (from http://abyz.me.uk/rpi/pigpio/download.html)
```
wget https://github.com/joan2937/pigpio/archive/master.zip
unzip master.zip
cd pigpio-master
make
sudo make install
```

If the Python part of the install fails it may be because you need the setup tools.
```
sudo apt install python-setuptools python3-setuptools
```

#### Install ros tools
Get rosdep
```
sudo apt update
sudo apt install -y python-rosdep
sudo rosdep init # if already initialized you may continue
rosdep update
```

Get colcon
```
sudo apt install python3-colcon-common-extensions
```

#### Install bb3
Download and build using colcon
```
cd ros2_ws
source /opt/ros/foxy/setup.bash
rosdep install -i --from-path src --rosdistro foxy -y
. install/setup.bash
```

#### Run bb3
Run pigpiod (pigpio daemon)
```
sudo pigpiod
```

Run bb3:
```
ros2 run bb3 bb3
```

## How does bb3 work?
beepboop subscribes to the `cmd_vel` topic and accepts Twist messages (geometry_msgs/Twist). To steer the robot run `teleop_twist_keyboard` on the same network and press keys. Fun!
