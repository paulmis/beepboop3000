# beepboop3000 [bb3]
A cool, SLAM-based, autonomous robotic platform.

## Platform
- Raspberry Pi 3B
- raspi with [ros2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
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
colcon build --packages-select bb3
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

#### Remote control with a joystick 
To control the robot using a joystick, use [teleop_twist_joy](https://index.ros.org/p/teleop_twist_joy/github-ros2-teleop_twist_joy/):
```
sudo apt-get install ros-foxy-teleop-twist-joy
```
Follwing [this](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick) tutorial, check what device number has been assigned to your joystick:
```
ls /dev/input
```
By default joysticks start with js and are appended with consecutive integers. In my case, the joystick is assigned to `js0`. Now let's test the joystick:
```
jstest -l /dev/input/js0
```
You should see something similar to
```
crw-rw-XX- 1 root dialout 188, 0 2009-08-14 12:04 /dev/input/jsX
```
If XX is rw: the js device is configured properly.

If XX is --: the js device is not configured properly and you need to: 
```
sudo chmod a+rw /dev/input/jsX
```
Now run the `teleop_twist_joy` node:
```
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'
```
Depending on the controller its messages will be translated differently. In my case I have to press RB and move the left stick.

## How does bb3 work?
beepboop subscribes to the `cmd_vel` topic and accepts Twist messages (geometry_msgs/Twist). To steer the robot run `teleop_twist_keyboard` on the same network and press keys, or set up a joystick through teleop_twist_joy and move the robot using its controls. Fun!
