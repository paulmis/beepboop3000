# beepboop3000 [bb3]
A cool, SLAM-based, smart robot. `bb3` is a proof of concept for low-cost, autonomous vehicle designed to fulfill tasks in warehouse-like environments. Built on `ros2`, it exposes steering functions and on-board sensor data over a local network, enabling the user to control and monitor it through a control panel.

## Platform
bb3 runs on raspi with [ros2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html) and is built with
- Raspberry Pi 3B
- Raspberry Pi HQ Camera with 6mm lens
- YDLIDAR X4 2D-lidar (5Hz, 0.5deg resolution up to 10m)
- x4 TT DC motors w/ 65mm rubber wheels
- x2 L298N dual h-bridges
- x16 AA NiMH 1.2V 1900mAh (in 2 x8 packs, one per bridge)
- MPU9250 9-DOF acc/gyro/mag breakout
- HC-SR04 ultrasonic sensor
- pre-cut plastic chasis with 3d-printed ABS lidar mounting plate

We're planning an upgrade for most of the components, including:
- Pi 3B -> Jetson Nano (edge visual recognition, complex ros setup)
- 65mm rubber wheels -> 80mm plastic meccanum wheels (naviagion flexibility)
- TT DC motors -> 12V 775 motors (more power)
- AA NiMH -> 4S LiPo battery pack (more power)
- L298N bridges -> breakout 4 motor bridge (lower power losses)
- plastic chasis -> fully custom chasis

<p align="center">
  <img src="https://i.imgur.com/c2YiahP.jpg" />
  <i>Current version of bb3</i>
</p>

## Installation

#### Install raspi
1. Donwload the raspi imager from [here](https://www.raspberrypi.com/software/) and burn the image onto the MicroSD card.
2. Plug the MicroSD into the pi and power. The system will take a couple of minutes to set up.
3. Connect to the local network.

#### Connect remotely
You can connect to the pi remotely on a local network through SSH. To enable SSH go to

`top left corner` -> `preferences` -> `Raspberry Pi Configuration` -> `Interfaces` -> toggle `SSH`

To find the local address type `ifconfig` in the console. The `inet` address that starts with `192.168` is the local `IPv4` address.
- `eth0` is the wired connection
- `wlan0` is the wireless connection

You can then remotely connect to the pi with `ssh username@192.168.x.x` (default username is `pi`). Note that there are issues with the wireless connection that require further configuration.

#### Install pigpio
Download and build using cmake (per [this](http://abyz.me.uk/rpi/pigpio/download.html))
```
wget https://github.com/joan2937/pigpio/archive/master.zip
unzip master.zip
cd pigpio-master
make
sudo make install
```

#### Install ros tools
1. Add in **ROS apt** repos (per [this](https://colcon.readthedocs.io/en/released/user/installation.html))
```
sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
```

2. Get **colcon**
```
sudo apt install python3-colcon-common-extensions
```

If the Python part of the install fails it may be because you need the setup tools.
```
sudo apt install python-setuptools python3-setuptools
```

3. Get **rosdep**
```
sudo apt install -y python3-rosdep2
sudo rosdep init # if already initialized you can continue
rosdep update
```

4. Get **ros2 foxy** by following [this](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html).

Note that locales do need to be updated on raspi. You can verify if locales are set up correctly with the guide [here](https://www.howtoraspberry.com/2020/04/fix-locale-problems-on-raspberry-pi/).

#### Install bb3
Download the source from `master` and build with colcon

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
