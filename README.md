# DTEK2084-open-project

## Description

Placeholder

## Installation

### Creating workspace
```
mkdir -p ~/DTEK2084-open-project_ws/src
cd ~/DTEK2084-open-project_ws/src
git clone https://github.com/npsand/DTEK2084-open-project
cd ..
```


### ROS 2
You need to install ROS2 to run this project. Tested with ROS2 Galactic. You can find instructions to install ROS2 Galactic [here](https://docs.ros.org/en/galactic/Installation.html).

### Webots
Install Webots simulator. You can find installation instructions [here](https://cyberbotics.com/doc/guide/installing-webots).

### webots-ros2 package

Install webots-ros2 package with the following command:
`sudo apt-get install ros-galactic-webots-ros2`

Alternatively you can just install webots-ros2-driver with the follwing command and the project should run just fine:
`sudo apt install ros-galactic-webots-ros2-driver`

You can replace `galactic` with `$ROS_DISTRO` commands above.

## Building
```
cd ~/DTEK2084-open-project_ws
source /opt/ros/galactic/setup.bash
colcon build
source install/setup.bash
```

## Run
```
cd ~/DTEK2084-open-project_ws
source /opt/ros/galactic/setup.bash
source install/setup.bash
ros2 launch formation_control test_launch.py

```
