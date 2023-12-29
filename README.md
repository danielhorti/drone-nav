# Description
Experimental simulation environment in ROS for drone navigation with MoveIt!, using 2D Lidar and ultrasonic sensors.

## Project Overview
The modified version of the MoveIt! planning framework is used in this project to support aerial navigation, using OMPL's path planning algorithms for three-dimensional path planning, integrating it with the drone's control system.
ArduPilot SITL is used for simulating the flight controller of the drone and MAVROS is used as a bridge between ROS nodes and the flight controller. The drone, sensors and obstacles are simulated in Gazebo.

The drone has a 2D Lidar on top, this is used for creating a detailed OctoMap of its surroundings. This map is crucial for the initial path planning, providing a comprehensive layout of the environment. The drone can't execute automatic mapping, it has to be controlled manually to fly around in manual control mode using a teleop node for controlling the drone. Once the the map is ready, a 3D path can be generated and executed using MoveIt! through Rviz.

## Installation and Setup Instructions
This package was tested with ROS Noetic and Ubuntu 20.04.
If you don't have ROS installed, here is a detailed guide on how to install it https://wiki.ros.org/noetic/Installation/Ubuntu

Installing dependencies
```
sudo apt install git
```

### ArduPilot
Clone the official repository
```
cd ~
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
```

Install dependencies
```
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

Reload profile
```
. ~/.profile
```

ArduCopter 4.3.4 was used for testing
```
git checkout Copter-4.3.4
git submodule update --init --recursive
```

Start ArduPilot SITL to set params
```
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w
```

### Gazebo
Accept software from http://packages.osrfoundation.org
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
```

Setup keys
```
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

Reload software list
```
sudo apt update
```

Install Gazebo (Ubuntu 20.04)
```
sudo apt-get install gazebo11 libgazebo11-dev
```


Install Gazebo plugin for APM (ArduPilot Master)
```
cd ~
git clone https://github.com/khancyr/ardupilot_gazebo.git
cd ardupilot_gazebo
```

Build and install plugin
```
mkdir build
cd build
cmake ..
make -j4
sudo make install
```
```
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
```

Setup paths for models
```
echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models' >> ~/.bashrc
. ~/.bashrc
```
