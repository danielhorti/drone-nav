# Description
Experimental simulation environment in ROS for drone navigation with MoveIt!, using 2D Lidar and ultrasonic sensors.

## Project Overview
The modified version of the MoveIt! planning framework is used in this project to support aerial navigation, using OMPL's path planning algorithms for three-dimensional path planning, integrating it with the drone's control system.
ArduPilot SITL is used for simulating the flight controller of the drone and MAVROS is used as a bridge between ROS nodes and the flight controller. The drone, sensors and obstacles are simulated in Gazebo.

The drone has a 2D Lidar on top, this is used for creating a detailed OctoMap of its surroundings. This map is crucial for the initial path planning, providing a comprehensive layout of the environment. The drone can't execute automatic mapping, it has to be controlled manually to fly around in manual control mode using a teleop node for controlling the drone. Once the the map is ready, a 3D path can be generated and executed using MoveIt! through Rviz.
