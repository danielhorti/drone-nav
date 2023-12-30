#!/bin/bash

# Function to handle script termination
cleanup() {
    echo "Stopping all ROS nodes..."
    # Killing all child processes
    kill 0
    # Add any other cleanup commands if necessary
    echo "ROS simulation stopped."
    exit 0
}

# Trap Ctrl+C (SIGINT) and call cleanup function
trap cleanup SIGINT

echo "Launching Gazebo, ArduPilot SITL and ROS nodes..."

# Start the runway launch
echo "Launching warehouse world..."
roslaunch iq_sim warehouse.launch &
sleep 2  # Short pause to ensure it starts

# Launch sitl in a new terminal
echo "Launching SITL in a new terminal, you can take off the drone using that terminal!"
gnome-terminal -- ~/catkin_ws/src/drone-nav/launch_sitl.sh

# Launch other components
echo "Launching additional components..."
roslaunch iq_sim apm.launch &
roslaunch laser_to_pointcloud laser_to_pointcloud.launch &
rosrun action_controller action_controller &
roslaunch mavros_pose_to_joint_converter mavros_pose_to_joint_converter.launch &
roslaunch iris_moveit iris_moveit.launch &
python ~/catkin_ws/src/drone-nav/iq_sim/scripts/navigation_manager.py &

# Wait a bit to ensure everything is up
echo "Waiting 5 seconds for all components to initialize..."
sleep 5

# Launch RViz with the specific configuration
echo "Launching RViz..."
rosrun rviz rviz -d ~/catkin_ws/src/drone-nav/iq_sim/rviz/iris.rviz &

echo "The drone simulation is up and running!"

wait
