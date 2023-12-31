#!/bin/bash

# Function to handle script termination
cleanup() {
    echo "Stopping all ROS nodes..."
    
    # Killing all child processes started by this script
    pkill -P $$

    # Add any other cleanup commands if necessary
    echo "Drone simulation stopped."
    exit 0
}

# Trap Ctrl+C (SIGINT) and call cleanup function
trap cleanup SIGINT

echo "Launching Gazebo, ArduPilot SITL and ROS nodes..."

# Start the warehouse launch
echo "Launching warehouse world..."
roslaunch iq_sim warehouse.launch > /dev/null 2>&1 &
sleep 10

# Launch ArduPilot SITL in a new terminal
echo "Launching SITL in a new terminal."
gnome-terminal -- ~/catkin_ws/src/drone-nav/launch_sitl.sh
sleep 10

# Launch APM (ArduPilot Master)
gnome-terminal -- roslaunch iq_sim apm.launch
sleep 15
echo "Done initalizing."


# Launch other components
echo "Launching additional components..."
roslaunch laser_to_pointcloud laser_to_pointcloud.launch > /dev/null 2>&1 &
rosrun action_controller action_controller > /dev/null 2>&1 &
roslaunch mavros_pose_to_joint_converter mavros_pose_to_joint_converter.launch > /dev/null 2>&1 &
roslaunch iris_moveit iris_moveit.launch > /dev/null 2>&1 &
python ~/catkin_ws/src/drone-nav/iq_sim/scripts/navigation_manager.py > /dev/null 2>&1 &
roslaunch ultrasonic_to_pointcloud ultrasonic_to_pointcloud.launch > /dev/null 2>&1 &

# Wait a bit to ensure everything is up
echo "Waiting 5 seconds for all components to initialize..."
sleep 5

# Launch RViz with the specific configuration
echo "Launching RViz..."
rosrun rviz rviz -d ~/catkin_ws/src/drone-nav/iq_sim/rviz/iris.rviz  > /dev/null 2>&1 &

echo -e "\e[32mThe drone simulation is up and running! \e[0m"
echo -e "\e[33mIt may take some time for Rviz to display sensor readings.\e[0m"
echo -e "\e[33mOnce the TF tree is complete, the drone is ready to take off.\e[0m"

# Keep the script running until Ctrl+C is pressed
wait

