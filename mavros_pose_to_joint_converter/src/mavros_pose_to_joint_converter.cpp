// This is a fake joint state publisher node
// MoveIt! updates TF along with JointState in the same callback function
// This node publishes empty JointState messages

// MAVROS publishes pose calculations, based on GPS and IMUs with EKF,
// there is no need to calculate TFs based on joint states.
// MAVROS also publishes Odom->base_link transform to /tf topic.
// In this case, robot_state_publisher node only publishes the drone's frames,
// static transforms based on the URDF and Map->Odom static (0,0,0) transform.

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>


ros::Publisher joint_pub;
sensor_msgs::JointState joint_state;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    joint_state.header.stamp = ros::Time::now();

    joint_pub.publish(joint_state);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mavros_pose_to_joint_node");
    ros::NodeHandle nh;

    joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
    ros::Subscriber pose_sub = nh.subscribe("/mavros/local_position/pose", 10, poseCallback);

    // Joint names
    // MoveIt! declares joint names | Rviz -> MotionPlanning --> Joints tab

    joint_state.header.frame_id = "map";

    ros::spin();

    return 0;
}
