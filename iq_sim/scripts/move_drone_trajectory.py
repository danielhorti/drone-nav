#!/usr/bin/env python

import rospy
import tf
import math
import sys
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import Thrust  # Import the correct message type
from std_msgs.msg import Float64

class WaypointConverter:

    def __init__(self):
        rospy.init_node('waypoint_converter')
        
        self.sub = rospy.Subscriber('/move_group/display_planned_path', DisplayTrajectory, self.callback)
        self.pose_pub = rospy.Publisher('/mavros/setpoint_attitude/attitude', PoseStamped, queue_size=10)
        self.thrust_pub = rospy.Publisher('/mavros/setpoint_attitude/thrust', Thrust, queue_size=10)  # Update the message type here

    def callback(self, msg):
        print('Press "s" to start path execution' )
        key = None
        while(key == None or key != 's'):
            key = sys.stdin.read(1)

        trajectory = msg.trajectory[0]  # Assuming a single trajectory in the message
        
        waypoints = trajectory.multi_dof_joint_trajectory.points

        for i in range(len(waypoints) - 1):
            current_point = waypoints[i].transforms[0].translation
            next_point = waypoints[i + 1].transforms[0].translation

            # Calculate yaw to face the next waypoint
            yaw = self.calculate_yaw(current_point, next_point)

            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.pose.position = current_point
            pose.pose.orientation = tf.transformations.quaternion_from_euler(0, 0, yaw)  # Roll and pitch are set to 0

            thrust_msg = Thrust()
            thrust_msg.thrust = 0.5  # Placeholder, adjust as needed

            self.pose_pub.publish(pose)
            self.thrust_pub.publish(thrust_msg)  # Publish the thrust message
            
            # You might want to add a sleep here to control the rate at which waypoints are sent

    def calculate_yaw(self, current, next):
        dx = next.x - current.x
        dy = next.y - current.y
        return math.atan2(dy, dx)

if __name__ == '__main__':
    converter = WaypointConverter()
    rospy.spin()
