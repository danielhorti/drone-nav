#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry

def odom_callback(msg):
    br = tf.TransformBroadcaster()
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    br.sendTransform((position.x, position.y, position.z),
                     (orientation.x, orientation.y, orientation.z, orientation.w),
                     rospy.Time.now(),
                     "odom",
                     "map")

def main():
    rospy.init_node('odom_to_map_tf_broadcaster')
    rospy.Subscriber('/mavros/local_position/odom', Odometry, odom_callback)
    rospy.spin()

if __name__ == '__main__':
    main()

