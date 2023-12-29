#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import sensor_msgs.point_cloud2 as pc2
import tf
import tf2_ros
import tf2_py
import tf2_sensor_msgs
import laser_geometry.laser_geometry as lg
import math


class LidarOdometry:
    def __init__(self):
        self.laser_projector = lg.LaserProjection()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.odom_received = False
        self.odom = Odometry()

        rospy.Subscriber('lidar_topic', LaserScan, self.lidar_callback)
        rospy.Subscriber('odom_topic', Odometry, self.odom_callback)

    def lidar_callback(self, data):
        if not self.odom_received:
            return
        
        try:
            # Transform LaserScan to PointCloud
            cloud_out = self.laser_projector.projectLaser(data)

            # Perform the transformation
            transform = self.tf_buffer.lookup_transform('world', # target frame
                                                        data.header.frame_id, # source frame
                                                        rospy.Time(0), # get the tf at first available time
                                                        rospy.Duration(1.0)) # timeout after 1
                
            cloud_out = tf2_sensor_msgs.do_transform_cloud(cloud_out, transform)
            # cloud_out now contains the transformed point cloud

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("TF exception")

    def odom_callback(self, data):
        self.odom = data
        self.odom_received = True

if __name__ == "__main__":
    rospy.init_node("laserscan_to_pointcloud")
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    pc_pub = rospy.Publisher("accumulated_pc", PointCloud2, queue_size=1)
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=1)  # publisher for the marker

    # This will hold the accumulated point cloud
    accumulated_points = []

    # Set a threshold distance value for filtering
    threshold_distance = 0.1
    
    lidar_odometry = LidarOdometry()
    rospy.spin()

