#!/usr/bin/env python3

import sensor_msgs.point_cloud2 as pc2
import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry.laser_geometry as lg
import math
import tf2_ros
import tf2_geometry_msgs
import tf2_sensor_msgs
import sys

rospy.init_node("laserscan_to_pointcloud")

lp = lg.LaserProjection()

pc_pub = rospy.Publisher("converted_pc", PointCloud2, queue_size=1)

# Initialize tf2 buffer and listener
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

# Generator function to yield rotating symbol, just for showing processing is active 
def rotating_symbol():
    while True:
        for symbol in '|/-\\':
            yield symbol

# Create an instance of the generator
symbols = rotating_symbol()

def scan_cb(msg):
    # convert the message of type LaserScan to a PointCloud2
    pc2_msg = lp.projectLaser(msg)

    # Transform the PointCloud2 message to the "map" frame
    try:
        # Wait for the transform to be available
        transform = tf_buffer.lookup_transform('map', msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
        # Transform the point cloud to the "map" frame
        transformed_pc2_msg = tf2_sensor_msgs.do_transform_cloud(pc2_msg, transform)
        # Publish the transformed PointCloud2 message
        pc_pub.publish(transformed_pc2_msg)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr('Transform error: %s', e)
        return
    
    # convert it to a generator of the individual points
    point_generator = pc2.read_points(pc2_msg)
    

    # Print rotating symbol without newline
    #sys.stdout.write("Processing laser sensor readings: " + next(symbols) + '\r')
    #sys.stdout.flush()



rospy.Subscriber("/laser/scan", LaserScan, scan_cb, queue_size=1)
rospy.spin()

