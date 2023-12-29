#!/usr/bin/env python3

import sensor_msgs.point_cloud2 as pc2
import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
from visualization_msgs.msg import Marker
import laser_geometry.laser_geometry as lg
import tf
import tf2_ros
import tf2_sensor_msgs
import math

rospy.init_node("laserscan_to_pointcloud")

lp = lg.LaserProjection()

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

pc_pub = rospy.Publisher("accumulated_pc", PointCloud2, queue_size=1)
marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=1)  # publisher for the marker


# This will hold the accumulated point cloud
accumulated_points = []

# Set a threshold distance value for filtering
threshold_distance = 0.1

def is_new_point(new_point):
    for existing_point in accumulated_points:
        dx = new_point[0] - existing_point[0]
        dy = new_point[1] - existing_point[1]
        dz = new_point[2] - existing_point[2]
        distance = math.sqrt(dx**2 + dy**2 + dz**2)

        if distance < threshold_distance:
            return False
    return True

def scan_cb(msg):
    # Transform the message of type LaserScan to a PointCloud2
    pc2_msg = lp.projectLaser(msg)

    # Transform point cloud to map frame
    try:
        # Wait for the necessary transform to be available
        trans = tf_buffer.lookup_transform('odom', msg.header.frame_id,
                                            msg.header.stamp, rospy.Duration(1.0))
        pc2_map_msg = tf2_sensor_msgs.do_transform_cloud(pc2_msg, trans)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn("TF Exception: %s" % (e,))
        return

    # Convert it to a generator of the individual points
    point_generator = pc2.read_points(pc2_map_msg)

    # We can access a generator in a loop
    num = 0
    for point in point_generator:
        if not math.isnan(point[2]) and is_new_point(point):
            accumulated_points.append(point)
            num += 1

    # Publish accumulated points
    header = pc2_map_msg.header
    fields = pc2_map_msg.fields
    is_dense = pc2_map_msg.is_dense
    accumulated_pc2 = pc2.create_cloud(header, fields, accumulated_points)
    pc_pub.publish(accumulated_pc2)

    # Create a marker at the center of the map frame
    marker = Marker()
    marker.header.frame_id = "map"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0

    # Publish the Marker
    marker_pub.publish(marker)
    
    

    # We can calculate the number of points in the cloud
    if(len(accumulated_points) >= 0):
        print(len(accumulated_points))
    else:
        print("Empty PointCloud")




rospy.Subscriber("/laser/scan", LaserScan, scan_cb, queue_size=1)
rospy.spin()
