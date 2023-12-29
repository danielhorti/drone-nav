#!/usr/bin/env python3

import sensor_msgs.point_cloud2 as pc2
import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry.laser_geometry as lg
import tf
import math

rospy.init_node("laserscan_to_pointcloud")

lp = lg.LaserProjection()

tf_listener = tf.TransformListener()

pc_pub = rospy.Publisher("converted_pc", PointCloud2, queue_size=1)

def scan_cb(msg):
    # transform the message of type LaserScan to a PointCloud2
    try:
        # wait for the necessary transform to be available
        tf_listener.waitForTransform(msg.header.frame_id, 'base_link',
                                     msg.header.stamp, rospy.Duration(1.0))
        pc2_msg = lp.transformLaserScanToPointCloud('base_link', msg, tf_listener)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logwarn("TF Exception: %s" % (e,))
        return

    # now we can do something with the PointCloud2, for example, publish it
    pc_pub.publish(pc2_msg)
    
    # rest of your code...
    
    # we can calculate the average z value for example
    print(str(sum/num))

    # or a list of the individual points which is less efficient
    point_list = list(pc2.read_points_list(pc2_msg)) # convert the generator to a list first

    # we can access the point list with an index, each element is a namedtuple
    print(point_list[len(point_list)//2].x)  # use integer division for indexing


rospy.Subscriber("/laser/scan", LaserScan, scan_cb, queue_size=1)
rospy.spin()

