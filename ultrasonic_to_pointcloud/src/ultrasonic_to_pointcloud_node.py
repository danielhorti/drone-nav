#!/usr/bin/env python3

import rospy
import math
import sys
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import Range, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Bool
from ultrasonic_to_pointcloud.msg import UltrasonicDanger

class UltrasonicToPointcloud:

    def __init__(self):
        rospy.init_node('ultrasonic_to_pointcloud')

        # Initialize PointCloud2 message
        self.pointcloud = PointCloud2()
        self.pointcloud.header.frame_id = "map"

        # Subscribers
        rospy.Subscriber("/distance_sensor/top", Range, self.top_cb)
        rospy.Subscriber("/distance_sensor/bottom", Range, self.bottom_cb)
        #rospy.Subscriber("/distance_sensor/front", Range, self.front_cb)
        #rospy.Subscriber("/distance_sensor/back", Range, self.back_cb)
        #rospy.Subscriber("/distance_sensor/left", Range, self.left_cb)
        #rospy.Subscriber("/distance_sensor/right", Range, self.right_cb)
        rospy.Subscriber("/ultrasonic/danger_resolving_progress", Bool, self.resolving_progress_cb)

        # Publishers
        self.pc_pub = rospy.Publisher("/ultrasonic/pc", PointCloud2, queue_size=10)
        self.ultraSonicDanger = rospy.Publisher("/ultrasonic/danger", UltrasonicDanger, queue_size=1)
        

        # Initialize tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.points = []
        
        # Threshold distance
        self.threshold_distance = 0.5
        self.threshold_distance_bottom = 1.0

        # Danger publishing variables
        self.isResolvingInProgress = False
        self.isDanger = False
        self.topIsDanger = False
        self.bottomIsDanger = False
        self.leftIsDanger = False
        self.rightIsDanger = False
        self.frontIsDanger = False
        self.backIsDanger = False

        # Create an instance of the generator
        self.symbols = self.rotating_symbol()

    # Generator function to yield rotating symbol, just for showing processing is active
    # Not in use - printing takes too much time
    def rotating_symbol(self):
        while True:
            for symbol in '|/-\\':
                yield symbol

    def transform_point_to_map(self, point, original_frame):
        try:
            # Get the transform from original frame to map frame
            trans = self.tf_buffer.lookup_transform("map", original_frame, rospy.Time(0), rospy.Duration(1.0))
            
            # Convert point to PointStamped
            point_stamped = PointStamped()
            point_stamped.point = point
            point_stamped.header.frame_id = original_frame

            # Transform the point using tf2
            transformed_point_stamped = tf2_geometry_msgs.do_transform_point(point_stamped, trans)

            return transformed_point_stamped.point

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("TF2 Transform error!")
            return None

    def generate_arc_points(self, msg):
        num_points = 10  
        fov_rad = 0.2967 # ~17 degrees

        for i in range(num_points):
            angle = -fov_rad / 2 + i * (fov_rad / (num_points - 1))
            point = Point(
                msg.range * math.cos(angle),
                msg.range * math.sin(angle),
                0
            )
            t_point = self.transform_point_to_map(point, msg.header.frame_id)
            if t_point:
                self.points.append(t_point)

        # Print rotating symbol without newline
        #sys.stdout.write("Processing ultrasonic sensor readings: " + next(self.symbols) + '\r')
        #sys.stdout.flush()

    def resolving_progress_cb(self, msg):
        self.isResolvingInProgress = msg.data

    def check_and_stop_drone(self, msg, sensor_direction):
        danger_flag = False
        if msg.range < self.threshold_distance or (sensor_direction == "bottom" and msg.range < self.threshold_distance_bottom):
            rospy.logwarn(f"Obstacle detected at {sensor_direction}! Stopping the drone.")
            stopMsg = UltrasonicDanger()
            stopMsg.danger = True
            stopMsg.direction = sensor_direction
            self.ultraSonicDanger.publish(stopMsg)
            danger_flag = True
            rospy.logerr(f'Measured distance below threshold ({self.threshold_distance}): {msg.range}')

    def top_cb(self, msg):
        self.check_and_stop_drone(msg, 'top')
        if msg.range < msg.max_range:
            self.generate_arc_points(msg)

    def bottom_cb(self, msg):
        self.check_and_stop_drone(msg, 'bottom')
        if msg.range < msg.max_range:
            self.generate_arc_points(msg)

    def front_cb(self, msg):
        self.check_and_stop_drone(msg, 'front')
        if msg.range < msg.max_range:
            self.generate_arc_points(msg)

    def back_cb(self, msg):
        self.check_and_stop_drone(msg, 'back')
        if msg.range < msg.max_range:
            self.generate_arc_points(msg)

    def left_cb(self, msg):
        self.check_and_stop_drone(msg, 'left')
        if msg.range < msg.max_range:
            self.generate_arc_points(msg)

    def right_cb(self, msg):
        self.check_and_stop_drone(msg, 'right')
        if msg.range < msg.max_range:
            self.generate_arc_points(msg)


    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Convert Point objects to tuple format
            point_tuples = [(p.x, p.y, p.z) for p in self.points]

            # Create PointCloud2 from points
            self.pointcloud = pc2.create_cloud_xyz32(self.pointcloud.header, point_tuples)

            # Publish the point cloud
            self.pc_pub.publish(self.pointcloud)

            # Clear points for next iteration
            self.points = []

            rate.sleep()


if __name__ == '__main__':
    node = UltrasonicToPointcloud()
    node.run()
