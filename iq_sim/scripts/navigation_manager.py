#!/usr/bin/env python

import rospy, time, math, threading, actionlib
from mav_msgs.msg import CommandTrajectory
from mavros_msgs.srv import WaypointPush, WaypointPushRequest
from mavros_msgs.msg import Waypoint, WaypointList, CommandCode
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped, Transform
from std_msgs.msg import Bool, Empty
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import DisplayTrajectory, RobotState, RobotTrajectory
from moveit_msgs.msg import ExecuteTrajectoryAction, ExecuteTrajectoryGoal
from action_controller.msg import MultiDofFollowJointTrajectoryActionResult
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint, MultiDOFJointTrajectory
from sensor_msgs.msg import MultiDOFJointState
from ultrasonic_to_pointcloud.msg import UltrasonicDanger
from enum import Enum



class DroneState(Enum):
    NORMAL = 1
    FAILURE = 2
    RESOLVING = 3
    SOLVED = 4

class TrajectoryConverter:
    def __init__(self):
        rospy.init_node('navigation_manager_node', anonymous=True)

        # Subscribers
        #self.subWayPoint = rospy.Subscriber('/cmd_3dnav', CommandTrajectory, self.trajectory_callback)
        self.subLocalPos = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.position_callback)
        self.subUltrasonicDanger = rospy.Subscriber('/ultrasonic/danger', UltrasonicDanger, self.ultrasonic_danger_callback)
        self.subPlannedPath = rospy.Subscriber('/move_group/display_planned_path', DisplayTrajectory, self.planned_path_callback)
        #self.subExecutionResult = rospy.Subscriber('/multi_dof_joint_trajectory_action/result', MultiDofFollowJointTrajectoryActionResult, self.execution_result_callback)

        # Publishers
        self.pubMarker = rospy.Publisher('/marker_array', MarkerArray, queue_size=10)
        self.pubSetpoint = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)
        #self.pubCmdVel = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size = 1)
        self.pubResolvingProgress = rospy.Publisher('/ultrasonic/danger_resolving_progress', Bool, queue_size=10)
        
        self.pubStartState = rospy.Publisher('/rviz/moveit/update_custom_start_state', RobotState, queue_size=10)
        self.pubGoalState = rospy.Publisher('/rviz/moveit/update_custom_goal_state', RobotState, queue_size=10)
        self.pubStartStateRviz = rospy.Publisher('/rviz/moveit/update_start_state', Empty, queue_size=10)
        self.pubGoalStateRviz = rospy.Publisher('/rviz/moveit/update_goal_state', Empty, queue_size=10)
        self.pubPlan = rospy.Publisher('/rviz/moveit/plan', Empty, queue_size=10)
        self.pubExecute = rospy.Publisher('/rviz/moveit/execute', Empty, queue_size=10)

        #Action client
        self.executeTrajectoryActionClient = actionlib.SimpleActionClient('/execute_trajectory', ExecuteTrajectoryAction)
        self.executeTrajectoryActionClient.wait_for_server()
        

        # Initialize MarkerArray for Rviz
        self.marker_array = MarkerArray()
        self.speed = rospy.get_param('~speed', 0.5)

        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.type = Marker.SPHERE_LIST
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.scale.z = 0.1
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0  # Color to RED

        # Initialize MoveGroupCommander needed to manage navigation waypoints
        self.group = MoveGroupCommander("Quad_base")

        # Variables needed for tracking drone's position,
        # local position - in which we will stop it once ultrasonics measure distance below threshold
        # goal position - this is the target once we replan
        self.localPosition = PoseStamped()
        self.safeLocalPositions = []
        self.goalPosition = PoseStamped()
        self.distance_threshold = 1.0 #[m]

        # Drone state variables
        self.state = DroneState.NORMAL
        self.resolving_lock = threading.Lock()
        self.resolving_in_progress = False
        self.topIsDanger = False
        self.bottomIsDanger = False
        self.frontIsDanger = False
        self.leftIsDanger = False
        self.rightIsDanger = False
        self.backIsDanger = False


    def position_callback(self, msg):
        self.localPosition = msg
        # Check if the current position is safe
        if not (self.topIsDanger or self.bottomIsDanger or self.frontIsDanger or
                self.leftIsDanger or self.rightIsDanger or self.backIsDanger):

            # Add to safeLocalPositions if it's either empty or the new position is sufficiently different
            if not self.safeLocalPositions or self.is_position_different(self.safeLocalPositions[-1], msg):
                self.safeLocalPositions.append(msg)

                # Limit the size of the stack if necessary
                if len(self.safeLocalPositions) > 5:
                    self.safeLocalPositions.pop(0)


    def is_position_different(self, position_one, position_two):
        # Calculate the distance between the last safe position and the current position
        dx = position_one.pose.position.x - position_two.pose.position.x
        dy = position_one.pose.position.y - position_two.pose.position.y
        dz = position_one.pose.position.z - position_two.pose.position.z
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        # Check if the distance is greater than the threshold (1 meter is defined above)
        return distance >= self.distance_threshold
        
    def planned_path_callback(self, msg):
        # Extract the last point from the trajectory, this will be the target/goal pose
        # It is saved, so can be used later if replanning is needed
        last_point = msg.trajectory[0].multi_dof_joint_trajectory.points[-1]

        # Store pose information from the last point
        if(last_point.transforms[0].translation.x != 0 and last_point.transforms[0].translation.y != 0 and last_point.transforms[0].translation.z != 0):
            self.goalPosition.header.stamp = rospy.Time.now()
            self.goalPosition.header.frame_id = msg.trajectory[0].multi_dof_joint_trajectory.header.frame_id
            self.goalPosition.pose.position.x = last_point.transforms[0].translation.x
            self.goalPosition.pose.position.y = last_point.transforms[0].translation.y
            self.goalPosition.pose.position.z = last_point.transforms[0].translation.z
            self.goalPosition.pose.orientation.x = last_point.transforms[0].rotation.x
            self.goalPosition.pose.orientation.y = last_point.transforms[0].rotation.y
            self.goalPosition.pose.orientation.z = last_point.transforms[0].rotation.z
            self.goalPosition.pose.orientation.w = last_point.transforms[0].rotation.w

            rospy.loginfo("Extracted goal pose from /move_group/display_planned_path")
        else:
            rospy.logerr("Did not save new goal pose because it was 0,0,0")

    """
    def trajectory_callback(self, msg):
        if self.state != DroneState.FAILURE:
            # Add the received point to the marker and publish the marker array
            point = Point()
            point.x = msg.pose.position.x
            point.y = msg.pose.position.y
            point.z = msg.pose.position.z
            self.marker.points.append(point)
            
            self.marker_array.markers.append(self.marker)
            self.pubMarker.publish(self.marker_array)
            
            # Publish the pose data to /mavros/setpoint_position/local
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = msg.header.frame_id
            pose.pose.position = point
            pose.pose.orientation.x = msg.pose.orientation.x
            pose.pose.orientation.y = msg.pose.orientation.y
            pose.pose.orientation.z = msg.pose.orientation.z
            pose.pose.orientation.w = msg.pose.orientation.w
            self.pubSetpoint.publish(pose)
    
            

    def execution_result_callback(self, msg):
        # msg.status.status = 3 ~~~ SUCCESS
        # Sometimes execution is not successful, but there is nothing to worry about.
        # This callback checks the result of executions and restarts navigation if needed.
        #
        # self.resolving_in_progress needs to be checked as well before restarting navigation
        # If navigation failed because ultrasonics alerted, it's being resolved already.
        #
        # But there might be a case when navigation fails because it could not even start!
        # If start position is not aligned with the current position of the drone
        # and move_group can't execute and will stop navigation rightaway.
        
        if(self.is_goal_reached() == False and self.resolving_in_progress != True):
            self.resolving_in_progress = True
            threading.Thread(target=self.resolve_failure).start()
        else:
            rospy.logerr("Won't start new thread, goal reached or resolving is in progress.")
    """        

    def ultrasonic_danger_callback(self, msg):
        if(msg.direction == "top"):
            self.topIsDanger == msg.danger
        elif(msg.direction == "bottom"):
            self.bottomIsDanger = msg.danger
        elif(msg.direction == "front"):
            self.frontIsDanger = msg.danger
        elif(msg.direction == "left"):
            self.leftIsDanger = msg.danger
        elif(msg.direction == "right"):
            self.rightIsDanger = msg.danger
        elif(msg.direction == "back"):
            self.backIsDanger = msg.danger
    
        if self.resolving_in_progress or not msg.danger:
            return  # Skip if resolution is already in progress

        with self.resolving_lock:
            self.resolving_in_progress = True
            threading.Thread(target=self.resolve_failure).start()



    def resolve_failure(self):
        try:
            self.state = DroneState.FAILURE

            # Stop execution
            self.stop_and_clear_execution()
            # Stop the drone near the detected obstacle
            self.move_to_safe_position()

            # Publishing that resolving failure is active
            resolveProgressMsg = Bool()
            resolveProgressMsg.data = True
            self.pubResolvingProgress.publish(resolveProgressMsg)

            # Plan and execute a new trajectory
            self.state = DroneState.RESOLVING
            self.execute_scan_process()
            self.plan_and_execute_trajectory()
        except Exception as e:
            rospy.logerr(f"Error in resolve_failure: {e}")
        finally:
            rospy.loginfo("\033[96mWaiting for the lock...\033[00m")
            with self.resolving_lock:
                self.resolving_in_progress = False
                rospy.loginfo("\033[96mResolving attempt.\033[0m")

    def stop_and_clear_execution(self):
        # This stops move_group to publish new trajectory waypoints for the drone to follow, since obstacles are in the way
        # /action_controller node won't keep publishing new waypoints
        # Eventually the main reason is to stop publishing onto /cmd_3dnav topic, to which this node is listening to
        self.group.stop()
        self.group.clear_pose_targets()

    def move_to_safe_position(self):
        # Publish the pose data to /mavros/setpoint_position/local
        # This is needed to stop the drone near at the position it was detecting 
        # an obstacle nearby, using ultrasonics
        pose = PoseStamped()
        if self.safeLocalPositions:
            pose = self.safeLocalPositions.pop()
        else:
            pose = self.localPosition
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        self.pubSetpoint.publish(pose)
        rospy.logwarn("Published local position to stop the drone. Waiting...")
        # The drone needs this time to stabilize after stopping so fast [7 seconds]
        rospy.sleep(7.)

    # Moves up and down and scans the area with the 2D lidar
    def execute_scan_process(self):
        i = 0
        while (not self.topIsDanger and i < 10):
            self.move_drone_vertically(0.5)
            rospy.sleep(0.5)
            i = i + 1

        i = 0
        while (not self.bottomIsDanger and i < 20):
            self.move_drone_vertically(-0.5)
            rospy.sleep(0.5)
            i = i + 1
            

    #Moves the drone vertically by a specified distance.
    #Positive distance means moving up, negative means moving down.
    def move_drone_vertically(self, distance):
        new_pose = PoseStamped()
        new_pose.header.stamp = rospy.Time.now()
        new_pose.header.frame_id = self.localPosition.header.frame_id
        new_pose.pose.position.x = self.localPosition.pose.position.x
        new_pose.pose.position.y = self.localPosition.pose.position.y
        new_pose.pose.position.z = self.localPosition.pose.position.z + distance
        new_pose.pose.orientation = self.localPosition.pose.orientation

        self.pubSetpoint.publish(new_pose)
        rospy.loginfo(f"Moving drone vertically by {distance} meters.")


    def plan_and_execute_trajectory(self):
        # Set start and goal pose using Rviz external communication topics
        # Convert PoseStamped to RobotState
        start_state_msg = self.convert_pose_to_robot_state(self.localPosition)
        goal_state_msg = self.convert_pose_to_robot_state(self.goalPosition)
        rospy.loginfo('Converted start and goal state (PoseStamped to RobotState)')
        self.pubStartState.publish(start_state_msg)
        self.pubStartStateRviz.publish(Empty())
        self.pubGoalState.publish(goal_state_msg)
        rospy.loginfo('Published start and goal state using Rviz topics (External comm)')
        
        # Set start and goal pose using Rviz external communication
        #### Plan trajectory
        self.pubPlan.publish(Empty())
        rospy.loginfo('Requested planning new trajectory. Waiting...')
        #### Wait for the plan to be ready (adjust the duration as needed)
        rospy.sleep(7.)
        #### Execute trajectory
        self.pubExecute.publish(Empty())
        rospy.loginfo('Published request for executing new trajectory.')

    def is_goal_reached(self):
        if(self.is_position_different(self.localPosition, self.goalPosition)):
            return False
        else:
            rospy.loginfo(f"\033[92mGoal Reached!\033[0m")
            return True

    def convert_pose_to_robot_state(self, pose_stamped):
        robot_state_msg = RobotState()
        robot_state_msg.multi_dof_joint_state = MultiDOFJointState()
        robot_state_msg.multi_dof_joint_state.header = pose_stamped.header
        robot_state_msg.multi_dof_joint_state.joint_names = ["virtual_joint"]

        # Create a Transform object from the Pose
        transform = Transform()
        transform.translation.x = pose_stamped.pose.position.x
        transform.translation.y = pose_stamped.pose.position.y
        transform.translation.z = pose_stamped.pose.position.z
        transform.rotation = pose_stamped.pose.orientation

        # Add the Transform to the multi-DOF joint state
        robot_state_msg.multi_dof_joint_state.transforms = [transform]
        return robot_state_msg



if __name__ == '__main__':
    try:
        converter = TrajectoryConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
