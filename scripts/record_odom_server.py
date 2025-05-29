#!/usr/bin/env python3
import rospy
import actionlib
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point32
from tf.transformations import euler_from_quaternion
from wall_follower.msg import OdomRecordAction, OdomRecordFeedback, OdomRecordResult

class OdomRecordServer:
    def __init__(self):
        rospy.loginfo("OdomRecordActionServer: Initializing...")
        self._action_name = 'record_odom'
        self._as = actionlib.SimpleActionServer(self._action_name, OdomRecordAction, execute_cb=self.execute_cb, auto_start=False)

        # parameters for lap detection
        self.pos_tolerance = rospy.get_param("~lap_completion_pos_tolerance", 0.5)
        self.orient_tolerance = rospy.get_param("~lap_completion_orient_tolerance", 0.35)
        self.min_dist_for_lap_check = rospy.get_param("~min_dist_for_lap_check", 1.0)

        self._feedback = OdomRecordFeedback()
        self._result = OdomRecordResult()
        
        self.list_of_odoms_recorded = []
        self.current_total_distance = 0.0
        self.last_odom_point = None
        self.start_odom_pose = None
        self.timer = None # For odom recording
        self._as.start()

    def cleanup_after_goal(self):
        """
        Resets state variables and stops the timer.
        Called when a goal succeeds, is preempted, aborts, or a new goal starts.
        """
        if self.timer:
            self.timer.shutdown()
            self.timer = None
        
        self.list_of_odoms_recorded = []
        self.current_total_distance = 0.0
        self.last_odom_point = None
        self.start_odom_pose = None

    def execute_cb(self, goal):  
        self.cleanup_after_goal()
        # define the start of the lap.
        try:
            initial_odom_msg = rospy.wait_for_message("/odom", Odometry, timeout=5.0)
            
            self.start_odom_pose = Point32()
            self.start_odom_pose.x = initial_odom_msg.pose.pose.position.x
            self.start_odom_pose.y = initial_odom_msg.pose.pose.position.y
            q = initial_odom_msg.pose.pose.orientation
            (_, _, self.start_odom_pose.z) = euler_from_quaternion([q.x, q.y, q.z, q.w]) # Yaw is stored in z
            
            rospy.loginfo(f"Start pose: x={self.start_odom_pose.x:.2f}, y={self.start_odom_pose.y:.2f}, theta={self.start_odom_pose.z:.2f}")

            # Record the starting pose
            self.list_of_odoms_recorded.append(self.start_odom_pose)
            self.last_odom_point = Point32(x=self.start_odom_pose.x, y=self.start_odom_pose.y, z=0)

        except rospy.ROSException as e:
            rospy.logerr(f"{self._action_name}: Failed to get initial odom - {e}. Aborting goal.")
            self._as.set_aborted(text="Failed to get initial odometry.")
            return 

        # Start the timer to call process_current_odom periodically (once per second)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.timer_driven_odom_processing)

        rate = rospy.Rate(10) # Check for preemption 10 times a second
        while not rospy.is_shutdown() and self._as.is_active():
            if self._as.is_preempt_requested():
                rospy.loginfo(f"{self._action_name}: Goal preempted by client.")
                self._as.set_preempted(text="Goal preempted by client.") 
                break 
            rate.sleep()
        
        # Final cleanup after the loop.
        self.cleanup_after_goal()

    def timer_driven_odom_processing(self, event):
        try:
            current_odom_msg = rospy.wait_for_message("/odom", Odometry, timeout=2.0)
            self.process_odom_message(current_odom_msg)
        except rospy.ROSException:
            rospy.logwarn_throttle(5.0, f"{self._action_name}: Could not get /odom message in timer.")

    def process_odom_message(self, odom_msg):
        """
        Core logic for processing an odometry message: recording, distance, feedback, lap check.
        """
        current_x = odom_msg.pose.pose.position.x
        current_y = odom_msg.pose.pose.position.y
        q = odom_msg.pose.pose.orientation
        (_, _, current_theta) = euler_from_quaternion([q.x, q.y, q.z, q.w])

        current_point32 = Point32(x=current_x, y=current_y, z=current_theta)
        self.list_of_odoms_recorded.append(current_point32)

        if self.last_odom_point:
            delta_x = current_x - self.last_odom_point.x
            delta_y = current_y - self.last_odom_point.y
            self.current_total_distance += math.sqrt(delta_x**2 + delta_y**2)
        
        self.last_odom_point = Point32(x=current_x, y=current_y, z=0) # z=0 as it's not used for distance

        # Publish feedback (total distance traveled)
        self._feedback.current_total = self.current_total_distance
        self._as.publish_feedback(self._feedback)

        # Check for lap completion
        if self.start_odom_pose and self.current_total_distance > self.min_dist_for_lap_check:
            dist_to_start = math.sqrt((current_x - self.start_odom_pose.x)**2 + \
                                      (current_y - self.start_odom_pose.y)**2)
            
            angle_diff = current_theta - self.start_odom_pose.z
            # Normalize angle_diff to be between -pi and pi
            while angle_diff > math.pi: angle_diff -= 2 * math.pi
            while angle_diff < -math.pi: angle_diff += 2 * math.pi
            
            if dist_to_start < self.pos_tolerance and abs(angle_diff) < self.orient_tolerance:
                rospy.loginfo("Lap completed!")
                rospy.loginfo(f" Lap End Position: x={current_point32.x:.2f}, y={current_point32.y:.2f}, theta={current_point32.z:.2f}")
                rospy.loginfo(f" Lap Total Distance Traveled: {self.current_total_distance:.2f} meters")
                self._result.list_of_odoms = self.list_of_odoms_recorded
                self._as.set_succeeded(self._result)
                self.cleanup_after_goal() 

if __name__ == '__main__':
    rospy.init_node('record_odom_server')
    server = OdomRecordServer()
    rospy.spin()
