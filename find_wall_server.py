#!/usr/bin/env python3
import rospy
import math 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from wall_follower.srv import FindWall, FindWallResponse

latest_scan = None
cmd_pub = None 

def scan_callback(scan_msg):
    global latest_scan
    latest_scan = scan_msg.ranges

def find_wall_index(ranges, wall=40, tolerance=0.05): 
    best_mean = None
    best_start = 0

    # If 'ranges' is empty, len(ranges) - wall + 1 can lead to an empty range for the loop,
    # causing 'best_mean' to remain None, and thus the function returns None. This is acceptable.
    if not ranges: # basic check to handle empty ranges 
        return None 

    for start in range(len(ranges) - wall + 1):
        segment = ranges[start:start + wall]
        try:
            mean_dist = sum(segment) / wall
            # Check if all points are close to the mean
            if max(abs(r - mean_dist) for r in segment) < tolerance:
                if best_mean is None or mean_dist < best_mean:
                    best_mean = mean_dist
                    best_start = start
        except (TypeError, ZeroDivisionError): # issues if segment contains non-numeric or wall is 0
            # If an error occurs processing a segment, just skip it.
            continue 

    if best_mean is None:
        return None
    return best_start + wall // 2

def handle_find_wall(req):
    global latest_scan
    rospy.loginfo("FindWall service called.") 
    rate = rospy.Rate(10) 

    # Wait for latest_scan to be filled
    while latest_scan is None and not rospy.is_shutdown():
        rospy.logwarn_throttle(2.0, "In FindWall: Waiting for laser scan data...")
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("ROS interrupt received. Exiting find_wall.")
            return FindWallResponse(wallfound=False) 
#        return FindWallResponse(wallfound=False)
        
    if not latest_scan: # Check if the ranges list is empty
        rospy.logerr("Laser scan data (ranges) is an empty list. Aborting find_wall.")
        return FindWallResponse(wallfound=False)

    cmd = Twist()
    front_index = len(latest_scan) // 2
    index_tolerance = 5 

    rospy.loginfo("Rotating to face nearest wall.") 
    while not rospy.is_shutdown():
        idx = find_wall_index(latest_scan)
        if idx is None:
            # If no wall segment is found, idx will be None.
            rospy.logwarn_throttle(2.0, "No wall segment found by find_wall_index. Continuing rotation.")
        elif abs(idx - front_index) <= index_tolerance: 
            rospy.loginfo("Facing wall cluster at index %d", idx)
            break
        cmd.linear.x = 0.0
        cmd.angular.z = 0.3 
        cmd_pub.publish(cmd)
        rate.sleep()
    cmd.angular.z = 0.0 # Stop rotation
    cmd_pub.publish(cmd)

    rospy.loginfo("Moving forward to wall…")
    while not rospy.is_shutdown(): 
        if front_index >= len(latest_scan): 
            rospy.logerr("front_index is out of bounds for current scan data. Stopping.")
            cmd.linear.x = 0.0
            cmd_pub.publish(cmd)
            return FindWallResponse(wallfound=False)
        if latest_scan[front_index] > 0.26 : 
            cmd.linear.x = 0.1 
            cmd_pub.publish(cmd)
            rate.sleep()
        else:
            break
    cmd.linear.x = 0.0 # Stop forward movement
    cmd_pub.publish(cmd)

    rospy.loginfo("Aligning with the wall on the right…") 
    rotate_duration = rospy.Duration(4.2) 
    end_time = rospy.Time.now() + rotate_duration
    while rospy.Time.now() < end_time and not rospy.is_shutdown(): 
        cmd.angular.z = 0.35 
        cmd_pub.publish(cmd)
        rate.sleep()

    cmd = Twist() 
    cmd_pub.publish(cmd)

    rospy.loginfo(" Ready to follow wall.")
    return FindWallResponse(wallfound=True)

if __name__ == "__main__":
    rospy.init_node("find_wall_server")
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rospy.Subscriber("/scan", LaserScan, scan_callback, queue_size=1) 
    rospy.Service("/find_wall", FindWall, handle_find_wall) 
    rospy.loginfo("Find Wall Server Ready.")
    rospy.spin()