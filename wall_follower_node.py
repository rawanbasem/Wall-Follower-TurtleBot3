#!/usr/bin/env python3
import rospy
import actionlib
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from wall_follower.srv import FindWall
from wall_follower.msg import OdomRecordAction, OdomRecordGoal  

# the service client
def find_wall_service():
    rospy.wait_for_service('/find_wall')
    try:
        rospy.ServiceProxy('/find_wall', FindWall)()
        rospy.loginfo(" Aligned to wall.")
    except rospy.ServiceException as e:
        rospy.logwarn(f"/find_wall service failed: {e}")

# the action client
def start_odom_recording():
    client = actionlib.SimpleActionClient('record_odom', OdomRecordAction)
    client.wait_for_server()
    client.send_goal(OdomRecordGoal()) # send an empty goal
    rospy.loginfo(" Odometry recording started.")

def scan_cb(msg):
    N = len(msg.ranges)
    right_dist = msg.ranges[N//4]
    front_dist = msg.ranges[N//2]

    cmd = Twist()
    if front_dist < 0.5:
        cmd.linear.x = 0.05
        cmd.angular.z = 0.5

    elif right_dist > 0.3:
#        rospy.loginfo(" drifting from wall → steer right")
        cmd.linear.x = 0.1
        cmd.angular.z = -0.05

    elif right_dist < 0.2:
#        rospy.loginfo(" too close → steer left")
        cmd.linear.x = 0.1
        cmd.angular.z = 0.05

    else:
        cmd.linear.x = 0.2
        cmd.angular.z = 0.0

    pub.publish(cmd)

if __name__ == '__main__':
    rospy.init_node('wall_follower_node')

    find_wall_service()
    start_odom_recording()

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/scan', LaserScan, scan_cb)

    rospy.loginfo("Wall follower running…")
    rospy.spin()