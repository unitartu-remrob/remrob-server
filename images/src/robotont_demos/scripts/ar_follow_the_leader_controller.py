#!/usr/bin/python3

import rospy
import tf
import numpy
from geometry_msgs.msg import Twist
from ar_track_alvar_msgs.msg import AlvarMarkers
from math import atan2, sqrt

MAX_LIN_VEL = 0.05
MAX_ANG_VEL = 0.6

GOAL_TOLERANCE = 0.04
ANGLE_TOLERANCE = 0.1
GOAL_DIST_FROM_MARKER = 0.5

last_heartbeat = 0

def callback(data):
        global last_heartbeat, marker_id
        for marker in data.markers:
                # Check the marker id
                if marker.id != marker_id:
                        continue

                rospy.loginfo(rospy.get_caller_id() + " I heard %s", marker)
                marker_pos = (
                        marker.pose.pose.position.x,
                        marker.pose.pose.position.y,
                        marker.pose.pose.position.z,
                        0)
                marker_ori = (
                        marker.pose.pose.orientation.x,
                        marker.pose.pose.orientation.y,
                        marker.pose.pose.orientation.z,
                        marker.pose.pose.orientation.w)

                # Start with a unit vector pointing forward
                goal_dir = numpy.array([0, 0, 1, 0])

                # Rotation matrix that represents to marker orientation
                rot_mat = tf.transformations.quaternion_matrix(marker_ori)

                # Rotate the unit vec so it points to the direction of the marker
                goal_dir = rot_mat.dot(goal_dir)

                goal_pos = marker_pos + goal_dir * GOAL_DIST_FROM_MARKER

                # Find planar distance to the goal
                dist_to_goal = numpy.linalg.norm(goal_pos[0:2])
                dist_to_marker = numpy.linalg.norm(marker_pos[0:2])

                twist_msg = Twist()
                angle = atan2(marker_pos[1],marker_pos[0])
                rospy.logdebug("ANGLE: %s", angle)
                rospy.logdebug("DISTTOMARKER: %s", dist_to_marker)
                rospy.logdebug("GoalDir: %s", goal_dir)
                rospy.logdebug("DISTTOGOAL: %s", dist_to_goal)
                rospy.logdebug("\nPOS: %s", marker_pos)
                rospy.logdebug("\nGOAL: %s", goal_pos)

                twist_msg.linear.x = min(max(goal_pos[0],-MAX_LIN_VEL),MAX_LIN_VEL)
                twist_msg.linear.y = min(max(goal_pos[1],-MAX_LIN_VEL),MAX_LIN_VEL)
                twist_msg.angular.z = 4*min(max(angle,-MAX_ANG_VEL),MAX_ANG_VEL)

                if abs(angle) > ANGLE_TOLERANCE or dist_to_goal > GOAL_TOLERANCE :
                        print("Updating cmd vel %s", twist_msg)
                        global cmd_vel_pub
                        last_heartbeat = rospy.get_time()
                        cmd_vel_pub.publish(twist_msg)

# Publish zero cmd_vel when no AR info has been received within given period
def timer_callback(event):
        global last_heartbeat
        if (rospy.get_time() - last_heartbeat) >= 0.5:
                cmd_vel_pub.publish(Twist())


def ar_demo():
        global marker_id
        # Initialize this ROS node
        rospy.init_node('ar_demo', anonymous=True)
        # get target marker id
        marker_id = rospy.get_param('~marker_id')

        #initialize heartbeat
        global last_heartbeat
        last_heartbeat = rospy.get_time()

        # Create publisher for command velocity
        global cmd_vel_pub
        cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        #Register heartbeat timer
        t = rospy.Timer(rospy.Duration(0.1), timer_callback)

        # Set up subscriber for /ar_pose_marker
        rospy.loginfo("Subscribing to ar_pose_marker")
        rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback)

        rospy.spin()


if __name__ == '__main__':
        ar_demo()
