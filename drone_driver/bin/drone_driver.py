#!/usr/bin/env python
import roslib; roslib.load_manifest('drone_driver')
import rospy

from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

import sys, select, termios, tty

prevTwist = TwistStamped()
twist = TwistStamped()

def process_image_position (data):
    global prevTwist
    global twist
#    twist.twist.linear.x = 0
#    twist.twist.linear.y = 0
#    twist.twist.linear.z = 0
#    twist.twist.angular.x = 0
#    twist.twist.angular.y = 0
#    twist.twist.angular.z = 0
#    prevTwist.twist.linear.x = 0
#    prevTwist.twist.linear.y = 0
#    prevTwist.twist.linear.z = 0
#    prevTwist.twist.angular.x = 0
#    prevTwist.twist.angular.y = 0
#    prevTwist.twist.angular.z = 0


    pub = rospy.Publisher('cmd_vel', Twist)
    land_pub = rospy.Publisher('/ardrone/land', Empty)
    reset_pub = rospy.Publisher('/ardrone/reset', Empty)
    takeoff_pub = rospy.Publisher('/ardrone/takeoff', Empty)
#    rospy.loginfo(rospy.get_name()+"I heard %s",data.data)

    print "*****"
    newTwist = TwistStamped()
    newTwist = data
    print newTwist.twist.linear
    key = newTwist.header.frame_id
    # takeoff and landing
    if key == 'land':
        rospy.loginfo(rospy.get_name()+"I heard %s",prevTwist.header.frame_id)
        land_pub.publish(Empty())
    if key == 'abort':
        rospy.loginfo(rospy.get_name()+"I heard %s",prevTwist.header.frame_id)
        reset_pub.publish(Empty())
    if key == 'takeoff':
        rospy.loginfo(rospy.get_name()+"I heard %s",prevTwist.header.frame_id)
        takeoff_pub.publish(Empty())
    if key == 'hover':
        # TODO Implement hover
        rospy.loginfo(rospy.get_name()+"I heard %s",prevTwist.header.frame_id)
        takeoff_pub.publish(Empty())
    # compute the difference from the last frame
    twist.twist.linear.x = prevTwist.twist.linear.x - newTwist.twist.linear.x
    twist.twist.linear.y = prevTwist.twist.linear.y - newTwist.twist.linear.y
    twist.twist.linear.z = prevTwist.twist.linear.z - newTwist.twist.linear.z
    twist.twist.angular.x = prevTwist.twist.angular.x - newTwist.twist.angular.x
    twist.twist.angular.y = prevTwist.twist.angular.y - newTwist.twist.angular.y
    twist.twist.angular.z = prevTwist.twist.angular.z - newTwist.twist.angular.z
    print twist.twist.linear
    prevTwist = newTwist
    # Only publish the twist parameters
    pub.publish(twist.twist)

def drone_driver():
    rospy.init_node('drone_driver')
    rospy.Subscriber("image_pos", TwistStamped, process_image_position )
    rospy.spin()

if __name__ == '__main__':
    drone_driver()
T
