#!/usr/bin/env python
import roslib; roslib.load_manifest('drone_driver')
import rospy

from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

import sys, select, termios, math

# Constants
X_VEL_SCALAR = 1
Y_VEL_SCALAR = 1
Z_VEL_SCALAR = 1
CONST_SCALAR = [X_VEL_SCALAR, Y_VEL_SCALAR, Z_VEL_SCALAR]
MIN_DISTANCE = 0
MAX_SCALAR = 1

DirectionLetters = ['x','y','z']

# Global Variables
MyTwist = TwistStamped()

def CalcScalarXYZ( LineVector ):
    ReturnScalar = [1,1,1]
    for i in range(len(DirectionLetters)):
        Distance = getattr(LineVector, DirectionLetters[i])
        if math.fabs(Distance) <= MIN_DISTANCE:    
            ReturnScalar[i] = 0 
        else:
            # TODO Apply function to Distance
            # Cap the Scalar to keep the speed under control
            ReturnScalar[i] = min(Distance, MAX_SCALAR)
    return ReturnScalar
    
def CalcVelocityFromImagePos( LineVector ):
    NewVel = Twist().linear
    Scalar = CalcScalarXYZ( LineVector )
    for i in range(len(DirectionLetters)):
        Velocity = CONST_SCALAR[i] * Scalar[i]
        setattr(NewVel, DirectionLetters[i], Velocity)
    # Do not allow for vertical motion
    NewVel.z = 0
    return NewVel

def CalcAngleFromImagePos( AngleVector ):
    # TODO Figure out how to compute angular motion
    NewTwist = Twist().angular
    for i in range(len(DirectionLetters)):
        setattr(NewTwist, DirectionLetters[i], 0)
    return NewTwist
    
def ProceImagePosition (data):
    global MyTwist

    pub = rospy.Publisher('cmd_vel', Twist)
    land_pub = rospy.Publisher('/ardrone/land', Empty)
    reset_pub = rospy.Publisher('/ardrone/reset', Empty)
    takeoff_pub = rospy.Publisher('/ardrone/takeoff', Empty)
    toggleCam = rospy.AddTwoInts('/ardrone/togglecam', toggleCam_service)

#    rospy.loginfo(rospy.get_name()+"I heard %s",data.data)

    print "*****"
    #NewTwist = TwistStamped()
    NewTwist = data
    print "In: "
    print NewTwist.twist.linear
    key = NewTwist.header.frame_id
    # takeoff and landing
    if key == 'land':
        rospy.loginfo(rospy.get_name()+"I heard %s",NewTwist.header.frame_id)
        land_pub.publish(Empty())
    if key == 'abort':
        rospy.loginfo(rospy.get_name()+"I heard %s",NewTwist.header.frame_id)
        reset_pub.publish(Empty())
    if key == 'takeoff':
        rospy.loginfo(rospy.get_name()+"I heard %s",NewTwist.header.frame_id)
        takeoff_pub.publish(Empty())
    if key == 'hover':
        # TODO Implement hover
        rospy.loginfo(rospy.get_name()+"I heard %s",NewTwist.header.frame_id)
        takeoff_pub.publish(Empty())
    
    # This input image twist give the delta position and pose for the tag relative 
    # to the center of the of the image frame.
    # Therefore the direction of movement is known from the direct input values and
    # only the magnitude (speed) must be calculated.  
    # The speed will be calculated based on the previous position and speed vectors
    # from the previous frames.
    # If the tag continues to be far from the center of the frame then the speed
    # will be increased.
    # If the tag is switching direction in the frame (oscillating) then reduce the
    # speed to hover over target
    # The exact function is unknown, so let's start with linear
    MyTwist.twist.linear = CalcVelocityFromImagePos( NewTwist.twist.linear )
    MyTwist.twist.angular = CalcAngleFromImagePos( NewTwist.twist.angular )
    
    print "Out: "
    print MyTwist.twist.linear
    # Only publish the twist parameters to the drone
    pub.publish(MyTwist.twist)
    
    #MyTwist.twist.linear.x = 0
    #MyTwist.twist.linear.y = 0
    #MyTwist.twist.linear.z = 0
    #pub.publish(MyTwist.twist)

def DroneDriver():
    rospy.init_node('drone_driver')
    rospy.Subscriber("image_pos", TwistStamped, ProceImagePosition )
    rospy.spin()

if __name__ == '__main__':
    DroneDriver()

