#!/usr/bin/env python
import roslib; roslib.load_manifest('drone_driver')
import rospy

from ar_recog.msg import Tag, Tags
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
import std_msgs.msg
import std_srvs.srv
import sys, select, termios, math
import pid 

# Constants
IMAGE_WIDTH = 320#180 #320
IMAGE_HEIGHT = 240#135 #240
MAX_VELOCITY = 0.2 
MAX_HISTORY = 5
X_VEL_SCALAR = 0.002
Y_VEL_SCALAR = 0.002
Z_VEL_SCALAR = 0
X_DERIVATIVE_SCALAR = 0.004
Y_DERIVATIVE_SCALAR = 0.004
CONST_SCALAR = [X_VEL_SCALAR, Y_VEL_SCALAR, Z_VEL_SCALAR]
MIN_DISTANCE = 0
MAX_SCALAR = 0.1
DIRECTION_LETTERS = ['x','y','z']

# Global Variables
MyTwist = TwistStamped()
PrevDiameter = 0
PrevVector = []
p_x=pid.PID(X_VEL_SCALAR,0,X_DERIVATIVE_SCALAR)
p_y=pid.PID(Y_VEL_SCALAR,0,Y_DERIVATIVE_SCALAR)
FwdCam = True
    
def CalcScaledVelocity( LineVector ):
    global p_x
    global p_y
    NewVel = Twist().linear
#    VelocityVector = CalcVelocity( LineVector )
    print "X",
    Velocity = p_x.update(getattr(LineVector, DIRECTION_LETTERS[0]))
    if Velocity > 0:
        Velocity = min( Velocity, MAX_VELOCITY )
    else:
        Velocity = max( Velocity, -MAX_VELOCITY )
    setattr(NewVel, DIRECTION_LETTERS[0], Velocity) 

    print "Y",
    Velocity = p_y.update(getattr(LineVector, DIRECTION_LETTERS[1]))
    if Velocity > 0:
        Velocity = min( Velocity, MAX_VELOCITY )
    else:
        Velocity = max( Velocity, -MAX_VELOCITY )
    setattr(NewVel, DIRECTION_LETTERS[1], Velocity )    

    # Do not allow for vertical motion
    NewVel.z = 0
    
    #print NewVel
    return NewVel
   
def CalcScaledAngle( AngleVector ):
    # TODO Figure out how to compute angular motion
    NewTwist = Twist().angular
#    AngleVector = CalcAngle( AngleVector )
    # TODO Make PID for angle
    for i in range(len(DIRECTION_LETTERS)):
        # TODO See if any scaling needed
        Angle = AngleVector[i]
        setattr(NewTwist, DIRECTION_LETTERS[i], Angle)
    return NewTwist
    
def ProcessImagePosition (data):
    global MyTwist
    global FwdCam
    
    pub = rospy.Publisher('cmd_vel', Twist)
    land_pub = rospy.Publisher('/ardrone/land', std_msgs.msg.Empty)
    reset_pub = rospy.Publisher('/ardrone/reset', std_msgs.msg.Empty)
    takeoff_pub = rospy.Publisher('/ardrone/takeoff', std_msgs.msg.Empty)
    toggleCam = rospy.ServiceProxy('/ardrone/togglecam', std_srvs.srv.Empty)
    
#    rospy.loginfo(rospy.get_name()+"I heard %s",data.data)

    #print "*****"
    NewTwist = data
    #print "In: "
    #print NewTwist.twist.linear
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
    if key == 'x':
        try:
            foo = toggleCam()
            FwdCam = not FwdCam
            print foo
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    
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
    MyTwist.twist.linear = CalcScaledVelocity( NewTwist.twist.linear )
    #MyTwist.twist.angular = CalcScaledAngle( NewTwist.twist.angular )
    if ( math.isnan( MyTwist.twist.linear.x ) ):
        print 'NaN',
        print MyTwist.twist.linear.x
        MyTwist.twist.linear.x = 0

    # Only publish the twist parameters to the drone
    pub.publish(MyTwist.twist)
    #if MyTwist.twist.linear.x != 0 or MyTwist.twist.linear.y != 0:
        #print MyTwist
        #rospy.loginfo(MyTwist)

    
def ProcessXlateImage( data ):
    global PrevDiameter
    global FwdCam
    
    InputTags = data
    NewTwist = TwistStamped()
    
    if InputTags.tag_count:
        if FwdCam:
            # Forward Camera
            # +Z_fwd_cam = +Z_base - points up
            # +Y_fwd_cam = +Y_base - points left
            # +X_fwd_cam = +X_base - points forward
            NewTwist.twist.linear.x = InputTags.tags[0].diameter - 40
            NewTwist.twist.linear.z = 0 #( IMAGE_WIDTH/2 ) - InputTags.tags[0].y
            NewTwist.twist.linear.y = 0 #( IMAGE_HEIGHT/2 ) - InputTags.tags[0].x
            #NewTwist.twist.angular.z = 0 # No rotation on fwd cam
            #PrevDiameter = InputTags.tags[0].diameter
        else:
            # Downward Camera
            # NOTE: the downward camera 
            # +Z_down_cam = -Z_base - points down, 
            # +Y_down_cam = +X_base - points forward
            # +X_down_cam = +Y_base - points left
            NewTwist.twist.linear.x = InputTags.tags[0].y - ( IMAGE_HEIGHT/2 )
            NewTwist.twist.linear.y = InputTags.tags[0].x - ( IMAGE_WIDTH/2 )
            NewTwist.twist.linear.z = 0 # No depth for down cam
            #NewTwist.twist.angular.z = InputTags.tags[0].zRot
        
        #rospy.Publisher("image_pos", NewTwist )
        ProcessImagePosition( NewTwist )

        # Keep some history for when the tag disappears
        while len(PrevVector) >= MAX_HISTORY:
            PrevVector.pop(0)
        PrevVector.append(NewTwist.twist)
        
#        print "Found Tag %d" % len(PrevVector)

    else:
        # Extrapolate history
#        try:
#            NewTwist.twist = PrevVector.pop(0)
#            print "Use History %d" % len(PrevVector)
#            # Save off some history
#        except IndexError, e:
#            # Ran out of history
#            print "No History"
        NewTwist.twist.linear.x = 0
        NewTwist.twist.linear.y = 0
        NewTwist.twist.linear.z = 0
        NewTwist.twist.angular.z = 0
        
        pub = rospy.Publisher("cmd_vel", Twist )
        pub.publish( NewTwist.twist )
        #ProcessImagePosition( NewTwist )
    
def DroneDriver():
    p_x.setPoint(0)
    p_y.setPoint(0)
    
    rospy.init_node('drone_driver')
    rospy.Subscriber("tags", Tags, ProcessXlateImage )   
    rospy.Subscriber("image_pos", TwistStamped, ProcessImagePosition )
    rospy.spin()

if __name__ == '__main__':
    DroneDriver()

