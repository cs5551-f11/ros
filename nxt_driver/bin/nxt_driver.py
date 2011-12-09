#!/usr/bin/env python
import roslib; roslib.load_manifest('nxt_driver')
import rospy

from ar_recog.msg import Tag, Tags
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist

import std_msgs.msg
import std_srvs.srv
import math, logging, datetime
import subprocess

logging.basicConfig()
log = logging.getLogger("NxtDriver")
hdlr = logging.FileHandler('nxt_driver%s.log' % datetime.datetime.now().strftime("%Y-%m-%d %H:%M"))
log.setLevel(logging.DEBUG) #set verbosity to show all messages of severity >= DEBUG
formatter = logging.Formatter('%(asctime)s %(message)s')
hdlr.setFormatter(formatter)
log.addHandler(hdlr)
log.info("Starting NxtDriver")

class PID:
    
    def __init__(self, Key='none', Kp=0.1, Ki=0.0, Kd=0.05, PIDMax=1, ErrInit=0, ErrMin=10, DerInit=0, DerMin=1, IntInit=0, IntMax=25, IntMin=-25, Goal=0):
        self.Key = Key
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Der = DerInit
        self.DerInit = DerInit
        self.DerMin = DerMin 
        self.Int = IntInit
        self.IntInit = IntInit
        self.IntMax = IntMax
        self.IntMin = IntMin
        self.ErrMin = ErrMin
        self.Goal = Goal
        self.Err = ErrInit
        self.ErrInit = ErrInit
        self.PIDMax = PIDMax
        self.IsInit = True
        self.Time = datetime.datetime.now()
        
    def ReInit(self, Key='none', Kp=0.1, Ki=0.0, Kd=0.05, PIDMax=1, ErrInit=0, ErrMin=10, DerInit=0, DerMin=1, IntInit=0, IntMax=25, IntMin=-25, Goal=0):
        log.info("%s ReInit" % (self.Key))
        self.Key = Key
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Der = DerInit
        self.DerInit = DerInit
        self.DerMin = DerMin 
        self.Int = IntInit
        self.IntInit = IntInit
        self.IntMax = IntMax
        self.IntMin = IntMin
        self.ErrMin = ErrMin
        self.Goal = Goal
        self.Err = ErrInit
        self.ErrInit = ErrInit
        self.PIDMax = PIDMax
        self.IsInit = True
        self.Time = datetime.datetime.now()
                
    def Reset(self):
        log.info("%s Reset" % (self.Key))
        self.Der = self.DerInit 
        self.Int = self.IntInit
        self.Err = self.ErrInit
        self.IsInit = True
        self.Time = datetime.datetime.now()

    def IncrementKp(self, Kp):
        self.Kp += Kp
        log.info("%s, Kp, %f, Ki, %f, Kd, %f" % (self.Key, self.Kp, self.Ki, self.Kd))
        self.Reset()
    
    def IncrementKi(self, Ki):
        self.Ki += Ki
        log.info("%s, Kp, %f, Ki, %f, Kd, %f" % (self.Key, self.Kp, self.Ki, self.Kd))
        self.Reset()
    
    def IncrementKd(self, Kd):
        self.Kd += Kd
        log.info("%s, Kp, %f, Ki, %f, Kd, %f" % (self.Key, self.Kp, self.Ki, self.Kd))
        self.Reset()
        
    def ComputePID(self,CurVal):
        global log
        self.Err = self.Goal - CurVal
        # check for "Dead Band" region where no controls are sent 
        if math.fabs(self.Err) < self.ErrMin and self.Der < self.DerMin:
            # Within the "Dead Band" region, so stop moving so not to induce error
            self.P = 0
            self.I = 0
            self.D = 0
        else:
            ###################################################################
            # Proportional
            ###################################################################
            self.P = self.Kp * self.Err
            from std_msgs.msg import Empty

            ###################################################################
            # Derivative
            ###################################################################
            # If IsInit that means this is first frame, so don't do anything with derivative
            TimeDelta = datetime.datetime.now() - self.Time
            if not self.IsInit:
                self.D = self.Kd * float( self.Err - self.Der ) / TimeDelta.microseconds
            else:
                self.D = 0
            self.Time = datetime.datetime.now()
            self.Der = self.Err
            
            ###################################################################
            # Integral
            ###################################################################
            self.Int = self.Int + self.Err
    
            if self.Int > self.IntMax:
                self.Int = self.IntMax
            elif self.Int < self.IntMin:
                self.Int = self.IntMin
    
            self.I = self.Int * self.Ki
        ###################################################################
        # PID Total
        ###################################################################
        PID = self.P + self.I + self.D
        if PID >= 0:
            PID = min(PID, self.PIDMax)
        else:
            PID = max(PID, -self.PIDMax)
        self.IsInit = False
        log.info("%s, Err, %f, P, %f, I, %f, D, %f, PID %f" % (self.Key, self.Err, self.P, self.I, self.D, PID))
        return PID

###################################################################
# Constants
###################################################################
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240
X_MAX_VELOCITY = 0.1
Z_ANG_MAX_VELOCITY = 0.001
X_VEL_SCALAR = 0.003
Z_ANG_VEL_SCALAR = 0.1
X_DERIVATIVE_SCALAR = 0 #7500
Z_ANG_DERIVATIVE_SCALAR = 0 #1500
X_INT_SCALAR = 0
Z_ANG_INT_SCALAR = 0
X_DEAD_BAND = 1
Z_ANG_DEAD_BAND = 8
TAG_DIAMETER = 58

MAX_HISTORY=5

DIRECTION_LETTERS = ['x','y','z']

# Global Variables1
MyTwist = TwistStamped()
LastTwist = Twist()
PrevDiameter = 0
PrevVector = []
p_x=PID('vel_x', X_VEL_SCALAR, X_INT_SCALAR, X_DERIVATIVE_SCALAR, X_MAX_VELOCITY, 0, X_DEAD_BAND)
p_z_ang=PID('vel_y', Z_ANG_VEL_SCALAR, Z_ANG_INT_SCALAR, Z_ANG_DERIVATIVE_SCALAR, Z_ANG_MAX_VELOCITY, 0, Z_ANG_DEAD_BAND)

TagFound = False
processHandle = subprocess.Popen("rosrun ar_recog ar_recog image:=/ardrone/image_raw camera_info:=/ardrone/camera_info", cwd="/home/base/ros/brown-ros-pkg/experimental/ar_recog/bin", shell=True) 
    
def CalcScaledVelocity( LineVector ):
    global p_x
    NewVel = Twist().linear
    Velocity = p_x.ComputePID(getattr(LineVector, DIRECTION_LETTERS[0]))
    setattr(NewVel, DIRECTION_LETTERS[0], Velocity) 
    return NewVel
   
def CalcScaledAngle( AngleVector ):
    global p_z_ang
    NewTwist = Twist().angular
    Angle = p_z_ang.ComputePID(getattr(AngleVector, DIRECTION_LETTERS[2]))
    print Angle
    setattr(NewTwist, DIRECTION_LETTERS[2], Angle )
    return NewTwist
   
def ProcessImagePosition (data):
    global MyTwist
    global p_x
    global p_z_ang
    global LastTwist
    global processHandle
    
    pub = rospy.Publisher('cmd_vel', Twist)
    
    NewTwist = data
    
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
    MyTwist.twist.angular = CalcScaledAngle( NewTwist.twist.angular )
    if ( math.isnan( MyTwist.twist.linear.x ) ):
        print 'NaN X',
        MyTwist.twist.linear.x = 0
    if ( math.isnan( MyTwist.twist.linear.y ) ):
        print 'NaN Y',
        MyTwist.twist.linear.y = 0
    if ( math.isnan( MyTwist.twist.linear.z ) ):
        print 'NaN Z',
        MyTwist.twist.linear.z = 0
    if ( math.isnan( MyTwist.twist.angular.z ) ):
        print 'NaN Ang Z'
        MyTwist.twist.angular.z = 0
               
    # Only publish the twist parameters to the drone
    LastTwist = MyTwist.twist
    pub.publish(MyTwist.twist)
#    if MyTwist.twist.linear.x != 0 or MyTwist.twist.linear.y != 0:
#        print MyTwist
#        rospy.loginfo(MyTwist)

    
def ProcessXlateImage( data ):
    global PrevDiameter
    global p_x
    global p_z_ang
    global TAG_DIAMETER
    global TagFound
    global LastTwist
        
    InputTags = data
    NewTwist = TwistStamped()
    
    if InputTags.tag_count > 0:
        TagFound = True
        # +Z_fwd_cam = +Z_base - points up
        # +Y_fwd_cam = +Y_base - points left
        # +X_fwd_cam = +X_base - points forward
        NewTwist.twist.linear.x = InputTags.tags[0].diameter - TAG_DIAMETER
        #NewTwist.twist.linear.y = InputTags.tags[0].x - ( FWD_IMAGE_WIDTH/2 ) #( IMAGE_WIDTH/2 ) - InputTags.tags[0].y
        #NewTwist.twist.linear.z = InputTags.tags[0].y - ( FWD_IMAGE_HEIGHT/2 ) #( IMAGE_HEIGHT/2 ) - InputTags.tags[0].x
        NewTwist.twist.angular.z = InputTags.tags[0].x - ( IMAGE_WIDTH/2 ) #( IMAGE_WIDTH/2 ) - InputTags.tags[0].y #
        #PrevDiameter = InputTags.tags[0].diameter
            
        #rospy.Publisher("image_pos", NewTwist )
        ProcessImagePosition( NewTwist )

        # Keep some history for when the tag disappears
        while len(PrevVector) >= MAX_HISTORY:
            PrevVector.pop(0)
        PrevVector.append(NewTwist.twist)
    else:
        # Extrapolate history
        try:
            NewTwist.twist = PrevVector.pop(0)
            print "Use History %d" % len(PrevVector)
            NewTwist.twist = LastTwist
            # Save off some history
        except IndexError, e:
            
            # Ran out of history
            NewTwist.twist.linear.x = 0
            NewTwist.twist.linear.y = 0
            NewTwist.twist.linear.z = 0
            NewTwist.twist.angular.z = 0
        if TagFound:
            p_x.Reset()
            p_z_ang.Reset()
        TagFound = False    
        pub = rospy.Publisher("cmd_vel", Twist )
        pub.publish( NewTwist.twist )
        #ProcessImagePosition( NewTwist )
    
def NxtDriver():
    global processHandle
    
    rospy.init_node('nxt_driver')
    rospy.Subscriber("tags", Tags, ProcessXlateImage )   
    rospy.spin()

if __name__ == '__main__':
    try:
        NxtDriver()
    except Exception as e:
        print e
        print repr(e)
        
    finally:
        processHandle.kill()
        # TODO emergency stop on exit
        print "Done"
