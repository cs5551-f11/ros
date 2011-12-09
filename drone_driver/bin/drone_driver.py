#!/usr/bin/env python
import roslib; roslib.load_manifest('drone_driver')
import rospy

from ar_recog.msg import Tag, Tags
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from ardrone_brown.msg import Navdata
from std_msgs.msg import Empty

import std_msgs.msg
import std_srvs.srv
import math, logging, datetime
import subprocess

logging.basicConfig()
log = logging.getLogger("DroneDriver")
hdlr = logging.FileHandler('driver%s.log' % datetime.datetime.now().strftime("%Y-%m-%d %H:%M"))
log.setLevel(logging.DEBUG) #set verbosity to show all messages of severity >= DEBUG
formatter = logging.Formatter('%(asctime)s %(message)s')
hdlr.setFormatter(formatter)
log.addHandler(hdlr)
log.info("Starting DroneDriver")

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
# Forward Cam Constants
###################################################################
FWD_IMAGE_WIDTH = 320
FWD_IMAGE_HEIGHT = 240
FWD_X_MAX_VELOCITY = 0.1 
FWD_Y_MAX_VELOCITY = 0.1
FWD_Z_MAX_VELOCITY = 0.5
FWD_Z_ANG_MAX_VELOCITY = 0.0
FWD_X_VEL_SCALAR = 0.02
FWD_Y_VEL_SCALAR = 0.001
FWD_Z_VEL_SCALAR = 0.002
FWD_Z_ANG_VEL_SCALAR = 0.0
FWD_X_DERIVATIVE_SCALAR = 7500
FWD_Y_DERIVATIVE_SCALAR = 1500
FWD_Z_DERIVATIVE_SCALAR = FWD_Y_DERIVATIVE_SCALAR*(float(FWD_IMAGE_WIDTH)/FWD_IMAGE_HEIGHT)
FWD_Z_ANG_DERIVATIVE_SCALAR = 0
FWD_X_INT_SCALAR = 0
FWD_Y_INT_SCALAR = 0
FWD_Z_INT_SCALAR = 0
FWD_Z_ANG_INT_SCALAR = 0
FWD_X_DEAD_BAND=1 ### FWD CAM ONLY....DWN CAM 8
FWD_Y_DEAD_BAND=8
FWD_Z_DEAD_BAND=FWD_Y_DEAD_BAND*float(float(FWD_IMAGE_WIDTH)/FWD_IMAGE_HEIGHT)
FWD_Z_ANG_DEAD_BAND=0
FWD_TAG_DIAMETER=20
###################################################################
# Downward Cam Constants 
###################################################################
DWN_IMAGE_WIDTH = 176
DWN_IMAGE_HEIGHT = 144
DWN_X_MAX_VELOCITY = 0.1 
DWN_Y_MAX_VELOCITY = 0.1
DWN_Z_MAX_VELOCITY = 0.2
DWN_Z_ANG_MAX_VELOCITY = 0.3
DWN_X_VEL_SCALAR = 0.0020
DWN_Y_VEL_SCALAR = 0.0015
DWN_Z_VEL_SCALAR = 0.01
DWN_Z_ANG_VEL_SCALAR = 0.3
DWN_X_DERIVATIVE_SCALAR = 550
DWN_Y_DERIVATIVE_SCALAR = 450
DWN_Z_DERIVATIVE_SCALAR = 7500
DWN_Z_ANG_DERIVATIVE_SCALAR = 100
DWN_X_INT_SCALAR = 0
DWN_Y_INT_SCALAR = 0
DWN_Z_INT_SCALAR = 0
DWN_Z_ANG_INT_SCALAR = 0
DWN_X_DEAD_BAND=8 
DWN_Y_DEAD_BAND=8
DWN_Z_DEAD_BAND=0.1
DWN_Z_ANG_DEAD_BAND=0
DWN_TAG_DIAMETER=13
###################################################################
# No Tag Constants 
###########################################from std_msgs.msg import Empty
########################
ALT_Z_MAX_VELOCITY = 0.5
ALT_Z_VEL_SCALAR = 0.0005
ALT_Z_DERIVATIVE_SCALAR = 100
ALT_Z_INT_SCALAR = 0
ALT_Z_DEAD_BAND=1
ALT_Z_GOAL=1150

ANG_VEL_TO_RADIANS_SCALAR = -0.01
MAX_HISTORY=5
DIRECTION_LETTERS = ['x','y','z']

# Global Variables1
MyTwist = TwistStamped()
NavTwist = Twist()
LastTwist = Twist()
PrevDiameter = 0
PrevVector = []
p_x=PID('vel_x', FWD_X_VEL_SCALAR, FWD_X_INT_SCALAR, FWD_X_DERIVATIVE_SCALAR, FWD_X_MAX_VELOCITY, 0, FWD_X_DEAD_BAND)
p_y=PID('vel_y', FWD_Y_VEL_SCALAR, FWD_Y_INT_SCALAR, FWD_Y_DERIVATIVE_SCALAR, FWD_Y_MAX_VELOCITY, 0, FWD_Y_DEAD_BAND)
p_z=PID('vel_z', FWD_Z_VEL_SCALAR, FWD_Z_INT_SCALAR, FWD_Z_DERIVATIVE_SCALAR, FWD_Z_MAX_VELOCITY, 0, FWD_Z_DEAD_BAND)
p_z_ang=PID('ang_z', FWD_Z_ANG_VEL_SCALAR, FWD_Z_ANG_INT_SCALAR, FWD_Z_ANG_DERIVATIVE_SCALAR, FWD_Z_ANG_MAX_VELOCITY, 0, FWD_Z_ANG_DEAD_BAND)
p_z_alt=PID('alt_z', ALT_Z_VEL_SCALAR, ALT_Z_INT_SCALAR, ALT_Z_DERIVATIVE_SCALAR, ALT_Z_MAX_VELOCITY, 0, ALT_Z_DEAD_BAND)

FwdCam = True
Flying = False
PrevCam = False
CurCam = False
prev_key = 'foobar'
TagFound = False
PoseMatch = False
AutoAltitude = False
processHandle = subprocess.Popen("rosrun ar_recog ar_recog image:=/ardrone/image_raw camera_info:=/ardrone/camera_info", cwd="/home/base/ros/brown-ros-pkg/experimental/ar_recog/bin", shell=True) 
    
def CalcScaledVelocity( LineVector ):
    global p_x
    global p_y
    global p_z
    NewVel = Twist().linear
    Velocity = p_x.ComputePID(getattr(LineVector, DIRECTION_LETTERS[0]))
    setattr(NewVel, DIRECTION_LETTERS[0], Velocity) 
    Velocity = p_y.ComputePID(getattr(LineVector, DIRECTION_LETTERS[1]))
    setattr(NewVel, DIRECTION_LETTERS[1], Velocity )
    Velocity = p_z.ComputePID(getattr(LineVector, DIRECTION_LETTERS[2]))
    setattr(NewVel, DIRECTION_LETTERS[2], Velocity )
    return NewVel
   
def CalcScaledAngle( AngleVector ):
    global p_z_ang
    NewTwist = Twist().angular
    Angle = p_z_ang.ComputePID(getattr(AngleVector, DIRECTION_LETTERS[2]))
    print Angle
    setattr(NewTwist, DIRECTION_LETTERS[2], Angle )
    print AngleVector
    print NewTwist
    print
    return NewTwist

def ProcessChangePID( data ):
    global p_x
    global p_y
    global p_z
    global p_z_ang
    global FwdCam
    
    if data.twist.linear.x != 0:
        if data.header.frame_id == "p":
            p_x.IncrementKp( data.twist.linear.x )
        elif data.header.frame_id == "i":
            p_x.IncrementKi( data.twist.linear.x )
        elif data.header.frame_id == "d":
            p_x.IncrementKd( data.twist.linear.x )
    if data.twist.linear.y != 0:
        if data.header.frame_id == "p":
            p_y.IncrementKp( data.twist.linear.y )
        elif data.header.frame_id == "i":
            p_y.IncrementKi( data.twist.linear.y )
        elif data.header.frame_id == "d":
            p_y.IncrementKd( data.twist.linear.y )
    
def ProcessImagePosition (data):
    global MyTwist
    global FwdCam
    global prev_key
    global p_x
    global p_y
    global p_z
    global p_z_ang
    global ANG_VEL_TO_RADIANS_SCALAR
    global PoseMatch
    global LastTwist
    global processHandle
    
    pub = rospy.Publisher('cmd_vel', Twist)
    land_pub = rospy.Publisher('/ardrone/land', std_msgs.msg.Empty)
    reset_pub = rospy.Publisher('/ardrone/reset', std_msgs.msg.Empty)
    takeoff_pub = rospy.Publisher('/ardrone/takeoff', std_msgs.msg.Empty)
    toggleCam = rospy.ServiceProxy('/ardrone/togglecam', std_srvs.srv.Empty)
    
    NewTwist = data
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
    if key == 'switch' and prev_key != key:
        try:
            toggleCam()
            FwdCam = not FwdCam
            processHandle.kill()
            processHandle = subprocess.Popen("rosrun ar_recog ar_recog image:=/ardrone/image_raw camera_info:=/ardrone/camera_info", cwd="/home/base/ros/brown-ros-pkg/experimental/ar_recog/bin", shell=True) 
            if FwdCam:
                p_x.ReInit('vel_x', FWD_X_VEL_SCALAR, FWD_X_INT_SCALAR, FWD_X_DERIVATIVE_SCALAR, FWD_X_MAX_VELOCITY, 0, FWD_X_DEAD_BAND)
                p_y.ReInit('vel_y', FWD_Y_VEL_SCALAR, FWD_Y_INT_SCALAR, FWD_Y_DERIVATIVE_SCALAR, FWD_Y_MAX_VELOCITY, 0, FWD_Y_DEAD_BAND)
                p_z.ReInit('vel_z', FWD_Z_VEL_SCALAR, FWD_Z_INT_SCALAR, FWD_Z_DERIVATIVE_SCALAR, FWD_Z_MAX_VELOCITY, 0, FWD_Z_DEAD_BAND)
                p_z_ang.ReInit('ang_z', FWD_Z_ANG_VEL_SCALAR, FWD_Z_ANG_INT_SCALAR, FWD_Z_ANG_DERIVATIVE_SCALAR, FWD_Z_ANG_MAX_VELOCITY, 0, FWD_Z_ANG_DEAD_BAND)
            else:
                p_x.ReInit('vel_x', DWN_X_VEL_SCALAR, DWN_X_INT_SCALAR, DWN_X_DERIVATIVE_SCALAR, DWN_X_MAX_VELOCITY, 0, DWN_X_DEAD_BAND)
                p_y.ReInit('vel_y', DWN_Y_VEL_SCALAR, DWN_Y_INT_SCALAR, DWN_Y_DERIVATIVE_SCALAR, DWN_Y_MAX_VELOCITY, 0, DWN_Y_DEAD_BAND)
                p_z.ReInit('vel_z', DWN_Z_VEL_SCALAR, DWN_Z_INT_SCALAR, DWN_Z_DERIVATIVE_SCALAR, DWN_Z_MAX_VELOCITY, 0, DWN_Z_DEAD_BAND)
                p_z_ang.ReInit('ang_z', DWN_Z_ANG_VEL_SCALAR, DWN_Z_ANG_INT_SCALAR, DWN_Z_ANG_DERIVATIVE_SCALAR, DWN_Z_ANG_MAX_VELOCITY, 0, DWN_Z_ANG_DEAD_BAND)
            print "ToggleCam %d" % FwdCam 
            log.warn("ToggleCam %d" % FwdCam)
        except rospy.ServiceException, e:
            log.critical("Service call failed: %s"%e)
            print "Service call failed: %s"%e

    prev_key = key
    
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
    if ( math.isnan( MyTwist.twist.linear.x ) ):
        print 'NaN X',
        MyTwist.twist.linear.x = 0
    if ( math.isnan( MyTwist.twist.linear.y ) ):
        print 'NaN Y',
        MyTwist.twist.linear.y = 0
    if ( math.isnan( MyTwist.twist.linear.z ) ):
        print 'NaN Z',
        MyTwist.twist.linear.z = 0

    if PoseMatch and not FwdCam:
        MyTwist.twist.angular = CalcScaledAngle( NewTwist.twist.angular )
        if ( math.isnan( MyTwist.twist.angular.z ) ):
            print 'NaN Z Ang',
            MyTwist.twist.angular.z = 0
        PrevX = MyTwist.twist.linear.x
        PrevY = MyTwist.twist.linear.y
        MyTwist.twist.linear.x = PrevX*math.cos(MyTwist.twist.angular.z*ANG_VEL_TO_RADIANS_SCALAR) - PrevY*math.sin(MyTwist.twist.angular.z*ANG_VEL_TO_RADIANS_SCALAR)
        MyTwist.twist.linear.y = PrevX*math.sin(MyTwist.twist.angular.z*ANG_VEL_TO_RADIANS_SCALAR) + PrevY*math.cos(MyTwist.twist.angular.z*ANG_VEL_TO_RADIANS_SCALAR) 
    else:
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
    global p_y
    global p_z
    global p_z_ang
    global FwdCam
    global prev_key
    global FWD_TAG_DIAMETER
    global DWN_TAG_DIAMETER
    global TagFound
    global NavTwist
    global LastTwist
        
    InputTags = data
    NewTwist = TwistStamped()
    
    if InputTags.image_width != 320 and FwdCam and prev_key != 'switch':
        print "Switch to Down Cam"
        p_x.ReInit('vel_x', DWN_X_VEL_SCALAR, DWN_X_INT_SCALAR, DWN_X_DERIVATIVE_SCALAR, DWN_X_MAX_VELOCITY, 0, DWN_X_DEAD_BAND)
        p_y.ReInit('vel_y', DWN_Y_VEL_SCALAR, DWN_Y_INT_SCALAR, DWN_Y_DERIVATIVE_SCALAR, DWN_Y_MAX_VELOCITY, 0, DWN_Y_DEAD_BAND)
        p_z.ReInit('vel_z', DWN_Z_VEL_SCALAR, DWN_Z_INT_SCALAR, DWN_Z_DERIVATIVE_SCALAR, DWN_Z_MAX_VELOCITY, 0, DWN_Z_DEAD_BAND)
        p_z_ang.ReInit('ang_z', DWN_Z_ANG_VEL_SCALAR, DWN_Z_ANG_INT_SCALAR, DWN_Z_ANG_DERIVATIVE_SCALAR, DWN_Z_ANG_MAX_VELOCITY, 0, DWN_Z_ANG_DEAD_BAND)
        FwdCam = False

    if InputTags.tag_count > 0:
        TagFound = True
        if InputTags.tags[0].id == 0:
            NewTwist.header.frame_id = 'switch'
        else:
            NewTwist.header.frame_id = 'move'
            
        if FwdCam:
            # Forward Camera
            # +Z_fwd_cam = +Z_base - points up
            # +Y_fwd_cam = +Y_base - points left
            # +X_fwd_cam = +X_base - points forward
            NewTwist.twist.linear.x = InputTags.tags[0].diameter - FWD_TAG_DIAMETER
            NewTwist.twist.linear.y = InputTags.tags[0].x - ( FWD_IMAGE_WIDTH/2 ) #( IMAGE_WIDTH/2 ) - InputTags.tags[0].y
            NewTwist.twist.linear.z = InputTags.tags[0].y - ( FWD_IMAGE_HEIGHT/2 ) #( IMAGE_HEIGHT/2 ) - InputTags.tags[0].x
            NewTwist.twist.angular.z = 0 # No rotation on fwd cam
            #PrevDiameter = InputTags.tags[0].diameter
        else:
            # Downward Camera
            # NOTE: the downward camera 
            # +Z_down_cam = -Z_base - points down, 
            # +Y_down_cam = +X_base - points forward
            # +X_down_cam = +Y_base - points left
            NewTwist.twist.linear.x = InputTags.tags[0].y - ( DWN_IMAGE_HEIGHT/2 )
            NewTwist.twist.linear.y = InputTags.tags[0].x - ( DWN_IMAGE_WIDTH/2 )
            NewTwist.twist.linear.z = DWN_TAG_DIAMETER - InputTags.tags[0].diameter
            print InputTags.tags[0].zRot
            NewTwist.twist.angular.z = 2*math.pi - InputTags.tags[0].zRot
            
        #rospy.Publisher("image_pos", NewTwist )
        ProcessImagePosition( NewTwist )

        # Keep some history for when the tag disappears
        while len(PrevVector) >= MAX_HISTORY:
            PrevVector.pop(0)
        PrevVector.append(NewTwist.twist)
    else:
        NewTwist.header.frame_id = 'lost'
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
            NewTwist.twist.linear.z = NavTwist.linear.z
            NewTwist.twist.angular.z = 0
        if TagFound:
            p_x.Reset()
            p_y.Reset()
            p_z.Reset()
            p_z_ang.Reset()
            prev_key = 'foobar'
        TagFound = False    
        pub = rospy.Publisher("cmd_vel", Twist )
        pub.publish( NewTwist.twist )
        #ProcessImagePosition( NewTwist )
    
def ProcessNavData(data):
    global p_z_alt
    global NavTwist
    global AutoAltitude
    if AutoAltitude:
        NavTwist.linear.z = p_z_alt.ComputePID(data.altd-ALT_Z_GOAL)
    else:
        NavTwist.linear.z = 0
    
def DroneDriver():
    global processHandle
    
    rospy.init_node('drone_driver')
    rospy.Subscriber("tags", Tags, ProcessXlateImage )   
    rospy.Subscriber("image_pos", TwistStamped, ProcessChangePID )
    rospy.Subscriber("/ardrone/navdata", Navdata, ProcessNavData )
    rospy.spin()

if __name__ == '__main__':
    try:
        DroneDriver()
    except Exception as e:
        print e
        print repr(e)
        
    finally:
        processHandle.kill()
        land_pub = rospy.Publisher('/ardrone/land', std_msgs.msg.Empty)
        # emergency land on exit
        land_pub.publish(Empty())
        print "Done"
