#!/usr/bin/env python
import roslib; roslib.load_manifest('fake_image')
import rospy

from geometry_msgs.msg import TwistStamped

import sys, select, termios, tty
import random

SavedTwist = TwistStamped()
SavedTwist.twist.linear.x = 160
SavedTwist.twist.linear.y = 120
PositiveValue = True

msg = """
Creating Fake Image position data and publishing to image_pos

CTRL+c to quit
"""

move_bindings = {
        68:('linear', 'y', 1), #left
        67:('linear', 'y', -1), #right
        65:('linear', 'x', 1), #forward
        66:('linear', 'x', -1), #back
#        'w':('linear', 'z', 0.3),
#        's':('linear', 'z', -0.3),
#        'a':('angular', 'z', 1),
#        'd':('angular', 'z', -1),
        'w':('linear', 'x', 10), #forward
        '9':('linear', 'x', 9), #forward
        '8':('linear', 'x', 8), #forward
        '7':('linear', 'x', 7), #forward
        '6':('linear', 'x', 6), #forward
        '5':('linear', 'x', 5), #forward
        '4':('linear', 'x', 4), #forward
        '3':('linear', 'x', 3), #forward
        '2':('linear', 'x', 2), #forward
        '1':('linear', 'x', 1), #forward
        's':('linear', 'x', -10),
        'a':('linear', 'y', 10), #left
        'd':('linear', 'y', -10),
        'm':('linear', 'x', 0),
        'n':('linear', 'y', 0),
           }

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    global PositiveValue
    settings = termios.tcgetattr(sys.stdin)

    print msg
    
    pub = rospy.Publisher('image_pos', TwistStamped)
#    pub = rospy.Publisher('cmd_vel', Twist)
    rospy.init_node('fake_image')
    prevTwist = TwistStamped()

    try:
        while(True):
            key = getKey()

            twist = TwistStamped()
            if ord(key) in move_bindings.keys():
                                key = ord(key)
            if key in move_bindings.keys():
                (lin_ang, xyz, speed) = move_bindings[key]
                setattr(getattr(twist.twist, lin_ang), xyz, speed)
            else:
                if (key == '-'):
                    PositiveValue = not PositiveValue
                    print "Positive Sign = ",
                    print PositiveValue
                if (key == '\x03'):
                    break
            twist.header.frame_id = "%c" % key
            twist.header.seq += 1
            if twist.twist.linear.x != 0:
                if PositiveValue:
                    SavedTwist.twist.linear.x += twist.twist.linear.x
                else:
                    SavedTwist.twist.linear.x -= twist.twist.linear.x
            elif twist.twist.linear.y != 0:
                if PositiveValue:
                    SavedTwist.twist.linear.y += twist.twist.linear.y
                else:
                    SavedTwist.twist.linear.y -= twist.twist.linear.y
            print SavedTwist
            pub.publish(SavedTwist)
#            pub.publish(twist.twist)

            



#            if (key == '\x03'):
#                break
#
#            for i in range(5):
#                prevTwist.twist.linear.x = 0.1
#                prevTwist.twist.linear.y = 0
#                prevTwist.twist.linear.z = 0
#                prevTwist.twist.angular.x = 0
#                prevTwist.twist.angular.y = 0
#                prevTwist.twist.angular.z = 0
#                pub.publish(prevTwist)
#            for i in range(5):
#                prevTwist.twist.linear.x = 0
#                prevTwist.twist.linear.y = 0.1
#                prevTwist.twist.linear.z = 0
#                prevTwist.twist.angular.x = 0
#                prevTwist.twist.angular.y = 0
#                prevTwist.twist.angular.z = 0
#                pub.publish(prevTwist)
#            for i in range(5):
#                prevTwist.twist.linear.x = -0.1
#                prevTwist.twist.linear.y = 0
#                prevTwist.twist.linear.z = 0
#                prevTwist.twist.angular.x = 0
#                prevTwist.twist.angular.y = 0
#                prevTwist.twist.angular.z = 0
#                pub.publish(prevTwist)
#            for i in range(5):
#                prevTwist.twist.linear.x = 0
#                prevTwist.twist.linear.y = -0.1
#                prevTwist.twist.linear.z = 0
#                prevTwist.twist.angular.x = 0
#                prevTwist.twist.angular.y = 0
#                prevTwist.twist.angular.z = 0
#                pub.publish(prevTwist)

    except Exception as e:
        print e
        print repr(e)

    finally:
        twist = TwistStamped()
        twist.header.frame_id = 'land'
        twist.twist.linear.x = 0; twist.twist.linear.y = 0; twist.twist.linear.z = 0
        twist.twist.angular.x = 0; twist.twist.angular.y = 0; twist.twist.angular.z = 0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

