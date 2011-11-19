#!/usr/bin/env python
import roslib; roslib.load_manifest('fake_image')
import rospy

from geometry_msgs.msg import TwistStamped

import sys, select, termios, tty
import random

msg = """
Creating Fake Image position data and publishing to image_pos

CTRL+c to quit
"""

move_bindings = {
        68:('linear', 'y', 0.3), #left
        67:('linear', 'y', -0.3), #right
        65:('linear', 'x', 0.3), #forward
        66:('linear', 'x', -0.3), #back
        'w':('linear', 'z', 0.3),
        's':('linear', 'z', -0.3),
        'a':('angular', 'z', 1),
        'd':('angular', 'z', -1),
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
                if (key == '\x03'):
                    break
            twist.header.frame_id = "%c" % key
            twist.header.seq += 1
            if twist.twist.linear.x != 0:
                twist.twist.linear.y = twist.twist.linear.x
            elif twist.twist.linear.y != 0:
                twist.twist.linear.x = twist.twist.linear.y
            print prevTwist
            pub.publish(twist)
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

