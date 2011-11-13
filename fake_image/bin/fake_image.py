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
        68:('linear', 'y', 0.1), #left
        67:('linear', 'y', -0.1), #right
        65:('linear', 'x', 0.1), #forward
        66:('linear', 'x', -0.1), #back
        'w':('linear', 'z', 0.1),
        's':('linear', 'z', -0.1),
        'a':('angular', 'z', 1),
        'd':('angular', 'z', -1),
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
    rospy.init_node('fake_image')
    prevTwist = TwistStamped()

    try:
        while(True):
            key = getKey()

#            twist = TwistStamped()
#            if ord(key) in move_bindings.keys():
#                                key = ord(key)
#            if key in move_bindings.keys():
#                (lin_ang, xyz, speed) = move_bindings[key]
#                setattr(getattr(twist.twist, lin_ang), xyz, speed)
#            else:
#                if (key == '\x03'):
#                    break
#            prevTwist.header.frame_id = "%c" % key
#            prevTwist.header.seq += 1
#            prevTwist.twist.linear.x += twist.twist.linear.x
#            prevTwist.twist.linear.y += twist.twist.linear.y
#            prevTwist.twist.linear.z += twist.twist.linear.z
#            prevTwist.twist.angular.x += twist.twist.angular.x
#            prevTwist.twist.angular.y += twist.twist.angular.y
#            prevTwist.twist.angular.z += twist.twist.angular.z
#            print prevTwist



            if (key == '\x03'):
                break
            prevTwist.twist.linear.x = random.uniform(0,0.2)
            prevTwist.twist.linear.y = random.uniform(0,0.2)
            prevTwist.twist.linear.z = 0
            prevTwist.twist.angular.x = 0
            prevTwist.twist.angular.y = 0
            prevTwist.twist.angular.z = random.uniform(0,1)
            pub.publish(prevTwist)

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

