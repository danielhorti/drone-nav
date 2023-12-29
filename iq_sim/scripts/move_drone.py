#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import os

msg = """
Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
    w
a   s   d

For up and down:
    r
    f

anything else : stop

CTRL+C to quit
"""

moveBindings = {
    'w': (0, 1, 0, 0),
    's': (0, -1, 0, 0),
    'a': (-1, 0, 0, 0),
    'd': (1, 0, 0, 0),
    'r': (0, 0, 1, 0),
    'f': (0, 0, -1, 0),
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size = 1)
    rospy.init_node('drone_teleop')
    speed = 1.0
    target_speed = rospy.get_param("~speed", 0.5)

    try:
        print(msg)
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                os.system('clear')
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
                print(key)
            else:
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            twist = Twist()

            twist.linear.x = x*target_speed; twist.linear.y = y*target_speed; twist.linear.z = z*target_speed;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*speed;

            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

