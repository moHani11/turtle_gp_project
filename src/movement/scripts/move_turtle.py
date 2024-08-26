#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# Function to get key press
def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

# Main function
if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    # Initialize the node
    rospy.init_node('move_turtle')

    # Get the turtle name from the command line argument
    turtle_name = rospy.get_param('~turtle', 'turtle1')  # Default is 'turtle1'

    # Publisher to the turtle's cmd_vel topic
    pub = rospy.Publisher(f'/{turtle_name}/cmd_vel', Twist, queue_size=10)
    
    speed = 2.0  # Linear speed
    turn = 2.0   # Angular speed

    while not rospy.is_shutdown():
        key = getKey()
        twist = Twist()

        if key == 'w':  # Move forward
            twist.linear.x = speed
        elif key == 's':  # Move backward
            twist.linear.x = -speed
        elif key == 'a':  # Turn left
            twist.angular.z = turn
        elif key == 'd':  # Turn right
            twist.angular.z = -turn
        elif key == '\x03':  # Press Ctrl-C to stop
            break

        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
