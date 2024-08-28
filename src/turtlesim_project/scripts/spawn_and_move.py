#!/usr/bin/env python3

import rospy
from turtlesim.srv import Spawn
import random, os, subprocess
from std_msgs.msg import String
from geometry_msgs.msg import Twist

import sys, select, termios, tty
from std_srvs.srv import Empty



def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def clear_track():
    rospy.wait_for_service('/clear')
    clear_service = rospy.ServiceProxy('/clear', Empty)

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        try:
            clear_service()
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
        rate.sleep()
    # da launch file el bey3mel run lel node el hatmove el turtle 
    # 34an at7akem fel turtle 3alatool awel mate3ml spawn

# x is from 0 to 11
# y is from 0 to 11


def spawn_turtle():
    turtle_name='turtle1'
    x = random.uniform(0, 11)
    y = random.uniform(0, 11)
    angle = random.uniform(0, 360)  

    isMaster = str(rospy.get_param('~master', 'no'))
    rospy.loginfo(f"isMaster: {isMaster}")

    if isMaster != 'True' and isMaster != 'yes':

        rospy.wait_for_service('/spawn')
        try:
            spawn_turtle = rospy.ServiceProxy('/spawn', Spawn)
            response = spawn_turtle(x, y, angle, '')
            turtle_name = response.name
            
            rospy.loginfo(f"Spawned a turtle named: {turtle_name}")
            pub = rospy.Publisher("/spawns", String, queue_size=10)
            rate = rospy.Rate(1)
            for i in range(2):
                pub.publish(turtle_name)
                rate.sleep()

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
    return turtle_name

def move_and_cleartracks(turtle_name):
    
    settings = termios.tcgetattr(sys.stdin)

    # Initialize the node
    # Get the turtle name from the command line argument

    # Publisher to the turtle's cmd_vel topic
    pub2 = rospy.Publisher(f'/{turtle_name}/cmd_vel', Twist, queue_size=10)
    pub3 = rospy.Publisher('/attacks', String, queue_size=10)

    speed = 2.0  # Linear speed
    turn = 2.2   # Angular speed

    clear_service = rospy.ServiceProxy('/clear', Empty)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():

        try:
            clear_service()
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
        
        key = getKey(settings)
        twist = Twist()
        
        if key == "q":
            pub3.publish(turtle_name)

        if key == 'w':  
            twist.linear.x = speed
        elif key == 's':  
            twist.linear.x = -speed
        elif key == 'a':  
            twist.angular.z = turn
        elif key == 'd':  
            twist.angular.z = -turn
        elif key == '\x03':  # Press Ctrl-C to stop
            break
        pub2.publish(twist)



    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    rospy.init_node('Turtle', anonymous=True)
    turtle_name = spawn_turtle()
    move_and_cleartracks(turtle_name)
