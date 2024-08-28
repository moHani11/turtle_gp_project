#!/usr/bin/env python3

import rospy
from turtlesim.srv import Spawn
import random, os, subprocess
from std_msgs.msg import String

# important !!!
MASTER_IP = "10.0.2.15"   # Change this variable to match the ip of the host master 
DEVICE_IP = "10.0.2.15"   # Change this variable to match the ip of your device 

current_directory = os.path.dirname(os.path.abspath(__file__))
os.chdir(current_directory)

launch_file_path = os.path.join(current_directory, "../launch/try1.launch")
    # da launch file el bey3mel run lel node el hatmove el turtle 
    # 34an at7akem fel turtle 3alatool awel mate3ml spawn


# x is from 0 to 11
# y is from 0 to 11


def spawn_turtle():

    command1 = f"export ROS_MASTER_URI=http://{MASTER_IP}"
    command2 = f"export ROS_IP={DEVICE_IP}"

    # beysta5dem el bash command line 34an ye export el master ip we ip el gehaz

    subprocess.run(command1, shell=True, capture_output=True, text=True)
    subprocess.run(command2, shell=True, capture_output=True, text=True)

    turtle_name='turtle1'
    x = random.uniform(0, 11)
    y = random.uniform(0, 11)
    angle = random.uniform(0, 360)  
    rospy.init_node('turtle_spawner', anonymous=True)  # bey3mel node  betspawn turtle fe makan random

    isMaster = str(rospy.get_param('~master', 'no'))
    rospy.loginfo(f"isMaster: {isMaster}")

    if isMaster != 'True':

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


    command = f"roslaunch turtlesim_project moveLaunch.launch turtle_name:={turtle_name}"
    # beysta5dem el bash command line 34an ycall el .launch file we yedeelo arguments

    subprocess.run(command, shell=True, capture_output=True, text=True)

if __name__ == '__main__':
    spawn_turtle()